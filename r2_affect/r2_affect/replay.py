# Copyright (c) 2026 David Smith
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Replay a recorded session through the mood model, offline.

``replay <bag_dir> [--config affect.yaml] [--csv out.csv] [--compare] [--rate-hz 10]``

Reads ``/stimuli`` (and ``/mood/state``) from a rosbag2 bag (mcap or sqlite3)
and drives a pure :class:`~r2_affect.model.AffectModel` with them, the way the
node does: ticks every ``1/rate_hz`` on a grid of the bag's *receive* times,
and each stimulus applied at its receive time, in order, before the next tick.
``header.stamp`` is not used, as in the node.

The model starts from the first recorded ``/mood/state`` (vector, label, idle
time, lock) when the bag has one, because the node was usually running before
the recording started; the grid then starts at that message, so the replayed
ticks fall in phase with the node's. Without a recorded state, or with
``--start temperament``, it starts at the temperament and the grid starts at
the first message. What a state message cannot carry starts empty: refractory
history, a pending label change, the focus ttl.

``--compare`` checks the replay against the recorded ``/mood/state``: each
recorded state is compared with the nearest replayed tick that has seen the
same stimuli (so a stimulus landing between a node tick and a grid tick does
not show up as a one-tick spike), and the largest difference per axis is
printed. Exit status 0 when every axis is within :data:`TOLERANCE`, 1 if not.
``SetMood`` calls are not in a bag, so a session with forces or locks does not
compare cleanly; the tool says so when a recorded state is locked.

The logic (grid, replay, alignment, comparison, CSV) is in plain functions
over plain tuples, tested without a bag in ``test/test_replay.py``.
"""

import argparse
import bisect
from collections import namedtuple
import csv
import math
import os
import sys

from r2_affect.config import AffectConfigError
from r2_affect.config import AXES
from r2_affect.config import AXIS_RANGES
from r2_affect.config import load_config
from r2_affect.model import AffectModel

STIMULI_TOPIC = '/stimuli'
STATE_TOPIC = '/mood/state'
STIMULUS_TYPE = 'astromech_msgs/msg/Stimulus'
STATE_TYPE = 'astromech_msgs/msg/MoodState'

TOLERANCE = 0.02
CSV_FIELDS = ('t_rel',) + AXES + ('mood', 'mood_strength', 'idle_s', 'locked')

# Tolerance on time comparisons, as in the model.
TIME_EPS = 1e-9

StimulusEvent = namedtuple(
    'StimulusEvent', 't kind intensity confidence subject bearing_rad ttl_s source')
StimulusEvent.__new__.__defaults__ = ('', math.nan, 0.0, '')

RecordedState = namedtuple(
    'RecordedState',
    't arousal valence curiosity confidence mood mood_strength locked lock_source idle_s')
RecordedState.__new__.__defaults__ = (False, '', 0.0)

ReplayTick = namedtuple('ReplayTick', 't state')          # state: model.AffectState
Transition = namedtuple('Transition', 't old new distance')
StimulusOutcome = namedtuple('StimulusOutcome', 't event result')   # result: StimulusResult
CompareResult = namedtuple(
    'CompareResult', 'compared max_diff max_diff_t label_disagreements any_locked')


# --- pure logic ------------------------------------------------------------------------------

def tick_grid(start, end, rate_hz):
    """
    Return the tick times ``start + k / rate_hz`` up to the first at or after ``end``.

    Times are computed from ``k`` rather than accumulated, so a long bag does
    not drift. ``end`` before ``start`` gives the single tick ``[start]``.
    """
    rate_hz = float(rate_hz)
    if not (math.isfinite(rate_hz) and rate_hz > 0.0):
        raise ValueError('rate_hz must be > 0, got %r' % rate_hz)
    period = 1.0 / rate_hz
    count = max(0, int(math.ceil((end - start) / period - TIME_EPS)))
    return [start + k * period for k in range(count + 1)]


def seed_model(model, state, now):
    """
    Start ``model`` from a recorded state as if it had just ticked at ``now``.

    Sets the vector, label (the nearest mood if the config does not have it),
    idle time and a lock until cleared (a recorded ``lock_until`` is on the
    node's clock, which need not be the bag's). Refractory history, a pending
    label change and focus are not in a state message and start empty.
    The model has no public setter for these, by design (only the node's
    inputs move it), so this reaches into it; ``test_replay`` pins it down.
    """
    for axis in AXES:
        lo, hi = AXIS_RANGES[axis]
        model._x[axis] = min(hi, max(lo, float(getattr(state, axis))))
    model._idle_s = max(0.0, float(state.idle_s))
    model._mood = state.mood if state.mood in model.config.moods else model._nearest()[0]
    model._pending = None
    if state.locked:
        model._locked = True
        model._lock_source = state.lock_source or 'unknown'
        model._lock_until = None
    model._last_tick = float(now)


def run_replay(model, stimuli, ticks):
    """
    Drive ``model`` through ``stimuli`` (sorted by ``t``) at the tick times ``ticks``.

    A stimulus at ``t`` is applied with ``stimulus(now=t)`` before the first
    tick at or after ``t``, as the node's subscription callback runs before
    its next timer callback. Stimuli after the last tick are not applied.
    Return ``(rows, transitions, outcomes)``: a :class:`ReplayTick` per tick,
    a :class:`Transition` per label change made by classification, and a
    :class:`StimulusOutcome` per stimulus applied or dropped.
    """
    rows, transitions, outcomes = [], [], []
    i = 0
    for t in ticks:
        while i < len(stimuli) and stimuli[i].t <= t + TIME_EPS:
            s = stimuli[i]
            result = model.stimulus(s.kind, s.intensity, s.confidence, s.subject,
                                    s.bearing_rad, s.ttl_s, s.t)
            outcomes.append(StimulusOutcome(s.t, s, result))
            i += 1
        result = model.tick(t)
        rows.append(ReplayTick(t, result.state))
        if result.transition is not None:
            tr = result.transition
            transitions.append(Transition(t, tr.old, tr.new, tr.distance))
    return rows, transitions, outcomes


def align(recorded_times, tick_times, stimulus_times):
    """
    Return, for each recorded time, the index of the replayed tick to compare it with.

    That is the nearest tick that has seen the same stimuli: a state received
    after a stimulus is compared with a tick at or after that stimulus and
    before the next one. When no tick falls in that window (two stimuli
    within one period), the nearest tick overall. All lists sorted.
    """
    if not tick_times:
        raise ValueError('no replayed ticks to align with')
    out = []
    for t in recorded_times:
        n = bisect.bisect_left(stimulus_times, t)          # stimuli received before t
        lo = bisect.bisect_left(tick_times, stimulus_times[n - 1] - TIME_EPS) if n else 0
        hi = (bisect.bisect_left(tick_times, stimulus_times[n] - TIME_EPS)
              if n < len(stimulus_times) else len(tick_times))
        if lo >= hi:
            lo, hi = 0, len(tick_times)
        j = bisect.bisect_left(tick_times, t, lo, hi)
        candidates = [k for k in (j - 1, j) if lo <= k < hi]
        out.append(min(candidates, key=lambda k: abs(tick_times[k] - t)))
    return out


def compare(recorded, rows, stimuli):
    """Compare recorded states with the replayed ticks; return a :class:`CompareResult`."""
    tick_times = [row.t for row in rows]
    index = align([r.t for r in recorded], tick_times, [s.t for s in stimuli])
    max_diff = {axis: 0.0 for axis in AXES}
    max_diff_t = {axis: None for axis in AXES}
    disagreements = 0
    for rec, k in zip(recorded, index):
        state = rows[k].state
        for axis in AXES:
            diff = abs(getattr(rec, axis) - getattr(state, axis))
            if diff > max_diff[axis] or max_diff_t[axis] is None:
                max_diff[axis] = diff
                max_diff_t[axis] = rec.t
        if rec.mood != state.mood:
            disagreements += 1
    return CompareResult(compared=len(recorded), max_diff=max_diff, max_diff_t=max_diff_t,
                         label_disagreements=disagreements,
                         any_locked=any(r.locked for r in recorded))


def worst_axis(result):
    """Return ``(axis, diff)`` of the largest difference in a :class:`CompareResult`."""
    axis = max(AXES, key=lambda a: result.max_diff[a])
    return axis, result.max_diff[axis]


def csv_rows(rows, t0):
    """Yield one CSV row (a list, :data:`CSV_FIELDS` order) per replayed tick."""
    for row in rows:
        s = row.state
        yield (['%.3f' % (row.t - t0)] +
               ['%.6f' % getattr(s, axis) for axis in AXES] +
               [s.mood, '%.6f' % s.mood_strength, '%.3f' % s.idle_s, int(s.locked)])


def write_csv(stream, rows, t0):
    """Write the header and one row per tick to an open text ``stream``."""
    writer = csv.writer(stream, lineterminator='\n')
    writer.writerow(CSV_FIELDS)
    writer.writerows(csv_rows(rows, t0))


def plan_replay(stimuli, recorded, rate_hz, start='recorded'):
    """
    Decide where the replay starts; return ``(seed, t0, ticks, stimuli)``.

    ``seed`` is the recorded state to start from (``None``: the temperament),
    ``t0`` the first tick, ``ticks`` the grid up to the last message, and
    ``stimuli`` the ones to apply (with a seed, those received before it are
    already in it and are left out).
    """
    if start not in ('recorded', 'temperament'):
        raise ValueError('start must be recorded or temperament, got %r' % start)
    times = [s.t for s in stimuli] + [r.t for r in recorded]
    if not times:
        raise ValueError('nothing to replay: no %s and no %s messages'
                         % (STIMULI_TOPIC, STATE_TOPIC))
    seed = recorded[0] if (start == 'recorded' and recorded) else None
    t0 = seed.t if seed is not None else min(times)
    stimuli = [s for s in stimuli if s.t >= t0]
    return seed, t0, tick_grid(t0, max(times), rate_hz), stimuli


def replay(config, stimuli, recorded, rate_hz=10.0, start='recorded', max_dt_s=0.5):
    """Plan and run a replay; return ``(t0, seed, rows, transitions, outcomes)``."""
    seed, t0, ticks, stimuli = plan_replay(stimuli, recorded, rate_hz, start)
    model = AffectModel(config, max_dt_s=max_dt_s)
    if seed is not None:
        seed_model(model, seed, t0)
    rows, transitions, outcomes = run_replay(model, stimuli, ticks)
    return t0, seed, rows, transitions, outcomes


def recorded_transitions(recorded):
    """Return a :class:`Transition` (distance NaN) for each label change in the recording."""
    out = []
    for prev, cur in zip(recorded, recorded[1:]):
        if cur.mood != prev.mood:
            out.append(Transition(cur.t, prev.mood, cur.mood, math.nan))
    return out


# --- the bag ---------------------------------------------------------------------------------

def stimulus_from_msg(t, msg):
    """Map a ``Stimulus`` message received at ``t`` to a :class:`StimulusEvent`."""
    return StimulusEvent(t=t, kind=msg.kind, intensity=msg.intensity,
                         confidence=msg.confidence, subject=msg.subject,
                         bearing_rad=msg.bearing_rad, ttl_s=msg.ttl_s, source=msg.source)


def state_from_msg(t, msg):
    """Map a ``MoodState`` message received at ``t`` to a :class:`RecordedState`."""
    return RecordedState(t=t, arousal=msg.arousal, valence=msg.valence,
                         curiosity=msg.curiosity, confidence=msg.confidence, mood=msg.mood,
                         mood_strength=msg.mood_strength, locked=msg.locked,
                         lock_source=msg.lock_source, idle_s=msg.idle_s)


def read_bag(uri):
    """
    Read ``/stimuli`` and ``/mood/state`` from a bag; return ``(stimuli, states)``.

    Times are the bag's receive timestamps in seconds. The storage plugin
    (mcap or sqlite3) is detected from the bag's metadata.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {meta.name: meta.type for meta in reader.get_all_topics_and_types()}
    expected = {STIMULI_TOPIC: STIMULUS_TYPE, STATE_TOPIC: STATE_TYPE}
    wanted = []
    for topic, type_name in expected.items():
        if topic not in types:
            continue
        if types[topic] != type_name:
            raise ValueError('%s in the bag is %s, expected %s'
                             % (topic, types[topic], type_name))
        wanted.append(topic)
    if not wanted:
        return [], []
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    classes = {topic: get_message(types[topic]) for topic in wanted}
    stimuli, states = [], []
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        msg = deserialize_message(data, classes[topic])
        t = stamp_ns * 1e-9
        if topic == STIMULI_TOPIC:
            stimuli.append(stimulus_from_msg(t, msg))
        else:
            states.append(state_from_msg(t, msg))
    # A bag is written in receive order; sort anyway (stable), the replay relies on it.
    stimuli.sort(key=lambda s: s.t)
    states.sort(key=lambda s: s.t)
    return stimuli, states


def default_config_path():
    """Return the installed ``affect.yaml``."""
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('r2_affect'), 'config', 'affect.yaml')


# --- the command -----------------------------------------------------------------------------

def _outcome_text(outcome, t0):
    s, r = outcome.event, outcome.result
    if r.applied:
        what = ' '.join('%s%+.3f' % (axis[:3], r.delta[axis])
                        for axis in AXES if r.delta[axis] != 0.0) or 'no change'
    else:
        what = 'dropped: %s' % r.reason if not r.accepted else 'not applied: %s' % r.reason
    return '  %8.3f s  %-16s from %-12s i=%.2f  %s' % (
        s.t - t0, s.kind or "''", s.source or '-', r.intensity, what)


def _transition_text(tr, t0):
    d = '' if math.isnan(tr.distance) else ' (d=%.2f)' % tr.distance
    return '  %8.3f s  %s -> %s%s' % (tr.t - t0, tr.old, tr.new, d)


def build_parser():
    """Return the argument parser."""
    parser = argparse.ArgumentParser(
        prog='replay',
        description='Replay the /stimuli of a recorded session through the mood model '
                    '(r2_affect) offline: CSV of the state per tick, label transitions, '
                    'and with --compare a check against the recorded /mood/state.')
    parser.add_argument('bag', help='the bag directory (mcap or sqlite3)')
    parser.add_argument('--config', help='affect.yaml to replay with '
                        '(default: the installed share/r2_affect/config/affect.yaml)')
    parser.add_argument('--csv', metavar='OUT', help="write the state per tick here ('-': stdout)")
    parser.add_argument('--compare', action='store_true',
                        help='compare with the recorded /mood/state; exit 1 if any axis '
                             'differs by %g or more' % TOLERANCE)
    parser.add_argument('--rate-hz', type=float, default=10.0,
                        help='tick rate, as the node (default 10)')
    parser.add_argument('--max-dt', type=float, default=0.5,
                        help="the node's max_dt_s (default 0.5)")
    parser.add_argument('--start', choices=('recorded', 'temperament'), default='recorded',
                        help='start from the first recorded /mood/state (default, when the '
                             'bag has one) or from the temperament')
    return parser


def main(argv=None):
    """Run the command; return the exit status."""
    args = build_parser().parse_args(argv)
    out = sys.stdout if args.csv != '-' else sys.stderr
    if not (math.isfinite(args.rate_hz) and args.rate_hz > 0.0):
        print('replay: --rate-hz must be > 0', file=sys.stderr)
        return 2
    config_path = args.config or default_config_path()
    try:
        config = load_config(config_path)
    except (AffectConfigError, OSError) as exc:
        print('replay: %s' % exc, file=sys.stderr)
        return 2
    if not os.path.exists(args.bag):
        print('replay: no bag at %s' % args.bag, file=sys.stderr)
        return 2
    try:
        stimuli, recorded = read_bag(args.bag)
    except (RuntimeError, ValueError) as exc:
        print('replay: cannot read %s: %s' % (args.bag, exc), file=sys.stderr)
        return 2
    if args.compare and not recorded:
        print('replay: --compare needs %s in the bag' % STATE_TOPIC, file=sys.stderr)
        return 2
    try:
        t0, seed, rows, transitions, outcomes = replay(
            config, stimuli, recorded, args.rate_hz, args.start, args.max_dt)
    except ValueError as exc:
        print('replay: %s' % exc, file=sys.stderr)
        return 2

    print('bag %s: %d %s, %d %s; config %s' % (
        args.bag, len(stimuli), STIMULI_TOPIC, len(recorded), STATE_TOPIC, config_path),
        file=out)
    if seed is not None:
        print('start: first recorded state, %s (%s), idle %.1f s%s' % (
            seed.mood, ' '.join('%s=%.3f' % (a[:3], getattr(seed, a)) for a in AXES),
            seed.idle_s, ', locked' if seed.locked else ''), file=out)
    else:
        print('start: temperament', file=out)
    print('%d ticks at %g Hz over %.1f s (t = receive time minus %.3f)' % (
        len(rows), args.rate_hz, rows[-1].t - t0, t0), file=out)
    print('stimuli:', file=out)
    for outcome in outcomes:
        print(_outcome_text(outcome, t0), file=out)
    if not outcomes:
        print('  (none)', file=out)
    print('transitions (replayed):', file=out)
    for tr in transitions:
        print(_transition_text(tr, t0), file=out)
    if not transitions:
        print('  (none)', file=out)
    last = rows[-1].state
    print('end: %s (%.2f), %s' % (
        last.mood, last.mood_strength,
        ' '.join('%s=%.3f' % (a[:3], getattr(last, a)) for a in AXES)), file=out)

    if args.csv:
        if args.csv == '-':
            write_csv(sys.stdout, rows, t0)
        else:
            with open(args.csv, 'w', newline='') as stream:
                write_csv(stream, rows, t0)
            print('csv: %d rows to %s' % (len(rows), args.csv), file=out)

    if not args.compare:
        return 0
    print('transitions (recorded):', file=out)
    recorded_tr = recorded_transitions(recorded)
    for tr in recorded_tr:
        print(_transition_text(tr, t0), file=out)
    if not recorded_tr:
        print('  (none)', file=out)
    result = compare(recorded, rows, stimuli)
    print('compare: %d recorded states against %d replayed ticks' % (
        result.compared, len(rows)), file=out)
    for axis in AXES:
        print('  max |diff| %-10s %.6f at %.3f s' % (
            axis, result.max_diff[axis], result.max_diff_t[axis] - t0), file=out)
    print('  label disagreements: %d of %d' % (result.label_disagreements, result.compared),
          file=out)
    if result.any_locked:
        print('note: the recording has locked states. SetMood calls (force, lock, clear) are '
              'not in the bag, so they are not replayed and the comparison is not meaningful '
              'across them.', file=out)
    axis, diff = worst_axis(result)
    if diff < TOLERANCE:
        print('OK: max diff %.6f (%s) < %g' % (diff, axis, TOLERANCE), file=out)
        return 0
    print('MISMATCH: %s differs by %.6f at %.3f s (tolerance %g)' % (
        axis, diff, result.max_diff_t[axis] - t0, TOLERANCE), file=out)
    return 1


if __name__ == '__main__':
    sys.exit(main())
