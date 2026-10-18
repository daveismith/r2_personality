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

Reads ``/stimuli`` and ``/mood/state`` from a rosbag2 bag (mcap or sqlite3)
and drives a pure :class:`~r2_affect.model.AffectModel` with them the way the
node does. A stimulus's ``header.stamp`` is never used, as in the node.

With ``/mood/state`` in the bag (the normal case) the replay ticks exactly
where the node ticked: at each recorded state's ``header.stamp``, the node's
own clock at that tick. The node's timer runs on a steady clock, so under sim
time its ticks are 0.1 s of wall time apart and its ``dt`` is whatever the sim
clock did meanwhile (0 while paused); a synthetic grid would drift from that.
Which tick a stimulus goes before comes from the recorded idle time: the node
resets ``idle_s`` when it handles a stimulus and a ``SetMood`` call never does,
so the first state near the stimulus's receive time that shows a reset is the
one (on a loaded machine the bag's order of a stimulus and a state can be
wrong by tens of milliseconds either way). Without a visible reset (a
low-confidence stimulus, a paused clock) it goes before the first state
received after it; if that state came within :data:`AMBIGUOUS_S`, both orders
are tried and the one closer to the recorded state is kept. Its time is the
receive time mapped onto the node clock by interpolating between the states
around it. States stamped 0 (a sim-time node before the first ``/clock``) are
skipped.

The model starts from the first recorded state (vector, label, idle time,
lock with its ``lock_until``, focus); ``--start temperament`` starts from the
temperament instead. A state cannot carry the refractory history or a label
change in progress, so those start empty.

``SetMood`` calls are not in a bag. They are inferred from the recorded
states and replayed before the tick that first shows them: a lock that
appears or changes (``locked``, ``lock_source``, ``lock_until``), a lock that
disappears before its ``lock_until`` (a clear), and a force (the label
changes with ``mood_strength`` at least :data:`FORCE_STRENGTH`, or an axis
jumps by more than :data:`FORCE_JUMP` with no stimulus around). A force
is replayed with ``set_mood``, so under another config it moves to that
config's prototype.

Without ``/mood/state`` the replay ticks every ``1/rate_hz`` on a grid of
receive times from the first stimulus, starting at the temperament.

``--compare`` pairs each recorded state with the replayed tick at its stamp
and prints the largest difference per axis; exit status 0 when every axis is
within :data:`TOLERANCE`, 1 if not, 2 for a bag or config it cannot use.

The logic is in plain functions over plain tuples, tested without a bag in
``test/test_replay.py``; the bag layer is tested on bags written there.
"""

import argparse
import bisect
from collections import namedtuple
import csv
import dataclasses
import math
import os
import sys

from r2_affect.config import AffectConfigError
from r2_affect.config import AXES
from r2_affect.config import load_config
from r2_affect.model import AffectModel

STIMULI_TOPIC = '/stimuli'
STATE_TOPIC = '/mood/state'
STIMULUS_TYPE = 'astromech_msgs/msg/Stimulus'
STATE_TYPE = 'astromech_msgs/msg/MoodState'

TOLERANCE = 0.02
CSV_FIELDS = ('t_rel',) + AXES + ('mood', 'mood_strength', 'idle_s', 'locked')

# A stimulus received this close to a state may have reached the node on the other side of
# that tick.
AMBIGUOUS_S = 0.010
# How far after a stimulus's receive time to look for the idle reset that shows the node
# handling it (a loaded machine delivers late).
SEARCH_S = 0.5
# A recorded label change at this strength or above, with no stimulus, is a force.
FORCE_STRENGTH = 0.98
# A recorded jump of any axis by more than this between two ticks, with no stimulus, is a force.
FORCE_JUMP = 0.2
# Tolerance on time comparisons, as in the model.
TIME_EPS = 1e-9
LOCK_UNTIL_EPS = 1e-3

StimulusEvent = namedtuple(
    'StimulusEvent', 't kind intensity confidence subject bearing_rad ttl_s source')
StimulusEvent.__new__.__defaults__ = ('', math.nan, 0.0, '')

# ``t``: receive time; ``stamp``: header.stamp, the node clock at the tick. The other
# fields are named as in model.AffectState, so a RecordedState can be restored from.
RecordedState = namedtuple(
    'RecordedState',
    't stamp arousal valence curiosity confidence mood mood_strength locked lock_source '
    'lock_until idle_s focus_subject focus_bearing')
RecordedState.__new__.__defaults__ = (False, '', None, 0.0, '', math.nan)

ReplayTick = namedtuple('ReplayTick', 't state')          # state: model.AffectState
Transition = namedtuple('Transition', 't old new distance')
StimulusOutcome = namedtuple('StimulusOutcome', 't event result')   # result: StimulusResult
SetMoodEvent = namedtuple('SetMoodEvent', 't what')
ReplayResult = namedtuple(
    'ReplayResult', 'mode t0 seed rows transitions outcomes events moved')
CompareResult = namedtuple(
    'CompareResult', 'compared max_diff max_diff_t label_disagreements any_locked')

MODE_STAMPS = 'stamps'
MODE_GRID = 'grid'


# --- shared pieces ---------------------------------------------------------------------------

def _apply(model, s, now):
    return model.stimulus(s.kind, s.intensity, s.confidence, s.subject, s.bearing_rad,
                          s.ttl_s, now)


def _axis_diff(a, b):
    return max(abs(getattr(a, axis) - getattr(b, axis)) for axis in AXES)


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


# --- without recorded states: a synthetic grid -----------------------------------------------

def run_replay(model, stimuli, ticks):
    """
    Drive ``model`` through ``stimuli`` (sorted by ``t``) at the tick times ``ticks``.

    A stimulus at ``t`` is applied with ``stimulus(now=t)`` before the first
    tick at or after ``t``, as the node's subscription callback runs before
    its next timer callback. Stimuli after the last tick are not applied.
    Return ``(rows, transitions, outcomes)``.
    """
    rows, transitions, outcomes = [], [], []
    i = 0
    for t in ticks:
        while i < len(stimuli) and stimuli[i].t <= t + TIME_EPS:
            s = stimuli[i]
            outcomes.append(StimulusOutcome(s.t, s, _apply(model, s, s.t)))
            i += 1
        result = model.tick(t)
        rows.append(ReplayTick(t, result.state))
        if result.transition is not None:
            tr = result.transition
            transitions.append(Transition(t, tr.old, tr.new, tr.distance))
    return rows, transitions, outcomes


# --- with recorded states: tick at their stamps ----------------------------------------------

def usable_states(recorded):
    """Drop states stamped 0 (a sim-time node that has not seen /clock yet)."""
    return [r for r in recorded if r.stamp != 0.0]


def node_times(stimuli, recorded):
    """
    Map each stimulus's receive time onto the node clock.

    Interpolates between the stamps of the recorded states received just
    before and after it (so a paused sim clock maps to the paused stamp, and a
    wall-clock bag of a sim-time node maps onto sim time); outside the
    recording, the nearest state's stamp plus the receive-time difference.
    """
    recv = [r.t for r in recorded]
    out = []
    for s in stimuli:
        i = bisect.bisect_right(recv, s.t) - 1
        if i < 0:
            out.append(recorded[0].stamp - (recv[0] - s.t))
        elif i >= len(recorded) - 1:
            out.append(recorded[-1].stamp + (s.t - recv[-1]))
        else:
            a, b = recorded[i], recorded[i + 1]
            span = b.t - a.t
            frac = (s.t - a.t) / span if span > 0.0 else 0.0
            out.append(a.stamp + frac * (b.stamp - a.stamp))
    return out


def idle_resets(recorded, max_dt_s=0.5):
    """
    Return, per recorded state, whether the node's idle time was reset since the previous one.

    Without a stimulus ``idle_s`` grows by exactly the tick's ``dt``; any
    stimulus that has an effect (applied, held off, unknown kind, locked)
    resets it, a ``SetMood`` call never does. So a reset marks the tick
    before which the node handled a stimulus, whatever the bag's timing says.
    """
    out = [False]
    for prev, rec in zip(recorded, recorded[1:]):
        dt = min(max(rec.stamp - prev.stamp, 0.0), max_dt_s)
        expected = prev.idle_s + dt
        out.append(rec.idle_s < expected - (1e-3 + 1e-6 * expected))
    return out


def assign_ticks(stimuli, recorded, resets):
    """
    Return, per stimulus, ``(k, sure)``: it is applied just before replayed tick ``k``.

    The default ``k`` is the first state received at or after the stimulus.
    The recorded idle time decides when it can (``sure``): ``k`` itself if
    that state shows a reset; else the state before, if it was received at
    most :data:`AMBIGUOUS_S` earlier and shows one (the node got the stimulus
    before the recorder did); else the first later state with a reset within
    :data:`SEARCH_S` (the node got it late, or published before handling it).
    ``k`` 0 means it is already in the first state; ``len(recorded)`` that it
    came after the last.
    """
    recv = [r.t for r in recorded]
    out = []
    for s in stimuli:
        d = bisect.bisect_left(recv, s.t)
        if d < len(recorded) and d > 0 and resets[d]:
            out.append((d, True))
            continue
        if d > 1 and s.t - recv[d - 1] <= AMBIGUOUS_S and resets[d - 1]:
            out.append((d - 1, True))
            continue
        k = d + 1
        while k < len(recorded) and recv[k] - s.t <= SEARCH_S and not resets[k]:
            k += 1
        if d > 0 and k < len(recorded) and recv[k] - s.t <= SEARCH_S:
            out.append((k, True))
        else:
            out.append((d, False))
    return out


def infer_set_mood(prev, rec, stimulus_seen):
    """
    Return what ``SetMood`` did between two consecutive recorded states.

    ``(clear, force, lock)``: ``clear`` when a lock vanished before its
    ``lock_until`` (or had none); ``force`` when the label changed at
    ``mood_strength`` >= :data:`FORCE_STRENGTH`, or an axis jumped by more
    than :data:`FORCE_JUMP` while ``stimulus_seen`` is false (no idle reset and
    no stimulus received around it: a stimulus resets the idle time, a force
    does not); ``lock`` when ``rec`` holds a lock that ``prev`` did not hold in
    the same form.
    """
    expired = (prev.lock_until is not None and rec.stamp >= prev.lock_until - TIME_EPS)
    clear = prev.locked and not rec.locked and not expired
    force = ((rec.mood != prev.mood and rec.mood_strength >= FORCE_STRENGTH) or
             (not stimulus_seen and _axis_diff(rec, prev) > FORCE_JUMP))
    same_lock = (prev.locked and prev.lock_source == rec.lock_source and
                 (prev.lock_until is None) == (rec.lock_until is None) and
                 (rec.lock_until is None or
                  abs(prev.lock_until - rec.lock_until) < LOCK_UNTIL_EPS))
    lock = rec.locked and not same_lock
    return clear, force, lock


def _replay_set_mood(model, rec, now, clear, force, lock):
    """Apply the inferred SetMood effects to ``model``; return their descriptions."""
    what = []
    if clear or (force and model.state.locked and model.state.lock_source != rec.lock_source):
        model.set_mood('', False, 0.0, 'replay', now)
        if clear:
            what.append('clear')
    if force:
        source = rec.lock_source or 'replay'
        accepted, _ = model.set_mood(rec.mood, False, 0.0, source, now)
        if not accepted:                       # a label this config does not have
            model.restore(dataclasses.replace(
                model.state, **{axis: getattr(rec, axis) for axis in AXES}, mood=rec.mood),
                now)
        what.append('force %s' % rec.mood)
    if lock or (force and rec.locked):         # a force without lock released it
        model.restore(dataclasses.replace(
            model.state, locked=True, lock_source=rec.lock_source, lock_until=rec.lock_until),
            now)
    if lock:
        what.append('lock by %s %s' % (
            rec.lock_source or 'unknown',
            'until cleared' if rec.lock_until is None else 'until %.3f' % rec.lock_until))
    return what


def _score(state, rec):
    """How far a replayed state is from a recorded one: idle time first, then the axes."""
    return abs(state.idle_s - rec.idle_s) + _axis_diff(state, rec)


def replay_recorded(model, stimuli, recorded, seed=True):
    """
    Replay at the recorded stamps; return ``(rows, transitions, outcomes, events, moved)``.

    ``recorded`` must be usable (no zero stamps) and sorted by receive time;
    ``stimuli`` sorted by receive time. ``rows[k]`` is the replayed state at
    ``recorded[k].stamp``. ``moved`` counts stimuli applied at another tick
    than their receive time suggests (see :func:`assign_ticks`).
    """
    est = node_times(stimuli, recorded)
    resets = idle_resets(recorded, model.max_dt_s)
    first = recorded[0]
    if seed:
        model.restore(first, first.stamp)
    else:
        model.tick(first.stamp)
    rows = [ReplayTick(first.stamp, model.state)]
    transitions, outcomes, events = [], [], []
    moved = 0

    recv = [r.t for r in recorded]
    stimulus_recv = [s.t for s in stimuli]
    batches = [[] for _ in range(len(recorded) + 1)]
    for s, t_node, (k, sure) in zip(stimuli, est, assign_ticks(stimuli, recorded, resets)):
        if k != bisect.bisect_left(recv, s.t):
            moved += 1
        batches[k].append((s, t_node, sure))
    carry = []
    for k in range(1, len(recorded)):
        prev, rec = recorded[k - 1], recorded[k]
        last_tick = rows[-1].t

        lo = bisect.bisect_left(stimulus_recv, rec.t - SEARCH_S)
        nearby = lo < len(stimulus_recv) and stimulus_recv[lo] <= rec.t + AMBIGUOUS_S
        clear, force, lock = infer_set_mood(prev, rec, resets[k] or nearby)
        what = _replay_set_mood(model, rec, last_tick, clear, force, lock)
        if what:
            events.append(SetMoodEvent(rec.stamp, ', '.join(what)))

        def when(t):
            return min(max(t, last_tick), rec.stamp)

        # Undecided stimuli close before this state: try both orders, keep the closer.
        ambiguous = [(s, t) for s, t, sure in batches[k]
                     if not sure and rec.t - s.t <= AMBIGUOUS_S]
        now = carry + [(s, t) for s, t, sure in batches[k]
                       if sure or rec.t - s.t > AMBIGUOUS_S]
        carry = []
        for s, t in now:
            outcomes.append(StimulusOutcome(when(t), s, _apply(model, s, when(t))))
        if ambiguous:
            alt = model.copy()
            seen = [StimulusOutcome(when(t), s, _apply(model, s, when(t))) for s, t in ambiguous]
            result = model.tick(rec.stamp)
            alt_result = alt.tick(rec.stamp)
            if _score(alt_result.state, rec) < _score(result.state, rec):
                # The node got them after this tick: apply them before the next one.
                model, result = alt, alt_result
                carry = [(s, rec.stamp) for s, _ in ambiguous]
                moved += len(ambiguous)
            else:
                outcomes.extend(seen)
        else:
            result = model.tick(rec.stamp)
        rows.append(ReplayTick(rec.stamp, result.state))
        if result.transition is not None:
            tr = result.transition
            transitions.append(Transition(rec.stamp, tr.old, tr.new, tr.distance))
    return rows, transitions, outcomes, events, moved


def replay(config, stimuli, recorded, rate_hz=10.0, start='recorded', max_dt_s=0.5,
           min_confidence=0.0):
    """Run a replay; return a :class:`ReplayResult`."""
    if start not in ('recorded', 'temperament'):
        raise ValueError('start must be recorded or temperament, got %r' % start)
    model = AffectModel(config, min_confidence=min_confidence, max_dt_s=max_dt_s)
    recorded = usable_states(recorded)
    if recorded:
        rows, transitions, outcomes, events, moved = replay_recorded(
            model, stimuli, recorded, seed=(start == 'recorded'))
        seed = recorded[0] if start == 'recorded' else None
        return ReplayResult(MODE_STAMPS, recorded[0].stamp, seed, rows, transitions,
                            outcomes, events, moved)
    if not stimuli:
        raise ValueError('nothing to replay: no %s and no usable %s messages'
                         % (STIMULI_TOPIC, STATE_TOPIC))
    t0 = stimuli[0].t
    ticks = tick_grid(t0, stimuli[-1].t, rate_hz)
    rows, transitions, outcomes = run_replay(model, stimuli, ticks)
    return ReplayResult(MODE_GRID, t0, None, rows, transitions, outcomes, [], 0)


def compare(recorded, rows):
    """Compare recorded states with the replayed ticks at their stamps (one to one)."""
    recorded = usable_states(recorded)
    if len(recorded) != len(rows):
        raise ValueError('%d recorded states but %d replayed ticks' % (len(recorded), len(rows)))
    max_diff = {axis: 0.0 for axis in AXES}
    max_diff_t = {axis: recorded[0].stamp if recorded else None for axis in AXES}
    disagreements = 0
    for rec, row in zip(recorded, rows):
        for axis in AXES:
            diff = abs(getattr(rec, axis) - getattr(row.state, axis))
            if diff > max_diff[axis]:
                max_diff[axis] = diff
                max_diff_t[axis] = rec.stamp
        if rec.mood != row.state.mood:
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


def recorded_transitions(recorded):
    """Return a :class:`Transition` (distance NaN) for each label change in the recording."""
    recorded = usable_states(recorded)
    return [Transition(cur.stamp, prev.mood, cur.mood, math.nan)
            for prev, cur in zip(recorded, recorded[1:]) if cur.mood != prev.mood]


# --- the bag ---------------------------------------------------------------------------------

def _seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def stimulus_from_msg(t, msg):
    """Map a ``Stimulus`` message received at ``t`` to a :class:`StimulusEvent`."""
    return StimulusEvent(t=t, kind=msg.kind, intensity=msg.intensity,
                         confidence=msg.confidence, subject=msg.subject,
                         bearing_rad=msg.bearing_rad, ttl_s=msg.ttl_s, source=msg.source)


def state_from_msg(t, msg):
    """Map a ``MoodState`` message received at ``t`` to a :class:`RecordedState`."""
    lock_until = _seconds(msg.lock_until)
    return RecordedState(
        t=t, stamp=_seconds(msg.header.stamp), arousal=msg.arousal, valence=msg.valence,
        curiosity=msg.curiosity, confidence=msg.confidence, mood=msg.mood,
        mood_strength=msg.mood_strength, locked=msg.locked, lock_source=msg.lock_source,
        lock_until=lock_until if (msg.locked and lock_until != 0.0) else None,
        idle_s=msg.idle_s, focus_subject=msg.focus_subject,
        focus_bearing=msg.focus_bearing_rad)


class BagError(ValueError):
    """The bag cannot be replayed (missing, wrong type, unreadable)."""


def read_bag(uri):
    """
    Read ``/stimuli`` and ``/mood/state`` from a bag; return ``(stimuli, states)``.

    Times are the bag's receive timestamps in seconds; other topics are
    skipped. The storage plugin (mcap or sqlite3) is detected from the bag.
    Raises :class:`BagError` when it is not a bag, has no ``/stimuli`` topic,
    or carries a topic of the wrong type.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    if not os.path.isdir(uri) and not os.path.isfile(uri):
        raise BagError('no bag at %s' % uri)
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=''),
                    rosbag2_py.ConverterOptions('', ''))
    except RuntimeError as exc:
        raise BagError('cannot open %s as a bag: %s' % (uri, exc))
    types = {meta.name: meta.type for meta in reader.get_all_topics_and_types()}
    if STIMULI_TOPIC not in types:
        raise BagError('no %s topic in the bag (record it with --include-unpublished-topics)'
                       % STIMULI_TOPIC)
    wanted = []
    for topic, type_name in ((STIMULI_TOPIC, STIMULUS_TYPE), (STATE_TOPIC, STATE_TYPE)):
        if topic not in types:
            continue
        if types[topic] != type_name:
            raise BagError('%s in the bag is %s, expected %s'
                           % (topic, types[topic], type_name))
        wanted.append(topic)
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    classes = {topic: get_message(types[topic]) for topic in wanted}
    stimuli, states = [], []
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic not in classes:          # a storage plugin that ignores the filter
            continue
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
        outcome.t - t0, s.kind or "''", s.source or '-', r.intensity, what)


def _transition_text(tr, t0):
    d = '' if math.isnan(tr.distance) else ' (d=%.2f)' % tr.distance
    return '  %8.3f s  %s -> %s%s' % (tr.t - t0, tr.old, tr.new, d)


def _vector(state):
    return ' '.join('%s=%.3f' % (a[:3], getattr(state, a)) for a in AXES)


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
                        help='tick rate for a bag without /mood/state (default 10)')
    parser.add_argument('--max-dt', type=float, default=0.5,
                        help="the node's max_dt_s (default 0.5)")
    parser.add_argument('--min-confidence', type=float, default=0.0,
                        help="the node's min_confidence (default 0)")
    parser.add_argument('--start', choices=('recorded', 'temperament'), default='recorded',
                        help='start from the first recorded /mood/state (default, when the '
                             'bag has one) or from the temperament')
    return parser


def main(argv=None):
    """Run the command; return the exit status (0 ok, 1 mismatch, 2 unusable input)."""
    args = build_parser().parse_args(argv)
    out = sys.stdout if args.csv != '-' else sys.stderr

    def fail(text):
        print('replay: %s' % text, file=sys.stderr)
        return 2

    if not (math.isfinite(args.rate_hz) and args.rate_hz > 0.0):
        return fail('--rate-hz must be > 0')
    config_path = args.config or default_config_path()
    try:
        config = load_config(config_path)
    except (AffectConfigError, OSError) as exc:
        return fail(str(exc))
    try:
        stimuli, recorded = read_bag(args.bag)
    except BagError as exc:
        return fail(str(exc))
    usable = usable_states(recorded)
    if args.compare and not usable:
        return fail('--compare needs %s in the bag' % STATE_TOPIC)
    try:
        result = replay(config, stimuli, recorded, args.rate_hz, args.start, args.max_dt,
                        args.min_confidence)
    except ValueError as exc:
        return fail(str(exc))
    t0, rows = result.t0, result.rows

    print('bag %s: %d %s, %d %s; config %s' % (
        args.bag, len(stimuli), STIMULI_TOPIC, len(recorded), STATE_TOPIC, config_path),
        file=out)
    if len(usable) != len(recorded):
        print('skipped %d states stamped 0 (no /clock yet)' % (len(recorded) - len(usable)),
              file=out)
    if result.seed is not None:
        print('start: first recorded state, %s (%s), idle %.1f s%s' % (
            result.seed.mood, _vector(result.seed), result.seed.idle_s,
            ', locked' if result.seed.locked else ''), file=out)
    else:
        print('start: temperament', file=out)
    if result.mode == MODE_STAMPS:
        print('%d ticks at the recorded stamps over %.1f s of node clock '
              '(t = header.stamp minus %.3f)' % (len(rows), rows[-1].t - t0, t0), file=out)
    else:
        print('%d ticks at %g Hz over %.1f s (no %s: t = receive time minus %.3f)' % (
            len(rows), args.rate_hz, rows[-1].t - t0, STATE_TOPIC, t0), file=out)
    print('stimuli:', file=out)
    for outcome in result.outcomes:
        print(_outcome_text(outcome, t0), file=out)
    if not result.outcomes:
        print('  (none)', file=out)
    if result.moved:
        print('  (%d applied at another tick than the bag order suggests: the recorded idle '
              'time shows when the node handled it)' % result.moved, file=out)
    if result.events:
        print('SetMood (inferred from the recorded state, not in the bag):', file=out)
        for event in result.events:
            print('  %8.3f s  %s' % (event.t - t0, event.what), file=out)
    print('transitions (replayed):', file=out)
    for tr in result.transitions:
        print(_transition_text(tr, t0), file=out)
    if not result.transitions:
        print('  (none)', file=out)
    last = rows[-1].state
    print('end: %s (%.2f), %s' % (last.mood, last.mood_strength, _vector(last)), file=out)

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
    cmp = compare(recorded, rows)
    print('compare: %d recorded states against the replayed ticks at their stamps'
          % cmp.compared, file=out)
    for axis in AXES:
        print('  max |diff| %-10s %.6f at %.3f s' % (
            axis, cmp.max_diff[axis], cmp.max_diff_t[axis] - t0), file=out)
    print('  label disagreements: %d of %d' % (cmp.label_disagreements, cmp.compared),
          file=out)
    if result.events:
        print('note: %d SetMood effect(s) were inferred and replayed; a force or lock the '
              'state does not show (e.g. forcing the current label) is missed.'
              % len(result.events), file=out)
    axis, diff = worst_axis(cmp)
    if diff < TOLERANCE:
        print('OK: max diff %.6f (%s) < %g' % (diff, axis, TOLERANCE), file=out)
        return 0
    print('MISMATCH: %s differs by %.6f at %.3f s (tolerance %g)' % (
        axis, diff, cmp.max_diff_t[axis] - t0, TOLERANCE), file=out)
    return 1


if __name__ == '__main__':
    sys.exit(main())
