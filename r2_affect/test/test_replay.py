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

"""Unit tests for the replay tool's logic (no bag, no ROS graph: synthetic sequences)."""

import io
import math
import os
import sys

import pytest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

from r2_affect.config import AXES, load_config, parse_config  # noqa: E402
from r2_affect.model import AffectModel  # noqa: E402
from r2_affect.replay import compare, CSV_FIELDS, infer_set_mood  # noqa: E402
from r2_affect.replay import MODE_GRID, MODE_STAMPS, node_times  # noqa: E402
from r2_affect.replay import recorded_transitions, RecordedState, replay  # noqa: E402
from r2_affect.replay import run_replay, StimulusEvent, tick_grid  # noqa: E402
from r2_affect.replay import TOLERANCE, worst_axis, write_csv  # noqa: E402

SHIPPED = os.path.join(PKG_ROOT, 'config', 'affect.yaml')
NAN = float('nan')
EXACT = 1e-9
SIM_T0 = 1000.0


def shipped():
    return load_config(SHIPPED)


def shipped_with(**impulse_scale):
    """Return the shipped config with the named kinds' impulses multiplied."""
    import yaml
    with open(SHIPPED) as f:
        data = yaml.safe_load(f)
    for kind, factor in impulse_scale.items():
        for key, value in data['impulses'][kind].items():
            if key != 'refractory_s':
                data['impulses'][kind][key] = value * factor
    return parse_config(data)


def stim(t, kind, **kw):
    kw.setdefault('intensity', 1.0)
    kw.setdefault('confidence', 1.0)
    kw.setdefault('source', 'test')
    return StimulusEvent(t=t, kind=kind, **kw)


def rec(t, stamp=None, mood='calm', **kw):
    """Return a RecordedState with defaults, for hand-made sequences."""
    fields = {'arousal': 0.0, 'valence': 0.2, 'curiosity': 0.4, 'confidence': 0.6,
              'mood_strength': 1.0}
    fields.update(kw)
    return RecordedState(t=t, stamp=t if stamp is None else stamp, mood=mood, **fields)


def to_recorded(t, stamp, s):
    return RecordedState(t=t, stamp=stamp, arousal=s.arousal, valence=s.valence,
                         curiosity=s.curiosity, confidence=s.confidence, mood=s.mood,
                         mood_strength=s.mood_strength, locked=s.locked,
                         lock_source=s.lock_source, lock_until=s.lock_until, idle_s=s.idle_s,
                         focus_subject=s.focus_subject, focus_bearing=s.focus_bearing)


def fake_node(config, stimuli, start, stop, phase=0.0, latency=0.002, record_from=0.0,
              rtf=1.0, pause=None, node_late=None, set_moods=(), rate_hz=10.0,
              min_confidence=0.0):
    """
    Run a model the way the node does, and record /mood/state the way a wall-clock bag does.

    The node's timer is steady: it ticks every ``1/rate_hz`` of wall time from
    ``start + phase``. Its clock is sim time, ``rtf`` times wall time, stopped
    during ``pause`` (wall interval). A state is "received" ``latency`` after
    its tick and stamped with the node clock. Stimuli are received by the bag
    at their ``t`` (wall) and by the node ``node_late[i]`` later (default 0).
    ``set_moods``: ``(wall_t, mood, lock, duration_s, source)`` calls.
    """
    node_late = node_late or {}

    def sim(w):            # offset, so that no real stamp is 0 (which means "no /clock")
        if pause is None or w < pause[0]:
            return SIM_T0 + w * rtf
        if w < pause[1]:
            return SIM_T0 + pause[0] * rtf
        return SIM_T0 + (w - (pause[1] - pause[0])) * rtf

    m = AffectModel(config, min_confidence=min_confidence)
    events = []
    k = 0
    while True:
        w = start + phase + k / rate_hz
        if w > stop:
            break
        events.append((w, 1, 'tick', None))
        k += 1
    for i, s in enumerate(stimuli):
        events.append((s.t + node_late.get(i, 0.0), 0, 'stim', s))
    for call in set_moods:
        events.append((call[0], 0, 'set', call))
    events.sort(key=lambda e: (e[0], e[1]))
    recorded = []
    for w, _, what, item in events:
        if what == 'stim':
            m.stimulus(item.kind, item.intensity, item.confidence, item.subject,
                       item.bearing_rad, item.ttl_s, sim(w))
        elif what == 'set':
            _, mood, lock, duration, source = item
            assert m.set_mood(mood, lock, duration, source, sim(w))[0]
        else:
            m.tick(sim(w))
            if w + latency >= record_from:
                recorded.append(to_recorded(w + latency, sim(w), m.state))
    return recorded


def max_diff(recorded, result):
    return worst_axis(compare(recorded, result.rows))[1]


# The dry-run scenario: bump, face_known, touched, 5 s apart, received between ticks.
SCENARIO = [
    stim(2.047, 'bump'),
    stim(7.012, 'face_known', subject='dave', bearing_rad=0.8),
    stim(12.088, 'touched'),
]


# --- the grid (bags without /mood/state) -----------------------------------------------------

def test_grid_covers_start_to_end_inclusive():
    ticks = tick_grid(0.0, 1.0, 10.0)
    assert len(ticks) == 11
    assert ticks[0] == 0.0 and ticks[-1] == pytest.approx(1.0)


def test_grid_ends_at_the_first_tick_at_or_after_end():
    ticks = tick_grid(5.0, 6.05, 10.0)
    assert len(ticks) == 12
    assert ticks[-1] == pytest.approx(6.1)


def test_grid_does_not_drift_over_a_long_bag():
    ticks = tick_grid(1.7e9, 1.7e9 + 3600.0, 10.0)
    assert len(ticks) == 36001
    assert ticks[-1] == pytest.approx(1.7e9 + 3600.0, abs=1e-6)
    assert ticks[12345] == pytest.approx(1.7e9 + 1234.5, abs=1e-6)


def test_grid_end_before_start_is_one_tick():
    assert tick_grid(3.0, 2.0, 10.0) == [3.0]


@pytest.mark.parametrize('rate', [0.0, -1.0, NAN, math.inf])
def test_grid_rejects_bad_rates(rate):
    with pytest.raises(ValueError):
        tick_grid(0.0, 1.0, rate)


def test_grid_stimulus_is_applied_before_the_next_tick():
    m = AffectModel(shipped())
    rows, _, outcomes = run_replay(m, [stim(0.25, 'bump')], tick_grid(0.0, 0.5, 10.0))
    arousal = [row.state.arousal for row in rows]
    assert arousal[2] < 0.01 and arousal[3] > 0.55
    assert outcomes[0].result.delta['arousal'] == pytest.approx(0.6)


def test_grid_stimulus_on_a_tick_is_applied_before_that_tick():
    m = AffectModel(shipped())
    rows, _, _ = run_replay(m, [stim(0.2, 'bump')], [0.0, 0.1, 0.2, 0.3])
    assert rows[1].state.arousal < 0.01 and rows[2].state.arousal > 0.55


def test_grid_stimuli_after_the_last_tick_are_not_applied():
    m = AffectModel(shipped())
    rows, _, outcomes = run_replay(m, [stim(0.45, 'bump')], [0.0, 0.1, 0.2])
    assert outcomes == [] and len(rows) == 3


def test_without_states_the_grid_starts_at_the_first_stimulus():
    stimuli = [stim(1.0, 'bump'), stim(1.5, 'bump'), stim(5.0, 'nonsense')]
    result = replay(shipped(), stimuli, [])
    assert result.mode == MODE_GRID and result.seed is None and result.t0 == 1.0
    assert [o.result.reason for o in result.outcomes] == ['', 'refractory', 'unknown_kind']
    assert [(tr.old, tr.new) for tr in result.transitions] == [('calm', 'annoyed')]
    assert result.transitions[0].t - 1.0 == pytest.approx(1.5, abs=0.1 + EXACT)


def test_nothing_to_replay():
    with pytest.raises(ValueError):
        replay(shipped(), [], [])
    with pytest.raises(ValueError):
        replay(shipped(), [stim(0.0, 'bump')], [], start='sideways')


# --- at the recorded stamps: the model against itself ----------------------------------------

@pytest.mark.parametrize('phase', [0.0, 0.03, 0.071])
@pytest.mark.parametrize('rtf', [1.0, 0.956])
def test_replay_of_the_model_against_itself_has_zero_diff(phase, rtf):
    config = shipped()
    # Running idle for 100 s before the recording: the replay must start from the state.
    recorded = fake_node(config, SCENARIO, -100.0, 30.0, phase, rtf=rtf)
    result = replay(config, list(SCENARIO), recorded)
    assert result.mode == MODE_STAMPS and result.seed is recorded[0]
    assert recorded[0].idle_s > 90.0 * rtf
    cmp = compare(recorded, result.rows)
    assert cmp.compared == len(recorded)
    for axis in AXES:
        assert cmp.max_diff[axis] < EXACT, axis
    assert cmp.label_disagreements == 0
    assert [(tr.old, tr.new) for tr in result.transitions] == \
        [(tr.old, tr.new) for tr in recorded_transitions(recorded)]
    assert result.transitions, 'the scenario should change the label at least once'
    assert [o.result.applied for o in result.outcomes] == [True, True, True]


def test_ticks_are_the_recorded_stamps_not_a_grid():
    recorded = fake_node(shipped(), SCENARIO, -10.0, 5.0, 0.03, rtf=0.956)
    result = replay(shipped(), list(SCENARIO), recorded)
    assert [row.t for row in result.rows] == [r.stamp for r in recorded]
    assert result.t0 == recorded[0].stamp


def test_a_paused_sim_clock_gives_dt_zero_as_in_the_node():
    config = shipped()
    recorded = fake_node(config, SCENARIO, -10.0, 20.0, 0.03, pause=(4.0, 9.0))
    stamps = [r.stamp for r in recorded]
    assert len(stamps) - len(set(stamps)) > 40          # the heartbeat with a stuck stamp
    assert max_diff(recorded, replay(config, list(SCENARIO), recorded)) < EXACT


def test_states_stamped_zero_are_skipped():
    config = shipped()
    recorded = fake_node(config, SCENARIO, -10.0, 15.0, 0.03)
    zero = [r._replace(t=r.t - 100.0, stamp=0.0) for r in recorded[:5]]
    result = replay(config, list(SCENARIO), zero + recorded)
    assert len(result.rows) == len(recorded)
    assert max_diff(zero + recorded, result) < EXACT


def test_node_times_interpolate_between_states_and_follow_a_pause():
    recorded = [rec(10.002, 5.0), rec(10.102, 5.1), rec(10.202, 5.1), rec(10.302, 5.2)]
    times = node_times([stim(10.052, 'x'), stim(10.152, 'x'), stim(9.0, 'x'),
                        stim(11.0, 'x')], recorded)
    assert times[0] == pytest.approx(5.05)
    assert times[1] == pytest.approx(5.1)                # paused: stamps did not move
    assert times[2] == pytest.approx(5.0 - 1.002)
    assert times[3] == pytest.approx(5.2 + 0.698)


# --- stimuli near a tick ---------------------------------------------------------------------

def test_stimulus_received_exactly_with_a_state_that_already_shows_it():
    config = shipped()
    probe = fake_node(config, [], -10.0, 6.0, 0.0)
    t = probe[50].t                                       # the receive time of a state
    stimuli = [stim(t, 'bump')]
    # The node got it 3 ms before that tick (the recorder got it late).
    recorded = fake_node(config, stimuli, -10.0, 6.0, 0.0, node_late={0: -0.005})
    result = replay(config, stimuli, recorded)
    assert max_diff(recorded, result) < EXACT
    assert result.moved == 0


def test_stimulus_received_exactly_with_a_state_that_does_not_show_it():
    config = shipped()
    probe = fake_node(config, [], -10.0, 6.0, 0.0)
    t = probe[50].t
    stimuli = [stim(t, 'bump')]
    # The node got it 1 ms after the bag did: after the tick that state came from.
    recorded = fake_node(config, stimuli, -10.0, 6.0, 0.0, node_late={0: 0.001})
    result = replay(config, stimuli, recorded)
    assert max_diff(recorded, result) < EXACT
    assert result.moved == 1


def test_stimulus_one_ms_after_a_node_tick_with_two_ms_latency():
    # The reviewer's case: tick at w, stimulus at w + 1 ms, that tick's state received at
    # w + 2 ms. The bag has the stimulus first, but the node applied it after the tick.
    config = shipped()
    w = 3.0
    stimuli = [stim(w + 0.001, 'bump')]
    recorded = fake_node(config, stimuli, -10.0, 8.0, 0.0, latency=0.002)
    assert any(abs(r.t - (w + 0.002)) < 1e-9 for r in recorded)
    result = replay(config, stimuli, recorded)
    assert max_diff(recorded, result) < EXACT        # was 0.599 on arousal
    assert result.moved == 1


def test_stimulus_the_node_handled_two_ticks_late():
    # A loaded machine: the bag has the stimulus 10+ ms before a state, but the node only
    # handled it 130 ms later. The idle reset in the recording says which tick.
    config = shipped()
    stimuli = [stim(3.0405, 'bump'), stim(8.0305, 'face_known', subject='dave')]
    recorded = fake_node(config, stimuli, -10.0, 12.0, 0.0, node_late={0: 0.13, 1: 0.2})
    result = replay(config, stimuli, recorded)
    assert max_diff(recorded, result) < EXACT
    assert result.moved == 2
    assert result.events == []          # the late jump is not mistaken for a force


def test_without_a_visible_idle_reset_both_orders_are_tried():
    # Two stimuli while the sim clock is paused: the second leaves idle_s at 0, so the
    # recording cannot say which tick it came before; the axes decide.
    config = shipped()
    probe = fake_node(config, [], -10.0, 12.0, 0.0, pause=(2.0, 9.0))
    t = probe[50].t                                  # a state received during the pause
    stimuli = [stim(t - 0.45, 'bump'), stim(t - 0.001, 'touched')]
    recorded = fake_node(config, stimuli, -10.0, 12.0, 0.0, pause=(2.0, 9.0),
                         node_late={1: 0.003})
    result = replay(config, stimuli, recorded)
    assert max_diff(recorded, result) < EXACT
    assert result.moved == 1


# --- SetMood inferred from the recording -----------------------------------------------------

def test_recording_that_starts_inside_a_timed_lock_unlocks_when_it_expires():
    config = shipped()
    calls = [(-1.05, 'excited', True, 4.0, 'console')]      # locked until 2.95 (wall)
    recorded = fake_node(config, SCENARIO, -10.0, 20.0, 0.0, set_moods=calls)
    assert recorded[0].locked and recorded[0].lock_until == pytest.approx(SIM_T0 + 2.95)
    result = replay(config, list(SCENARIO), recorded)
    assert max_diff(recorded, result) < EXACT
    assert not result.rows[-1].state.locked
    assert result.events == []                         # an expiry is not a SetMood


def test_a_clear_mid_recording_is_replayed():
    config = shipped()
    calls = [(-1.05, '', True, 0.0, 'console'), (5.55, '', False, 0.0, 'console')]
    recorded = fake_node(config, SCENARIO, -10.0, 20.0, 0.0, set_moods=calls)
    result = replay(config, list(SCENARIO), recorded)
    assert max_diff(recorded, result) < EXACT
    assert [e.what for e in result.events] == ['clear']


@pytest.mark.parametrize('call, what', [
    (('sleepy', False, 0.0, 'console'), 'force sleepy'),
    (('sleepy', True, 0.0, 'console'), 'force sleepy, lock by console until cleared'),
    (('excited', True, 3.0, 'console'), 'force excited, lock by console until 1008.550'),
    (('', True, 0.0, 'console'), 'lock by console until cleared'),
])
def test_a_force_or_lock_mid_recording_is_replayed(call, what):
    config = shipped()
    recorded = fake_node(config, SCENARIO, -10.0, 20.0, 0.0, set_moods=[(5.55,) + call])
    result = replay(config, list(SCENARIO), recorded)
    assert max_diff(recorded, result) < EXACT
    assert [e.what for e in result.events] == [what]


def test_a_replayed_force_goes_to_the_replay_configs_prototype():
    config = shipped()
    recorded = fake_node(config, [], -10.0, 10.0, 0.0,
                         set_moods=[(5.55, 'sleepy', True, 0.0, 'console')])
    moved = parse_config_with(moods={'sleepy': {'arousal': -0.5, 'valence': 0.1}})
    result = replay(moved, [], recorded)
    assert result.rows[-1].state.arousal == pytest.approx(-0.5)
    assert result.rows[-1].state.locked


def parse_config_with(**changes):
    import yaml
    with open(SHIPPED) as f:
        data = yaml.safe_load(f)
    for key, value in changes.items():
        data[key].update(value)
    return parse_config(data)


def test_infer_set_mood_rules():
    a = rec(0.1, mood='calm', mood_strength=0.9)
    # Natural relax: no force.
    assert infer_set_mood(a, a._replace(t=0.2, stamp=0.2, arousal=0.01), False) == \
        (False, False, False)
    # A label change at full strength is a force, even with a stimulus around.
    forced = a._replace(t=0.2, stamp=0.2, mood='sleepy', mood_strength=0.995, arousal=-0.8)
    assert infer_set_mood(a, forced, True) == (False, True, False)
    # A jump without a stimulus is a force (same label); with one it is the stimulus.
    jump = a._replace(t=0.2, stamp=0.2, arousal=0.5)
    assert infer_set_mood(a, jump, False)[1] and not infer_set_mood(a, jump, True)[1]
    # A lock that vanishes before its time is a clear; at its time it expired.
    locked = a._replace(locked=True, lock_source='x', lock_until=5.0)
    assert infer_set_mood(locked, a._replace(t=0.2, stamp=0.2), False)[0]
    assert not infer_set_mood(locked, a._replace(t=5.0, stamp=5.0), False)[0]
    # The same lock again is not a new lock; another holder or end is.
    assert not infer_set_mood(locked, locked._replace(t=0.2, stamp=0.2), False)[2]
    assert infer_set_mood(locked, locked._replace(lock_until=9.0), False)[2]
    assert infer_set_mood(locked, locked._replace(lock_source='y'), False)[2]


# --- parameters and config -------------------------------------------------------------------

def test_min_confidence_mirrors_the_node():
    config = shipped()
    stimuli = [stim(2.047, 'bump', confidence=0.3), stim(7.012, 'touched')]
    recorded = fake_node(config, stimuli, -10.0, 12.0, 0.0, min_confidence=0.5)
    assert max_diff(recorded, replay(config, stimuli, recorded, min_confidence=0.5)) < EXACT
    assert max_diff(recorded, replay(config, stimuli, recorded)) > TOLERANCE


def test_doubling_an_impulse_doubles_its_delta():
    # Half intensity, so that the doubled impulse is not clamped at the end of a range.
    stimuli = [stim(0.05, 'bump', intensity=0.5)]
    base = replay(shipped(), stimuli, []).outcomes[0].result.delta
    double = replay(shipped_with(bump=2.0), stimuli, []).outcomes[0].result.delta
    for axis in ('arousal', 'valence', 'confidence'):
        assert double[axis] == pytest.approx(2.0 * base[axis])


def test_starting_from_the_temperament_does_not_reproduce_a_running_node():
    config = shipped()
    recorded = fake_node(config, SCENARIO, -100.0, 30.0, 0.03)
    result = replay(config, list(SCENARIO), recorded, start='temperament')
    assert result.seed is None
    assert max_diff(recorded, result) > TOLERANCE


def test_a_different_config_is_reported_as_a_difference():
    recorded = fake_node(shipped(), SCENARIO, -10.0, 20.0, 0.03)
    result = replay(shipped_with(bump=2.0), list(SCENARIO), recorded)
    axis, diff = worst_axis(compare(recorded, result.rows))
    assert axis in ('arousal', 'valence') and diff > 0.3


def test_compare_reports_the_axis_time_labels_and_locks():
    config = shipped()
    recorded = fake_node(config, SCENARIO, -10.0, 20.0, 0.03)
    rows = replay(config, list(SCENARIO), recorded).rows
    bad = recorded[50]._replace(valence=recorded[50].valence + 0.05, mood='sleepy',
                                locked=True)
    recorded[50] = bad
    cmp = compare(recorded, rows)
    assert worst_axis(cmp) == ('valence', pytest.approx(0.05))
    assert cmp.max_diff_t['valence'] == bad.stamp
    assert cmp.label_disagreements == 1 and cmp.any_locked


def test_compare_needs_one_tick_per_state():
    recorded = fake_node(shipped(), [], -1.0, 1.0)
    rows = replay(shipped(), [], recorded).rows
    with pytest.raises(ValueError):
        compare(recorded, rows[:-1])


# --- output ----------------------------------------------------------------------------------

def test_csv_has_a_header_and_a_row_per_tick():
    recorded = fake_node(shipped(), SCENARIO, -10.0, 15.0, 0.03)
    result = replay(shipped(), list(SCENARIO), recorded)
    buf = io.StringIO()
    write_csv(buf, result.rows, result.t0)
    lines = buf.getvalue().splitlines()
    assert lines[0].split(',') == list(CSV_FIELDS)
    assert len(lines) == len(recorded) + 1
    first = lines[1].split(',')
    assert first[0] == '0.000' and first[5] == recorded[0].mood and first[-1] == '0'


def test_recorded_transitions_lists_label_changes():
    states = [rec(t, mood=mood) for t, mood in
              [(0.1, 'calm'), (0.2, 'calm'), (0.3, 'annoyed'), (0.4, 'annoyed'), (0.5, 'calm')]]
    assert [(tr.t, tr.old, tr.new) for tr in recorded_transitions(states)] == \
        [(0.3, 'calm', 'annoyed'), (0.5, 'annoyed', 'calm')]
