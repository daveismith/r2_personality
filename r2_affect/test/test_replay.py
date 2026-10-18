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

import copy
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
from r2_affect.replay import align, compare, CSV_FIELDS, plan_replay  # noqa: E402
from r2_affect.replay import recorded_transitions, RecordedState, replay  # noqa: E402
from r2_affect.replay import run_replay, seed_model, StimulusEvent  # noqa: E402
from r2_affect.replay import tick_grid, TOLERANCE, worst_axis, write_csv  # noqa: E402

SHIPPED = os.path.join(PKG_ROOT, 'config', 'affect.yaml')
NAN = float('nan')


def shipped():
    return load_config(SHIPPED)


def shipped_with(**impulse_scale):
    """Return the shipped config with the named kinds' impulses multiplied."""
    import yaml
    with open(SHIPPED) as f:
        data = yaml.safe_load(f)
    data = copy.deepcopy(data)
    for kind, factor in impulse_scale.items():
        for key, value in data['impulses'][kind].items():
            if key != 'refractory_s':
                data['impulses'][kind][key] = value * factor
    return parse_config(data)


def to_recorded(t, state):
    return RecordedState(t=t, arousal=state.arousal, valence=state.valence,
                         curiosity=state.curiosity, confidence=state.confidence,
                         mood=state.mood, mood_strength=state.mood_strength,
                         locked=state.locked, lock_source=state.lock_source,
                         idle_s=state.idle_s)


def fake_node(config, stimuli, start, stop, phase, latency=0.002, record_from=None,
              rate_hz=10.0):
    """
    Run a model the way the node does and record its /mood/state.

    The "node" ticks at ``phase + k / rate_hz`` from ``start``, applies each
    stimulus at its time before the next tick, and each published state is
    "received" ``latency`` after its tick. States before ``record_from`` are
    not recorded (the node was running before the bag started).
    """
    m = AffectModel(config)
    recorded = []
    i = 0
    k = 0
    while True:
        t = phase + start + k / rate_hz
        if t > stop:
            return recorded
        while i < len(stimuli) and stimuli[i].t <= t:
            s = stimuli[i]
            m.stimulus(s.kind, s.intensity, s.confidence, s.subject, s.bearing_rad, s.ttl_s, s.t)
            i += 1
        m.tick(t)
        if record_from is None or t + latency >= record_from:
            recorded.append(to_recorded(t + latency, m.state))
        k += 1


# The dry-run scenario of the README: bump, face_known, touched, 5 s apart, received
# between ticks (never within the latency of one).
SCENARIO = [
    StimulusEvent(t=2.047, kind='bump', intensity=1.0, confidence=1.0, source='test'),
    StimulusEvent(t=7.012, kind='face_known', intensity=1.0, confidence=1.0,
                  subject='dave', bearing_rad=0.8, source='test'),
    StimulusEvent(t=12.088, kind='touched', intensity=1.0, confidence=1.0, source='test'),
]


# --- the grid --------------------------------------------------------------------------------

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


# --- the replay ------------------------------------------------------------------------------

def test_stimulus_is_applied_before_the_next_tick_as_in_the_node():
    m = AffectModel(shipped())
    stim = [StimulusEvent(t=0.25, kind='bump', intensity=1.0, confidence=1.0)]
    rows, transitions, outcomes = run_replay(m, stim, tick_grid(0.0, 0.5, 10.0))
    arousal = [row.state.arousal for row in rows]
    assert arousal[2] < 0.01                 # t = 0.2: not yet
    assert arousal[3] > 0.55                 # t = 0.3: applied, then one tick of relax
    assert len(outcomes) == 1 and outcomes[0].result.applied
    assert outcomes[0].result.delta['arousal'] == pytest.approx(0.6)


def test_stimulus_on_a_tick_is_applied_before_that_tick():
    m = AffectModel(shipped())
    stim = [StimulusEvent(t=0.2, kind='bump', intensity=1.0, confidence=1.0)]
    rows, _, _ = run_replay(m, stim, [0.0, 0.1, 0.2, 0.3])
    assert rows[1].state.arousal < 0.01
    assert rows[2].state.arousal > 0.55


def test_stimuli_after_the_last_tick_are_not_applied():
    m = AffectModel(shipped())
    stim = [StimulusEvent(t=0.45, kind='bump', intensity=1.0, confidence=1.0)]
    rows, _, outcomes = run_replay(m, stim, [0.0, 0.1, 0.2])
    assert outcomes == [] and len(rows) == 3


def test_replay_records_refractory_drops_and_transitions():
    stim = [StimulusEvent(t=1.0, kind='bump', intensity=1.0, confidence=1.0),
            StimulusEvent(t=1.5, kind='bump', intensity=1.0, confidence=1.0),
            StimulusEvent(t=5.0, kind='nonsense', intensity=1.0, confidence=1.0)]
    t0, seed, rows, transitions, outcomes = replay(shipped(), stim, [], start='temperament')
    assert seed is None and t0 == 1.0
    assert [o.result.reason for o in outcomes] == ['', 'refractory', 'unknown_kind']
    assert [(tr.old, tr.new) for tr in transitions] == [('calm', 'annoyed')]
    # The label waits for the dwell (1.5 s) after the first tick that prefers annoyed.
    assert transitions[0].t - 1.0 == pytest.approx(1.5, abs=0.1 + 1e-9)


def test_doubling_an_impulse_doubles_its_delta():
    # Half intensity, so that the doubled impulse is not clamped at the end of a range.
    stim = [StimulusEvent(t=0.05, kind='bump', intensity=0.5, confidence=1.0)]
    base = replay(shipped(), stim, [], start='temperament')[4][0].result.delta
    double = replay(shipped_with(bump=2.0), stim, [], start='temperament')[4][0].result.delta
    for axis in ('arousal', 'valence', 'confidence'):
        assert double[axis] == pytest.approx(2.0 * base[axis])


# --- the model against itself ----------------------------------------------------------------

@pytest.mark.parametrize('phase', [0.0, 0.03, 0.071])
def test_replay_of_the_model_against_itself_has_zero_diff(phase):
    config = shipped()
    # The node runs for 100 s idle before the recording starts at t = 0, so the replay
    # has to start from the recorded state (idle drift under way), not the temperament.
    recorded = fake_node(config, SCENARIO, -100.0, 30.0, phase, record_from=0.0)
    stimuli = list(SCENARIO)
    t0, seed, rows, transitions, _ = replay(config, stimuli, recorded)
    assert seed is recorded[0] and seed.idle_s > 90.0
    result = compare(recorded, rows, stimuli)
    assert result.compared == len(recorded)
    for axis in AXES:
        assert result.max_diff[axis] < 1e-9, axis
    assert result.label_disagreements == 0
    assert not result.any_locked
    assert [(tr.old, tr.new) for tr in transitions] == \
        [(tr.old, tr.new) for tr in recorded_transitions(recorded)]
    assert transitions, 'the scenario should change the label at least once'


def test_starting_from_the_temperament_does_not_reproduce_a_running_node():
    config = shipped()
    recorded = fake_node(config, SCENARIO, -100.0, 30.0, 0.03, record_from=0.0)
    _, _, rows, _, _ = replay(config, list(SCENARIO), recorded, start='temperament')
    assert worst_axis(compare(recorded, rows, SCENARIO))[1] > TOLERANCE


def test_a_different_config_is_reported_as_a_difference():
    recorded = fake_node(shipped(), SCENARIO, -10.0, 20.0, 0.03, record_from=0.0)
    _, _, rows, _, _ = replay(shipped_with(bump=2.0), list(SCENARIO), recorded)
    axis, diff = worst_axis(compare(recorded, rows, SCENARIO))
    assert axis in ('arousal', 'valence') and diff > 0.3


def test_compare_reports_the_axis_time_labels_and_locks():
    config = shipped()
    recorded = fake_node(config, SCENARIO, -10.0, 20.0, 0.03, record_from=0.0)
    _, _, rows, _, _ = replay(config, list(SCENARIO), recorded)
    bad = recorded[50]._replace(valence=recorded[50].valence + 0.05, mood='sleepy',
                                locked=True)
    recorded[50] = bad
    result = compare(recorded, rows, SCENARIO)
    assert worst_axis(result) == ('valence', pytest.approx(0.05))
    assert result.max_diff_t['valence'] == bad.t
    assert result.label_disagreements == 1
    assert result.any_locked


# --- alignment -------------------------------------------------------------------------------

def test_align_picks_the_nearest_tick():
    ticks = [0.0, 0.1, 0.2, 0.3]
    assert align([0.04, 0.06, 0.29, 5.0, -1.0], ticks, []) == [0, 1, 3, 3, 0]


def test_align_keeps_a_state_on_its_side_of_a_stimulus():
    ticks = [0.0, 0.1, 0.2, 0.3]
    # Received after the stimulus at 0.12: compared with 0.2 although 0.1 is nearer.
    assert align([0.125], ticks, [0.12]) == [2]
    # Received before it: compared with 0.1 although 0.2 would be as near.
    assert align([0.119], ticks, [0.12]) == [1]


def test_align_falls_back_to_the_nearest_when_no_tick_has_seen_the_same_stimuli():
    ticks = [0.0, 0.1, 0.2, 0.3]
    # Two stimuli within one period: no tick has seen the first and not the second.
    assert align([0.135], ticks, [0.12, 0.15]) == [1]


def test_align_needs_ticks():
    with pytest.raises(ValueError):
        align([0.0], [], [])


# --- start and seed --------------------------------------------------------------------------

def test_plan_starts_at_the_first_recorded_state_and_drops_earlier_stimuli():
    stim = [StimulusEvent(t=0.5, kind='bump', intensity=1.0, confidence=1.0),
            StimulusEvent(t=2.0, kind='touched', intensity=1.0, confidence=1.0)]
    recorded = [RecordedState(1.03, 0.1, 0.2, 0.4, 0.6, 'calm', 0.9),
                RecordedState(3.03, 0.1, 0.2, 0.4, 0.6, 'calm', 0.9)]
    seed, t0, ticks, kept = plan_replay(stim, recorded, 10.0)
    assert seed is recorded[0] and t0 == 1.03
    assert [s.kind for s in kept] == ['touched']
    assert ticks[0] == 1.03 and ticks[-1] == pytest.approx(3.03)


def test_plan_from_the_temperament_starts_at_the_first_message():
    stim = [StimulusEvent(t=0.5, kind='bump', intensity=1.0, confidence=1.0)]
    recorded = [RecordedState(1.03, 0.1, 0.2, 0.4, 0.6, 'calm', 0.9)]
    seed, t0, ticks, kept = plan_replay(stim, recorded, 10.0, 'temperament')
    assert seed is None and t0 == 0.5 and len(kept) == 1
    assert plan_replay(stim, [], 10.0)[0] is None       # no state in the bag: temperament


def test_plan_needs_something():
    with pytest.raises(ValueError):
        plan_replay([], [], 10.0)
    with pytest.raises(ValueError):
        plan_replay([], [RecordedState(0.0, 0, 0, 0, 0, 'calm', 0)], 10.0, 'sideways')


def test_seed_sets_vector_label_idle_and_lock():
    m = AffectModel(shipped())
    rec = RecordedState(12.0, -0.5, 0.1, 0.3, 0.45, 'sleepy', 0.3, True, 'mood_cli', 42.0)
    seed_model(m, rec, 12.0)
    s = m.state
    assert (s.arousal, s.valence, s.curiosity, s.confidence) == (-0.5, 0.1, 0.3, 0.45)
    assert s.mood == 'sleepy' and s.idle_s == 42.0
    assert s.locked and s.lock_source == 'mood_cli' and s.lock_until is None
    # It counts as a tick at 12.0: the next tick advances by the difference.
    assert m.tick(12.1).dt == pytest.approx(0.1)


def test_seed_with_an_unknown_label_takes_the_nearest_and_clamps():
    m = AffectModel(shipped())
    seed_model(m, RecordedState(0.0, -1.5, 0.0, 0.4, 0.6, 'gone', 0.0), 0.0)
    assert m.state.arousal == -1.0 and m.state.mood == 'sleepy'


# --- output ----------------------------------------------------------------------------------

def test_csv_has_a_header_and_a_row_per_tick():
    _, _, rows, _, _ = replay(shipped(), list(SCENARIO), [], start='temperament')
    buf = io.StringIO()
    write_csv(buf, rows, rows[0].t)
    lines = buf.getvalue().splitlines()
    assert lines[0].split(',') == list(CSV_FIELDS)
    assert len(lines) == len(rows) + 1
    first = lines[1].split(',')
    assert first[0] == '0.000' and first[5] == 'calm' and first[-1] == '0'
    assert len(first) == len(CSV_FIELDS)


def test_recorded_transitions_lists_label_changes():
    rec = [RecordedState(t, 0, 0, 0, 0, mood, 0) for t, mood in
           [(0.0, 'calm'), (0.1, 'calm'), (0.2, 'annoyed'), (0.3, 'annoyed'), (0.4, 'calm')]]
    assert [(tr.t, tr.old, tr.new) for tr in recorded_transitions(rec)] == \
        [(0.2, 'calm', 'annoyed'), (0.4, 'annoyed', 'calm')]
