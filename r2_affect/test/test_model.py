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

"""Unit tests for the mood model (no ROS graph, no sleeping: tests advance ``now``)."""

import copy
import math
import os
import sys

import pytest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

from r2_affect.config import load_config, parse_config  # noqa: E402
from r2_affect.model import AffectModel, distance  # noqa: E402
from r2_affect.model import DROP_LOW_CONFIDENCE, DROP_REFRACTORY  # noqa: E402
from r2_affect.model import DROP_RESERVED_KIND, DROP_UNKNOWN_KIND  # noqa: E402
from r2_affect.model import MAX_DURATION_S, NOT_APPLIED_LOCKED  # noqa: E402

NAN = float('nan')
DT = 0.1
SLOW = 1e9   # a tau long enough that relaxation is invisible

BASE = {
    'temperament': {'arousal': 0.0, 'valence': 0.0, 'curiosity': 0.5, 'confidence': 0.5},
    'relax_tau_s': {'arousal': SLOW, 'valence': SLOW, 'curiosity': SLOW, 'confidence': SLOW},
    'strength_radius': 0.5,
    'hysteresis': {'margin': 0.1, 'dwell_s': 1.0},
    'focus': {'default_ttl_s': 2.0},
    'idle': {'after_s': 1000.0, 'ramp_s': 2.0},
    'impulses': {
        'poke': {'arousal': 0.4, 'valence': -0.2},
        'lift': {'arousal': 0.8},
        'drop': {'arousal': -0.8},
        'pat': {'valence': 0.3, 'refractory_s': 1.0},
        'slam': {'arousal': 3.0, 'curiosity': -5.0},
        'idle': {'arousal': -0.1},
    },
    'moods': {
        'neutral': {'arousal': 0.0, 'valence': 0.0},
        'up': {'arousal': 0.8},
        'down': {'arousal': -0.8},
        'happy': {'valence': 0.8, 'confidence': 0.9},
    },
}


def config(**changes):
    """Return BASE with ``changes``: sections merge key by key, except ``moods``, replaced."""
    data = copy.deepcopy(BASE)
    for key, value in changes.items():
        if key != 'moods' and isinstance(value, dict) and isinstance(data.get(key), dict):
            data[key].update(value)
        else:
            data[key] = value
    return parse_config(data)


def model(min_confidence=0.0, max_dt_s=0.5, **changes):
    m = AffectModel(config(**changes), min_confidence=min_confidence, max_dt_s=max_dt_s)
    m.tick(0.0)
    return m


def stim(m, kind, now, intensity=1.0, confidence=1.0, subject='', bearing=NAN, ttl=0.0):
    return m.stimulus(kind, intensity, confidence, subject, bearing, ttl, now)


def run(m, start, stop):
    """Tick every DT from ``start`` to ``stop`` inclusive; return the tick results."""
    results = []
    k = 0
    while True:
        now = round(start + k * DT, 9)
        if now > stop + 1e-9:
            return results
        results.append(m.tick(now))
        k += 1


def assert_same_state(a, b):
    for field in a.__dataclass_fields__:
        x, y = getattr(a, field), getattr(b, field)
        if isinstance(x, float) and math.isnan(x):
            assert isinstance(y, float) and math.isnan(y), field
        else:
            assert x == y, field


# --- dynamics ---------------------------------------------------------------------------

def test_relaxes_toward_baseline_with_the_configured_tau():
    taus = {'arousal': 5.0, 'valence': 10.0, 'curiosity': 5.0, 'confidence': 20.0}
    m = model(relax_tau_s=taus)
    assert m.set_mood('up', False, 0.0, 'test', 0.0)[0]      # arousal 0.8
    assert m.set_mood('happy', False, 0.0, 'test', 0.0)[0]   # valence 0.8, confidence 0.9
    x0 = m.state.vector()
    base = m.config.temperament
    results = run(m, DT, 20 * DT)
    assert len(results) == 20
    t = 20 * DT
    for axis in ('arousal', 'valence', 'confidence'):
        expected = base[axis] + (x0[axis] - base[axis]) * math.exp(-t / taus[axis])
        assert m.state.vector()[axis] == pytest.approx(expected, rel=0.02), axis
    assert m.state.curiosity == pytest.approx(0.5)            # at its baseline already


def test_relaxation_never_overshoots_when_tau_is_shorter_than_dt():
    m = model(relax_tau_s={'arousal': 0.01})
    stim(m, 'lift', 0.0)
    m.tick(DT)
    assert m.state.arousal == pytest.approx(0.0)


def test_impulse_scales_with_intensity():
    m = model()
    r = stim(m, 'poke', 0.0, intensity=0.5)
    assert r.applied and r.reason == ''
    assert r.delta['arousal'] == pytest.approx(0.2)
    assert r.delta['valence'] == pytest.approx(-0.1)
    assert r.delta['curiosity'] == 0.0 and r.delta['confidence'] == 0.0
    stim(m, 'poke', 0.0, intensity=0.25)
    assert m.state.arousal == pytest.approx(0.3)
    assert m.state.valence == pytest.approx(-0.15)


def test_intensity_is_clamped_to_unit_range():
    m = model()
    assert stim(m, 'poke', 0.0, intensity=7.0).delta['arousal'] == pytest.approx(0.4)
    assert stim(m, 'poke', 0.0, intensity=-1.0).delta['arousal'] == 0.0


def test_nan_inputs_never_reach_the_state():
    m = model()
    r = stim(m, 'poke', 0.0, intensity=NAN)
    assert r.intensity == 0.0 and r.delta['arousal'] == 0.0
    assert stim(m, 'poke', 0.0, confidence=NAN).reason == DROP_LOW_CONFIDENCE
    assert m.set_mood('', True, NAN, 'a', 0.0) == (
        False, 'duration_s must be a finite number >= 0, got nan')
    assert all(math.isfinite(v) for v in m.state.vector().values())
    with pytest.raises(ValueError):
        m.tick(NAN)


def test_axes_clamp_to_their_ranges():
    m = model()
    r = stim(m, 'slam', 0.0)
    assert m.state.arousal == 1.0 and m.state.curiosity == 0.0
    assert r.delta['arousal'] == pytest.approx(1.0)       # the change actually made
    assert r.delta['curiosity'] == pytest.approx(-0.5)
    for _ in range(3):
        stim(m, 'drop', 0.0)
    assert m.state.arousal == -1.0


def test_idle_drift_clamps_too():
    m = model(idle={'after_s': 0.0, 'ramp_s': 0.1}, impulses={'idle': {'arousal': -50.0}})
    run(m, DT, 2.0)
    assert m.state.arousal == -1.0


# --- refractory and unknown kinds -------------------------------------------------------

def test_refractory_drops_the_second_of_two_fast_stimuli_and_accepts_after_the_period():
    m = model()
    assert stim(m, 'pat', 0.0).applied
    before = m.state
    second = stim(m, 'pat', 0.5)
    assert not second.accepted and second.reason == DROP_REFRACTORY
    assert second.delta['valence'] == 0.0
    assert m.state.valence == before.valence
    assert stim(m, 'pat', 0.9).reason == DROP_REFRACTORY    # drops do not extend the window
    assert stim(m, 'pat', 1.0).applied                      # exactly one period after
    assert m.state.valence == pytest.approx(0.6)


def test_refractory_is_measured_with_float_ticks():
    m = model()
    stim(m, 'pat', 0.1)
    now = 0.1
    for _ in range(10):
        now += 0.1                                          # 1.0999999999999999
    assert stim(m, 'pat', now).applied


def test_kind_without_refractory_is_never_dropped():
    m = model()
    results = [stim(m, 'poke', 0.0, intensity=0.1) for _ in range(10)]
    assert all(r.applied for r in results)
    assert m.state.arousal == pytest.approx(0.4)


def test_unknown_kind_is_reported_not_applied():
    m = model()
    before = m.state
    r = stim(m, 'nonsense', 0.0)
    assert not r.accepted and not r.applied and r.reason == DROP_UNKNOWN_KIND
    assert all(v == 0.0 for v in r.delta.values())
    assert m.state.vector() == before.vector()


# --- classification ---------------------------------------------------------------------

def test_startup_label_is_the_nearest_prototype():
    m = AffectModel(config(temperament={'valence': 0.7, 'confidence': 0.8}))
    assert m.state.mood == 'happy'


def test_equal_distances_go_to_the_first_mood_in_file_order():
    moods = {'left': {'arousal': -0.5}, 'right': {'arousal': 0.5}}
    assert AffectModel(config(moods=moods)).state.mood == 'left'
    moods = {'right': {'arousal': 0.5}, 'left': {'arousal': -0.5}}
    assert AffectModel(config(moods=moods)).state.mood == 'right'


def test_distance_uses_only_specified_axes_scaled_by_half_range_and_weight():
    x = {'arousal': 0.0, 'valence': 0.0, 'curiosity': 0.5, 'confidence': 0.5}
    ones = {'arousal': 1.0, 'valence': 1.0, 'curiosity': 1.0, 'confidence': 1.0}
    assert distance(x, {'arousal': 0.3, 'valence': 0.4}, ones) == pytest.approx(0.5)
    assert distance(x, {'curiosity': 1.0}, ones) == pytest.approx(1.0)   # half-range 0.5
    assert distance(x, {'curiosity': 1.0}, dict(ones, curiosity=2.0)) == pytest.approx(2.0)
    x['confidence'] = 0.0
    assert distance(x, {'arousal': 0.3, 'valence': 0.4}, ones) == pytest.approx(0.5)
    m = AffectModel(config(distance_weights={'arousal': 3.0}))
    assert m.distances()['up'] == pytest.approx(2.4)


def test_label_switches_only_after_dwell():
    m = model()
    stim(m, 'lift', 0.05)                                   # arousal 0.8: 'up' wins by 0.8
    results = run(m, DT, 1.1)
    labels = [r.state.mood for r in results]
    # The condition first holds at the 0.1 tick; the switch comes dwell_s (1.0) later.
    assert labels[:-1] == ['neutral'] * 10
    assert labels[-1] == 'up'
    assert results[-1].transition.old == 'neutral'
    assert results[-1].transition.new == 'up'
    assert results[-1].transition.distance == pytest.approx(0.0, abs=1e-6)
    assert all(r.transition is None for r in results[:-1])


def test_label_switches_only_past_the_margin():
    m = model()
    stim(m, 'lift', 0.0, intensity=0.55)                    # d_up 0.36, d_neutral 0.44
    results = run(m, DT, 5.0)
    assert all(r.state.mood == 'neutral' for r in results)
    stim(m, 'lift', 5.0, intensity=0.025)                   # d_up 0.34 < 0.46 - 0.1
    results = run(m, 5.1, 6.1)
    assert results[-2].state.mood == 'neutral'
    assert results[-1].state.mood == 'up'


def test_dwell_timer_restarts_when_the_margin_condition_lapses():
    m = model()
    stim(m, 'lift', 0.05)
    run(m, DT, 0.5)
    stim(m, 'drop', 0.55)                                   # back at neutral
    m.tick(0.6)
    stim(m, 'lift', 0.65)
    results = run(m, 0.7, 1.7)
    assert [r.state.mood for r in results[:-1]] == ['neutral'] * 10
    assert results[-1].state.mood == 'up'


def test_mood_strength_is_one_at_the_prototype_and_zero_at_strength_radius():
    m = model()
    m.set_mood('up', False, 0.0, 'test', 0.0)
    assert m.state.mood_strength == pytest.approx(1.0)
    stim(m, 'drop', 0.0, intensity=0.3125)                  # arousal 0.55: d = 0.25
    assert m.state.mood_strength == pytest.approx(0.5)
    stim(m, 'drop', 0.0, intensity=0.3125)                  # arousal 0.3: d = 0.5
    assert m.state.mood_strength == pytest.approx(0.0, abs=1e-9)
    stim(m, 'drop', 0.0)                                    # beyond the radius
    assert m.state.mood_strength == 0.0


# --- force, lock, clear -----------------------------------------------------------------

def test_force_jumps_specified_axes_only():
    m = model()
    stim(m, 'poke', 0.0)                                    # valence -0.2
    stim(m, 'slam', 0.0)                                    # curiosity 0
    assert m.set_mood('up', False, 0.0, 'test', 0.0) == (True, '')
    s = m.state
    assert s.arousal == 0.8
    assert s.valence == pytest.approx(-0.2)
    assert s.curiosity == 0.0 and s.confidence == 0.5
    assert not s.locked


def test_force_switches_the_label_immediately_and_restarts_the_dwell_timer():
    m = model()
    stim(m, 'lift', 0.05)
    run(m, DT, 0.9)                                         # 'up' pending since 0.1
    assert m.state.mood == 'neutral'
    m.set_mood('down', False, 0.0, 'test', 0.95)
    assert m.state.mood == 'down'                           # no tick needed
    assert m.state.arousal == -0.8
    stim(m, 'lift', 0.95, intensity=1.0)
    stim(m, 'lift', 0.95, intensity=1.0)                    # arousal 0.8 again
    results = run(m, 1.0, 2.0)
    assert [r.state.mood for r in results[:-1]] == ['down'] * 10
    assert results[-1].state.mood == 'up'


def test_force_without_lock_relaxes_back():
    m = model(relax_tau_s={'arousal': 5.0})
    m.set_mood('up', False, 0.0, 'test', 0.0)
    run(m, DT, 1.0)
    assert 0.0 < m.state.arousal < 0.8 * math.exp(-1.0 / 5.0) + 0.01


def test_locked_state_ignores_relax_and_impulses_but_updates_focus():
    m = model(relax_tau_s={'arousal': 1.0},
              idle={'after_s': 0.5, 'ramp_s': 0.1})
    assert m.set_mood('up', True, 0.0, 'console', 0.0) == (True, '')
    run(m, DT, 2.0)
    assert m.state.arousal == 0.8 and m.state.locked
    assert m.state.idle_s == pytest.approx(2.0)             # idle time still counts
    r = stim(m, 'poke', 2.0, subject='dave', bearing=0.8)
    assert r.accepted and not r.applied and r.reason == NOT_APPLIED_LOCKED
    assert all(v == 0.0 for v in r.delta.values())
    assert m.state.arousal == 0.8 and m.state.valence == 0.0
    assert m.state.focus_subject == 'dave'
    assert m.state.focus_bearing == pytest.approx(0.8)
    assert m.state.idle_s == 0.0


def test_locked_stimulus_still_starts_its_refractory_period():
    m = model()
    m.set_mood('', True, 0.0, 'console', 0.0)
    assert stim(m, 'pat', 0.0).reason == NOT_APPLIED_LOCKED
    m.set_mood('', False, 0.0, 'console', 0.1)
    assert stim(m, 'pat', 0.5).reason == DROP_REFRACTORY


def test_lock_with_empty_mood_freezes_the_vector_where_it_is():
    m = model(relax_tau_s={'arousal': 1.0})
    stim(m, 'poke', 0.0)
    before = m.state
    assert m.set_mood('', True, 0.0, 'console', 0.0) == (True, '')
    run(m, DT, 1.0)
    assert m.state.vector() == before.vector()
    assert m.state.mood == before.mood


def test_a_timed_lock_expires():
    m = model(relax_tau_s={'arousal': 1.0})
    m.set_mood('up', True, 1.0, 'console', 0.0)
    assert m.state.lock_until == pytest.approx(1.0)
    results = run(m, DT, 1.0)
    assert all(r.state.locked for r in results[:-1])
    assert results[-1].lock_expired
    s = results[-1].state
    assert not s.locked and s.lock_source == '' and s.lock_until is None
    assert s.arousal == 0.8                                 # that tick was still frozen
    m.tick(1.1)
    assert m.state.arousal < 0.8                            # dynamics resumed


def test_another_sources_lock_is_refused_and_the_reason_names_the_holder():
    m = model()
    m.set_mood('up', True, 0.0, 'console', 0.0)
    before = m.state
    accepted, reason = m.set_mood('', True, 10.0, 'other', 0.5)
    assert not accepted and "'console'" in reason
    accepted, reason = m.set_mood('down', False, 0.0, 'other', 0.5)
    assert not accepted and "'console'" in reason
    assert_same_state(m.state, before)


def test_the_holder_may_change_its_own_lock():
    m = model()
    m.set_mood('up', True, 0.0, 'console', 0.0)
    assert m.set_mood('down', True, 5.0, 'console', 1.0) == (True, '')
    assert m.state.mood == 'down' and m.state.lock_until == pytest.approx(6.0)
    assert m.set_mood('up', False, 0.0, 'console', 1.0) == (True, '')
    assert not m.state.locked                               # force without lock releases


def test_clear_always_works_from_any_source():
    m = model()
    assert m.set_mood('', False, 0.0, 'nobody', 0.0) == (True, '')   # nothing to clear
    m.set_mood('up', True, 0.0, 'console', 0.0)
    assert m.set_mood('', False, 0.0, 'other', 0.1) == (True, '')
    s = m.state
    assert not s.locked and s.lock_source == '' and s.lock_until is None
    assert s.arousal == 0.8 and s.mood == 'up'              # clear does not move the vector
    assert m.set_mood('', True, 0.0, 'other', 0.2) == (True, '')     # free to lock now


def test_a_lock_past_its_time_does_not_hold_others_off_before_the_next_tick():
    m = model()
    m.set_mood('', True, 1.0, 'console', 0.0)
    assert m.set_mood('', True, 0.0, 'other', 1.05) == (True, '')
    assert m.state.lock_source == 'other'


def test_empty_source_is_recorded_as_unknown():
    m = model()
    m.set_mood('', True, 0.0, '', 0.0)
    assert m.state.lock_source == 'unknown'


def test_unknown_mood_is_refused_and_changes_nothing():
    m = model()
    stim(m, 'poke', 0.0, subject='dave')
    before = m.state
    accepted, reason = m.set_mood('grumpy', True, 0.0, 'console', 0.1)
    assert not accepted and 'grumpy' in reason
    assert_same_state(m.state, before)


def test_negative_duration_is_refused_and_changes_nothing():
    m = model()
    before = m.state
    accepted, _ = m.set_mood('up', True, -1.0, 'console', 0.0)
    assert not accepted
    assert_same_state(m.state, before)


@pytest.mark.parametrize('duration', [MAX_DURATION_S + 1.0, 1e10, 1e20])
def test_duration_above_one_day_is_refused_and_changes_nothing(duration):
    m = model()
    before = m.state
    accepted, reason = m.set_mood('up', True, duration, 'console', 0.0)
    assert not accepted and reason == 'duration_s above one day'
    assert_same_state(m.state, before)
    assert m.set_mood('up', True, MAX_DURATION_S, 'console', 0.0) == (True, '')


# --- idle -------------------------------------------------------------------------------

def test_idle_ramps_after_after_s():
    m = model(idle={'after_s': 1.0, 'ramp_s': 2.0})
    run(m, DT, 1.0)
    assert m.state.arousal == 0.0                           # not idle long enough yet
    previous = m.state.arousal
    steps = {}
    for r in run(m, 1.1, 4.0):
        steps[round(r.state.idle_s, 6)] = r.state.arousal - previous
        previous = r.state.arousal
    # intensity = (idle_s - after_s) / ramp_s, applied as a rate of -0.1 per second
    assert steps[2.0] == pytest.approx(-0.1 * 0.5 * DT)
    assert steps[3.0] == pytest.approx(-0.1 * 1.0 * DT)
    assert steps[4.0] == pytest.approx(-0.1 * 1.0 * DT)     # full rate, no further growth


def test_idle_resets_on_any_confident_stimulus():
    m = model(min_confidence=0.5)
    run(m, DT, 2.0)
    assert m.state.idle_s == pytest.approx(2.0)
    stim(m, 'poke', 2.0, confidence=0.9)
    assert m.state.idle_s == 0.0


def test_low_confidence_stimulus_does_not_reset_idle_or_focus():
    m = model(min_confidence=0.5)
    run(m, DT, 2.0)
    r = stim(m, 'poke', 2.0, confidence=0.2, subject='dave', bearing=0.3)
    assert r.reason == DROP_LOW_CONFIDENCE and not r.focus_updated
    assert m.state.idle_s == pytest.approx(2.0)
    assert m.state.arousal == 0.0 and m.state.focus_subject == ''


def test_refractory_dropped_and_unknown_kind_stimuli_still_reset_idle():
    m = model()
    stim(m, 'pat', 0.0)
    run(m, DT, 0.5)
    assert stim(m, 'pat', 0.5).reason == DROP_REFRACTORY
    assert m.state.idle_s == 0.0
    run(m, 0.6, 0.9)
    assert stim(m, 'nonsense', 0.9, subject='box').reason == DROP_UNKNOWN_KIND
    assert m.state.idle_s == 0.0
    assert m.state.focus_subject == 'box'                   # focus comes before the kind check


# --- focus ------------------------------------------------------------------------------

def test_focus_expires_at_ttl():
    m = model()
    r = stim(m, 'poke', 0.05, subject='dave', bearing=0.8, ttl=1.0)
    assert r.focus_updated
    results = run(m, DT, 1.1)
    assert all(x.state.focus_subject == 'dave' for x in results[:-1])   # ticks 0.1 .. 1.0
    assert not any(x.focus_expired for x in results[:-1])
    assert results[-1].focus_expired
    s = results[-1].state
    assert s.focus_subject == '' and math.isnan(s.focus_bearing)


def test_focus_ttl_zero_uses_the_default():
    m = model()
    stim(m, 'poke', 0.0, subject='dave', ttl=0.0)
    run(m, DT, 2.0)
    assert m.state.focus_subject == 'dave'
    m.tick(2.1)
    assert m.state.focus_subject == ''


def test_focus_follows_the_latest_stimulus_with_a_subject_or_a_bearing():
    m = model()
    stim(m, 'poke', 0.0, subject='dave', bearing=0.8)
    stim(m, 'poke', 0.1)                                    # neither: focus unchanged
    assert m.state.focus_subject == 'dave'
    stim(m, 'poke', 0.2, bearing=-1.0)                      # a bearing alone takes focus
    assert m.state.focus_subject == '' and m.state.focus_bearing == pytest.approx(-1.0)
    stim(m, 'poke', 0.3, subject='box')                     # a subject with no bearing
    assert m.state.focus_subject == 'box' and math.isnan(m.state.focus_bearing)


def test_no_focus_reads_as_empty_subject_and_nan_bearing():
    s = model().state
    assert s.focus_subject == '' and math.isnan(s.focus_bearing)


# --- time -------------------------------------------------------------------------------

def test_dt_of_zero_changes_nothing():
    m = model(relax_tau_s={'arousal': 1.0}, idle={'after_s': 0.0})
    stim(m, 'lift', 0.0)
    m.tick(0.5)
    before = m.state
    for _ in range(5):
        r = m.tick(0.5)
        assert r.dt == 0.0 and r.transition is None and not r.dt_clamped
    assert_same_state(m.state, before)


def test_first_tick_establishes_the_time_base():
    m = AffectModel(config(relax_tau_s={'arousal': 1.0}))
    stim(m, 'lift', 1000.0)
    first = m.tick(1000.0)
    assert first.dt == 0.0 and first.raw_dt == 0.0 and not first.dt_clamped
    assert m.state.arousal == 0.8
    assert m.tick(1000.1).dt == pytest.approx(0.1)


def test_large_dt_is_clamped_and_reported():
    m = model(relax_tau_s={'arousal': 10.0})
    stim(m, 'lift', 0.0)
    r = m.tick(5.0)
    assert r.dt_clamped and r.dt == 0.5 and r.raw_dt == 5.0
    assert m.state.arousal == pytest.approx(0.8 * (1 - 0.5 / 10.0))
    assert m.state.idle_s == pytest.approx(0.5)
    assert not m.tick(5.1).dt_clamped


def test_backwards_clock_is_clamped_to_zero_and_rebased():
    m = model()
    m.tick(10.0)
    r = m.tick(9.0)
    assert r.dt_clamped and r.dt == 0.0 and r.raw_dt == pytest.approx(-1.0)
    assert m.tick(9.1).dt == pytest.approx(0.1)


# --- reload and independence ------------------------------------------------------------

def test_reload_keeps_the_state():
    m = model()
    stim(m, 'poke', 0.0, subject='dave')
    stim(m, 'pat', 0.0)
    m.set_mood('', True, 0.0, 'console', 0.0)
    before = m.state
    m.reload(config(impulses={'poke': {'arousal': 0.1}}))
    assert_same_state(m.state, before)
    assert stim(m, 'pat', 0.5).reason == DROP_REFRACTORY    # refractory history kept
    m.set_mood('', False, 0.0, 'console', 0.5)
    assert stim(m, 'poke', 0.5).delta['arousal'] == pytest.approx(0.1)   # new table used


def test_reload_relabels_when_the_current_mood_is_gone():
    m = model()
    m.set_mood('up', False, 0.0, 'test', 0.0)
    m.reload(config(moods={'neutral': {'arousal': 0.0}, 'high': {'arousal': 0.7}}))
    assert m.state.mood == 'high'


def test_models_share_no_state():
    shipped = load_config(os.path.join(PKG_ROOT, 'config', 'affect.yaml'))
    a, b = AffectModel(shipped), AffectModel(shipped)
    a.tick(0.0)
    b.tick(0.0)
    a.stimulus('bump', 1.0, 1.0, 'dave', 0.5, 0.0, 0.0)
    a.set_mood('', True, 0.0, 'console', 0.0)
    run(a, DT, 2.0)
    assert b.state.arousal == shipped.temperament['arousal']
    assert not b.state.locked and b.state.focus_subject == '' and b.state.idle_s == 0.0


# --- review additions -------------------------------------------------------------------

def test_backwards_clock_shifts_pending_label_lock_and_focus_times():
    m = AffectModel(config())
    m.tick(999.9)
    stim(m, 'lift', 1000.0, subject='dave', ttl=2.0)        # focus until 1002
    m.set_mood('', True, 5.0, 'console', 1000.0)            # lock until 1005
    m.tick(1000.0)                                          # 'up' pending since 1000
    reset = m.tick(0.0)                                     # the clock restarts at 0
    assert reset.dt_clamped and reset.dt == 0.0
    assert m.state.lock_until == pytest.approx(5.0)
    results = {round(k * DT, 6): r for k, r in enumerate(run(m, DT, 5.0), start=1)}
    assert results[0.9].state.mood == 'neutral'
    assert results[1.0].state.mood == 'up'                  # dwell 1.0 after 0 (was 1000)
    assert results[2.0].state.focus_subject == 'dave'
    assert results[2.1].focus_expired                       # 2 s after 0 (was 1000)
    assert results[4.9].state.locked
    assert results[5.0].lock_expired                        # 5 s after 0 (was 1000)


def test_backwards_clock_shifts_refractory_times():
    m = model()
    m.tick(1000.0)
    stim(m, 'pat', 1000.0)
    m.tick(0.0)
    assert stim(m, 'pat', 0.5).reason == DROP_REFRACTORY
    assert stim(m, 'pat', 1.0).applied


def test_idle_kind_is_reserved_and_dropped_before_idle_reset_and_focus():
    m = model()
    run(m, DT, 1.0)
    r = stim(m, 'idle', 1.0, subject='dave', bearing=0.3)
    assert not r.accepted and not r.applied and r.reason == DROP_RESERVED_KIND
    assert all(v == 0.0 for v in r.delta.values()) and not r.focus_updated
    assert m.state.idle_s == pytest.approx(1.0)
    assert m.state.focus_subject == '' and m.state.arousal == 0.0


def test_dwell_timer_restarts_when_the_candidate_changes():
    m = model(impulses={'half': {'arousal': 0.5}},
              moods={'neutral': {'arousal': 0.0}, 'mid': {'arousal': 0.5},
                     'up': {'arousal': 1.0}})
    stim(m, 'half', 0.05)                                   # 'mid' pending from 0.1
    run(m, DT, 0.8)
    stim(m, 'half', 0.85)                                   # 'up' nearest from 0.9
    results = run(m, 0.9, 1.9)
    assert [r.state.mood for r in results[:-1]] == ['neutral'] * 10   # 0.9 .. 1.8
    assert results[-1].state.mood == 'up'                   # a full dwell as candidate
    assert results[-1].transition.old == 'neutral'


def test_margin_boundary_at_exact_equality_does_not_switch():
    # Dyadic values, so the distances are exact: d_up 0.25 == d_neutral 0.5 - margin 0.25.
    m = model(relax_tau_s={'arousal': 1e300},
              hysteresis={'margin': 0.25, 'dwell_s': 0.0},
              impulses={'nudge': {'arousal': 0.5}},
              moods={'neutral': {'arousal': 0.0}, 'up': {'arousal': 0.75}})
    stim(m, 'nudge', 0.0)
    assert m.distances() == {'neutral': 0.5, 'up': 0.25}
    assert all(r.state.mood == 'neutral' for r in run(m, DT, 1.0))
    stim(m, 'nudge', 1.0, intensity=0.125)                  # arousal 0.5625: now past it
    assert m.tick(1.1).state.mood == 'up'


def test_confidence_equal_to_min_confidence_is_accepted():
    m = model(min_confidence=0.5)
    assert stim(m, 'poke', 0.0, confidence=0.5).applied


@pytest.mark.parametrize('bad', [-1.0, float('inf'), float('-inf'), NAN])
def test_min_confidence_must_be_finite_and_non_negative(bad):
    with pytest.raises(ValueError):
        AffectModel(config(), min_confidence=bad)


@pytest.mark.parametrize('ttl', [-1.0, float('inf')])
def test_negative_and_infinite_ttl_fall_back_to_the_default(ttl):
    m = model()
    stim(m, 'poke', 0.0, subject='dave', ttl=ttl)
    run(m, DT, 2.0)
    assert m.state.focus_subject == 'dave'
    m.tick(2.1)
    assert m.state.focus_subject == ''


def test_infinite_bearing_becomes_nan():
    m = model()
    assert not stim(m, 'poke', 0.0, bearing=float('inf')).focus_updated
    stim(m, 'poke', 0.0, subject='dave', bearing=float('-inf'))
    assert m.state.focus_subject == 'dave' and math.isnan(m.state.focus_bearing)


def test_focus_expires_while_locked():
    m = model()
    m.set_mood('', True, 0.0, 'console', 0.0)
    stim(m, 'poke', 0.0, subject='dave', ttl=1.0)
    run(m, DT, 1.1)
    assert m.state.locked and m.state.focus_subject == ''


def test_zero_refractory_never_drops():
    m = model(impulses={'tap': {'valence': 0.1, 'refractory_s': 0.0}})
    assert all(stim(m, 'tap', 0.0).applied for _ in range(5))
    assert m.state.valence == pytest.approx(0.5)


def test_reload_clears_a_pending_label_change():
    m = model()
    stim(m, 'lift', 0.05)
    run(m, DT, 0.9)                                         # 'up' pending since 0.1
    m.reload(config())
    results = run(m, 1.0, 2.0)
    assert [r.state.mood for r in results[:-1]] == ['neutral'] * 10
    assert results[-1].state.mood == 'up'                   # dwell counted from 1.0


def test_zero_dwell_switches_on_the_first_tick_past_the_margin():
    m = model(hysteresis={'dwell_s': 0.0})
    stim(m, 'lift', 0.05)
    r = m.tick(DT)
    assert r.state.mood == 'up' and r.transition.new == 'up'


def test_classification_continues_under_an_empty_mood_lock():
    m = model()
    stim(m, 'lift', 0.0)
    m.set_mood('', True, 0.0, 'console', 0.0)
    results = run(m, DT, 1.1)
    assert results[-1].state.mood == 'up' and results[-1].state.locked
    assert results[-1].state.arousal == 0.8


def test_another_sources_force_with_lock_is_refused_while_locked():
    m = model()
    m.set_mood('up', True, 0.0, 'console', 0.0)
    before = m.state
    accepted, reason = m.set_mood('down', True, 0.0, 'other', 0.5)
    assert not accepted and "'console'" in reason
    assert_same_state(m.state, before)


def test_dt_of_zero_defers_lock_and_focus_expiry():
    m = AffectModel(config())                               # no time base yet
    m.set_mood('', True, 1.0, 'console', 0.0)
    stim(m, 'poke', 0.0, subject='dave', ttl=1.0)
    first = m.tick(5.0)                                     # first tick: dt 0
    assert not first.lock_expired and not first.focus_expired
    assert first.state.locked and first.state.focus_subject == 'dave'
    second = m.tick(5.1)
    assert second.lock_expired and second.focus_expired


def test_stimulus_after_lock_until_reports_the_expiry_and_is_applied():
    m = model()
    m.set_mood('', True, 1.0, 'console', 0.0)
    run(m, DT, 0.9)
    r = stim(m, 'poke', 1.05)
    assert r.lock_expired and r.applied and not r.state.locked
    assert r.delta['arousal'] == pytest.approx(0.4)
    assert not stim(m, 'poke', 1.06).lock_expired


def test_set_mood_lock_expiry_is_seen_by_comparing_locked():
    m = model()
    m.set_mood('', True, 1.0, 'console', 0.0)
    was_locked = m.state.locked
    accepted, _ = m.set_mood('grumpy', False, 0.0, 'other', 1.05)   # refused, but...
    assert not accepted
    assert was_locked and not m.state.locked                # ...the lapsed lock is gone


# --- copy and restore (for replay) ------------------------------------------------------

def test_copy_is_independent():
    m = model()
    stim(m, 'pat', 0.5)
    c = m.copy()
    stim(c, 'poke', 0.6)
    run(c, 0.7, 1.0)
    assert m.state.arousal == 0.0 and c.state.arousal == pytest.approx(0.4)
    assert stim(m, 'pat', 1.0).reason == DROP_REFRACTORY        # history copied, not shared
    assert stim(c, 'pat', 1.6).applied


def test_restore_sets_the_state_as_if_ticked_at_now():
    import dataclasses
    m = model()
    snap = dataclasses.replace(
        m.state, arousal=0.7, valence=-0.3, curiosity=0.2, confidence=0.9, mood='up',
        idle_s=12.0, locked=True, lock_source='console', lock_until=7.0,
        focus_subject='dave', focus_bearing=0.5)
    m.restore(snap, 5.0)
    s = m.state
    assert (s.arousal, s.valence, s.curiosity, s.confidence) == (0.7, -0.3, 0.2, 0.9)
    assert s.mood == 'up' and s.idle_s == 12.0
    assert s.locked and s.lock_source == 'console' and s.lock_until == 7.0
    assert s.focus_subject == 'dave' and s.focus_bearing == 0.5
    assert m.tick(5.1).dt == pytest.approx(0.1)                # the time base is now
    assert m.tick(7.0).lock_expired
    assert m.tick(7.0 + 2.0 + 0.1).focus_expired               # default ttl from now (5 + 2)


def test_restore_clamps_unknown_labels_and_clears_lock_and_focus():
    import dataclasses
    m = model()
    m.set_mood('', True, 0.0, 'console', 0.0)
    stim(m, 'poke', 0.0, subject='dave')
    snap = dataclasses.replace(m.state, arousal=-3.0, mood='gone', locked=False,
                               focus_subject='', focus_bearing=float('nan'))
    m.restore(snap, 1.0)
    s = m.state
    assert s.arousal == -1.0 and s.mood == 'down'
    assert not s.locked and s.lock_source == '' and s.focus_subject == ''
    with pytest.raises(ValueError):
        m.restore(snap, float('nan'))
