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
``DomePlant`` against the numbers of plan 2.1.

Dead band 0.44 rad/s positive and 0.567 rad/s negative (the velocity map's two floors).
By default the droid's own lag and coast (step 8b, 2026-10-23): 90 % of the speed at 0.6 s,
coast ``0.52 s * v`` with no offset, and a command of 0.6 rad/s (the floor) achieves 0.35. The
``findings_2026_10`` preset keeps the calibration findings' 1.5 s and ``0.37 s * v - 4 deg``.
The 355-to-4 deg step at each crossing of 0.
"""

import math

import pytest
from r2_mocks.dome_plant import DEAD_BAND_NEGATIVE_RAD_S
from r2_mocks.dome_plant import DEAD_BAND_POSITIVE_RAD_S
from r2_mocks.dome_plant import DomePlant
from r2_mocks.dome_plant import wrap

DT = 1.0 / 30.0
TWO_PI = 2.0 * math.pi


def run(plant, command, seconds, dt=DT):
    for _ in range(round(seconds / dt)):
        plant.step(command, dt)


PRESET_LAGS_COASTS = {'measured_2026_10': (0.6, 0.52, 0.0),
                      'findings_2026_10': (1.5, 0.37, math.radians(4.0))}


def settle(plant, command=0.0, limit_s=30.0):
    run(plant, command, limit_s)
    assert plant.velocity_rad_s == 0.0


@pytest.mark.parametrize('command', [0.0, 0.1, 0.43, -0.43, -0.2, -0.5, -0.565])
def test_dead_band_below_0_44_positive_and_0_567_negative_the_dome_does_not_move(command):
    plant = DomePlant(initial_rad=1.0)
    run(plant, command, 5.0)
    assert plant.position_rad == 1.0 and plant.velocity_rad_s == 0.0


@pytest.mark.parametrize('command', [0.44, 0.5, -0.567, -0.6])
def test_dead_band_at_the_floor_of_each_direction_the_dome_moves(command):
    plant = DomePlant()
    run(plant, command, 5.0)
    assert abs(plant.velocity_rad_s) > 0.2 and abs(plant.position_rad) > 1.0
    assert math.copysign(1.0, plant.position_rad) == math.copysign(1.0, command)


def test_dead_band_floors_are_the_velocity_maps():
    # ros2_control.xacro: round(p0 + k * |v|) percent, 0 below min_percent.
    for floor, (p0, k, min_percent) in ((DEAD_BAND_POSITIVE_RAD_S, (13.9, 12.75, 20)),
                                        (DEAD_BAND_NEGATIVE_RAD_S, (14.5, 12.36, 22))):
        assert floor == pytest.approx((min_percent - 0.5 - p0) / k, abs=1e-3)


@pytest.mark.parametrize('preset', PRESET_LAGS_COASTS)
@pytest.mark.parametrize('dt', [DT, 0.01, 0.1])
def test_lag_reaches_90_percent_of_the_commanded_speed_at_the_presets_time(preset, dt):
    lag_90_s = PRESET_LAGS_COASTS[preset][0]
    plant = DomePlant(preset=preset)
    run(plant, 2.0, lag_90_s, dt)
    assert plant.velocity_rad_s == pytest.approx(0.9 * 2.0, rel=1e-6)


def test_lag_default_is_the_measured_0_6_s():
    assert DomePlant().lag_90_s == 0.6 and DomePlant().preset == 'measured_2026_10'


def test_lag_is_first_order_the_position_integrates_the_velocity():
    plant = DomePlant()
    fine = DomePlant()
    run(plant, 1.0, 3.0, 0.1)
    run(fine, 1.0, 3.0, 0.001)
    assert plant.position_rad == pytest.approx(fine.position_rad, rel=1e-6)
    assert plant.velocity_rad_s == pytest.approx(fine.velocity_rad_s, rel=1e-6)


@pytest.mark.parametrize('preset', PRESET_LAGS_COASTS)
@pytest.mark.parametrize('speed', [1.0, 2.0])
def test_coast_after_a_stop_is_the_presets_fit_at_v(preset, speed):
    _, coast_s, offset = PRESET_LAGS_COASTS[preset]
    plant = DomePlant(preset=preset, speed_gain=(1.0, 1.0))
    run(plant, speed, 10.0)                       # settled at the commanded speed
    at_stop, v = plant.position_rad, plant.velocity_rad_s
    assert v == pytest.approx(speed, abs=1e-3)
    settle(plant, 0.0)
    assert plant.position_rad - at_stop == pytest.approx(coast_s * v - offset, abs=1e-9)


def test_floor_a_command_of_0_6_achieves_0_58_of_it_and_coasts_0_12_rad():
    # The one point measured near the floor (step 8b): 0.35 rad/s achieved, coast 0.12 rad.
    plant = DomePlant()
    run(plant, 0.6, 10.0)
    at_stop = plant.position_rad
    assert plant.velocity_rad_s == pytest.approx(0.348, abs=1e-3)
    settle(plant, 0.0)
    assert plant.position_rad - at_stop == pytest.approx(0.12 * 0.348 / 0.35, abs=1e-9)
    assert plant.position_rad - at_stop == pytest.approx(0.122, abs=0.005)   # not the line's 0.18
    findings = DomePlant(preset='findings_2026_10')
    run(findings, 0.6, 10.0)
    assert findings.velocity_rad_s == pytest.approx(0.6, abs=1e-3)


def test_coast_default_is_0_52_v_with_no_offset():
    plant = DomePlant()
    run(plant, 1.5, 10.0)
    at_stop, v = plant.position_rad, plant.velocity_rad_s
    settle(plant, 0.0)
    assert plant.position_rad - at_stop == pytest.approx(0.52 * v, abs=1e-9)


def test_coast_a_sub_dead_band_command_stops_the_motor_like_a_zero():
    plant = DomePlant()
    run(plant, 2.0, 10.0)
    at_stop, v = plant.position_rad, plant.velocity_rad_s
    settle(plant, 0.3)
    assert plant.position_rad - at_stop == pytest.approx(0.52 * v, abs=1e-9)


def test_coast_never_negative_a_slow_dome_stops_at_once():
    plant = DomePlant(preset='findings_2026_10')
    plant.step(0.45, 0.01)                        # about 0.007 rad/s: the 4 deg offset wins
    at_stop = plant.position_rad
    plant.step(0.0, DT)
    assert plant.velocity_rad_s == 0.0 and plant.position_rad == at_stop


def test_coast_does_not_depend_on_the_step_size():
    ends = []
    for dt in (0.1, DT, 0.001):
        plant = DomePlant()
        run(plant, 1.5, 6.0, dt)
        at_stop = plant.position_rad
        settle(plant, 0.0)
        ends.append(plant.position_rad - at_stop)
    assert ends[0] == pytest.approx(ends[1], abs=1e-9) == pytest.approx(ends[2], abs=1e-9)


def test_a_new_command_ends_the_coast():
    plant = DomePlant()
    run(plant, 2.0, 6.0)
    plant.step(0.0, DT)
    assert plant.coasting
    plant.step(1.0, DT)
    assert not plant.coasting


def test_negative_commands_above_both_floors_mirror_positive_ones():
    up, down = DomePlant(speed_gain=(1.0, 1.0)), DomePlant(speed_gain=(1.0, 1.0))
    for plant, sign in ((up, 1.0), (down, -1.0)):
        run(plant, sign * 1.0, 4.0)
        run(plant, 0.0, 4.0)
    assert down.position_rad == pytest.approx(-up.position_rad, abs=1e-12)
    assert down.velocity_rad_s == up.velocity_rad_s == 0.0


def test_a_non_finite_command_counts_as_zero():
    plant = DomePlant()
    run(plant, 2.0, 3.0)
    run(plant, float('nan'), 6.0)
    assert plant.velocity_rad_s == 0.0 and math.isfinite(plant.position_rad)


def test_wrap_the_reported_position_is_in_0_to_2pi():
    assert wrap(-0.1) == pytest.approx(TWO_PI - 0.1)
    assert wrap(TWO_PI + 0.5) == pytest.approx(0.5)
    assert wrap(-1e-18) == 0.0
    plant = DomePlant(firmware_quirks=False, initial_rad=-3.0)
    for _ in range(300):
        assert 0.0 <= plant.step(2.0, 0.1) < TWO_PI
    assert plant.position_rad > TWO_PI


def test_wrap_without_quirks_reports_the_true_angle():
    plant = DomePlant(firmware_quirks=False, initial_rad=TWO_PI - 0.01)
    assert plant.step(2.0, 0.0) == pytest.approx(TWO_PI - 0.01)
    plant.step(2.0, 0.01)
    assert plant.reported_rad == plant.wrapped_rad


def jumps(plant, speed, steps=100):
    """Return the successive changes of the reported angle (shortest way round)."""
    run(plant, speed, 3.0)                         # up to speed, not yet round to 0 either way
    out, last = [], plant.reported_rad
    for _ in range(steps):
        now = plant.step(speed, DT)
        out.append(abs((now - last + math.pi) % TWO_PI - math.pi))
        last = now
    return out


@pytest.mark.parametrize('speed', [2.0, -2.0])
def test_crossing_artefact_each_crossing_of_0_reads_about_9_degrees_long(speed):
    steps = jumps(DomePlant(), speed)
    normal = 2.0 * DT                              # a step of the dome at 2 rad/s: about 3.8 deg
    assert sum(1 for s in steps if s > 1.5 * normal) == 1     # one crossing in this run
    assert max(steps) == pytest.approx(math.radians(9.0), abs=normal)


def test_crossing_artefact_never_reports_inside_355_to_4_degrees():
    plant = DomePlant(initial_rad=TWO_PI - 0.5)
    seen = set()
    for _ in range(600):
        seen.add(plant.step(1.0, 0.002))
    for angle in seen:
        assert not (math.radians(355.0) < angle < TWO_PI or 0.0 <= angle < math.radians(4.0))
    assert any(a == pytest.approx(math.radians(355.0)) for a in seen)
    assert any(a == pytest.approx(math.radians(4.0)) for a in seen)


def test_crossing_artefact_is_off_when_firmware_quirks_is_false():
    plant = DomePlant(firmware_quirks=False, initial_rad=TWO_PI - 0.5)
    for _ in range(600):
        assert plant.step(1.0, 0.01) == plant.wrapped_rad
    assert plant.position_rad > TWO_PI              # it did cross 0


def test_crossing_artefact_error_is_bounded_by_4_5_degrees_and_does_not_accumulate():
    plant = DomePlant()
    run(plant, 2.0, 60.0)
    error = (plant.reported_rad - plant.wrapped_rad + math.pi) % TWO_PI - math.pi
    assert abs(error) <= math.radians(4.5) + 1e-9


def test_ideal_is_a_pure_integrator_exact_at_any_step():
    plant = DomePlant(ideal=True, initial_rad=0.25)
    position = 0.25
    for command in (0.1, 0.3, 1.0, -2.5, 0.0, 6.0):
        for _ in range(7):
            position += command * 0.037
            reported = plant.step(command, 0.037)
            assert plant.position_rad == pytest.approx(position, abs=1e-12)
            assert reported == pytest.approx(wrap(position), abs=1e-12)
            assert plant.velocity_rad_s == command


def test_ideal_stops_at_once_with_no_dead_band_lag_or_coast():
    plant = DomePlant(ideal=True)
    plant.step(0.1, 1.0)                           # under the dead band, still moves
    assert plant.position_rad == pytest.approx(0.1)
    run(plant, 2.0, 1.0)
    at_stop = plant.position_rad
    plant.step(0.0, DT)
    assert plant.position_rad == at_stop and plant.velocity_rad_s == 0.0


def test_ideal_has_no_crossing_artefact():
    plant = DomePlant(ideal=True, firmware_quirks=True, initial_rad=TWO_PI - 0.01)
    assert plant.step(1.0, 0.02) == pytest.approx(0.01)


def test_a_bad_dt_changes_nothing():
    plant = DomePlant(initial_rad=1.0)
    for dt in (0.0, -1.0, float('nan'), float('inf')):
        plant.step(2.0, dt)
    assert plant.position_rad == 1.0 and plant.velocity_rad_s == 0.0


def sampled(plant, speed, seconds, hz=10):
    """Return the readings at ``hz`` while running at ``speed`` (stepped at 60 Hz)."""
    out, per = [], round(60 / hz)
    for n in range(round(seconds * 60)):
        plant.step(speed, 1.0 / 60.0)
        if n % per == 0:
            out.append(plant.reported_rad)
    return out


def deltas(readings):
    """Return the successive changes of the readings (shortest way round)."""
    return [(b - a + math.pi) % TWO_PI - math.pi for a, b in zip(readings, readings[1:])]


def test_misread_is_off_by_default():
    plant = DomePlant(firmware_quirks=False, initial_rad=math.radians(300.0))
    readings = sampled(plant, 1.0, 5.0)
    assert all(abs(d) < 0.2 for d in deltas(readings))


def test_misread_clockwise_reads_279_for_one_sample_at_each_pass_of_the_sector():
    plant = DomePlant(firmware_quirks=False, initial_rad=math.radians(300.0),
                      misread_at=(313.0, 331.0, 279.0))
    readings = sampled(plant, 1.0, 20.0)                 # 4 passes at about 1 rad/s
    bogus = [i for i, r in enumerate(readings) if r == pytest.approx(math.radians(279.0))]
    assert len(bogus) == 4 and all(b - a > 20 for a, b in zip(bogus, bogus[1:]))
    assert deltas(readings)[bogus[0]] > 0.5              # and it jumps on, as measured
    assert deltas(readings)[bogus[0] - 1] < -0.4         # after dropping 34 degrees
    assert readings[bogus[0] - 1] < math.radians(331.0)  # the sample before is true


def test_misread_anticlockwise_and_at_rest_never_misread():
    plant = DomePlant(firmware_quirks=False, initial_rad=math.radians(340.0),
                      misread_at=(313.0, 331.0, 279.0))
    readings = sampled(plant, -1.0, 15.0)
    assert all(abs(d) < 0.2 for d in deltas(readings))
    resting = DomePlant(initial_rad=math.radians(320.0), misread_at=(313.0, 331.0, 279.0))
    assert set(sampled(resting, 0.0, 2.0)) == {math.radians(320.0)}


def test_misread_is_ignored_by_the_ideal_plant():
    plant = DomePlant(ideal=True, initial_rad=math.radians(300.0),
                      misread_at=(313.0, 331.0, 279.0))
    readings = sampled(plant, 1.0, 3.0)
    assert all(abs(d) < 0.2 for d in deltas(readings))


def test_a_command_of_1_runs_1_29_clockwise_and_0_76_anticlockwise_by_default():
    up, down = DomePlant(), DomePlant()
    run(up, 1.0, 10.0)
    run(down, -1.0, 10.0)
    assert up.velocity_rad_s == pytest.approx(1.29, abs=1e-3)
    assert down.velocity_rad_s == pytest.approx(-0.76, abs=1e-3)
    fast = DomePlant()
    run(fast, 2.0, 10.0)
    assert fast.velocity_rad_s == pytest.approx(2.0, abs=1e-3)      # above the band: the command
    flat = DomePlant(speed_gain=(1.0, 1.0))
    run(flat, 1.0, 10.0)
    assert flat.velocity_rad_s == pytest.approx(1.0, abs=1e-3)


def test_coast_below_the_line_near_the_floor_is_continuous():
    from r2_mocks.dome_plant import coast_distance
    assert coast_distance(0.35) == pytest.approx(0.12)
    assert coast_distance(0.76) == pytest.approx(0.52 * 0.76)
    speeds = [0.01 * n for n in range(0, 200)]
    values = [coast_distance(v) for v in speeds]
    assert all(b >= a and b - a < 0.02 for a, b in zip(values, values[1:]))
