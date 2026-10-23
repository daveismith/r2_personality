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

Dead band 0.44 rad/s positive and 0.567 rad/s negative (the velocity map's two floors),
90 % of the commanded speed at 1.5 s, coast ``0.37 s * v - 4 deg``, the 355-to-4 deg step
at each crossing of 0.
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
    assert abs(plant.velocity_rad_s) > 0.4 and abs(plant.position_rad) > 1.0
    assert math.copysign(1.0, plant.position_rad) == math.copysign(1.0, command)


def test_dead_band_floors_are_the_velocity_maps():
    # ros2_control.xacro: round(p0 + k * |v|) percent, 0 below min_percent.
    for floor, (p0, k, min_percent) in ((DEAD_BAND_POSITIVE_RAD_S, (13.9, 12.75, 20)),
                                        (DEAD_BAND_NEGATIVE_RAD_S, (14.5, 12.36, 22))):
        assert floor == pytest.approx((min_percent - 0.5 - p0) / k, abs=1e-3)


@pytest.mark.parametrize('dt', [DT, 0.01, 0.1])
def test_lag_reaches_90_percent_of_the_commanded_speed_at_1_5_s(dt):
    plant = DomePlant()
    run(plant, 2.0, 1.5, dt)
    assert plant.velocity_rad_s == pytest.approx(0.9 * 2.0, rel=1e-6)


def test_lag_is_first_order_the_position_integrates_the_velocity():
    plant = DomePlant()
    fine = DomePlant()
    run(plant, 1.0, 3.0, 0.1)
    run(fine, 1.0, 3.0, 0.001)
    assert plant.position_rad == pytest.approx(fine.position_rad, rel=1e-6)
    assert plant.velocity_rad_s == pytest.approx(fine.velocity_rad_s, rel=1e-6)


@pytest.mark.parametrize('speed', [0.6, 1.0, 2.0])
def test_coast_after_a_stop_is_0_37_v_minus_4_degrees_at_v(speed):
    plant = DomePlant()
    run(plant, speed, 10.0)                       # settled at the commanded speed
    at_stop, v = plant.position_rad, plant.velocity_rad_s
    assert v == pytest.approx(speed, abs=1e-3)
    settle(plant, 0.0)
    assert plant.position_rad - at_stop == pytest.approx(0.37 * v - math.radians(4.0), abs=1e-9)


def test_coast_a_sub_dead_band_command_stops_the_motor_like_a_zero():
    plant = DomePlant()
    run(plant, 2.0, 10.0)
    at_stop, v = plant.position_rad, plant.velocity_rad_s
    settle(plant, 0.3)
    assert plant.position_rad - at_stop == pytest.approx(0.37 * v - math.radians(4.0), abs=1e-9)


def test_coast_never_negative_a_slow_dome_stops_at_once():
    plant = DomePlant()
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
    up, down = DomePlant(), DomePlant()
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
