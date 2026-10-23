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

"""Tests for the dome motion model, named for its rules (phase 2 plan 4.4), on r2_mocks' plant."""

import math

import pytest

from fixtures import PlantDouble, simulate  # noqa: I100
from r2_expression import motion
from r2_expression.motion import (Coast, CoastMemory, glance, Heading, Limits, look_at,
                                  MAX_TURN_RAD, Profile, RateLimiter, rest, RestHold, turn,
                                  wrap)
from r2_mocks.dome_plant import (DEAD_BAND_NEGATIVE_RAD_S, DEAD_BAND_POSITIVE_RAD_S,
                                 GAP_HIGH_RAD, GAP_LOW_RAD)

PROFILES = {'snappy': Profile(v_max=2.0, accel=4.0, k=3.0, settle_band_rad=0.05),
            'smooth': Profile(v_max=1.0, accel=1.5, k=1.5, settle_band_rad=0.05),
            'lazy': Profile(v_max=0.6, accel=0.8, k=1.0, settle_band_rad=0.09)}
LIMITS = Limits()
READING_ERROR_RAD = math.radians(4.5)
GRID = [sign * math.radians(deg) for deg in range(5, 181, 5) for sign in (1, -1)]
LOGS = []                        # every simulated command log, for the rule 1 and 2 sweeps


def run(move, plant, **kwargs):
    reason, log = simulate(move, plant, **kwargs)
    LOGS.append((move.profile, log))
    return reason, log


def landed(plant, target):
    return abs(wrap(plant.position - target))


# --- angles --------------------------------------------------------------------------------

@pytest.mark.parametrize('angle, expected', [(0.0, 0.0), (math.pi, math.pi), (-math.pi, math.pi),
                                             (3 * math.pi, math.pi), (2 * math.pi, 0.0),
                                             (-0.5, -0.5), (7.0, 7.0 - 2 * math.pi)])
def test_wrap_maps_every_angle_into_minus_pi_exclusive_to_pi_inclusive(angle, expected):
    assert wrap(angle) == pytest.approx(expected)
    assert -math.pi < wrap(angle) <= math.pi


def test_heading_a_bearing_to_the_left_is_a_negative_joint_angle():
    heading = Heading()                                   # sign -1, offset 0 (plan 2.5)
    assert heading.joint(0.3) == pytest.approx(-0.3)
    assert heading.joint(0.0) == 0.0
    assert heading.bearing(heading.joint(1.2)) == pytest.approx(1.2)
    assert heading.delta(-1.57) == pytest.approx(1.57)
    offset = Heading(sign=1, offset_rad=0.5)
    assert offset.joint(3.0) == pytest.approx(wrap(3.5))
    assert offset.bearing(offset.joint(-2.0)) == pytest.approx(-2.0)


def test_coast_is_c1_v_minus_c2_and_never_negative():
    coast = Coast()
    assert coast.distance(2.0) == pytest.approx(0.67)
    assert coast.distance(-1.0) == pytest.approx(0.30)
    assert coast.distance(0.1) == 0.0


def test_profile_is_capped_to_the_converters_limits():
    capped = PROFILES['snappy'].capped(1.5, 2.0)
    assert (capped.v_max, capped.accel, capped.k) == (1.5, 2.0, 3.0)


# --- look_at: the grid ---------------------------------------------------------------------

@pytest.mark.parametrize('name', sorted(PROFILES))
@pytest.mark.parametrize('plant_kind', ['ideal', 'asymmetric'])
def test_look_at_reaches_within_the_band_from_every_5_degree_error_both_ways_across_the_wrap(
        name, plant_kind):
    profile = PROFILES[name]
    for error in GRID:
        for start in (1.0, math.pi - error / 2.0, -error / 2.0):   # none, pi, 0 crossed
            target = wrap(start + error)
            plant = PlantDouble(start, ideal=plant_kind == 'ideal')
            reason, log = run(look_at(target, profile, LIMITS, 0.0), plant)
            case = (name, plant_kind, math.degrees(error), start)
            assert reason == 'reached', case
            assert landed(plant, target) <= profile.settle_band_rad, case
            assert log[-1][1] == 0.0, case                 # every move ends with a 0
            assert log[-1][0] <= LIMITS.max_move_s, case


@pytest.mark.parametrize('sign', [1, -1])
def test_the_floor_min_speed_is_above_the_dead_band_of_each_direction(sign):
    # The droid's velocity_map: 0 % under 0.44 rad/s positive, under 0.567 negative. A
    # floor of 0.5 (the first default) stalled every leftward (negative) creep.
    assert LIMITS.min_speed > max(DEAD_BAND_POSITIVE_RAD_S, DEAD_BAND_NEGATIVE_RAD_S)
    for profile in PROFILES.values():
        plant = PlantDouble(0.0)
        reason, log = run(look_at(sign * 0.3, profile, LIMITS, 0.0), plant)
        assert reason == 'reached' and landed(plant, sign * 0.3) <= profile.settle_band_rad
    stalled = PlantDouble(0.0)
    reason, _ = simulate(look_at(-0.3, PROFILES['lazy'], Limits(min_speed=0.5), 0.0), stalled)
    assert reason == 'timeout' and stalled.position == 0.0


def in_gap(angle):
    """Return True near the firmware's gap at 0, where a reading is up to 4.5 degrees off."""
    angle %= 2 * math.pi
    return angle < GAP_HIGH_RAD + 0.02 or angle > GAP_LOW_RAD - 0.02


@pytest.mark.parametrize('name', sorted(PROFILES))
def test_look_at_across_0_with_the_firmwares_9_degree_step_lands_within_its_reading_error(
        name):
    # Plan risk "the 0 crossing reads 9 degrees long": a reading near 0 is up to 4.5 degrees
    # off, which no loop can see, and it can skew the coast the move learns. The retry
    # corrects most of it; what is left is within the band plus that reading error. The
    # retry creeps at min_speed (0.6, above the negative dead band), which coasts about 9
    # degrees on the real dome, so a small move whose first stop the step skewed may settle
    # just outside the band: at least 9 in 10 are reached.
    profile, reached, cases = PROFILES[name], 0, 0
    for error in GRID:
        for distance in (0.2, 0.3):
            target = math.copysign(distance, error)
            if in_gap(target - error):
                continue                                    # a start the reading misplaces
            plant = PlantDouble(target - error, quirks=True)
            reason, log = run(look_at(target, profile, LIMITS, 0.0), plant)
            case = (name, math.degrees(error), distance)
            assert reason in ('reached', 'settled_short'), case
            assert landed(plant, target) <= profile.settle_band_rad + READING_ERROR_RAD, case
            assert log[-1][1] == 0.0, case
            reached += reason == 'reached'
            cases += 1
    assert reached >= 0.9 * cases, (reached, cases)


def test_look_at_turns_the_short_way():
    plant = PlantDouble(3.0)
    run(look_at(-3.0, PROFILES['smooth'], LIMITS, 0.0), plant)
    assert plant.position == pytest.approx(2 * math.pi - 3.0, abs=0.05)   # through pi


def test_look_at_where_the_dome_already_is_ends_reached_with_a_zero():
    move = look_at(0.5, PROFILES['smooth'], LIMITS, 0.0)
    assert move.step(0.51, 0.0) == (0.0, True, 'reached')
    assert move.step(0.51, 0.05) == (0.0, True, 'reached')        # a finished move stays so


# --- the stop and the retry ----------------------------------------------------------------

def test_early_stop_by_the_coast_sends_the_0_before_the_target_and_coasts_in():
    target = 1.5
    move = look_at(target, PROFILES['snappy'], LIMITS, 0.0)
    reason, log = run(move, PlantDouble(0.0))
    first_zero = next(i for i, (_, v, _) in enumerate(log) if v == 0.0)
    assert reason == 'reached'
    assert target - log[first_zero][2] > 0.2              # the 0 went out well short of it
    assert all(v == 0.0 for _, v, _ in log[first_zero:])   # and the coast took it in
    assert move.coast_scale == pytest.approx(1.0, abs=0.35)


def test_retry_approaches_again_at_min_speed_when_settled_outside_the_band():
    profile = PROFILES['snappy']                          # the ideal plant never coasts
    move = look_at(2.0, profile, LIMITS, 0.0)
    reason, log = run(move, PlantDouble(0.0, ideal=True))
    first_zero = next(t for t, v, _ in log if v == 0.0)
    retry = [v for t, v, _ in log if t > first_zero and v != 0.0]
    assert reason == 'reached'
    assert retry and all(v == LIMITS.min_speed for v in retry)
    assert move.coast_scale < 0.2                         # it learnt there is no coast


def test_settle_waits_for_the_dome_to_stop_but_at_most_twice_settle_s():
    move = look_at(1.0, PROFILES['smooth'], LIMITS, 0.0)
    t, position, stopped_at, again = 0.0, 0.0, None, None
    while again is None and t < 8.0:
        velocity, _, _ = move.step(position, t)
        if stopped_at is None and velocity == 0.0:
            stopped_at = t
        elif stopped_at is not None and velocity != 0.0:
            again = t
        position += (velocity if stopped_at is None else 0.2) * 0.05    # pushed on
        t = round(t + 0.05, 6)
    assert again == pytest.approx(stopped_at + 2 * LIMITS.settle_s, abs=0.06)


def test_one_retry_only_then_settled_short():
    limits = Limits(retries=0)
    reason, _ = run(look_at(2.0, PROFILES['snappy'], limits, 0.0), PlantDouble(0.0, ideal=True))
    assert reason == 'settled_short'


def test_timeout_at_max_move_s_with_a_0():
    move = look_at(2.0, PROFILES['smooth'], LIMITS, 10.0)
    assert move.deadline == 18.0
    t = 10.0
    while True:                                           # the dome never moves
        velocity, done, reason = move.step(0.0, t)
        if done:
            break
        t += 0.05
    assert (velocity, reason) == (0.0, 'timeout')
    assert t == pytest.approx(18.0)


def test_no_new_sample_for_feedback_timeout_s_ends_the_move_with_a_0():
    move = look_at(2.0, PROFILES['smooth'], LIMITS, 0.0)
    assert move.step(0.0, 0.0, 0.0)[0] == LIMITS.min_speed
    assert move.step(0.0, 0.45, 0.0)[1] is False
    assert move.step(0.0, 0.55, 0.0) == (0.0, True, 'no feedback')


def test_stop_sends_0_and_ends_stopped():
    move = look_at(2.0, PROFILES['smooth'], LIMITS, 0.0)
    move.step(0.0, 0.0)
    assert move.stop() == (0.0, True, 'stopped')
    assert move.step(0.1, 0.05) == (0.0, True, 'stopped') and move.velocity == 0.0
    assert move.stop() == (0.0, True, 'stopped')


# --- glance, turn, rest --------------------------------------------------------------------

@pytest.mark.parametrize('ideal', [True, False])
@pytest.mark.parametrize('name', sorted(PROFILES))
def test_glance_goes_out_dwells_and_returns_within_the_band(ideal, name):
    profile = PROFILES[name]
    for start, amplitude in ((0.4, 0.6), (-3.0, -0.9), (3.0, 0.5)):
        plant = PlantDouble(start, ideal=ideal)
        move = glance(amplitude, 1.0, profile, LIMITS, 0.0)
        reason, log = run(move, plant)
        assert reason == 'reached', (name, start)
        assert landed(plant, start) <= profile.settle_band_rad, (name, start)
        zeros = [t for t, v, _ in log if v == 0.0]
        assert zeros[-1] - zeros[0] >= 1.0               # it dwelt at least that long


def test_glance_budget_is_max_move_s_plus_its_dwell():
    move = glance(0.5, 2.0, PROFILES['smooth'], LIMITS, 0.0)
    assert move.deadline == pytest.approx(10.0) and move.verb == 'glance'


@pytest.mark.parametrize('ideal', [True, False])
def test_turn_beyond_pi_goes_the_whole_way_round(ideal):
    plant = PlantDouble(0.0, ideal=ideal)
    reason, _ = run(turn(4.0, PROFILES['snappy'], LIMITS, 0.0), plant)
    assert reason == 'reached'
    assert plant.position == pytest.approx(4.0, abs=0.05)     # not -2.28 the short way


def test_turn_is_capped_at_a_full_turn():
    plant = PlantDouble(0.0, ideal=True)
    run(turn(-9.0, PROFILES['snappy'], LIMITS, 0.0), plant)
    assert plant.position == pytest.approx(-MAX_TURN_RAD, abs=0.05)


def test_rest_is_a_look_at_named_rest():
    plant = PlantDouble(0.0)
    move = rest(-0.35, PROFILES['lazy'], LIMITS, 0.0)
    reason, _ = run(move, plant)
    assert (move.verb, reason) == ('rest', 'reached')
    assert landed(plant, -0.35) <= PROFILES['lazy'].settle_band_rad


def test_rest_recheck_cadence_is_at_most_once_per_rest_recheck_s():
    hold = RestHold(30.0)
    assert not hold.poll(1.0, 0.05, 0.0)                  # not armed
    hold.arm(100.0)
    assert not hold.poll(1.0, 0.05, 129.9)                # not yet due
    assert hold.poll(1.0, 0.05, 130.0)                    # due, drifted: correct it
    assert not hold.poll(1.0, 0.05, 130.1)                # next check at 160
    assert not hold.poll(0.01, 0.05, 160.0)               # due, within the band
    assert hold.next_check == 190.0


# --- replacement ---------------------------------------------------------------------------

def test_a_replacement_takes_over_the_command_without_a_0():
    profile = PROFILES['smooth']
    first = look_at(2.5, profile, LIMITS, 0.0)
    t, velocity = 0.0, 0.0
    while t < 0.6:
        velocity, _, _ = first.step(0.4 * t, t)
        t += 0.05
    second = look_at(0.6, profile, LIMITS, t, follows=first)      # nearer: slow down
    out, _, _ = second.step(0.4 * t, t)
    assert LIMITS.min_speed <= out < velocity
    assert velocity - out <= profile.accel * 0.05 + 1e-9
    back = look_at(-2.0, profile, LIMITS, t + 0.05, follows=second)
    assert back.step(0.4 * t, t + 0.05)[0] == 0.0          # a reversal stops first


def test_rate_limiter_leaves_0_at_the_floor_and_reverses_through_0():
    limiter = RateLimiter(0.5)
    assert limiter.step(2.0, 2.0, 1.0, 0.0) == 0.5         # 0 -> the floor at once
    assert limiter.step(2.0, 2.0, 1.0, 0.25) == 0.75
    assert limiter.step(-1.0, 2.0, 1.0, 0.5) == 0.5
    assert limiter.step(-1.0, 2.0, 1.0, 0.75) == 0.0       # never in (0, min_speed)
    assert limiter.step(-1.0, 2.0, 1.0, 1.0) == -0.5
    assert limiter.step(0.0, 2.0, 1.0, 1.25) == 0.0        # a stop is never delayed


def test_rule1_a_slower_move_taking_over_a_faster_one_is_at_its_own_v_max_at_once():
    first = look_at(3.0, PROFILES['snappy'], LIMITS, 0.0)
    plant = PlantDouble(0.0, ideal=True)
    t = 0.0
    while t < 1.0:
        plant.step(first.step(plant.reading(), t)[0], 0.05)
        t = round(t + 0.05, 6)
    assert first.velocity == 2.0
    second = look_at(3.0, PROFILES['lazy'], LIMITS, t, follows=first)
    reason, log = run(second, plant, t0=t)
    assert reason == 'reached' and max(abs(v) for _, v, _ in log) <= 0.6


def test_a_replacement_behind_the_command_stops_then_approaches_fully_keeping_its_retry():
    first = look_at(2.0, PROFILES['snappy'], LIMITS, 0.0)
    plant = PlantDouble(0.0, ideal=True)
    t = 0.0
    while t < 1.0:
        plant.step(first.step(plant.reading(), t)[0], 0.05)
        t = round(t + 0.05, 6)
    second = look_at(-1.0, PROFILES['snappy'], LIMITS, t, follows=first)
    reason, log = run(second, plant, t0=t)
    assert reason == 'reached' and landed(plant, -1.0) <= 0.05
    assert min(v for _, v, _ in log) == -2.0                # a full approach, not a creep
    assert log[-1][0] - t < 5.0


def test_a_reversal_inside_a_move_settles_and_tries_again():
    # A move that sees itself overshoot sends 0 at once (the error is behind it).
    move = look_at(0.3, PROFILES['smooth'], LIMITS, 0.0)
    move.step(0.0, 0.0)
    move.step(0.05, 0.05)
    assert move.step(0.5, 0.10)[0] == 0.0                  # a push past the target


# --- the sweeps over every simulated command (keep last) -------------------------------------

def test_rule1_every_command_is_0_or_between_min_speed_and_v_max():
    assert len(LOGS) > 500
    for profile, log in LOGS:
        for _, v, _ in log:
            assert v == 0.0 or LIMITS.min_speed <= abs(v) <= profile.v_max + 1e-9, (profile, v)


def test_rule2_acceleration_between_non_zero_commands_never_above_the_profile():
    for profile, log in LOGS:
        for (t0, v0, _), (t1, v1, _) in zip(log, log[1:]):
            if v0 != 0.0 and v1 != 0.0:
                assert abs(v1 - v0) <= profile.accel * (t1 - t0) + 1e-9, (profile, v0, v1)


def test_module_has_no_ros_imports():
    assert not any(name.startswith(('rclpy', 'astromech')) for name in vars(motion))


# --- rule 5 across moves: the coast memory ------------------------------------------------

def test_rule5_the_coast_memory_carries_a_learned_scale_and_fades_it_toward_1():
    plant = PlantDouble(0.0, ideal=True)                  # Gazebo: no coast
    first = look_at(-1.5, PROFILES['smooth'], LIMITS, 0.0)
    run(first, plant)
    assert first.coast_learned and first.coast_scale < 0.1
    memory = CoastMemory(half_life_s=600.0)
    assert memory.recall(0.0) == 1.0                      # nothing learned yet
    memory.learn(first, 10.0)
    assert memory.recall(10.0) == pytest.approx(first.coast_scale)
    assert memory.recall(610.0) == pytest.approx(1.0 - (1.0 - first.coast_scale) / 2.0)
    assert memory.recall(1e6) == pytest.approx(1.0)
    assert memory.recall(0.0) == pytest.approx(first.coast_scale)   # no fade backwards


def test_rule5_a_move_that_never_stopped_teaches_the_memory_nothing():
    memory = CoastMemory()
    taught = look_at(-1.5, PROFILES['smooth'], LIMITS, 0.0)
    run(taught, PlantDouble(0.0, ideal=True))
    memory.learn(taught, 20.0)
    untaught = look_at(1.0, PROFILES['smooth'], LIMITS, 20.0, coast_scale=memory.recall(20.0))
    untaught.step(0.0, 20.0)
    untaught.stop()
    assert not untaught.coast_learned
    memory.learn(untaught, 620.0)                         # the fade is not restarted
    assert memory.recall(620.0) == pytest.approx(1.0 - (1.0 - taught.coast_scale) / 2.0)


def glance_until_learned():
    """Return a glance stepped on the ideal plant until a stop taught it, and the time."""
    move = glance(1.0, 2.0, PROFILES['smooth'], LIMITS, 0.0)
    plant = PlantDouble(0.0, ideal=True)
    t = 0.0
    while not move.coast_learned and t < 10.0:            # out, stopped, settled: learned
        move.step(plant.reading(), t)
        plant.step(move.velocity, 0.05)
        t += 0.05
    return move, t


def test_rule5_a_stopped_move_teaches_the_memory_nothing_even_if_a_stop_taught_it():
    memory = CoastMemory()
    move, t = glance_until_learned()
    assert move.coast_learned and not move.done           # now in its dwell
    move.stop()
    memory.learn(move, t)
    assert memory.recall(t) == 1.0


def test_rule5_a_move_cut_short_without_a_stop_teaches_the_memory_nothing():
    memory = CoastMemory()
    move, t = glance_until_learned()
    assert move.coast_learned and (move.done, move.reason) == (False, '')
    memory.learn(move, t)                                 # interrupted: never finished
    assert memory.recall(t) == 1.0


def test_rule5_a_carried_scale_spares_the_ideal_plant_its_second_approach():
    def approaches(scale):
        move = look_at(-1.5, PROFILES['smooth'], LIMITS, 0.0, coast_scale=scale)
        _, log = run(move, PlantDouble(0.0, ideal=True))
        return sum(1 for (_, a, _), (_, b, _) in zip([(0, 0.0, 0)] + log, log) if b and not a)
    assert approaches(1.0) == 2                           # stops short by the fit, creeps
    assert approaches(0.0) == 1
