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
The dome's motion model: one move at a time, a velocity loop closed on the joint position.

Pure Python (no ROS imports). Every call takes ``now`` (seconds, any monotonic clock); the
node steps a move at 20 Hz with the latest joint position, which arrives at 10 Hz (plan
4.4). Angles are radians.

Angles and frames (plan 2.5). :func:`wrap` maps any angle to (-pi, pi]; every position
read is wrapped first. A *bearing* is the stimulus convention: +ve left, 0 forward. The
joint is positive clockwise seen from above, so ``joint = wrap(sign * bearing + offset)``
with :class:`Heading` ``sign`` -1 and ``offset_rad`` 0 on the droid. A move keeps its own
unwrapped position (the sum of the wrapped steps between samples), so a ``turn`` may
exceed pi and a crossing of 0 or pi costs nothing.

A :class:`Move` is a list of legs: ``look_at`` and ``rest`` go to a joint angle the short
way, ``turn`` goes by a delta (capped at :data:`MAX_TURN_RAD`), ``glance`` goes by a delta,
dwells and comes back to where it started. :meth:`Move.step` returns ``(velocity, done,
reason)``. Rules, each named by a test:

1. **The floor.** Every command is 0 or in [``min_speed``, ``v_max``] (plan rule 4): a
   command under the hardware's dead band would send 0 % and the dome would not move. The
   band is per direction (about 0.44 rad/s positive, 0.567 negative), so ``min_speed``
   (0.6) is above the higher of the two.
2. **Rate limit.** Between non-zero commands the change is at most ``accel * dt``; leaving
   0 jumps straight to ``min_speed``, and 0 itself (a stop) is never delayed.
3. **Approach.** ``v = clamp(k * |err|, min_speed, v_max)`` toward the target (rule 1 and
   2 applied after).
4. **Early stop by the coast.** The dome coasts ``max(0, c1 * v - c2)`` after a 0
   (measured on the droid, step 8b: 0.52 s and 0), so the move sends 0 once ``|err|`` is
   within that of the target (plus half of what the dome covers in one step: the stop
   falls on the nearest step, not the one after), or once it has passed the target.
5. **Settle.** After the 0 the move waits ``settle_s`` (longer, up to twice that, while the
   samples still show motion); then ``|err| <= settle_band`` is a leg ``reached``; else,
   with the retry budget (``retries``, 1) left, it approaches again at ``min_speed``; else
   the leg ends ``settled_short``. The settled position also shows how far the dome
   really coasted: the move scales its coast prediction by observed over predicted
   (``coast_scale``, 0 to 2) from then on, so the retry is not stopped by a coast an
   ideal plant (Gazebo) never has, and a dome coasting longer than the fit is caught.
   The arbiter carries the learned scale to the next move (:class:`CoastMemory`), so
   only the first move after a start (or a long rest) learns the plant from scratch.
6. **Bounded.** At ``max_move_s`` after it began (plus the dwell, for a glance) the move
   sends 0 and ends ``timeout``; with no new sample for ``feedback_timeout_s`` it sends 0
   and ends ``no feedback``; :meth:`Move.stop` sends 0 and ends ``stopped``. A finished
   move returns 0 for ever.
7. **The velocity estimate** (plan risk "10 Hz feedback"). Between samples the position is
   the last sample moved on by the estimated velocity (for at most
   :data:`EXTRAPOLATE_MAX_S`). The estimate is the speed measured between the last two
   samples, bounded by the fastest command of the current approach (the rate limiter's
   output). The plan names the rate limiter alone; on the real dome, whose step response
   takes 1.5 s to 90 %, the command runs well ahead of the dome and the coast it predicts
   would stop small moves before they start, so the measurement leads and the limiter
   bounds it. The bound also absorbs the firmware's 9-degree step at 0, which would
   otherwise read as a burst of speed. Before a fresh move has two samples the dome is
   taken to be at rest: on the real dome the command would predict a coast it has not
   built up yet and stop a short move before it starts. (A replacement before two samples
   takes its command as the estimate.)
8. **Replacement.** A move built with ``follows`` takes over its predecessor's command,
   rate limiter and samples, so the velocity simply changes (plan 4.3, one move at a time);
   a command above the new ``v_max`` comes down to it at once. If the new target is behind
   the command, the move sends 0, settles and makes a full approach, keeping its retry.
9. **Plausibility gate.** The droid's ring sensor misreads: turning clockwise at 1 rad/s the
   reading drops by about 0.6 rad for one sample as the dome passes 313 to 331 degrees, then
   jumps forward (measured 2026-10-23, ``docs/dome-motion-measurements.md``). A sample whose
   speed implied from the last accepted sample exceeds ``Limits.max_plausible_rad_s`` (4.0,
   twice the converter's cap) is held back and dropped: the previous estimate stands and
   rule 7's frozen-reading rule applies. If the next sample agrees with the held one within
   the plausible speed, both are accepted (a genuine fast push, or a wrap crossing with
   the firmware's 9-degree step, costs one sample); otherwise only the held one is
   dropped. A held sample still counts as feedback for ``feedback_timeout_s``.

:class:`RestHold` is the ``rest`` recheck cadence: at most one drift check per
``rest_recheck_s`` while the dome is meant to rest.
"""

from dataclasses import dataclass, replace
import math

TWO_PI = 2.0 * math.pi
MAX_TURN_RAD = TWO_PI          # a full turn; the slip ring allows more, the plan does not
EXTRAPOLATE_MAX_S = 0.35       # past this a sample is not moved on any further
STILL_RAD_S = 0.05             # measured speed under this: the dome has stopped
LEARN_MIN_RAD = 0.02           # a predicted coast shorter than this teaches nothing
MAX_COAST_SCALE = 2.0
COAST_HALF_LIFE_S = 600.0      # a carried coast scale halves its distance to 1.0 this often

REACHED = 'reached'
SETTLED_SHORT = 'settled_short'
TIMEOUT = 'timeout'
STOPPED = 'stopped'
NO_FEEDBACK = 'no feedback'


def wrap(angle):
    """Return ``angle`` in (-pi, pi]."""
    out = math.remainder(angle, TWO_PI)
    return math.pi if out <= -math.pi else out


@dataclass(frozen=True)
class Heading:
    """Bearing (+ve left, 0 forward) to joint angle: ``wrap(sign * bearing + offset_rad)``."""

    sign: int = -1
    offset_rad: float = 0.0

    def joint(self, bearing):
        """Return the joint angle that faces ``bearing``."""
        return wrap(self.sign * bearing + self.offset_rad)

    def bearing(self, joint):
        """Return the bearing the joint angle ``joint`` faces."""
        return wrap(self.sign * (joint - self.offset_rad))

    def delta(self, delta_bearing):
        """Return a relative bearing as a relative joint angle (not wrapped)."""
        return self.sign * delta_bearing


@dataclass(frozen=True)
class Coast:
    """How far the dome runs on after a 0: ``max(0, c1_s * |v| - c2_rad)``."""

    c1_s: float = 0.52
    c2_rad: float = 0.0

    def distance(self, speed):
        """Return the coast after a 0 at ``speed`` rad/s."""
        return max(0.0, self.c1_s * abs(speed) - self.c2_rad)


@dataclass(frozen=True)
class Profile:
    """How a move moves: top speed, acceleration, gain, and the band that counts as there."""

    v_max: float
    accel: float
    k: float
    settle_band_rad: float

    def capped(self, max_speed, max_accel):
        """Return this profile with ``v_max`` and ``accel`` at most the caps."""
        return replace(self, v_max=min(self.v_max, max_speed),
                       accel=min(self.accel, max_accel))


@dataclass(frozen=True)
class Limits:
    """The settings every move shares (``defaults.dome``)."""

    min_speed: float = 0.6       # above both floors of the velocity map (0.44 and 0.567)
    max_move_s: float = 8.0
    settle_s: float = 0.8
    coast: Coast = Coast()
    retries: int = 1
    feedback_timeout_s: float = 0.5
    max_plausible_rad_s: float = 4.0    # rule 9: the fastest dome speed a sample may imply


class RateLimiter:
    """The command (rules 1 and 2); its ``velocity`` is the last value sent."""

    def __init__(self, min_speed):
        self.min_speed = min_speed
        self.velocity = 0.0
        self._at = None

    def dt(self, now):
        """Return the time since the last step (0 before the first)."""
        return 0.0 if self._at is None else max(0.0, now - self._at)

    def step(self, wanted, v_max, accel, now):
        """
        Move toward ``wanted`` (0, or at least ``min_speed`` and at most ``v_max``).

        The caller clamps ``wanted``; a reversal passes through 0 for a tick. A value above
        ``v_max`` (a faster move's, taken over) comes down to it at once (rule 1 over 2).
        """
        dt = self.dt(now)
        self._at = now
        v, room = self.velocity, accel * dt
        if wanted == 0.0:
            out = 0.0
        elif v == 0.0:
            out = math.copysign(self.min_speed, wanted)
        elif (v > 0.0) != (wanted > 0.0):
            out = v - math.copysign(room, v)
            if abs(out) < self.min_speed:
                out = 0.0
        else:
            out = v + max(-room, min(room, wanted - v))
        if abs(out) > v_max:
            out = math.copysign(v_max, out)
        self.velocity = out
        return out


class Move:
    """One ``look_at``, ``glance``, ``turn`` or ``rest`` (see the module docstring)."""

    def __init__(self, verb, legs, profile, limits, now, budget_s, follows=None,
                 coast_scale=1.0):
        self.verb = verb
        self.profile = profile
        self.limits = limits
        self.deadline = now + budget_s
        self.done = False
        self.reason = ''
        self._legs = list(legs)
        self._retries = limits.retries
        self._phase = ''
        self._target = 0.0           # the leg's target, unwrapped
        self._until = 0.0            # end of the settle wait or the dwell
        self._creep = False          # the retry: at min_speed
        self._leg_reason = REACHED
        self._v_peak = 0.0           # the fastest command of this approach (rule 7)
        self._start = None           # unwrapped position where the move began
        self.coast_scale = coast_scale   # observed coast over predicted (rule 5)
        self.coast_learned = False   # a stop of this move has set coast_scale
        self._stop = None            # (position, direction, predicted coast) at the last 0
        self._inherited = follows is not None    # the first command is the predecessor's
        self._reversed = False       # stopped a taken-over command going the wrong way
        self._from_rest = follows is None   # rule 7: a fresh move starts from a still dome
        if follows is not None:
            self._limiter = follows._limiter
            self._u, self._sample = follows._u, follows._sample
            self._sample_at, self._moved_at = follows._sample_at, follows._moved_at
            self._v_meas = follows._v_meas
            self._held = follows._held
            self._v_peak = follows._v_peak
            self.coast_scale = follows.coast_scale
            self.coast_learned = follows.coast_learned   # its scale, to teach when done
        else:
            self._limiter = RateLimiter(limits.min_speed)
            self._u = self._sample = 0.0
            self._sample_at = self._moved_at = None
            self._v_meas = None
            self._held = None        # rule 9: a sample held back, (position, time)

    @property
    def velocity(self):
        """Return the last command."""
        return self._limiter.velocity

    def stop(self):
        """End the move now: ``(0.0, True, 'stopped')``, unless it had already ended."""
        return self._finish(STOPPED) if not self.done else (0.0, True, self.reason)

    def step(self, position_rad, now, sample_time=None):
        """
        Take the latest joint position; return ``(velocity, done, reason)``.

        ``sample_time`` is when ``position_rad`` was measured (default ``now``); the same
        sample given twice is read once.
        """
        if self.done:
            return 0.0, True, self.reason
        self._observe(position_rad, now if sample_time is None else sample_time)
        if now - self._sample_at > self.limits.feedback_timeout_s:
            return self._finish(NO_FEEDBACK)
        if now >= self.deadline:
            return self._finish(TIMEOUT)
        if self._start is None:
            self._start = self._u
            self._next_leg(now)
        while True:
            if self._phase == 'approach':
                out = self._approach(now)
                if out is not None:
                    return out
                continue
            if now < self._until or (self._phase == 'settle' and self._moving() and
                                     now < self._until + self.limits.settle_s):
                return 0.0, False, ''
            if self._phase == 'settle' and self._reversed:
                self._reversed = False           # it was going the other way: start over
                self._begin_approach(creep=False)
                continue
            if self._phase == 'settle':
                self._learn_coast()
                err = self._target - self._u
                if abs(err) > self.profile.settle_band_rad and self._retries > 0:
                    self._retries -= 1
                    self._begin_approach(creep=True)
                    continue
                self._leg_reason = (REACHED if abs(err) <= self.profile.settle_band_rad
                                    else SETTLED_SHORT)
            if not self._next_leg(now):
                return self._finish(self._leg_reason)

    # --- internals -------------------------------------------------------------------------

    def _observe(self, position, at):
        position = wrap(position)
        if self._sample_at is None:
            self._u, self._sample = position, position
            self._sample_at = self._moved_at = at
        elif at > self._sample_at:
            self._sample_at = at
            held, self._held = self._held, None
            if held is not None and self._plausible(*held, position, at):
                self._accept(*held)             # rule 9: the next sample vouches for it
                self._accept(position, at)
            elif self._plausible(self._sample, self._moved_at, position, at):
                self._accept(position, at)      # (a held sample that nothing vouched for is gone)
            else:
                self._held = (position, at)     # rule 9: wait for the next sample

    def _plausible(self, from_position, from_at, position, at):
        """Rule 9: whether the speed from ``from_position`` to ``position`` is possible."""
        return abs(wrap(position - from_position)) <= self.limits.max_plausible_rad_s * (
            at - from_at)

    def _accept(self, position, at):
        step = wrap(position - self._sample)
        if step == 0.0 and self._limiter.velocity != 0.0 and self._moving():
            return                   # a commanded dome does not stop dead: a frozen reading
        self._v_meas = step / (at - self._moved_at)
        self._u += step
        self._sample, self._moved_at = position, at

    def _moving(self):
        return self._v_meas is not None and abs(self._v_meas) > STILL_RAD_S

    def _estimate(self):
        """Rule 7: the measured speed, bounded by the fastest command of this approach."""
        if self._v_meas is None:     # a fresh move starts from rest; a replacement's dome
            return 0.0 if self._from_rest else self._limiter.velocity   # runs at its command
        return max(-self._v_peak, min(self._v_peak, self._v_meas))

    def _next_leg(self, now):
        """Start the next leg; return False when there is none."""
        if not self._legs:
            return False
        kind, value = self._legs.pop(0)
        if kind == 'dwell':
            self._phase, self._until = 'dwell', now + value
            return True
        if kind == 'to':
            self._target = self._u + wrap(value - wrap(self._u))
        elif kind == 'by':
            self._target = self._start + value
        else:                        # back where the move began
            self._target = self._start
        self._retries = self.limits.retries
        self._begin_approach(creep=False)
        return True

    def _begin_approach(self, creep):
        self._phase, self._creep = 'approach', creep
        self._v_peak = abs(self._limiter.velocity)

    def _settle(self, now, until):
        self._phase, self._until = 'settle', until
        self._limiter.step(0.0, 0.0, 0.0, now)

    def _approach(self, now):
        v_est = self._estimate()
        age = min(now - self._moved_at, EXTRAPOLATE_MAX_S)
        err = self._target - (self._u + v_est * age)
        cmd = self._limiter.velocity
        tick = self._limiter.dt(now)
        if cmd == 0.0 and abs(err) <= self.profile.settle_band_rad:
            self._settle(now, now)   # there already: confirm on the samples, no motion
            return None
        if cmd != 0.0:
            toward = abs(v_est) if v_est * cmd > 0.0 else 0.0
            coast = self.limits.coast.distance(toward)
            if err * cmd <= 0.0 and self._inherited:
                self._settle(now, now + self.limits.settle_s)
                self._inherited, self._reversed = False, True
                return 0.0, False, ''
            # The nearest tick: waiting for the next one would overshoot by more.
            if err * cmd <= 0.0 or abs(err) <= self.coast_scale * coast + 0.5 * toward * tick:
                self._stop = (self._target - err, math.copysign(1.0, cmd), coast)
                self._settle(now, now + self.limits.settle_s)
                return 0.0, False, ''
        self._inherited = False
        floor = self.limits.min_speed
        speed = floor if self._creep else max(
            floor, min(self.profile.v_max, self.profile.k * abs(err)))
        top = floor if self._creep else self.profile.v_max
        out = self._limiter.step(math.copysign(speed, err), top, self.profile.accel, now)
        self._v_peak = max(self._v_peak, abs(out))
        return out, False, ''

    def _learn_coast(self):
        """Rule 5: scale the coast by what the last stop showed (a sim has none)."""
        if self._stop is not None:
            at, direction, predicted = self._stop
            if predicted >= LEARN_MIN_RAD:
                observed = max(0.0, (self._u - at) * direction)
                self.coast_scale = min(MAX_COAST_SCALE, observed / predicted)
                self.coast_learned = True
            self._stop = None

    def _finish(self, reason):
        self._limiter.velocity = 0.0
        self.done, self.reason = True, reason
        return 0.0, True, reason


# Every builder takes ``follows`` (rule 8; it brings its own coast scale) and ``coast_scale``
# (the starting scale of a move that follows none: the arbiter's :class:`CoastMemory`).

def look_at(target_joint, profile, limits, now, follows=None, verb='look_at', coast_scale=1.0):
    """Return a move to joint angle ``target_joint``, the short way."""
    return Move(verb, [('to', wrap(target_joint))], profile, limits, now, limits.max_move_s,
                follows, coast_scale)


def rest(target_joint, profile, limits, now, follows=None, coast_scale=1.0):
    """Return a ``look_at`` that the arbiter then holds (see :class:`RestHold`)."""
    return look_at(target_joint, profile, limits, now, follows, 'rest', coast_scale)


def turn(delta_joint, profile, limits, now, follows=None, coast_scale=1.0):
    """Return a move by ``delta_joint`` (may exceed pi; capped at +-:data:`MAX_TURN_RAD`)."""
    delta = max(-MAX_TURN_RAD, min(MAX_TURN_RAD, delta_joint))
    return Move('turn', [('by', delta)], profile, limits, now, limits.max_move_s, follows,
                coast_scale)


def glance(delta_joint, dwell_s, profile, limits, now, follows=None, coast_scale=1.0):
    """Return a move out by ``delta_joint``, a dwell of ``dwell_s``, and back to the start."""
    delta = max(-math.pi, min(math.pi, delta_joint))
    return Move('glance', [('by', delta), ('dwell', dwell_s), ('back', None)], profile,
                limits, now, limits.max_move_s + dwell_s, follows, coast_scale)


class CoastMemory:
    """
    Rule 5 across moves: the coast scale the last move learned, for the next one.

    The scale fades toward 1.0 (the fit) with a half-life of ``half_life_s``: the coast
    drifts with the battery and the temperature (plan section 8), and one odd stop (a hand
    on the dome while it settled) should not outlive a quiet spell. Every move that stops
    learns again, so on a busy dome the fade hardly matters.
    """

    def __init__(self, half_life_s=COAST_HALF_LIFE_S):
        self.half_life_s = half_life_s
        self._scale = 1.0
        self._at = None

    def learn(self, move, now):
        """
        Keep ``move``'s scale if one of its stops taught it one and it ended by itself.

        Not from a move that was stopped (a person, a claim, suppression), lost its
        feedback or was cut short without being told (a stop intent, a claim's pre-emption,
        a replacement: not ``done``): what its last settle saw may have been a hand on the
        dome, and a replacement carries the scale on by itself.
        """
        if move.coast_learned and move.done and move.reason not in (STOPPED, NO_FEEDBACK):
            self._scale, self._at = move.coast_scale, now

    def recall(self, now):
        """Return the scale for a move starting at ``now``."""
        if self._at is None:
            return 1.0
        fade = 0.5 ** (max(0.0, now - self._at) / self.half_life_s)
        return 1.0 + (self._scale - 1.0) * fade


class RestHold:
    """The ``rest`` recheck: one drift check per ``recheck_s`` at most, once armed."""

    def __init__(self, recheck_s):
        self.recheck_s = recheck_s
        self.next_check = None       # None: not armed

    def arm(self, now):
        """Check next ``recheck_s`` from ``now`` (a move just ended)."""
        self.next_check = now + self.recheck_s

    def poll(self, error_rad, band_rad, now):
        """Return True when a check is due and the drift ``error_rad`` is beyond the band."""
        if self.next_check is None or now < self.next_check:
            return False
        self.next_check = now + self.recheck_s
        return abs(error_rad) > band_rad
