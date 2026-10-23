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
``DomePlant``: the dome's motor, as the personality stack sees it. Pure, no ROS.

The numbers are plan 2.1: the dead bands from ``ros2_control.xacro``, the lag and coast from
the droid itself (step 8b, ``r2_expression/docs/dome-motion-measurements.md``, 2026-10-23):

* **Dead band**, per direction: ``velocity_map`` sends ``round(p0 + k * |v|)`` percent and
  zeroes anything under ``min_percent``, so a positive command under about 0.44 rad/s (p0 13.9,
  k 12.75, ``min_percent`` 20) and a negative one under about 0.567 rad/s (p0 14.5, k 12.36,
  ``min_percent`` 22) sends 0 % and the motor is off.
* **Lag**: the driver has no ramp; the response is first order and reaches 90 % of the
  achieved speed at ``lag_90_s``, default 0.6 s (measured 0.4 to 1.0 s at 0.6 to 2 rad/s), so
  ``tau = lag_90_s / ln(10)`` (about 0.26 s).
* **Coast**: when the motor goes off (a command of 0, or one under the dead band) the dome
  coasts a further ``coast_s * v_at_stop - coast_offset_rad`` rad, default ``0.52 v`` with no
  offset (the measured least-squares fit), never negative, and stops. The mock slows at a
  constant rate over exactly that distance (so the distance is exact whatever ``dt``). The fit
  is linear; it is not the real curve.

``preset`` picks the lag and coast: ``'measured_2026_10'`` (default, the droid at personality
speeds) or ``'findings_2026_10'`` (the calibration findings' step response, 1.5 s to 90 %, and
coast ``0.37 v - 4 deg``, measured at higher speeds), kept so earlier tests keep their intent.
* **Floor**: the velocity map is loose below 2 rad/s. The one point measured near the floor, a
  command of 0.6 rad/s, achieved 0.35 (0.58 of it) and coasted 0.12 rad. Commands under
  ``FLOOR_BAND_RAD_S`` (0.7) achieve ``floor_gain`` of the command: 0.58 by default, 1 in the
  findings preset. Above that the achieved speed is the command: the measured 1.0 to 2.0 rad/s
  run 0.76 to 1.29 of it, about a mean near 1, which the position loop absorbs.
* **Position**: the firmware's angle wrapped to [0, 2*pi). The firmware never reports 359 to
  2 deg, so each crossing of 0 reads about 9 deg long (findings: "the 355 to 4 deg dead
  zone"). With ``firmware_quirks`` the reported angle never takes a value inside
  (355 deg, 4 deg): a true angle in [0, 4 deg) reads 4 deg and one in (355 deg, 360 deg) reads
  355 deg, so a crossing, in either direction, is one step of 9 deg between 355 and 4 deg
  readings, while the dome moved less. The reading is a function of the true angle alone (no
  hysteresis, no drift: the error is at most 4.5 deg and only near 0).

* **Misread** (optional, off by default): ``misread_at=(low_deg, high_deg, bogus_deg)`` models the
  droid's ring sensor (measured 2026-10-23, ``r2_expression/docs/dome-motion-measurements.md``,
  "Position misreads"): each time the true wrapped angle enters ``[low, high]`` moving
  positive, the reported position reads ``bogus`` for the next ``MISREAD_S`` (0.08 s, so a
  10 Hz sample sees it once) and then the true angle again. The droid's numbers are
  ``(313, 331, 279)``; ``MEASURED_MISREAD`` has them. Anticlockwise never misreads.

``ideal=True`` is a pure integrator, like Gazebo's joint under a velocity controller: the
velocity is the command at once (no dead band, lag or coast, and no quirks), and the position
is still reported wrapped to [0, 2*pi). Apart from the dead band, positive and negative are
symmetric (the map's per-direction speeds above the floor are not modelled).
"""

import math

# ros2_control.xacro velocity_map: the speed at which round(p0 + k * v) reaches min_percent.
DEAD_BAND_POSITIVE_RAD_S = (20 - 0.5 - 13.9) / 12.75    # min_percent, p0, k: about 0.44
DEAD_BAND_NEGATIVE_RAD_S = (22 - 0.5 - 14.5) / 12.36    # about 0.567 (0.5663)
# Lag and coast: the coast distance is coast_s * v - coast_offset_rad.
PRESETS = {
    'measured_2026_10': {'lag_90_s': 0.6, 'coast_s': 0.52, 'coast_offset_rad': 0.0,
                         'floor_gain': 0.58},
    'findings_2026_10': {'lag_90_s': 1.5, 'coast_s': 0.37, 'coast_offset_rad': math.radians(4.0),
                         'floor_gain': 1.0},
}
FLOOR_BAND_RAD_S = 0.7          # commands under this achieve floor_gain of the command
DEFAULT_PRESET = 'measured_2026_10'
MEASURED_MISREAD = (313.0, 331.0, 279.0)    # degrees: sector low, sector high, the bogus reading
MISREAD_S = 0.08                # shorter than a 10 Hz sample period: one bad sample
GAP_LOW_RAD = math.radians(355.0)       # the firmware reports neither side of 0 between these
GAP_HIGH_RAD = math.radians(4.0)
TWO_PI = 2.0 * math.pi


def wrap(angle):
    """Return ``angle`` wrapped to [0, 2*pi)."""
    wrapped = angle % TWO_PI
    return 0.0 if wrapped >= TWO_PI else wrapped


def coast_distance(speed, preset=DEFAULT_PRESET):
    """Return the distance in rad the dome coasts from ``|speed|`` rad/s under ``preset``."""
    fit = PRESETS[preset]
    return max(0.0, fit['coast_s'] * abs(speed) - fit['coast_offset_rad'])


class DomePlant:
    """The dome's true position and velocity, and the position the firmware reports."""

    def __init__(self, ideal=False, firmware_quirks=True, initial_rad=0.0,
                 preset=DEFAULT_PRESET, misread_at=None):
        fit = PRESETS[preset]
        self.preset = preset
        self.lag_90_s = fit['lag_90_s']
        self._floor_gain = fit['floor_gain']
        self._tau = self.lag_90_s / math.log(10.0)
        self.ideal = ideal
        self.firmware_quirks = firmware_quirks and not ideal
        self._position = float(initial_rad)
        self._velocity = 0.0
        self._coast_decel = 0.0         # > 0 while coasting after the motor went off
        self._misread = None if misread_at is None or ideal else (
            math.radians(misread_at[0]), math.radians(misread_at[1]),
            math.radians(misread_at[2]))
        self._misread_left = 0.0        # > 0 while the reading is the bogus one

    @property
    def position_rad(self):
        """Return the true, unwrapped position in rad."""
        return self._position

    @property
    def velocity_rad_s(self):
        """Return the true velocity in rad/s."""
        return self._velocity

    @property
    def coasting(self):
        """Return whether the dome is coasting after the motor went off."""
        return self._coast_decel > 0.0

    @property
    def wrapped_rad(self):
        """Return the true position wrapped to [0, 2*pi), without the firmware's artefact."""
        return wrap(self._position)

    @property
    def reported_rad(self):
        """Return the position the firmware reports, in [0, 2*pi) (with the quirk if enabled)."""
        angle = wrap(self._position)
        if self._misread_left > 0.0:
            return self._misread[2]
        if self.firmware_quirks and (angle < GAP_HIGH_RAD or angle > GAP_LOW_RAD):
            return GAP_HIGH_RAD if angle < GAP_HIGH_RAD else GAP_LOW_RAD
        return angle

    def step(self, command_rad_s, dt):
        """Advance ``dt`` s under the velocity command and return the reported position."""
        if dt > 0.0 and math.isfinite(dt):
            before = self._position
            if not math.isfinite(command_rad_s):
                command_rad_s = 0.0
            if self.ideal:
                self._velocity = command_rad_s
                self._position += command_rad_s * dt
            elif abs(command_rad_s) < (DEAD_BAND_NEGATIVE_RAD_S if command_rad_s < 0.0
                                       else DEAD_BAND_POSITIVE_RAD_S):
                self._coast(dt)
            else:
                self._drive(command_rad_s, dt)
            self._track_misread(before, dt)
        return self.reported_rad

    def _track_misread(self, before, dt):
        """Start the bogus reading when the dome enters the sector moving positive."""
        self._misread_left = max(0.0, self._misread_left - dt)
        if self._misread is not None and self._position > before:
            low, high, _ = self._misread
            if not low <= wrap(before) <= high and low <= wrap(self._position) <= high:
                self._misread_left = MISREAD_S

    def _drive(self, target, dt):
        """Lag first-order toward ``target``, integrating the position exactly."""
        self._coast_decel = 0.0
        if abs(target) < FLOOR_BAND_RAD_S:
            target *= self._floor_gain
        decay = math.exp(-dt / self._tau)
        self._position += target * dt - (target - self._velocity) * self._tau * (1.0 - decay)
        self._velocity = target + (self._velocity - target) * decay

    def _coast(self, dt):
        """Slow, the motor being off, at a constant rate over exactly ``coast_distance``."""
        speed = abs(self._velocity)
        if speed == 0.0:
            self._coast_decel = 0.0
            return
        if not self.coasting:
            distance = coast_distance(speed, self.preset)
            if distance == 0.0:
                self._velocity = 0.0
                return
            self._coast_decel = speed * speed / (2.0 * distance)
        sign = math.copysign(1.0, self._velocity)
        remaining = speed / self._coast_decel      # time left until it stops
        if dt >= remaining:
            self._position += sign * speed * remaining / 2.0
            self._velocity = 0.0
            self._coast_decel = 0.0
        else:
            slower = speed - self._coast_decel * dt
            self._position += sign * (speed + slower) / 2.0 * dt
            self._velocity = sign * slower
