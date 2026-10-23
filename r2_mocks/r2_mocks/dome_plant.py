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

The numbers are plan 2.1 (``ros2_roamadome``'s calibration findings and ``ros2_control.xacro``):

* **Dead band**: a velocity command with ``|v| < 0.44 rad/s`` sends 0 % and the motor is off.
* **Lag**: the driver has no ramp; the step response is first order and reaches 90 % of the
  commanded speed at 1.5 s, so ``tau = 1.5 / ln(10)`` (about 0.65 s).
* **Coast**: when the motor goes off (a command of 0, or one under the dead band) the dome
  coasts a further ``0.37 * v_at_stop - 0.07`` rad (``0.37 s * v - 4 deg``), never negative,
  and stops. The mock slows at a constant rate over exactly that distance (so the distance is
  exact whatever ``dt``). The fit is linear and is the findings' own; it is not the real curve.
* **Position**: the firmware's angle wrapped to [0, 2*pi). The firmware never reports 359 to
  2 deg, so each crossing of 0 reads about 9 deg long (findings: "the 355 to 4 deg dead
  zone"). With ``firmware_quirks`` the reported angle never takes a value inside
  (355 deg, 4 deg): a true angle in [0, 4 deg) reads 4 deg and one in (355 deg, 360 deg) reads
  355 deg, so a crossing, in either direction, is one step of 9 deg between 355 and 4 deg
  readings, while the dome moved less. The reading is a function of the true angle alone (no
  hysteresis, no drift: the error is at most 4.5 deg and only near 0).

``ideal=True`` is a pure integrator, like Gazebo's joint under a velocity controller: the
velocity is the command at once (no dead band, lag or coast, and no quirks), and the position
is still reported wrapped to [0, 2*pi). Positive and negative are symmetric (the real map's
per-direction asymmetry is not modelled).
"""

import math

DEAD_BAND_RAD_S = 0.44
LAG_90_PERCENT_S = 1.5
TAU_S = LAG_90_PERCENT_S / math.log(10.0)
COAST_S = 0.37                  # the coast distance is COAST_S * v - COAST_OFFSET_RAD
COAST_OFFSET_RAD = math.radians(4.0)
GAP_LOW_RAD = math.radians(355.0)       # the firmware reports neither side of 0 between these
GAP_HIGH_RAD = math.radians(4.0)
TWO_PI = 2.0 * math.pi


def wrap(angle):
    """Return ``angle`` wrapped to [0, 2*pi)."""
    wrapped = angle % TWO_PI
    return 0.0 if wrapped >= TWO_PI else wrapped


def coast_distance(speed):
    """Return the distance in rad the dome coasts from ``|speed|`` rad/s: ``0.37 v - 4 deg``."""
    return max(0.0, COAST_S * abs(speed) - COAST_OFFSET_RAD)


class DomePlant:
    """The dome's true position and velocity, and the position the firmware reports."""

    def __init__(self, ideal=False, firmware_quirks=True, initial_rad=0.0):
        self.ideal = ideal
        self.firmware_quirks = firmware_quirks and not ideal
        self._position = float(initial_rad)
        self._velocity = 0.0
        self._coast_decel = 0.0         # > 0 while coasting after the motor went off

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
        if self.firmware_quirks and (angle < GAP_HIGH_RAD or angle > GAP_LOW_RAD):
            return GAP_HIGH_RAD if angle < GAP_HIGH_RAD else GAP_LOW_RAD
        return angle

    def step(self, command_rad_s, dt):
        """Advance ``dt`` s under the velocity command and return the reported position."""
        if dt > 0.0 and math.isfinite(dt):
            if not math.isfinite(command_rad_s):
                command_rad_s = 0.0
            if self.ideal:
                self._velocity = command_rad_s
                self._position += command_rad_s * dt
            elif abs(command_rad_s) < DEAD_BAND_RAD_S:
                self._coast(dt)
            else:
                self._drive(command_rad_s, dt)
        return self.reported_rad

    def _drive(self, target, dt):
        """Lag first-order toward ``target``, integrating the position exactly."""
        self._coast_decel = 0.0
        decay = math.exp(-dt / TAU_S)
        self._position += target * dt - (target - self._velocity) * TAU_S * (1.0 - decay)
        self._velocity = target + (self._velocity - target) * decay

    def _coast(self, dt):
        """Slow, the motor being off, at a constant rate over exactly ``coast_distance``."""
        speed = abs(self._velocity)
        if speed == 0.0:
            self._coast_decel = 0.0
            return
        if not self.coasting:
            distance = coast_distance(speed)
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
