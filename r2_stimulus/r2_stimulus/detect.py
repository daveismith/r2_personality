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
Pure threshold logic of the stimulus adapters: no ROS imports, time is an argument.

Each detector takes ``now`` (seconds, any monotonic origin) and returns the stimuli to
emit as plain tuples, so the nodes stay thin and the thresholds can be tested with
synthetic streams.
"""

from collections import deque
import json
import math


def _clamp01(value):
    return max(0.0, min(1.0, value))


def _norm(value, full_scale):
    """Return |value| / full_scale clamped to [0, 1]; 1 when the scale is unusable."""
    if not full_scale > 0.0:
        return 1.0
    return _clamp01(abs(value) / full_scale)


def wrap_pi(angle):
    """Wrap an angle difference into (-pi, pi]."""
    wrapped = math.fmod(angle + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


class DriveDetector:
    """From odometry speed to ``being_driven`` and ``stopped``."""

    def __init__(self, moving_mps=0.05, stop_after_s=1.0, being_driven_period_s=1.0,
                 max_speed_mps=1.0):
        self.moving_mps = moving_mps
        self.stop_after_s = stop_after_s
        self.being_driven_period_s = being_driven_period_s
        self.max_speed_mps = max_speed_mps
        self._last_emit = None
        self._moved = False
        self._below_since = None

    def update(self, speed_mps, now):
        """Feed the planar speed (m/s); return [('being_driven', i)] / [('stopped', 1.0)]."""
        if speed_mps is None or math.isnan(speed_mps):
            return []
        speed = abs(speed_mps)
        out = []
        if speed > self.moving_mps:
            self._moved = True
            self._below_since = None
            if self._last_emit is None or now - self._last_emit >= self.being_driven_period_s:
                self._last_emit = now
                out.append(('being_driven', _norm(speed, self.max_speed_mps)))
        elif self._moved:
            if self._below_since is None:
                self._below_since = now
            if now - self._below_since >= self.stop_after_s:
                self._moved = False
                self._below_since = None
                out.append(('stopped', 1.0))
        return out


class DomePushDetector:
    """
    Dome moving while nobody commands it: ``dome_pushed``.

    Velocity is the displacement over the last ``window_s`` (0.4 s) of unwrapped
    positions divided by the time spanned, never a sample-to-sample difference: the
    joint state is 10 Hz and quantised (about 1 degree), which makes a single step
    differ by 0.17 rad/s, but over 0.4 s the error is at most about 0.05 rad/s. At least
    ``min_span_s`` of history is needed before any velocity is reported.
    """

    def __init__(self, pushed_omega=0.3, commanded_eps=0.02, pushed_after_s=0.3,
                 max_omega=6.65, command_timeout_s=0.5, pushed_period_s=1.0,
                 settle_s=0.5, window_s=0.4, min_span_s=0.25):
        self.pushed_omega = pushed_omega
        self.commanded_eps = commanded_eps
        self.pushed_after_s = pushed_after_s
        self.max_omega = max_omega
        self.command_timeout_s = command_timeout_s
        self.pushed_period_s = pushed_period_s
        self.settle_s = settle_s
        self.window_s = window_s
        self.min_span_s = min_span_s
        self._samples = deque()          # (time, unwrapped position)
        self._raw_last = None
        self._unwrapped = 0.0
        self._cmd = 0.0
        self._cmd_time = None
        self._last_active = None         # last time a non-zero command was in force
        self._since = None               # when "pushed" conditions began to hold
        self._last_emit = None

    def _effective_command(self, now):
        if self._cmd_time is None or now - self._cmd_time > self.command_timeout_s:
            return 0.0
        return self._cmd

    def _note_activity(self, now):
        """Record up to when the last non-zero command was in force."""
        if self._cmd_time is not None and abs(self._cmd) >= self.commanded_eps:
            until = min(now, self._cmd_time + self.command_timeout_s)
            if self._last_active is None or until > self._last_active:
                self._last_active = until

    def command(self, omega, now):
        """Feed a commanded dome velocity (rad/s)."""
        if omega is None or math.isnan(omega):
            return
        self._note_activity(now)
        self._cmd = omega
        self._cmd_time = now
        self._note_activity(now)

    def velocity(self):
        """Return the windowed velocity in rad/s, or None while history is too short."""
        if len(self._samples) < 2:
            return None
        t0, p0 = self._samples[0]
        t1, p1 = self._samples[-1]
        if t1 - t0 < self.min_span_s:
            return None
        return (p1 - p0) / (t1 - t0)

    def position(self, rad, now):
        """Take a dome position in rad; return the dome_pushed stimuli to emit, if any."""
        if rad is None or math.isnan(rad):
            return []
        if self._raw_last is not None:
            self._unwrapped += wrap_pi(rad - self._raw_last)
        self._raw_last = rad
        self._samples.append((now, self._unwrapped))
        # Keep the newest sample older than the window as its left edge.
        while len(self._samples) > 2 and self._samples[1][0] <= now - self.window_s:
            self._samples.popleft()

        self._note_activity(now)
        omega = self.velocity()
        uncommanded = abs(self._effective_command(now)) < self.commanded_eps
        settled = (self._last_active is None
                   or now - self._last_active >= self.settle_s)
        if omega is None or abs(omega) <= self.pushed_omega or not uncommanded or not settled:
            self._since = None
            return []
        if self._since is None:
            self._since = now
        if now - self._since < self.pushed_after_s:
            return []
        if self._last_emit is not None and now - self._last_emit < self.pushed_period_s:
            return []
        self._last_emit = now
        return [('dome_pushed', _norm(omega, self.max_omega))]


def button_stimulus(taps, value, seq_id, source):
    """Return (kind, subject, intensity, extra_json) for one DomeButtonSequence."""
    extra = json.dumps({'value': int(value), 'id': int(seq_id), 'source': int(source)},
                       separators=(',', ':'))
    return ('button_sequence', taps, min(1.0, len(taps) / 6.0), extra)
