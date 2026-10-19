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
    """
    From odometry speed to ``being_driven`` and ``stopped``.

    diff_cont's odometry velocity is a rolling mean, so it ramps up over about 0.2 s. The
    first ``being_driven`` is therefore held back until ``confirm_s`` of continuous motion
    and reports the peak speed seen since the motion (or the last emission) began, so it
    is not always weak. A drive shorter than ``confirm_s`` emits nothing, not even
    ``stopped``.
    """

    def __init__(self, moving_mps=0.05, stop_after_s=1.0, being_driven_period_s=1.0,
                 max_speed_mps=1.0, confirm_s=0.25):
        self.moving_mps = moving_mps
        self.stop_after_s = stop_after_s
        self.being_driven_period_s = being_driven_period_s
        self.max_speed_mps = max_speed_mps
        self.confirm_s = confirm_s
        self._reset()

    def _reset(self):
        self._last_seen = None
        self._last_emit = None
        self._moved = False             # motion confirmed, a stopped is owed
        self._moving_since = None
        self._peak = 0.0
        self._below_since = None

    def update(self, speed_mps, now):
        """Feed the speed (m/s); return [('being_driven', i)] / [('stopped', 1.0)] / []."""
        if speed_mps is None or not math.isfinite(speed_mps):
            return []
        if self._last_seen is not None and now < self._last_seen:
            self._reset()                # the clock jumped back (sim reset, looping bag)
        self._last_seen = now
        speed = abs(speed_mps)
        if speed > self.moving_mps:
            self._below_since = None
            if self._moving_since is None:
                self._moving_since = now
            self._peak = max(self._peak, speed)
            if not self._moved and now - self._moving_since < self.confirm_s:
                return []
            self._moved = True
            if self._last_emit is None or now - self._last_emit >= self.being_driven_period_s:
                self._last_emit = now
                peak, self._peak = self._peak, 0.0
                return [('being_driven', _norm(peak, self.max_speed_mps))]
            return []
        if not self._moved:
            self._moving_since = None    # too brief to count
            self._peak = 0.0
            return []
        if self._below_since is None:
            self._below_since = now
        if now - self._below_since >= self.stop_after_s:
            self._moved = False
            self._moving_since = None
            self._below_since = None
            self._peak = 0.0
            return [('stopped', 1.0)]
        return []


class DomePushDetector:
    """
    Dome moving while nobody commands it: ``dome_pushed``.

    Velocity is the displacement over the last ``window_s`` (0.4 s) of unwrapped
    positions divided by the time spanned, never a sample-to-sample difference: the
    joint state is 10 Hz and quantised (about 1 degree), which makes a single step
    differ by 0.17 rad/s, but over 0.4 s the error is at most about 0.05 rad/s. At least
    ``min_span_s`` of history is needed before any velocity is reported.

    The dome coasts for a second or more and tens of degrees after a command ends, and the
    position stream lags. So a push is only reported once the dome has been seen at rest
    (windowed speed at or below ``pushed_omega``) at least once, after ``settle_s`` has
    passed since the last non-zero command: the coast must be over first.
    """

    def __init__(self, pushed_omega=0.3, commanded_eps=0.02, pushed_after_s=0.3,
                 max_omega=6.65, command_timeout_s=0.5, pushed_period_s=1.0,
                 settle_s=1.5, window_s=0.4, min_span_s=0.25):
        self.pushed_omega = pushed_omega
        self.commanded_eps = commanded_eps
        self.pushed_after_s = pushed_after_s
        self.max_omega = max_omega
        self.command_timeout_s = command_timeout_s
        self.pushed_period_s = pushed_period_s
        self.settle_s = settle_s
        self.window_s = window_s
        self.min_span_s = min_span_s
        self._raw_last = None
        self._unwrapped = 0.0
        self._reset()

    def _reset(self):
        self._samples = deque()          # (time, unwrapped position)
        self._cmd = 0.0
        self._cmd_time = None
        self._last_active = None         # last time a non-zero command was in force
        self._since = None               # when "pushed" conditions began to hold
        self._last_emit = None
        self._armed = False              # seen at rest since the last command
        self._last_seen = None

    def _see(self, now):
        if self._last_seen is not None and now < self._last_seen:
            self._reset()                # the clock jumped back (sim reset, looping bag)
        self._last_seen = now

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
        if omega is None or not math.isfinite(omega):
            return
        self._see(now)
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
        if rad is None or not math.isfinite(rad):
            return []
        self._see(now)
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
        if not uncommanded or not settled:
            self._armed = False
            self._since = None
            return []
        if omega is None:
            self._since = None
            return []
        if abs(omega) <= self.pushed_omega:
            self._armed = True           # at rest (or crawling) since the last command
            self._since = None
            return []
        if not self._armed:              # still coasting down from a command
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


class Decimator:
    """Pass the first sample and then every ``every``-th one (1 passes all)."""

    def __init__(self, every):
        if every < 1:
            raise ValueError('every must be >= 1, got %r' % every)
        self._every = every
        self._count = 0

    def take(self):
        """Count one sample; return True if it is to be processed."""
        take = self._count == 0
        self._count = (self._count + 1) % self._every
        return take
