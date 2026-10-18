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

import json
import math

from r2_stimulus.detect import button_stimulus
from r2_stimulus.detect import DomePushDetector
from r2_stimulus.detect import DriveDetector
from r2_stimulus.detect import wrap_pi


def _run_drive(det, speeds, dt=0.1, t0=0.0):
    out = []
    for i, v in enumerate(speeds):
        for item in det.update(v, t0 + i * dt):
            out.append((t0 + i * dt, item))
    return out


def test_being_driven_cadence_one_per_period():
    det = DriveDetector(max_speed_mps=1.0)
    out = _run_drive(det, [0.5] * 35)          # 3.5 s of motion at 10 Hz
    times = [t for t, (kind, _) in out if kind == 'being_driven']
    assert len(times) == 4
    assert all(abs((b - a) - 1.0) < 1e-6 for a, b in zip(times, times[1:]))


def test_intensity_scaling_and_clamp():
    det = DriveDetector(max_speed_mps=2.0)
    assert det.update(0.5, 0.0) == [('being_driven', 0.25)]
    det = DriveDetector(max_speed_mps=0.4)
    assert det.update(-3.0, 0.0) == [('being_driven', 1.0)]


def test_stopped_once_after_stop_after_s():
    det = DriveDetector()
    out = _run_drive(det, [0.4] * 5 + [0.0] * 40)
    stopped = [t for t, (kind, _) in out if kind == 'stopped']
    assert len(stopped) == 1
    assert abs(stopped[0] - (0.5 + 1.0)) < 1e-6    # last moving sample at 0.4, below from 0.5
    assert [i for _, (k, i) in out if k == 'stopped'] == [1.0]


def test_no_stopped_without_motion():
    det = DriveDetector()
    assert _run_drive(det, [0.0] * 50) == []
    assert _run_drive(det, [0.04] * 50) == []


def test_brief_pause_does_not_stop_and_nan_ignored():
    det = DriveDetector()
    out = _run_drive(det, [0.4] * 3 + [0.0] * 5 + [0.4] * 3 + [math.nan] * 30)
    assert all(kind == 'being_driven' for _, (kind, _) in out)
    assert det.update(math.nan, 100.0) == []


def test_moving_again_after_stop_restarts():
    det = DriveDetector()
    out = _run_drive(det, [0.4] * 3 + [0.0] * 20 + [0.4] * 3)
    kinds = [k for _, (k, _) in out]
    assert kinds == ['being_driven', 'stopped', 'being_driven']


# ---- dome -------------------------------------------------------------------------------

STEP = 0.1


def _quantise(rad):
    deg = math.radians(1.0)
    return round(rad / deg) * deg


def _stream(det, t_from, t_to, omega, start=0.0, commands=None):
    """Feed positions at 10 Hz (1-degree quantised, wrapped to 0..2pi); return emissions."""
    out = []
    n0 = int(round(t_from / STEP))
    n1 = int(round(t_to / STEP))
    for n in range(n0, n1 + 1):
        t = n * STEP
        if commands is not None:
            c = commands(t)
            if c is not None:
                det.command(c, t)
        pos = _quantise(start + omega * t) % (2 * math.pi)
        out += [(t, e) for e in det.position(pos, t)]
    return out


def test_real_push_emits_once_after_pushed_after_s_then_per_period():
    det = DomePushDetector(max_omega=6.65)
    # Idle dome with an explicit zero command at 10 Hz, then pushed at 0.5 rad/s from t=2.
    for n in range(0, 21):
        det.command(0.0, n * STEP)
        assert det.position(0.0, n * STEP) == []
    start = 0.0

    out = []
    for n in range(21, 80):
        t = n * STEP
        det.command(0.0, t)
        pos = _quantise(start + 0.5 * (t - 2.0)) % (2 * math.pi)
        out += [(t, e) for e in det.position(pos, t)]
    assert out, 'push not detected'
    first_t = out[0][0]
    # velocity needs min_span_s of history, then pushed_after_s of holding: well under 1 s.
    assert 2.0 + 0.3 <= first_t <= 3.2
    assert abs(out[0][1][1] - 0.5 / 6.65) < 0.03
    assert all(e[0] == 'dome_pushed' for _, e in out)
    times = [t for t, _ in out]
    assert all(b - a >= 1.0 - 1e-6 for a, b in zip(times, times[1:]))
    assert len(times) <= 6          # 5.9 s of push, period 1 s


def test_matching_command_emits_nothing():
    det = DomePushDetector()
    out = _stream(det, 0.0, 6.0, 0.5, commands=lambda t: 0.5)
    assert out == []


def test_motion_after_command_ends_within_settle_emits_nothing():
    det = DomePushDetector()
    # Commanded at 0.5 rad/s until t=2.0, then it coasts for 0.4 s more and stops.
    out = []
    for n in range(0, 60):
        t = n * STEP
        if t <= 2.0 + 1e-9:
            det.command(0.5, t)
            pos = 0.5 * t
        elif t <= 2.4 + 1e-9:
            det.command(0.0, t)
            pos = 0.5 * t
        else:
            det.command(0.0, t)
            pos = 0.5 * 2.4
        out += det.position(_quantise(pos) % (2 * math.pi), t)
    assert out == []


def test_stale_command_counts_as_zero():
    det = DomePushDetector()
    det.command(1.0, 0.0)            # never refreshed
    out = _stream(det, 0.0, 5.0, 0.0)
    assert out == []
    out = _stream(det, 5.1, 8.0, 0.6, start=0.0)
    # moved again with the command long stale: counts as a push
    assert out


def test_wrap_across_two_pi_both_directions():
    for omega in (0.5, -0.5):
        det = DomePushDetector()
        start = 2 * math.pi - 0.3 if omega > 0 else 0.3
        out = _stream(det, 0.0, 3.0, omega, start=start)
        assert out, 'wrap broke the velocity for omega=%s' % omega
        assert abs(abs(det.velocity()) - 0.5) < 0.06


def test_slow_drift_below_threshold_emits_nothing():
    det = DomePushDetector()
    assert _stream(det, 0.0, 10.0, 0.2) == []


def test_intensity_clamped():
    det = DomePushDetector(max_omega=0.4)
    out = _stream(det, 0.0, 3.0, 0.8)
    assert out and out[0][1][1] == 1.0


def test_wrap_pi():
    assert abs(wrap_pi(2 * math.pi - 0.1) + 0.1) < 1e-9
    assert abs(wrap_pi(-2 * math.pi + 0.1) - 0.1) < 1e-9


# ---- buttons ----------------------------------------------------------------------------

def test_button_mapping():
    kind, subject, intensity, extra = button_stimulus('LRR', 4, 1, 1)
    assert (kind, subject) == ('button_sequence', 'LRR')
    assert abs(intensity - 0.5) < 1e-9
    assert json.loads(extra) == {'value': 4, 'id': 1, 'source': 1}
    assert button_stimulus('LLRLRLRL', 0, 2, 0)[2] == 1.0
