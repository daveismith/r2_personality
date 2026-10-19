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

import pytest
from r2_stimulus.detect import button_stimulus
from r2_stimulus.detect import Decimator
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
    det = DriveDetector(max_speed_mps=2.0, confirm_s=0.0)
    assert det.update(0.5, 0.0) == [('being_driven', 0.25)]
    det = DriveDetector(max_speed_mps=0.4, confirm_s=0.0)
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
    out = _run_drive(det, [0.4] * 6 + [0.0] * 5 + [0.4] * 6 + [math.nan] * 30)
    # The 0.5 s pause neither stops nor restarts: one per 1 s period, and no stopped.
    assert [k for _, (k, _) in out] == ['being_driven', 'being_driven']
    assert det.update(math.nan, 100.0) == []


def test_moving_again_after_stop_restarts():
    det = DriveDetector()
    out = _run_drive(det, [0.4] * 5 + [0.0] * 20 + [0.4] * 5)
    kinds = [k for _, (k, _) in out]
    assert kinds == ['being_driven', 'stopped', 'being_driven']


def test_inf_speed_ignored():
    det = DriveDetector(confirm_s=0.0)
    assert det.update(math.inf, 0.0) == []
    assert det.update(-math.inf, 0.1) == []
    assert det.update(0.5, 0.2) == [('being_driven', 0.5)]


def test_spin_in_place_is_driving_not_stopped():
    # The node feeds max(|v|, |wz| * track / 2); here: forward, then a spin at 0.3 m/s.
    det = DriveDetector()
    out = _run_drive(det, [0.4] * 5 + [0.3] * 40)
    kinds = [k for _, (k, _) in out]
    assert 'stopped' not in kinds
    assert kinds.count('being_driven') >= 4


def _ramp(peak, rise_s, hold_s, rate_hz=50.0):
    n = int((rise_s + hold_s) * rate_hz)
    return [min(peak, peak * i / (rise_s * rate_hz)) for i in range(n)]


def test_ramped_odom_first_intensity_is_the_peak():
    det = DriveDetector(max_speed_mps=1.0)
    out = _run_drive(det, _ramp(0.5, 0.2, 1.5), dt=0.02)
    first = out[0][1]
    assert first[0] == 'being_driven' and abs(first[1] - 0.5) < 0.01


def test_drive_shorter_than_confirm_emits_nothing():
    det = DriveDetector()
    out = _run_drive(det, _ramp(0.5, 0.1, 0.1) + [0.0] * 200, dt=0.02)
    assert out == []


def test_drive_clock_jump_back_resets():
    det = DriveDetector()
    assert [k for _, (k, _) in _run_drive(det, [0.5] * 5, t0=1000.0)] == ['being_driven']
    out = _run_drive(det, [0.5] * 5, t0=0.0)         # a sim reset: time starts over
    assert [k for _, (k, _) in out] == ['being_driven']


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
        _stream(det, 0.0, 1.0, 0.0, start=start)             # at rest first (arms)
        out = _stream(det, 1.1, 4.0, omega, start=start - omega * 1.1)
        assert out, 'wrap broke the velocity for omega=%s' % omega
        assert abs(abs(det.velocity()) - 0.5) < 0.06


def test_slow_drift_below_threshold_emits_nothing():
    det = DomePushDetector()
    assert _stream(det, 0.0, 10.0, 0.2) == []


def test_intensity_clamped():
    det = DomePushDetector(max_omega=0.4)
    _stream(det, 0.0, 1.0, 0.0)
    out = _stream(det, 1.1, 4.0, 0.8, start=-0.8 * 1.1)
    assert out and out[0][1][1] == 1.0


def test_wrap_pi():
    assert abs(wrap_pi(2 * math.pi - 0.1) + 0.1) < 1e-9
    assert abs(wrap_pi(-2 * math.pi + 0.1) - 0.1) < 1e-9


def _coast_stream(det, lag, settle_end=8.0):
    """Command 0.5 rad/s until t=2, then the -35 % bench coast; positions lag by `lag`."""
    v0, T, dist = 1.867, 0.77, math.radians(36.0)    # 107 deg/s decaying to rest, 36 deg
    k = v0 * T / dist - 1.0
    out = []
    for n in range(0, int(settle_end / STEP)):
        t = n * STEP
        if t <= 2.0 + 1e-9:
            det.command(0.5, t)
        else:
            det.command(0.0, t)
        te = max(0.0, t - lag)                      # when the reported position was true
        if te <= 2.0:
            pos = 0.5 * te                          # a steady run at 0.5 rad/s
        else:
            x = min(1.0, (te - 2.0) / T)
            pos = 0.5 * 2.0 + dist * (1.0 - (1.0 - x) ** (k + 1.0))
        out += det.position(_quantise(pos) % (2 * math.pi), t)
    return out


def test_measured_coast_with_position_lag_emits_nothing():
    for lag in (0.0, 0.1, 0.2):
        assert _coast_stream(DomePushDetector(), lag) == [], lag
        # Even with the old short settle, the arming rule alone must hold.
        assert _coast_stream(DomePushDetector(settle_s=0.5), lag) == [], lag


def test_push_after_the_dome_came_to_rest_is_emitted():
    det = DomePushDetector()
    assert _coast_stream(det, 0.1, settle_end=6.0) == []
    out = []
    for n in range(61, 100):                          # t = 6.1 .. 9.9, pushed at 0.5 rad/s
        t = n * STEP
        det.command(0.0, t)
        out += det.position(_quantise(1.0 + 0.5 * (t - 6.1) + 0.9) % (2 * math.pi), t)
    assert out and out[0][0] == 'dome_pushed'


def test_non_finite_inputs_are_ignored_and_do_not_poison_the_detector():
    det = DomePushDetector()
    assert det.position(math.inf, 0.0) == []
    assert det.position(-math.inf, 0.1) == []
    det.command(math.inf, 0.1)                        # not a command
    det.command(math.nan, 0.1)
    assert _stream(det, 0.2, 1.0, 0.0) == []
    out = _stream(det, 1.1, 4.0, 0.5, start=-0.5 * 1.1)
    assert out and out[0][1][0] == 'dome_pushed'      # inf did not count as commanded


def test_dome_clock_jump_back_resets_and_window_stays_bounded():
    det = DomePushDetector()
    _stream(det, 100.0, 105.0, 0.0)
    out = _stream(det, 0.0, 3.0, 0.0)
    assert out == [] and len(det._samples) <= 8
    out = _stream(det, 3.1, 6.0, 0.5, start=-0.5 * 3.1)
    assert out


# ---- buttons ----------------------------------------------------------------------------

def test_button_mapping():
    kind, subject, intensity, extra = button_stimulus('LRR', 4, 1, 1)
    assert (kind, subject) == ('button_sequence', 'LRR')
    assert abs(intensity - 0.5) < 1e-9
    assert json.loads(extra) == {'value': 4, 'id': 1, 'source': 1}
    assert button_stimulus('LLRLRLRL', 0, 2, 0)[2] == 1.0


def test_decimator_passes_first_then_every_nth():
    d = Decimator(5)
    passed = [i for i in range(23) if d.take()]
    assert passed == [0, 5, 10, 15, 20]


def test_decimator_one_passes_all_and_rejects_zero():
    d = Decimator(1)
    assert all(d.take() for _ in range(10))
    with pytest.raises(ValueError):
        Decimator(0)


def _polled(det, t_from, t_to, omega, pos0, source_period=0.1, poll=0.1, phase=0.037):
    """
    Feed samples the way a polled depth-1 subscription sees a source_period stream.

    Each poll delivers only the newest sample sent since the previous one, stamped with the
    poll time; with a source period slightly off the poll period, samples are skipped
    now and then and others arrive up to one poll late.
    """
    out = []
    last = -1
    for k in range(int(round(t_from / poll)), int(round(t_to / poll)) + 1):
        now = k * poll
        n = int(math.floor((now - phase) / source_period))
        if n < 0 or n == last:
            continue
        last = n
        t_sample = n * source_period + phase
        pos = _quantise(pos0 + omega * t_sample) % (2 * math.pi)
        out += [(now, e) for e in det.position(pos, now)]
    return out


def test_polled_stream_with_skipped_samples_still_detects_a_push():
    for period in (0.1, 0.099, 0.101, 0.095, 0.105):
        det = DomePushDetector(max_omega=6.65)
        assert _polled(det, 0.0, 2.0, 0.0, 0.0, period) == []
        out = _polled(det, 2.0, 6.0, 0.5, -1.0, period)
        assert out, 'push not detected at source period %s' % period
        assert out[0][1][0] == 'dome_pushed'
        assert abs(out[0][1][1] - 0.5 / 6.65) < 0.03, (period, out[0])


def test_polled_stream_idle_and_commanded_emit_nothing():
    for period in (0.1, 0.099, 0.101, 0.095, 0.105):
        det = DomePushDetector(max_omega=6.65)
        assert _polled(det, 0.0, 10.0, 0.0, 1.0, period) == []
        det = DomePushDetector(max_omega=6.65)
        for n in range(0, 100):
            det.command(0.5, n * 0.1)
        # commanded 0.5 rad/s while moving at 0.5: not a push even with skipped samples
        out = []
        for k in range(0, 60):
            det.command(0.5, k * 0.1)
            out += _polled(det, k * 0.1, k * 0.1, 0.5, 0.0, period)
        assert out == []
