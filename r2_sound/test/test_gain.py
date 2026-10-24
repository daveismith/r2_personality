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

"""Tests for the loudness measurement and gain arithmetic (plan 2.3)."""

import math

import numpy as np
import pytest

from fixtures import RATE, tone  # noqa: I100
from r2_sound import gain


def test_sine_measures_rms_peak_and_duration():
    m = gain.measure(tone(1000, 2.0, 0.5), RATE)
    assert m.rms_dbfs == pytest.approx(-9.03, abs=0.05)
    assert m.peak_dbfs == pytest.approx(-6.02, abs=0.05)
    assert m.duration_s == pytest.approx(2.0, abs=0.001)
    assert m.clip_fraction == 0.0


def test_rms_ignores_the_silent_part():
    x = np.concatenate([tone(1000, 1.0, 0.5), np.zeros(RATE * 3)])
    assert gain.measure(x, RATE).rms_dbfs == pytest.approx(-9.03, abs=0.1)


def test_rms_ignores_a_quiet_tail_below_minus_50():
    x = np.concatenate([tone(1000, 1.0, 0.5), tone(1000, 2.0, 0.001)])   # -60 dBFS
    assert gain.measure(x, RATE).rms_dbfs == pytest.approx(-9.03, abs=0.1)


def test_a_quiet_sine_above_the_gate_is_not_biased():
    gated = tone(1000, 2.0, 10 ** (-60 / 20) * math.sqrt(2))   # -60 dBFS RMS
    quiet = tone(1000, 2.0, 10 ** (-43 / 20) * math.sqrt(2))   # -43 dBFS RMS
    assert gain.measure(quiet, RATE).rms_dbfs == pytest.approx(-43.0, abs=0.1)
    assert gain.measure(gated, RATE).rms_dbfs == gain.FLOOR_DBFS


def test_stereo_frames_count_once():
    left = tone(1000, 1.0, 0.5)
    stereo = np.stack([left, left], axis=1)
    assert gain.measure(stereo, RATE).rms_dbfs == pytest.approx(-9.03, abs=0.05)


def test_clipped_fraction_counts_samples_at_full_scale():
    x = np.zeros(1000)
    x[:100] = 1.0
    x[100:150] = -0.999
    x[150:200] = 0.99
    m = gain.measure(x, RATE)
    assert m.clip_fraction == pytest.approx(0.15)
    assert m.peak_dbfs == pytest.approx(0.0)


def test_peak_above_full_scale_is_reported_not_clamped():
    assert gain.measure(np.array([1.5, -0.2]), RATE).peak_dbfs == pytest.approx(3.52, abs=0.01)


def test_silence_and_empty_measure_as_the_floor():
    assert gain.measure(np.zeros(100), RATE).rms_dbfs == gain.FLOOR_DBFS
    assert gain.measure(np.zeros(100), RATE).peak_dbfs == gain.FLOOR_DBFS
    assert gain.measure(np.zeros(0), RATE).duration_s == 0.0


def test_set_gain_brings_the_median_to_the_house_level():
    assert gain.set_gain_db(-10.6) == pytest.approx(-5.4)
    assert gain.set_gain_db(-17.5) == pytest.approx(1.5)
    assert gain.set_gain_db(-20.0, house_level_dbfs=-18.0) == pytest.approx(2.0)


def test_median_of_values():
    assert gain.median([3.0, 1.0, 2.0]) == 2.0
    assert gain.median([]) == 0.0


@pytest.mark.parametrize('rms, trim', [
    (-10.0, 0.0), (-2.0, 0.0),            # exactly 8 dB or inside: kept
    (-1.0, -4.5),                         # 9 dB above the median of -10: half the deviation
    (2.0, -6.0), (-19.0, 4.5), (-30.0, 10.0)])
def test_outlier_trim_pulls_halfway_back(rms, trim):
    assert gain.outlier_trim_db(rms, -10.0) == pytest.approx(trim)


def test_clip_gain_adds_set_trim_and_override():
    assert gain.clip_gain_db(-5.0, -2.0, 1.5, peak_dbfs=-20.0) == pytest.approx(-5.5)


def test_clip_gain_is_limited_by_the_peak_ceiling():
    # +6 dB would put a -3 dBFS peak at +3: limited to -1 - (-3) = +2
    assert gain.clip_gain_db(6.0, 0.0, 0.0, peak_dbfs=-3.0) == pytest.approx(2.0)
    # a peak already above the ceiling gets a cut, even with a positive override
    assert gain.clip_gain_db(0.0, 0.0, 3.0, peak_dbfs=1.0) == pytest.approx(-2.0)


def test_clip_gain_stays_in_the_catalogue_range():
    assert gain.clip_gain_db(30.0, 0.0, 0.0, peak_dbfs=-90.0) == gain.MAX_GAIN_DB
    assert gain.clip_gain_db(-40.0, 0.0, 0.0, peak_dbfs=-6.0) == gain.MIN_GAIN_DB


def test_aligned_sets_land_on_the_house_level():
    medians = {'a': -13.3, 'b': -10.6}
    for median in medians.values():
        assert median + gain.set_gain_db(median) == pytest.approx(gain.HOUSE_LEVEL_DBFS)


def test_level_mapping_endpoints_and_clamps():
    assert gain.level_to_byte(-40.0) == 0
    assert gain.level_to_byte(-60.0) == 0
    assert gain.level_to_byte(-6.0) == 255
    assert gain.level_to_byte(0.0) == 255
    assert gain.level_to_byte(-23.0) in (127, 128)


def test_level_mapping_is_monotonic():
    values = [gain.level_to_byte(db) for db in np.arange(-50, 0, 0.5)]
    assert values == sorted(values)
    assert math.isclose(gain.level_to_byte(-31.5, -40, -6), 64, abs_tol=1)
