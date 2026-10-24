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

"""Tests for the acoustic features and the bucket suggestion (plan step 3)."""

import numpy as np
import pytest

from fixtures import burst_train, chirp, RATE, tone  # noqa: I100
from r2_sound import analysis
from r2_sound.catalogue import effective_tags


def features(samples, rate=RATE):
    return analysis.extract_features(samples, rate)


def feature_dict(**over):
    base = {k: 1.0 for k in analysis.FEATURE_KEYS}
    base.update(over)
    return base


def test_features_of_a_steady_tone():
    f = features(tone(1000, 1.0, 0.3))
    assert f['duration_s'] == pytest.approx(1.0)
    assert f['zcr_hz'] == pytest.approx(1000, rel=0.05)
    assert f['bursts'] == 1
    assert abs(f['zcr_slope']) < 0.1
    assert 700 < f['centroid_hz'] < 1300


def test_slopes_follow_the_pitch():
    assert features(chirp(500, 1800, 1.0))['zcr_slope'] > 0.5
    assert features(chirp(1800, 500, 1.0))['zcr_slope'] < -0.5
    assert features(chirp(500, 1800, 1.0))['centroid_slope'] > 0.3


def test_bursts_are_counted_and_rated():
    f = features(burst_train(3000, bursts=8, on=0.06, off=0.06))
    assert f['bursts'] == 8
    assert f['burst_rate'] == pytest.approx(8 / 0.96, rel=0.25)


def test_envelope_attack_and_decay():
    triangle = 1 - abs(1 - 2 * np.arange(RATE) / RATE)       # up for 0.5 s, down for 0.5 s
    f = features(tone(800, 1.0, 1.0) * (0.01 + 0.99 * triangle))
    assert f['attack_s'] > 0.3
    assert 0.2 < f['decay_frac'] < 0.7


def test_silence_has_zero_features_but_a_duration():
    f = features([0.0] * RATE)
    assert f['duration_s'] == pytest.approx(1.0)
    assert f['bursts'] == 0 and f['zcr_hz'] == 0


def test_other_sample_rates_are_resampled_and_stereo_is_mixed():
    x = tone(1000, 1.0, 0.3, rate=44100)
    f = features(np.stack([x, x], axis=1), 44100)
    assert f['duration_s'] == pytest.approx(1.0)
    assert f['zcr_hz'] == pytest.approx(1000, rel=0.1)
    g = features(tone(1000, 1.0, 0.3, rate=8000), 8000)
    assert g['zcr_hz'] == pytest.approx(1000, rel=0.1)


def test_very_short_clips_do_not_fail():
    assert features(tone(1000, 0.01, 0.3))['duration_s'] == pytest.approx(0.01, abs=1e-3)


# --- the rules -----------------------------------------------------------------------------

EMPTY = analysis.build_references([])


def test_a_short_rising_chirp_is_a_question():
    s = analysis.suggest(features(chirp(500, 1800, 1.0, 0.5)), EMPTY)
    assert (s.tag, s.source) == ('question', 'rule')
    assert s.confidence <= 0.5


def test_a_long_descending_tone_is_sad():
    s = analysis.suggest(features(chirp(1800, 400, 2.0, 0.5)), EMPTY)
    assert (s.tag, s.source) == ('sad', 'rule')
    assert s.confidence <= 0.5


def test_a_fast_burst_train_with_a_high_centroid_is_excited():
    s = analysis.suggest(features(burst_train(3000, 10, 0.05, 0.05, 0.5)), EMPTY)
    assert (s.tag, s.source) == ('excited', 'rule')
    assert s.confidence <= 0.5


def test_a_long_quiet_clip_is_sleepy():
    s = analysis.suggest(features(tone(400, 3.0, 0.02)), EMPTY)
    assert (s.tag, s.source) == ('sleepy', 'rule')


def test_rule_bucket_is_none_for_an_ordinary_clip():
    assert analysis.rule_bucket(features(tone(1000, 1.0, 0.3))) is None


def test_a_clip_with_nothing_to_match_is_chatter_below_the_threshold():
    s = analysis.suggest(features(tone(1000, 1.0, 0.3)), EMPTY)
    assert (s.tag, s.confidence) == ('chatter', 0.0)
    assert effective_tags((), s.tag, s.confidence) == ('chatter',)


def test_a_far_neighbour_gives_a_low_confidence():
    near = feature_dict(duration_s=1.0)
    refs = analysis.build_references([(near, 'ooh'), (feature_dict(duration_s=1.1), 'hum')],
                                     [near, feature_dict(duration_s=1.1)])
    far = feature_dict(duration_s=500.0, zcr_hz=9000.0, centroid_hz=9000.0, bursts=40.0)
    assert analysis.suggest(far, refs).confidence < 0.6


# --- k nearest neighbours ------------------------------------------------------------------

def tiny_references():
    items = [(feature_dict(duration_s=1.0, zcr_hz=1000.0), 'hum'),
             (feature_dict(duration_s=1.1, zcr_hz=1020.0), 'hum'),
             (feature_dict(duration_s=3.0, zcr_hz=3000.0), 'ooh'),
             (feature_dict(duration_s=3.2, zcr_hz=3100.0), 'ooh')]
    return analysis.build_references(items)


def test_knn_picks_the_label_of_the_nearest_cluster():
    refs = tiny_references()
    hum = analysis.suggest(feature_dict(duration_s=1.05, zcr_hz=1010.0), refs)
    ooh = analysis.suggest(feature_dict(duration_s=3.1, zcr_hz=3050.0), refs)
    assert (hum.tag, hum.source) == ('hum', 'knn')
    assert ooh.tag == 'ooh'
    assert hum.confidence > 0.6 and ooh.confidence > 0.6


def test_knn_confidence_drops_when_the_neighbours_disagree():
    refs = tiny_references()
    between = analysis.suggest(feature_dict(duration_s=2.0, zcr_hz=2000.0), refs)
    clear = analysis.suggest(feature_dict(duration_s=1.05, zcr_hz=1010.0), refs)
    assert between.confidence < clear.confidence


def test_knn_with_fewer_references_than_k_uses_them_all():
    refs = analysis.build_references([(feature_dict(), 'hum')])
    assert analysis.suggest(feature_dict(), refs).tag == 'hum'


def test_a_confident_neighbour_match_beats_a_rule():
    chirp_features = features(chirp(500, 1800, 1.0, 0.5))
    refs = analysis.build_references([(chirp_features, 'ooh')] * 3)
    s = analysis.suggest(chirp_features, refs)
    assert (s.tag, s.source) == ('ooh', 'knn')
    assert s.confidence >= 0.6


def test_standardisation_uses_the_population_when_given():
    a, b = feature_dict(duration_s=1.0), feature_dict(duration_s=2.0)
    alone = analysis.build_references([(a, 'x'), (b, 'y')])
    wide = analysis.build_references([(a, 'x'), (b, 'y')],
                                     [feature_dict(duration_s=float(d)) for d in range(1, 50)])
    assert (alone.std != wide.std).any()
