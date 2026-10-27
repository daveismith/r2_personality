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

"""Pure tests of the synthesiser (plan step 7, rule 9): determinism, bounds, the mood mapping."""

import math
import time

import numpy as np
import pytest
from r2_sound import synth
from r2_sound.gain import active_rms_dbfs, HOUSE_LEVEL_DBFS, PEAK_CEILING_DBFS, to_dbfs

RATE = 44100
GRID = (0.1, 0.3, 0.5, 0.7, 0.9)
SEEDS = range(1, 9)


def zero_crossing_rate(x, rate):
    """Return the zero crossings per second over the sounding samples: a pitch proxy."""
    x = x[np.abs(x) > 1e-4]
    return np.count_nonzero(np.signbit(x[1:]) != np.signbit(x[:-1])) * rate / max(1, len(x))


def mean_over_seeds(fn):
    return float(np.mean([fn(seed) for seed in SEEDS]))


def test_a_seed_replays_bit_for_bit_and_seeds_differ():
    a, info_a = synth.render(0.6, 0.4, 1.5, 1234)
    b, info_b = synth.render(0.6, 0.4, 1.5, 1234)
    assert a.dtype == np.float32 and a.ndim == 1
    assert a.tobytes() == b.tobytes() and info_a == info_b
    c, _ = synth.render(0.6, 0.4, 1.5, 1235)
    assert c.tobytes() != a.tobytes()
    assert synth.render(0.6, 0.4, 1.5, -7)[0].tobytes() == synth.render(0.6, 0.4, 1.5, -7)[
        0].tobytes()


@pytest.mark.parametrize('arousal', GRID)
@pytest.mark.parametrize('valence', GRID)
def test_level_bounds_over_the_grid(arousal, valence):
    for seed in (1, 2, 3):
        x, info = synth.render(arousal, valence, 0.0, seed, RATE)
        peak = to_dbfs(float(np.max(np.abs(x))))
        assert peak <= PEAK_CEILING_DBFS + 1e-6
        assert abs(active_rms_dbfs(x) - HOUSE_LEVEL_DBFS) <= 1.0
        assert info.peak_dbfs == pytest.approx(peak) and np.all(np.isfinite(x))
        assert 2 <= len(info.phrase.syllables) <= 8


@pytest.mark.parametrize('length_s', [0.5, 1.0, 2.0, 3.5, 6.0])
def test_a_requested_length_is_met(length_s):
    for seed in (1, 2, 3):
        for arousal in (0.1, 0.9):
            x, info = synth.render(arousal, 0.5, length_s, seed, RATE)
            assert abs(len(x) / RATE - length_s) <= 0.1 * length_s
            assert info.phrase.requested_s == length_s


def test_an_unrequested_length_is_the_synths_own():
    for arousal in GRID:
        for seed in SEEDS:
            x, info = synth.render(arousal, 0.5, 0.0, seed, RATE)
            lo, hi = synth.FREE_LENGTH_S
            assert lo - 1e-3 <= len(x) / RATE <= hi + 1e-3
            assert info.phrase.requested_s == 0.0


def test_valence_raises_the_pitch():
    rates = [mean_over_seeds(lambda s: zero_crossing_rate(
        synth.render(0.5, v, 1.5, s, RATE)[0], RATE)) for v in GRID]
    assert all(b > a for a, b in zip(rates, rates[1:])), rates


def test_arousal_quickens_the_phrase():
    def sps(arousal, seed):
        phrase = synth.compose(arousal, 0.5, 2.0, seed)
        return len(phrase.syllables) / phrase.length_s

    def gap_share(arousal, seed):
        phrase = synth.compose(arousal, 0.5, 2.0, seed)
        sounding = sum(s.duration_s for s in phrase.syllables)
        return 1.0 - sounding / phrase.length_s

    def free_length(arousal, seed):
        return synth.compose(arousal, 0.5, 0.0, seed).length_s

    rates = [mean_over_seeds(lambda s: sps(a, s)) for a in GRID]
    assert all(b >= a for a, b in zip(rates, rates[1:])) and rates[-1] > rates[0], rates
    assert mean_over_seeds(lambda s: gap_share(0.9, s)) < mean_over_seeds(
        lambda s: gap_share(0.1, s))
    lengths = [mean_over_seeds(lambda s: free_length(a, s)) for a in GRID]
    assert all(b < a for a, b in zip(lengths, lengths[1:])), lengths


def test_vibrato_follows_the_mood():
    assert synth.mood_of(0.9, 0.5).vibrato_st > synth.mood_of(0.1, 0.5).vibrato_st
    assert synth.mood_of(0.5, 0.1).vibrato_st > synth.mood_of(0.5, 0.9).vibrato_st


def test_the_mood_shifts_the_syllable_distribution():
    kinds = list(synth.KINDS)
    happy, sad = synth.weights(0.5, 0.9), synth.weights(0.5, 0.1)
    assert happy[kinds.index('chirp_up')] > sad[kinds.index('chirp_up')]
    assert sad[kinds.index('buzz')] > happy[kinds.index('buzz')]
    assert sad[kinds.index('sigh')] > happy[kinds.index('sigh')]
    excited, calm = synth.weights(0.9, 0.5), synth.weights(0.1, 0.5)
    assert excited[kinds.index('trill')] > calm[kinds.index('trill')]
    assert excited[kinds.index('beep')] > calm[kinds.index('beep')]
    assert calm[kinds.index('whistle')] > excited[kinds.index('whistle')]
    assert all(np.isclose(w.sum(), 1.0) and np.all(w >= 0) for w in (happy, sad, excited, calm))
    assert excited[kinds.index('sigh')] == 0.0 and happy[kinds.index('buzz')] == 0.0
    contours = list(synth.CONTOURS)
    low, high = synth.contour_weights(0.1, 0.1), synth.contour_weights(0.5, 0.9)
    assert low[contours.index('sigh')] > high[contours.index('sigh')]
    assert high[contours.index('rise')] > low[contours.index('rise')]


# --- v2: the composer (David's notes on the v1 grid, 2026-10-26) ------------------------------

def every_phrase(length_s=0.0):
    for arousal in GRID:
        for valence in GRID:
            for seed in SEEDS:
                yield synth.compose(arousal, valence, length_s, seed)


def test_a_phrase_opens_on_a_chirp_a_whistle_or_a_sigh():
    for phrase in every_phrase():
        assert phrase.kinds[0] in synth.OPENERS, phrase.kinds
    assert 'beep' not in synth.OPENERS and 'buzz' not in synth.OPENERS


def test_syllables_come_in_groups_with_pauses_between():
    pauses, inner = [], []
    for phrase in every_phrase():
        assert sum(phrase.groups) == len(phrase.syllables) and all(g >= 1 for g in phrase.groups)
        assert max(phrase.groups) <= phrase.mood.group_max
        gaps = [b - a - s.duration_s
                for a, b, s in zip(phrase.onsets_s, phrase.onsets_s[1:], phrase.syllables)]
        edge, i = set(), 0
        for size in phrase.groups[:-1]:
            i += size
            edge.add(i - 1)
        pauses += [g for k, g in enumerate(gaps) if k in edge]
        inner += [g for k, g in enumerate(gaps) if k not in edge]
    assert pauses and inner
    assert np.median(pauses) > 2.5 * np.median(inner), (np.median(pauses), np.median(inner))


def test_the_contour_is_centred_and_followed():
    for name, shape in synth.CONTOURS.items():
        u = np.linspace(0.0, 1.0, 10001)
        assert abs(float(np.mean(shape(u)))) < 0.01, name
    rising = [p for p in every_phrase() if p.contour == 'rise' and len(p.syllables) >= 4]
    assert rising
    firsts, lasts = [], []
    for p in rising:
        pitch = [math.sqrt(s.f_start * s.f_end) for s in p.syllables]
        firsts.append(np.mean(pitch[:2]))
        lasts.append(np.mean(pitch[-2:]))
    assert np.median(np.log2(np.array(lasts) / np.array(firsts))) > 0.2    # over 2 semitones


def test_the_pitch_floor_holds_in_every_mood():
    for phrase in every_phrase():
        for s in phrase.syllables:
            assert math.sqrt(s.f_start * s.f_end) >= synth.PITCH_FLOOR_HZ * 0.6, (phrase.mood, s)
            assert max(s.f_start, s.f_end) <= synth.PITCH_CEILING_HZ * 1.6


def spectral_centroid(x, rate):
    """Return the median amplitude-weighted centroid (Hz) of the sounding 2048-frames."""
    n, freqs, w = 2048, np.fft.rfftfreq(2048, 1.0 / rate), np.hanning(2048)
    frames = [x[i:i + n] for i in range(0, len(x) - n, 512)]
    rms = np.array([np.sqrt(np.mean(f ** 2)) for f in frames])
    out = []
    for f, a in zip(frames, rms):
        if a < rms.max() * 10 ** (-30 / 20):
            continue
        spec = np.abs(np.fft.rfft(f * w))
        spec[freqs < 150] = 0.0
        out.append(float((spec * freqs).sum() / spec.sum()))
    return float(np.median(out))


@pytest.mark.parametrize('arousal, valence', [(0.1, 0.1), (0.5, 0.1), (0.9, 0.1), (0.5, 0.5)])
def test_every_mood_keeps_its_brightness(arousal, valence):
    # R2 never hums low: even the sad corner keeps its centroid in the kilohertz (the sad
    # reference clips sit at 3.7-5 kHz; v1's sad corner was 400 Hz).
    centroids = [spectral_centroid(synth.render(arousal, valence, 0.0, seed, 22050)[0]
                                   .astype(float), 22050) for seed in SEEDS]
    assert np.mean(centroids) > 1200.0, centroids


def test_sad_phrases_are_few_long_syllables_and_often_sigh():
    sad = [synth.compose(0.1, 0.1, 0.0, seed) for seed in range(1, 41)]
    excited = [synth.compose(0.9, 0.9, 0.0, seed) for seed in range(1, 41)]
    assert np.mean([len(p.syllables) for p in sad]) < np.mean([len(p.syllables) for p in excited])
    assert np.mean([np.mean([s.duration_s for s in p.syllables]) for p in sad]) > 2 * np.mean(
        [np.mean([s.duration_s for s in p.syllables]) for p in excited])
    assert sum('sigh' in p.kinds for p in sad) >= 12
    assert not any('sigh' in p.kinds for p in excited)
    assert sum(p.contour in ('sigh', 'fall') for p in sad) > sum(
        p.contour in ('sigh', 'fall') for p in excited)


@pytest.mark.parametrize('kind', synth.KINDS)
def test_every_syllable_renders(kind):
    rng = np.random.default_rng(1)
    for f0 in (200.0, 800.0, 2500.0):
        syllable = synth.BUILDERS[kind](rng, f0, synth.mood_of(0.5, 0.5))
        for rate in (22050, 44100):
            x = synth.render_syllable(syllable, rate)
            assert len(x) == int(round(syllable.duration_s * rate)) > 0
            assert np.all(np.isfinite(x)) and 0.0 < np.max(np.abs(x)) <= 1.0 + 1e-9
            assert abs(x[-1]) < 0.05 and abs(x[0]) < 0.05      # the ADSR's ends


def test_the_band_pass_passes_its_centre_and_cuts_away_from_it():
    t = np.arange(RATE) / RATE

    def gain(f):
        y = synth.resonate(np.sin(2 * np.pi * f * t), 1000.0, 4.0, RATE)[RATE // 2:]
        return np.sqrt(np.mean(y ** 2)) / np.sqrt(0.5)

    assert gain(1000.0) == pytest.approx(1.0, abs=0.1)
    assert gain(4000.0) < 0.2 and gain(250.0) < 0.2


def test_rendering_2_s_takes_under_100_ms():
    synth.render(0.5, 0.5, 2.0, 99, RATE)                   # warm up
    worst = 0.0
    for seed in range(1, 11):
        began = time.perf_counter()
        synth.render(0.9, 0.5, 2.0, seed, RATE)
        worst = max(worst, time.perf_counter() - began)
    assert worst < 0.1, worst
