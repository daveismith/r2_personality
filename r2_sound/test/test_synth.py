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
            assert 0.6 - 1e-3 <= len(x) / RATE <= 2.5 + 1e-3
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
    excited, calm = synth.weights(0.9, 0.5), synth.weights(0.1, 0.5)
    assert excited[kinds.index('trill')] > calm[kinds.index('trill')]
    assert calm[kinds.index('purr')] > excited[kinds.index('purr')]
    assert all(np.isclose(w.sum(), 1.0) and np.all(w > 0) for w in (happy, sad, excited, calm))


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
