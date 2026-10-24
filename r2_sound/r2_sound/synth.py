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
Synth v1 (phase 3 plan step 7, architecture section 7): R2 phrases from the mood, pure numpy.

The model, HCR style. A :class:`Phrase` is 2-8 syllables separated by gaps, composed by
:func:`compose` from ``(arousal, valence)``, both 0..1, and a ``seed``; :func:`render` turns it
into float32 mono samples at the house level. Everything random is drawn from one
``numpy.random.Generator`` seeded with ``seed``, in a fixed order, so a seed replays a phrase
bit for bit (at the same sample rate).

**What the mood sets** (``a`` arousal, ``v`` valence; ``lerp(lo, hi, t)``):

* *length*: ``length_s`` when asked (clamped to :data:`LENGTH_RANGE_S`; the output is that
  long to the frame), else the synth's own: ``lerp(2.5, 0.6, a)`` s, +-10 % from the seed,
  kept within :data:`FREE_LENGTH_S`. Calm phrases are long, frantic ones short.
* *syllable count*: ``length * lerp(2.5, 9, a)`` syllables a second (:data:`RATE_SPS`),
  rounded and kept within 2..8. Arousal up: more syllables a second.
* *gaps*: ``lerp(0.16, 0.03, a)`` s each (:data:`GAP_S`), x0.6-1.4 from the seed;
  *syllable durations* are each kind's natural range times ``lerp(1.25, 0.7, a)``
  (:data:`TEMPO`). Arousal up: shorter syllables and shorter gaps. The phrase is then fitted
  to the length: syllables stretch by at most 0.6-1.4x, the gaps take the rest (never under
  :data:`MIN_GAP_S`).
* *base pitch*: ``330 * 4 ** v`` Hz (two octaves, :data:`BASE_HZ`), raised up to
  :data:`AROUSAL_LIFT_ST` semitones by arousal; each syllable scatters around it by a normal
  draw of ``lerp(2, 5, a)`` semitones (:data:`PITCH_SPREAD_ST`). Valence up: higher.
* *vibrato*: ``0.1 + 0.5 a + 0.3 (1 - v)`` semitones at 5-7 Hz on every syllable
  (:func:`vibrato_st`): excitement and unease shake the voice.
* *syllable distribution*: each kind's weight is ``max(0.05, w0 + wa a + wv v)`` from
  :data:`WEIGHTS`. Happy: chirps up, whistles, warbles; unhappy: chirps down, buzzes;
  excited: trills, buzzes; calm: purrs, whistles.

**A syllable** (:class:`Syllable`) is one oscillator: a sine, or 2-operator FM (``fm_ratio``,
``fm_index`` falling by ``fm_decay`` over the syllable), whose frequency follows a pitch
envelope (``f_start`` to ``f_end``, exponential in frequency with shape ``curve``, plus an
``arch_st`` bump at the middle, the vibrato and a square-ish ``trill``), optionally
amplitude-pulsed (``am_hz``, ``am_depth``: the purr), through a resonant band-pass at
``centre_ratio`` times the syllable's mean pitch with quality ``q``, mixed ``wet`` with the dry
tone, shaped by an ADSR (``attack_s``, ``decay_s``, ``sustain``, ``release_s``) and scaled to
peak ``amplitude``. The band-pass is a complex one-pole resonator ``y[n] = p y[n-1] + x[n]``
with ``p = r e^(j w)`` (its real part is a two-pole band-pass), computed in numpy blocks
with the state carried from block to block within the syllable.

**The inventory** (:data:`BUILDERS`): ``beep`` (short, flat), ``chirp_up``/``chirp_down``
(an exponential glide of 6-13 semitones), ``warble`` (a fast deep vibrato), ``trill`` (a
square pitch alternation at 18-28 Hz), ``whistle`` (long, pure, an arch), ``buzz`` (low FM
with a high index: the raspberry), ``purr`` (low, mild FM, pulsed at 22-32 Hz).

**The level**: the phrase is scaled to :data:`~r2_sound.gain.HOUSE_LEVEL_DBFS` RMS (measured
as the catalogue measures clips, :func:`~r2_sound.gain.active_rms_dbfs`), with a soft limiter
above 0.6 of the ceiling, until the peak is at most
:data:`~r2_sound.gain.PEAK_CEILING_DBFS`; the last step is always the limiter, so the
ceiling holds and the RMS lands within a fraction of a dB.

**Adding a syllable**: write ``_build_<name>(rng, f0, mood) -> Syllable`` (draw from ``rng``
only; durations are natural, before the tempo), add it to :data:`BUILDERS` and its
``(w0, wa, wv)`` row to :data:`WEIGHTS`. Adding one changes every seed's phrase (the draws
move), so tell David before his notes refer to seeds.
"""

from dataclasses import dataclass, replace
import math
from typing import Tuple

import numpy as np
from r2_sound.gain import (active_rms_dbfs, db_to_linear, HOUSE_LEVEL_DBFS, measure,
                           PEAK_CEILING_DBFS)

DEFAULT_RATE = 44100
SYLLABLES_RANGE = (2, 8)
FREE_LENGTH_S = (0.6, 2.5)       # the synth's own length, from arousal
LENGTH_RANGE_S = (0.4, 8.0)      # a requested length is clamped to this
BASE_HZ = (330.0, 1320.0)        # valence 0 .. 1
AROUSAL_LIFT_ST = 3.0            # arousal 1 raises the base this many semitones
RATE_SPS = (2.5, 9.0)            # syllables per second, arousal 0 .. 1
GAP_S = (0.16, 0.03)             # natural gap, arousal 0 .. 1
TEMPO = (1.25, 0.7)              # natural syllable duration factor, arousal 0 .. 1
PITCH_SPREAD_ST = (2.0, 5.0)     # syllable pitch scatter (sd), arousal 0 .. 1
STRETCH = (0.6, 1.4)             # how far syllables stretch to fit the length
MIN_GAP_S = 0.015
FREQ_RANGE_HZ = (100.0, 6000.0)  # every instantaneous frequency is kept within this
LIMIT_KNEE = 0.6                 # the soft limiter starts at this fraction of the ceiling
LEVEL_PASSES = 4
_TWO_PI = 2.0 * math.pi


@dataclass(frozen=True)
class Syllable:
    """One syllable's parameters (the module docstring); times in s, pitches in Hz."""

    kind: str
    duration_s: float
    f_start: float
    f_end: float
    curve: float = 1.0           # glide shape: u ** curve, u the syllable's 0..1 time
    arch_st: float = 0.0         # semitones added at the middle (a half sine)
    vibrato_hz: float = 6.0
    vibrato_st: float = 0.0
    trill_hz: float = 0.0
    trill_st: float = 0.0
    fm_ratio: float = 1.0
    fm_index: float = 0.0        # 0: a plain sine
    fm_decay: float = 0.0        # the index falls by this fraction over the syllable
    am_hz: float = 0.0
    am_depth: float = 0.0
    attack_s: float = 0.008
    decay_s: float = 0.03
    sustain: float = 0.8
    release_s: float = 0.03
    centre_ratio: float = 1.0
    q: float = 3.0
    wet: float = 0.6
    amplitude: float = 1.0


@dataclass(frozen=True)
class Mood:
    """The phrase-wide parameters :func:`compose` derived from ``(arousal, valence)``."""

    arousal: float
    valence: float
    base_hz: float
    vibrato_st: float
    tempo: float
    gap_s: float
    spread_st: float
    rate_sps: float


@dataclass(frozen=True)
class Phrase:
    """A composed phrase: its syllables, where each starts, and its length."""

    mood: Mood
    seed: int
    length_s: float
    requested_s: float               # 0: the synth chose
    syllables: Tuple[Syllable, ...]
    onsets_s: Tuple[float, ...]

    @property
    def kinds(self):
        """Return the syllable kinds in order."""
        return tuple(s.kind for s in self.syllables)

    def render(self, rate=DEFAULT_RATE):
        """Return the raw samples (float64, not levelled), exactly ``length_s`` long."""
        total = int(round(self.length_s * rate))
        out = np.zeros(total)
        for syllable, onset in zip(self.syllables, self.onsets_s):
            start = min(total, int(round(onset * rate)))
            frames = min(total - start, int(round(syllable.duration_s * rate)))
            if frames > 0:
                out[start:start + frames] += render_syllable(syllable, rate, frames)
        return out


@dataclass(frozen=True)
class PhraseInfo:
    """What :func:`render` made: the phrase and its measured level."""

    phrase: Phrase
    rms_dbfs: float
    peak_dbfs: float

    @property
    def seed(self):
        return self.phrase.seed

    @property
    def length_s(self):
        return self.phrase.length_s


def lerp(lo_hi, t):
    """Return ``lo + (hi - lo) t`` for a ``(lo, hi)`` pair."""
    lo, hi = lo_hi
    return lo + (hi - lo) * t


def clamp(value, lo_hi):
    """Return ``value`` within a ``(lo, hi)`` pair."""
    return min(lo_hi[1], max(lo_hi[0], value))


def unit(value):
    """Return ``value`` within 0..1 (NaN is 0.5)."""
    value = float(value)
    return 0.5 if math.isnan(value) else clamp(value, (0.0, 1.0))


def vibrato_st(arousal, valence):
    """Return the mood's vibrato depth in semitones."""
    return 0.1 + 0.5 * arousal + 0.3 * (1.0 - valence)


def mood_of(arousal, valence):
    """Return the :class:`Mood` of ``(arousal, valence)`` in 0..1."""
    a, v = unit(arousal), unit(valence)
    base = BASE_HZ[0] * (BASE_HZ[1] / BASE_HZ[0]) ** v * 2.0 ** (AROUSAL_LIFT_ST * a / 12.0)
    return Mood(arousal=a, valence=v, base_hz=base, vibrato_st=vibrato_st(a, v),
                tempo=lerp(TEMPO, a), gap_s=lerp(GAP_S, a), spread_st=lerp(PITCH_SPREAD_ST, a),
                rate_sps=lerp(RATE_SPS, a))


def _st(semitones):
    return 2.0 ** (semitones / 12.0)


# --- the inventory ----------------------------------------------------------------------------
# Each builder draws only from ``rng``; durations are natural (before the mood's tempo).

def _build_beep(rng, f0, mood):
    return Syllable('beep', rng.uniform(0.06, 0.14), f0, f0, attack_s=0.005, decay_s=0.02,
                    sustain=0.8, release_s=0.025, q=4.0, wet=0.5,
                    vibrato_st=mood.vibrato_st * 0.5, amplitude=rng.uniform(0.8, 1.0))


def _build_chirp_up(rng, f0, mood):
    return Syllable('chirp_up', rng.uniform(0.08, 0.2), f0 * _st(-rng.uniform(3, 6)),
                    f0 * _st(rng.uniform(3, 7)), curve=rng.uniform(1.2, 2.0),
                    vibrato_st=mood.vibrato_st, release_s=0.03, q=2.0,
                    amplitude=rng.uniform(0.8, 1.0))


def _build_chirp_down(rng, f0, mood):
    return Syllable('chirp_down', rng.uniform(0.08, 0.2), f0 * _st(rng.uniform(3, 7)),
                    f0 * _st(-rng.uniform(3, 6)), curve=rng.uniform(0.5, 0.8),
                    vibrato_st=mood.vibrato_st, release_s=0.03, q=2.0,
                    amplitude=rng.uniform(0.8, 1.0))


def _build_warble(rng, f0, mood):
    return Syllable('warble', rng.uniform(0.16, 0.34), f0, f0 * _st(rng.uniform(-2, 2)),
                    vibrato_hz=rng.uniform(9, 14),
                    vibrato_st=rng.uniform(1.5, 3.0) + mood.vibrato_st,
                    attack_s=0.015, release_s=0.05, q=2.5, amplitude=rng.uniform(0.75, 0.95))


def _build_trill(rng, f0, mood):
    return Syllable('trill', rng.uniform(0.15, 0.3), f0, f0 * _st(rng.uniform(-1, 3)),
                    trill_hz=rng.uniform(18, 28), trill_st=rng.uniform(3, 5),
                    vibrato_st=mood.vibrato_st * 0.5, attack_s=0.01, release_s=0.04,
                    q=2.0, amplitude=rng.uniform(0.75, 0.95))


def _build_whistle(rng, f0, mood):
    arch = rng.uniform(1, 3) * (1.0 if rng.random() < 0.5 else -1.0)
    return Syllable('whistle', rng.uniform(0.22, 0.5), f0, f0 * _st(rng.uniform(-2, 2)),
                    arch_st=arch, vibrato_hz=rng.uniform(5, 6.5), vibrato_st=mood.vibrato_st,
                    attack_s=0.03, decay_s=0.05, sustain=0.9, release_s=0.08, q=5.0,
                    wet=0.4, amplitude=rng.uniform(0.7, 0.9))


def _build_buzz(rng, f0, mood):
    f = f0 * rng.uniform(0.35, 0.5)
    return Syllable('buzz', rng.uniform(0.1, 0.24), f, f * _st(rng.uniform(-3, 1)),
                    fm_ratio=1.0 if rng.random() < 0.6 else 0.5,
                    fm_index=rng.uniform(2.5, 5.0), fm_decay=0.3,
                    vibrato_st=mood.vibrato_st, attack_s=0.006, release_s=0.03,
                    centre_ratio=2.0, q=1.5, wet=0.7, amplitude=rng.uniform(0.7, 0.9))


def _build_purr(rng, f0, mood):
    f = f0 * rng.uniform(0.25, 0.4)
    return Syllable('purr', rng.uniform(0.2, 0.4), f, f * _st(-rng.uniform(0, 3)),
                    fm_ratio=2.0, fm_index=rng.uniform(0.8, 1.5),
                    am_hz=rng.uniform(22, 32), am_depth=0.7, vibrato_st=mood.vibrato_st,
                    attack_s=0.03, release_s=0.08, centre_ratio=1.5, q=1.2, wet=0.6,
                    amplitude=rng.uniform(0.75, 0.95))


BUILDERS = {
    'beep': _build_beep,
    'chirp_up': _build_chirp_up,
    'chirp_down': _build_chirp_down,
    'warble': _build_warble,
    'trill': _build_trill,
    'whistle': _build_whistle,
    'buzz': _build_buzz,
    'purr': _build_purr,
}
# kind: (w0, wa, wv); the weight is max(MIN_WEIGHT, w0 + wa * arousal + wv * valence).
WEIGHTS = {
    'beep': (1.0, 0.0, 0.0),
    'chirp_up': (0.3, 0.3, 1.2),
    'chirp_down': (1.0, 0.2, -0.8),
    'warble': (0.2, 0.6, 0.6),
    'trill': (0.1, 1.4, 0.2),
    'whistle': (0.3, -0.4, 1.0),
    'buzz': (0.5, 0.8, -1.2),
    'purr': (0.8, -0.8, 0.0),
}
MIN_WEIGHT = 0.05
KINDS = tuple(BUILDERS)


def weights(arousal, valence):
    """Return the syllable kinds' probabilities (in :data:`KINDS` order) for the mood."""
    w = np.array([max(MIN_WEIGHT, w0 + wa * arousal + wv * valence)
                  for w0, wa, wv in (WEIGHTS[k] for k in KINDS)])
    return w / w.sum()


# --- composing --------------------------------------------------------------------------------

def compose(arousal, valence, length_s=0.0, seed=1):
    """Return the :class:`Phrase` of the mood, ``length_s`` long (0: the synth's choice)."""
    seed = int(seed)
    rng = np.random.default_rng(seed % 2 ** 32)
    mood = mood_of(arousal, valence)
    a = mood.arousal
    requested = float(length_s) if math.isfinite(length_s) and length_s > 0 else 0.0
    if requested:
        length = clamp(requested, LENGTH_RANGE_S)
    else:
        length = lerp((FREE_LENGTH_S[1], FREE_LENGTH_S[0]), a) * rng.uniform(0.9, 1.1)
        length = clamp(length, FREE_LENGTH_S)
    count = int(clamp(round(length * mood.rate_sps), SYLLABLES_RANGE))
    kinds = rng.choice(len(KINDS), size=count, p=weights(a, mood.valence))
    syllables = []
    for k in kinds:
        f0 = mood.base_hz * _st(rng.normal(0.0, mood.spread_st))
        s = BUILDERS[KINDS[k]](rng, float(f0), mood)
        syllables.append(replace(s, duration_s=s.duration_s * mood.tempo))
    gaps = mood.gap_s * rng.uniform(0.6, 1.4, size=count - 1)
    syllables, gaps = _fit(syllables, gaps, length)
    onsets, t = [], 0.0
    for s, gap in zip(syllables, list(gaps) + [0.0]):
        onsets.append(t)
        t += s.duration_s + gap
    return Phrase(mood, seed, length, requested, tuple(syllables), tuple(onsets))


def _fit(syllables, gaps, length):
    """Stretch the syllables (within :data:`STRETCH`) and the gaps to fill ``length``."""
    sounding = sum(s.duration_s for s in syllables)
    natural = sounding + float(gaps.sum())
    stretch = clamp(length / natural, STRETCH)
    floor = MIN_GAP_S * len(gaps)
    if length - stretch * sounding < floor:
        stretch = (length - floor) / sounding
    room = length - stretch * sounding
    gaps = gaps * (room / gaps.sum())
    return [_stretched(s, stretch) for s in syllables], gaps


def _stretched(s, factor):
    """Return ``s`` lasting ``factor`` times as long, its ADSR fitted within it."""
    d = s.duration_s * factor
    attack = min(s.attack_s, 0.3 * d)
    release = min(s.release_s, 0.4 * d)
    decay = min(s.decay_s, max(0.0, d - attack - release))
    return replace(s, duration_s=d, attack_s=attack, release_s=release, decay_s=decay)


# --- rendering --------------------------------------------------------------------------------

def render_syllable(s, rate, frames=None):
    """Return the syllable's samples (float64), ``frames`` long (default its duration)."""
    n = int(round(s.duration_s * rate)) if frames is None else int(frames)
    if n <= 0:
        return np.zeros(0)
    t = np.arange(n) / rate
    u = t / max(s.duration_s, 1e-9)
    semis = s.arch_st * np.sin(np.pi * np.minimum(u, 1.0))
    if s.vibrato_st:
        semis = semis + s.vibrato_st * np.sin(_TWO_PI * s.vibrato_hz * t)
    if s.trill_st:
        semis = semis + s.trill_st * np.tanh(4.0 * np.sin(_TWO_PI * s.trill_hz * t))
    freq = s.f_start * (s.f_end / s.f_start) ** (np.minimum(u, 1.0) ** s.curve) * 2.0 ** (
        semis / 12.0)
    np.clip(freq, FREQ_RANGE_HZ[0], min(FREQ_RANGE_HZ[1], 0.45 * rate), out=freq)
    phase = np.cumsum(freq) * (_TWO_PI / rate)
    if s.fm_index:
        index = s.fm_index * (1.0 - s.fm_decay * u)
        x = np.sin(phase + index * np.sin(s.fm_ratio * phase))
    else:
        x = np.sin(phase)
    if s.am_depth:
        x *= 1.0 - s.am_depth * (0.5 - 0.5 * np.cos(_TWO_PI * s.am_hz * t))
    if s.wet:
        centre = math.sqrt(s.f_start * s.f_end) * s.centre_ratio
        centre = min(max(centre, FREQ_RANGE_HZ[0]), 0.45 * rate)
        x = s.wet * resonate(x, centre, s.q, rate) + (1.0 - s.wet) * x
    x *= adsr(s, t)
    peak = float(np.max(np.abs(x)))
    if peak > 0.0:
        x *= s.amplitude / peak
    return x


def adsr(s, t):
    """Return the syllable's ADSR gain at times ``t`` (linear segments; times as fitted)."""
    d, a, dec, r = s.duration_s, s.attack_s, s.decay_s, s.release_s
    xs = [0.0, a, a + dec, max(a + dec, d - r), d]
    return np.interp(t, xs, [0.0, 1.0, s.sustain, s.sustain, 0.0])


def resonate(x, centre_hz, q, rate):
    """
    Return ``x`` through a resonant band-pass at ``centre_hz`` with quality ``q``.

    A complex one-pole ``y[n] = p y[n-1] + x[n]``, ``p = r e^(j w)``, ``r = exp(-pi B / rate)``
    with ``B = centre / q``; the result is the real part, scaled to unit gain at the centre.
    In blocks: within one, ``y = p^k (y0 + cumsum(x p^-k))``; the block is short enough that
    ``|p|^-k`` stays under e^200, and the state carries over to the next block.
    """
    r = math.exp(-math.pi * centre_hz / q / rate)
    w = _TWO_PI * centre_hz / rate
    p = r * complex(math.cos(w), math.sin(w))
    block = int(min(2048, max(16, 200.0 / -math.log(r))))
    powers = p ** np.arange(1, block + 1)
    inverse = 1.0 / powers
    y = np.empty(len(x), dtype=np.complex128)
    state = 0j
    for start in range(0, len(x), block):
        chunk = x[start:start + block]
        m = len(chunk)
        out = powers[:m] * (state + np.cumsum(chunk * inverse[:m]))
        y[start:start + m] = out
        state = out[-1]
    return y.real * (2.0 * (1.0 - r))


def soft_limit(x, ceiling):
    """Return ``x`` with every sample over ``LIMIT_KNEE * ceiling`` bent below ``ceiling``."""
    knee = LIMIT_KNEE * ceiling
    magnitude = np.abs(x)
    over = magnitude > knee
    if not over.any():
        return x
    span = ceiling - knee
    y = x.copy()
    y[over] = np.sign(x[over]) * (knee + span * np.tanh((magnitude[over] - knee) / span))
    return y


def level(x, house_dbfs=HOUSE_LEVEL_DBFS, ceiling_dbfs=PEAK_CEILING_DBFS):
    """Return ``x`` at ``house_dbfs`` RMS (active, as for clips), its peak under the ceiling."""
    ceiling = db_to_linear(ceiling_dbfs)
    rms = active_rms_dbfs(x)
    for _ in range(LEVEL_PASSES):
        if rms <= -100.0:
            break
        x = soft_limit(x * db_to_linear(house_dbfs - rms), ceiling)    # always last
        rms = active_rms_dbfs(x)
        if abs(rms - house_dbfs) < 0.1:
            break
    return x


def render(arousal, valence, length_s=0.0, seed=1, sample_rate=DEFAULT_RATE):
    """
    Return ``(samples, info)``: the phrase of the mood as float32 mono at ``sample_rate``.

    ``arousal`` and ``valence`` are 0..1 (clamped); ``length_s`` 0 lets the synth choose from
    the arousal (0.6-2.5 s), otherwise the phrase is that long (within :data:`LENGTH_RANGE_S`);
    the same ``seed`` and rate give the same samples. ``info`` is a :class:`PhraseInfo`.
    """
    phrase = compose(arousal, valence, length_s, seed)
    rate = int(sample_rate)
    x = level(phrase.render(rate)).astype(np.float32)
    measured = measure(x, rate)
    return x, PhraseInfo(phrase, measured.rms_dbfs, measured.peak_dbfs)


__all__ = ['BUILDERS', 'KINDS', 'Mood', 'Phrase', 'PhraseInfo', 'Syllable', 'compose',
           'mood_of', 'render', 'render_syllable', 'resonate', 'weights']
