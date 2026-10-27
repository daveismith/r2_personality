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
Synth v2 (phase 3 plan step 7, architecture section 7): R2 phrases from the mood, pure numpy.

The model, HCR style. A :class:`Phrase` is 2-8 syllables in groups, composed by
:func:`compose` from ``(arousal, valence)``, both 0..1, and a ``seed``; :func:`render` turns it
into float32 mono samples at the house level. Everything random is drawn from one
``numpy.random.Generator`` seeded with ``seed``, in a fixed order, so a seed replays a phrase
bit for bit (at the same sample rate). v2 (2026-10-27) rebuilt the composer on David's
judging of the v1 grid and on the reference clips (``mp3-trigger`` ``sad-*``/``happy-*``,
Padawan ``SENT*``): a phrase contour instead of random pitch jumps, syllables bunched into
groups with pauses between instead of an even pace, a chirp, whistle or sigh to open, a
pitch floor and harmonics in every mood (R2 sighs when sad, never hums low), the purr gone.

**What the mood sets** (``a`` arousal, ``v`` valence; ``lerp(lo, hi, t)``):

* *length*: ``length_s`` when asked (clamped to :data:`LENGTH_RANGE_S`; the output is that
  long to the frame), else the synth's own: ``lerp(2.6, 0.7, a)`` s, +-10 % from the seed,
  kept within :data:`FREE_LENGTH_S`. Calm phrases are long, frantic ones short.
* *syllables*: drawn one by one until their natural lengths and gaps fill the phrase
  (2..8, :data:`SYLLABLES_RANGE`), so a sad phrase is two or three long syllables and an
  excited one six short ones. Each kind's natural duration is scaled by ``lerp(1.25, 0.7, a)``
  (:data:`TEMPO`); the phrase is then fitted to the length (syllables stretch by at most
  0.6-1.4x, the gaps take the rest, never under :data:`MIN_GAP_S`).
* *groups*: syllables come in groups of 1 to ``2 + round(2 a)`` (:data:`GROUP_SIZE`); inside
  a group the gaps are ``lerp(0.09, 0.025, a)`` s (:data:`INNER_GAP_S`), between groups
  ``lerp(0.45, 0.12, a)`` s (:data:`PAUSE_S`), each x0.6-1.4 from the seed. The reference
  chatter has two thirds of its gaps under 0.1 s and a tail to 0.5 s: bunches and pauses.
* *contour*: one of :data:`CONTOURS` (``flat``, ``rise``, ``fall``, ``arch``, ``sigh``)
  drawn by the mood (:data:`CONTOUR_WEIGHTS`: happy rises and arches, unhappy falls, low
  and slow sighs), a shape in semitones over the phrase's 0..1 time, averaging zero (the
  base pitch is the phrase's mean) with a magnitude of 5-14 semitones; every syllable's
  pitch follows it, plus a per-group register offset
  (normal, sd ``lerp(3, 5, a)`` semitones, :data:`GROUP_OFFSET_ST`: R2 changes register
  between bunches) and a small per-syllable scatter (sd ``lerp(1.5, 2.5, a)``,
  :data:`PITCH_SPREAD_ST`).
* *base pitch*: ``450 * (2400 / 450) ** v`` Hz (:data:`BASE_HZ`; the reference chatter's
  median pitch is 1.4-1.6 kHz, the sad clips' 350-430 Hz), raised up to
  :data:`AROUSAL_LIFT_ST` semitones by arousal. No syllable's mean pitch goes under
  :data:`PITCH_FLOOR_HZ` or over :data:`PITCH_CEILING_HZ`, whatever the contour says.
* *vibrato*: ``0.1 + 0.5 a + 0.3 (1 - v)`` semitones at 5-7 Hz on every syllable
  (:func:`vibrato_st`): excitement and unease shake the voice.
* *edge*: the tonal kinds' FM index is multiplied by ``1 + 0.9 a (1 - v)`` (:data:`EDGE`):
  an aroused, unhappy R2 rasps (more harmonics), a sad or a happy one stays rounder.
* *the opener*: the first syllable is a chirp, a whistle or a sigh (:data:`OPENERS`, by the
  mood's weights among them), never a beep, buzz, trill or warble.
* *syllable distribution*: each kind's weight is ``max(0, w0 + wa a + wv v)`` from
  :data:`WEIGHTS`, so a kind can be absent from a mood (no sigh in excitement). Excited is
  percussive (beeps, chirps, trills); happy-calm whistles and warbles; unhappy falls and
  sighs; annoyed (aroused, unhappy) buzzes.

**A syllable** (:class:`Syllable`) is one oscillator: a sine, or 2-operator FM (``fm_ratio``,
``fm_index`` falling by ``fm_decay`` over the syllable), whose frequency follows a pitch
envelope (``f_start`` to ``f_end``, exponential in frequency with shape ``curve``, plus an
``arch_st`` bump at the middle, the vibrato and a square-ish ``trill``), optionally
amplitude-pulsed (``am_hz``, ``am_depth``), with a breath of white noise (``noise``, from
``noise_seed``) through a resonant band-pass at ``centre_ratio`` times the syllable's mean
pitch with quality ``q``, mixed ``wet`` with the dry tone, shaped by an ADSR (``attack_s``,
``decay_s``, ``sustain``, ``release_s``) and scaled to peak ``amplitude``. Every tonal kind
carries FM at ratio 1 or 2 so it has harmonics, and its band-pass sits :data:`FORMANT_RATIO`
above the pitch, a formant on the harmonics rather than a filter on the fundamental: R2's
tone is bright, never a bare sine (the v1 sad corner's centroid was 400 Hz against the sad
clips' kilohertz). The
band-pass is a complex one-pole resonator ``y[n] = p y[n-1] + x[n]`` with ``p = r e^(j w)``
(its real part is a two-pole band-pass), computed in numpy blocks with the state carried
from block to block within the syllable.

**The inventory** (:data:`BUILDERS`): ``beep`` (short, flat), ``chirp_up``/``chirp_down``
(an exponential glide of 6-14 semitones), ``warble`` (a fast deep vibrato), ``trill`` (a
square pitch alternation at 18-28 Hz), ``whistle`` (long, pure, an arch), ``buzz`` (FM with a
high index just under the pitch: the raspberry), ``sigh`` (long, a slow fall of 4-9
semitones from a soft start, breathy, dying away: the sad clips' shape).

**The level**: the phrase is scaled to :data:`~r2_sound.gain.HOUSE_LEVEL_DBFS` RMS (measured
as the catalogue measures clips, :func:`~r2_sound.gain.active_rms_dbfs`), with a soft limiter
above 0.6 of the ceiling, until the peak is at most
:data:`~r2_sound.gain.PEAK_CEILING_DBFS`; the last step is always the limiter, so the
ceiling holds and the RMS lands within a fraction of a dB.

**Adding a syllable**: write ``_build_<name>(rng, f0, mood) -> Syllable`` (draw from ``rng``
only; durations are natural, before the tempo), add it to :data:`BUILDERS` and its
``(w0, wa, wv)`` row to :data:`WEIGHTS` (and to :data:`OPENERS` if it may open a phrase).
Adding one changes every seed's phrase (the draws move), so tell David before his notes
refer to seeds.
"""

from dataclasses import dataclass, replace
import math
from typing import Tuple

import numpy as np
from r2_sound.gain import (active_rms_dbfs, db_to_linear, HOUSE_LEVEL_DBFS, measure,
                           PEAK_CEILING_DBFS)

DEFAULT_RATE = 44100
SYLLABLES_RANGE = (2, 8)
FREE_LENGTH_S = (0.7, 2.6)       # the synth's own length, from arousal
LENGTH_RANGE_S = (0.4, 8.0)      # a requested length is clamped to this
BASE_HZ = (450.0, 2400.0)        # valence 0 .. 1
AROUSAL_LIFT_ST = 2.0            # arousal 1 raises the base this many semitones
PITCH_FLOOR_HZ = 330.0           # no syllable's mean pitch under this (R2 never hums low)
PITCH_CEILING_HZ = 3600.0        # ... or over this
GROUP_SIZE = (1, 4)              # syllables in a group: 1 .. 2 + round(2 a)
INNER_GAP_S = (0.09, 0.025)      # gap inside a group, arousal 0 .. 1
PAUSE_S = (0.45, 0.12)           # pause between groups, arousal 0 .. 1
TEMPO = (1.25, 0.7)              # natural syllable duration factor, arousal 0 .. 1
GROUP_OFFSET_ST = (3.0, 5.0)     # a group's register offset (sd), arousal 0 .. 1
PITCH_SPREAD_ST = (1.5, 2.5)     # syllable pitch scatter inside a group (sd), arousal 0 .. 1
CONTOUR_ST = (5.0, 14.0)         # a contour's magnitude is drawn from this range
STRETCH = (0.6, 1.4)             # how far syllables stretch to fit the length
MIN_GAP_S = 0.015
FREQ_RANGE_HZ = (100.0, 6000.0)  # every instantaneous frequency is kept within this
EDGE = 0.9                       # the FM index grows by this much at arousal 1, valence 0
FORMANT_RATIO = 3.0              # the band-pass sits here above the pitch: harmonics, not a sine
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
    noise: float = 0.0           # white noise mixed in before the band-pass (0: none)
    noise_seed: int = 0
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
    inner_gap_s: float
    pause_s: float
    group_max: int
    offset_st: float
    spread_st: float
    edge: float                  # multiplies the tonal kinds' FM index: a rasp when annoyed


@dataclass(frozen=True)
class Phrase:
    """A composed phrase: its syllables, where each starts, its groups, and its length."""

    mood: Mood
    seed: int
    length_s: float
    requested_s: float               # 0: the synth chose
    syllables: Tuple[Syllable, ...]
    onsets_s: Tuple[float, ...]
    contour: str = 'flat'
    groups: Tuple[int, ...] = ()     # the group sizes, in order (they sum to the count)

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
                tempo=lerp(TEMPO, a), inner_gap_s=lerp(INNER_GAP_S, a),
                pause_s=lerp(PAUSE_S, a), group_max=GROUP_SIZE[0] + 1 + int(round(2 * a)),
                offset_st=lerp(GROUP_OFFSET_ST, a), spread_st=lerp(PITCH_SPREAD_ST, a),
                edge=1.0 + EDGE * a * (1.0 - v))


def _st(semitones):
    return 2.0 ** (semitones / 12.0)


# --- the inventory ----------------------------------------------------------------------------
# Each builder draws only from ``rng``; durations are natural (before the mood's tempo).

def _build_beep(rng, f0, mood):
    return Syllable('beep', rng.uniform(0.06, 0.14), f0, f0, attack_s=0.005, decay_s=0.02,
                    sustain=0.8, release_s=0.025, fm_ratio=1.0,
                    fm_index=rng.uniform(1.0, 1.6) * mood.edge,
                    centre_ratio=FORMANT_RATIO, q=2.0, wet=0.5,
                    vibrato_st=mood.vibrato_st * 0.5, amplitude=rng.uniform(0.8, 1.0))


def _build_chirp_up(rng, f0, mood):
    return Syllable('chirp_up', rng.uniform(0.08, 0.2), f0 * _st(-rng.uniform(3, 7)),
                    f0 * _st(rng.uniform(3, 7)), curve=rng.uniform(1.2, 2.0),
                    fm_ratio=1.0, fm_index=rng.uniform(1.0, 1.6) * mood.edge,
                    vibrato_st=mood.vibrato_st,
                    release_s=0.03, centre_ratio=FORMANT_RATIO, q=1.5,
                    amplitude=rng.uniform(0.8, 1.0))


def _build_chirp_down(rng, f0, mood):
    return Syllable('chirp_down', rng.uniform(0.08, 0.2), f0 * _st(rng.uniform(3, 7)),
                    f0 * _st(-rng.uniform(3, 7)), curve=rng.uniform(0.5, 0.8),
                    fm_ratio=1.0, fm_index=rng.uniform(1.0, 1.6) * mood.edge,
                    vibrato_st=mood.vibrato_st,
                    release_s=0.03, centre_ratio=FORMANT_RATIO, q=1.5,
                    amplitude=rng.uniform(0.8, 1.0))


def _build_warble(rng, f0, mood):
    return Syllable('warble', rng.uniform(0.16, 0.34), f0, f0 * _st(rng.uniform(-2, 2)),
                    vibrato_hz=rng.uniform(9, 14),
                    vibrato_st=rng.uniform(1.5, 3.0) + mood.vibrato_st,
                    fm_ratio=2.0, fm_index=rng.uniform(0.5, 0.9) * mood.edge,
                    attack_s=0.015, release_s=0.05, centre_ratio=FORMANT_RATIO, q=1.5,
                    amplitude=rng.uniform(0.75, 0.95))


def _build_trill(rng, f0, mood):
    return Syllable('trill', rng.uniform(0.15, 0.3), f0, f0 * _st(rng.uniform(-1, 3)),
                    trill_hz=rng.uniform(18, 28), trill_st=rng.uniform(3, 5),
                    fm_ratio=1.0, fm_index=rng.uniform(0.8, 1.3) * mood.edge,
                    vibrato_st=mood.vibrato_st * 0.5, attack_s=0.01, release_s=0.04,
                    centre_ratio=FORMANT_RATIO, q=1.5, amplitude=rng.uniform(0.75, 0.95))


def _build_whistle(rng, f0, mood):
    arch = rng.uniform(1, 3) * (1.0 if rng.random() < 0.5 else -1.0)
    return Syllable('whistle', rng.uniform(0.22, 0.5), f0, f0 * _st(rng.uniform(-2, 2)),
                    arch_st=arch, vibrato_hz=rng.uniform(5, 6.5), vibrato_st=mood.vibrato_st,
                    fm_ratio=2.0, fm_index=rng.uniform(0.3, 0.6),
                    attack_s=0.03, decay_s=0.05, sustain=0.9, release_s=0.08,
                    centre_ratio=FORMANT_RATIO, q=3.0, wet=0.4, amplitude=rng.uniform(0.7, 0.9))


def _build_buzz(rng, f0, mood):
    f = max(PITCH_FLOOR_HZ, f0 * rng.uniform(0.6, 0.9))
    return Syllable('buzz', rng.uniform(0.1, 0.24), f, f * _st(rng.uniform(-3, 1)),
                    fm_ratio=1.0 if rng.random() < 0.6 else 0.5,
                    fm_index=rng.uniform(2.5, 5.0), fm_decay=0.3,
                    vibrato_st=mood.vibrato_st, attack_s=0.006, release_s=0.03,
                    centre_ratio=2.0, q=1.5, wet=0.7, amplitude=rng.uniform(0.7, 0.9))


def _build_sigh(rng, f0, mood):
    fall = rng.uniform(4, 9)
    return Syllable('sigh', rng.uniform(0.4, 0.9), f0 * _st(0.5 * fall), f0 * _st(-0.5 * fall),
                    curve=rng.uniform(0.7, 1.0), arch_st=rng.uniform(0.5, 1.5),
                    vibrato_hz=rng.uniform(4.5, 5.5), vibrato_st=mood.vibrato_st * 0.7,
                    fm_ratio=2.0, fm_index=rng.uniform(1.0, 1.6), fm_decay=0.4,
                    noise=rng.uniform(0.1, 0.2), noise_seed=int(rng.integers(1, 2 ** 31)),
                    attack_s=0.06, decay_s=0.2, sustain=0.55, release_s=0.15,
                    centre_ratio=FORMANT_RATIO, q=1.5, wet=0.5, amplitude=rng.uniform(0.7, 0.9))


BUILDERS = {
    'beep': _build_beep,
    'chirp_up': _build_chirp_up,
    'chirp_down': _build_chirp_down,
    'warble': _build_warble,
    'trill': _build_trill,
    'whistle': _build_whistle,
    'buzz': _build_buzz,
    'sigh': _build_sigh,
}
# kind: (w0, wa, wv); the weight is max(0, w0 + wa * arousal + wv * valence): a kind can be
# absent from a mood (no sigh in excitement, no trill in calm). beep and chirp_down never are.
WEIGHTS = {
    'beep': (0.3, 1.2, 0.1),
    'chirp_up': (0.15, 0.5, 1.2),
    'chirp_down': (0.8, 0.3, -0.5),
    'warble': (0.2, 0.3, 0.7),
    'trill': (-0.5, 1.8, 0.2),
    'whistle': (0.5, -0.6, 1.0),
    'buzz': (0.1, 0.8, -0.9),
    'sigh': (0.9, -1.0, -1.4),
}
KINDS = tuple(BUILDERS)
OPENERS = ('chirp_up', 'chirp_down', 'whistle', 'sigh')     # what a phrase may start with
# contour: (w0, wa, wv), as WEIGHTS; the shape is a function of the phrase's 0..1 time.
CONTOUR_WEIGHTS = {
    'flat': (0.4, 0.2, 0.0),
    'rise': (0.1, 0.3, 1.0),
    'fall': (0.35, 0.2, -0.3),
    'arch': (0.2, 0.2, 0.6),
    'sigh': (0.5, -0.6, -1.2),
}
CONTOURS = {                     # each averages 0 over 0..1, so the base pitch is the mean
    'flat': lambda u: 0.0 * u,
    'rise': lambda u: u - 0.5,
    'fall': lambda u: 0.5 - u,
    'arch': lambda u: np.sin(np.pi * u) - 2.0 / np.pi,
    'sigh': lambda u: 1.0 / 1.7 - u ** 0.7,
}


def _mood_weights(table, order, arousal, valence):
    w = np.array([max(0.0, w0 + wa * arousal + wv * valence)
                  for w0, wa, wv in (table[k] for k in order)])
    return w / w.sum()


def weights(arousal, valence):
    """Return the syllable kinds' probabilities (in :data:`KINDS` order) for the mood."""
    return _mood_weights(WEIGHTS, KINDS, arousal, valence)


def contour_weights(arousal, valence):
    """Return the contours' probabilities (in :data:`CONTOURS` order) for the mood."""
    return _mood_weights(CONTOUR_WEIGHTS, tuple(CONTOURS), arousal, valence)


# --- composing --------------------------------------------------------------------------------

def compose(arousal, valence, length_s=0.0, seed=1):
    """Return the :class:`Phrase` of the mood, ``length_s`` long (0: the synth's choice)."""
    seed = int(seed)
    rng = np.random.default_rng(seed % 2 ** 32)
    mood = mood_of(arousal, valence)
    a, v = mood.arousal, mood.valence
    requested = float(length_s) if math.isfinite(length_s) and length_s > 0 else 0.0
    if requested:
        length = clamp(requested, LENGTH_RANGE_S)
    else:
        length = lerp((FREE_LENGTH_S[1], FREE_LENGTH_S[0]), a) * rng.uniform(0.9, 1.1)
        length = clamp(length, FREE_LENGTH_S)
    names = tuple(CONTOURS)
    contour = names[rng.choice(len(names), p=contour_weights(a, v))]
    magnitude = rng.uniform(*CONTOUR_ST)
    kind_p = weights(a, v)
    opener_p = np.array([kind_p[KINDS.index(k)] for k in OPENERS])
    opener_p /= opener_p.sum()
    # Draw syllables until their natural lengths fill the phrase (the kinds, the groups and
    # the pitches first; the count falls out of the durations).
    kinds, groups, offsets = [], [], []
    scatter = []
    left = 0                                        # syllables left in the current group
    while len(kinds) < SYLLABLES_RANGE[1]:
        if left == 0:
            left = int(rng.integers(GROUP_SIZE[0], mood.group_max + 1))
            groups.append(0)
            offsets.append(float(rng.normal(0.0, mood.offset_st)))
        if not kinds:
            kinds.append(OPENERS[rng.choice(len(OPENERS), p=opener_p)])
        else:
            k = KINDS[rng.choice(len(KINDS), p=kind_p)]
            if len(kinds) >= 2 and kinds[-1] == kinds[-2] == k:      # not three in a row
                k = KINDS[rng.choice(len(KINDS), p=kind_p)]
            kinds.append(k)
        groups[-1] += 1
        left -= 1
        scatter.append(float(rng.normal(0.0, mood.spread_st)))
    # The pitch of each candidate from the contour, its group's register and its scatter.
    count_max = len(kinds)
    group_of = [g for g, size in enumerate(groups) for _ in range(size)]
    syllables, gaps, natural = [], [], 0.0
    for i, kind in enumerate(kinds):
        u = i / max(1, count_max - 1)
        semis = magnitude * float(CONTOURS[contour](u)) + offsets[group_of[i]] + scatter[i]
        f0 = clamp(mood.base_hz * _st(semis), (PITCH_FLOOR_HZ, PITCH_CEILING_HZ))
        s = BUILDERS[kind](rng, float(f0), mood)
        s = replace(s, duration_s=s.duration_s * mood.tempo)
        last_of_group = i + 1 < count_max and group_of[i + 1] != group_of[i]
        gap = (mood.pause_s if last_of_group else mood.inner_gap_s) * rng.uniform(0.6, 1.4)
        if len(syllables) >= SYLLABLES_RANGE[0] and natural + 0.5 * s.duration_s > length:
            break
        syllables.append(s)
        gaps.append(gap)
        natural += s.duration_s + gap
    gaps = np.array(gaps[:-1]) if len(gaps) > 1 else np.zeros(0)
    sizes = _group_sizes(group_of[:len(syllables)])
    syllables, gaps = _fit(syllables, gaps, length)
    onsets, t = [], 0.0
    for s, gap in zip(syllables, list(gaps) + [0.0]):
        onsets.append(t)
        t += s.duration_s + gap
    return Phrase(mood, seed, length, requested, tuple(syllables), tuple(onsets), contour,
                  sizes)


def _group_sizes(group_of):
    sizes = []
    for g in group_of:
        if sizes and g == len(sizes) - 1:
            sizes[-1] += 1
        else:
            sizes.append(1)
    return tuple(sizes)


def _fit(syllables, gaps, length):
    """Stretch the syllables (within :data:`STRETCH`) and the gaps to fill ``length``."""
    sounding = sum(s.duration_s for s in syllables)
    natural = sounding + float(gaps.sum())
    stretch = clamp(length / natural, STRETCH)
    floor = MIN_GAP_S * len(gaps)
    if length - stretch * sounding < floor:
        stretch = (length - floor) / sounding
    room = length - stretch * sounding
    if len(gaps):
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
    if s.noise:
        x = x + s.noise * np.random.default_rng(s.noise_seed).standard_normal(n)
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


__all__ = ['BUILDERS', 'CONTOURS', 'KINDS', 'Mood', 'OPENERS', 'Phrase', 'PhraseInfo',
           'Syllable', 'compose', 'contour_weights', 'mood_of', 'render', 'render_syllable',
           'resonate', 'weights']
