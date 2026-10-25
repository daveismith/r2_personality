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
Acoustic features and the first-pass bucket suggestion for the catalogue (plan step 3).

Pure numpy, no ROS imports. :func:`extract_features` reduces a decoded clip to a few
numbers (all at 22.05 kHz mono, the first :data:`MAX_ANALYSIS_S` seconds):

* ``duration_s`` (of the whole clip), ``level_dbfs`` (RMS of the non-silent part);
* the RMS envelope (512-sample frames, hop 256), normalised to its peak: ``attack_s``
  (first active frame to 90 % of the peak), ``decay_frac`` (peak to last active frame, as a
  share of the active span), ``bursts`` (syllable-like bursts: rising edges of a
  hysteresis threshold, up at 0.35 and down at 0.2 of the peak) and ``burst_rate`` (per
  second of the active span);
* ``zcr_hz`` (zero crossings per second over two, a pitch proxy) and ``zcr_slope``;
* ``centroid_hz`` (spectral centroid from a Hann-windowed FFT per frame) and
  ``centroid_slope``.

A slope is the fitted change across the active span relative to the mean (+1: the value
doubles-ish from start to end, -1: it falls to nothing), clamped to -3..3. Frames count as
active above a tenth of the peak RMS and above -50 dBFS.

:func:`suggest` is the suggestion: k nearest neighbours (k = 3) over the labelled
reference clips in standardised feature space, with a confidence from the neighbours'
agreement (distance-weighted) times a distance factor. For the buckets no reference
covers there are rules (:func:`rule_bucket`), at confidence :data:`RULE_CONFIDENCE`
(<= 0.5), used only when the neighbours are not confident themselves:

* ``excited``: at least 6 bursts at 6 or more per second with a centroid >= 2500 Hz (R2's
  chatter is bursty anyway: the median clip has 5 bursts at 3 per second, so only the
  extreme tail counts);
* ``question``: up to 1.6 s, rising pitch proxy (``zcr_slope`` >= 0.3), at most 2 bursts;
* ``sad``: at least 1.5 s, falling pitch proxy (``zcr_slope`` <= -0.3), at most 3 bursts;
* ``sleepy``: at least 2.5 s and ``level_dbfs`` <= -22.

``calm``, ``scared`` and ``greeting`` have no rule; they come from the sheet only. All of
this is a first pass to be confirmed by ear.

Duplicates: :func:`fingerprint` reduces a clip (its first :data:`MAX_ANALYSIS_S` seconds,
mono, resampled to :data:`FP_RATE`) to log band energies, :data:`FP_BANDS` log-spaced
bands from 125 Hz to 4 kHz over 50 ms Hann frames every 25 ms, in dB floored 60 dB below
the loudest band-frame. It keeps two parts: the band profile (the mean over time, centred)
and the time pattern (each band minus its mean, which also removes any gain difference).
:func:`duplicate_score` (0..1) is the lesser of the profiles' correlation and the best
normalised cross-correlation of the time patterns over a lag of up to
:data:`FP_MAX_LAG` frames (100 ms) either way; two steady sounds (no time pattern) score
their profile correlation. Two clips of the same sample count and rate whose 8 kHz
signals agree within :data:`EXACT_TOLERANCE` score 1.0. :func:`duplicate_pairs` scores
only the pairs whose durations are within 25 % or 0.3 s (:func:`durations_close`): the
music set's copies of the Padawan clips carry 5-10 % more padding, so a tight gate hid them.
"""

from dataclasses import dataclass
import math
from typing import Dict, Mapping, Optional, Sequence, Tuple

import numpy as np
from r2_sound.gain import active_rms_dbfs, SILENCE_DBFS

SAMPLE_RATE = 22050
MAX_ANALYSIS_S = 10.0
FRAME = 512
HOP = 256
ACTIVE_REL = 0.1
BURST_ON = 0.35
BURST_OFF = 0.2
SLOPE_LIMIT = 3.0
K = 3
RULE_CONFIDENCE = 0.4
RULE_YIELDS_TO = 0.6           # a neighbour match at least this confident beats a rule
NO_MATCH_TAG = 'chatter'

FP_RATE = 8000
FP_FRAME = 400                 # 50 ms at FP_RATE
FP_HOP = 200                   # 25 ms
FP_BANDS = 16
FP_LOW_HZ = 125.0
FP_FLOOR_DB = 60.0             # band-frames are floored this far below the loudest
FP_MAX_LAG = 4                 # frames either way (100 ms)
FP_STEADY_DB = 0.5             # a time pattern with an RMS below this is a steady sound
EXACT_TOLERANCE = 2e-3         # max |difference| of the 8 kHz signals for an exact match
DURATION_REL = 0.25            # pairs are scored only when their durations are this close
DURATION_ABS_S = 0.3           # ... or within this
# A suggested duplicate scores at least this. On the two real sets (2026-10-24) the pairs
# at 0.917 and above are the same recording (their aligned waveforms correlate at 0.35 to
# 0.83); the next ones (0.818 to 0.891) correlate at 0.1 to 0.34: variants, not copies.
DUPLICATE_THRESHOLD = 0.85

FEATURE_KEYS = ('duration_s', 'attack_s', 'decay_frac', 'bursts', 'burst_rate', 'zcr_hz',
                'zcr_slope', 'centroid_hz', 'centroid_slope')
_LOG_KEYS = ('duration_s', 'attack_s', 'bursts')    # compressed with log1p before standardising
DISTANCE_SCALE = math.sqrt(len(FEATURE_KEYS))       # a typical distance between random clips

Features = Mapping[str, float]


def to_mono(samples):
    """Return a float64 mono signal from (n,) or (n, channels) samples."""
    x = np.asarray(samples, dtype=np.float64)
    return x if x.ndim == 1 else x.mean(axis=1)


RESAMPLE_BLOCK = 65536               # output samples interpolated at a time


def resample(x, sample_rate, target=SAMPLE_RATE):
    """
    Return mono ``x`` at ``target`` Hz: box-filtered decimation, then linear interpolation.

    The result has the dtype of ``x`` (float32 stays float32: the player's decode) and is
    interpolated a block at a time, so no float64 copy of the whole output is ever made.
    """
    if sample_rate == target or len(x) == 0:
        return x
    x = np.asarray(x)
    dtype = x.dtype if np.issubdtype(x.dtype, np.floating) else np.float64
    if sample_rate > target:
        width = int(math.ceil(sample_rate / target))
        x = np.convolve(x, np.full(width, 1.0 / width, dtype=dtype), mode='same')
    n_out = int(round(len(x) * target / sample_rate))
    step = sample_rate / target
    source = np.arange(len(x))
    out = np.empty(n_out, dtype=dtype)
    for start in range(0, n_out, RESAMPLE_BLOCK):
        positions = np.arange(start, min(n_out, start + RESAMPLE_BLOCK)) * step
        out[start:start + len(positions)] = np.interp(positions, source, x)
    return out


def _frames(x):
    if len(x) < FRAME:
        x = np.pad(x, (0, FRAME - len(x)))
    return np.lib.stride_tricks.sliding_window_view(x, FRAME)[::HOP]


def _slope(values, active):
    """Fitted change across the active frames relative to their mean, clamped."""
    t = np.flatnonzero(active).astype(np.float64)
    if len(t) < 3:
        return 0.0
    y = values[active]
    mean = float(np.mean(y))
    span = t[-1] - t[0]
    if mean <= 0.0 or span <= 0.0:
        return 0.0
    slope = np.polyfit(t - t[0], y, 1)[0]
    return float(np.clip(slope * span / mean, -SLOPE_LIMIT, SLOPE_LIMIT))


def _count_bursts(envelope):
    count, inside = 0, False
    for value in envelope:
        if not inside and value >= BURST_ON:
            count, inside = count + 1, True
        elif inside and value < BURST_OFF:
            inside = False
    return count


def extract_features(samples, sample_rate) -> Dict[str, float]:
    """Return the feature dict of one decoded clip; see the module docstring."""
    full = to_mono(samples)
    duration = len(full) / float(sample_rate)
    x = resample(full[:int(MAX_ANALYSIS_S * sample_rate)], sample_rate)
    features = {key: 0.0 for key in FEATURE_KEYS}
    features['duration_s'] = duration
    features['level_dbfs'] = active_rms_dbfs(x)
    frames = _frames(x)
    rms = np.sqrt(np.mean(frames ** 2, axis=1))
    peak = float(rms.max())
    if peak < 10.0 ** (SILENCE_DBFS / 20.0):
        return features
    active = rms > max(ACTIVE_REL * peak, 10.0 ** (SILENCE_DBFS / 20.0))
    idx = np.flatnonzero(active)
    first, last = int(idx[0]), int(idx[-1])
    envelope = rms / peak
    peak_at = int(np.argmax(envelope))
    span = last - first + 1
    hop_s = HOP / SAMPLE_RATE
    features['attack_s'] = int(np.argmax(envelope[first:] >= 0.9)) * hop_s
    features['decay_frac'] = max(0, last - peak_at) / span
    features['bursts'] = float(_count_bursts(envelope[first:last + 1]))
    features['burst_rate'] = features['bursts'] / max(span * hop_s, 0.1)
    negative = frames < 0
    crossings = np.count_nonzero(negative[:, 1:] != negative[:, :-1], axis=1)
    zcr = crossings * SAMPLE_RATE / (2.0 * FRAME)
    spectrum = np.abs(np.fft.rfft(frames * np.hanning(FRAME), axis=1))
    freqs = np.fft.rfftfreq(FRAME, 1.0 / SAMPLE_RATE)
    centroid = (spectrum @ freqs) / np.maximum(spectrum.sum(axis=1), 1e-12)
    features['zcr_hz'] = float(np.mean(zcr[active]))
    features['zcr_slope'] = _slope(zcr, active)
    features['centroid_hz'] = float(np.mean(centroid[active]))
    features['centroid_slope'] = _slope(centroid, active)
    return features


def vector(features: Features):
    """Return the raw feature array, log-compressed where the feature is skewed."""
    return np.array([math.log1p(max(features[k], 0.0)) if k in _LOG_KEYS else features[k]
                     for k in FEATURE_KEYS], dtype=np.float64)


@dataclass(frozen=True)
class References:
    """Labelled clips in standardised feature space, with the standardisation they use."""

    vectors: np.ndarray
    labels: Tuple[str, ...]
    mean: np.ndarray
    std: np.ndarray

    def standardise(self, features: Features):
        """Return ``features`` in this reference set's standardised space."""
        return (vector(features) - self.mean) / self.std


def build_references(labelled: Sequence[Tuple[Features, str]],
                     population: Optional[Sequence[Features]] = None) -> References:
    """
    Return the references from ``(features, label)`` pairs.

    The standardisation (mean and standard deviation per feature) comes from
    ``population`` (every clip being catalogued) if given, else from the references.
    """
    refs = np.array([vector(f) for f, _ in labelled]).reshape(len(labelled), len(FEATURE_KEYS))
    base = np.array([vector(f) for f in population]) if population else refs
    mean = base.mean(axis=0) if len(base) else np.zeros(len(FEATURE_KEYS))
    std = base.std(axis=0) if len(base) else np.ones(len(FEATURE_KEYS))
    std = np.where(std > 1e-9, std, 1.0)
    return References((refs - mean) / std, tuple(label for _, label in labelled), mean, std)


@dataclass(frozen=True)
class Suggestion:
    """A suggested bucket, how sure, and where it came from (``knn`` or ``rule``)."""

    tag: str
    confidence: float
    source: str


def rule_bucket(features: Features) -> Optional[str]:
    """Return the bucket a documented rule gives these features, or None."""
    f = features
    if f['bursts'] >= 6 and f['burst_rate'] >= 6.0 and f['centroid_hz'] >= 2500.0:
        return 'excited'
    if f['duration_s'] <= 1.6 and f['zcr_slope'] >= 0.3 and f['bursts'] <= 2:
        return 'question'
    if f['duration_s'] >= 1.5 and f['zcr_slope'] <= -0.3 and f['bursts'] <= 3:
        return 'sad'
    if f['duration_s'] >= 2.5 and f.get('level_dbfs', 0.0) <= -22.0:
        return 'sleepy'
    return None


def nearest(features: Features, references: References, k: int = K) -> Suggestion:
    """Return the k-nearest-neighbour bucket and a confidence in 0..1."""
    if not len(references.labels):
        return Suggestion(NO_MATCH_TAG, 0.0, 'knn')
    distances = np.linalg.norm(references.vectors - references.standardise(features), axis=1)
    order = np.argsort(distances, kind='stable')[:k]
    votes: Dict[str, float] = {}
    for i in order:
        label = references.labels[i]
        votes[label] = votes.get(label, 0.0) + 1.0 / (distances[i] + 0.5)
    total = sum(votes.values())
    # the winner is the heaviest vote; a tie goes to the nearest neighbour's label
    nearest_label = references.labels[order[0]]
    winner = max(votes, key=lambda label: (votes[label], label == nearest_label))
    near = [distances[i] for i in order if references.labels[i] == winner]
    closeness = 1.0 / (1.0 + (float(np.mean(near)) / DISTANCE_SCALE) ** 2)
    return Suggestion(winner, round(votes[winner] / total * closeness, 3), 'knn')


def suggest(features: Features, references: References, k: int = K) -> Suggestion:
    """
    Return the first-pass bucket for one unlabelled clip.

    The neighbours decide unless a rule fires and the neighbours are not confident
    (below :data:`RULE_YIELDS_TO`); a rule's confidence is :data:`RULE_CONFIDENCE`.
    """
    knn = nearest(features, references, k)
    rule = rule_bucket(features)
    if rule is not None and knn.confidence < RULE_YIELDS_TO:
        return Suggestion(rule, RULE_CONFIDENCE, 'rule')
    return knn


@dataclass(frozen=True)
class Fingerprint:
    """A clip reduced for duplicate detection; see the module docstring."""

    profile: np.ndarray          # (bands,) mean log band energy, centred, dB
    pattern: np.ndarray          # (frames, bands) each band minus its mean, dB
    signal: np.ndarray           # the mono FP_RATE signal (float32), for the exact check
    frames: int                  # sample count of the whole decoded clip
    sample_rate: int
    duration_s: float


def _band_edges():
    edges = np.geomspace(FP_LOW_HZ, FP_RATE / 2.0, FP_BANDS + 1)
    bins = np.round(edges / (FP_RATE / FP_FRAME)).astype(int)
    return np.maximum.accumulate(np.maximum(bins, 1))


def fingerprint(samples, sample_rate) -> Fingerprint:
    """Return the duplicate-detection fingerprint of one decoded clip."""
    full = np.asarray(samples)
    frames_total = int(full.shape[0])
    head = to_mono(full[:int(MAX_ANALYSIS_S * sample_rate)])
    signal = np.asarray(resample(head, sample_rate, FP_RATE), dtype=np.float32)
    x = signal.astype(np.float64)
    if len(x) < FP_FRAME:
        x = np.pad(x, (0, FP_FRAME - len(x)))
    windows = np.lib.stride_tricks.sliding_window_view(x, FP_FRAME)[::FP_HOP]
    power = np.abs(np.fft.rfft(windows * np.hanning(FP_FRAME), axis=1)) ** 2
    edges = _band_edges()
    bands = np.stack([power[:, lo:max(hi, lo + 1)].sum(axis=1)
                      for lo, hi in zip(edges[:-1], edges[1:])], axis=1)
    db = 10.0 * np.log10(np.maximum(bands, 1e-20))
    db = np.maximum(db, db.max() - FP_FLOOR_DB)
    mean = db.mean(axis=0)
    return Fingerprint(profile=mean - mean.mean(), pattern=db - mean,
                       signal=signal,
                       frames=frames_total, sample_rate=int(sample_rate),
                       duration_s=frames_total / float(sample_rate))


def _correlation(a, b):
    na, nb = float(np.sqrt(np.sum(a * a))), float(np.sqrt(np.sum(b * b)))
    if na < 1e-9 or nb < 1e-9:
        return 1.0 if na < 1e-9 and nb < 1e-9 else 0.0
    return float(np.sum(a * b)) / (na * nb)


def _steady(pattern):
    return float(np.sqrt(np.mean(pattern ** 2))) < FP_STEADY_DB


def exact_match(a: Fingerprint, b: Fingerprint) -> bool:
    """Return True when the two clips have the same samples, within the tolerance."""
    if a.frames != b.frames or a.sample_rate != b.sample_rate or len(a.signal) != len(b.signal):
        return False
    return bool(np.max(np.abs(a.signal - b.signal), initial=0.0) <= EXACT_TOLERANCE)


def duplicate_score(a: Fingerprint, b: Fingerprint) -> float:
    """Return how alike two clips sound, 0..1; 1.0 for an exact match."""
    if exact_match(a, b):
        return 1.0
    spectral = _correlation(a.profile, b.profile)
    steady_a, steady_b = _steady(a.pattern), _steady(b.pattern)
    if steady_a and steady_b:
        temporal = 1.0
    elif steady_a or steady_b:
        temporal = 0.0
    else:
        temporal = -1.0
        for lag in range(-FP_MAX_LAG, FP_MAX_LAG + 1):
            pa = a.pattern[max(0, lag):]
            pb = b.pattern[max(0, -lag):]
            n = min(len(pa), len(pb))
            if n >= 2:
                temporal = max(temporal, _correlation(pa[:n], pb[:n]))
    return float(min(max(min(spectral, temporal), 0.0), 1.0))


def durations_close(a_s: float, b_s: float) -> bool:
    """Return True when two durations are within 25 % (of the longer) or 0.3 s."""
    return abs(a_s - b_s) <= max(DURATION_REL * max(a_s, b_s), DURATION_ABS_S)


def duplicate_pairs(prints: Sequence[Fingerprint]):
    """Return ``[(score, i, j)]`` for every pair ``i < j`` of close duration, best first."""
    order = sorted(range(len(prints)), key=lambda i: prints[i].duration_s)
    pairs = []
    for k, i in enumerate(order):
        for j in order[k + 1:]:
            if not durations_close(prints[i].duration_s, prints[j].duration_s):
                break
            a, b = (i, j) if i < j else (j, i)
            pairs.append((round(duplicate_score(prints[a], prints[b]), 3), a, b))
    return sorted(pairs, key=lambda p: (-p[0], p[1], p[2]))
