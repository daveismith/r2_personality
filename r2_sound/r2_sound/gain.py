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
Loudness measurement and gain arithmetic for the sound catalogue (plan 2.3).

Pure numpy, no ROS imports. Levels are dBFS on float samples in -1..1.

* :func:`measure` takes decoded samples and returns a :class:`Measurement`: duration,
  RMS over the non-silent part (10 ms blocks above :data:`SILENCE_DBFS`), peak, and the
  fraction of samples at or above :data:`CLIP_LEVEL`.
* :func:`set_gain_db` brings a set's median RMS to the house level; :func:`outlier_trim_db`
  pulls a clip more than :data:`OUTLIER_DB` from its set median halfway back (the whole
  deviation is halved, so the trim is discontinuous at the threshold: a scream stays a
  scream, a hum stays a hum, only the extremes are tamed); :func:`clip_gain_db` adds the
  sheet's override and limits the result to the peak ceiling and to the catalogue's range.
* :func:`level_to_byte` maps a dBFS level to the dome's 0..255 audio-level input (floor
  -40, ceiling -6), the mapping the player uses; :func:`block_rms` is what it is fed.
* :func:`play_gain_db` is rule 4: the catalogue gain plus a request's, clamped so the peak
  stays at or below :data:`PEAK_CEILING_DBFS`.
* :func:`fade_out` is the player's fade shape (a half cosine from 1 to 0).
"""

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

HOUSE_LEVEL_DBFS = -16.0
SILENCE_DBFS = -50.0           # sample frames below this do not count towards the RMS
BLOCK = 441                    # 10 ms at 44.1 kHz, the silence gate's block
CLIP_LEVEL = 0.999             # |x| at or above this is a clipped sample
OUTLIER_DB = 8.0
PEAK_CEILING_DBFS = -1.0
MIN_GAIN_DB = -20.0            # the catalogue's accepted range for gain_db
MAX_GAIN_DB = 12.0
FLOOR_DBFS = -120.0            # what silence measures as
LEVEL_FLOOR_DBFS = -40.0
LEVEL_CEILING_DBFS = -6.0


@dataclass(frozen=True)
class Measurement:
    """What one decoded clip measures."""

    duration_s: float
    rms_dbfs: float
    peak_dbfs: float
    clip_fraction: float


def to_dbfs(amplitude):
    """Return 20 log10 of a linear amplitude, floored at :data:`FLOOR_DBFS`."""
    if amplitude <= 0.0:
        return FLOOR_DBFS
    return max(FLOOR_DBFS, 20.0 * math.log10(amplitude))


def active_rms_dbfs(samples):
    """
    Return the RMS in dBFS over the non-silent part.

    ``samples`` is mono (n,) or (n, channels). The signal is cut into :data:`BLOCK`-sample
    blocks (all channels together); a block is non-silent when its own RMS is above
    :data:`SILENCE_DBFS`. An all-silent clip measures :data:`FLOOR_DBFS`.
    """
    x = np.asarray(samples, dtype=np.float64)
    if x.size == 0:
        return FLOOR_DBFS
    power = x ** 2 if x.ndim == 1 else np.mean(x ** 2, axis=1)
    whole = len(power) // BLOCK * BLOCK
    blocks = power[:whole].reshape(-1, BLOCK).mean(axis=1) if whole else power.mean(keepdims=True)
    tail = power[whole:]
    if whole and len(tail):
        blocks = np.append(blocks, tail.mean())
    keep = blocks > 10.0 ** (SILENCE_DBFS / 10.0)
    if not keep.any():
        return FLOOR_DBFS
    return to_dbfs(math.sqrt(float(blocks[keep].mean())))


def measure(samples, sample_rate):
    """Return the :class:`Measurement` of decoded ``samples`` at ``sample_rate``."""
    x = np.asarray(samples, dtype=np.float64)
    if x.size == 0:
        return Measurement(0.0, FLOOR_DBFS, FLOOR_DBFS, 0.0)
    a = np.abs(x)
    return Measurement(
        duration_s=len(x) / float(sample_rate),
        rms_dbfs=active_rms_dbfs(x),
        peak_dbfs=to_dbfs(float(a.max())),
        clip_fraction=float(np.mean(a >= CLIP_LEVEL)))


def median(values: Sequence[float]):
    """Return the median of ``values`` (0.0 for none)."""
    return float(np.median(values)) if len(values) else 0.0


def set_gain_db(set_median_rms_dbfs, house_level_dbfs=HOUSE_LEVEL_DBFS):
    """Return the gain that brings a set's median RMS to the house level."""
    return house_level_dbfs - set_median_rms_dbfs


def outlier_trim_db(rms_dbfs, set_median_rms_dbfs, threshold_db=OUTLIER_DB):
    """Return the trim for a clip: half the deviation back when beyond ``threshold_db``."""
    deviation = rms_dbfs - set_median_rms_dbfs
    if abs(deviation) > threshold_db:
        return -deviation / 2.0
    return 0.0


def clip_gain_db(set_gain, trim_db, override_db, peak_dbfs):
    """
    Return a clip's catalogue gain (plan 2.3).

    ``min(set_gain + trim + override, ceiling - peak)``, then kept within the catalogue's
    ``MIN_GAIN_DB..MAX_GAIN_DB`` range.
    """
    gain = min(set_gain + trim_db + override_db, PEAK_CEILING_DBFS - peak_dbfs)
    return min(MAX_GAIN_DB, max(MIN_GAIN_DB, gain))


def level_to_byte(dbfs, floor_dbfs=LEVEL_FLOOR_DBFS, ceiling_dbfs=LEVEL_CEILING_DBFS):
    """Map a level in dBFS to 0..255: ``floor`` and below is 0, ``ceiling`` and above 255."""
    fraction = (dbfs - floor_dbfs) / (ceiling_dbfs - floor_dbfs)
    return int(round(255 * min(1.0, max(0.0, fraction))))


def db_to_linear(db):
    """Return the linear amplitude factor of a gain in dB."""
    return 10.0 ** (db / 20.0)


def block_rms(samples):
    """Return the RMS (linear, 0..1 for float audio) of one output block; 0.0 when empty."""
    x = np.asarray(samples, dtype=np.float32)
    if x.size == 0:
        return 0.0
    return math.sqrt(float(np.dot(x, x)) / x.size)     # no temporary: the audio callback


def play_gain_db(catalogue_gain_db, request_gain_db, peak_dbfs):
    """
    Return ``(gain_db, clamped)`` for one play (rule 4).

    The sum of the clip's catalogue gain and the request's gain, limited so that the clip's
    measured peak plus the gain stays at or below :data:`PEAK_CEILING_DBFS`; ``clamped`` says
    the request asked for more. A NaN request counts as 0 dB, clamped.
    """
    if math.isnan(request_gain_db):
        return min(catalogue_gain_db, PEAK_CEILING_DBFS - peak_dbfs), True
    wanted = catalogue_gain_db + request_gain_db
    ceiling = PEAK_CEILING_DBFS - peak_dbfs
    if wanted > ceiling:
        return ceiling, True
    return wanted, False


def fade_out(frames):
    """
    Return the fade-out gain curve over ``frames`` samples (float32, 1 down to 0).

    A half cosine: ``0.5 * (1 + cos(pi * i / (frames - 1)))``, so the first sample is
    untouched and the last is silent. ``frames`` <= 1 gives ``[0.]`` or nothing.
    """
    if frames <= 1:
        return np.zeros(max(frames, 0), dtype=np.float32)
    i = np.arange(frames, dtype=np.float64)
    return (0.5 * (1.0 + np.cos(np.pi * i / (frames - 1)))).astype(np.float32)
