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
Decode a clip for the player (plan 4.3): float32 mono at the output rate, gain applied.

The only module that imports ``soundfile``. A file at another rate is resampled with
:func:`r2_sound.analysis.resample` (linear interpolation, box-filtered when it decimates),
the catalogue builder's own conversion; good enough for the 8-22 kHz sources of the sets.
:class:`DecodeCache` keeps the decoded buffers, least recently used first out, under a byte
cap. Decoding runs
on the player's worker thread, never on the audio callback; the cached buffers are
read-only, so a backend can play one without a copy.
"""

from collections import OrderedDict
import threading

import numpy as np
from r2_sound.analysis import resample, to_mono
from r2_sound.gain import db_to_linear
import soundfile


def decode(path, rate, gain_db=0.0):
    """Return the file at ``path`` as a read-only float32 mono buffer at ``rate``, gained."""
    data, source_rate = soundfile.read(path, dtype='float32', always_2d=True)
    x = np.array(resample(to_mono(data), source_rate, rate), dtype=np.float32)   # owned
    if gain_db:
        x *= np.float32(db_to_linear(gain_db))
    x.flags.writeable = False
    return x


class DecodeCache:
    """
    Decoded clips by ``(path, gain_db)``, least recently used evicted beyond the byte cap.

    A buffer larger than the whole cap is returned but not kept. Thread-safe; ``loader`` is
    :func:`decode` (a test passes its own).
    """

    def __init__(self, capacity_bytes, rate, loader=decode):
        self.capacity_bytes = int(capacity_bytes)
        self.rate = rate
        self._loader = loader
        self._lock = threading.Lock()
        self._items = OrderedDict()
        self.bytes = 0
        self._generation = 0         # moved by clear(): a decode begun before is not kept
        self.hits = 0
        self.misses = 0

    def get(self, path, gain_db=0.0):
        """Return the buffer of ``path`` at ``gain_db``, decoding it on a miss."""
        key = (path, round(float(gain_db), 3))
        with self._lock:
            buffer = self._items.get(key)
            if buffer is not None:
                self._items.move_to_end(key)
                self.hits += 1
                return buffer
            self.misses += 1
            generation = self._generation
        buffer = self._loader(path, self.rate, key[1])    # outside the lock: it is slow
        with self._lock:
            if (generation == self._generation and key not in self._items
                    and buffer.nbytes <= self.capacity_bytes):
                self._items[key] = buffer
                self.bytes += buffer.nbytes
                while self.bytes > self.capacity_bytes:
                    _, old = self._items.popitem(last=False)
                    self.bytes -= old.nbytes
        return buffer

    def keys(self):
        """Return the cached keys, least recently used first."""
        with self._lock:
            return list(self._items)

    def clear(self):
        """Forget every buffer (a reload: a file may have changed under the same name)."""
        with self._lock:
            self._items.clear()
            self.bytes = 0
            self._generation += 1
