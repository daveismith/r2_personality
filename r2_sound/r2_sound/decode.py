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

The player's only module that imports ``soundfile`` (``build_catalogue``, a tool, is the
other), and only at the first decode, so importing it loads no audio library. A file at
another rate is resampled with :func:`r2_sound.analysis.resample` (linear interpolation,
box-filtered when it decimates), the catalogue builder's own conversion; good enough for the
8-22 kHz sources of the sets.
:class:`DecodeCache` keeps the decoded buffers, least recently used first out, under a byte
cap; a clip over half the cap is decoded for its play but never kept, so one long clip
cannot evict all the others. Decoding runs on the player's decoder thread, never on the
worker or the audio callback (the worker looks a hit up with :meth:`DecodeCache.peek`, which
never decodes); the cached buffers are read-only, so a backend can play one without a copy.
A file is read in chunks of :data:`CHUNK_FRAMES`, each made float32 mono as it is read, so a
long clip's decode holds about two float32 copies at most (the chunks and the joined
buffer); an ``abandoned`` callable, checked before each chunk, stops a decode nobody wants
any more (:class:`DecodeCancelled`, nothing cached).
"""

from collections import OrderedDict
import threading

import numpy as np
from r2_sound.analysis import resample, to_mono
from r2_sound.gain import db_to_linear

CHUNK_FRAMES = 65536                 # read at a time: about 1.5 s at 44.1 kHz


class DecodeCancelled(Exception):
    """A decode stopped because its ``abandoned`` callable said so; nothing was kept."""


def never():
    """Return False: the ``abandoned`` of a decode nobody can abandon."""
    return False


def decode(path, rate, gain_db=0.0, abandoned=never):
    """
    Return the file at ``path`` as a read-only float32 mono buffer at ``rate``, gained.

    The loader contract of :class:`DecodeCache`: ``abandoned`` is a zero-argument callable,
    checked before each chunk of the file; once it returns True the decode stops and raises
    :class:`DecodeCancelled`.
    """
    import soundfile                 # here, not at the top: see the module docstring
    chunks = []
    with soundfile.SoundFile(path) as f:
        source_rate = f.samplerate
        while True:
            if abandoned():
                raise DecodeCancelled(path)
            chunk = f.read(CHUNK_FRAMES, dtype='float32', always_2d=True)
            if not len(chunk):
                break
            chunks.append(to_mono(chunk).astype(np.float32))   # float32 from here on
    data = np.concatenate(chunks) if chunks else np.zeros(0, dtype=np.float32)   # owned
    chunks.clear()
    x = resample(data, source_rate, rate)                 # float32 in, float32 out
    del data
    if gain_db:
        x *= np.float32(db_to_linear(gain_db))
    x.flags.writeable = False
    return x


class DecodeCache:
    """
    Decoded clips by ``(path, gain_db)``, least recently used evicted beyond the byte cap.

    A buffer larger than half the cap is returned but not kept. Thread-safe.

    ``loader`` is called as ``loader(path, rate, gain_db, abandoned)`` and returns a
    float32 mono buffer; ``abandoned`` is always a zero-argument callable (:func:`never`
    when the caller of :meth:`get` gives none) that the loader may check to stop early with
    :class:`DecodeCancelled`. :func:`decode` is the default; a test passes its own.
    """

    def __init__(self, capacity_bytes, rate, loader=decode):
        """Keep up to ``capacity_bytes``; ``loader(path, rate, gain_db, abandoned)`` decodes."""
        self.capacity_bytes = int(capacity_bytes)
        self.rate = rate
        self._loader = loader
        self._lock = threading.Lock()
        self._items = OrderedDict()
        self.bytes = 0
        self._generation = 0         # moved by clear(): a decode begun before is not kept
        self.hits = 0
        self.misses = 0

    @staticmethod
    def _key(path, gain_db):
        return path, round(float(gain_db), 3)

    def _hit(self, key):
        """Return the cached buffer of ``key`` (counted, made most recent), or None; locked."""
        buffer = self._items.get(key)
        if buffer is not None:
            self._items.move_to_end(key)
            self.hits += 1
        return buffer

    def peek(self, path, gain_db=0.0):
        """Return the cached buffer of ``path`` at ``gain_db``, or None; never decodes."""
        with self._lock:
            return self._hit(self._key(path, gain_db))

    def get(self, path, gain_db=0.0, abandoned=None):
        """
        Return the buffer of ``path`` at ``gain_db``, decoding it on a miss.

        ``abandoned`` (a zero-argument callable, :func:`never` when None) goes to the
        loader; a cancelled decode raises :class:`DecodeCancelled` and caches nothing.
        """
        key = self._key(path, gain_db)
        with self._lock:
            buffer = self._hit(key)
            if buffer is not None:
                return buffer
            self.misses += 1
            generation = self._generation
        if abandoned is None:
            abandoned = never
        buffer = self._loader(path, self.rate, key[1], abandoned)   # outside the lock: slow
        with self._lock:
            if (generation == self._generation and key not in self._items
                    and buffer.nbytes <= self.capacity_bytes / 2):
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
