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
The player's outputs (plan 4.3): ``sounddevice``, ``wav`` and ``null``.

The only module that imports ``sounddevice``. Every backend plays one buffer at a time
(float32 mono at the output rate, gain already applied) through the same interface:

* ``start(buffer, on_block, on_end, name='')`` begins a play; a play still running is ended
  at once (its ``on_end`` called with ``stopped`` true), so the caller fades it first.
* ``stop(fade_ms)`` fades the current play out and ends it; False when nothing plays.
* ``position_frames()`` is the frame count of the current (or last) play: the clock the
  player's state reports (rule 13).
* ``housekeeping()`` (the worker calls it while idle), ``abort()`` (drop the play without
  ``on_end``, for a stuck output) and ``close()``.

``on_block(rms)`` is called once per output block (1470 frames at 44.1 kHz, 30 a second) with
the block's RMS, and ``on_end(frames, stopped)`` once when the play ends. Both are called from
the backend's own thread (the PortAudio callback for ``sounddevice``) and must only store a
value or queue a message: never rclpy, never anything slow.

:class:`Play` holds one buffer's position and pending fade, and renders blocks; the
``sounddevice`` callback and the ``wav`` thread both use it, so all three backends fade, cut
and measure the same way.
"""

import datetime
import os
import re
import threading
import time
import wave

import numpy as np
from r2_sound.gain import block_rms, fade_out

try:
    import sounddevice
except (ImportError, OSError):       # no PortAudio: only the sounddevice backend is lost
    sounddevice = None

BLOCKS_PER_S = 30.0                  # 1470 frames at 44.1 kHz


def block_frames(rate):
    """Return the output block length at ``rate``: a 30th of a second."""
    return max(1, int(round(rate / BLOCKS_PER_S)))


def fade_frames(rate, fade_ms):
    """Return the length of a ``fade_ms`` fade at ``rate``, at least one frame."""
    return max(1, int(round(rate * fade_ms / 1000.0)))


class Play:
    """One buffer being played: its position, a pending fade, and the block renderer."""

    def __init__(self, buffer, on_block, on_end):
        self.buffer = buffer
        self.on_block = on_block
        self._on_end = on_end
        self.position = 0
        self.stopped = False
        self.done = False
        self._fade = None
        self._fade_at = 0
        self._ended = False

    def begin_fade(self, frames):
        """Fade out over ``frames`` from the next block on; False when ended or fading."""
        if self.done or self._fade is not None:
            return False
        self.stopped = True
        self._fade_at = 0
        self._fade = fade_out(max(1, frames))
        return True

    def render_into(self, out):
        """
        Fill ``out`` (a float32 view) with the next block, hand its RMS to ``on_block``.

        Return ``(rms, frames)``: ``frames`` is how many came from the buffer; the rest of
        the block is silence. The play is ``done`` once the buffer or the fade runs out.
        """
        start = self.position
        left = len(self.buffer) - start
        fade = self._fade
        if fade is None:
            count = min(len(out), left)
            out[:count] = self.buffer[start:start + count]
        else:
            at = self._fade_at
            count = min(len(out), left, len(fade) - at)
            np.multiply(self.buffer[start:start + count], fade[at:at + count],
                        out=out[:count])
            self._fade_at = at + count
            if self._fade_at >= len(fade):
                self.done = True
        out[count:] = 0.0
        self.position = start + count
        if self.position >= len(self.buffer):
            self.done = True
        rms = block_rms(out)
        self.on_block(rms)
        return rms, count

    def end(self):
        """Call ``on_end`` once."""
        if not self._ended:
            self._ended = True
            self.done = True
            self._on_end(self.position, self.stopped)


class Backend:
    """What every backend shares: the current :class:`Play` and its hand-over."""

    name = ''

    def __init__(self, rate):
        self.rate = int(rate)
        self.block = block_frames(self.rate)
        self._lock = threading.Lock()
        self._play = None
        self._last_position = 0

    def start(self, buffer, on_block, on_end, name=''):
        """Play ``buffer``; see the module docstring."""
        raise NotImplementedError

    def stop(self, fade_ms):
        """Fade the current play out over ``fade_ms``; False when nothing plays."""
        play = self._play
        if play is None:
            return False
        return play.begin_fade(fade_frames(self.rate, fade_ms))

    def position_frames(self):
        """Return the current play's frame position, or the last play's."""
        play = self._play
        return play.position if play is not None else self._last_position

    def housekeeping(self):
        """Do the idle work (close a lingering stream); called from the worker."""

    def abort(self):
        """Drop the current play at once without calling its ``on_end``."""
        with self._lock:
            play, self._play = self._play, None
        if play is not None:
            play.done = True
            self._last_position = play.position

    def close(self):
        """Stop at once and let go of the output."""
        self.abort()

    def _swap(self, play):
        """Make ``play`` current; a play still running ends now, stopped."""
        with self._lock:
            old, self._play = self._play, play
        if old is not None and not old.done:
            old.stopped = True
            old.end()

    def _finish(self, play):
        """End ``play`` from the rendering thread, unless it was replaced or aborted."""
        with self._lock:
            if self._play is not play:
                return False
            self._play = None
            self._last_position = play.position
        play.end()
        return True


def find_device(substring):
    """Return the index of the first output device whose name contains ``substring``."""
    if not substring:
        return None
    found = []
    for index, device in enumerate(sounddevice.query_devices()):
        if device['max_output_channels'] > 0:
            if substring.lower() in device['name'].lower():
                return index
            found.append(device['name'])
    raise ValueError('no output device matches %r (have: %s)' % (
        substring, ', '.join(found) or 'none'))


class SoundDeviceBackend(Backend):
    """
    PortAudio output: a stream opened by a play, closed ``linger_s`` after the last ends.

    ``device`` is a name substring ('' for the default device). The stream callback copies
    frames, fades, measures the block and hands the RMS on; it allocates nothing.
    """

    name = 'sounddevice'

    def __init__(self, rate, device='', linger_s=2.0, clock=time.monotonic):
        super().__init__(rate)
        if sounddevice is None:
            raise RuntimeError('sounddevice cannot be imported (is PortAudio installed?)')
        self.device = device
        self.linger_s = float(linger_s)
        self._clock = clock
        self._stream = None
        self._idle_since = None
        # Found once: PortAudio's device list is fixed from its start, so a DAC plugged in
        # later needs a restart anyway; a missing one stops the node at startup.
        self._device_index = find_device(device)

    def start(self, buffer, on_block, on_end, name=''):
        """Play ``buffer``, opening the stream if it is closed."""
        play = Play(buffer, on_block, on_end)
        self._idle_since = None
        self._swap(play)
        if self._stream is None:
            try:
                stream = sounddevice.OutputStream(
                    samplerate=self.rate, blocksize=self.block, channels=1,
                    dtype='float32', device=self._device_index, callback=self._callback)
                stream.start()
            except Exception:
                self.abort()
                raise
            self._stream = stream

    def _callback(self, outdata, frames, time_info, status):
        out = outdata[:, 0]
        play = self._play
        if play is None:
            out.fill(0.0)
            return
        play.render_into(out)
        if play.done and self._finish(play):
            self._idle_since = self._clock()

    def housekeeping(self):
        """Close the stream once nothing has played for ``linger_s``."""
        since = self._idle_since
        if (self._stream is not None and self._play is None and since is not None
                and self._clock() - since >= self.linger_s):
            self._close_stream()

    def abort(self):
        """Drop the play and close the stream (a stuck output gets a fresh one)."""
        super().abort()
        self._close_stream()

    def _close_stream(self):
        stream, self._stream = self._stream, None
        self._idle_since = None
        if stream is not None:
            try:
                stream.abort()
            finally:
                stream.close()

    @property
    def stream_open(self):
        """Return True while an output stream is open."""
        return self._stream is not None


def wav_name(name, now=None):
    """Return ``<utc stamp>-<name>.wav`` with ``name`` made safe for a file name."""
    now = now or datetime.datetime.now(datetime.timezone.utc)
    stamp = now.strftime('%Y%m%dT%H%M%S.%f')[:-3] + 'Z'
    return '%s-%s.wav' % (stamp, re.sub(r'[^A-Za-z0-9._-]+', '-', name) or 'sound')


def write_wav(path, samples, rate):
    """Write float samples as a 16-bit mono WAV (clipped to -1..1)."""
    data = (np.clip(samples, -1.0, 1.0) * 32767.0).astype('<i2')
    with wave.open(path, 'wb') as out:
        out.setnchannels(1)
        out.setsampwidth(2)
        out.setframerate(rate)
        out.writeframes(data.tobytes())


class WavBackend(Backend):
    """
    No device: each play advances on wall time from a thread, as if it were heard.

    The blocks are rendered at the stream's cadence (levels and the frame count behave as
    on a device) and, when ``wav_dir`` is set, what was played (fade and cut included) is
    written to ``<wav_dir>/<utc stamp>-<name>.wav`` as the play ends.
    """

    name = 'wav'

    def __init__(self, rate, wav_dir, warn=None):
        super().__init__(rate)
        self.wav_dir = wav_dir
        self._warn = warn or (lambda text: None)
        self._closing = threading.Event()
        self.last_path = None

    def start(self, buffer, on_block, on_end, name=''):
        """Play ``buffer`` on a thread of its own."""
        play = Play(buffer, on_block, on_end)
        path = None
        if self.wav_dir:
            os.makedirs(self.wav_dir, exist_ok=True)
            path = os.path.join(self.wav_dir, wav_name(name))
        self._swap(play)
        threading.Thread(target=self._run, args=(play, path), daemon=True,
                         name='%s backend' % self.name).start()

    def _run(self, play, path):
        out = np.zeros(self.block, dtype=np.float32)
        played = []
        began = time.monotonic()
        while not play.done and not self._closing.is_set():
            _, count = play.render_into(out)
            if path is not None:
                played.append(out[:count].copy())
            # The frames rendered so far are "heard" by now + their length.
            self._closing.wait(max(0.0, began + play.position / self.rate - time.monotonic()))
        if path is not None:
            try:
                write_wav(path, np.concatenate(played) if played else np.zeros(0), self.rate)
                self.last_path = path
            except OSError as exc:
                self._warn('cannot write %s: %s' % (path, exc))
        if not self._closing.is_set():
            self._finish(play)

    def close(self):
        """End the play thread without its ``on_end``."""
        self._closing.set()
        super().close()


class NullBackend(WavBackend):
    """The ``wav`` backend without the file: for the tests."""

    name = 'null'

    def __init__(self, rate, warn=None):
        super().__init__(rate, None, warn)


_FACTORIES = {
    'sounddevice': lambda rate, device, linger_s, wav_dir, warn:
        SoundDeviceBackend(rate, device, linger_s),
    'wav': lambda rate, device, linger_s, wav_dir, warn: WavBackend(rate, wav_dir, warn),
    'null': lambda rate, device, linger_s, wav_dir, warn: NullBackend(rate, warn),
}
BACKENDS = tuple(_FACTORIES)


def check_backend(kind):
    """Raise ValueError unless ``kind`` is one of :data:`BACKENDS`."""
    if kind not in _FACTORIES:
        raise ValueError('backend must be one of %s, got %r' % (', '.join(BACKENDS), kind))


def make_backend(kind, rate, device='', linger_s=2.0, wav_dir=None, warn=None):
    """Return the backend named ``kind`` (one of :data:`BACKENDS`)."""
    check_backend(kind)
    return _FACTORIES[kind](rate, device, linger_s, wav_dir, warn)
