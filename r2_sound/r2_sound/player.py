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
The player's worker (plan 4.3): one thread that owns the backend. No ROS imports.

The node's service callbacks hand requests over with :meth:`Player.play`, :meth:`Player.stop`
and :meth:`Player.close` and wait for the worker's answer; the backend's thread reports each
block's RMS and the end of a play by storing a value and queueing a message. The worker does
everything else: decoding (through the cache), the gain and the duration cut, replacing a
play with a fade, and all the publishing through a ``sink`` (``state(PlayerState)``,
``level(rms)``, ``playing(bool)``, ``warn(text)``): the level at ``level_hz`` and the state
at ``state_hz`` while playing, and while idle only the state once a second. A play's end
publishes a 0 level, ``playing`` false and the IDLE state at once.
"""

from dataclasses import dataclass
import queue
import threading
import time

import numpy as np
from r2_sound.backends import fade_frames
from r2_sound.gain import db_to_linear, fade_out

IDLE, PLAYING, FADING = 0, 1, 2
END_NONE, END_ENDED, END_DURATION, END_STOPPED, END_REPLACED = range(5)
IDLE_STATE_S = 1.0       # the state while idle (rule 11: nothing faster than 1 Hz)
WATCHDOG_S = 2.0         # a play past its length by this much: the output is stuck
END_WAIT_S = 0.5         # beyond the fade, for the fading play's end before the next starts
REPLY_TIMEOUT_S = 5.0
WARN_S = 60.0            # a failing worker step is reported at most this often


@dataclass(frozen=True)
class PlayJob:
    """One accepted play request, resolved to a file and a gain by the node."""

    name: str
    path: str
    catalogue_gain_db: float
    gain_db: float               # the play's total gain (catalogue + request, clamped)
    duration_ms: int = 0         # 0: the whole clip
    priority: int = 0
    owner: str = ''
    intent_id: str = ''


@dataclass(frozen=True)
class PlayerState:
    """What the state topic reports."""

    state: int
    name: str = ''
    owner: str = ''
    priority: int = 0
    intent_id: str = ''
    elapsed_ms: int = 0
    remaining_ms: int = 0
    last_end: int = END_NONE
    level: float = 0.0
    backend: str = ''


class _Current:
    """The play the worker is running."""

    def __init__(self, job, frames, cut):
        self.job = job
        self.frames = frames
        self.cut = cut
        self.reason = None           # set when the worker asked it to end
        self.level = 0.0
        self.ended = threading.Event()
        self.deadline = 0.0


class _Request:
    """A hand-over to the worker and its answer (``fallback`` on a timeout or a failure)."""

    def __init__(self, kind, job=None, fallback=None):
        self.kind = kind
        self.job = job
        self.fallback = fallback
        self.result = fallback
        self.done = threading.Event()
        self._lock = threading.Lock()
        self._state = None           # 'taken' by the worker or 'cancelled' by the caller

    def _mark(self, state):
        """Return True if this call settled who has the request."""
        with self._lock:
            if self._state is None:
                self._state = state
            return self._state == state

    def take(self):
        """Return True if the worker may serve it (the caller has not given up)."""
        return self._mark('taken')

    def cancel(self):
        """Return True if the worker will never serve it."""
        return self._mark('cancelled')

    def reply(self, result):
        self.result = result
        self.done.set()


class _Ended:
    """The backend's word that a play ended: queued from its thread, so nothing to build."""

    kind = 'ended'
    done = None

    def __init__(self, job):
        self.job = job


class Player:
    """The worker: see the module docstring. ``start()`` runs it, ``close()`` ends it."""

    def __init__(self, backend, cache, sink, rate, fade_ms=20.0, state_hz=5.0, level_hz=30.0,
                 clock=time.monotonic):
        self.backend = backend
        self.cache = cache
        self.sink = sink
        self.rate = int(rate)
        self.fade_ms = float(fade_ms)
        self._fade_frames = fade_frames(self.rate, self.fade_ms)
        self._fade = fade_out(self._fade_frames)          # the duration cut's tail
        self._end_wait_s = self.fade_ms / 1000.0 + END_WAIT_S
        self._state_period = 1.0 / float(state_hz)
        self._level_period = 1.0 / float(level_hz)
        self._clock = clock
        self._queue = queue.Queue()
        self._current = None
        self._last_end = END_NONE
        self._closed = False
        self._warned = {}
        self._thread = threading.Thread(target=self._run, daemon=True, name='sound worker')

    # --- the node's side --------------------------------------------------------------------

    def start(self):
        """Run the worker."""
        self._thread.start()

    @property
    def closed(self):
        """Return True once :meth:`close` was called."""
        return self._closed

    def play(self, job, timeout=REPLY_TIMEOUT_S):
        """Hand ``job`` over; return ``(accepted, reason, duration_ms)``."""
        if self._closed:
            return False, 'closed', 0
        return self._ask(_Request('play', job, (False, 'timeout', 0)), timeout)

    def stop(self, timeout=REPLY_TIMEOUT_S):
        """Fade the current play out; return False when nothing was playing."""
        if self._closed:
            return False
        return self._ask(_Request('stop', fallback=False), timeout)

    def close(self, timeout=REPLY_TIMEOUT_S):
        """Stop the play, publish the zeros and IDLE, let go of the backend, end the worker."""
        if self._closed:
            return
        self._closed = True
        if self._thread.is_alive():
            self._ask(_Request('close'), timeout)
            self._thread.join(timeout)
        else:                        # never started: still leave the outputs at rest
            self._finish(self._last_end)
            self.backend.close()

    def snapshot(self):
        """Return the state as the worker sees it now."""
        current = self._current
        if current is None:
            return PlayerState(IDLE, last_end=self._last_end, backend=self.backend.name)
        job = current.job
        position = min(self.backend.position_frames(), current.frames)
        remaining = current.frames - position
        state = PLAYING
        if current.reason is not None:
            state = FADING
            remaining = min(remaining, self._fade_frames)
        return PlayerState(
            state, job.name, job.owner, job.priority, job.intent_id,
            elapsed_ms=self._ms(position), remaining_ms=self._ms(remaining),
            last_end=self._last_end, level=current.level, backend=self.backend.name)

    def _ask(self, request, timeout):
        """Queue ``request``; on a timeout it is withdrawn unless the worker has it already."""
        self._queue.put(request)
        if not request.done.wait(timeout) and not request.cancel():
            request.done.wait(timeout)   # being served: the answer is close
        return request.result

    def _ms(self, frames):
        return int(round(frames * 1000.0 / self.rate))

    # --- the worker -------------------------------------------------------------------------

    def _run(self):
        next_level = next_state = 0.0
        while True:
            current = self._current
            if current is not None:
                wait = min(next_level, next_state, current.deadline) - self._clock()
            else:
                wait = next_state - self._clock()
            try:
                item = self._queue.get(timeout=max(0.0, wait))
            except queue.Empty:
                item = None
            if item is not None and item.done is not None and not item.take():
                continue                     # its caller gave up waiting: never served
            if item is not None and item.kind == 'close':
                try:
                    self._close()
                except Exception as exc:
                    self._warn(exc)
                finally:
                    item.reply(None)
                return
            try:
                changed = item is not None and self._handle(item)
                now = self._clock()
                if self._current is not None and now >= self._current.deadline:
                    self._stuck(self._current)
                if self._current is None:
                    if current is not None:          # it just ended: _finish published
                        next_state = now + IDLE_STATE_S
                    elif now >= next_state:
                        self.sink.state(self.snapshot())
                        self.backend.housekeeping()
                        next_state = now + IDLE_STATE_S
                    continue
                if changed:
                    next_level = next_state = now
                if now >= next_level:
                    self.sink.level(self._current.level)
                    next_level = now + self._level_period
                if now >= next_state:
                    self.sink.state(self.snapshot())
                    next_state = now + self._state_period
            except Exception as exc:     # the worker must survive anything a play throws
                self._warn(exc)
                next_level = next_state = self._clock() + IDLE_STATE_S   # no busy loop
                if item is not None and item.done is not None and not item.done.is_set():
                    item.reply((False, 'internal error', 0) if item.kind == 'play'
                               else item.fallback)

    def _warn(self, exc):
        """Report a worker failure, each kind at most once per WARN_S."""
        key, now = type(exc).__name__, self._clock()
        if now - self._warned.get(key, -WARN_S) >= WARN_S:
            self._warned[key] = now
            self.sink.warn('sound worker: %s: %s' % (key, exc))

    def _handle(self, item):
        """Serve one request; return True when the state changed."""
        if item.kind == 'play':
            item.reply(self._play(item.job))
            return True
        if item.kind == 'stop':
            item.reply(self._stop())
            return True
        if item.kind == 'ended' and item.job is self._current:   # from the backend
            current = item.job
            if current.reason is None:
                current.reason = self._natural_end(current)
            self._finish(current.reason)
        return False

    def _buffer(self, job):
        """Return ``(buffer, cut)``: decoded, gained, and cut with a fade at the duration."""
        buffer = self.cache.get(job.path, job.catalogue_gain_db)
        extra_db = job.gain_db - job.catalogue_gain_db
        frames = len(buffer)
        wanted = int(round(job.duration_ms * self.rate / 1000.0)) if job.duration_ms else frames
        cut = wanted < frames
        frames = max(1, min(frames, wanted))
        if not cut and abs(extra_db) < 1e-6:
            return buffer, False             # the cached buffer, read-only, no copy
        out = buffer[:frames] * np.float32(db_to_linear(extra_db))
        if cut:
            fade = self._fade[-frames:]          # the tail of the curve for a tiny cut
            out[frames - len(fade):] *= fade
        return out, cut

    def _play(self, job):
        try:
            buffer, cut = self._buffer(job)
        except Exception as exc:
            return False, 'decode failed: %s' % str(exc)[:120], 0
        if len(buffer) == 0:
            return False, 'empty clip', 0
        previous = self._fade_current(END_REPLACED)
        current = _Current(job, len(buffer), cut)
        try:
            self.backend.start(buffer, self._on_block(current), self._on_end(current),
                               name=job.name)
        except Exception as exc:
            if previous is not None:
                self._finish(END_STOPPED)
            return False, 'backend: %s' % str(exc)[:120], 0
        current.deadline = self._clock() + len(buffer) / self.rate + WATCHDOG_S
        if previous is not None:
            self._last_end = previous.reason
        else:
            self.sink.playing(True)
        self._current = current
        return True, '', self._ms(len(buffer))

    def _on_block(self, current):
        def on_block(rms):
            current.level = rms
        return on_block

    def _on_end(self, current):
        def on_end(frames, stopped):
            current.ended.set()
            self._queue.put(_Ended(current))
        return on_end

    def _stop(self):
        current = self._current
        if current is None:
            return False
        if current.reason is None:
            if not self.backend.stop(self.fade_ms):
                return False                  # it ended on its own: its 'ended' is queued
            current.reason = END_STOPPED
        return True

    def _finish(self, reason):
        """End the current play: the zeros, ``playing`` false and the IDLE state, now."""
        had = self._current is not None
        self._current = None
        self._last_end = reason
        self.sink.level(0.0)
        if had:
            self.sink.playing(False)
        self.sink.state(self.snapshot())

    def _stuck(self, current):
        self.sink.warn('sound: %s did not end %g s past its length; output reset' % (
            current.job.name, WATCHDOG_S))
        self.backend.abort()
        self._finish(current.reason or END_ENDED)

    def _fade_current(self, reason):
        """Fade the current play out and wait for its end (bounded); return it, or None."""
        current = self._current
        if current is not None:
            if current.reason is None:
                faded = self.backend.stop(self.fade_ms)
                current.reason = reason if faded else self._natural_end(current)
            current.ended.wait(self._end_wait_s)
        return current

    @staticmethod
    def _natural_end(current):
        return END_DURATION if current.cut else END_ENDED

    def _close(self):
        current = self._fade_current(END_STOPPED)
        try:
            self.backend.close()
        finally:
            self._finish(current.reason if current is not None else self._last_end)
