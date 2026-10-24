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
everything else: rendering a phrase (a job with a :class:`Voice`, through the ``synth``
given, :func:`r2_sound.synth.render`), the gain and the duration cut, replacing a play with a
fade, and all the publishing through a ``sink`` (``state(PlayerState)``, ``level(rms)``,
``playing(bool)``, ``warn(text)``): the level at ``level_hz`` and the state at ``state_hz``
while playing, and while idle only the state once a second. A play's end publishes a 0
level, ``playing`` false and the IDLE state at once. ``playing`` agrees with the state: it
goes true when a play is accepted (a pending decode included) and false when nothing is
pending or playing, published on a change only (a replacement does not toggle it).

The worker never decodes and never waits on a decode. A clip in the cache (a hit, looked
up without decoding) starts at once on the worker and is answered with the play's exact
length. A miss goes to the decoder thread and is answered at once, accepted with the
catalogue's length (``clip_ms``, cut): the current play fades out (``last_end`` 4), the
state shows the new play PLAYING with ``elapsed_ms`` 0 (at 1 Hz, so the arbiter sees its
run, rule 13) and the stream starts when the decode is done; a decode (or a backend start)
that fails then ends it with ``last_end`` 5 (failed) and a WARN. A stop, or another play,
meanwhile drops it, and the decoder stops reading the file at its next chunk instead of
finishing a decode nobody wants.

A new play disturbs neither the pending play nor the current one until it is known to
start: a phrase is rendered first (a synth failure is refused with nothing dropped), a file
that is not there is refused at once (``decode failed: no such file``), and a hit is gained
and cut first (an empty clip is refused with nothing dropped). Only then is the pending play
dropped (``last_end`` 4) and the current one faded.

A ``duration_ms`` within a millisecond of the clip's length (a length rounded to the
millisecond) is not a cut: the clip plays to its end (``last_end`` 1) from the cached buffer.
Only a shorter one cuts (``last_end`` 2). The arbiter sends 0 (the whole clip) rather than
a clip's own length.
"""

from dataclasses import dataclass
import os
import queue
import threading
import time
from typing import Optional

import numpy as np
from r2_sound.backends import fade_frames
from r2_sound.decode import DecodeCancelled
from r2_sound.gain import db_to_linear, fade_out, play_gain_db

IDLE, PLAYING, FADING = 0, 1, 2
END_NONE, END_ENDED, END_DURATION, END_STOPPED, END_REPLACED, END_FAILED = range(6)
IDLE_STATE_S = 1.0       # the state while idle (rule 11: nothing faster than 1 Hz)
WATCHDOG_S = 2.0         # a play past its length by this much: the output is stuck
END_WAIT_S = 0.5         # beyond the fade, for the fading play's end before the next starts
REPLY_TIMEOUT_S = 5.0
WARN_S = 60.0            # a failing worker step is reported at most this often


@dataclass(frozen=True)
class Voice:
    """A phrase to render instead of a file (``vocalize``): the synth's arguments."""

    arousal: float
    valence: float
    length_s: float
    seed: int


@dataclass(frozen=True)
class PlayJob:
    """
    One accepted play request, resolved to a file and a gain by the node.

    With ``voice`` the worker renders that phrase instead of decoding ``path``; the phrase
    is at the house level with its peak at most -1 dBFS, so ``gain_db`` is the request's
    gain alone, clamped by the worker against the rendered peak (rule 4).
    """

    name: str
    path: str
    catalogue_gain_db: float
    gain_db: float               # the play's total gain (catalogue + request, clamped)
    duration_ms: int = 0         # 0: the whole clip
    priority: int = 0
    owner: str = ''
    intent_id: str = ''
    voice: Optional[Voice] = None
    clip_ms: int = 0             # the catalogue's length: the answer while decoding


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


class _Decoding:
    """A file's decode on the decoder thread; queued back to the worker once done."""

    kind = 'decoded'
    done = None

    def __init__(self, job):
        self.job = job
        self.abandoned = False       # replaced or stopped before it was decoded
        self.buffer = None
        self.error = None

    def is_abandoned(self):
        """Return True once dropped: the decoder checks it between chunks of the file."""
        return self.abandoned


class Player:
    """The worker: see the module docstring. ``start()`` runs it, ``close()`` ends it."""

    def __init__(self, backend, cache, sink, rate, fade_ms=20.0, state_hz=5.0, level_hz=30.0,
                 clock=time.monotonic, synth=None):
        self.backend = backend
        self.cache = cache
        self.synth = synth           # render(arousal, valence, length_s, seed, rate), or None
        self.sink = sink
        self.rate = int(rate)
        self.fade_ms = float(fade_ms)
        self._fade_frames = fade_frames(self.rate, self.fade_ms)
        self._ms_frames = self.rate / 1000.0             # the rounding of a length in ms
        self._fade = fade_out(self._fade_frames)          # the duration cut's tail
        self._end_wait_s = self.fade_ms / 1000.0 + END_WAIT_S
        self._state_period = 1.0 / float(state_hz)
        self._level_period = 1.0 / float(level_hz)
        self._clock = clock
        self._queue = queue.Queue()
        self._current = None
        self._pending = None         # the _Decoding a play waits on, while nothing plays
        self._last_end = END_NONE
        self._playing = False        # what ``playing`` last published (the node starts it false)
        self._closed = False
        self._warned = {}
        self._thread = threading.Thread(target=self._run, daemon=True, name='sound worker')
        self._decodes = queue.Queue()
        self._decoder = threading.Thread(target=self._decode_loop, daemon=True,
                                         name='sound decoder')

    # --- the node's side --------------------------------------------------------------------

    def start(self):
        """Run the worker and the decoder."""
        self._decoder.start()
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
            pending = self._pending
            if pending is None:
                return PlayerState(IDLE, last_end=self._last_end, backend=self.backend.name)
            job = pending.job                # accepted and decoding: the play has begun
            return PlayerState(PLAYING, job.name, job.owner, job.priority, job.intent_id,
                               remaining_ms=self._accepted_ms(job), last_end=self._last_end,
                               backend=self.backend.name)
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
        if item.kind == 'decoded':
            if item is not self._pending:
                return False                  # dropped by a stop or another play
            self._pending = None
            self._start_decoded(item)
            return True
        if item.kind == 'ended' and item.job is self._current:   # from the backend
            current = item.job
            if current.reason is None:
                current.reason = self._natural_end(current)
            self._finish(current.reason)
        return False

    def _decode_loop(self):
        while True:
            decoding = self._decodes.get()
            if decoding is None:
                return
            if decoding.abandoned:
                continue                      # dropped while queued: never read
            try:
                decoding.buffer = self.cache.get(decoding.job.path,
                                                 decoding.job.catalogue_gain_db,
                                                 abandoned=decoding.is_abandoned)
            except DecodeCancelled:
                continue                      # dropped while read: nothing kept, no warning
            except Exception as exc:
                decoding.error = exc
            self._queue.put(decoding)

    def _drop_pending(self, end):
        """Forget the play waiting on its decode, ending it with ``end``; True if there was one."""
        pending, self._pending = self._pending, None
        if pending is None:
            return False
        pending.abandoned = True
        self._last_end = end
        return True

    def _render(self, job):
        """Return ``(buffer, extra_db)``: the phrase and its gain, clamped (rule 4)."""
        v = job.voice
        buffer, info = self.synth(v.arousal, v.valence, v.length_s, v.seed, self.rate)
        gain_db, clamped = play_gain_db(0.0, job.gain_db, info.peak_dbfs)
        if clamped:
            self.sink.warn('vocalize %s: gain %+.1f dB clamped to %+.1f dB (peak ceiling)' % (
                job.name, job.gain_db, gain_db))
        return buffer, gain_db

    def _cut(self, frames, duration_ms):
        """Return ``(frames to play, cut)``; a duration within 1 ms of the end is no cut."""
        if duration_ms:
            wanted = max(1, int(round(duration_ms * self.rate / 1000.0)))
            if frames - wanted > self._ms_frames:
                return wanted, True
        return frames, False

    def _prepare(self, buffer, extra_db, duration_ms):
        """Return ``(buffer, cut)``: gained and cut with a fade, or the buffer itself."""
        frames, cut = self._cut(len(buffer), duration_ms)
        if not cut and abs(extra_db) < 1e-6:
            return buffer, False             # the cached buffer, read-only, no copy
        out = buffer[:frames] * np.float32(db_to_linear(extra_db))
        if cut:
            fade = self._fade[-frames:]          # the tail of the curve for a tiny cut
            out[frames - len(fade):] *= fade
        return out, cut

    def _play(self, job):
        if job.voice is not None:
            try:
                buffer, extra_db = self._render(job)
            except Exception as exc:          # refused: the pending and the current untouched
                return False, 'synth failed: %s' % str(exc)[:120], 0
            return self._start(job, buffer, extra_db)
        if not os.path.isfile(job.path):
            return False, 'decode failed: no such file', 0
        extra_db = job.gain_db - job.catalogue_gain_db
        buffer = self.cache.peek(job.path, job.catalogue_gain_db)
        if buffer is not None:                # a hit: no decode, start now
            return self._start(job, buffer, extra_db)
        decoding = _Decoding(job)             # a miss: decoded on the decoder, answered now
        self._drop_pending(END_REPLACED)
        self._decodes.put(decoding)
        previous = self._fade_current(END_REPLACED)
        self._pending = decoding             # the state shows the play from here on
        self._set_playing(True)
        if previous is not None:
            self._finish(previous.reason)     # the pending play keeps ``playing`` true
        else:
            self.sink.state(self.snapshot())
        return True, '', self._accepted_ms(job)

    def _accepted_ms(self, job):
        """Return the length a play has before its decode is done: the catalogue's, cut."""
        if not job.clip_ms:
            return job.duration_ms
        clip_frames = int(round(job.clip_ms * self.rate / 1000.0))
        return self._ms(self._cut(clip_frames, job.duration_ms)[0])

    def _start_decoded(self, decoding):
        """Start a play whose decode is done, or end it (``last_end`` 5) if it failed."""
        job, reason = decoding.job, ''
        if decoding.error is not None:
            reason = 'decode failed: %s' % str(decoding.error)[:120]
        else:
            accepted, reason, _ = self._start(job, decoding.buffer,
                                              job.gain_db - job.catalogue_gain_db)
            if accepted:
                return
        self.sink.warn('sound: %s: %s; play ended' % (job.name, reason))
        self._finish(END_FAILED)

    def _start(self, job, buffer, extra_db):
        """
        Start ``buffer`` (decoded or rendered); return ``(accepted, reason, duration_ms)``.

        An empty clip is refused before anything is touched; only then is the pending play
        dropped and the current one faded. A backend that fails after that ends what was
        dropped or faded (``last_end`` 3) with the IDLE state at once.
        """
        buffer, cut = self._prepare(buffer, extra_db, job.duration_ms)
        if len(buffer) == 0:
            return False, 'empty clip', 0
        dropped = self._drop_pending(END_REPLACED)
        previous = self._fade_current(END_REPLACED)
        current = _Current(job, len(buffer), cut)
        try:
            self.backend.start(buffer, self._on_block(current), self._on_end(current),
                               name=job.name)
        except Exception as exc:
            if previous is not None or dropped:
                self._finish(END_STOPPED)
            return False, 'backend: %s' % str(exc)[:120], 0
        current.deadline = self._clock() + len(buffer) / self.rate + WATCHDOG_S
        if previous is not None:
            self._last_end = previous.reason
        self._current = current
        self._set_playing(True)
        return True, '', self._ms(len(buffer))

    def _set_playing(self, value):
        """Publish ``playing`` when it changes, and only then."""
        if value != self._playing:
            self._playing = value
            self.sink.playing(value)

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
        if self._drop_pending(END_STOPPED):  # nothing plays while a decode is pending
            self._finish(END_STOPPED)
            return True
        current = self._current
        if current is None:
            return False
        if current.reason is None:
            if not self.backend.stop(self.fade_ms):
                return False                  # it ended on its own: its 'ended' is queued
            current.reason = END_STOPPED
        return True

    def _finish(self, reason):
        """
        End the current play: the zeros and the state, now.

        ``playing`` goes false unless a play is pending (a miss that replaced this one).
        """
        self._current = None
        self._last_end = reason
        self.sink.level(0.0)
        if self._pending is None:
            self._set_playing(False)
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
        self._drop_pending(END_STOPPED)
        self._decodes.put(None)
        current = self._fade_current(END_STOPPED)
        try:
            self.backend.close()
        finally:
            self._finish(current.reason if current is not None else self._last_end)
