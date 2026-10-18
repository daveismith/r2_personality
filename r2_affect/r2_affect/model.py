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
The mood model: four numbers, how they move, and the label read from them.

Pure Python (no ROS imports) so the rules can be unit tested and replayed
offline; ``node.py`` maps messages onto it. Times are seconds from any
monotonic clock and every call takes ``now``; the first :meth:`AffectModel.tick`
establishes the time base. The model holds no global state, so any number of
models can run side by side in one process.

State: ``arousal`` and ``valence`` in [-1, 1], ``curiosity`` and ``confidence``
in [0, 1] (fixed here, see :data:`r2_affect.config.AXES`). It starts at the
temperament, labelled with the nearest mood.

Each tick (:meth:`AffectModel.tick`):

1. ``dt`` is ``now`` minus the previous tick, clamped to ``[0, max_dt_s]``
   (``dt_clamped`` in the result says so). A ``dt`` of 0 changes nothing,
   expiry included. When the clock has gone backwards (a sim restart), every
   stored time (lock end, focus end, dwell start, refractory) is shifted by
   the same amount, so they fall due at the same relative times on the new
   clock. ``idle_s`` grows by ``dt``.
2. Unless locked: each axis relaxes toward its baseline,
   ``x += (baseline - x) * dt / tau`` (explicit Euler; the factor is capped
   at 1 so a ``tau`` shorter than ``dt`` cannot overshoot). Then, once
   ``idle_s > idle.after_s``, the ``idle`` impulse is applied as a rate,
   ``x += impulse * intensity * dt`` with
   ``intensity = min(1, (idle_s - after_s) / ramp_s)``. Axes are clamped.
3. A timed lock expires on the first tick at or after ``lock_until`` (its
   last interval was still frozen, so it froze exactly ``duration_s``). A
   focus expires on the first tick after its ttl has run out; a tick at
   exactly the end still shows it.
4. Classification. The distance to a prototype is Euclidean over the axes it
   specifies, each difference divided by the axis's half-range and multiplied
   by its ``distance_weights`` entry. The nearest prototype (the first in file
   order on a tie) is the candidate. The label moves to it only once
   ``d_candidate < d_label - hysteresis.margin`` has held at every tick for
   ``hysteresis.dwell_s`` with the same candidate; the timer restarts whenever
   the condition lapses or the candidate changes.
   ``mood_strength = clamp(1 - d_label / strength_radius, 0, 1)``.

Each stimulus (:meth:`AffectModel.stimulus`), in order:

0. Dropped if its kind is ``idle``: that entry is the drift rate, not a
   stimulus. Nothing else happens.
1. Dropped if ``confidence < min_confidence``; nothing else happens.
2. Otherwise ``idle_s`` resets to 0, whatever happens next: the stimulus is
   evidence of the world even if it is then dropped.
3. If it names a subject or a bearing it becomes the focus, for ``ttl_s``
   (``focus.default_ttl_s`` when ``ttl_s`` is 0). The focus is one record:
   a stimulus with a bearing and no subject replaces it and blanks the
   subject. Also when locked.
4. Dropped if its kind is not in ``impulses``, or if the kind has a
   ``refractory_s`` and the last *accepted* stimulus of that kind is younger
   than it.
5. Unless locked: each axis the impulse names gets ``impulse * intensity``
   (intensity clamped to [0, 1]); axes are clamped. When locked the stimulus
   still counts as accepted for the refractory period.

Force and lock (:meth:`AffectModel.set_mood`), as ``SetMood``:

* ``mood`` set: the axes its prototype specifies jump to it, the others keep
  their value, and the label is that mood at once (the dwell timer restarts).
  With ``lock`` false the dynamics resume from there and any lock is released.
* ``lock`` true: relax, idle drift and impulses stop until ``duration_s``
  passes (0: until cleared); focus and ``idle_s`` keep going.
* ``mood`` "" and ``lock`` false: clear. Always accepted, from any source.
* Any other request while another source holds the lock is refused, with
  the holder named in the reason. An unknown label, or a negative or
  non-finite ``duration_s``, is refused. A refused request changes nothing.
  An empty ``source`` is recorded as ``unknown``.

A timed lock is also expired by a stimulus or a ``set_mood`` that arrives
after ``lock_until`` but before the next tick, so it never holds anything
off past its time. A stimulus reports that in ``StimulusResult.lock_expired``;
for ``set_mood`` the caller compares ``state.locked`` before and after.
"""

from dataclasses import dataclass
import math
from types import MappingProxyType
from typing import Mapping, Optional

from r2_affect.config import AXES, AXIS_RANGES, IDLE_KIND

HALF_RANGE = MappingProxyType({axis: (hi - lo) / 2.0 for axis, (lo, hi) in AXIS_RANGES.items()})

# Tolerance on time comparisons so that 15 ticks of 0.1 s count as 1.5 s.
TIME_EPS = 1e-9

DROP_LOW_CONFIDENCE = 'low_confidence'
DROP_UNKNOWN_KIND = 'unknown_kind'
DROP_REFRACTORY = 'refractory'
DROP_RESERVED_KIND = 'reserved_kind'
NOT_APPLIED_LOCKED = 'locked'

UNKNOWN_SOURCE = 'unknown'

_ZERO_DELTA = MappingProxyType({axis: 0.0 for axis in AXES})


@dataclass(frozen=True)
class AffectState:
    """A snapshot of the model, field for field what ``MoodState`` carries."""

    arousal: float
    valence: float
    curiosity: float
    confidence: float
    mood: str
    mood_strength: float
    locked: bool
    lock_source: str              # '' when not locked
    lock_until: Optional[float]   # None: not locked, or locked until cleared
    focus_subject: str            # '' when none
    focus_bearing: float          # NaN when none or unknown
    idle_s: float

    def vector(self):
        """Return the four axes as a dict."""
        return {axis: getattr(self, axis) for axis in AXES}


@dataclass(frozen=True)
class MoodTransition:
    """A label change made by classification (not by a force)."""

    old: str
    new: str
    distance: float               # from the state to the new prototype


@dataclass(frozen=True)
class TickResult:
    """What one tick did."""

    dt: float                     # the dt used, after clamping
    raw_dt: float                 # now minus the previous tick (0 on the first tick)
    dt_clamped: bool              # raw_dt was negative or above max_dt_s
    transition: Optional[MoodTransition]
    lock_expired: bool
    focus_expired: bool
    state: AffectState


@dataclass(frozen=True)
class StimulusResult:
    """What one stimulus did."""

    kind: str
    accepted: bool                # passed the confidence, kind and refractory gates
    applied: bool                 # accepted and its impulse added (not when locked)
    reason: str                   # '' if applied, else one of DROP_* / NOT_APPLIED_LOCKED
    intensity: float              # as used, clamped to [0, 1]
    delta: Mapping[str, float]    # change of every axis, after clamping to the ranges
    focus_updated: bool
    lock_expired: bool            # this stimulus ended a timed lock that had run out
    state: AffectState


def distance(vector, center, weights):
    """Weighted, half-range-normalised Euclidean distance over ``center``'s axes."""
    total = 0.0
    for axis, target in center.items():
        diff = weights.get(axis, 1.0) * (vector[axis] - target) / HALF_RANGE[axis]
        total += diff * diff
    return math.sqrt(total)


def _clamp(value, lo, hi):
    return lo if value < lo else hi if value > hi else value


def _check_time(now):
    now = float(now)
    if not math.isfinite(now):
        raise ValueError('now must be a finite time in seconds, got %r' % now)
    return now


def _unit(value):
    """Clamp to [0, 1]; NaN and None count as 0."""
    try:
        value = float(value)
    except (TypeError, ValueError):
        return 0.0
    if math.isnan(value):
        return 0.0
    return _clamp(value, 0.0, 1.0)


class AffectModel:
    """The numeric mood model of ``affect.yaml`` (see the module docstring)."""

    def __init__(self, config, min_confidence=0.0, max_dt_s=0.5):
        max_dt_s = float(max_dt_s)
        if not (math.isfinite(max_dt_s) and max_dt_s >= 0.0):
            raise ValueError('max_dt_s must be a finite number >= 0, got %r' % max_dt_s)
        min_confidence = float(min_confidence)
        if not (math.isfinite(min_confidence) and min_confidence >= 0.0):
            raise ValueError('min_confidence must be a finite number >= 0, got %r'
                             % min_confidence)
        self.min_confidence = min_confidence
        self.max_dt_s = max_dt_s
        self._config = config
        self._x = {axis: float(config.temperament[axis]) for axis in AXES}
        self._last_tick = None
        self._idle_s = 0.0
        self._locked = False
        self._lock_source = ''
        self._lock_until = None
        self._focus_subject = ''
        self._focus_bearing = math.nan
        self._focus_until = None
        self._last_accepted = {}       # kind -> time of its last accepted stimulus
        self._pending = None           # (candidate, since): the hysteresis condition holding
        self._mood = self._nearest()[0]

    # --- read-only views -------------------------------------------------------------------

    @property
    def config(self):
        """Return the :class:`~r2_affect.config.AffectConfig` in use."""
        return self._config

    @property
    def state(self):
        """Return an :class:`AffectState` snapshot of the model now."""
        return AffectState(
            arousal=self._x['arousal'],
            valence=self._x['valence'],
            curiosity=self._x['curiosity'],
            confidence=self._x['confidence'],
            mood=self._mood,
            mood_strength=self._strength(),
            locked=self._locked,
            lock_source=self._lock_source,
            lock_until=self._lock_until,
            focus_subject=self._focus_subject,
            focus_bearing=self._focus_bearing,
            idle_s=self._idle_s,
        )

    def distances(self):
        """Return {mood: distance from the current state} in file order."""
        weights = self._config.distance_weights
        return {name: distance(self._x, proto.center, weights)
                for name, proto in self._config.moods.items()}

    # --- inputs ----------------------------------------------------------------------------

    def tick(self, now):
        """Advance the dynamics to ``now`` and classify; return a :class:`TickResult`."""
        now = _check_time(now)
        raw_dt = 0.0 if self._last_tick is None else now - self._last_tick
        self._last_tick = now
        if raw_dt < 0.0:
            self._shift_times(raw_dt)
        dt = _clamp(raw_dt, 0.0, self.max_dt_s)
        clamped = dt != raw_dt
        if dt == 0.0:
            return TickResult(dt=0.0, raw_dt=raw_dt, dt_clamped=clamped, transition=None,
                              lock_expired=False, focus_expired=False, state=self.state)

        self._idle_s += dt
        if not self._locked:
            self._relax(dt)
            self._idle_drift(dt)
            self._clamp_axes()

        lock_expired = self._expire_lock(now)
        focus_expired = False
        # A focus is still shown on a tick at exactly its end; it is gone after.
        if self._focus_until is not None and now > self._focus_until + TIME_EPS:
            self._clear_focus()
            focus_expired = True

        transition = self._classify(now)
        return TickResult(dt=dt, raw_dt=raw_dt, dt_clamped=clamped, transition=transition,
                          lock_expired=lock_expired, focus_expired=focus_expired,
                          state=self.state)

    def stimulus(self, kind, intensity, confidence, subject, bearing_rad, ttl_s, now):
        """Apply one stimulus received at ``now``; return a :class:`StimulusResult`."""
        now = _check_time(now)
        kind = kind or ''
        intensity = _unit(intensity)

        def result(accepted, applied, reason, delta=_ZERO_DELTA, focus_updated=False,
                   lock_expired=False):
            return StimulusResult(kind=kind, accepted=accepted, applied=applied, reason=reason,
                                  intensity=intensity, delta=delta,
                                  focus_updated=focus_updated, lock_expired=lock_expired,
                                  state=self.state)

        if kind == IDLE_KIND:
            return result(False, False, DROP_RESERVED_KIND)
        try:
            confidence = float(confidence)
        except (TypeError, ValueError):
            confidence = math.nan
        if not confidence >= self.min_confidence:      # NaN fails the gate too
            return result(False, False, DROP_LOW_CONFIDENCE)

        self._idle_s = 0.0
        lock_expired = self._expire_lock(now)

        focus_updated = self._update_focus(subject, bearing_rad, ttl_s, now)

        impulse = self._config.impulses.get(kind)
        if impulse is None:
            return result(False, False, DROP_UNKNOWN_KIND, focus_updated=focus_updated,
                          lock_expired=lock_expired)
        if impulse.refractory_s:
            last = self._last_accepted.get(kind)
            # A clock that went backwards (sim reset) does not hold a kind off forever.
            if last is not None and 0.0 <= now - last < impulse.refractory_s - TIME_EPS:
                return result(False, False, DROP_REFRACTORY, focus_updated=focus_updated,
                              lock_expired=lock_expired)
        self._last_accepted[kind] = now

        if self._locked:
            return result(True, False, NOT_APPLIED_LOCKED, focus_updated=focus_updated,
                          lock_expired=lock_expired)
        before = dict(self._x)
        for axis, value in impulse.deltas.items():
            self._x[axis] += value * intensity
        self._clamp_axes()
        delta = MappingProxyType({axis: self._x[axis] - before[axis] for axis in AXES})
        return result(True, True, '', delta=delta, focus_updated=focus_updated,
                      lock_expired=lock_expired)

    def set_mood(self, mood, lock, duration_s, source, now):
        """
        Force, lock or clear as ``SetMood`` does; return ``(accepted, reason)``.

        A timed lock that has run out is expired first, even if the request is
        then refused; a caller that reports lock expiry compares
        ``state.locked`` before and after the call.
        """
        now = _check_time(now)
        mood = mood or ''
        lock = bool(lock)
        source = source or UNKNOWN_SOURCE

        # A lock that ran out since the last tick no longer holds anyone off.
        self._expire_lock(now)

        if not mood and not lock:
            self._unlock()
            return True, ''
        if mood and mood not in self._config.moods:
            return False, 'unknown mood %r (known: %s)' % (
                mood, ', '.join(self._config.moods))
        try:
            duration_s = float(duration_s or 0.0)
        except (TypeError, ValueError):
            duration_s = math.nan
        if not (math.isfinite(duration_s) and duration_s >= 0.0):
            return False, 'duration_s must be a finite number >= 0, got %r' % duration_s
        if self._locked and self._lock_source != source:
            return False, 'locked by %r%s' % (
                self._lock_source,
                ' until cleared' if self._lock_until is None
                else ' for another %.1f s' % (self._lock_until - now))

        if mood:
            for axis, value in self._config.moods[mood].center.items():
                self._x[axis] = value
            self._mood = mood
            self._pending = None
        if lock:
            self._locked = True
            self._lock_source = source
            self._lock_until = now + duration_s if duration_s > 0.0 else None
        else:
            self._unlock()
        return True, ''

    def reload(self, config):
        """
        Swap in a new, already validated config, keeping the state.

        The vector, idle time, lock and focus are kept. The label is kept if the
        new config still has it, else it becomes the nearest mood; a pending
        label change starts over. Refractory history is kept for kinds that
        still exist.
        """
        self._config = config
        self._last_accepted = {kind: t for kind, t in self._last_accepted.items()
                               if kind in config.impulses}
        if self._mood not in config.moods:
            self._mood = self._nearest()[0]
        self._pending = None

    # --- internals -------------------------------------------------------------------------

    def _shift_times(self, offset):
        """Move every stored absolute time by ``offset`` (the clock went backwards)."""
        if self._pending is not None:
            self._pending = (self._pending[0], self._pending[1] + offset)
        if self._lock_until is not None:
            self._lock_until += offset
        if self._focus_until is not None:
            self._focus_until += offset
        self._last_accepted = {kind: t + offset for kind, t in self._last_accepted.items()}

    def _relax(self, dt):
        cfg = self._config
        for axis in AXES:
            k = min(1.0, dt / cfg.relax_tau_s[axis])
            self._x[axis] += (cfg.temperament[axis] - self._x[axis]) * k

    def _idle_drift(self, dt):
        idle = self._config.idle
        if self._idle_s <= idle.after_s:
            return
        intensity = min(1.0, (self._idle_s - idle.after_s) / idle.ramp_s)
        for axis, rate in self._config.impulses[IDLE_KIND].deltas.items():
            self._x[axis] += rate * intensity * dt

    def _clamp_axes(self):
        for axis in AXES:
            lo, hi = AXIS_RANGES[axis]
            self._x[axis] = _clamp(self._x[axis], lo, hi)

    def _expire_lock(self, now):
        if (self._locked and self._lock_until is not None and
                now >= self._lock_until - TIME_EPS):
            self._unlock()
            return True
        return False

    def _unlock(self):
        self._locked = False
        self._lock_source = ''
        self._lock_until = None

    def _update_focus(self, subject, bearing_rad, ttl_s, now):
        subject = subject or ''
        try:
            bearing = float(bearing_rad)
        except (TypeError, ValueError):
            bearing = math.nan
        if not math.isfinite(bearing):
            bearing = math.nan
        if not subject and math.isnan(bearing):
            return False
        try:
            ttl = float(ttl_s)
        except (TypeError, ValueError):
            ttl = 0.0
        if not (math.isfinite(ttl) and ttl > 0.0):
            ttl = self._config.focus.default_ttl_s
        self._focus_subject = subject
        self._focus_bearing = bearing
        self._focus_until = now + ttl
        return True

    def _clear_focus(self):
        self._focus_subject = ''
        self._focus_bearing = math.nan
        self._focus_until = None

    def _nearest(self):
        """Return ``(name, distance)`` of the nearest mood; the first wins a tie."""
        best, best_d = None, math.inf
        weights = self._config.distance_weights
        for name, proto in self._config.moods.items():
            d = distance(self._x, proto.center, weights)
            if d < best_d:
                best, best_d = name, d
        return best, best_d

    def _label_distance(self):
        return distance(self._x, self._config.moods[self._mood].center,
                        self._config.distance_weights)

    def _strength(self):
        return _clamp(1.0 - self._label_distance() / self._config.strength_radius, 0.0, 1.0)

    def _classify(self, now):
        candidate, d_candidate = self._nearest()
        margin = self._config.hysteresis.margin
        if candidate == self._mood or not d_candidate < self._label_distance() - margin:
            self._pending = None
            return None
        if self._pending is None or self._pending[0] != candidate:
            self._pending = (candidate, now)            # a new candidate starts its own dwell
        if now - self._pending[1] < self._config.hysteresis.dwell_s - TIME_EPS:
            return None
        old, self._mood, self._pending = self._mood, candidate, None
        return MoodTransition(old=old, new=candidate, distance=d_candidate)
