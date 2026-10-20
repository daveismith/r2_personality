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
The arbiter model: channels, claims, leases, priorities, the device's holds, the watchdog.

Pure Python (no ROS imports): the arbiter nodes map messages onto it and turn
the :class:`Action` values it returns into service calls. Every call takes
``now`` (seconds, any monotonic clock); the first call sets the time base.
One :class:`Arbiter` serves one channel group: the LED node passes
``fld, tfld, bfld, rld, psi``, the Magic Panel node passes ``magic``.

Channels (plan 4.1). ``fld`` runs on surface ``fld`` (displays ``tfld`` and
``bfld``); ``tfld`` and ``bfld`` are its sub-channels, one board each; ``rld``
is ``rld``; ``psi`` fans out to surfaces ``fpsi`` and ``rpsi``; ``magic`` is the
panel. Two channels *overlap* when they share a display (``fld`` overlaps
``tfld`` and ``bfld``, which do not overlap each other). Claims and runs act
across overlapping channels, because the device replaces any run sharing a
display with a new one.

Rules (plan 4.2), each named by a test:

1. **Every request carries the owner and a non-zero priority** (rule 6).
   :class:`Action` refuses priority 0 (``ValueError``) and any ``hue``
   (rule 9). Device requests go out at the intent's priority.
2. **Ladder.** ``idle`` <= 10, ``behavior`` <= 100, ``manual`` <= 200,
   ``safety`` <= 255; a source above its slot, priority 0, an unknown source
   or an unknown action is dropped.
3. **Claims** (:meth:`Arbiter.claim`). Accepted unless another owner holds
   the channel, or an overlapping one, at an equal or higher priority (the
   refusal names it); the same owner renews; ``lease_s`` 0 releases; the
   lease is capped at ``max_lease_s``; expiry releases (on :meth:`tick`).
   Claims layer: a higher claim does not erase a lower one, which holds
   again if still leased when the higher one goes. An accepted claim
   pre-empts: every run of ours below its priority on an overlapping channel
   is stopped on the device at once.
4. **Intents** (:meth:`Arbiter.intent`), checked in order: ladder; vocabulary
   (:data:`LED_ACTIONS`, :data:`MAGIC_ACTIONS`); params JSON without ``hue``;
   an ``idle`` or ``behavior`` intent is dropped while its channel is foreign,
   dark or offline, an ``idle`` one also while the mood is stale; dropped below
   the holder's priority (a ``manual`` intent passes only its own auto-claim,
   owner ``manual``, which ClaimChannel may not use); a run (effect, clip,
   text, sequence) or stop is dropped below a running intent's priority, and a
   run at the same priority as a running ``interruptible: false`` intent is
   dropped. Every drop is counted in ``dropped`` and returned as the reason. A
   new run replaces every run on an overlapping channel; if the device refuses
   it outright, the runs it displaced are tracked again.
5. **Duration cap** (rule 3, decision 11.4). ``idle`` and ``behavior`` runs
   carry ``min(duration, max_duration_ms)``, ``max_duration_ms`` for 0; the
   personality never sends 0. ``manual`` and ``safety`` are exempt. LED runs
   end on the device by their duration; a Magic Panel run is stopped by the
   arbiter at its duration (the panel has none).
6. **Manual auto-claim.** A ``manual`` intent claims its channel as owner
   ``manual`` at its own priority (never lowered by a renewal), ending
   ``manual_hold_s`` after the later of the last intent and the end of its
   run: sized from the run's duration when it starts (forever, -1, for a run
   with none) and re-sized when the run ends. A manual configure, stop or
   brightness pre-empts lower runs.
7. **A person at the dome is the device's hold.** A ``held by console|ui|
   text topic`` or ``held: manual`` refusal (:meth:`Arbiter.device_reply`)
   or ``held_by`` on the state topic makes the channel FOREIGN with the
   holder as owner and ``hold_remaining_ms`` as the lease (a refusal alone:
   :data:`DEVICE_HOLD_S`; ``manual``: forever). A run on an overlapping
   surface that this arbiter did not start (another source, or another
   owner) keeps the channel FOREIGN after the hold, with lease 0, until the
   surface is idle. Nothing is sent to a foreign channel by the personality,
   and no run of another is ever stopped: a stop names a surface, so a
   surface the snapshot shows running someone else's run is skipped, whoever
   asked for the stop. A hold ends when its time runs out (or a snapshot
   without it arrives); a refusal's hold covers every display the refused
   request touched, so overlapping channels see it too.
8. **Palette** (plan 4.3, decision 11.6). A palette channel (``fld``,
   ``rld``, ``psi``, ``magic``) is re-styled at priority 10 when the label
   changes, when the mood goes stale (neutral) or comes back, on
   :meth:`reload`, and whenever the channel becomes eligible again: no
   claim on it or an overlapping channel, not foreign, dark or offline. So
   a release, the end of a hold (once the person's run has ended), and the
   device coming back all re-apply it. A display left in a mode other than
   random or effect is put back to random with the drift. The Magic Panel
   brightness is only sent while the panel's ``held_by`` is empty.
9. **Watchdog.** ``/mood/state`` older than ``mood_stale_s`` (or never seen
   ``mood_stale_s`` after the first call): neutral palette, no flourishes,
   status STALE when nothing else applies. No snapshot for
   ``device_stale_s``, no catalogue yet, or a Magic Panel link that is not
   present: OFFLINE. ``strip_enabled`` false: the LED channels are DARK.
10. **Flourishes** fire within the mood's ``every_s`` as idle intents through
    rule 4, so a foreign, dark or offline channel drops (and counts) them; a
    claim at any priority drops them too; none fire while stale.
11. **Own runs survive a restart.** On the first snapshot a run with
    ``source ros`` and our owner is adopted (source ``adopted``, priority 10)
    with ``max_duration_ms`` from now, then stopped if still running.
12. **State evidence.** A run of ours is cleared when a snapshot taken
    :data:`SNAPSHOT_GRACE_S` after its confirmation no longer shows it; a
    refusal-inferred hold gives way to a snapshot that arrives that long
    after the refusal.

Status precedence: OFFLINE, DARK, FOREIGN, RUNNING, CLAIMED, STALE, IDLE.
"""

from dataclasses import dataclass, field
import json
import math
import random
from types import MappingProxyType
from typing import Any, Mapping, Optional, Tuple

from r2_expression.config import BRIGHTNESS_RANGE, DRIFT_PRESETS, HUE_KEY, LED_STYLE_CHANNELS
from r2_expression.palette import (field_of as _field, FlourishScheduler, LedCatalogue,
                                   MagicCatalogue, pick_intent_fields, resolve_styles,
                                   validate_picks)

# ChannelStatus.STATE_*
STATE_IDLE = 0
STATE_RUNNING = 1
STATE_CLAIMED = 2
STATE_FOREIGN = 3
STATE_STALE = 4
STATE_OFFLINE = 5
STATE_DARK = 6
STATE_NAMES = ('idle', 'running', 'claimed', 'foreign', 'stale', 'offline', 'dark')

# ExpressionIntent.PRIORITY_* and the ladder: the highest priority each source may use.
PRIORITY_IDLE = 10
PRIORITY_BEHAVIOR = 100
PRIORITY_MANUAL = 200
PRIORITY_SAFETY = 255
LADDER = MappingProxyType({'idle': PRIORITY_IDLE, 'behavior': PRIORITY_BEHAVIOR,
                           'manual': PRIORITY_MANUAL, 'safety': PRIORITY_SAFETY})
CAPPED_SOURCES = ('idle', 'behavior')
STOPPING_SOURCES = ('manual', 'safety')   # may stop a channel with no run of ours

# Device calls an Action names; the request fields are those of the service.
CALL_RUN = 'run'                            # RunLedEffect
CALL_SAY = 'say'                            # SayLedText
CALL_STOP = 'stop'                          # StopLedEffect
CALL_CONFIGURE = 'configure'                # ConfigureLedDisplay
CALL_START = 'start'                        # StartMagicSequence
CALL_MAGIC_STOP = 'magic_stop'              # StopMagicSequence
CALL_MAGIC_BRIGHTNESS = 'magic_brightness'  # ConfigureMagicPanel (takes no priority)
CALLS = (CALL_RUN, CALL_SAY, CALL_STOP, CALL_CONFIGURE, CALL_START, CALL_MAGIC_STOP,
         CALL_MAGIC_BRIGHTNESS)
_RUN_CALLS = (CALL_RUN, CALL_SAY, CALL_START)

LED_ACTIONS = ('effect', 'clip', 'text', 'configure', 'stop')
MAGIC_ACTIONS = ('sequence', 'brightness', 'stop')
_RUN_ACTIONS = ('effect', 'clip', 'text', 'sequence')

# Device constants (astromech_msgs 2.0.0), repeated so this module needs no ROS.
APPLY_MODE = 1                 # ConfigureLedDisplay
APPLY_BRIGHTNESS = 2
APPLY_HUE = 4                  # never set (rule 9)
APPLY_DRIFT = 8
MAGIC_APPLY_BRIGHTNESS = 1     # ConfigureMagicPanel
MODE_RANDOM = 0                # LedDisplayState
MODE_EFFECT = 5
TEXT_MOTIONS = ('bounce', 'loop', 'left', 'right', 'up')   # SayLedText.MOTION_* order
END_VALUES = MappingProxyType({'leave': 0, 'blank': 1})    # StartMagicSequence.END_*
STOP_MODES = MappingProxyType({'blank': 0, 'freeze': 1})   # StopMagicSequence.STOP_*
MAX_UINT32 = 4294967295
HOLD_FOREVER = MAX_UINT32
LINK_PRESENT = 1
MAGIC_RUNNING = 1              # MagicPanelState.STATE_RUNNING
MAX_OWNER_BYTES = 31

# A refusal says who holds the display, not for how long: assume the device's 5 s.
DEVICE_HOLD_S = 5.0
# A snapshot younger than this after a confirmation or a refusal may predate it.
SNAPSHOT_GRACE_S = 1.5
MANUAL_OWNER = 'manual'
ADOPTED = 'adopted'

HELD_REFUSALS = MappingProxyType({'held by console': 'console', 'held by ui': 'ui',
                                  'held by text topic': 'text_topic',
                                  'held: manual': 'manual'})


@dataclass(frozen=True)
class Channel:
    """A channel: its device surfaces, the displays they cover, its palette channel."""

    name: str
    kind: str                     # led | magic
    surfaces: Tuple[str, ...]     # where its runs go
    displays: Tuple[str, ...]     # what they cover
    parent: str = ''              # fld for tfld and bfld

    @property
    def palette_channel(self):
        """Return the channel whose palette styles this one."""
        return self.parent or self.name

    def overlaps(self, other):
        """Return True when the two share a display."""
        return bool(set(self.displays) & set(other.displays))


CHANNELS = MappingProxyType({
    'fld': Channel('fld', 'led', ('fld',), ('tfld', 'bfld')),
    'tfld': Channel('tfld', 'led', ('tfld',), ('tfld',), parent='fld'),
    'bfld': Channel('bfld', 'led', ('bfld',), ('bfld',), parent='fld'),
    'rld': Channel('rld', 'led', ('rld',), ('rld',)),
    'psi': Channel('psi', 'led', ('fpsi', 'rpsi'), ('fpsi', 'rpsi')),
    'magic': Channel('magic', 'magic', ('magic',), ('magic',)),
})
LED_CHANNELS = ('fld', 'tfld', 'bfld', 'rld', 'psi')
MAGIC_CHANNELS = ('magic',)
_SURFACE_CHANNEL = MappingProxyType({'fpsi': 'psi', 'rpsi': 'psi'})
# The displays of every device surface (2.2), for runs reported on the state topic.
SURFACE_DISPLAYS = MappingProxyType({
    'fld': ('tfld', 'bfld'), 'tfld': ('tfld',), 'bfld': ('bfld',), 'rld': ('rld',),
    'fpsi': ('fpsi',), 'rpsi': ('rpsi',)})


# The device surfaces whose runs touch each channel's displays.
_TOUCHING = MappingProxyType({
    name: frozenset(s for s, displays in SURFACE_DISPLAYS.items()
                    if set(displays) & set(spec.displays))
    for name, spec in CHANNELS.items()})


def _valid_owner(owner):
    return bool(owner) and len(owner.encode('utf-8')) <= MAX_OWNER_BYTES


def _valid_priority(priority):
    return not isinstance(priority, bool) and isinstance(priority, int) and \
        1 <= priority <= 255


def channel_of(name):
    """Return the :class:`Channel` named, or holding surface ``fpsi``/``rpsi``; else None."""
    return CHANNELS.get(_SURFACE_CHANNEL.get(name, name))


@dataclass(frozen=True)
class Intent:
    """The fields of an ``ExpressionIntent`` (header aside)."""

    channel: str
    source: str
    priority: int
    action: str
    params: Any = ''              # JSON text, or a mapping
    duration_ms: int = 0
    interruptible: bool = True
    intent_id: str = ''


@dataclass(frozen=True)
class Action:
    """One device call to make: ``call`` with the service ``request`` fields."""

    call: str                     # CALL_*
    channel: str
    target: str                   # the surface or display; 'magic' for the panel
    request: Mapping[str, Any]
    owner: str
    priority: int                 # what the request carries (rule 6); never 0
    intent_id: str = ''
    why: str = ''                 # intent | palette | preempt | duration

    def __post_init__(self):
        if self.call not in CALLS:
            raise ValueError('unknown call %r' % self.call)
        if not _valid_priority(self.priority):
            raise ValueError('a device request needs priority 1-255, never 0 (rule 6): '
                             'got %r' % (self.priority,))
        if 'priority' in self.request and self.request['priority'] != self.priority:
            raise ValueError('request priority %r differs from %r'
                             % (self.request['priority'], self.priority))
        if not _valid_owner(self.owner):
            raise ValueError('owner must be 1-%d bytes' % MAX_OWNER_BYTES)
        if HUE_KEY in self.request or (self.call == CALL_CONFIGURE and
                                       self.request.get('apply', 0) & APPLY_HUE):
            raise ValueError('the personality never sets hue (rule 9)')
        object.__setattr__(self, 'request', MappingProxyType(dict(self.request)))


@dataclass(frozen=True)
class Status:
    """One channel as its arbiter sees it: the fields of ``ChannelStatus``."""

    channel: str
    state: int                    # STATE_*
    owner: str
    priority: int
    lease_remaining_s: float      # -1: forever
    active_source: str
    active_action: str
    active_intent_id: str
    active_remaining_ms: int      # 0: unknown or unbounded
    mood: str
    dropped: int
    refused: int


@dataclass(frozen=True)
class ClaimResult:
    """The answer to a claim, and the device calls it needs (pre-emption, palette)."""

    accepted: bool
    reason: str
    status: Optional[Status]      # None: not my channel
    actions: Tuple[Action, ...] = ()


@dataclass(frozen=True)
class IntentResult:
    """What one intent did: the device calls to make, or why it was dropped."""

    accepted: bool
    reason: str                   # '' when accepted (or a note such as 'nothing running')
    actions: Tuple[Action, ...]
    intent_id: str


@dataclass
class _Claim:
    owner: str
    priority: int
    until: float                  # inf while a manual run of unknown length runs
    auto: bool = False            # a manual intent's auto-claim
    floor: float = 0.0            # auto: its last intent plus manual_hold_s


@dataclass
class _Hold:
    owner: str                    # console | ui | text_topic | manual
    until: Optional[float]        # None: forever
    seen_at: float = 0.0


@dataclass
class _Run:
    intent_id: str
    source: str
    action: str
    priority: int
    interruptible: bool
    effect: str                   # what the state topic will show: name, 'text', sequence id
    surfaces: set
    until: Optional[float]        # None: unbounded (manual 0)
    device_ends: bool = True      # the device ends it at its duration; else the arbiter stops it
    confirmed_at: Optional[float] = None
    displaced: list = field(default_factory=list)   # (channel state, run) it replaced
    stopping: bool = False        # its duration stop is sent; cleared once accepted


@dataclass
class _ChannelState:
    spec: Channel
    claims: dict = field(default_factory=dict)      # owner -> _Claim
    run: Optional[_Run] = None
    dropped: int = 0
    refused: int = 0


class _Refuse(Exception):
    """An intent the arbiter cannot turn into a request; the message is the reason."""


def _as_intent(obj):
    if isinstance(obj, Intent):
        return obj
    return Intent(channel=_field(obj, 'channel', '') or '',
                  source=_field(obj, 'source', '') or '',
                  priority=int(_field(obj, 'priority', 0) or 0),
                  action=_field(obj, 'action', '') or '',
                  params=_field(obj, 'params', '') or '',
                  duration_ms=int(_field(obj, 'duration_ms', 0) or 0),
                  interruptible=bool(_field(obj, 'interruptible', True)),
                  intent_id=_field(obj, 'intent_id', '') or '')


def _params(raw):
    if isinstance(raw, Mapping):
        return dict(raw)
    if raw is None or raw == '':
        return {}
    if isinstance(raw, str):
        try:
            value = json.loads(raw)
        except ValueError as exc:
            raise _Refuse('params: not JSON: %s' % exc) from exc
        if isinstance(value, dict):
            return value
    raise _Refuse('params: must be a JSON object')


def _int_param(params, key, default, lo, hi):
    value = params.get(key, default)
    if isinstance(value, bool) or not isinstance(value, int) or not lo <= value <= hi:
        raise _Refuse('out of range')
    return value


def _rgb_param(params):
    """Return ``rgb``: an integer, or a string of six hex digits or ``0x`` and hex."""
    value = params.get('rgb', 0)
    if isinstance(value, str):
        text = value[2:] if value.lower().startswith('0x') else value
        if not text or (text is value and len(text) != 6) or \
                not all(c in '0123456789abcdefABCDEF' for c in text):
            raise _Refuse('out of range')
        value = int(text, 16)
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 0xFFFFFF:
        raise _Refuse('out of range')
    return value


def held_by(reason):
    """Return the holder a device refusal names (console, ui, text_topic, manual), or ''."""
    return HELD_REFUSALS.get((reason or '').strip(), '')


def _effect_base(effect):
    return (effect or '').split(':', 1)[0]   # cycle:<child> is the cycle run


class Arbiter:
    """The arbiter of one channel group (see the module docstring for the rules)."""

    def __init__(self, config, channels, owner='r2_expression', rng=None):
        specs = []
        for name in channels:
            spec = CHANNELS.get(name)
            if spec is None:
                raise ValueError('unknown channel %r' % (name,))
            specs.append(spec)
        kinds = {spec.kind for spec in specs}
        if len(kinds) != 1:
            raise ValueError('an arbiter serves one channel group (LEDs or the Magic Panel)')
        if not _valid_owner(owner):
            raise ValueError('owner must be 1-%d bytes' % MAX_OWNER_BYTES)
        self.kind = kinds.pop()
        self.owner = owner
        self._config = config
        self._rng = rng if rng is not None else random.Random()
        self._ch = {spec.name: _ChannelState(spec) for spec in specs}
        self._overlap = {name: tuple(o for o in self._ch.values() if o.spec.overlaps(cs.spec))
                         for name, cs in self._ch.items()}
        styled = LED_STYLE_CHANNELS if self.kind == 'led' else MAGIC_CHANNELS
        self._palette_channels = tuple(ch for ch in styled if ch in self._ch)
        self._catalogue = None
        self._picks = MappingProxyType({})
        self._warned = set()
        self._warnings = []
        # Device state, from the last snapshot.
        self._displays = {}           # LED display -> {mode, hold}
        self._refusal_holds = {}      # display (or 'magic') -> _Hold inferred from a refusal
        self._surfaces = {}           # LED surface -> {running, effect, source, owner}
        self._strip_enabled = True
        self._manual = False
        self._magic = None            # the panel's fields, or None before a snapshot
        self._snapshot_at = None
        self._adopted = False
        # Mood.
        self._label = ''
        self._strength = 0.0
        self._mood_at = None
        self._t0 = None
        self._last_now = 0.0
        self._was_stale = False
        self._needs_palette = set()
        self._eligible = {}
        self._flourish = FlourishScheduler(self._rng)
        self._serial = 0
        self._validate_picks()

    # --- read-only views -------------------------------------------------------------------

    @property
    def channels(self):
        """Return the channel names this arbiter answers for, in order."""
        return tuple(self._ch)

    @property
    def config(self):
        """Return the :class:`~r2_expression.config.ExpressionConfig` in use."""
        return self._config

    def status(self, channel, now):
        """Return the :class:`Status` of ``channel`` at ``now``."""
        now = self._clock(now)
        cs = self._ch.get(channel)
        if cs is None:
            raise ValueError('not my channel: %r' % (channel,))
        state = self._state(cs, now)
        owner, priority, lease = '', 0, 0.0
        foreign = self._foreign(cs, now) if state == STATE_FOREIGN else None
        if foreign is not None:
            owner, lease = foreign
        else:
            holder = self._holder(cs, now)
            if holder is not None:
                owner, priority = holder.owner, holder.priority
                lease = -1.0 if holder.until == math.inf else holder.until - now
        run = cs.run
        remaining = 0
        if run is not None and run.until is not None:
            remaining = max(0, int(round((run.until - now) * 1000.0)))
        if self._stale(now):
            mood = 'neutral'
        elif self._label:
            mood = self._label if self._label in self._config.moods else 'neutral'
        else:
            mood = ''
        return Status(channel=channel, state=state, owner=owner, priority=priority,
                      lease_remaining_s=lease,
                      active_source=run.source if run else '',
                      active_action=run.action if run else '',
                      active_intent_id=run.intent_id if run else '',
                      active_remaining_ms=remaining, mood=mood,
                      dropped=cs.dropped, refused=cs.refused)

    def statuses(self, now):
        """Return the :class:`Status` of every channel, in order."""
        return [self.status(name, now) for name in self._ch]

    def take_warnings(self):
        """Return and clear the warnings for the node to log (each is raised once)."""
        out, self._warnings = self._warnings, []
        return out

    # --- inputs ----------------------------------------------------------------------------

    def set_catalogue(self, catalogue, now):
        """Take the device's catalogue (``ListLeds``/``ListMagicPanel``); return actions."""
        now = self._clock(now)
        wanted = LedCatalogue if self.kind == 'led' else MagicCatalogue
        if not isinstance(catalogue, wanted):
            raise TypeError('a %s arbiter needs a %s' % (self.kind, wanted.__name__))
        self._catalogue = catalogue
        self._validate_picks()
        return self._settle(now)

    def reload(self, config, now=None):
        """Swap in a new, validated config, keeping claims, runs, holds and counts; re-style."""
        now = self._clock(self._last_now if now is None else now)
        self._config = config
        self._validate_picks()
        self._needs_palette.update(self._palette_channels)
        if self._label and not self._was_stale:
            self._restart_flourish(now)
        return self._settle(now)

    def mood(self, label, strength, now):
        """Take one ``/mood/state``; return the actions (a new label re-styles)."""
        now = self._clock(now)
        if not label:
            return []
        self._mood_at = now
        self._strength = strength
        if label != self._label:
            self._label = label
            if label not in self._config.moods:
                self._warn(('label', label), 'mood %r has no palette in expression.yaml; '
                                             'using neutral' % label)
            self._needs_palette.update(self._palette_channels)
            if not self._was_stale:
                self._restart_flourish(now)
        return self._settle(now)

    def claim(self, channel, owner, priority, lease_s, now):
        """Claim, renew or release (``lease_s`` 0) a channel; return a :class:`ClaimResult`."""
        now = self._clock(now)
        cs = self._ch.get(channel)
        if cs is None:
            return ClaimResult(False, 'not my channel', None)

        def refuse(reason):
            return ClaimResult(False, reason, self.status(channel, now))

        if not owner:
            return refuse('owner required')
        if owner == MANUAL_OWNER:
            return refuse("owner 'manual' is reserved for manual intents' auto-claim")
        try:
            lease_s = float(lease_s)
        except (TypeError, ValueError):
            lease_s = math.nan
        if not (math.isfinite(lease_s) and lease_s >= 0.0):
            return refuse('lease_s must be a finite number >= 0')
        if lease_s == 0.0:                    # a release, whatever priority it names
            if owner not in cs.claims:
                elsewhere = [o.spec.name for o in self._overlapping(cs) if owner in o.claims]
                if elsewhere:
                    return refuse('%r holds %s, not %s: release that channel'
                                  % (owner, elsewhere[0], channel))
            cs.claims.pop(owner, None)
            actions = self._settle(now)
            return ClaimResult(True, '', self.status(channel, now), tuple(actions))
        if not _valid_priority(priority):
            return refuse('priority must be 1-255')
        for other in self._overlapping(cs):
            for held in other.claims.values():
                if held.owner != owner and held.until > now and held.priority >= priority:
                    where = '' if other is cs else ' on %s' % other.spec.name
                    return refuse('claimed by %r at priority %d%s'
                                  % (held.owner, held.priority, where))
        cs.claims[owner] = _Claim(owner, priority,
                                  now + min(lease_s, self._config.defaults.max_lease_s))
        actions = self._preempt(cs, priority, now)
        actions += self._settle(now)
        return ClaimResult(True, '', self.status(channel, now), tuple(actions))

    def intent(self, intent, now):
        """Arbitrate one :class:`Intent` (or a mapping, or an ``ExpressionIntent``)."""
        now = self._clock(now)
        f = _as_intent(intent)
        cs = self._ch.get(f.channel)
        intent_id = f.intent_id or self._new_id('r2e')
        if cs is None:
            return IntentResult(False, 'not my channel', (), intent_id)

        def drop(reason):
            cs.dropped += 1
            return IntentResult(False, reason, (), intent_id)

        slot = LADDER.get(f.source)
        if slot is None:
            return drop('unknown source %r' % f.source)
        if f.priority == 0:
            return drop('priority 0 is never sent: the device would let it override a person')
        if not _valid_priority(f.priority):
            return drop('priority must be an integer 1-255, got %r' % (f.priority,))
        if f.priority > slot:
            return drop('priority %d is above the %s slot (%d)' % (f.priority, f.source, slot))
        if f.action not in (LED_ACTIONS if self.kind == 'led' else MAGIC_ACTIONS):
            return drop('unknown action %r' % f.action)
        try:
            params = _params(f.params)
        except _Refuse as exc:
            return drop(str(exc))
        if HUE_KEY in params:
            return drop("hue is the droid's colour scheme; the personality never sets it")
        if f.source in CAPPED_SOURCES:         # the personality, not an operator
            state = self._state(cs, now)
            if state in (STATE_FOREIGN, STATE_DARK, STATE_OFFLINE):
                return drop('channel is %s' % STATE_NAMES[state])
            if f.source == 'idle' and self._stale(now):
                return drop('mood is stale')
        # A manual intent passes its own auto-claim, never anyone else's higher claim.
        # Idle (the palette's layer) steps off any claim, even one at priority 10.
        holder = self._holder(cs, now, MANUAL_OWNER if f.source == 'manual' else '')
        if holder is not None and (f.priority < holder.priority or f.source == 'idle'):
            return drop('claimed by %r at priority %d' % (holder.owner, holder.priority))
        replaces = f.action in _RUN_ACTIONS
        for other in self._overlapping(cs):
            run = other.run
            if run is None:
                continue
            if run.priority > f.priority:     # configure too: brightness shows through
                return drop('%s intent %s running at priority %d'
                            % (run.source, run.intent_id, run.priority))
            if replaces and run.priority == f.priority and not run.interruptible:
                return drop('non-interruptible intent %s running' % run.intent_id)
        try:
            actions, run = self._render(cs, f, params, intent_id, now)
        except _Refuse as exc:
            return drop(str(exc))

        if run is not None:
            for other in self._overlapping(cs):
                if other.run is not None:     # the device replaces them, once it accepts
                    run.displaced.append((other, other.run))
                    self._end_run(other, now)
            cs.run = run
        pre = []
        if f.source == 'manual':
            hold_s = self._config.defaults.manual_hold_s
            auto = cs.claims.get(MANUAL_OWNER)
            live = auto is not None and auto.until > now
            cs.claims[MANUAL_OWNER] = _Claim(
                MANUAL_OWNER, max(f.priority, auto.priority) if live else f.priority,
                max(auto.until, now + hold_s) if live else now + hold_s,
                auto=True, floor=now + hold_s)          # renewal never lowers either
            self._size_auto_claim(cs)
            if not replaces:
                pre = self._preempt(cs, f.priority, now)
        note = '' if actions else 'nothing running'
        return IntentResult(True, note, tuple(pre + actions), intent_id)

    def device_reply(self, action, accepted, reason, now):
        """
        Take the device's answer to ``action``; return a line to log ('' for none).

        A hold refusal makes the channel FOREIGN and counts in ``refused``; any
        refusal takes the action's surface out of its run (the run's other
        surfaces keep going: the device's ``all`` rule).
        """
        now = self._clock(now)
        cs = self._ch.get(action.channel)
        if cs is None:
            return ''
        reason = reason or ''
        run = cs.run if cs.run is not None and cs.run.intent_id == action.intent_id else None
        what = '%s: %s on %s' % (action.channel, action.call, action.target)
        if accepted:
            if run is not None and action.call in _RUN_CALLS:
                run.confirmed_at = now
                run.displaced = []
            elif run is not None and run.stopping and action.call in (CALL_STOP,
                                                                      CALL_MAGIC_STOP):
                run.surfaces.discard(action.target)
                if not run.surfaces:
                    self._end_run(cs, now)
            return '%s: %s' % (what, reason) if reason else ''
        if run is not None and action.call in _RUN_CALLS:
            run.surfaces.discard(action.target)
            if not run.surfaces:
                self._end_run(cs, now)
                # Refused outright: what it would have replaced is still running.
                for other, old in run.displaced:
                    if other.run is None and old.confirmed_at is not None:
                        other.run = old
                        self._size_auto_claim(other)
        holder = held_by(reason)
        if holder:
            cs.refused += 1
            hold = _Hold(holder, None if holder == MANUAL_OWNER else now + DEVICE_HOLD_S, now)
            for display in SURFACE_DISPLAYS.get(action.target, (action.target,)):
                self._refusal_holds[display] = hold     # overlapping channels see it too
            line = '%s refused %r: channel foreign (%s)' % (what, reason, holder)
        else:
            line = '%s failed: %s' % (what, reason or 'no reason given')
        if cs.run is run and run is not None and run.surfaces:
            line += '; %s keeps its run' % ', '.join(sorted(run.surfaces))
        return line

    def led_state(self, snapshot, now):
        """Take one ``LedStateArray`` (a message or a dict of its fields); return actions."""
        now = self._clock(now)
        self._require('led')
        displays = {}
        for d in _field(snapshot, 'displays', ()) or ():
            displays[_field(d, 'name')] = {
                'mode': int(_field(d, 'mode', MODE_RANDOM) or 0),
                'hold': self._snapshot_hold(_field(d, 'held_by', ''),
                                            _field(d, 'hold_remaining_ms', 0), now)}
        surfaces = {}
        for s in _field(snapshot, 'surfaces', ()) or ():
            surfaces[_field(s, 'name')] = {
                'running': bool(_field(s, 'running', False)),
                'effect': _field(s, 'effect', '') or '',
                'source': _field(s, 'source', '') or '',
                'owner': _field(s, 'owner', '') or ''}
        self._displays, self._surfaces = displays, surfaces
        self._strip_enabled = bool(_field(snapshot, 'strip_enabled', True))
        self._manual = bool(_field(snapshot, 'manual', False))
        self._took_snapshot(now)
        for cs in self._ch.values():
            run = cs.run
            if run is None or not self._evidence(run, now):
                continue
            for name in list(run.surfaces):
                snap = surfaces.get(name)
                if not (snap and snap['running'] and self._ours(snap) and
                        _effect_base(snap['effect']) == _effect_base(run.effect)):
                    run.surfaces.discard(name)
            if not run.surfaces:
                self._end_run(cs, now)
        return self._settle(now)

    def magic_state(self, snapshot, now):
        """Take one ``MagicPanelState`` (a message or a dict of its fields); return actions."""
        now = self._clock(now)
        self._require('magic')
        manual = bool(_field(snapshot, 'manual', False))
        self._magic = {
            'link': int(_field(snapshot, 'link', LINK_PRESENT)),
            'running': int(_field(snapshot, 'state', 0) or 0) == MAGIC_RUNNING,
            'source': _field(snapshot, 'started_by', '') or '',
            'owner': _field(snapshot, 'owner', '') or '',
            'effect': str(_field(snapshot, 'sequence_id', '')),
            'hold': (_Hold(MANUAL_OWNER, None, now) if manual else
                     self._snapshot_hold(_field(snapshot, 'held_by', ''),
                                         _field(snapshot, 'hold_remaining_ms', 0), now))}
        self._took_snapshot(now)
        cs = self._ch['magic']
        run, m = cs.run, self._magic
        if run is not None and self._evidence(run, now) and not (
                m['running'] and self._ours(m) and m['effect'] == run.effect):
            self._end_run(cs, now)
        return self._settle(now)

    def tick(self, now):
        """Advance timers to ``now``: leases, holds, durations, staleness, flourishes."""
        now = self._clock(now)
        actions = []
        for cs in self._ch.values():
            for owner in [o for o, c in cs.claims.items() if c.until <= now]:
                del cs.claims[owner]
            run = cs.run
            if run is not None and run.until is not None and now >= run.until:
                if not run.device_ends:
                    if self._hold(cs, now) is not None:
                        continue              # a person holds it: stop once they let go
                    stops = self._stops(cs, run.surfaces, run.priority, run.intent_id,
                                        'duration')
                    if stops:
                        # Kept until the device takes every stop; retried after a hold.
                        actions += stops
                        run.stopping = True
                        run.until = now + DEVICE_HOLD_S
                        self._size_auto_claim(cs)
                        continue
                self._end_run(cs, now)
        actions += self._settle(now)
        if not self._was_stale and self._label in self._config.moods:
            pick = self._flourish.poll(self._picks.get(self._label, ()), now)
            if pick is not None and pick.channel in self._ch:
                actions += self.intent(Intent(**pick_intent_fields(
                    pick, self._new_id('flourish'))), now).actions
        return actions

    # --- internals -------------------------------------------------------------------------

    def _clock(self, now):
        now = float(now)
        if not math.isfinite(now):
            raise ValueError('now must be a finite time in seconds, got %r' % now)
        if self._t0 is None:
            self._t0 = now
        self._last_now = now
        return now

    def _require(self, kind):
        if self.kind != kind:
            raise TypeError('a %s arbiter takes no %s state' % (self.kind, kind))

    def _new_id(self, prefix):
        self._serial += 1
        return '%s-%d' % (prefix, self._serial)

    def _warn(self, key, text):
        if key not in self._warned:
            self._warned.add(key)
            self._warnings.append(text)

    def _validate_picks(self):
        led = self._catalogue if self.kind == 'led' else None
        magic = self._catalogue if self.kind == 'magic' else None
        self._picks, warnings = validate_picks(self._config, led=led, magic=magic)
        for key, text in warnings:
            self._warn(key, text)

    def _restart_flourish(self, now):
        mood = self._config.moods.get(self._label)
        self._flourish.restart(mood.flourish if mood is not None else None, now)

    def _overlapping(self, cs):
        return self._overlap[cs.spec.name]

    def _ours(self, snap):
        return snap['source'] == 'ros' and snap['owner'] == self.owner

    def _evidence(self, run, now):
        return run.confirmed_at is not None and now - run.confirmed_at >= SNAPSHOT_GRACE_S

    @staticmethod
    def _snapshot_hold(holder, remaining_ms, now):
        if not holder:
            return None
        remaining_ms = int(remaining_ms or 0)
        return _Hold(holder, None if remaining_ms >= HOLD_FOREVER
                     else now + remaining_ms / 1000.0, now)

    def _took_snapshot(self, now):
        self._snapshot_at = now
        self._refusal_holds = {d: h for d, h in self._refusal_holds.items()
                               if now - h.seen_at < SNAPSHOT_GRACE_S}
        if not self._adopted:
            self._adopted = True
            self._adopt(now)

    def _adopt(self, now):
        """Rule 11: take over the runs a previous arbiter with our owner left running."""
        until = now + self._config.defaults.max_duration_ms / 1000.0
        for cs in self._ch.values():
            if cs.run is not None:
                continue
            if self.kind == 'magic':
                snaps = {'magic': self._magic}
            else:
                snaps = {s: self._surfaces.get(s) for s in cs.spec.surfaces}
            ours = {name: s for name, s in snaps.items()
                    if s and s['running'] and self._ours(s)}
            if not ours:
                continue
            effect = next(iter(ours.values()))['effect']
            action = ('sequence' if self.kind == 'magic'
                      else 'text' if effect == 'text' else 'effect')
            cs.run = _Run(ADOPTED, ADOPTED, action, PRIORITY_IDLE, True, effect, set(ours),
                          until, device_ends=False, confirmed_at=now - SNAPSHOT_GRACE_S)

    def _offline(self, now):
        if self._catalogue is None or self._snapshot_at is None:
            return True
        if now - self._snapshot_at > self._config.defaults.device_stale_s:
            return True
        return self.kind == 'magic' and self._magic['link'] != LINK_PRESENT

    def _stale(self, now):
        since = self._t0 if self._mood_at is None else self._mood_at
        return now - since > self._config.defaults.mood_stale_s

    def _hold(self, cs, now):
        """
        Return the device's hold on the channel's displays, or None.

        The snapshot's holds win; without one, a recent refusal's. A hold ends
        when its time runs out, before the next snapshot confirms it.
        """
        if self.kind == 'magic':
            snapped = [self._magic['hold']] if self._magic else []
        elif self._manual:
            return _Hold(MANUAL_OWNER, None)
        else:
            snapped = [self._displays[d]['hold'] for d in cs.spec.displays
                       if d in self._displays]
        holds = [h for h in snapped if h is not None and (h.until is None or now < h.until)]
        if not holds:
            holds = [h for h in (self._refusal_holds.get(d) for d in cs.spec.displays)
                     if h is not None and (h.until is None or now < h.until)]
        if not holds:
            return None
        owners = [h.owner for h in holds]
        ends = [h.until for h in holds]
        return _Hold(MANUAL_OWNER if MANUAL_OWNER in owners else owners[0],
                     None if None in ends else max(ends))

    def _foreign_run_owner(self, cs):
        """Who runs something on the channel's displays that we did not start, or ''."""
        if self.kind == 'magic':
            snaps = [self._magic] if self._magic else []
        else:
            touching = _TOUCHING[cs.spec.name]
            snaps = [s for name, s in self._surfaces.items() if name in touching]
        for snap in snaps:
            if snap['running'] and not self._ours(snap):
                if snap['source'] == 'ros':
                    return snap['owner'] or 'ros'
                return snap['source'] or 'other'
        return ''

    def _foreign(self, cs, now):
        """Return ``(owner, lease_s)`` when the channel is foreign, else None."""
        hold = self._hold(cs, now)
        if hold is not None:
            return hold.owner, -1.0 if hold.until is None else max(0.0, hold.until - now)
        owner = self._foreign_run_owner(cs)
        return (owner, 0.0) if owner else None

    def _holder(self, cs, now, ignore=''):
        """Return the highest live claim on the channel or an overlapping one."""
        best = None
        for other in self._overlapping(cs):
            for held in other.claims.values():
                if held.owner != ignore and held.until > now and (
                        best is None or held.priority > best.priority):
                    best = held
        return best

    def _blocked(self, cs, now):
        """Return OFFLINE, DARK or FOREIGN when one holds the channel off, else None."""
        if self._offline(now):
            return STATE_OFFLINE
        if self.kind == 'led' and not self._strip_enabled:
            return STATE_DARK
        if self._foreign(cs, now) is not None:
            return STATE_FOREIGN
        return None

    def _state(self, cs, now):
        blocked = self._blocked(cs, now)
        if blocked is not None:
            return blocked
        if cs.run is not None:
            return STATE_RUNNING
        if self._holder(cs, now) is not None:
            return STATE_CLAIMED
        if self._stale(now):
            return STATE_STALE
        return STATE_IDLE

    def _end_run(self, cs, now):
        """
        Forget ``cs``'s run: every run ends here.

        Rule 6: a manual run's auto-claim then ends ``manual_hold_s`` after the
        later of now and the last manual intent.
        """
        run, cs.run = cs.run, None
        auto = cs.claims.get(MANUAL_OWNER)
        if run is not None and run.source == 'manual' and auto is not None and auto.auto:
            auto.until = max(auto.floor, now + self._config.defaults.manual_hold_s)

    def _size_auto_claim(self, cs):
        """Rule 6: a manual run keeps its auto-claim until manual_hold_s after its end."""
        auto, run = cs.claims.get(MANUAL_OWNER), cs.run
        if auto is not None and auto.auto and run is not None and run.source == 'manual':
            end = math.inf if run.until is None else run.until
            auto.until = max(auto.until, end + self._config.defaults.manual_hold_s)

    def _preempt(self, cs, priority, now):
        actions = []
        for other in self._overlapping(cs):
            run = other.run
            if run is not None and run.priority < priority:
                actions += self._stops(other, run.surfaces, run.priority, run.intent_id,
                                       'preempt')
                self._end_run(other, now)
        return actions

    def _stops(self, cs, surfaces, priority, intent_id, why, mode='blank'):
        """
        Return the stop calls for ``surfaces``; never one for another's run (rule 7).

        A stop names a surface, not a run, so a surface the snapshot shows
        running a run we did not start is skipped.
        """
        actions = []
        for surface in sorted(surfaces):
            if self.kind == 'magic':
                if self._foreign_run_owner(cs):
                    continue
                call, request = CALL_MAGIC_STOP, {'mode': STOP_MODES[mode],
                                                  'priority': priority}
            else:
                snap = self._surfaces.get(surface)
                if snap and snap['running'] and not self._ours(snap):
                    continue
                call, request = CALL_STOP, {'surface': surface, 'priority': priority}
            actions.append(Action(call, cs.spec.name, surface, request, self.owner,
                                  priority, intent_id, why))
        return actions

    def _configure(self, channel, display, drift, brightness, priority, intent_id, why,
                   restore_mode=False):
        apply = 0
        if brightness is not None:
            apply |= APPLY_BRIGHTNESS
        if drift:
            apply |= APPLY_DRIFT
        if restore_mode:
            apply |= APPLY_MODE
        return Action(CALL_CONFIGURE, channel, display,
                      {'display': display, 'apply': apply, 'mode': MODE_RANDOM,
                       'brightness': brightness or 0, 'drift': drift or '',
                       'owner': self.owner, 'priority': priority},
                      self.owner, priority, intent_id, why)

    def _duration(self, f, params):
        duration = f.duration_ms or params.get('duration_ms', 0)
        if isinstance(duration, bool) or not isinstance(duration, int) or \
                not 0 <= duration <= MAX_UINT32:
            raise _Refuse('out of range')
        if f.source in CAPPED_SOURCES:
            cap = self._config.defaults.max_duration_ms
            duration = min(duration, cap) if duration else cap
        return duration

    def _render(self, cs, f, params, intent_id, now):
        """Return ``(actions, run)`` for an intent that passed the gates."""
        spec, pr, owner = cs.spec, f.priority, self.owner

        def action(call, target, request):
            return Action(call, spec.name, target, request, owner, pr, intent_id, 'intent')

        def new_run(effect, surfaces, duration):
            return _Run(intent_id, f.source, f.action, pr, f.interruptible, effect,
                        set(surfaces), None if not duration else now + duration / 1000.0,
                        device_ends=spec.kind == 'led')

        if f.action in ('effect', 'clip'):
            name = params.get('name')
            if not isinstance(name, str) or not name:
                raise _Refuse('unknown effect')
            rgb, speed = _rgb_param(params), _int_param(params, 'speed', 0, 0, 9)
            duration = self._duration(f, params)
            return ([action(CALL_RUN, s, {'surface': s, 'effect': name, 'rgb': rgb,
                                          'speed': speed, 'duration_ms': duration,
                                          'owner': owner, 'priority': pr})
                     for s in spec.surfaces], new_run(name, spec.surfaces, duration))
        if f.action == 'text':
            if spec.name == 'psi':
                raise _Refuse('display has no text mode')
            lines = params.get('lines')
            if not isinstance(lines, list) or not lines or \
                    not all(isinstance(line, str) for line in lines):
                raise _Refuse('no text')
            if len(lines) > len(spec.displays):
                raise _Refuse('more lines than the surface has displays')
            motion = params.get('motion', 'bounce')
            if motion not in TEXT_MOTIONS:
                raise _Refuse('unknown text motion')
            rgb, speed = _rgb_param(params), _int_param(params, 'speed', 0, 0, 9)
            duration = self._duration(f, params)
            (surface,) = spec.surfaces
            return ([action(CALL_SAY, surface, {
                'surface': surface, 'lines': list(lines),
                'linked': bool(params.get('linked', False)),
                'motion': TEXT_MOTIONS.index(motion),
                'on_black': bool(params.get('on_black', False)), 'rgb': rgb, 'speed': speed,
                'duration_ms': duration, 'owner': owner, 'priority': pr})],
                new_run('text', spec.surfaces, duration))
        if f.action == 'configure':
            drift, brightness = params.get('drift'), params.get('brightness')
            if drift is None and brightness is None:
                raise _Refuse('configure needs drift or brightness')
            if drift is not None:
                if spec.name == 'psi':
                    raise _Refuse('display has no drift')
                if drift not in DRIFT_PRESETS:
                    raise _Refuse('unknown drift preset')
            if brightness is not None:
                brightness = _int_param(params, 'brightness', 0, *BRIGHTNESS_RANGE['fld'])
            return ([self._configure(spec.name, d, drift, brightness, pr, intent_id, 'intent')
                     for d in spec.displays], None)
        if f.action == 'sequence':
            ref = params.get('id', params.get('name'))
            if isinstance(ref, str):
                ref = self._catalogue.resolve(ref) if self._catalogue is not None else None
            if isinstance(ref, bool) or not isinstance(ref, int):
                raise _Refuse('unknown sequence')
            if not 0 <= ref <= 255:
                raise _Refuse('out of range')
            repeat = _int_param(params, 'repeat', 1, 0, 255)
            end = params.get('end', 0)
            end = END_VALUES.get(end, end) if isinstance(end, str) else end
            if end not in (0, 1) or isinstance(end, bool):
                raise _Refuse('out of range')
            duration = self._duration(f, params)
            return ([action(CALL_START, 'magic', {'sequence_id': ref, 'repeat': repeat,
                                                  'end': end, 'owner': owner,
                                                  'priority': pr})],
                    new_run(str(ref), ('magic',), duration))
        if f.action == 'brightness':
            value = _int_param(params, 'value', None, *BRIGHTNESS_RANGE['magic'])
            hold = self._hold(cs, now)
            if hold is not None:
                raise _Refuse('held by %s: ConfigureMagicPanel takes no priority, so the '
                              'arbiter does not send it' % hold.owner)
            return ([action(CALL_MAGIC_BRIGHTNESS, 'magic',
                            {'apply': MAGIC_APPLY_BRIGHTNESS, 'brightness': value})], None)
        # stop: our runs on the channel and on every overlapping one (the gate above has
        # made sure none outranks the stop); with none, manual and safety stop the
        # channel's surfaces. Either way never a surface running another's run.
        mode = params.get('mode', 'blank') if spec.kind == 'magic' else 'blank'
        if not isinstance(mode, str) or mode not in STOP_MODES:
            raise _Refuse('unknown stop mode')
        ours = [o for o in self._overlapping(cs) if o.run is not None]
        if not ours:
            if f.source not in STOPPING_SOURCES:
                return [], None
            return self._stops(cs, spec.surfaces, pr, intent_id, 'intent', mode), None
        actions = []
        for other in ours:
            actions += self._stops(other, other.run.surfaces, pr, intent_id, 'intent', mode)
            self._end_run(other, now)
        return actions, None

    def _palette_eligible(self, cs, now):
        """Return True when nothing keeps the palette off: a blocking state, a claim, a run."""
        return (self._blocked(cs, now) is None and
                self._holder(cs, now) is None and
                not any(o.run is not None and o.run.priority > PRIORITY_IDLE
                        for o in self._overlapping(cs)))

    def _settle(self, now):
        """Rule 8 and 9: follow staleness, and re-style every channel that needs it."""
        stale = self._stale(now)
        if stale != self._was_stale:
            self._was_stale = stale
            self._needs_palette.update(self._palette_channels)
            if stale:
                self._flourish.stop()
            else:
                self._restart_flourish(now)
        actions = []
        for name in self._palette_channels:
            cs = self._ch[name]
            eligible = self._palette_eligible(cs, now)
            if eligible and not self._eligible.get(name, False):
                self._needs_palette.add(name)
            self._eligible[name] = eligible
            if eligible and name in self._needs_palette and (stale or self._label):
                actions += self._palette_actions(cs, stale)
                self._needs_palette.discard(name)
        return actions

    def _palette_actions(self, cs, stale):
        style = resolve_styles(self._config, self._label, self._strength,
                               stale).get(cs.spec.name)
        if style is None:
            return []
        if self.kind == 'magic':
            if style.brightness is None:
                return []
            return [Action(CALL_MAGIC_BRIGHTNESS, 'magic', 'magic',
                           {'apply': MAGIC_APPLY_BRIGHTNESS, 'brightness': style.brightness},
                           self.owner, PRIORITY_IDLE, '', 'palette')]
        actions = []
        for display in cs.spec.displays:
            snap = self._displays.get(display)
            restore = snap is not None and snap['mode'] not in (MODE_RANDOM, MODE_EFFECT)
            actions.append(self._configure(cs.spec.name, display, style.drift,
                                           style.brightness, PRIORITY_IDLE, '', 'palette',
                                           restore_mode=restore))
        return actions
