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
``fld, tfld, bfld, rld, psi``, the Magic Panel node passes ``magic``, the dome
node passes ``dome``, the sound node passes ``sound``.

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
   personality never sends 0, except a sound play of a known clip no longer
   than the cap, whose 0 is the whole clip (rule 21). ``manual`` and
   ``safety`` are exempt. LED runs end on the device by their duration; a
   Magic Panel run is stopped by the arbiter at its duration (the panel has
   none).
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
   brightness is only sent while the panel's ``held_by`` is empty. A palette
   request the device fails for any other reason (a timeout, a refusal that
   is not a hold) is sent again after ``device_stale_s``; a new label sends
   at once.
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

The dome (phase 2 plan 4.3). ``dome`` is a channel with no device services:
its "device" is ``dome_twist_converter`` and the controller behind it. Its runs
are *moves* (:mod:`r2_expression.motion`), and its one call is
:data:`CALL_VELOCITY`, ``{velocity_rad_s}`` for ``/dome/cmd/personality``,
where 0 is the stop. The node feeds :meth:`Arbiter.dome_state` (the
converter's source, the joint position, the body's speed and yaw), calls
:meth:`Arbiter.tick` at 20 Hz while :attr:`Arbiter.moving` and with each
joint state (10 Hz) otherwise, and publishes every velocity returned. Rules 1-4, 6 and 8-10 apply
as above, with these differences, each named by a test:

13. **Vocabulary** (:data:`DOME_ACTIONS`): ``look_at`` ``{bearing, profile}``,
    ``glance`` ``{amplitude, dwell_s, side}``, ``turn`` ``{delta}``, ``rest``
    ``{bearing}`` or ``{}`` (the mood's ``rest_rad``), ``stop``; ``track`` is
    reserved and refused ``not in this phase``. Bearings are +ve left, 0
    forward; malformed params are refused ``bad params: ...``. A move takes the
    mood's profile when idle, else ``params.profile`` (a named profile or
    ``mood``). ``duration_ms`` is ignored but for a glance without ``dwell_s``,
    whose dwell it is; the duration cap (rule 5) does not apply: every move is
    bounded by ``max_move_s`` (plus a glance's dwell), which
    ``active_remaining_ms`` counts down.
14. **Every move ends with a 0**: when it ends by itself (``why`` ``done``),
    by a stop, a claim's pre-emption, the channel going foreign, offline or
    suppressed, or the mood going stale. A 0 goes to the converter's
    lowest-priority input and cannot override a person, so it is sent even to
    a foreign channel. A new move of equal or higher priority replaces the one
    in progress without a 0 (the velocity changes; the new move takes over the
    old one's rate limiter and samples); ``interruptible: false`` holds a move
    against equal priority. The coast scale a move learns (motion rule 5) is
    carried to the next move, fading toward 1.0 (:class:`motion.CoastMemory`).
15. **Foreign.** The converter's source ``joystick`` or ``foxglove`` makes the
    channel FOREIGN at once, owner the source, lease 0, and abandons the move
    (it is not resumed). It is IDLE again when the source is ``idle`` or
    ``personality``; the mood's rest, if any, is re-applied then (rule 8),
    and idle glances wait ``after_foreign_s``. Moves of every source are
    refused while foreign or offline.
16. **Suppressed.** The body faster than ``body_speed_m_s`` or yawing faster
    than ``body_yaw_rad_s``: SUPPRESSED until ``after_still_s`` after it is
    still. The onset stops the move in progress; while it lasts ``idle`` and
    ``behavior`` intents are dropped and counted, ``manual`` and ``safety``
    ones still run. With ``body_silence_s`` (the drive's velocity command,
    which falls silent when nobody drives) a silence that long is a still
    reading; without it (odometry) only a still reading ends it, and
    odometry that falls silent while the body moves leaves the dome
    SUPPRESSED (the safe side).
17. **Offline.** No joint position, or no converter status, for
    ``device_stale_s`` (or never): OFFLINE, the move ends with its 0, nothing
    else is sent.
18. **Stale.** The mood going stale stops an ``idle`` or ``behavior`` move;
    no glances while stale; ``manual`` still runs.
19. **Idle glances and rest** (plan 4.5). The mood's ``rest_rad`` is the
    dome's palette: a ``rest`` at idle priority whenever rule 8 re-styles.
    Glances fire from :class:`~r2_expression.palette.GlanceScheduler` as idle
    intents through rule 4 (a claimed, foreign or suppressed channel drops and
    counts them), never during a move; the schedule restarts when a move
    ends. A resting dome that drifts beyond the profile's band is corrected
    at ``min_speed``, at most once per ``rest_recheck_s``.

Sound (phase 3 plan 4.2). ``sound`` is a channel on ``sound_player``, a device with
services (:data:`CALL_PLAY`, :data:`CALL_VOCALIZE`, :data:`CALL_SOUND_STOP`) and a state topic,
fed to :meth:`Arbiter.sound_state`. It has no holds, no console and no foreign state: the
player does one thing at a time and replaces what plays when asked. Rules 1-4, 6, 10 and 11 apply
as above, with these differences, each named by a test:

20. **Vocabulary** (:data:`SOUND_ACTIONS`): ``play`` ``{name}`` or ``{pool}`` with an optional
    ``gain_db``, ``vocalize`` ``{arousal, valence, length_s, seed}`` (refused ``synth not
    available`` while the player has no synthesiser; ``arousal`` and ``valence`` default to
    the mood's, from ``/mood/state``'s -1..1 mapped to 0..1, each axis keeping its last
    finite value; 0.5 before one and while stale), ``stop``; ``clip``
    is refused ``use play``. A name or pool the catalogue lacks is refused ``unknown name``
    / ``unknown pool``. A personality play carries the mood's ``gain_db`` added to the
    intent's; a manual one its own alone.
21. **Interruptibility and duration.** A run at the same priority is replaced only if its
    intent said ``interruptible: true``; an intent that does not say takes
    ``defaults.sound.interruptible`` (false: a cut phrase sounds like a fault). A higher
    priority always pre-empts, the player fading the old sound out. ``idle`` and ``behavior``
    plays carry ``duration_ms`` = ``max_duration_ms`` when it is shorter than the clip, 0 (the
    whole clip) when the clip is known and no longer than the cap, and ``max_duration_ms`` for
    a pool play (clip not known yet); ``manual`` plays are uncapped and 0 is the clip's length.
    A phrase's ``length_s`` is capped the same way; 0 (the synth's own, at most
    :data:`PHRASE_OWN_MAX_S`) stays 0 unless the intent's ``duration_ms`` sets it or the cap
    is shorter.
22. **Chatter.** The mood's ``chatter`` is the flourish scheduler: due within ``every_s``
    (``mood_strength`` pulling the window's top down), it draws a pool by weight, then a clip
    uniformly from it, and plays it as an idle intent; a ``voice`` pick is a ``vocalize``
    of the mood's arousal and valence instead. A slot that falls while the channel is
    claimed, stale, offline or playing is skipped.
23. **Stale.** The mood going stale stops a running ``idle`` play (with the player's fade)
    and silences the chatter; ``behavior`` and ``manual`` plays go on.
24. **State evidence.** A run of ours the player shows playing (owner and intent id) ends
    where ``remaining_ms`` says, ``active_remaining_ms`` counts it down, and one it no longer
    shows ends. A play of our owner found on the first state is adopted (rule 11). The
    channel is OFFLINE with no catalogue or no state for ``device_stale_s``; a run ends then.

Status precedence: OFFLINE, DARK, FOREIGN, SUPPRESSED, RUNNING, CLAIMED, STALE, IDLE.
"""

from dataclasses import dataclass, field, replace
import json
import math
import random
from types import MappingProxyType
from typing import Any, Mapping, Optional, Tuple

from r2_expression import motion
from r2_expression.config import (BRIGHTNESS_RANGE, DRIFT_PRESETS, HUE_KEY,
                                  LED_STYLE_CHANNELS, MOOD_PROFILE, VOICE_POOL)
from r2_expression.palette import (ChatterScheduler, choose_clip, field_of as _field,
                                   FlourishScheduler, glance_intent_fields, GlanceScheduler,
                                   LedCatalogue, MagicCatalogue, pick_intent_fields,
                                   random_side, resolve_styles, SoundCatalogue, uniform,
                                   validate_picks, validate_voices)

# ChannelStatus.STATE_*
STATE_IDLE = 0
STATE_RUNNING = 1
STATE_CLAIMED = 2
STATE_FOREIGN = 3
STATE_STALE = 4
STATE_OFFLINE = 5
STATE_DARK = 6
STATE_SUPPRESSED = 7
STATE_NAMES = ('idle', 'running', 'claimed', 'foreign', 'stale', 'offline', 'dark',
               'suppressed')

# ExpressionIntent.PRIORITY_* and the ladder: the highest priority each source may use.
PRIORITY_IDLE = 10
PRIORITY_BEHAVIOR = 100
PRIORITY_MANUAL = 200
PRIORITY_SAFETY = 255
LADDER = MappingProxyType({'idle': PRIORITY_IDLE, 'behavior': PRIORITY_BEHAVIOR,
                           'manual': PRIORITY_MANUAL, 'safety': PRIORITY_SAFETY})
# An excluded clip plays by name from here up, in the arbiter and in the player alike: the
# same value as r2_sound.catalogue.MANUAL_PRIORITY (no import: r2_sound is not a dependency).
MANUAL_PLAY_PRIORITY = PRIORITY_MANUAL - 1
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
CALL_VELOCITY = 'velocity'                  # /dome/cmd/personality; 0 is the stop
CALL_PLAY = 'play'                          # PlaySound
CALL_VOCALIZE = 'vocalize'                  # Vocalize
CALL_SOUND_STOP = 'sound_stop'              # StopSound (takes an owner, no priority)
CALLS = (CALL_RUN, CALL_SAY, CALL_STOP, CALL_CONFIGURE, CALL_START, CALL_MAGIC_STOP,
         CALL_MAGIC_BRIGHTNESS, CALL_VELOCITY, CALL_PLAY, CALL_VOCALIZE, CALL_SOUND_STOP)
RUN_CALLS = (CALL_RUN, CALL_SAY, CALL_START, CALL_PLAY, CALL_VOCALIZE)

LED_ACTIONS = ('effect', 'clip', 'text', 'configure', 'stop')
MAGIC_ACTIONS = ('sequence', 'brightness', 'stop')
DOME_ACTIONS = ('look_at', 'glance', 'turn', 'rest', 'stop', 'track')
SOUND_ACTIONS = ('play', 'vocalize', 'stop', 'clip')     # clip: refused, `use play`
_MOVE_ACTIONS = ('look_at', 'glance', 'turn', 'rest')
_RUN_ACTIONS = ('effect', 'clip', 'text', 'sequence', 'play', 'vocalize') + _MOVE_ACTIONS
_ACTIONS = MappingProxyType({'led': LED_ACTIONS, 'magic': MAGIC_ACTIONS,
                             'dome': DOME_ACTIONS, 'sound': SOUND_ACTIONS})

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
SOUND_PLAYING = 1              # SoundState.PLAYING (astromech_msgs 2.3.0)
SOUND_FADING = 2
SOUND_GAIN_RANGE_DB = (-60.0, 20.0)    # an intent's gain_db; the player clamps the sum
PHRASE_OWN_MAX_S = 2.5         # Vocalize length_s 0: the synth's own length is at most this
MID_VECTOR = (0.5, 0.5)        # a phrase's (arousal, valence) with no mood, or a stale one
MAX_INT32 = 2147483647
# DomeCommandStatus.source values (astromech_msgs 2.2.0) of a person driving the dome; the
# others, idle and personality, leave the channel to us.
FOREIGN_SOURCES = ('joystick', 'foxglove')
# A glance asked of a mood that has none.
DEFAULT_GLANCE_AMPLITUDE_RAD = 0.5
DEFAULT_GLANCE_DWELL_S = 1.0

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
    kind: str                     # led | magic | dome | sound
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


# Why a device call failed without the device refusing it: worth sending the palette again.
RETRYABLE_REASONS = ('timeout', 'device offline', 'stale')

CHANNELS = MappingProxyType({
    'fld': Channel('fld', 'led', ('fld',), ('tfld', 'bfld')),
    'tfld': Channel('tfld', 'led', ('tfld',), ('tfld',), parent='fld'),
    'bfld': Channel('bfld', 'led', ('bfld',), ('bfld',), parent='fld'),
    'rld': Channel('rld', 'led', ('rld',), ('rld',)),
    'psi': Channel('psi', 'led', ('fpsi', 'rpsi'), ('fpsi', 'rpsi')),
    'magic': Channel('magic', 'magic', ('magic',), ('magic',)),
    'dome': Channel('dome', 'dome', ('dome',), ('dome',)),
    'sound': Channel('sound', 'sound', ('sound',), ('sound',)),
})
LED_CHANNELS = ('fld', 'tfld', 'bfld', 'rld', 'psi')
MAGIC_CHANNELS = ('magic',)
DOME_CHANNELS = ('dome',)
SOUND_CHANNELS = ('sound',)
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
    interruptible: Optional[bool] = None    # None: the channel's default (sound: the config's)
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
    why: str = ''                 # intent | palette | preempt | duration | move | done |
    #                               the dome's halts: foreign | offline | suppressed | stale

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
    move: Any = None              # the dome: its motion.Move


@dataclass
class _ChannelState:
    spec: Channel
    claims: dict = field(default_factory=dict)      # owner -> _Claim
    run: Optional[_Run] = None
    dropped: int = 0
    refused: int = 0


class _Refuse(Exception):
    """An intent the arbiter cannot turn into a request; the message is the reason."""


def _interruptible_of(value):
    return None if value is None else bool(value)


def _as_intent(obj):
    if isinstance(obj, Intent):
        return obj
    return Intent(channel=_field(obj, 'channel', '') or '',
                  source=_field(obj, 'source', '') or '',
                  priority=int(_field(obj, 'priority', 0) or 0),
                  action=_field(obj, 'action', '') or '',
                  params=_field(obj, 'params', '') or '',
                  duration_ms=int(_field(obj, 'duration_ms', 0) or 0),
                  interruptible=_interruptible_of(_field(obj, 'interruptible', None)),
                  intent_id=_field(obj, 'intent_id', '') or '')


def _params(raw):
    if isinstance(raw, Mapping):
        return dict(raw)
    if raw is None or raw == '':
        return {}
    if isinstance(raw, str):
        try:
            value = json.loads(raw)
        except (ValueError, RecursionError, MemoryError, TypeError) as exc:
            # RecursionError: deeply nested text
            raise _Refuse('params: not JSON: %s' % (
                exc if isinstance(exc, ValueError) else type(exc).__name__)) from exc
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


def _least(a, b):
    """Return the smaller of two limits where 0 means none."""
    return min(a, b) if a and b else a or b


def _signed_to_unit(value):
    """Return a -1..1 mood axis as 0..1 (clamped); None for a missing or non-finite one."""
    if value is None or not math.isfinite(value):
        return None
    return min(1.0, max(0.0, (value + 1.0) / 2.0))


def _unit_param(params, key, default=0.5):
    """Return ``params[key]`` as a number in [0, 1], ``default`` when absent."""
    if key not in params:
        return default
    value = _number_param(params, key)
    if not 0.0 <= value <= 1.0:
        raise _Refuse('out of range')
    return value


def _number_param(params, key):
    """Return ``params[key]`` as a finite float, else refuse it (rule 13)."""
    value = params.get(key)
    try:
        number = None if isinstance(value, bool) else float(value)
    except (TypeError, ValueError, OverflowError):      # OverflowError: a huge integer
        number = None
    if number is None or isinstance(value, str) or not math.isfinite(number):
        raise _Refuse('bad params: %s must be a finite number, got %.40r' % (key, value))
    return number


def held_by(reason):
    """Return the holder a device refusal names (console, ui, text_topic, manual), or ''."""
    return HELD_REFUSALS.get((reason or '').strip(), '')


def _effect_base(effect):
    return (effect or '').split(':', 1)[0]   # cycle:<child> is the cycle run


class Arbiter:
    """The arbiter of one channel group (see the module docstring for the rules)."""

    def __init__(self, config, channels, owner='r2_expression', rng=None,
                 body_silence_s=None):
        specs = []
        for name in channels:
            spec = CHANNELS.get(name)
            if spec is None:
                raise ValueError('unknown channel %r' % (name,))
            specs.append(spec)
        kinds = {spec.kind for spec in specs}
        if len(kinds) != 1:
            raise ValueError('an arbiter serves one channel group (the LEDs, the Magic Panel, '
                             'the dome or the sound)')
        if not _valid_owner(owner):
            raise ValueError('owner must be 1-%d bytes' % MAX_OWNER_BYTES)
        self.kind = kinds.pop()
        self.owner = owner
        self._config = config
        self._rng = rng if rng is not None else random.Random()
        self._ch = {spec.name: _ChannelState(spec) for spec in specs}
        self._overlap = {name: tuple(o for o in self._ch.values() if o.spec.overlaps(cs.spec))
                         for name, cs in self._ch.items()}
        styled = {'led': LED_STYLE_CHANNELS, 'magic': MAGIC_CHANNELS,
                  'dome': DOME_CHANNELS, 'sound': ()}[self.kind]
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
        self._vector = MID_VECTOR     # (arousal, valence) in 0..1; the middle before a vector
        self._mood_at = None
        self._t0 = None
        self._last_now = 0.0
        self._was_stale = False
        self._needs_palette = set()
        self._palette_retry_at = {}      # channel -> earliest re-send of a failed palette
        self._eligible = {}
        # The flourishes, or for the dome its idle glances (rule 19).
        self._flourish = {'dome': GlanceScheduler, 'sound': ChatterScheduler}.get(
            self.kind, FlourishScheduler)(self._rng)
        # The sound player: what its last state said (phase 3 plan 5.1), or None before one.
        self._sound = None
        # The dome: what the converter, the joint state and the odometry last said.
        self._source = self._source_at = None
        self._position = self._position_at = None
        self._body_speed = self._body_yaw = 0.0
        self._body_moving = False
        self._body_at = None
        self._body_silence_s = body_silence_s   # rule 16: silence this long is still
        self._still_since = None
        self._was_foreign = self._was_suppressed = False
        self._glance_wait_until = 0.0
        self._rest = motion.RestHold(config.defaults.dome.rest_recheck_s)
        self._coast = motion.CoastMemory()   # rule 14: the coast scale, from move to move
        self._limits = self._dome_limits(config)
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

    @property
    def moving(self):
        """Return True while a dome move runs (the node then ticks at 20 Hz)."""
        return any(cs.run is not None and cs.run.move is not None for cs in self._ch.values())

    @property
    def coast_scale(self):
        """Return the coast scale the next dome move starts with (rule 14)."""
        return self._coast.recall(self._last_now)

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
        """Take the catalogue (``ListLeds``/``ListMagicPanel``/``ListSounds``); return actions."""
        now = self._clock(now)
        if self.kind == 'dome':
            raise TypeError('the dome has no catalogue')
        wanted = {'led': LedCatalogue, 'magic': MagicCatalogue,
                  'sound': SoundCatalogue}[self.kind]
        if not isinstance(catalogue, wanted):
            raise TypeError('a %s arbiter needs a %s' % (self.kind, wanted.__name__))
        self._catalogue = catalogue
        self._validate_picks()
        return self._settle(now)

    def reload(self, config, now=None):
        """Swap in a new, validated config, keeping claims, runs, holds and counts; re-style."""
        now = self._clock(self._last_now if now is None else now)
        self._config = config
        self._rest.recheck_s = config.defaults.dome.rest_recheck_s
        self._limits = self._dome_limits(config)
        self._validate_picks()
        self._want_palette()
        if self._label and not self._was_stale:
            self._restart_flourish(now)
        return self._settle(now)

    def mood(self, label, strength, now, arousal=None, valence=None):
        """
        Take one ``/mood/state``; return the actions (a new label re-styles).

        ``arousal`` and ``valence`` are the state's -1..1 vector; the sound channel's phrases
        follow it (rules 20, 22). A missing or non-finite axis keeps its last value.
        """
        now = self._clock(now)
        if not label:
            return []
        self._mood_at = now
        self._strength = strength
        self._vector = tuple(old if new is None else new for old, new in zip(
            self._vector, (_signed_to_unit(arousal), _signed_to_unit(valence))))
        if label != self._label:
            self._label = label
            if label not in self._config.moods:
                self._warn(('label', label), 'mood %r has no palette in expression.yaml; '
                                             'using neutral' % label)
            self._want_palette()
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
                return refuse('no claim by %r on %s' % (owner, channel))
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
        if f.interruptible is None:
            f = replace(f, interruptible=self._config.defaults.sound.interruptible
                        if self.kind == 'sound' else True)
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
        if f.action not in _ACTIONS[self.kind]:
            return drop('unknown action %r' % f.action)
        try:
            params = _params(f.params)
        except _Refuse as exc:
            return drop(str(exc))
        if HUE_KEY in params:
            return drop("hue is the droid's colour scheme; the personality never sets it")
        if f.source in CAPPED_SOURCES:         # the personality, not an operator
            state = self._state(cs, now)
            if state in (STATE_FOREIGN, STATE_DARK, STATE_OFFLINE, STATE_SUPPRESSED):
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
            self._start_run(cs, run, now)
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

    def _start_run(self, cs, run, now):
        """Make ``run`` the channel's, replacing every run on an overlapping channel."""
        for other in self._overlapping(cs):
            if other.run is not None:         # the device replaces them, once it accepts
                if run.move is None:          # a move has no device to refuse it
                    run.displaced.append((other, other.run))
                self._end_run(other, now)
        cs.run = run
        if run.move is not None and run.move.done:
            self._end_run(cs, now)            # there already: its first command was the 0

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
            if run is not None and action.call in RUN_CALLS:
                run.confirmed_at = now
                run.displaced = []
            elif run is not None and run.stopping and action.call in (CALL_STOP,
                                                                      CALL_MAGIC_STOP):
                run.surfaces.discard(action.target)
                if not run.surfaces:
                    self._end_run(cs, now)
            return '%s: %s' % (what, reason) if reason else ''
        if run is not None and action.call in RUN_CALLS:
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
            if action.why == 'palette' and reason in RETRYABLE_REASONS:
                # Not a hold (that re-applies when it ends): try again after a backoff.
                self._needs_palette.add(cs.spec.palette_channel)
                self._palette_retry_at[cs.spec.palette_channel] = (
                    now + self._config.defaults.device_stale_s)
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

    def dome_state(self, state, now):
        """
        Take what the dome node hears; return actions.

        ``state`` is a mapping (or an object) with any of: ``source`` (the
        converter's ``DomeCommandStatus.source``), ``position_rad`` (the joint
        position), ``body_speed_m_s`` and ``body_yaw_rad_s`` (from the odometry,
        signed or not). A field left out keeps its last value.
        """
        now = self._clock(now)
        self._require('dome')
        source = _field(state, 'source', None)
        if source is not None:
            self._source, self._source_at = str(source), now
        position = _field(state, 'position_rad', None)
        if position is not None and math.isfinite(position):
            self._position, self._position_at = motion.wrap(float(position)), now
        speed = _field(state, 'body_speed_m_s', None)
        yaw = _field(state, 'body_yaw_rad_s', None)
        if speed is not None or yaw is not None:
            self._body_at = now
            if speed is not None:
                self._body_speed = abs(speed)
            if yaw is not None:
                self._body_yaw = abs(yaw)
            dome = self._config.defaults.dome
            if self._body_speed > dome.body_speed_m_s or self._body_yaw > dome.body_yaw_rad_s:
                self._body_moving = True
            elif self._body_moving:
                self._body_moving, self._still_since = False, now
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

    def sound_state(self, snapshot, now):
        """
        Take one ``SoundState`` (a message or a dict of its fields); return actions.

        Its ``state`` (0 idle, 1 playing, 2 fading), ``name``, ``owner``, ``priority``,
        ``intent_id``, ``elapsed_ms``, ``remaining_ms`` and ``last_end`` (0 none, 1 ended, 2
        duration, 3 stopped, 4 replaced, 5 failed). A run of ours that the player shows
        playing takes its end from ``remaining_ms`` (the stream's own clock, rule 13); one it
        no longer shows, :data:`SNAPSHOT_GRACE_S` after its confirmation, has ended on the
        device.
        """
        now = self._clock(now)
        self._require('sound')
        self._sound = {
            'state': int(_field(snapshot, 'state', 0) or 0),
            'name': _field(snapshot, 'name', '') or '',
            'owner': _field(snapshot, 'owner', '') or '',
            'priority': int(_field(snapshot, 'priority', 0) or 0),
            'intent_id': _field(snapshot, 'intent_id', '') or '',
            'elapsed_ms': int(_field(snapshot, 'elapsed_ms', 0) or 0),
            'remaining_ms': int(_field(snapshot, 'remaining_ms', 0) or 0),
            'last_end': int(_field(snapshot, 'last_end', 0) or 0)}
        self._took_snapshot(now)
        cs = self._ch['sound']
        run = cs.run
        if run is not None:
            if self._shows(run):
                if self._sound['remaining_ms'] > 0:      # 0: unknown, or the last block
                    run.until = now + self._sound['remaining_ms'] / 1000.0
                    self._size_auto_claim(cs)
            elif self._evidence(run, now):
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
            if run is not None and run.move is None and run.until is not None and \
                    now >= run.until:
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
        actions += self._step_moves(now)
        actions += self._settle(now)
        if self.kind == 'dome':
            actions += self._idle_dome(now)
        elif self.kind == 'sound':
            actions += self._chatter(now)
        elif not self._was_stale and self._label in self._config.moods:
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
        if self.kind == 'sound':
            self._picks, warnings = validate_voices(self._config, self._catalogue)
            for key, text in warnings:
                self._warn(key, text)
            for text in self._config.sound_warnings:
                self._warn(('sound', text), text)
            return
        led = self._catalogue if self.kind == 'led' else None
        magic = self._catalogue if self.kind == 'magic' else None
        self._picks, warnings = validate_picks(self._config, led=led, magic=magic)
        for key, text in warnings:
            self._warn(key, text)
        if self.kind == 'dome':
            for text in self._config.dome_warnings:
                self._warn(('dome', text), text)

    def _restart_flourish(self, now):
        if self.kind == 'dome':
            glance = self._config.mood_dome(self._label).glance
            self._flourish.restart(glance, now, self._strength)
            return
        mood = self._config.moods.get(self._label)
        if self.kind == 'sound':
            sound = mood.sound if mood is not None else None
            self._flourish.restart(sound.chatter if sound is not None else None, now,
                                   self._strength)
            return
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
        if self.kind == 'sound':
            self._adopt_sound(now)
            return
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

    def _adopt_sound(self, now):
        """Rule 11 for the player: a play of our owner is ours, at idle priority, to its end."""
        cs, snap = self._ch['sound'], self._sound
        if cs.run is not None or snap['owner'] != self.owner or \
                snap['state'] not in (SOUND_PLAYING, SOUND_FADING):
            return
        cs.run = _Run(ADOPTED, ADOPTED, 'play', PRIORITY_IDLE, True, snap['name'], {'sound'},
                      now + snap['remaining_ms'] / 1000.0, confirmed_at=now - SNAPSHOT_GRACE_S)

    def _shows(self, run):
        """Return True when the player's last state shows ``run`` (sound only)."""
        snap = self._sound
        if snap is None or snap['state'] not in (SOUND_PLAYING, SOUND_FADING) or \
                snap['owner'] != self.owner:
            return False
        if run.source == ADOPTED:
            return snap['name'] == run.effect
        return snap['intent_id'] == run.intent_id

    def _offline(self, now):
        if self.kind == 'dome':
            stale_s = self._config.defaults.device_stale_s
            return any(at is None or now - at > stale_s
                       for at in (self._position_at, self._source_at))
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
        if self.kind in ('dome', 'sound'):
            return None                   # the converter has no hold timer (rule 15); nor has
        #                                   the player (phase 3 plan 4.1)
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
        if self.kind == 'sound':
            return ''                     # nobody else plays sound on the robot (plan 4.1)
        if self.kind == 'dome':
            return self._source if self._source in FOREIGN_SOURCES else ''
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
        if self._suppressed(now):
            return STATE_SUPPRESSED
        return None

    def _body(self, now):
        """Rule 16: ``(moving, still_since)``, a silence of body_silence_s being still."""
        silence = self._body_silence_s
        if self._body_moving and silence is not None and now - self._body_at > silence:
            return False, self._body_at + silence
        return self._body_moving, self._still_since

    def _suppressed(self, now):
        """Rule 16: the body moves, or was moving less than after_still_s ago."""
        if self.kind != 'dome':
            return False
        moving, still_since = self._body(now)
        return moving or (still_since is not None and
                          now - still_since < self._config.defaults.dome.after_still_s)

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
        if run is not None and run.move is not None:
            self._coast.learn(run.move, now)  # rule 14
            self._rest.arm(now)               # rule 19: the next drift check, and glance
            if not self._was_stale:
                self._restart_flourish(now)

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
        if self.kind == 'dome':               # a 0 overrides nobody (rule 14)
            return [self._velocity(0.0, priority, intent_id, why)]
        if self.kind == 'sound':              # one sound at a time: one stop, with the fade
            return [Action(CALL_SOUND_STOP, 'sound', 'sound', {'owner': self.owner},
                           self.owner, priority, intent_id, why)]
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
                        device_ends=spec.kind in ('led', 'sound'))

        if spec.kind == 'sound' and f.action != 'stop':
            return self._render_sound(cs, f, params, intent_id, now)
        if f.action == 'track':
            raise _Refuse('not in this phase')
        if f.action in _MOVE_ACTIONS:
            return self._render_move(cs, f, params, intent_id, now)
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
            text_motion = params.get('motion', 'bounce')
            if text_motion not in TEXT_MOTIONS:
                raise _Refuse('unknown text motion')
            rgb, speed = _rgb_param(params), _int_param(params, 'speed', 0, 0, 9)
            duration = self._duration(f, params)
            (surface,) = spec.surfaces
            return ([action(CALL_SAY, surface, {
                'surface': surface, 'lines': list(lines),
                'linked': bool(params.get('linked', False)),
                'motion': TEXT_MOTIONS.index(text_motion),
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
            if f.source in CAPPED_SOURCES and (
                    repeat == 0 or (self._catalogue is not None and
                                    ref in self._catalogue.unbounded)):
                raise _Refuse('unbounded sequence')
            end = params.get('end', END_VALUES['blank'])    # a run never leaves the panel lit
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

    # --- sound -----------------------------------------------------------------------------

    def _silence_idle(self, now):
        """Rule 23: the mood went stale; stop an idle play with the fade."""
        cs = self._ch['sound']
        run = cs.run
        if run is None or run.source != 'idle':
            return []
        stops = self._stops(cs, run.surfaces, run.priority, run.intent_id, 'stale')
        self._end_run(cs, now)
        return stops

    def _chatter(self, now):
        """
        Rule 22: the mood's chatter, when it is due and the voice is free.

        A due slot that falls while the channel is claimed, offline, stale or playing is
        skipped, not kept for later. The pick goes through rule 4 as an idle intent.
        """
        if self._was_stale or self._label not in self._config.moods:
            return []
        pick = self._flourish.poll(self._picks.get(self._label, ()), now, self._strength)
        cs = self._ch['sound']
        if pick is None or cs.run is not None or self._state(cs, now) != STATE_IDLE or \
                self._sound['state'] != 0:
            return []
        if pick.tag == VOICE_POOL:
            action, params = 'vocalize', {}           # rule 20: the mood's vector
        else:
            name = choose_clip(self._catalogue, pick.tag, self._rng)
            if name is None:
                return []
            action, params = 'play', {'name': name}
        fields = {'channel': 'sound', 'source': 'idle', 'priority': PRIORITY_IDLE,
                  'action': action, 'params': json.dumps(params),
                  'intent_id': self._new_id('chatter')}
        return list(self.intent(Intent(**fields), now).actions)

    def _sound_gain(self, f, params, now):
        """Return the request's gain: the intent's, plus the mood's for the personality."""
        gain = params.get('gain_db', 0.0)
        lo, hi = SOUND_GAIN_RANGE_DB
        if isinstance(gain, bool) or not isinstance(gain, (int, float)) or \
                not lo <= gain <= hi:         # NaN fails the comparison
            raise _Refuse('bad params: gain_db must be a number from %g to %g' % (lo, hi))
        if f.source in CAPPED_SOURCES:
            gain += self._config.mood_sound(self._label, self._stale(now)).gain_db
        return float(gain)

    def _sound_duration(self, f, params, clip_ms):
        """
        Return ``(duration_ms, length_ms)`` of a play.

        The first is what the request carries, the second how long the play will run (0:
        unknown). The personality's is capped by ``max_duration_ms`` and by the clip's length
        (rule 21), and is 0 (the whole clip) when the cap does not bite on a known clip; a
        manual play is not capped, and 0 means the clip's length.
        """
        requested = f.duration_ms or params.get('duration_ms', 0)
        if isinstance(requested, bool) or not isinstance(requested, int) or \
                not 0 <= requested <= MAX_UINT32:
            raise _Refuse('out of range')
        limit = 0
        if f.source in CAPPED_SOURCES:
            limit = self._config.defaults.max_duration_ms
            if clip_ms:
                limit = min(limit, clip_ms)
        duration = _least(requested, limit)
        length = _least(duration, clip_ms)
        if f.source in CAPPED_SOURCES and clip_ms and duration >= clip_ms:
            duration = 0              # the cap does not bite: the whole clip, never a cut
        return duration, length

    def _render_sound(self, cs, f, params, intent_id, now):
        """Return ``(actions, run)`` for a ``play`` or ``vocalize`` (rules 20, 21); else refuse."""
        if f.action == 'clip':
            raise _Refuse('use play')         # `clip` is the LED vocabulary
        catalogue = self._catalogue
        if catalogue is None:
            raise _Refuse('channel is offline')
        gain = self._sound_gain(f, params, now)
        pr, owner = f.priority, self.owner
        common = {'gain_db': gain, 'priority': pr, 'owner': owner, 'intent_id': intent_id}
        if f.action == 'play':
            name, pool = params.get('name'), params.get('pool')
            if (name is None) == (pool is None):
                raise _Refuse('bad params: give a name or a pool, not both')
            if name is not None and name not in catalogue.clips:
                raise _Refuse('unknown name')
            if name is not None and catalogue.clips[name].excluded and \
                    pr < MANUAL_PLAY_PRIORITY:
                raise _Refuse('excluded: manual play only')
            if pool is not None and pool not in catalogue.pools:
                raise _Refuse('unknown pool')
            duration, length = self._sound_duration(f, params, catalogue.duration_ms(name))
            request = dict(common, name=name or '', pool=pool or '', duration_ms=duration)
            call, effect = CALL_PLAY, name or pool
        else:
            if not catalogue.synth:
                raise _Refuse('synth not available')
            arousal, valence = MID_VECTOR if self._stale(now) else self._vector
            request = dict(common, arousal=_unit_param(params, 'arousal', arousal),
                           valence=_unit_param(params, 'valence', valence),
                           length_s=self._phrase_length_s(f, params),
                           seed=_int_param(params, 'seed', 0, -MAX_INT32, MAX_INT32))
            call, effect = CALL_VOCALIZE, 'voice'
            length = int(round(request['length_s'] * 1000))
        run = _Run(intent_id, f.source, f.action, pr, f.interruptible, effect, {'sound'},
                   now + length / 1000.0 if length else None)
        return [Action(call, 'sound', 'sound', request, owner, pr, intent_id, 'intent')], run

    def _phrase_length_s(self, f, params):
        """Return a phrase's ``length_s``: asked for, cut by the intent's duration and the cap."""
        length = _number_param(params, 'length_s') if 'length_s' in params else 0.0
        if not 0.0 <= length <= MAX_UINT32 / 1000.0:
            raise _Refuse('out of range')
        length = length or f.duration_ms / 1000.0
        cap = self._config.defaults.max_duration_ms / 1000.0
        if not length:                # the synth's own length, unless the cap is shorter
            if f.source not in CAPPED_SOURCES or cap >= PHRASE_OWN_MAX_S:
                return 0.0
            length = cap
        ceiling = f.duration_ms / 1000.0
        if f.source in CAPPED_SOURCES:
            ceiling = min(ceiling, cap) if ceiling else cap
        return _least(length, ceiling)

    # --- the dome --------------------------------------------------------------------------

    def _velocity(self, velocity, priority, intent_id, why):
        return Action(CALL_VELOCITY, 'dome', 'dome', {'velocity_rad_s': float(velocity)},
                      self.owner, priority, intent_id, why)

    @staticmethod
    def _dome_limits(config):
        dome = config.defaults.dome
        return motion.Limits(min_speed=dome.min_speed_rad_s, max_move_s=dome.max_move_s,
                             settle_s=dome.settle_s, coast=dome.coast, floor=dome.floor,
                             final_pulse_below_rad=dome.final_pulse_below_rad,
                             retries=dome.retries,
                             feedback_timeout_s=dome.feedback_timeout_s,
                             max_plausible_rad_s=dome.max_plausible_rad_s)

    def _mood_dome(self, now):
        return self._config.mood_dome(self._label, self._stale(now))

    def _move_profile(self, f, params, now):
        """Rule 13: the mood's profile for idle intents, else ``params.profile``."""
        name = MOOD_PROFILE if f.source == 'idle' else params.get('profile', MOOD_PROFILE)
        if name == MOOD_PROFILE:
            return self._config.profile(self._mood_dome(now).profile)
        if not isinstance(name, str) or name not in self._config.profiles:
            raise _Refuse('bad params: unknown profile %r' % (name,))
        return self._config.profiles[name]

    def _render_move(self, cs, f, params, intent_id, now, creep=False):
        """Return ``(actions, run)`` for a ``look_at``, ``glance``, ``turn`` or ``rest``."""
        state = self._state(cs, now)
        if state in (STATE_OFFLINE, STATE_FOREIGN):
            raise _Refuse('channel is %s' % STATE_NAMES[state])
        dome = self._config.defaults.dome
        heading, limits = dome.heading, self._limits
        profile = self._move_profile(f, params, now)
        if creep:                             # rule 19: a drift is corrected at min_speed
            profile = replace(profile, v_max=limits.min_speed)
        old = cs.run.move if cs.run is not None else None
        # A replacement takes its predecessor's coast scale; a fresh move the carried one.
        carried = {'coast_scale': self._coast.recall(now)}
        if f.action == 'look_at':
            move = motion.look_at(heading.joint(_number_param(params, 'bearing')), profile,
                                  limits, now, old, **carried)
        elif f.action == 'turn':
            move = motion.turn(heading.delta(_number_param(params, 'delta')), profile, limits,
                               now, old, **carried)
        elif f.action == 'rest':
            bearing = (_number_param(params, 'bearing') if 'bearing' in params
                       else self._mood_dome(now).rest_rad)
            if bearing is None:
                raise _Refuse('bad params: the mood has no rest heading; give a bearing')
            move = motion.rest(heading.joint(bearing), profile, limits, now, old, **carried)
        else:
            amplitude, dwell = self._glance_draw(f, params, now)
            side = params.get('side')
            if side is None:
                side = random_side(self._rng)
            if side not in ('left', 'right'):
                raise _Refuse('bad params: side must be left or right')
            move = motion.glance(heading.delta(amplitude if side == 'left' else -amplitude),
                                 dwell, profile, limits, now, old, **carried)
        velocity, _, _ = move.step(self._position, now, self._position_at)
        run = _Run(intent_id, f.source, f.action, f.priority, f.interruptible, f.action,
                   {'dome'}, move.deadline, device_ends=False, move=move)
        return [self._velocity(velocity, f.priority, intent_id, 'intent')], run

    def _glance_draw(self, f, params, now):
        """Return a glance's ``(amplitude, dwell_s)``: the params', else the mood's draw."""
        glance = self._mood_dome(now).glance
        if 'amplitude' in params:
            amplitude = _number_param(params, 'amplitude')
        elif glance is not None:
            amplitude = uniform(glance.amplitude_rad, self._rng)
        else:
            amplitude = DEFAULT_GLANCE_AMPLITUDE_RAD
        if not 0.0 < amplitude <= math.pi:
            raise _Refuse('bad params: amplitude must be in (0, pi]')
        if 'dwell_s' in params:
            dwell = _number_param(params, 'dwell_s')
        elif f.duration_ms:
            dwell = f.duration_ms / 1000.0
        elif glance is not None:
            dwell = uniform(glance.dwell_s, self._rng)
        else:
            dwell = DEFAULT_GLANCE_DWELL_S
        if not 0.0 <= dwell <= self._config.defaults.dome.max_move_s:
            raise _Refuse('bad params: the dwell (dwell_s, or duration_ms) must be 0 to '
                          'max_move_s')
        return amplitude, dwell

    def _step_moves(self, now):
        """Step the move in progress; it sends its velocity every tick and a 0 at its end."""
        actions = []
        for cs in self._ch.values():
            run = cs.run
            if run is None or run.move is None:
                continue
            velocity, done, _ = run.move.step(self._position, now, self._position_at)
            actions.append(self._velocity(velocity, run.priority, run.intent_id,
                                          'done' if done else 'move'))
            if done:
                self._end_run(cs, now)
        return actions

    def _halt(self, cs, why, now, sources=None):
        """Rule 14: stop the move in progress (of ``sources``, default any) with its 0."""
        run = cs.run
        if run is None or run.move is None or (sources is not None and
                                               run.source not in sources):
            return []
        run.move.stop()
        self._end_run(cs, now)
        return [self._velocity(0.0, run.priority, run.intent_id, why)]

    def _dome_guard(self, now):
        """Rules 15-17: halt on foreign, offline and the onset of suppression."""
        cs = self._ch['dome']
        moving, still_since = self._body(now)
        if self._body_moving and not moving:          # rule 16: nobody drives any more
            self._body_speed = self._body_yaw = 0.0
            self._body_moving, self._still_since = moving, still_since
        state = self._blocked(cs, now)
        foreign = state == STATE_FOREIGN
        if self._was_foreign and not foreign:
            self._glance_wait_until = now + self._config.defaults.dome.after_foreign_s
        suppressed = self._suppressed(now)
        onset = suppressed and not self._was_suppressed
        self._was_foreign, self._was_suppressed = foreign, suppressed
        if state in (STATE_OFFLINE, STATE_FOREIGN):
            return self._halt(cs, STATE_NAMES[state], now)
        return self._halt(cs, 'suppressed', now) if onset else []

    def _idle_dome(self, now):
        """Rule 19: the rest's drift check, then the idle glances."""
        cs = self._ch['dome']
        if self._was_stale or cs.run is not None:
            return []
        mood = self._mood_dome(now)
        if mood.rest_rad is not None and self._palette_eligible(cs, now):
            dome = self._config.defaults.dome
            drift = motion.wrap(dome.heading.joint(mood.rest_rad) - self._position)
            band = self._config.profile(mood.profile).settle_band_rad
            if self._rest.poll(drift, band, now):
                f = Intent('dome', 'idle', PRIORITY_IDLE, 'rest', {},
                           intent_id=self._new_id('rest'))
                actions, run = self._render_move(cs, f, {}, f.intent_id, now, creep=True)
                self._start_run(cs, run, now)
                return actions
        if now < self._glance_wait_until:
            return []
        pick = self._flourish.poll(now, self._strength)
        if pick is None:
            return []
        return self.intent(Intent(**glance_intent_fields(pick, self._new_id('glance'))),
                           now).actions

    def _palette_eligible(self, cs, now):
        """Return True when nothing keeps the palette off: a blocking state, a claim, a run."""
        return (self._blocked(cs, now) is None and
                self._holder(cs, now) is None and
                not any(o.run is not None and o.run.priority > PRIORITY_IDLE
                        for o in self._overlapping(cs)))

    def _settle(self, now):
        """Rule 8 and 9: follow staleness, and re-style every channel that needs it."""
        stale = self._stale(now)
        actions = []
        if stale != self._was_stale:
            self._was_stale = stale
            self._want_palette()
            if stale:
                self._flourish.stop()
                if self.kind == 'dome':       # rule 18
                    actions += self._halt(self._ch['dome'], 'stale', now, CAPPED_SOURCES)
                elif self.kind == 'sound':    # rule 23
                    actions += self._silence_idle(now)
            else:
                self._restart_flourish(now)
        if self.kind == 'dome':
            actions += self._dome_guard(now)
        elif self.kind == 'sound' and self._ch['sound'].run is not None and self._offline(now):
            self._end_run(self._ch['sound'], now)     # rule 24: a gone player plays nothing
        for name in self._palette_channels:
            cs = self._ch[name]
            eligible = self._palette_eligible(cs, now)
            if eligible and not self._eligible.get(name, False):
                self._want_palette((name,))
            self._eligible[name] = eligible
            if (eligible and name in self._needs_palette and (stale or self._label) and
                    now >= self._palette_retry_at.get(name, 0.0)):
                actions += self._palette_actions(cs, stale)
                self._needs_palette.discard(name)
                self._palette_retry_at.pop(name, None)
        return actions

    def _want_palette(self, names=None):
        """Mark channels as needing their palette now (a new look or a return is not a retry)."""
        for name in self._palette_channels if names is None else names:
            self._needs_palette.add(name)
            self._palette_retry_at.pop(name, None)

    def _palette_actions(self, cs, stale):
        if self.kind == 'dome':               # rule 19: the mood's rest is the dome's palette
            if stale or self._config.mood_dome(self._label).rest_rad is None:
                return []
            return list(self.intent(Intent('dome', 'idle', PRIORITY_IDLE, 'rest', {},
                                           intent_id=self._new_id('rest')),
                                    self._last_now).actions)
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
