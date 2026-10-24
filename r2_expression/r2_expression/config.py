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
Load and validate ``expression.yaml``: the arbiters' settings and the mood palettes.

Pure Python (PyYAML, no ROS imports). The result is an immutable
:class:`ExpressionConfig`; the core and the nodes never see raw YAML. The
schema (plan section 4.3):

* ``defaults`` (optional): ``manual_hold_s`` (5.0), ``mood_stale_s`` (3.0),
  ``max_duration_ms`` (30000, an integer), ``status_hz`` (2.0),
  ``max_lease_s`` (60.0), ``device_stale_s`` (5.0). Each > 0. And ``dome``
  (phase 2 plan 4.5, :class:`DomeDefaults`): ``min_speed_rad_s`` (0.6, above
  both floors of the velocity map: 0.44 positive, 0.567 negative),
  ``max_speed_rad_s`` (2.0) and ``max_accel_rad_s2`` (6.0), the renderer's own
  clamps (the converter's personality caps are the authority; keep them
  equal); ``max_move_s`` (8.0), ``settle_s`` (0.8), ``retries`` (2, an integer
  >= 0), ``feedback_timeout_s`` (0.5, at least 0.2), ``max_plausible_rad_s`` (4.0, the fastest
  dome speed a joint-state sample may imply before the move holds it back, plan 2.1),
  ``rest_recheck_s`` (30.0),
  ``coast: {c1_s: 0.52, c2_rad: 0.0, learn: false, scale_limits: [0.7, 1.4],
  learn_rate: 0.3}`` (``learn`` a boolean: with it off the coast scale stays 1.0 and the
  measured fit is trusted; on, each lesson moves the scale ``learn_rate`` (in (0, 1]) of the
  way to the observed ratio, kept within ``scale_limits``, 0 < min <= 1 <= max),
  ``floor: {coast_rad: 0.12, speed_rad_s: 0.35}`` (what the dome does at ``min_speed``,
  measured; ``coast_rad`` may be 0) and ``final_pulse_below_rad`` (0.35: a retry from
  closer than this is a timed pulse at the floor, not an approach),
  ``heading: {sign: -1, offset_rad: 0.0}`` (sign -1 or 1, offset in (-pi, pi]),
  ``body_speed_m_s`` (0.15), ``body_yaw_rad_s`` (0.3), ``after_foreign_s`` (3.0), ``after_still_s``
  (1.0). ``c2_rad`` may be 0; ``min_speed_rad_s`` may not exceed
  ``max_speed_rad_s``.
* ``profiles`` (optional): name -> dome motion profile ``{v_max, accel, k,
  settle_band_rad}``, all > 0, ``settle_band_rad`` at most pi, ``v_max`` at
  least ``min_speed_rad_s``. A ``v_max`` or ``accel`` above the cap is clamped
  to it with one warning (:attr:`ExpressionConfig.dome_warnings`). ``mood``
  is reserved (an intent's ``profile: mood`` means the mood's own).
* ``neutral`` (required): the styles used while ``/mood/state`` is stale and
  for any mood without its own entry. Per style channel, as below, but
  ``brightness`` is a single number (there is no strength to scale by) and
  there is no ``flourish``. A channel left out is not touched while neutral.
  ``dome`` here is ``{profile}`` only (no glances, no rest); absent, the dome
  moves with :data:`DEFAULT_PROFILE`.
* ``moods`` (required, non-empty): label -> mood. A mood has a style per
  channel and an optional ``flourish``; a channel it leaves out takes the
  neutral style. Its ``dome`` is ``{profile, glance, rest_rad}``: ``profile``
  (required) names a profile; ``glance`` (optional) is ``{every_s,
  amplitude_rad, dwell_s}``, each ``[min, max]`` with 0 < min <= max and
  amplitudes at most pi and dwells at most ``max_move_s``; ``rest_rad``
  (optional) is the bearing the dome rests at (+ve left), in (-pi, pi]. A mood
  without ``dome`` moves as neutral, with one warning.

Sound (phase 3 plan 4.4). ``defaults.sound`` is ``{interruptible: false}``
(:class:`SoundDefaults`): the default for the intents the arbiter builds itself (chatter,
voice); a ROS ``ExpressionIntent`` carries its own bool and ``express`` sends true. The fade,
the house level and the level range are the player's own parameters. A mood, and ``neutral``,
may have ``sound: {gain_db, chatter}``: ``gain_db`` within :data:`GAIN_RANGE_DB` (default 0) and,
for a mood only, ``chatter: {every_s: [min, max], pools: {tag: weight}}`` where every tag is one of
:data:`SOUND_TAGS` (never ``excluded``) or :data:`VOICE_POOL` and each weight > 0. ``voice`` is
not a catalogue tag: its picks are phrases the synthesiser renders from the mood's arousal and
valence (plan 4.4, step 7). A mood without ``sound`` takes
neutral's gain and has no chatter, with one warning (:attr:`ExpressionConfig.sound_warnings`).

A style channel is one of :data:`STYLE_CHANNELS`:

* ``fld``, ``rld``: ``drift`` (``calm`` or ``lively``) and ``brightness``
  (percent 0-100, an integer or ``[min, max]``), either or both. ``fld``
  styles both front boards; ``tfld`` and ``bfld`` have no style of their own.
* ``psi``: ``brightness`` only (the PSIs have no drift).
* ``magic``: ``brightness`` 0-15, an integer or ``[min, max]``.

``mood_strength`` scales ``[min, max]`` linearly (:mod:`r2_expression.palette`).

A ``flourish`` has ``every_s: [min, max]`` (seconds, 0 < min <= max) and a
non-empty list of ``picks``. A pick names a ``channel`` (one of
:data:`PICK_CHANNELS`) and exactly one of ``effect`` or ``clip`` (LED
channels; no clips on ``tfld``/``bfld``) or ``sequence`` (``magic``: an id
0-255 or a catalogue name), with optional ``weight`` (> 0, default 1),
``rgb`` (0-0xFFFFFF) and ``speed`` (1-9) for the LEDs, ``repeat`` (1-255: 0 would
run until stopped) and ``end`` (``leave`` or ``blank``, default ``blank``) for the panel,
and ``duration_ms`` (>= 1; values above ``defaults.max_duration_ms`` are capped to it;
absent: the cap).
A pick never carries ``duration_ms`` 0 (rule 3: the personality never sends
0). Effect, clip and sequence names are checked against the device's
catalogue at runtime, not here (:func:`r2_expression.palette.validate_picks`).

Rejected anywhere: a ``hue`` key (rule 9: hue is the droid's colour scheme,
never a mood knob), unknown keys, unknown channels, out-of-range values.
Every rejection raises :class:`ExpressionConfigError` whose message starts
with the dotted path of the offending key.
"""

from dataclasses import asdict, dataclass, field, fields, replace
import math
from types import MappingProxyType
from typing import Hashable, Mapping, Optional, Tuple, Union

from r2_expression.motion import Coast, Floor, Heading, Profile
import yaml

LED_STYLE_CHANNELS = ('fld', 'rld', 'psi')
STYLE_CHANNELS = LED_STYLE_CHANNELS + ('magic',)
LED_PICK_CHANNELS = ('fld', 'tfld', 'bfld', 'rld', 'psi')
PICK_CHANNELS = LED_PICK_CHANNELS + ('magic',)
CLIPLESS_CHANNELS = ('tfld', 'bfld')
DRIFT_PRESETS = ('calm', 'lively')
END_MODES = ('leave', 'blank')
PICK_KINDS = ('effect', 'clip', 'sequence')

BRIGHTNESS_RANGE = MappingProxyType({'fld': (0, 100), 'rld': (0, 100), 'psi': (0, 100),
                                     'magic': (0, 15)})
_DRIFT_CHANNELS = ('fld', 'rld')

HUE_KEY = 'hue'
TOP_LEVEL_KEYS = ('defaults', 'profiles', 'neutral', 'moods')
MOOD_PROFILE = 'mood'          # an intent's profile: the mood's own; never a profile name
# The dome's profile when nothing names one (the plan's ``smooth``).
DEFAULT_PROFILE = Profile(v_max=1.0, accel=1.5, k=1.5, settle_band_rad=0.10)
_PICK_KEYS = ('channel',) + PICK_KINDS + ('weight', 'rgb', 'speed', 'repeat', 'end',
                                          'duration_ms')
_LED_ONLY_PICK_KEYS = ('rgb', 'speed')
_MAGIC_ONLY_PICK_KEYS = ('repeat', 'end')
MAX_UINT32 = 4294967295
# The pool vocabulary (phase 3 plan step 0, decision 11.13): new tags need a plan change.
SOUND_TAGS = ('chatter', 'excited', 'sad', 'question', 'ooh', 'hum', 'alarm', 'scream',
              'laugh', 'annoyed', 'whistle', 'calm', 'scared', 'sleepy', 'greeting', 'misc',
              'excluded')
EXCLUDED_TAG = 'excluded'      # music, speech, silence: manual play only, never in a pool
FALLBACK_TAG = 'chatter'       # a mood whose pools all resolve to nothing falls back to it
VOICE_POOL = 'voice'           # a chatter pool of synthesised phrases, not a catalogue tag
# What a mood's chatter may name: the playable tags and the synthesiser's voice.
CHATTER_POOLS = tuple(t for t in SOUND_TAGS if t != EXCLUDED_TAG) + (VOICE_POOL,)
GAIN_RANGE_DB = (-20.0, 6.0)   # a mood's sound gain
MIN_FEEDBACK_TIMEOUT_S = 0.2   # defaults.dome.feedback_timeout_s: two 10 Hz joint states


class ExpressionConfigError(ValueError):
    """An ``expression.yaml`` that cannot be used; the message names the key."""


@dataclass(frozen=True)
class DomeDefaults:
    """The dome arbiter's settings (``defaults.dome``)."""

    min_speed_rad_s: float = 0.6
    max_speed_rad_s: float = 2.0
    max_accel_rad_s2: float = 6.0
    max_move_s: float = 8.0
    settle_s: float = 0.8
    retries: int = 2
    feedback_timeout_s: float = 0.5
    max_plausible_rad_s: float = 4.0
    rest_recheck_s: float = 30.0
    coast: Coast = Coast()
    floor: Floor = Floor()
    final_pulse_below_rad: float = 0.35
    heading: Heading = Heading()
    body_speed_m_s: float = 0.15
    body_yaw_rad_s: float = 0.3
    after_foreign_s: float = 3.0
    after_still_s: float = 1.0


@dataclass(frozen=True)
class SoundDefaults:
    """The sound channel's settings (``defaults.sound``)."""

    interruptible: bool = False       # for the intents the arbiter builds itself


@dataclass(frozen=True)
class Defaults:
    """The arbiters' timing settings."""

    manual_hold_s: float = 5.0
    mood_stale_s: float = 3.0
    max_duration_ms: int = 30000
    status_hz: float = 2.0
    max_lease_s: float = 60.0
    device_stale_s: float = 5.0
    dome: DomeDefaults = DomeDefaults()
    sound: SoundDefaults = SoundDefaults()


_DEFAULT_FIELDS = tuple(f.name for f in fields(Defaults))
_DOME_FIELDS = tuple(f.name for f in fields(DomeDefaults))
_PROFILE_FIELDS = tuple(f.name for f in fields(Profile))
_SOUND_FIELDS = tuple(f.name for f in fields(SoundDefaults))
_GLANCE_KEYS = ('every_s', 'amplitude_rad', 'dwell_s')


@dataclass(frozen=True)
class Style:
    """One channel's ambient look: drift preset (or None) and a brightness range (or None)."""

    drift: Optional[str] = None
    brightness: Optional[Tuple[int, int]] = None    # (min, max); min == max for a number

    def to_dict(self):
        """Return the YAML form."""
        out = {}
        if self.drift is not None:
            out['drift'] = self.drift
        if self.brightness is not None:
            lo, hi = self.brightness
            out['brightness'] = lo if lo == hi else [lo, hi]
        return out


@dataclass(frozen=True)
class Pick:
    """One weighted flourish: a run on a channel."""

    channel: str
    kind: str                         # effect | clip | sequence
    name: Union[str, int]             # effect or clip name; sequence id or name
    weight: float = 1.0
    rgb: Optional[int] = None
    speed: Optional[int] = None
    repeat: Optional[int] = None
    end: Optional[str] = None
    duration_ms: Optional[int] = None  # None: the arbiter's max_duration_ms

    def to_dict(self):
        """Return the YAML form."""
        out = {'channel': self.channel, self.kind: self.name, 'weight': self.weight}
        for key in ('rgb', 'speed', 'repeat', 'end', 'duration_ms'):
            value = getattr(self, key)
            if value is not None:
                out[key] = value
        return out


@dataclass(frozen=True)
class Flourish:
    """How often a mood's flourishes fire, and the weighted set they are drawn from."""

    every_s: Tuple[float, float]
    picks: Tuple[Pick, ...]


@dataclass(frozen=True)
class Glance:
    """A mood's idle glances: each a ``(min, max)`` range drawn from uniformly."""

    every_s: Tuple[float, float]
    amplitude_rad: Tuple[float, float]
    dwell_s: Tuple[float, float]


@dataclass(frozen=True)
class MoodDome:
    """A mood's dome: its profile's name ('' for :data:`DEFAULT_PROFILE`), glances, rest."""

    profile: str = ''
    glance: Optional[Glance] = None
    rest_rad: Optional[float] = None     # a bearing, +ve left

    def to_dict(self):
        """Return the YAML form."""
        out = {'profile': self.profile}
        if self.glance is not None:
            out['glance'] = {key: list(getattr(self.glance, key)) for key in _GLANCE_KEYS}
        if self.rest_rad is not None:
            out['rest_rad'] = self.rest_rad
        return out


@dataclass(frozen=True)
class Chatter:
    """A mood's chatter: how often, and the weighted tag pools a clip is drawn from."""

    every_s: Tuple[float, float]
    pools: Tuple[Tuple[str, float], ...]      # (tag, weight), in file order


@dataclass(frozen=True)
class MoodSound:
    """A mood's voice: its gain and its chatter (neutral has none)."""

    gain_db: float = 0.0
    chatter: Optional[Chatter] = None

    def to_dict(self):
        """Return the YAML form."""
        out = {'gain_db': self.gain_db}
        if self.chatter is not None:
            out['chatter'] = {'every_s': list(self.chatter.every_s),
                              'pools': dict(self.chatter.pools)}
        return out


@dataclass(frozen=True)
class Mood:
    """A palette: a style per channel (only those given), a flourish, a dome and a voice."""

    styles: Mapping[str, Style]
    flourish: Optional[Flourish] = None
    dome: Optional[MoodDome] = None      # None: the neutral dome
    sound: Optional[MoodSound] = None    # None: the neutral gain and no chatter

    def to_dict(self):
        """Return the YAML form."""
        out = {ch: style.to_dict() for ch, style in self.styles.items()}
        if self.flourish is not None:
            out['flourish'] = {'every_s': list(self.flourish.every_s),
                               'picks': [p.to_dict() for p in self.flourish.picks]}
        if self.dome is not None and self.dome.profile:
            out['dome'] = self.dome.to_dict()
        if self.sound is not None:
            out['sound'] = self.sound.to_dict()
        return out


@dataclass(frozen=True)
class ExpressionConfig:
    """A validated ``expression.yaml``. Mappings are read-only and keep file order."""

    defaults: Defaults
    neutral: Mood
    moods: Mapping[str, Mood]
    profiles: Mapping[str, Profile] = MappingProxyType({})
    default_profile: Profile = DEFAULT_PROFILE    # within the caps and the floor
    # What loading clamped or defaulted for the dome; the dome arbiter logs each once.
    dome_warnings: Tuple[str, ...] = field(default=(), compare=False)
    sound_warnings: Tuple[str, ...] = field(default=(), compare=False)

    def to_dict(self):
        """Return the plain-data form, which :func:`parse_config` accepts back."""
        return {
            'defaults': asdict(self.defaults),
            'profiles': {name: asdict(p) for name, p in self.profiles.items()},
            'neutral': self.neutral.to_dict(),
            'moods': {name: mood.to_dict() for name, mood in self.moods.items()},
        }

    def profile(self, name):
        """Return a named profile, '' the default one; KeyError for another name."""
        return self.profiles[name] if name else self.default_profile

    def mood_dome(self, label, stale=False):
        """Return the :class:`MoodDome` for ``label`` (neutral's when stale or absent)."""
        mood = None if stale else self.moods.get(label)
        if mood is not None and mood.dome is not None:
            return mood.dome
        return self.neutral.dome

    def mood_sound(self, label, stale=False):
        """Return the :class:`MoodSound` for ``label`` (neutral's when stale or absent)."""
        mood = None if stale else self.moods.get(label)
        if mood is not None and mood.sound is not None:
            return mood.sound
        return self.neutral.sound or MoodSound()


class _UniqueKeyLoader(yaml.SafeLoader):
    """A SafeLoader that refuses duplicate mapping keys instead of keeping the last."""


def _construct_unique_mapping(loader, node, deep=False):
    seen = set()
    for key_node, _ in node.value:
        if key_node.tag == 'tag:yaml.org,2002:merge':
            continue                # `<<: *style` merges; explicit keys override it
        key = loader.construct_object(key_node, deep=deep)
        if not isinstance(key, Hashable):
            raise ExpressionConfigError('%r: a key must be a plain name (line %d)'
                                        % (key, key_node.start_mark.line + 1))
        if key in seen:
            raise ExpressionConfigError(
                '%r: duplicate key (line %d)' % (key, key_node.start_mark.line + 1))
        seen.add(key)
    return loader.construct_mapping(node, deep=deep)


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_unique_mapping)


def load_config(path):
    """Read, parse and validate the YAML file at ``path``."""
    try:
        with open(path, 'r', encoding='utf-8') as stream:
            text = stream.read()
    except OSError as exc:
        raise ExpressionConfigError(
            '%s: cannot read: %s' % (path, exc.strerror or exc)) from exc
    except UnicodeDecodeError as exc:
        raise ExpressionConfigError('%s: not valid UTF-8: %s' % (path, exc)) from exc
    return load_config_string(text, source=str(path))


def load_config_string(text, source='<string>'):
    """Parse and validate YAML ``text``; ``source`` only labels YAML syntax errors."""
    try:
        data = yaml.load(text, Loader=_UniqueKeyLoader)
    except yaml.YAMLError as exc:
        raise ExpressionConfigError('%s: invalid YAML: %s' % (source, exc)) from exc
    return parse_config(data)


def parse_config(data):
    """Validate already-parsed YAML (nested dicts) and return an :class:`ExpressionConfig`."""
    if not isinstance(data, Mapping):
        raise ExpressionConfigError(
            '<top level>: must be a mapping, got %s' % _type_name(data))
    _reject_hue(data, '')
    _known_keys(data, TOP_LEVEL_KEYS, '', 'top-level key')
    for key in ('neutral', 'moods'):
        if key not in data:
            raise ExpressionConfigError('%s: required key missing' % key)
    defaults = _defaults(data.get('defaults', {}))
    warnings = []
    profiles = _profiles(data.get('profiles', {}), defaults.dome, warnings)
    neutral = _mood(data['neutral'], 'neutral', defaults, profiles, neutral=True)
    table = _mapping(data['moods'], 'moods')
    if not table:
        raise ExpressionConfigError('moods: must name at least one mood')
    moods = {}
    for name, entry in table.items():
        if not isinstance(name, str) or not name:
            raise ExpressionConfigError('moods.%r: names must be non-empty strings' % (name,))
        moods[name] = _mood(entry, 'moods.%s' % name, defaults, profiles, neutral=False)
        if moods[name].dome is None:
            warnings.append('moods.%s: no dome entry; the dome moves as neutral, with no '
                            'glances' % name)
    sound_warnings = tuple("moods.%s: no sound entry; its voice is neutral's gain with no "
                           'chatter' % name for name, mood in moods.items()
                           if mood.sound is None)
    dome = defaults.dome
    default = DEFAULT_PROFILE.capped(dome.max_speed_rad_s, dome.max_accel_rad_s2)
    default = replace(default, v_max=max(default.v_max, dome.min_speed_rad_s))
    return ExpressionConfig(defaults=defaults, neutral=neutral,
                            moods=MappingProxyType(moods),
                            profiles=MappingProxyType(profiles), default_profile=default,
                            dome_warnings=tuple(warnings), sound_warnings=sound_warnings)


def missing_moods(config, labels):
    """Return the labels in ``labels`` that have no entry (they take ``neutral``)."""
    return [label for label in labels if label not in config.moods]


# --- helpers -----------------------------------------------------------------------------

def _type_name(value):
    return 'nothing' if value is None else type(value).__name__


def _join(path, key):
    return '%s.%s' % (path, key) if path else str(key)


def _reject_hue(node, path):
    """Rule 9: no ``hue`` key at any depth, whatever else is wrong around it."""
    if isinstance(node, Mapping):
        for key, value in node.items():
            here = _join(path, key)
            if key == HUE_KEY:
                raise ExpressionConfigError(
                    "%s: hue is the droid's colour scheme, never a mood knob; the "
                    'personality never writes it (rule 9)' % here)
            _reject_hue(value, here)
    elif isinstance(node, (list, tuple)):
        for index, value in enumerate(node):
            _reject_hue(value, '%s[%d]' % (path, index))


def _mapping(value, path):
    if not isinstance(value, Mapping):
        raise ExpressionConfigError(
            '%s: must be a mapping, got %s' % (path, _type_name(value)))
    return value


def _known_keys(mapping, allowed, path, what='key'):
    for key in mapping:
        if key not in allowed:
            raise ExpressionConfigError('%s: unknown %s (expected one of: %s)'
                                        % (_join(path, key), what, ', '.join(allowed)))


def _required(mapping, keys, path):
    for key in keys:
        if key not in mapping:
            raise ExpressionConfigError('%s: required key missing' % _join(path, key))


def _number(value, path):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ExpressionConfigError('%s: must be a number, got %s %r'
                                    % (path, _type_name(value), value))
    try:
        value = float(value)
    except OverflowError:
        raise ExpressionConfigError('%s: out of range, got %r' % (path, value)) from None
    if not math.isfinite(value):
        raise ExpressionConfigError('%s: must be finite, got %r' % (path, value))
    return value


def _integer(value, path, lo, hi):
    number = _number(value, path)
    if not number.is_integer():
        raise ExpressionConfigError('%s: must be an integer, got %r' % (path, value))
    number = int(number)
    if not lo <= number <= hi:
        raise ExpressionConfigError('%s: %d is out of range [%d, %d]' % (path, number, lo, hi))
    return number


def _positive(value, path):
    number = _number(value, path)
    if number <= 0.0:
        raise ExpressionConfigError('%s: must be > 0, got %g' % (path, number))
    return number


def _angle(value, path):
    """Return an angle in (-pi, pi]."""
    number = _number(value, path)
    if not -math.pi < number <= math.pi:
        raise ExpressionConfigError('%s: %g is outside (-pi, pi]' % (path, number))
    return number


def _range(value, path, top=None):
    """Return ``(min, max)`` with 0 < min <= max (<= ``top`` when given)."""
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ExpressionConfigError('%s: must be [min, max]' % path)
    lo = _positive(value[0], '%s[0]' % path)
    hi = _positive(value[1], '%s[1]' % path)
    if lo > hi:
        raise ExpressionConfigError('%s: min %g is above max %g' % (path, lo, hi))
    if top is not None and hi > top:
        raise ExpressionConfigError('%s: max %g is above %g' % (path, hi, top))
    return (lo, hi)


def _defaults(value):
    mapping = _mapping(value, 'defaults')
    _known_keys(mapping, _DEFAULT_FIELDS, 'defaults')
    out = {}
    for key, raw in mapping.items():
        path = 'defaults.%s' % key
        if key == 'max_duration_ms':
            out[key] = _integer(raw, path, 1, MAX_UINT32)
        elif key == 'dome':
            out[key] = _dome_defaults(raw, path)
        elif key == 'sound':
            out[key] = _sound_defaults(raw, path)
        else:
            out[key] = _positive(raw, path)
    return Defaults(**out)


def _sound_defaults(value, path):
    mapping = _mapping(value, path)
    _known_keys(mapping, _SOUND_FIELDS, path)
    out = {}
    for key, raw in mapping.items():
        here = _join(path, key)
        if not isinstance(raw, bool):
            raise ExpressionConfigError('%s: must be true or false, got %r' % (here, raw))
        out[key] = raw
    return SoundDefaults(**out)


_COAST_KEYS = ('c1_s', 'c2_rad', 'learn', 'scale_limits', 'learn_rate')


def _coast_learning(coast, here):
    """Return the validated ``learn``, ``scale_limits`` and ``learn_rate`` of a coast mapping."""
    learn = coast.get('learn', Coast.learn)
    if not isinstance(learn, bool):
        raise ExpressionConfigError('%s.learn: must be true or false, got %r' % (here, learn))
    path = _join(here, 'scale_limits')
    limits = coast.get('scale_limits', Coast.scale_limits)
    if not isinstance(limits, (list, tuple)) or len(limits) != 2:
        raise ExpressionConfigError('%s: must be [min, max], got %r' % (path, limits))
    low, high = (_positive(limits[0], path + '[0]'), _positive(limits[1], path + '[1]'))
    if not low <= 1.0 <= high:
        raise ExpressionConfigError('%s: [%g, %g] must have min <= 1 <= max (1.0 is the fit)'
                                    % (path, low, high))
    rate = _positive(coast.get('learn_rate', Coast.learn_rate), _join(here, 'learn_rate'))
    if rate > 1.0:
        raise ExpressionConfigError('%s.learn_rate: %g is above 1' % (here, rate))
    return {'learn': learn, 'scale_limits': (low, high), 'learn_rate': rate}


def _dome_defaults(value, path):
    mapping = _mapping(value, path)
    _known_keys(mapping, _DOME_FIELDS, path)
    out = {}
    for key, raw in mapping.items():
        here = _join(path, key)
        if key == 'coast':
            coast = _mapping(raw, here)
            _known_keys(coast, _COAST_KEYS, here)
            c2 = _number(coast.get('c2_rad', Coast.c2_rad), _join(here, 'c2_rad'))
            if c2 < 0.0:
                raise ExpressionConfigError('%s.c2_rad: must be >= 0, got %g' % (here, c2))
            out[key] = Coast(
                c1_s=_positive(coast.get('c1_s', Coast.c1_s), _join(here, 'c1_s')), c2_rad=c2,
                **_coast_learning(coast, here))
        elif key == 'floor':
            floor = _mapping(raw, here)
            _known_keys(floor, ('coast_rad', 'speed_rad_s'), here)
            coast_rad = _number(floor.get('coast_rad', Floor.coast_rad),
                                _join(here, 'coast_rad'))
            if coast_rad < 0.0:
                raise ExpressionConfigError('%s.coast_rad: must be >= 0, got %g'
                                            % (here, coast_rad))
            out[key] = Floor(coast_rad=coast_rad, speed_rad_s=_positive(
                floor.get('speed_rad_s', Floor.speed_rad_s), _join(here, 'speed_rad_s')))
        elif key == 'heading':
            heading = _mapping(raw, here)
            _known_keys(heading, ('sign', 'offset_rad'), here)
            sign = heading.get('sign', Heading.sign)
            if sign not in (-1, 1) or isinstance(sign, bool):
                raise ExpressionConfigError('%s.sign: must be -1 or 1, got %r' % (here, sign))
            out[key] = Heading(sign=int(sign), offset_rad=_angle(
                heading.get('offset_rad', 0.0), _join(here, 'offset_rad')))
        elif key == 'retries':
            out[key] = _integer(raw, here, 0, 10)
        elif key == 'feedback_timeout_s':
            out[key] = _positive(raw, here)
            if out[key] < MIN_FEEDBACK_TIMEOUT_S:
                raise ExpressionConfigError(
                    '%s: %g is under %g s (two joint states at 10 Hz): every move would end '
                    'no feedback' % (here, out[key], MIN_FEEDBACK_TIMEOUT_S))
        else:
            out[key] = _positive(raw, here)
    dome = DomeDefaults(**out)
    if dome.min_speed_rad_s > dome.max_speed_rad_s:
        raise ExpressionConfigError('%s.min_speed_rad_s: %g is above max_speed_rad_s %g'
                                    % (path, dome.min_speed_rad_s, dome.max_speed_rad_s))
    return dome


def _profiles(value, dome, warnings):
    table = _mapping(value, 'profiles')
    out = {}
    for name, raw in table.items():
        path = 'profiles.%s' % (name,)
        if not isinstance(name, str) or not name:
            raise ExpressionConfigError('%s: names must be non-empty strings' % path)
        if name == MOOD_PROFILE:
            raise ExpressionConfigError("%s: 'mood' is reserved (an intent's profile: mood "
                                        "is the mood's own)" % path)
        mapping = _mapping(raw, path)
        _known_keys(mapping, _PROFILE_FIELDS, path)
        _required(mapping, _PROFILE_FIELDS, path)
        values = {key: _positive(mapping[key], '%s.%s' % (path, key))
                  for key in _PROFILE_FIELDS}
        if values['settle_band_rad'] > math.pi:
            raise ExpressionConfigError('%s.settle_band_rad: %g is above pi'
                                        % (path, values['settle_band_rad']))
        if values['v_max'] < dome.min_speed_rad_s:
            raise ExpressionConfigError(
                '%s.v_max: %g is below defaults.dome.min_speed_rad_s %g (rule 4: a command is '
                '0 or at least min_speed)' % (path, values['v_max'], dome.min_speed_rad_s))
        for key, cap, cap_name in (('v_max', dome.max_speed_rad_s, 'max_speed_rad_s'),
                                   ('accel', dome.max_accel_rad_s2, 'max_accel_rad_s2')):
            if values[key] > cap:
                warnings.append('%s.%s: %g is above defaults.dome.%s; clamped to %g'
                                % (path, key, values[key], cap_name, cap))
                values[key] = cap
        out[name] = Profile(**values)
    return out


def _mood_dome(value, path, profiles, neutral, max_move_s):
    mapping = _mapping(value, path)
    for key in ('glance', 'rest_rad'):
        if neutral and key in mapping:
            raise ExpressionConfigError('%s.%s: neutral has no glances and no rest (they '
                                        'stop while the mood is stale)' % (path, key))
    _known_keys(mapping, ('profile', 'glance', 'rest_rad'), path)
    _required(mapping, ('profile',), path)
    profile = mapping['profile']
    if not isinstance(profile, str) or profile not in profiles:
        raise ExpressionConfigError('%s.profile: unknown profile %r (profiles: %s)' % (
            path, profile, ', '.join(profiles) or 'none'))
    glance = None
    if 'glance' in mapping:
        gpath = '%s.glance' % path
        gmap = _mapping(mapping['glance'], gpath)
        _known_keys(gmap, _GLANCE_KEYS, gpath)
        _required(gmap, _GLANCE_KEYS, gpath)
        glance = Glance(every_s=_range(gmap['every_s'], '%s.every_s' % gpath),
                        amplitude_rad=_range(gmap['amplitude_rad'],
                                             '%s.amplitude_rad' % gpath, top=math.pi),
                        dwell_s=_range(gmap['dwell_s'], '%s.dwell_s' % gpath,
                                       top=max_move_s))
    rest_rad = None
    if 'rest_rad' in mapping:
        rest_rad = _angle(mapping['rest_rad'], '%s.rest_rad' % path)
    return MoodDome(profile=profile, glance=glance, rest_rad=rest_rad)


def _brightness(value, path, channel, allow_range):
    lo, hi = BRIGHTNESS_RANGE[channel]
    if isinstance(value, (list, tuple)):
        if not allow_range:
            raise ExpressionConfigError('%s: must be a single number here (no strength to '
                                        'scale a range by)' % path)
        if len(value) != 2:
            raise ExpressionConfigError('%s: a range is [min, max], got %d values'
                                        % (path, len(value)))
        bmin = _integer(value[0], '%s[0]' % path, lo, hi)
        bmax = _integer(value[1], '%s[1]' % path, lo, hi)
        if bmin > bmax:
            raise ExpressionConfigError('%s: min %d is above max %d' % (path, bmin, bmax))
        return (bmin, bmax)
    number = _integer(value, path, lo, hi)
    return (number, number)


def _style(channel, value, path, neutral):
    mapping = _mapping(value, path)
    allowed = ('drift', 'brightness') if channel in _DRIFT_CHANNELS else ('brightness',)
    for key in mapping:
        if key == 'drift' and channel not in _DRIFT_CHANNELS:
            raise ExpressionConfigError('%s.drift: display has no drift (%s styles '
                                        'brightness only)' % (path, channel))
    _known_keys(mapping, allowed, path)
    drift = None
    if 'drift' in mapping:
        drift = mapping['drift']
        if drift not in DRIFT_PRESETS:
            raise ExpressionConfigError('%s.drift: unknown drift preset %r (expected one of: '
                                        '%s)' % (path, drift, ', '.join(DRIFT_PRESETS)))
    brightness = None
    if 'brightness' in mapping:
        brightness = _brightness(mapping['brightness'], '%s.brightness' % path, channel,
                                 allow_range=not neutral)
    if drift is None and brightness is None:
        raise ExpressionConfigError('%s: a style needs %s' % (path, ' or '.join(allowed)))
    return Style(drift=drift, brightness=brightness)


def _mood_sound(value, path, neutral):
    mapping = _mapping(value, path)
    if neutral and 'chatter' in mapping:
        raise ExpressionConfigError('%s.chatter: neutral has no chatter (it stops while the '
                                    'mood is stale)' % path)
    _known_keys(mapping, ('gain_db', 'chatter'), path)
    gain = _number(mapping.get('gain_db', 0.0), _join(path, 'gain_db'))
    lo, hi = GAIN_RANGE_DB
    if not lo <= gain <= hi:
        raise ExpressionConfigError('%s.gain_db: %g is outside %g to %g dB'
                                    % (path, gain, lo, hi))
    chatter = None
    if 'chatter' in mapping:
        cpath = _join(path, 'chatter')
        cmap = _mapping(mapping['chatter'], cpath)
        _known_keys(cmap, ('every_s', 'pools'), cpath)
        _required(cmap, ('every_s', 'pools'), cpath)
        pools = _mapping(cmap['pools'], _join(cpath, 'pools'))
        if not pools:
            raise ExpressionConfigError('%s.pools: must name at least one pool' % cpath)
        table = []
        for tag, weight in pools.items():
            here = '%s.pools.%s' % (cpath, tag)
            if tag not in CHATTER_POOLS:
                raise ExpressionConfigError('%s: unknown pool (expected one of: %s)' % (
                    here, ', '.join(CHATTER_POOLS)))
            table.append((tag, _positive(weight, here)))
        chatter = Chatter(every_s=_range(cmap['every_s'], _join(cpath, 'every_s')),
                          pools=tuple(table))
    return MoodSound(gain_db=gain, chatter=chatter)


def _mood(value, path, defaults, profiles, neutral):
    mapping = _mapping(value, path)
    allowed = STYLE_CHANNELS + (('dome', 'sound') if neutral else ('flourish', 'dome', 'sound'))
    for key in mapping:
        if key not in allowed:
            if key == 'flourish':
                raise ExpressionConfigError('%s.flourish: neutral has no flourishes (they '
                                            'stop while the mood is stale)' % path)
            raise ExpressionConfigError(
                '%s: unknown channel (styles are per %s; tfld and bfld follow fld%s)'
                % (_join(path, key), ', '.join(STYLE_CHANNELS),
                   '' if neutral else '; flourishes go under flourish'))
    styles = {ch: _style(ch, mapping[ch], '%s.%s' % (path, ch), neutral)
              for ch in STYLE_CHANNELS if ch in mapping}
    flourish = None
    if 'flourish' in mapping:
        flourish = _flourish(mapping['flourish'], '%s.flourish' % path, defaults)
    dome = MoodDome() if neutral else None
    if 'dome' in mapping:
        dome = _mood_dome(mapping['dome'], '%s.dome' % path, profiles, neutral,
                          defaults.dome.max_move_s)
    sound = _mood_sound(mapping['sound'], _join(path, 'sound'), neutral) \
        if 'sound' in mapping else None
    return Mood(styles=MappingProxyType(styles), flourish=flourish, dome=dome, sound=sound)


def _flourish(value, path, defaults):
    mapping = _mapping(value, path)
    _known_keys(mapping, ('every_s', 'picks'), path)
    _required(mapping, ('every_s', 'picks'), path)
    every = _range(mapping['every_s'], '%s.every_s' % path)
    picks = mapping['picks']
    ppath = '%s.picks' % path
    if not isinstance(picks, (list, tuple)) or not picks:
        raise ExpressionConfigError('%s: must be a non-empty list' % ppath)
    out = tuple(_pick(p, '%s[%d]' % (ppath, i), defaults) for i, p in enumerate(picks))
    return Flourish(every_s=every, picks=out)


def _pick(value, path, defaults):
    mapping = _mapping(value, path)
    _known_keys(mapping, _PICK_KEYS, path)
    _required(mapping, ('channel',), path)
    channel = mapping['channel']
    if channel not in PICK_CHANNELS:
        raise ExpressionConfigError('%s.channel: unknown channel %r (expected one of: %s)'
                                    % (path, channel, ', '.join(PICK_CHANNELS)))
    kinds = [k for k in PICK_KINDS if k in mapping]
    if len(kinds) != 1:
        raise ExpressionConfigError('%s: a pick names exactly one of %s, got %s'
                                    % (path, ', '.join(PICK_KINDS),
                                       ', '.join(kinds) if kinds else 'none'))
    kind = kinds[0]
    magic = channel == 'magic'
    if magic != (kind == 'sequence'):
        raise ExpressionConfigError('%s.%s: %s picks name %s' % (
            path, kind, channel, 'a sequence' if magic else 'an effect or a clip'))
    if kind == 'clip' and channel in CLIPLESS_CHANNELS:
        raise ExpressionConfigError('%s.clip: no clip for this surface (%s has no clip '
                                    'target; use fld)' % (path, channel))
    wrong = _LED_ONLY_PICK_KEYS if magic else _MAGIC_ONLY_PICK_KEYS
    for key in wrong:
        if key in mapping:
            raise ExpressionConfigError('%s.%s: not a %s setting' % (
                path, key, 'Magic Panel' if magic else 'LED'))

    raw = mapping[kind]
    if kind == 'sequence':
        if isinstance(raw, str):
            if not raw:
                raise ExpressionConfigError('%s.sequence: empty name' % path)
            name = raw
        else:
            name = _integer(raw, '%s.sequence' % path, 0, 255)
    else:
        if not isinstance(raw, str) or not raw:
            raise ExpressionConfigError('%s.%s: must be a non-empty name' % (path, kind))
        name = raw

    fields = {}
    if 'weight' in mapping:
        fields['weight'] = _positive(mapping['weight'], '%s.weight' % path)
    if 'rgb' in mapping:
        fields['rgb'] = _integer(mapping['rgb'], '%s.rgb' % path, 0, 0xFFFFFF)
    if 'speed' in mapping:
        fields['speed'] = _integer(mapping['speed'], '%s.speed' % path, 1, 9)
    if 'repeat' in mapping:
        fields['repeat'] = _integer(mapping['repeat'], '%s.repeat' % path, 1, 255)
    if 'end' in mapping:
        end = mapping['end']
        if end not in END_MODES:
            raise ExpressionConfigError('%s.end: must be one of %s, got %r'
                                        % (path, ', '.join(END_MODES), end))
        fields['end'] = end
    if 'duration_ms' in mapping:
        dpath = '%s.duration_ms' % path
        duration = _integer(mapping['duration_ms'], dpath, 0, MAX_UINT32)
        if duration == 0:
            raise ExpressionConfigError('%s: 0 means "until stopped" and the personality '
                                        'never sends it (rule 3); give a duration' % dpath)
        fields['duration_ms'] = min(duration, defaults.max_duration_ms)
    return Pick(channel=channel, kind=kind, name=name, **fields)
