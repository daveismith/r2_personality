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
  ``max_lease_s`` (60.0), ``device_stale_s`` (5.0). Each > 0.
* ``neutral`` (required): the styles used while ``/mood/state`` is stale and
  for any mood without its own entry. Per style channel, as below, but
  ``brightness`` is a single number (there is no strength to scale by) and
  there is no ``flourish``. A channel left out is not touched while neutral.
* ``moods`` (required, non-empty): label -> mood. A mood has a style per
  channel and an optional ``flourish``; a channel it leaves out takes the
  neutral style.

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

from dataclasses import asdict, dataclass, fields
import math
from types import MappingProxyType
from typing import Hashable, Mapping, Optional, Tuple, Union

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
TOP_LEVEL_KEYS = ('defaults', 'neutral', 'moods')
_PICK_KEYS = ('channel',) + PICK_KINDS + ('weight', 'rgb', 'speed', 'repeat', 'end',
                                          'duration_ms')
_LED_ONLY_PICK_KEYS = ('rgb', 'speed')
_MAGIC_ONLY_PICK_KEYS = ('repeat', 'end')
MAX_UINT32 = 4294967295


class ExpressionConfigError(ValueError):
    """An ``expression.yaml`` that cannot be used; the message names the key."""


@dataclass(frozen=True)
class Defaults:
    """The arbiters' timing settings."""

    manual_hold_s: float = 5.0
    mood_stale_s: float = 3.0
    max_duration_ms: int = 30000
    status_hz: float = 2.0
    max_lease_s: float = 60.0
    device_stale_s: float = 5.0


_DEFAULT_FIELDS = tuple(f.name for f in fields(Defaults))


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
class Mood:
    """A palette: a style per channel (only those given) and an optional flourish."""

    styles: Mapping[str, Style]
    flourish: Optional[Flourish] = None

    def to_dict(self):
        """Return the YAML form."""
        out = {ch: style.to_dict() for ch, style in self.styles.items()}
        if self.flourish is not None:
            out['flourish'] = {'every_s': list(self.flourish.every_s),
                               'picks': [p.to_dict() for p in self.flourish.picks]}
        return out


@dataclass(frozen=True)
class ExpressionConfig:
    """A validated ``expression.yaml``. Mappings are read-only and keep file order."""

    defaults: Defaults
    neutral: Mood
    moods: Mapping[str, Mood]

    def to_dict(self):
        """Return the plain-data form, which :func:`parse_config` accepts back."""
        return {
            'defaults': asdict(self.defaults),
            'neutral': self.neutral.to_dict(),
            'moods': {name: mood.to_dict() for name, mood in self.moods.items()},
        }


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
    neutral = _mood(data['neutral'], 'neutral', defaults, neutral=True)
    table = _mapping(data['moods'], 'moods')
    if not table:
        raise ExpressionConfigError('moods: must name at least one mood')
    moods = {}
    for name, entry in table.items():
        if not isinstance(name, str) or not name:
            raise ExpressionConfigError('moods.%r: names must be non-empty strings' % (name,))
        moods[name] = _mood(entry, 'moods.%s' % name, defaults, neutral=False)
    return ExpressionConfig(defaults=defaults, neutral=neutral,
                            moods=MappingProxyType(moods))


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


def _defaults(value):
    mapping = _mapping(value, 'defaults')
    _known_keys(mapping, _DEFAULT_FIELDS, 'defaults')
    out = {}
    for key, raw in mapping.items():
        path = 'defaults.%s' % key
        if key == 'max_duration_ms':
            out[key] = _integer(raw, path, 1, MAX_UINT32)
        else:
            out[key] = _positive(raw, path)
    return Defaults(**out)


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


def _mood(value, path, defaults, neutral):
    mapping = _mapping(value, path)
    allowed = STYLE_CHANNELS if neutral else STYLE_CHANNELS + ('flourish',)
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
    return Mood(styles=MappingProxyType(styles), flourish=flourish)


def _flourish(value, path, defaults):
    mapping = _mapping(value, path)
    _known_keys(mapping, ('every_s', 'picks'), path)
    for key in ('every_s', 'picks'):
        if key not in mapping:
            raise ExpressionConfigError('%s.%s: required key missing' % (path, key))
    every = mapping['every_s']
    epath = '%s.every_s' % path
    if not isinstance(every, (list, tuple)) or len(every) != 2:
        raise ExpressionConfigError('%s: must be [min, max] in seconds' % epath)
    emin = _positive(every[0], '%s[0]' % epath)
    emax = _positive(every[1], '%s[1]' % epath)
    if emin > emax:
        raise ExpressionConfigError('%s: min %g is above max %g' % (epath, emin, emax))
    picks = mapping['picks']
    ppath = '%s.picks' % path
    if not isinstance(picks, (list, tuple)) or not picks:
        raise ExpressionConfigError('%s: must be a non-empty list' % ppath)
    out = tuple(_pick(p, '%s[%d]' % (ppath, i), defaults) for i, p in enumerate(picks))
    return Flourish(every_s=(emin, emax), picks=out)


def _pick(value, path, defaults):
    mapping = _mapping(value, path)
    _known_keys(mapping, _PICK_KEYS, path)
    if 'channel' not in mapping:
        raise ExpressionConfigError('%s.channel: required key missing' % path)
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
