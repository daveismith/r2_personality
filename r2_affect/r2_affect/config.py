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
Load and validate ``affect.yaml``, the whole configuration of the mood model.

Pure Python (PyYAML, no ROS imports). The result is an immutable
:class:`AffectConfig`; the model and the node never see raw YAML. The schema:

* ``temperament`` (required): the baseline each axis relaxes toward, one value
  per axis, inside that axis's range.
* ``relax_tau_s`` (required): the relaxation time constant of each axis, in
  seconds, one value per axis, each > 0.
* ``distance_weights`` (optional): a per-axis weight for mood classification;
  axes left out weigh 1.0. Each >= 0.
* ``strength_radius`` (optional, 0.5): the distance at which ``mood_strength``
  reaches 0. > 0.
* ``hysteresis`` (optional): ``margin`` (0.15, >= 0) and ``dwell_s``
  (1.5, >= 0).
* ``focus`` (optional): ``default_ttl_s`` (5.0, > 0), used when a stimulus
  carries ``ttl_s`` 0.
* ``idle`` (optional): ``after_s`` (30, >= 0) and ``ramp_s`` (120, > 0).
* ``impulses`` (required): stimulus kind -> per-axis impulse, plus an optional
  ``refractory_s`` (>= 0). The ``idle`` kind is required: it is the rate, per
  second, applied while nothing has happened for ``idle.after_s``.
* ``moods`` (required, non-empty): label -> prototype centre over one or more
  axes, each inside its range. File order is kept and breaks distance ties.

Axes and their ranges are fixed here (:data:`AXES`, :data:`AXIS_RANGES`); a
YAML file cannot add one. Every rejection raises :class:`AffectConfigError`
whose message starts with the dotted path of the offending key.
"""

from dataclasses import dataclass
import math
from types import MappingProxyType
from typing import Mapping, Optional

import yaml

AXES = ('arousal', 'valence', 'curiosity', 'confidence')
AXIS_RANGES = MappingProxyType({
    'arousal': (-1.0, 1.0),
    'valence': (-1.0, 1.0),
    'curiosity': (0.0, 1.0),
    'confidence': (0.0, 1.0),
})

REFRACTORY_KEY = 'refractory_s'
IDLE_KIND = 'idle'

DEFAULT_STRENGTH_RADIUS = 0.5
DEFAULT_MARGIN = 0.15
DEFAULT_DWELL_S = 1.5
DEFAULT_FOCUS_TTL_S = 5.0
DEFAULT_IDLE_AFTER_S = 30.0
DEFAULT_IDLE_RAMP_S = 120.0

TOP_LEVEL_KEYS = ('temperament', 'relax_tau_s', 'distance_weights', 'strength_radius',
                  'hysteresis', 'focus', 'idle', 'impulses', 'moods')
_REQUIRED_KEYS = ('temperament', 'relax_tau_s', 'impulses', 'moods')


class AffectConfigError(ValueError):
    """An ``affect.yaml`` that cannot be used; the message names the key."""


@dataclass(frozen=True)
class Hysteresis:
    """How far and for how long a new mood must win before the label moves."""

    margin: float = DEFAULT_MARGIN
    dwell_s: float = DEFAULT_DWELL_S


@dataclass(frozen=True)
class FocusConfig:
    """Focus defaults."""

    default_ttl_s: float = DEFAULT_FOCUS_TTL_S


@dataclass(frozen=True)
class IdleConfig:
    """When the ``idle`` impulse starts and how long it takes to reach full rate."""

    after_s: float = DEFAULT_IDLE_AFTER_S
    ramp_s: float = DEFAULT_IDLE_RAMP_S


@dataclass(frozen=True)
class Impulse:
    """The effect of one stimulus kind: per-axis deltas at intensity 1."""

    kind: str
    deltas: Mapping[str, float]          # only the axes the YAML names
    refractory_s: Optional[float] = None


@dataclass(frozen=True)
class Prototype:
    """A mood's centre, over the axes the YAML names for it."""

    name: str
    center: Mapping[str, float]


@dataclass(frozen=True)
class AffectConfig:
    """A validated ``affect.yaml``. Mappings are read-only and keep file order."""

    temperament: Mapping[str, float]
    relax_tau_s: Mapping[str, float]
    distance_weights: Mapping[str, float]   # every axis, defaults filled in
    strength_radius: float
    hysteresis: Hysteresis
    focus: FocusConfig
    idle: IdleConfig
    impulses: Mapping[str, Impulse]
    moods: Mapping[str, Prototype]

    def to_dict(self):
        """Return the plain-data form, which :func:`parse_config` accepts back."""
        return {
            'temperament': dict(self.temperament),
            'relax_tau_s': dict(self.relax_tau_s),
            'distance_weights': dict(self.distance_weights),
            'strength_radius': self.strength_radius,
            'hysteresis': {'margin': self.hysteresis.margin,
                           'dwell_s': self.hysteresis.dwell_s},
            'focus': {'default_ttl_s': self.focus.default_ttl_s},
            'idle': {'after_s': self.idle.after_s, 'ramp_s': self.idle.ramp_s},
            'impulses': {
                kind: dict(imp.deltas, **({REFRACTORY_KEY: imp.refractory_s}
                                          if imp.refractory_s is not None else {}))
                for kind, imp in self.impulses.items()},
            'moods': {name: dict(p.center) for name, p in self.moods.items()},
        }


class _UniqueKeyLoader(yaml.SafeLoader):
    """A SafeLoader that refuses duplicate mapping keys instead of keeping the last."""


def _construct_unique_mapping(loader, node, deep=False):
    seen = set()
    for key_node, _ in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in seen:
            raise AffectConfigError(
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
        raise AffectConfigError('%s: cannot read: %s' % (path, exc.strerror or exc)) from exc
    return load_config_string(text, source=str(path))


def load_config_string(text, source='<string>'):
    """Parse and validate YAML ``text``; ``source`` only labels YAML syntax errors."""
    try:
        data = yaml.load(text, Loader=_UniqueKeyLoader)
    except yaml.YAMLError as exc:
        raise AffectConfigError('%s: invalid YAML: %s' % (source, exc)) from exc
    return parse_config(data)


def parse_config(data):
    """Validate already-parsed YAML (nested dicts) and return an :class:`AffectConfig`."""
    if not isinstance(data, Mapping):
        raise AffectConfigError('<top level>: must be a mapping, got %s' % _type_name(data))
    for key in data:
        if key not in TOP_LEVEL_KEYS:
            raise AffectConfigError('%s: unknown top-level key (expected one of: %s)'
                                    % (key, ', '.join(TOP_LEVEL_KEYS)))
    for key in _REQUIRED_KEYS:
        if key not in data:
            raise AffectConfigError('%s: required key missing' % key)

    temperament = _axis_values(data['temperament'], 'temperament', require_all=True,
                               check=_in_range)
    relax_tau_s = _axis_values(data['relax_tau_s'], 'relax_tau_s', require_all=True,
                               check=_positive)
    weights = {axis: 1.0 for axis in AXES}
    if 'distance_weights' in data:
        weights.update(_axis_values(data['distance_weights'], 'distance_weights',
                                    require_all=False, check=_non_negative))
    strength_radius = DEFAULT_STRENGTH_RADIUS
    if 'strength_radius' in data:
        strength_radius = _number(data['strength_radius'], 'strength_radius')
        _positive(strength_radius, 'strength_radius', None)

    hyst = _section(data, 'hysteresis', {'margin': (DEFAULT_MARGIN, _non_negative),
                                         'dwell_s': (DEFAULT_DWELL_S, _non_negative)})
    focus = _section(data, 'focus', {'default_ttl_s': (DEFAULT_FOCUS_TTL_S, _positive)})
    idle = _section(data, 'idle', {'after_s': (DEFAULT_IDLE_AFTER_S, _non_negative),
                                   'ramp_s': (DEFAULT_IDLE_RAMP_S, _positive)})

    return AffectConfig(
        temperament=_frozen(temperament),
        relax_tau_s=_frozen(relax_tau_s),
        distance_weights=_frozen(weights),
        strength_radius=strength_radius,
        hysteresis=Hysteresis(**hyst),
        focus=FocusConfig(**focus),
        idle=IdleConfig(**idle),
        impulses=_impulses(data['impulses']),
        moods=_moods(data['moods']),
    )


# --- helpers -----------------------------------------------------------------------------

def _frozen(mapping):
    return MappingProxyType(dict(mapping))


def _type_name(value):
    return 'nothing' if value is None else type(value).__name__


def _number(value, path):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise AffectConfigError('%s: must be a number, got %s %r'
                                % (path, _type_name(value), value))
    value = float(value)
    if not math.isfinite(value):
        raise AffectConfigError('%s: must be finite, got %r' % (path, value))
    return value


def _mapping(value, path):
    if not isinstance(value, Mapping):
        raise AffectConfigError('%s: must be a mapping, got %s' % (path, _type_name(value)))
    return value


def _name(key, path):
    if not isinstance(key, str) or not key:
        raise AffectConfigError('%s.%r: names must be non-empty strings' % (path, key))
    return key


def _unknown_axis(path, key):
    return AffectConfigError('%s.%s: unknown axis %r (axes are: %s)'
                             % (path, key, key, ', '.join(AXES)))


def _in_range(value, path, axis):
    lo, hi = AXIS_RANGES[axis]
    if not lo <= value <= hi:
        raise AffectConfigError('%s: %g is outside the %s range [%g, %g]'
                                % (path, value, axis, lo, hi))


def _positive(value, path, axis):
    if value <= 0.0:
        raise AffectConfigError('%s: must be > 0, got %g' % (path, value))


def _non_negative(value, path, axis):
    if value < 0.0:
        raise AffectConfigError('%s: must be >= 0, got %g' % (path, value))


def _axis_values(value, path, require_all, check=None, extra_keys=()):
    """Validate an axis -> number mapping; return it as a dict in AXES order."""
    mapping = _mapping(value, path)
    out = {}
    for key, raw in mapping.items():
        if key in extra_keys:
            continue
        if key not in AXIS_RANGES:
            raise _unknown_axis(path, key)
        number = _number(raw, '%s.%s' % (path, key))
        if check is not None:
            check(number, '%s.%s' % (path, key), key)
        out[key] = number
    if require_all:
        for axis in AXES:
            if axis not in out:
                raise AffectConfigError('%s.%s: required axis missing' % (path, axis))
    return {axis: out[axis] for axis in AXES if axis in out}


def _section(data, path, fields):
    """Validate an optional mapping of named numbers; fill defaults for absent fields."""
    out = {name: default for name, (default, _) in fields.items()}
    if path not in data:
        return out
    mapping = _mapping(data[path], path)
    for key, raw in mapping.items():
        if key not in fields:
            raise AffectConfigError('%s.%s: unknown key (expected one of: %s)'
                                    % (path, key, ', '.join(fields)))
        number = _number(raw, '%s.%s' % (path, key))
        fields[key][1](number, '%s.%s' % (path, key), None)
        out[key] = number
    return out


def _impulses(value):
    table = _mapping(value, 'impulses')
    out = {}
    for kind, entry in table.items():
        path = 'impulses.%s' % _name(kind, 'impulses')
        entry = _mapping(entry, path)
        deltas = _axis_values(entry, path, require_all=False, extra_keys=(REFRACTORY_KEY,))
        refractory = None
        if REFRACTORY_KEY in entry:
            refractory = _number(entry[REFRACTORY_KEY], '%s.%s' % (path, REFRACTORY_KEY))
            _non_negative(refractory, '%s.%s' % (path, REFRACTORY_KEY), None)
        out[kind] = Impulse(kind=kind, deltas=_frozen(deltas), refractory_s=refractory)
    if IDLE_KIND not in out:
        raise AffectConfigError('impulses.%s: required impulse missing (it is the drift '
                                'rate applied while idle)' % IDLE_KIND)
    return _frozen(out)


def _moods(value):
    table = _mapping(value, 'moods')
    if not table:
        raise AffectConfigError('moods: must name at least one mood')
    out = {}
    for name, entry in table.items():
        path = 'moods.%s' % _name(name, 'moods')
        center = _axis_values(entry, path, require_all=False, check=_in_range)
        if not center:
            raise AffectConfigError('%s: a prototype must specify at least one axis' % path)
        out[name] = Prototype(name=name, center=_frozen(center))
    return _frozen(out)
