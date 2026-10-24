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
Load and validate ``clips.yaml``, the sound catalogue (plan 2.1-2.3, 5.2).

Pure Python (PyYAML, no ROS imports). ``build_catalogue`` writes the file; the player
and the arbiter read it through :func:`load_catalogue`, which returns immutable
dataclasses. The schema::

    house_level_dbfs: -16.0        # what gain_db targets
    generated: '2026-10-24'        # date of the last change of the content
    sets:                          # summary, one entry per sound set
      R2 Padawan: {clips: 55, median_rms_dbfs: -13.3, set_gain_db: -2.7}
    clips:                         # sorted by name
    - name: padawan-sent01         # unique, lower-case
      set: R2 Padawan
      path: R2 Padawan/32SENT1.mp3 # relative to sounds_dir
      duration_ms: 2100
      rms_dbfs: -13.1              # over the non-silent part
      peak_dbfs: -0.4
      clip_fraction: 0.0           # share of samples at or above 0.999
      set_gain_db: -2.7
      gain_db: -2.7                # set gain + outlier trim + override, peak-limited
      tags: [chatter]              # effective pools, see effective_tags
      suggested_tag: chatter       # the first pass of build_catalogue
      confidence: 1.0              # of the suggestion, 0..1
      excluded: false              # music, speech, silence: manual play only
      label: chatter               # from the file name, or ''

Rejected: unknown keys, duplicate names, tags outside :data:`TAGS`, a ``gain_db`` outside
-20..+12, a ``confidence`` outside 0..1. Every rejection raises :class:`CatalogueError`
whose message starts with the dotted path of the offending key.

A clip whose file is missing under ``sounds_dir`` is dropped with a warning (rule 7: a
missing file never crashes the player); the warnings come back beside the catalogue.
"""

from dataclasses import dataclass
import math
import os
from types import MappingProxyType
from typing import Mapping, Optional, Sequence, Tuple

from r2_sound.gain import MAX_GAIN_DB, MIN_GAIN_DB
import yaml

TAGS = ('chatter', 'excited', 'sad', 'question', 'ooh', 'hum', 'alarm', 'scream', 'laugh',
        'annoyed', 'whistle', 'calm', 'scared', 'sleepy', 'greeting', 'misc', 'excluded')
EXCLUDED_TAG = 'excluded'
FALLBACK_TAG = 'chatter'
DEFAULT_THRESHOLD = 0.6
# An excluded clip plays only at this priority or above: PRIORITY_MANUAL - 1, the same value
# as r2_expression.core.MANUAL_PLAY_PRIORITY, the arbiter's own check of the rule.
MANUAL_PRIORITY = 199
EXCLUDED_REFUSAL = 'excluded: manual play only'   # the play's reason below it
CLIPPED_FRACTION = 0.01          # a clip clipped more than this is picked at half weight

_TOP_KEYS = ('house_level_dbfs', 'generated', 'sets', 'clips')
_SET_KEYS = ('clips', 'median_rms_dbfs', 'set_gain_db')
_CLIP_KEYS = ('name', 'set', 'path', 'duration_ms', 'rms_dbfs', 'peak_dbfs', 'clip_fraction',
              'set_gain_db', 'gain_db', 'tags', 'suggested_tag', 'confidence', 'excluded',
              'label')


class CatalogueError(ValueError):
    """A ``clips.yaml`` that cannot be used; the message names the key."""


@dataclass(frozen=True)
class SetInfo:
    """One sound set's summary."""

    clips: int
    median_rms_dbfs: float
    set_gain_db: float


@dataclass(frozen=True)
class SoundClip:
    """One catalogue entry."""

    name: str
    set_name: str                 # the ``set`` key of the YAML
    path: str
    duration_ms: int
    rms_dbfs: float
    peak_dbfs: float
    clip_fraction: float
    set_gain_db: float
    gain_db: float
    tags: Tuple[str, ...]
    suggested_tag: str
    confidence: float
    excluded: bool
    label: str = ''

    def to_dict(self):
        """Return the YAML form."""
        return {'name': self.name, 'set': self.set_name, 'path': self.path,
                'duration_ms': self.duration_ms, 'rms_dbfs': self.rms_dbfs,
                'peak_dbfs': self.peak_dbfs, 'clip_fraction': self.clip_fraction,
                'set_gain_db': self.set_gain_db, 'gain_db': self.gain_db,
                'tags': list(self.tags), 'suggested_tag': self.suggested_tag,
                'confidence': self.confidence, 'excluded': self.excluded,
                'label': self.label}


@dataclass(frozen=True)
class Catalogue:
    """The validated catalogue."""

    house_level_dbfs: float
    generated: str
    sets: Mapping[str, SetInfo]
    clips: Tuple[SoundClip, ...]

    def by_name(self):
        """Return name -> clip."""
        return {c.name: c for c in self.clips}

    def to_dict(self):
        """Return the YAML form."""
        return {'house_level_dbfs': self.house_level_dbfs, 'generated': self.generated,
                'sets': {name: {'clips': s.clips, 'median_rms_dbfs': s.median_rms_dbfs,
                                'set_gain_db': s.set_gain_db}
                         for name, s in self.sets.items()},
                'clips': [c.to_dict() for c in self.clips]}


def effective_tags(sheet_tags: Sequence[str], suggested_tag: str, confidence: float,
                   threshold: float = DEFAULT_THRESHOLD) -> Tuple[str, ...]:
    """
    Return a clip's pools: the sheet's tags if any, else the suggestion, else ``chatter``.

    The suggestion counts when its confidence is at or above ``threshold``.
    """
    if sheet_tags:
        return tuple(sheet_tags)
    if suggested_tag and confidence >= threshold:
        return (suggested_tag,)
    return (FALLBACK_TAG,)


def pools(catalogue: Catalogue) -> Mapping[str, Tuple[SoundClip, ...]]:
    """Return tag -> the playable (not excluded) clips carrying it; every tag has a key."""
    out = {tag: [] for tag in TAGS}
    for clip in catalogue.clips:
        if not clip.excluded:
            for tag in clip.tags:
                out[tag].append(clip)
    return MappingProxyType({tag: tuple(clips) for tag, clips in out.items()})


def pick_weights(clips: Sequence[SoundClip]):
    """Return each clip's pick weight: 1, or 0.5 when ``clip_fraction`` is over 0.01."""
    return [0.5 if c.clip_fraction > CLIPPED_FRACTION else 1.0 for c in clips]


def pick(clips: Sequence[SoundClip], rng):
    """Return one of ``clips`` (non-empty), uniformly but for the clipped ones' half weight."""
    return rng.choices(clips, weights=pick_weights(clips))[0]


def resolve_play(by_name, pool_map, name, pool, priority, rng):
    """
    Return ``(clip, '')`` for a play request, or ``(None, reason)``.

    ``name`` wins over ``pool``. The reasons are the player's refusals: ``unknown name``,
    ``excluded: manual play only`` (an excluded clip below :data:`MANUAL_PRIORITY`),
    ``unknown pool`` (not in :data:`TAGS`), ``empty pool`` (no playable clip carries the tag).
    """
    if name:
        clip = by_name.get(name)
        if clip is None:
            return None, 'unknown name'
        if clip.excluded and priority < MANUAL_PRIORITY:
            return None, EXCLUDED_REFUSAL
        return clip, ''
    if pool not in pool_map:
        return None, 'unknown pool'
    if not pool_map[pool]:
        return None, 'empty pool'
    return pick(pool_map[pool], rng), ''


def _fail(path, message):
    raise CatalogueError(f'{path}: {message}')


def _mapping(value, path, allowed):
    if not isinstance(value, dict):
        _fail(path, f'must be a mapping, got {type(value).__name__}')
    for key in value:
        if key not in allowed:
            _fail(f'{path}.{key}' if path else str(key), 'unknown key')
    return value


def _number(value, path, low=-math.inf, high=math.inf):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        _fail(path, f'must be a number, got {value!r}')
    if not math.isfinite(value) or not low <= value <= high:
        _fail(path, f'{value!r} is outside {low}..{high}')
    return float(value)


def _text(value, path):
    if not isinstance(value, str) or not value:
        _fail(path, f'must be a non-empty string, got {value!r}')
    return value


def _tag(value, path):
    if value not in TAGS:
        _fail(path, f'{value!r} is not one of {", ".join(TAGS)}')
    return value


def _parse_clip(raw, path):
    raw = _mapping(raw, path, _CLIP_KEYS)
    for key in _CLIP_KEYS:
        if key not in raw and key != 'label':
            _fail(f'{path}.{key}', 'is required')
    name = _text(raw['name'], f'{path}.name')
    if name != name.lower() or name.strip() != name:
        _fail(f'{path}.name', f'{name!r} must be lower-case without padding')
    tags = raw['tags']
    if not isinstance(tags, list) or not tags:
        _fail(f'{path}.tags', 'must be a non-empty list')
    duration = raw['duration_ms']
    if isinstance(duration, bool) or not isinstance(duration, int) or duration < 0:
        _fail(f'{path}.duration_ms', f'must be an integer >= 0, got {duration!r}')
    excluded = raw['excluded']
    if not isinstance(excluded, bool):
        _fail(f'{path}.excluded', f'must be true or false, got {excluded!r}')
    if (EXCLUDED_TAG in tags) != excluded and EXCLUDED_TAG in tags:
        _fail(f'{path}.excluded', f"must be true when tags has '{EXCLUDED_TAG}'")
    label = raw.get('label', '')
    if label != '':
        _tag(label, f'{path}.label')
    return SoundClip(
        name=name, set_name=_text(raw['set'], f'{path}.set'),
        path=_text(raw['path'], f'{path}.path'),
        duration_ms=duration,
        rms_dbfs=_number(raw['rms_dbfs'], f'{path}.rms_dbfs', high=20.0),
        peak_dbfs=_number(raw['peak_dbfs'], f'{path}.peak_dbfs', high=20.0),
        clip_fraction=_number(raw['clip_fraction'], f'{path}.clip_fraction', 0.0, 1.0),
        set_gain_db=_number(raw['set_gain_db'], f'{path}.set_gain_db'),
        gain_db=_number(raw['gain_db'], f'{path}.gain_db', MIN_GAIN_DB, MAX_GAIN_DB),
        tags=tuple(_tag(t, f'{path}.tags[{i}]') for i, t in enumerate(tags)),
        suggested_tag=_tag(raw['suggested_tag'], f'{path}.suggested_tag'),
        confidence=_number(raw['confidence'], f'{path}.confidence', 0.0, 1.0),
        excluded=excluded, label=label)


def parse_catalogue(data, sounds_dir: Optional[str] = None):
    """
    Validate ``data`` (the parsed YAML) and return ``(Catalogue, warnings)``.

    With ``sounds_dir``, clips whose file is missing there are dropped and each drop is a
    warning string; without it every file is assumed present.
    """
    data = _mapping(data, '', _TOP_KEYS)
    for key in _TOP_KEYS:
        if key not in data:
            _fail(key, 'is required')
    sets = {}
    for name, raw in _mapping(data['sets'], 'sets', data['sets'] or ()).items():
        raw = _mapping(raw, f'sets.{name}', _SET_KEYS)
        if set(raw) != set(_SET_KEYS):
            _fail(f'sets.{name}', f'needs exactly {", ".join(_SET_KEYS)}')
        count = raw['clips']
        if isinstance(count, bool) or not isinstance(count, int) or count < 0:
            _fail(f'sets.{name}.clips', f'must be an integer >= 0, got {count!r}')
        median = _number(raw['median_rms_dbfs'], f'sets.{name}.median_rms_dbfs')
        sets[name] = SetInfo(count, median,
                             _number(raw['set_gain_db'], f'sets.{name}.set_gain_db'))
    if not isinstance(data['clips'], list):
        _fail('clips', 'must be a list')
    clips, seen, warnings = [], set(), []
    for i, raw in enumerate(data['clips']):
        clip = _parse_clip(raw, f'clips[{i}]')
        if clip.name in seen:
            _fail(f'clips[{i}].name', f'duplicate name {clip.name!r}')
        seen.add(clip.name)
        if clip.set_name not in sets:
            _fail(f'clips[{i}].set', f'{clip.set_name!r} is not in sets')
        if sounds_dir is not None and not os.path.isfile(os.path.join(sounds_dir, clip.path)):
            warnings.append(f'{clip.name}: {clip.path} is missing under {sounds_dir}; dropped')
            continue
        clips.append(clip)
    house = _number(data['house_level_dbfs'], 'house_level_dbfs', -60.0, 0.0)
    generated = _text(str(data['generated']), 'generated')
    return Catalogue(house, generated, MappingProxyType(sets), tuple(clips)), warnings


def load_catalogue(path, sounds_dir: Optional[str] = None):
    """Read ``path`` and return ``(Catalogue, warnings)``; see :func:`parse_catalogue`."""
    with open(path) as f:
        data = yaml.safe_load(f)
    try:
        return parse_catalogue(data, sounds_dir)
    except CatalogueError as error:
        raise CatalogueError(f'{path}: {error}') from None
