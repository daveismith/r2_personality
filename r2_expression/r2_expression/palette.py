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
Mood palettes: label and strength to a style per channel, and the flourish scheduler.

Pure helpers used by :mod:`r2_expression.core`; no ROS imports.

* :func:`resolve_styles`: the mood's style per channel, a channel the mood
  leaves out taking the neutral style; an unknown label is neutral throughout.
  ``mood_strength`` (clamped to [0, 1]) picks the brightness linearly between
  the style's ``min`` and ``max``, rounded to an integer.
* :func:`validate_picks`: drops the picks the device cannot play, judged
  against its catalogue (``ListLeds`` for effects and clips, ``ListMagicPanel``
  for sequences). A pick of the other device's kind is kept as it is: the
  other arbiter judges it. Each dropped pick produces one warning string.
* :class:`FlourishScheduler`: the next flourish is due a uniform draw from the
  mood's ``every_s`` after the previous one (or after the label changed); when
  it is due, one pick is drawn by weight from the mood's whole set. Each
  arbiter draws from the whole set and acts only on its own channels, so the
  two arbiters together keep the mood's rate and its weights across channels.
  The RNG is injected (anything with ``random()`` in [0, 1)), so tests are
  deterministic.
"""

from dataclasses import dataclass
import json
from types import MappingProxyType
from typing import Any, FrozenSet, Mapping, Optional

from r2_expression.config import LED_PICK_CHANNELS, STYLE_CHANNELS


@dataclass(frozen=True)
class ResolvedStyle:
    """One channel's look for one label and strength."""

    drift: Optional[str]
    brightness: Optional[int]


@dataclass(frozen=True)
class LedCatalogue:
    """What ``ListLeds`` says the device can play."""

    effects: FrozenSet[str]
    clips: Mapping[str, FrozenSet[str]]     # clip name -> its targets (fld, psi, rld)

    def has(self, kind, name, channel):
        """Return True when ``kind`` ``name`` plays on LED ``channel``."""
        if kind == 'effect':
            return name in self.effects
        # A clip target is named as its channel (2.2): fld, rld, psi; tfld/bfld have none.
        return channel in self.clips.get(name, ())


@dataclass(frozen=True)
class MagicCatalogue:
    """What ``ListMagicPanel`` says the panel can play: id -> name (empty without names)."""

    sequences: Mapping[int, str]

    def resolve(self, ref):
        """Return the id for an id or a name, or None when the panel has no such sequence."""
        if isinstance(ref, str):
            for seq_id, name in self.sequences.items():
                if name and name == ref:
                    return seq_id
            return None
        return ref if ref in self.sequences else None


def field_of(obj, name, default=None):
    """Return ``obj[name]`` for a mapping, else ``obj.name``; ``default`` when absent."""
    if isinstance(obj, Mapping):
        return obj.get(name, default)
    return getattr(obj, name, default)


def led_catalogue(response):
    """Build a :class:`LedCatalogue` from a ``ListLeds`` response (a message or a dict)."""
    clips = {}
    for clip in field_of(response, 'clips', ()) or ():
        clips.setdefault(field_of(clip, 'name'), set()).add(field_of(clip, 'target'))
    return LedCatalogue(effects=frozenset(field_of(response, 'effects', ()) or ()),
                        clips=MappingProxyType({k: frozenset(v) for k, v in clips.items()}))


def magic_catalogue(response):
    """Build a :class:`MagicCatalogue` from a ``ListMagicPanel`` response."""
    return MagicCatalogue(sequences=MappingProxyType({
        int(field_of(s, 'id')): field_of(s, 'name', '') or ''
        for s in field_of(response, 'sequences', ()) or ()}))


def scale_brightness(brightness, strength):
    """Brightness for ``strength`` in [0, 1] between ``(min, max)``; None stays None."""
    if brightness is None:
        return None
    try:
        strength = float(strength)
    except (TypeError, ValueError):
        strength = 0.0
    if not strength >= 0.0:          # NaN too
        strength = 0.0
    strength = min(strength, 1.0)
    lo, hi = brightness
    return int(round(lo + (hi - lo) * strength))


def resolve_styles(config, label, strength, stale=False):
    """
    Return ``{channel: ResolvedStyle}`` for every style channel that has a style.

    ``stale`` (or an unknown label) gives the neutral styles; a channel the
    mood leaves out takes neutral's; a channel neither names is absent (it is
    not touched).
    """
    mood = config.neutral if stale else config.moods.get(label, config.neutral)
    out = {}
    for channel in STYLE_CHANNELS:
        style = mood.styles.get(channel) or config.neutral.styles.get(channel)
        if style is None:
            continue
        out[channel] = ResolvedStyle(drift=style.drift,
                                     brightness=scale_brightness(style.brightness, strength))
    return out


def is_led_pick(pick):
    """Return True for a pick the LED arbiter plays."""
    return pick.channel in LED_PICK_CHANNELS


def validate_picks(config, led=None, magic=None):
    """
    Return ``({mood: picks}, [(key, warning)])`` with the unplayable picks removed.

    ``led`` and ``magic`` are :class:`LedCatalogue`/:class:`MagicCatalogue`
    or None (that kind is then not judged). ``key`` identifies the dropped
    pick, so a caller can warn once per pick across reloads.
    """
    out, warnings = {}, []
    for name, mood in config.moods.items():
        if mood.flourish is None:
            out[name] = ()
            continue
        kept = []
        for index, pick in enumerate(mood.flourish.picks):
            catalogue = led if is_led_pick(pick) else magic
            if catalogue is None:
                kept.append(pick)
                continue
            if is_led_pick(pick):
                ok = catalogue.has(pick.kind, pick.name, pick.channel)
            else:
                ok = catalogue.resolve(pick.name) is not None
            if ok:
                kept.append(pick)
            else:
                key = (name, index, pick.kind, pick.name, pick.channel)
                text = ('moods.%s.flourish.picks[%d]: the device has no %s %r for %s; pick '
                        'dropped' % (name, index, pick.kind, pick.name, pick.channel))
                warnings.append((key, text))
        out[name] = tuple(kept)
    return MappingProxyType(out), warnings


def pick_intent_fields(pick, intent_id):
    """Return the ``ExpressionIntent`` fields of an idle intent playing ``pick``."""
    if pick.kind == 'sequence':
        key = 'name' if isinstance(pick.name, str) else 'id'
        params = {key: pick.name}
        for opt in ('repeat', 'end'):
            if getattr(pick, opt) is not None:
                params[opt] = getattr(pick, opt)
    else:
        params = {'name': pick.name}
        for opt in ('rgb', 'speed'):
            if getattr(pick, opt) is not None:
                params[opt] = getattr(pick, opt)
    return {'channel': pick.channel, 'source': 'idle', 'priority': 10,
            'action': pick.kind, 'params': json.dumps(params),
            'duration_ms': pick.duration_ms or 0, 'interruptible': True,
            'intent_id': intent_id}


class FlourishScheduler:
    """When the next flourish is due, and which pick it is (see the module docstring)."""

    def __init__(self, rng):
        self._rng = rng
        self._every = None
        self.next_due = None          # None: no flourishes (none in the mood, or stopped)

    def restart(self, flourish, now):
        """Follow ``flourish`` (a Flourish or None) from ``now``."""
        self._every = None if flourish is None else flourish.every_s
        self.next_due = None if self._every is None else now + self._draw()

    def stop(self):
        """No flourishes until :meth:`restart`."""
        self.next_due = None

    def poll(self, picks, now):
        """Return a pick when one is due (rescheduling the next), else None."""
        if self.next_due is None or now < self.next_due:
            return None
        self.next_due = now + self._draw()
        return choose(picks, self._rng)

    def _draw(self):
        lo, hi = self._every
        return lo + (hi - lo) * self._rng.random()


def choose(picks, rng) -> Any:
    """Draw one pick by weight; None when there are none."""
    total = sum(p.weight for p in picks)
    if not picks or total <= 0.0:
        return None
    point = rng.random() * total
    for pick in picks:
        point -= pick.weight
        if point < 0.0:
            return pick
    return picks[-1]


__all__ = ['FlourishScheduler', 'LedCatalogue', 'MagicCatalogue', 'ResolvedStyle', 'choose',
           'field_of', 'is_led_pick', 'led_catalogue', 'magic_catalogue',
           'pick_intent_fields', 'resolve_styles', 'scale_brightness', 'validate_picks']
