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
The dome device's LED side as a pure model (no ROS): plan 2.2-2.4 of the phase 1 plan.

Every public method takes ``now`` (seconds, any monotonic origin) and first advances the
model to it, so a test drives time by passing numbers. A method returns plain values or
dicts; ``dome_mock`` maps them to messages. Not thread-safe: the node serialises
every call with a lock.

Who wrote matters. ``source`` is ``ros``, ``console``, ``ui`` or ``text_topic``; a write
by any of the last three holds the displays it touched for ``hold_ms``, and a ROS request
with a priority of 1..199 that touches a held display is refused (``held by console`` ...).
Priority 0 and 200 or more are never refused, and ROS writes never hold.
"""

import random

from r2_mocks.holds import Hold
from r2_mocks.holds import hold_refusal
from r2_mocks.holds import Manual
from r2_mocks.holds import owner_too_long
from r2_mocks.holds import REFUSE_NOT_FITTED
from r2_mocks.holds import REFUSE_OWNER_TOO_LONG
import yaml

TEXT_MAX_CHARS = 63
CYCLE_PERIOD_S = 10.0
CYCLE_CHILDREN = ('rainbow', 'roam', 'fire')
DEFAULT_BRIGHTNESS = 40
DEFAULT_DRIFT = 'calm'
DEFAULT_SPEED = 5
WIPE_STEP_MS = 500.0            # per column at speed 5 is 100 ms; scaled below by 5 / speed
TEXT_COLUMN_MS = 600.0          # 120 ms a column at speed 5, scaled by 5 / speed
TEXT_GLYPH_COLUMNS = 6          # a character's width in columns, spacing included
TEXT_GLYPH_ROWS = 7

MODE_RANDOM = 0
MODE_TEXT = 1
MODE_SOLID = 2
MODE_MANUAL = 3
MODE_OFF = 4
MODE_EFFECT = 5

APPLY_MODE = 1
APPLY_BRIGHTNESS = 2
APPLY_HUE = 4
APPLY_DRIFT = 8
APPLY_SAVE = 16

MOTION_NAMES = ('bounce', 'loop', 'left', 'right', 'up')
ROS_MODES = (MODE_RANDOM, MODE_SOLID, MODE_OFF)
ALL = 'all'
ALL_ORDER = ('rpsi', 'rld', 'fld', 'fpsi')          # the surfaces `all` runs on, in order

REFUSE_UNKNOWN_SURFACE = 'unknown surface'
REFUSE_UNKNOWN_DISPLAY = 'unknown display'
REFUSE_UNKNOWN_EFFECT = 'unknown effect'
REFUSE_NO_CLIP = 'no clip for this surface'
REFUSE_RANGE = 'out of range'
REFUSE_TEXT_LONG = 'text too long'
REFUSE_NO_TEXT = 'no text'
REFUSE_LINES = 'more lines than the surface has displays'
REFUSE_NO_TEXT_MODE = 'display has no text mode'
REFUSE_NO_DRIFT = 'display has no drift'
REFUSE_DRIFT = 'unknown drift preset'
REFUSE_MODE = 'unknown mode'
REFUSE_MOTION = 'unknown text motion'


def load_catalogue(path):
    """Return the catalogue dict (``dome_mock.yaml``) at ``path``."""
    with open(path) as handle:
        return yaml.safe_load(handle)


class _Display:
    def __init__(self, spec):
        self.spec = spec
        self.name = spec['name']
        self.mode = MODE_RANDOM
        self.rgb = 0
        self.text = ''
        self.brightness = DEFAULT_BRIGHTNESS
        self.hue = 0
        self.drift = DEFAULT_DRIFT if spec['has_drift'] else ''
        self.changed_by = ''
        self.changed_by_owner = ''
        self.hold = None            # a holds.Hold, set by the model


class _Run:
    def __init__(self, seq, surface, effect, now, rgb, speed, duration_ms, source, owner):
        self.seq = seq
        self.surface = surface            # the surface spec
        self.effect = effect              # effect or clip name, or "text"
        self.start = now
        self.rgb = rgb
        self.speed = speed or DEFAULT_SPEED
        self.duration_ms = duration_ms
        self.source = source
        self.owner = owner
        self.saved = {}                   # display name -> (mode, rgb, text)
        self.self_end_ms = None           # when the effect ends by itself, from the start
        self.lines = []
        self.cycle = None

    def end_at(self):
        """Return (time, reason) of the run's own end, or (None, '')."""
        ends = []
        if self.self_end_ms is not None:
            ends.append((self.start + self.self_end_ms / 1000.0, 'ended'))
        if self.duration_ms:
            ends.append((self.start + self.duration_ms / 1000.0, 'duration'))
        if not ends:
            return None, ''
        # The duration wins only when it comes strictly first.
        return min(ends, key=lambda e: (e[0], e[1] != 'ended'))


class _Record:
    """What a surface remembers about its previous run."""

    def __init__(self):
        self.last_end = 'none'
        self.last_effect = ''
        self.last_source = ''
        self.last_owner = ''


class LedModel:
    """The LED displays and surfaces of the dome device."""

    def __init__(self, catalogue, hold_ms=5000, rostext=True, seed=1, manual=None):
        self.catalogue = catalogue
        self.hold_s = hold_ms / 1000.0
        self.rostext = rostext
        self.strip_enabled = True
        self.manual = manual or Manual()
        self.level = 0
        self.level_set = False
        self.displays = {d['name']: _Display(d) for d in catalogue['displays']}
        for display in self.displays.values():
            display.hold = Hold(self.manual)
        self.surfaces = {s['name']: s for s in catalogue['surfaces']}
        self.records = {name: _Record() for name in self.surfaces}
        self.effects = {e['name'] for e in catalogue['effects']}
        self.clips = {}
        for clip in catalogue['clips']:
            self.clips[(clip['name'], clip['target'])] = clip
        self.clip_names = {name for name, _ in self.clips}
        self.runs = []
        self.slots = ['', '']
        self._seq = 0
        self._seed = seed

    # --- time ---------------------------------------------------------------------------

    def advance(self, now):
        """End every run whose duration or own end has passed."""
        while True:
            due = None
            for run in self.runs:
                at, reason = run.end_at()
                if at is not None and at <= now and (due is None or at < due[0]):
                    due = (at, reason, run)
            if due is None:
                return
            self._end(due[2], due[1])

    # --- holds --------------------------------------------------------------------------

    def _hold(self, names, source, now):
        for name in names:
            self.displays[name].hold.touch(source, now, self.hold_s)

    def _refused_by_hold(self, names, now, source, priority):
        for name in names:
            reason = hold_refusal(self.displays[name].hold.holder(now), source, priority)
            if reason:
                return reason
        return ''

    def _targets(self, surface):
        """Return the surface specs a name stands for (``all`` is four of them)."""
        if surface == ALL:
            return [self.surfaces[n] for n in ALL_ORDER if n in self.surfaces]
        return [self.surfaces[surface]]

    def _touched(self, surface):
        names = []
        for spec in self._targets(surface):
            names += [n for n in spec['displays'] if n not in names]
        return names

    def _gate(self, now, surface, owner, priority, source, what='surface'):
        """Return the refusal that precedes everything else, or ''."""
        if owner_too_long(owner):
            return REFUSE_OWNER_TOO_LONG
        if not self.strip_enabled:
            return REFUSE_NOT_FITTED
        known = self.surfaces if what == 'surface' else self.displays
        if surface != ALL and surface not in known:
            return REFUSE_UNKNOWN_SURFACE if what == 'surface' else REFUSE_UNKNOWN_DISPLAY
        names = self._touched(surface) if what == 'surface' else self._display_names(surface)
        return self._refused_by_hold(names, now, source, priority)

    def _display_names(self, display):
        return list(self.displays) if display == ALL else [display]

    # --- runs ---------------------------------------------------------------------------

    def _resolve(self, spec, effect):
        """Return ('code'|'clip', clip or None) for an effect on a surface, or a refusal."""
        if effect in self.effects:
            return 'code', None
        target = spec['clip_target']
        if target and (effect, target) in self.clips:
            return 'clip', self.clips[(effect, target)]
        if effect in self.clip_names:
            return REFUSE_NO_CLIP, None
        return REFUSE_UNKNOWN_EFFECT, None

    def _end(self, run, reason, released=()):
        """Take ``run`` off its displays: restore them (but not the ``released`` ones)."""
        for name, (mode, rgb, text) in run.saved.items():
            if name not in released:
                display = self.displays[name]
                display.mode, display.rgb, display.text = mode, rgb, text
        record = self.records[run.surface['name']]
        record.last_end = reason
        record.last_effect = run.effect
        record.last_source = run.source
        record.last_owner = run.owner
        self.runs.remove(run)

    def _begin(self, spec, effect, now, rgb, speed, duration_ms, source, owner):
        names = spec['displays']
        for run in list(self.runs):
            if set(run.surface['displays']) & set(names):
                self._end(run, 'replaced')
        self._seq += 1
        run = _Run(self._seq, spec, effect, now, rgb, speed, duration_ms, source, owner)
        for name in names:
            display = self.displays[name]
            run.saved[name] = (display.mode, display.rgb, display.text)
            display.mode = MODE_EFFECT
        self.runs.append(run)
        return run

    def run(self, now, surface, effect, rgb=0, speed=0, duration_ms=0, owner='', priority=0,
            source='ros'):
        """
        Start ``effect`` (a code effect or a clip) on ``surface`` or ``all``.

        Return ``(accepted, reason)``. A run replaces every run sharing a display with it.
        ``all`` is accepted only if every surface accepts; those that did keep their runs.
        """
        self.advance(now)
        reason = self._gate(now, surface, owner, priority, source)
        if reason:
            return False, reason
        failures = []
        for spec in self._targets(surface):
            kind, clip = self._resolve(spec, effect)
            if kind not in ('code', 'clip'):
                failures.append(kind)
            elif speed > 9 or rgb > 0xFFFFFF:
                failures.append(REFUSE_RANGE)
            else:
                run = self._begin(spec, effect, now, rgb, speed, duration_ms, source, owner)
                run.self_end_ms = self._self_end_ms(run, kind, clip)
        self._hold(self._touched(surface), source, now)
        return (False, failures[0]) if failures else (True, '')

    def _self_end_ms(self, run, kind, clip):
        if kind == 'clip':
            return None if clip['loops'] else clip['length_ms']
        if run.effect == 'wipe':
            return run.surface['width'] * WIPE_STEP_MS / run.speed
        return None

    def say(self, now, surface, lines, linked=False, motion=0, on_black=False, rgb=0, speed=0,
            duration_ms=0, owner='', priority=0, source='ros'):
        """Start a text run on ``surface`` or ``all``; return ``(accepted, reason)``."""
        self.advance(now)
        reason = self._gate(now, surface, owner, priority, source)
        if reason:
            return False, reason
        failures = []
        for spec in self._targets(surface):
            problem = self._text_problem(spec, lines, motion, speed, rgb)
            if problem:
                failures.append(problem)
                continue
            run = self._begin(spec, 'text', now, rgb, speed, duration_ms, source, owner)
            run.lines = list(lines)
            run.self_end_ms = self._text_end_ms(run, spec, motion)
        self._hold(self._touched(surface), source, now)
        return (False, failures[0]) if failures else (True, '')

    def _text_problem(self, spec, lines, motion, speed, rgb):
        if not any(lines):
            return REFUSE_NO_TEXT
        if len(lines) > len(spec['displays']):
            return REFUSE_LINES
        if any(len(line) > TEXT_MAX_CHARS for line in lines):
            return REFUSE_TEXT_LONG
        if not all(self.displays[n].spec['has_text'] for n in spec['displays']):
            return REFUSE_NO_TEXT_MODE
        if not 0 <= motion < len(MOTION_NAMES):
            return REFUSE_MOTION
        if speed > 9 or rgb > 0xFFFFFF:
            return REFUSE_RANGE
        return ''

    @staticmethod
    def _text_end_ms(run, spec, motion):
        """Return when left/right/up text has scrolled off; None for bounce and loop."""
        column_ms = TEXT_COLUMN_MS / run.speed
        longest = max(len(line) for line in run.lines)
        if MOTION_NAMES[motion] in ('left', 'right'):
            return (spec['width'] + longest * TEXT_GLYPH_COLUMNS) * column_ms
        if MOTION_NAMES[motion] == 'up':
            return (spec['height'] + len(run.lines) * TEXT_GLYPH_ROWS) * column_ms
        return None

    def stop(self, now, surface, priority=0, source='ros'):
        """
        End the run whose surface is exactly ``surface`` (``all``: every run).

        Return ``(accepted, reason, stopped)``; accepted even when nothing was running.
        """
        self.advance(now)
        reason = self._gate(now, surface, '', priority, source)
        if reason:
            return False, reason, []
        stopped = []
        for run in list(self.runs):
            if surface == ALL or run.surface['name'] == surface:
                self._end(run, 'stopped')
                stopped.append(run.surface['name'])
        self._hold(self._touched(surface), source, now)
        return True, '', stopped

    # --- configure ----------------------------------------------------------------------

    def configure(self, now, display, apply, mode=0, brightness=0, hue=0, drift='', rgb=0,
                  owner='', priority=0, source='ros', text=''):
        """
        Change display settings; every field asked for is checked before any is applied.

        Return ``(accepted, reason, state)`` where ``state`` is the display's state dict as
        applied (name empty for ``all``). Nothing is persisted: APPLY_SAVE is accepted and
        ignored. A console ``source`` may also set the text and manual modes.
        """
        self.advance(now)
        reason = self._gate(now, display, owner, priority, source, what='display')
        if reason:
            return False, reason, None
        names = self._display_names(display)
        targets = list(names)
        if apply & APPLY_MODE:
            reason, targets = self._check_mode(display, names, mode, source, text)
        if not reason and apply & APPLY_BRIGHTNESS and not 0 <= brightness <= 100:
            reason = REFUSE_RANGE
        drift_names = names
        if not reason and apply & APPLY_DRIFT:
            if drift not in self.catalogue['drift_presets']:
                reason = REFUSE_DRIFT
            else:
                drift_names = [n for n in names if self.displays[n].spec['has_drift']]
                if display != ALL and not drift_names:
                    reason = REFUSE_NO_DRIFT
        if reason:
            return False, reason, None
        for name in names:
            self._apply_one(now, self.displays[name], apply, mode, brightness, hue, drift, rgb,
                            owner, source, text, name in targets, name in drift_names)
        self._hold(names, source, now)
        first = self.display_states(now)[list(self.displays).index(names[0])]
        if display == ALL:
            first = dict(first, name='')
        return True, '', first

    def _check_mode(self, display, names, mode, source, text):
        """Return (refusal, the displays the mode applies to)."""
        allowed = ROS_MODES if source == 'ros' else (
            MODE_RANDOM, MODE_TEXT, MODE_SOLID, MODE_MANUAL, MODE_OFF)
        if mode not in allowed:
            return REFUSE_MODE, []
        if mode != MODE_TEXT:
            return '', names
        texty = [n for n in names if self.displays[n].spec['has_text']]
        if display != ALL and not texty:
            return REFUSE_NO_TEXT_MODE, []
        return '', texty

    def _apply_one(self, now, disp, apply, mode, brightness, hue, drift, rgb, owner, source,
                   text, mode_applies, drift_applies):
        if apply & APPLY_MODE and mode_applies:
            for run in list(self.runs):
                if disp.name in run.surface['displays']:
                    self._end(run, 'released', released=(disp.name,))
            disp.mode = mode
            if mode == MODE_SOLID and rgb:
                disp.rgb = rgb
            if mode == MODE_TEXT and text:
                disp.text = text
        if apply & APPLY_BRIGHTNESS:
            disp.brightness = brightness
        if apply & APPLY_HUE:
            disp.hue = hue
        if apply & APPLY_DRIFT and drift_applies:
            disp.drift = drift
        disp.changed_by = source
        disp.changed_by_owner = owner if source == 'ros' else ''

    # --- the other writers --------------------------------------------------------------

    def set_enabled(self, enabled):
        """Switch the strip on or off (``leds enable``); off refuses requests as not fitted."""
        self.strip_enabled = bool(enabled)

    def set_level(self, value):
        """Store the audio level (``/dome/audio/level`` or ``leds level``); it has no effect."""
        self.level = max(0, min(255, int(value)))
        self.level_set = True

    def text_slot(self, now, index, text):
        """
        Apply a message on ``/dome/text/line0|1`` as the device does.

        A new message starts ``say fld`` with both slots (bounce, linked, over the drift, no
        duration); both slots empty stops a ``text`` run on ``fld``. Either way it is a
        ``text_topic`` write and holds ``fld``. Return ``(accepted, reason)``.
        """
        self.advance(now)
        if not self.rostext:
            return True, ''
        slots = list(self.slots)
        slots[index] = text
        if any(slots):
            accepted, reason = self.say(now, 'fld', slots, linked=True, source='text_topic')
            if accepted:                     # a refused message must not poison the other line
                self.slots = slots
            return accepted, reason
        self.slots = slots
        for run in list(self.runs):
            if run.surface['name'] == 'fld' and run.effect == 'text':
                self._end(run, 'stopped')
        self._hold(self._touched('fld'), 'text_topic', now)
        return True, ''

    # --- state --------------------------------------------------------------------------

    def display_states(self, now):
        """Return one dict per display, in catalogue order, with LedDisplayState's fields."""
        self.advance(now)
        return [{
            'name': d.name, 'mode': d.mode, 'brightness': d.brightness, 'hue': d.hue,
            'drift': d.drift, 'text': d.text, 'changed_by': d.changed_by,
            'changed_by_owner': d.changed_by_owner, 'held_by': d.hold.holder(now),
            'hold_remaining_ms': d.hold.remaining_ms(now),
            'rgb': d.rgb,
        } for d in self.displays.values()]

    def _effect_name(self, run, now):
        if run.effect != 'cycle':
            return run.effect
        if run.cycle is None:             # [rng, slot, child], advanced as time passes
            rng = random.Random(self._seed * 1000003 + run.seq)
            run.cycle = [rng, 0, rng.choice(CYCLE_CHILDREN)]
        rng, slot, child = run.cycle
        wanted = int((now - run.start) // CYCLE_PERIOD_S)
        while slot < wanted:
            child = rng.choice([c for c in CYCLE_CHILDREN if c != child])
            slot += 1
        run.cycle[1:] = [slot, child]
        return 'cycle:' + child

    def surface_states(self, now):
        """Return one dict per surface, in catalogue order, with LedSurfaceState's fields."""
        self.advance(now)
        states = []
        for name in self.surfaces:
            record = self.records[name]
            state = {
                'name': name, 'running': False, 'effect': '', 'elapsed_ms': 0,
                'duration_ms': 0, 'last_effect': record.last_effect,
                'last_end': record.last_end, 'source': '', 'owner': '',
                'last_source': record.last_source, 'last_owner': record.last_owner,
                'rgb': 0, 'speed': 0,
            }
            for run in self.runs:
                if run.surface['name'] == name:
                    state.update(
                        running=True, effect=self._effect_name(run, now),
                        elapsed_ms=int((now - run.start) * 1000), duration_ms=run.duration_ms,
                        source=run.source, owner=run.owner, rgb=run.rgb, speed=run.speed)
            states.append(state)
        return states

    def state(self, now):
        """Return the whole ``LedStateArray`` as a dict."""
        return {
            'level': self.level, 'level_set': self.level_set,
            'displays': self.display_states(now), 'surfaces': self.surface_states(now),
            'strip_enabled': self.strip_enabled, 'manual': self.manual.on,
        }
