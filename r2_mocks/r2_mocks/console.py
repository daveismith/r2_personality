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
The device's console vocabulary for LEDs and the Magic Panel, applied to the models.

``Console.execute(now, line)`` takes one line as a person would type it at the dome's USB or
telnet console (or tap on its screen, with a leading ``ui``) and does what the firmware
does: the same functions as a ROS request, attributed ``console`` or ``ui`` and never
refused for a hold. It returns ``(ok, message)``.
"""

import shlex

from r2_mocks import led_model as led
from r2_mocks import magic_model as magic

LED_MODES = {'random': led.MODE_RANDOM, 'solid': led.MODE_SOLID, 'off': led.MODE_OFF,
             'text': led.MODE_TEXT, 'manual': led.MODE_MANUAL}
RUN_FLAGS = {'-c': 'rgb', '-s': 'speed', '-t': 'seconds'}
SAY_FLAGS = {'-c': 'rgb', '-s': 'speed', '-t': 'seconds', '-m': 'motion', '-k': None,
             '-b': None}
START_FLAGS = {'-r': 'repeat', '-e': 'end'}
LED_VERBS = ('run', 'say', 'stop', 'mode', 'fill', 'text', 'brightness', 'drift', 'enable',
             'owner', 'rostext', 'level')


class ConsoleError(Exception):
    """A line the console cannot make sense of."""


def _parse(tokens, flags, positionals, greedy=False, collect=False):
    """
    Split tokens into options and words.

    ``greedy`` joins everything after the positionals into one last word; ``collect`` takes
    every token that is not a flag as a word (flags may then come anywhere).
    """
    options, words, index = {}, [], 0
    while index < len(tokens):
        token = tokens[index]
        if token in flags and (not greedy or len(words) >= positionals):
            key = flags[token]
            if key is None:
                options[token] = True
            else:
                index += 1
                if index >= len(tokens):
                    raise ConsoleError('%s needs a value' % token)
                options[key] = tokens[index]
        elif collect or len(words) < positionals:
            words.append(token)
        elif greedy:
            words.append(' '.join(tokens[index:]))
            break
        else:
            raise ConsoleError('unexpected %r' % token)
        index += 1
    if len(words) < positionals + (1 if greedy else 0):
        raise ConsoleError('missing argument')
    return options, words


def _number(text, low, high, what):
    try:
        value = int(text)
    except ValueError:
        raise ConsoleError('%s must be a number' % what) from None
    if not low <= value <= high:
        raise ConsoleError('%s: %s' % (what, led.REFUSE_RANGE))
    return value


def _colour(text):
    try:
        value = int(text.lstrip('#'), 16)
    except ValueError:
        raise ConsoleError('colour must be rrggbb') from None
    if not 0 <= value <= 0xFFFFFF:
        raise ConsoleError('colour must be rrggbb')
    return value


def _run_options(opts):
    """Return the colour, speed and duration keywords the -c, -s and -t flags give."""
    try:
        seconds = float(opts.get('seconds', 0))
    except ValueError:
        raise ConsoleError('seconds must be a number') from None
    if not 0.0 <= seconds < 4294967.0:
        raise ConsoleError('seconds: %s' % led.REFUSE_RANGE)
    return {'rgb': _colour(opts['rgb']) if 'rgb' in opts else 0,
            'speed': _number(opts.get('speed', 0), 0, 255, 'speed'),
            'duration_ms': int(seconds * 1000)}


def _on_off(text):
    if text not in ('on', 'off'):
        raise ConsoleError('expected on or off')
    return text == 'on'


class Console:
    """Executes console lines against a LedModel and a MagicModel."""

    def __init__(self, leds, panel):
        self.leds = leds
        self.panel = panel

    def execute(self, now, line):
        """Run one console line; return ``(ok, message)``."""
        try:
            tokens = shlex.split(line)
            source = 'console'
            if tokens and tokens[0] == 'ui':
                source, tokens = 'ui', tokens[1:]
                if tokens and tokens[0] in LED_VERBS:
                    tokens.insert(0, 'leds')
            if not tokens:
                raise ConsoleError('empty line')
            if tokens[0] == 'leds' and len(tokens) > 1:
                return self._leds(now, source, tokens[1], tokens[2:])
            if tokens[0] == 'magic_panel' and len(tokens) > 1:
                return self._magic(now, source, tokens[1], tokens[2:])
            if tokens[0] == 'rotary' and len(tokens) == 2:
                code = _number(tokens[1], 0, 9, 'rotary')
                reason = self.panel.rotary(now, code)
                return (False, reason) if reason else (True, 'ok')
            raise ConsoleError('unknown command %r' % line)
        except (ConsoleError, ValueError) as exc:
            return False, str(exc)

    # --- leds ---------------------------------------------------------------------------

    def _leds(self, now, source, verb, args):
        handler = getattr(self, '_leds_' + verb, None) if verb in LED_VERBS else None
        if handler is None:
            raise ConsoleError('unknown leds command %r' % verb)
        return handler(now, source, args)

    @staticmethod
    def _result(accepted, reason):
        return accepted, 'ok' if accepted else reason

    def _leds_run(self, now, source, args):
        opts, (surface, effect) = _parse(args, RUN_FLAGS, 2)
        return self._result(*self.leds.run(
            now, surface, effect, source=source, **_run_options(opts)))

    def _leds_say(self, now, source, args):
        opts, (surface, text) = _parse(args, SAY_FLAGS, 1, greedy=True)
        motion = 0
        if 'motion' in opts:
            if opts['motion'] not in led.MOTION_NAMES:
                raise ConsoleError(led.REFUSE_MOTION)
            motion = led.MOTION_NAMES.index(opts['motion'])
        return self._result(*self.leds.say(
            now, surface, text.split('|'), linked='-k' in opts, motion=motion,
            on_black='-b' in opts, source=source, **_run_options(opts)))

    def _leds_stop(self, now, source, args):
        _, (surface,) = _parse(args, {}, 1)
        accepted, reason, _ = self.leds.stop(now, surface, source=source)
        return self._result(accepted, reason)

    def _configure(self, now, source, display, **fields):
        accepted, reason, _ = self.leds.configure(now, display, source=source, **fields)
        return self._result(accepted, reason)

    def _leds_mode(self, now, source, args):
        _, (display, mode) = _parse(args, {}, 2)
        if mode not in LED_MODES:
            raise ConsoleError(led.REFUSE_MODE)
        return self._configure(now, source, display, apply=led.APPLY_MODE, mode=LED_MODES[mode])

    def _leds_fill(self, now, source, args):
        _, (display, colour) = _parse(args, {}, 2)
        return self._configure(now, source, display, apply=led.APPLY_MODE,
                               mode=led.MODE_SOLID, rgb=_colour(colour))

    def _leds_text(self, now, source, args):
        _, (display, text) = _parse(args, {}, 1, greedy=True)
        return self._configure(now, source, display, apply=led.APPLY_MODE,
                               mode=led.MODE_TEXT, text=text)

    def _leds_brightness(self, now, source, args):
        _, (display, value) = _parse(args, {}, 2)
        return self._configure(now, source, display, apply=led.APPLY_BRIGHTNESS,
                               brightness=_number(value, 0, 255, 'brightness'))

    def _leds_drift(self, now, source, args):
        _, (display, preset) = _parse(args, {}, 2)
        return self._configure(now, source, display, apply=led.APPLY_DRIFT, drift=preset)

    def _leds_enable(self, now, source, args):
        _, (value,) = _parse(args, {}, 1)
        self.leds.set_enabled(_on_off(value))
        return True, 'ok'

    def _leds_owner(self, now, source, args):
        _, (value,) = _parse(args, {}, 1)
        if value not in ('manual', 'auto'):
            raise ConsoleError('expected manual or auto')
        self.leds.manual.set_on(value == 'manual')       # one flag, shared with the panel
        return True, 'ok'

    def _leds_rostext(self, now, source, args):
        _, (value,) = _parse(args, {}, 1)
        self.leds.rostext = _on_off(value)
        return True, 'ok'

    def _leds_level(self, now, source, args):
        _, (value,) = _parse(args, {}, 1)
        self.leds.set_level(_number(value, 0, 255, 'level'))
        return True, 'ok'

    # --- magic_panel --------------------------------------------------------------------

    def _magic(self, now, source, verb, args):
        if verb == 'start':
            opts, words = _parse(args, START_FLAGS, 0, collect=True)
            sequence = self.panel.sequence_id(' '.join(words))
            end = {'blank': magic.END_BLANK, 'leave': 0}.get(opts.get('end', 'leave'))
            if end is None:
                raise ConsoleError('end is blank or leave')
            reply = self.panel.start(
                now, -1 if sequence is None else sequence,
                repeat=_number(opts.get('repeat', 1), 0, 255, 'repeat'),
                end=end, source=source)
        elif verb == 'stop':
            mode = {'blank': magic.STOP_BLANK, 'freeze': magic.STOP_FREEZE}.get(
                args[0] if args else 'blank')
            if mode is None:
                raise ConsoleError('stop is blank or freeze')
            reply = self.panel.stop(now, mode, source=source)
        elif verb == 'fit':
            _, (value,) = _parse(args, {}, 1)
            self.panel.set_fitted(_on_off(value))
            return True, 'ok'
        else:
            raise ConsoleError('unknown magic_panel command %r' % verb)
        return self._result(reply['accepted'], reply['reason'])
