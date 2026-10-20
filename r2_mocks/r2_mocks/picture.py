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
What the mock shows to a person: a text picture and an RGB image of the five displays.

No effect animation. A display's colour comes from its mode (off is black, solid is its
colour, random is a colour typical of its drift) or, while a run holds it, from the effect's
name (one distinct hue per name), scaled by the display's brightness. The grids follow the
firmware's ``tools/led_layout.py``: 7x5 PSIs with the four corners absent, 5x8 FLDs, a 4x24
RLD whose odd rows sit half a cell to the left (laid like bricks); cells are row-major.
"""

import colorsys
import functools
import zlib

import numpy as np
from r2_mocks.holds import HOLD_FOREVER
from r2_mocks.led_model import MODE_EFFECT
from r2_mocks.led_model import MODE_MANUAL
from r2_mocks.led_model import MODE_OFF
from r2_mocks.led_model import MODE_RANDOM
from r2_mocks.led_model import MODE_SOLID
from r2_mocks.led_model import MODE_TEXT

CELL_PX = 12
GAP_PX = 2
PITCH_PX = CELL_PX + GAP_PX
MARGIN_PX = 8
BACKGROUND = 16
DISPLAY_ORDER = ('tfld', 'bfld', 'rld', 'fpsi', 'rpsi')
MODE_NAMES = {MODE_RANDOM: 'random', MODE_TEXT: 'text', MODE_SOLID: 'solid',
              MODE_MANUAL: 'manual', MODE_OFF: 'off', MODE_EFFECT: 'effect'}
PSI_ABSENT = ((0, 0), (0, 4), (6, 0), (6, 4))
DRIFT_COLOURS = {'fld': (64, 32, 192), 'rld': (128, 192, 32)}
SOLID_DEFAULT = 0x808080
TEXT_COLOUR = (255, 255, 255)
MANUAL_COLOUR = (128, 128, 128)
# kind -> (seconds per colour, the two colours it alternates between)
PSI_PAIRS = {'front_psi': (10.0, ((255, 0, 0), (0, 0, 255))),
             'rear_psi': (5.0, ((0, 192, 0), (255, 0, 0)))}


@functools.lru_cache(maxsize=None)
def effect_colour(name):
    """Return the RGB colour that stands for effect ``name``: stable, distinct per name."""
    hue = (zlib.crc32(name.split(':')[0].encode()) % 360) / 360.0
    red, green, blue = colorsys.hsv_to_rgb(hue, 0.85, 1.0)
    return int(red * 255), int(green * 255), int(blue * 255)


def _unpack(rgb):
    return (rgb >> 16) & 255, (rgb >> 8) & 255, rgb & 255


def display_colour(kind, state, effect, effect_rgb, now):
    """Return the RGB tuple a display of ``kind`` shows, given its state dict and its run."""
    mode = state['mode']
    if effect:
        colour = _unpack(effect_rgb) if effect_rgb else effect_colour(effect)
    elif mode == MODE_OFF:
        colour = (0, 0, 0)
    elif mode == MODE_SOLID:
        colour = _unpack(state['rgb'] or SOLID_DEFAULT)
    elif mode == MODE_TEXT:
        colour = TEXT_COLOUR
    elif mode == MODE_MANUAL:
        colour = MANUAL_COLOUR
    elif kind in PSI_PAIRS:
        period, pair = PSI_PAIRS[kind]
        colour = pair[int(now // period) % 2]
    else:
        colour = DRIFT_COLOURS[kind]
    scale = state['brightness'] / 100.0
    return tuple(int(c * scale) for c in colour)


def grid_cells(spec):
    """Return the (row, col) of every cell with an LED in a display of ``spec``."""
    absent = PSI_ABSENT if spec['kind'] in PSI_PAIRS else ()
    return [(r, c) for r in range(spec['rows']) for c in range(spec['cols'])
            if (r, c) not in absent]


class Layout:
    """The picture's geometry for a catalogue, computed once; ``render`` only paints."""

    def __init__(self, catalogue):
        self.catalogue = catalogue
        specs = {d['name']: d for d in catalogue['displays']}
        self.order = [n for n in DISPLAY_ORDER if n in specs]
        self.kinds = [specs[n]['kind'] for n in self.order]
        self.surfaces = [(s['name'], s['displays']) for s in catalogue['surfaces']]
        self.width = 2 * MARGIN_PX + PITCH_PX // 2 + max(
            specs[n]['cols'] for n in self.order) * PITCH_PX
        self.height = 2 * MARGIN_PX - PITCH_PX + sum(
            (specs[n]['rows'] + 1) * PITCH_PX for n in self.order)
        self.cells = []             # per display: the (y, x) top-left pixel of each cell
        top = MARGIN_PX
        for name in self.order:
            spec = specs[name]
            odd_row_shift = -PITCH_PX // 2 if spec['kind'] == 'rld' else 0
            self.cells.append([
                (top + row * PITCH_PX,
                 MARGIN_PX + PITCH_PX // 2 + col * PITCH_PX + (row % 2) * odd_row_shift)
                for row, col in grid_cells(spec)])
            top += (spec['rows'] + 1) * PITCH_PX

    def colours(self, state, now):
        """Return the colour of each display, in picture order: the image's whole content."""
        running = {}
        by_name = {s['name']: s for s in state['surfaces']}
        for name, displays in self.surfaces:
            info = by_name[name]
            if info['running']:
                for display in displays:
                    running[display] = (info['effect'], info['rgb'])
        states = {d['name']: d for d in state['displays']}
        return tuple(
            display_colour(kind, states[name], *running.get(name, ('', 0)), now)
            for name, kind in zip(self.order, self.kinds))

    def render(self, colours):
        """Return the rgb8 bytes of the picture for ``colours`` (see :meth:`colours`)."""
        image = np.full((self.height, self.width, 3), BACKGROUND, dtype=np.uint8)
        for cells, colour in zip(self.cells, colours):
            for y, x in cells:
                image[y:y + CELL_PX, x:x + CELL_PX] = colour
        return image.tobytes()


def picture_text(state, panel):
    """Return the one-line-per-surface-and-display text picture of the model states."""
    lines = []
    for s in state['surfaces']:
        lines.append(
            'surface %s: %s effect=%s elapsed=%dms duration=%dms source=%s owner=%s '
            'last_end=%s last_effect=%s last_source=%s' % (
                s['name'], 'running' if s['running'] else 'idle', s['effect'] or '-',
                s['elapsed_ms'], s['duration_ms'], s['source'] or '-', s['owner'] or '-',
                s['last_end'], s['last_effect'] or '-', s['last_source'] or '-'))
    for d in state['displays']:
        lines.append(
            'display %s: mode=%s brightness=%d drift=%s changed_by=%s held_by=%s hold=%s' % (
                d['name'], MODE_NAMES[d['mode']], d['brightness'], d['drift'] or '-',
                d['changed_by'] or '-', d['held_by'] or '-',
                'forever' if d['hold_remaining_ms'] == HOLD_FOREVER
                else '%dms' % d['hold_remaining_ms']))
    lines.append('strip_enabled=%s manual=%s level=%d%s' % (
        state['strip_enabled'], state['manual'], state['level'],
        '' if state['level_set'] else ' (default)'))
    lines.append(
        'magic: link=%d state=%d sequence=%d started_by=%s owner=%s held_by=%s' % (
            panel['link'], panel['state'], panel['sequence_id'], panel['started_by'] or '-',
            panel['owner'] or '-', panel['held_by'] or '-'))
    return '\n'.join(lines)
