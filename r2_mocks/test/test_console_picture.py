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

"""The console vocabulary and the picture, without ROS."""

import copy
import os

import pytest
from r2_mocks import led_model as led
from r2_mocks import magic_model as mg
from r2_mocks import picture
from r2_mocks.console import Console
from r2_mocks.holds import Manual
from r2_mocks.led_model import LedModel
from r2_mocks.magic_model import MagicModel

CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      'config', 'dome_mock.yaml')
CATALOGUE = led.load_catalogue(CONFIG)


@pytest.fixture
def rig():
    manual = Manual()
    leds = LedModel(copy.deepcopy(CATALOGUE), manual=manual)
    panel = MagicModel(copy.deepcopy(CATALOGUE), manual=manual)
    return leds, panel, Console(leds, panel)


def surface(leds, name, now=1):
    return next(s for s in leds.surface_states(now) if s['name'] == name)


def display(leds, name, now=1):
    return next(d for d in leds.display_states(now) if d['name'] == name)


def test_leds_run_with_colour_speed_and_seconds(rig):
    leds, _, con = rig
    assert con.execute(0, 'leds run fld fire -c ff8000 -s 7 -t 5') == (True, 'ok')
    s = surface(leds, 'fld')
    assert (s['source'], s['effect'], s['duration_ms'], s['rgb'], s['speed']) == (
        'console', 'fire', 5000, 0xFF8000, 7)
    assert display(leds, 'tfld')['held_by'] == 'console'


def test_leds_say_with_motion_and_two_lines(rig):
    leds, _, con = rig
    assert con.execute(0, 'leds say fld -m left -k hello|world') == (True, 'ok')
    assert surface(leds, 'fld')['effect'] == 'text'
    assert surface(leds, 'fld', 3600)['last_end'] == 'ended'      # left ends by itself


def test_leds_say_text_with_spaces(rig):
    leds, _, con = rig
    assert con.execute(0, 'leds say rld -t 10 hello big world')[0]
    assert surface(leds, 'rld')['duration_ms'] == 10000


def test_leds_stop_mode_fill_text_brightness_drift(rig):
    leds, _, con = rig
    con.execute(0, 'leds run all fire')
    assert con.execute(1, 'leds stop all') == (True, 'ok')
    assert not any(s['running'] for s in leds.surface_states(1))
    assert con.execute(1, 'leds mode tfld off') == (True, 'ok')
    assert display(leds, 'tfld')['mode'] == led.MODE_OFF
    assert con.execute(1, 'leds fill rld 00ff00')[0]
    assert (display(leds, 'rld')['mode'], display(leds, 'rld')['rgb']) == (led.MODE_SOLID, 0xFF00)
    assert con.execute(1, 'leds text bfld hi there')[0]
    assert (display(leds, 'bfld')['mode'], display(leds, 'bfld')['text']) == (
        led.MODE_TEXT, 'hi there')
    assert con.execute(1, 'leds brightness all 25')[0]
    assert {d['brightness'] for d in leds.display_states(1)} == {25}
    assert con.execute(1, 'leds drift all lively')[0]
    assert display(leds, 'rld')['drift'] == 'lively' and display(leds, 'fpsi')['drift'] == ''
    assert con.execute(1, 'leds drift fpsi lively') == (False, 'display has no drift')
    assert con.execute(1, 'leds mode tfld manual')[0]


def test_console_refusals_use_the_device_strings(rig):
    _, _, con = rig
    assert con.execute(0, 'leds run nowhere fire') == (False, 'unknown surface')
    assert con.execute(0, 'leds run fld nosuch') == (False, 'unknown effect')
    assert con.execute(0, 'leds say fld -m sideways hi') == (False, 'unknown text motion')
    assert con.execute(0, 'leds mode tfld sleepy') == (False, 'unknown mode')


def test_a_console_write_is_never_refused_by_a_hold(rig):
    leds, _, con = rig
    con.execute(0, 'leds owner manual')
    assert con.execute(1, 'leds run fld fire')[0]


def test_ui_writes_are_attributed_ui(rig):
    leds, _, con = rig
    assert con.execute(0, 'ui leds run fld fire')[0]
    assert surface(leds, 'fld')['source'] == 'ui' and display(leds, 'tfld')['held_by'] == 'ui'
    assert con.execute(1, 'ui brightness tfld 10')[0]
    assert display(leds, 'tfld')['changed_by'] == 'ui'


def test_enable_owner_rostext_level(rig):
    leds, panel, con = rig
    con.execute(0, 'leds enable off')
    assert leds.strip_enabled is False
    con.execute(0, 'leds enable on')
    con.execute(0, 'leds owner manual')
    assert leds.manual.on and panel.manual.on
    con.execute(0, 'leds owner auto')
    assert not leds.manual.on and not panel.manual.on
    con.execute(0, 'leds rostext off')
    assert leds.rostext is False
    con.execute(0, 'leds level 77')
    assert (leds.level, leds.level_set) == (77, True)
    assert con.execute(0, 'leds level 300')[0] is False


def test_magic_panel_start_by_id_and_name_with_flags(rig):
    _, panel, con = rig
    assert con.execute(0, 'magic_panel start 44') == (True, 'ok')
    assert con.execute(0, 'magic_panel start Trace up line -r 3 -e blank') == (True, 'ok')
    panel.poll(1)
    s = panel.state(1)
    assert (s['sequence_id'], s['repeat'], s['end'], s['started_by']) == (9, 3, 1, 'console')
    assert con.execute(0, 'magic_panel start nosuch') == (False, 'unknown sequence')
    assert con.execute(0, 'magic_panel start 3 -e fade')[0] is False


def test_magic_panel_stop_and_fit(rig):
    _, panel, con = rig
    con.execute(0, 'magic_panel start 1 -r 0')
    assert con.execute(1, 'magic_panel stop freeze') == (True, 'ok')
    assert con.execute(1, 'magic_panel fit off') == (True, 'ok')
    assert con.execute(1, 'magic_panel start 3') == (False, 'not fitted')
    con.execute(1, 'magic_panel fit on')
    assert con.execute(1, 'magic_panel start 3')[0]


def test_rotary_is_a_gpio_takeover(rig):
    _, panel, con = rig
    assert con.execute(0, 'rotary 3') == (True, 'ok')
    panel.poll(1)
    s = panel.state(1)
    assert (s['source'], s['started_by'], s['takeovers'], s['rotary_code']) == (
        mg.SOURCE_GPIO, 'other', 1, 3)
    assert con.execute(0, 'rotary 10')[0] is False


@pytest.mark.parametrize('line', ['', 'dance', 'leds', 'leds frob x', 'magic_panel dance',
                                  "leds say 'unterminated", 'leds run fld'])
def test_unknown_or_malformed_lines_are_not_ok(rig, line):
    assert rig[2].execute(0, line)[0] is False


# --- picture ----------------------------------------------------------------------------

def pixel(image, x, y):
    height, width, data = image
    i = (y * width + x) * 3
    return tuple(data[i:i + 3])


def test_image_has_the_five_grids_stacked():
    leds = LedModel(copy.deepcopy(CATALOGUE))
    layout = picture.Layout(CATALOGUE)
    colours = layout.colours(leds.state(0), 0)
    data = layout.render(colours)
    assert len(colours) == 5 and len(data) == layout.height * layout.width * 3
    assert layout.height > (5 + 5 + 4 + 7 + 7) * picture.PITCH_PX
    assert layout.width > 24 * picture.PITCH_PX
    assert [len(c) for c in layout.cells] == [40, 40, 96, 31, 31]


def test_modes_colour_the_cells_off_black_solid_its_colour():
    leds = LedModel(copy.deepcopy(CATALOGUE))
    leds.configure(0, 'tfld', led.APPLY_MODE | led.APPLY_BRIGHTNESS, mode=led.MODE_SOLID,
                   rgb=0x2040FF, brightness=100)
    leds.configure(0, 'bfld', led.APPLY_MODE, mode=led.MODE_OFF)
    layout = picture.Layout(CATALOGUE)
    colours = layout.colours(leds.state(0), 0)
    assert colours[:2] == ((0x20, 0x40, 0xFF), (0, 0, 0))
    image = (layout.height, layout.width, layout.render(colours))
    top = picture.MARGIN_PX + 1
    left = picture.MARGIN_PX + picture.PITCH_PX // 2 + 1
    assert pixel(image, left, top) == (0x20, 0x40, 0xFF)
    bottom = top + 5 * picture.PITCH_PX + picture.PITCH_PX
    assert pixel(image, left, bottom) == (0, 0, 0)
    assert pixel(image, left + picture.CELL_PX, top) == (picture.BACKGROUND,) * 3     # the gap


def test_rld_odd_rows_sit_half_a_cell_left_and_the_colours_key_changes_with_a_run():
    layout = picture.Layout(CATALOGUE)
    rld = layout.cells[2]
    assert rld[0][1] - rld[24][1] == picture.PITCH_PX // 2
    leds = LedModel(copy.deepcopy(CATALOGUE))
    before = layout.colours(leds.state(0), 0)
    leds.run(0, 'rld', 'fire')
    after = layout.colours(leds.state(1), 1)
    assert after[2] != before[2] and after[:2] == before[:2]


def test_psi_corners_are_absent_and_brightness_scales():
    spec = CATALOGUE['displays'][0]
    cells = picture.grid_cells(spec)
    assert len(cells) == spec['leds'] == 31 and (0, 0) not in cells and (3, 2) in cells
    half = picture.display_colour('fld', {'mode': 0, 'brightness': 50, 'rgb': 0}, '', 0, 0)
    full = picture.display_colour('fld', {'mode': 0, 'brightness': 100, 'rgb': 0}, '', 0, 0)
    assert half == tuple(c // 2 for c in full)


def test_each_effect_name_gets_its_own_colour_and_a_cycle_child_shares_the_family():
    names = [e['name'] for e in CATALOGUE['effects']]
    assert len({picture.effect_colour(n) for n in names}) == len(names)
    assert picture.effect_colour('cycle:fire') == picture.effect_colour('cycle')


def test_psi_alternates_every_period():
    st = {'mode': 0, 'brightness': 100, 'rgb': 0}
    assert picture.display_colour('front_psi', st, '', 0, 0) != \
        picture.display_colour('front_psi', st, '', 0, 10)
    assert picture.display_colour('rear_psi', st, '', 0, 0) != \
        picture.display_colour('rear_psi', st, '', 0, 5)


def test_picture_text_has_a_line_per_surface_and_display_with_attribution():
    leds = LedModel(copy.deepcopy(CATALOGUE))
    leds.run(0, 'fld', 'fire', owner='r2_expression', source='ros')
    leds.configure(0, 'rld', led.APPLY_BRIGHTNESS, brightness=20, source='console')
    text = picture.picture_text(leds.state(1), MagicModel(copy.deepcopy(CATALOGUE)).state(1))
    lines = text.splitlines()
    assert len([ln for ln in lines if ln.startswith('surface ')]) == 6
    assert len([ln for ln in lines if ln.startswith('display ')]) == 5
    assert any('surface fld: running effect=fire' in ln and 'owner=r2_expression' in ln
               for ln in lines)
    assert any(ln.startswith('display rld:') and 'held_by=console' in ln and 'hold=4000ms' in ln
               for ln in lines)


@pytest.mark.parametrize('line', ['leds run fld fire -t inf', 'leds run fld fire -t -5',
                                  'leds run fld fire -t 1e400', 'leds say fld -t x hi'])
def test_bad_seconds_are_a_console_error_not_a_crash(rig, line):
    assert rig[2].execute(0, line)[0] is False
    assert not any(s['running'] for s in rig[0].surface_states(0))


def test_rotary_on_an_unknown_code_names_the_reason(rig):
    leds, panel, con = rig
    panel.sequences.pop(9)
    assert con.execute(0, 'rotary 9') == (False, 'unknown sequence')
