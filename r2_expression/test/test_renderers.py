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


"""The LED and Magic Panel renderers: requests built exactly, local refusals, rules 6 and 9."""

import copy

from fixtures import BASE, LED_CAT, MAGIC_CAT
import pytest
from r2_expression import core
from r2_expression import led_renderer
from r2_expression import magic_renderer
from r2_expression.config import parse_config
from r2_expression.render_common import RenderRefusal

OWNER = 'r2_expression'


def action(call, request, channel='fld', target='fld', priority=10):
    return core.Action(call, channel, target, request, OWNER, priority, 'id1', 'intent')


def run_request(**changes):
    request = {'surface': 'fld', 'effect': 'fire', 'rgb': 0xff2200, 'speed': 3,
               'duration_ms': 1500, 'owner': OWNER, 'priority': 10}
    request.update(changes)
    return request


def say_request(**changes):
    request = {'surface': 'fld', 'lines': ['hello', 'world'], 'linked': True, 'motion': 2,
               'on_black': True, 'rgb': 0x112233, 'speed': 4, 'duration_ms': 900,
               'owner': OWNER, 'priority': 10}
    request.update(changes)
    return request


def raw_action(call, request, channel='fld', target='fld'):
    """Return an Action that skipped the core's own checks, to test the renderer's second look."""
    act = core.Action.__new__(core.Action)
    for key, value in (('call', call), ('channel', channel), ('target', target),
                       ('request', request), ('owner', OWNER), ('priority', 10),
                       ('intent_id', ''), ('why', '')):
        object.__setattr__(act, key, value)
    return act


def refusal(render, act):
    with pytest.raises(RenderRefusal) as info:
        render(act)
    return info.value.reason


# --- LED requests are built field for field ----------------------------------------------------

def test_run_request_has_every_field():
    msg = led_renderer.render(action(core.CALL_RUN, run_request()))
    assert type(msg).__name__ == 'RunLedEffect_Request'
    assert (msg.surface, msg.effect, msg.rgb, msg.speed, msg.duration_ms, msg.owner,
            msg.priority) == ('fld', 'fire', 0xff2200, 3, 1500, OWNER, 10)


def test_say_request_has_every_field():
    msg = led_renderer.render(action(core.CALL_SAY, say_request()))
    assert (msg.surface, list(msg.lines), msg.linked, msg.motion, msg.on_black, msg.rgb,
            msg.speed, msg.duration_ms, msg.owner, msg.priority) == (
        'fld', ['hello', 'world'], True, 2, True, 0x112233, 4, 900, OWNER, 10)


def test_led_stop_request():
    msg = led_renderer.render(action(core.CALL_STOP, {'surface': 'rld', 'priority': 199},
                                     priority=199))
    assert (msg.surface, msg.priority) == ('rld', 199)


def test_configure_request_applies_only_what_is_asked():
    request = {'display': 'tfld', 'apply': core.APPLY_BRIGHTNESS | core.APPLY_DRIFT,
               'mode': 0, 'brightness': 70, 'drift': 'lively', 'owner': OWNER, 'priority': 10}
    msg = led_renderer.render(action(core.CALL_CONFIGURE, request, target='tfld'))
    assert (msg.display, msg.apply, msg.brightness, msg.drift, msg.owner, msg.priority) == (
        'tfld', 10, 70, 'lively', OWNER, 10)


# --- local refusals, with the device's strings, before any call ---------------------------------

@pytest.mark.parametrize('surface', ['fpsi', 'rpsi'])
def test_text_on_a_psi_is_refused_like_the_device(surface):
    act = action(core.CALL_SAY, say_request(surface=surface, lines=['x']), 'psi', surface)
    assert refusal(led_renderer.render, act) == 'display has no text mode'


@pytest.mark.parametrize('surface,lines', [
    ('fld', ['a', 'b', 'c']), ('rld', ['a', 'b']), ('tfld', ['a', 'b'])])
def test_more_lines_than_displays(surface, lines):
    act = action(core.CALL_SAY, say_request(surface=surface, lines=lines), surface, surface)
    assert refusal(led_renderer.render, act) == 'more lines than the surface has displays'


def test_two_lines_fit_fld_and_one_fits_rld():
    led_renderer.render(action(core.CALL_SAY, say_request()))
    led_renderer.render(action(core.CALL_SAY, say_request(surface='rld', lines=['a']),
                               'rld', 'rld'))


def test_a_line_of_64_characters_is_text_too_long_and_63_is_fine():
    led_renderer.render(action(core.CALL_SAY, say_request(lines=['x' * 63])))
    for lines in (['x' * 64], ['ok', 'y' * 64]):
        assert refusal(led_renderer.render,
                       action(core.CALL_SAY, say_request(lines=lines))) == 'text too long'


def test_text_length_is_counted_in_utf_8_bytes_like_the_devices_strlen():
    led_renderer.render(action(core.CALL_SAY, say_request(lines=['\u00e9' * 31 + 'x'])))
    for lines in (['\u00e9' * 40], ['ok', '\u00e9' * 32]):       # 80 and 64 bytes, 40/32 chars
        assert refusal(led_renderer.render,
                       action(core.CALL_SAY, say_request(lines=lines))) == 'text too long'


def test_no_text_is_refused():
    assert refusal(led_renderer.render,
                   action(core.CALL_SAY, say_request(lines=[]))) == 'no text'


def test_a_name_over_63_is_refused_and_63_is_fine():
    led_renderer.render(action(core.CALL_RUN, run_request(effect='e' * 63)))
    assert refusal(led_renderer.render,
                   action(core.CALL_RUN, run_request(effect='e' * 70))) == 'name too long'


def test_an_owner_over_31_is_refused_by_the_device_string():
    act = raw_action(core.CALL_RUN, run_request(owner='o' * 32))
    assert refusal(led_renderer.render, act) == 'owner too long'


def test_a_call_of_the_other_renderer_is_an_unknown_action():
    act = action(core.CALL_START, {'sequence_id': 1, 'repeat': 1, 'end': 0, 'owner': OWNER,
                                   'priority': 10}, 'magic', 'magic')
    assert refusal(led_renderer.render, act) == 'unknown action'
    assert refusal(magic_renderer.render, action(core.CALL_RUN, run_request())) == \
        'unknown action'


# --- rules 4, 6 and 9 ---------------------------------------------------------------------------

def test_a_request_with_priority_zero_cannot_be_built():
    with pytest.raises(ValueError):
        action(core.CALL_RUN, run_request(priority=0), priority=0)
    # and the renderer checks the message itself, whatever produced the Action
    act = raw_action(core.CALL_RUN, run_request(priority=0))
    with pytest.raises(ValueError, match='priority'):
        led_renderer.render(act)


def test_a_request_without_an_owner_is_a_bug():
    act = raw_action(core.CALL_RUN, run_request(owner=''))
    with pytest.raises(ValueError, match='owner'):
        led_renderer.render(act)


@pytest.mark.parametrize('apply', [core.APPLY_HUE, core.APPLY_BRIGHTNESS | 16])
def test_configure_never_carries_hue_or_save(apply):
    act = raw_action(core.CALL_CONFIGURE,
                     {'display': 'fld', 'apply': apply, 'owner': OWNER, 'priority': 10})
    with pytest.raises(ValueError):
        led_renderer.render(act)


def test_a_request_field_the_service_lacks_is_a_bug():
    with pytest.raises(ValueError, match='no field'):
        led_renderer.render(action(core.CALL_RUN, run_request(colour=3)))


# --- Magic Panel -------------------------------------------------------------------------------

def test_start_request():
    act = action(core.CALL_START, {'sequence_id': 44, 'repeat': 2, 'end': 1, 'owner': OWNER,
                                   'priority': 199}, 'magic', 'magic', 199)
    msg = magic_renderer.render(act)
    assert (msg.sequence_id, msg.repeat, msg.end, msg.owner, msg.priority) == (
        44, 2, 1, OWNER, 199)


def test_magic_stop_request():
    msg = magic_renderer.render(action(core.CALL_MAGIC_STOP, {'mode': 1, 'priority': 200},
                                       'magic', 'magic', 200))
    assert (msg.mode, msg.priority) == (1, 200)


def test_panel_brightness_is_brightness_only_and_has_no_priority():
    act = action(core.CALL_MAGIC_BRIGHTNESS, {'apply': 1, 'brightness': 9}, 'magic', 'magic')
    msg = magic_renderer.render(act)
    assert (msg.apply, msg.brightness) == (1, 9)
    assert not hasattr(msg, 'priority') and not hasattr(msg, 'owner')


@pytest.mark.parametrize('apply', [2, 64, 128, 3])
def test_panel_configure_never_saves_or_resets(apply):
    act = raw_action(core.CALL_MAGIC_BRIGHTNESS, {'apply': apply, 'brightness': 3},
                     'magic', 'magic')
    with pytest.raises(ValueError):
        magic_renderer.render(act)


def test_panel_brightness_out_of_range():
    act = action(core.CALL_MAGIC_BRIGHTNESS, {'apply': 1, 'brightness': 16}, 'magic', 'magic')
    assert refusal(magic_renderer.render, act) == 'out of range'


def test_start_with_priority_zero_is_a_bug():
    act = raw_action(core.CALL_START, {'sequence_id': 1, 'repeat': 1, 'end': 0,
                                       'owner': OWNER, 'priority': 0}, 'magic', 'magic')
    with pytest.raises(ValueError):
        magic_renderer.render(act)


# --- the core's own actions all render ---------------------------------------------------------

def test_every_action_the_core_produces_renders():
    arb = core.Arbiter(parse_config(copy.deepcopy(BASE)), core.LED_CHANNELS)
    actions = list(arb.set_catalogue(LED_CAT, 0.0))
    actions += arb.mood('excited', 1.0, 0.1)
    for fields in ({'channel': 'psi', 'action': 'effect', 'params': '{"name": "fire"}'},
                   {'channel': 'fld', 'action': 'text', 'params': '{"lines": ["hi"]}'},
                   {'channel': 'tfld', 'action': 'configure',
                    'params': '{"brightness": 50, "drift": "calm"}'},
                   {'channel': 'fld', 'action': 'stop'}):
        actions += arb.intent(core.Intent(source='manual', priority=199, **fields), 0.2).actions
    assert actions
    for act in actions:
        msg = led_renderer.render(act)
        assert msg.priority != 0 and getattr(msg, 'owner', OWNER) != ''


def test_magic_catalogue_actions_render():
    arb = core.Arbiter(parse_config(copy.deepcopy(BASE)), core.MAGIC_CHANNELS)
    actions = list(arb.set_catalogue(MAGIC_CAT, 0.0))
    actions += arb.mood('excited', 1.0, 0.1)
    actions += arb.intent(core.Intent('magic', 'manual', 199, 'sequence',
                                      '{"name": "Smile"}'), 0.2).actions
    assert any(a.call == core.CALL_START for a in actions)
    for act in actions:
        magic_renderer.render(act)
