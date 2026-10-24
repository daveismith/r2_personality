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


"""The renderers: requests built exactly, local refusals, rules 6 and 9; the dome's Twist."""

import copy

from fixtures import BASE, dome_cfg, LED_CAT, MAGIC_CAT, sound_cfg, sound_list
import pytest
from r2_expression import core
from r2_expression import dome_renderer
from r2_expression import led_renderer
from r2_expression import magic_renderer
from r2_expression import sound_renderer
from r2_expression.config import parse_config
from r2_expression.palette import sound_catalogue
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


# --- the dome --------------------------------------------------------------------------------

def velocity(value):
    return action(core.CALL_VELOCITY, {'velocity_rad_s': value}, 'dome', 'dome')


@pytest.mark.parametrize('value', [0.0, 0.5, -1.25, 2.0, -2.0])
def test_dome_velocity_is_angular_z_of_a_twist_stamped_and_nothing_else(value):
    msg = dome_renderer.render(velocity(value), 0.5, 2.0)
    assert type(msg).__name__ == 'TwistStamped'
    assert msg.twist.angular.z == value
    assert (msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
            msg.twist.angular.x, msg.twist.angular.y) == (0.0,) * 5
    assert msg.header.stamp.sec == 0 and msg.header.frame_id == ''   # the node stamps it


@pytest.mark.parametrize('value', [0.3, -0.49, 2.01, float('nan'), float('inf'), True, '1.0',
                                   None])
def test_dome_velocity_outside_0_or_min_to_cap_is_a_bug_not_a_refusal(value):
    act = raw_action(core.CALL_VELOCITY, {'velocity_rad_s': value}, 'dome', 'dome')
    with pytest.raises(ValueError):
        dome_renderer.render(act, 0.5, 2.0)


def test_dome_renderer_refuses_every_call_but_velocity():
    with pytest.raises(RenderRefusal) as info:
        dome_renderer.render(action(core.CALL_STOP, {'surface': 'fld'}))
    assert info.value.reason == 'unknown action'


def test_dome_core_actions_render():
    arb = core.Arbiter(dome_cfg(), core.DOME_CHANNELS)
    actions = list(arb.dome_state({'source': 'idle', 'position_rad': 0.0}, 0.0))
    actions += arb.mood('excited', 1.0, 0.0)
    actions += arb.intent(core.Intent('dome', 'manual', 199, 'look_at',
                                      '{"bearing": 1.0}'), 0.1).actions
    actions += arb.intent(core.Intent('dome', 'manual', 199, 'stop'), 0.2).actions
    assert [a.call for a in actions] == [core.CALL_VELOCITY] * 2
    for act in actions:
        dome_renderer.render(act, 0.5, 2.0)


# --- sound -------------------------------------------------------------------------------------

def play_request(**changes):
    request = {'name': 'sent1', 'pool': '', 'gain_db': -2.5, 'duration_ms': 1500,
               'priority': 100, 'owner': OWNER, 'intent_id': 'id1'}
    request.update(changes)
    return request


def sound_action(call, request, priority=100):
    return action(call, request, 'sound', 'sound', priority)


def test_sound_services_and_list_are_the_players_names():
    assert sound_renderer.LIST[0] == 'list' and sound_renderer.LIST[1].__name__ == 'ListSounds'
    assert {call: (suffix, srv.__name__)
            for call, (suffix, srv) in sound_renderer.SERVICES.items()} == {
        core.CALL_PLAY: ('play', 'PlaySound'), core.CALL_VOCALIZE: ('vocalize', 'Vocalize'),
        core.CALL_SOUND_STOP: ('stop', 'StopSound')}


def test_sound_play_by_name_is_built_exactly():
    msg = sound_renderer.render(sound_action(core.CALL_PLAY, play_request()))
    assert type(msg).__name__ == 'PlaySound_Request'
    assert (msg.name, msg.pool, msg.gain_db, msg.duration_ms, msg.priority, msg.owner,
            msg.intent_id) == ('sent1', '', -2.5, 1500, 100, OWNER, 'id1')


def test_sound_play_by_pool_is_built_exactly():
    msg = sound_renderer.render(sound_action(core.CALL_PLAY, play_request(name='', pool='hum')))
    assert (msg.name, msg.pool) == ('', 'hum')


def test_sound_vocalize_is_built_exactly():
    request = {'arousal': 0.8, 'valence': 0.25, 'length_s': 1.5, 'seed': -7, 'gain_db': 1.0,
               'priority': 199, 'owner': OWNER, 'intent_id': 'v1'}
    msg = sound_renderer.render(sound_action(core.CALL_VOCALIZE, request, 199))
    assert type(msg).__name__ == 'Vocalize_Request'
    assert (msg.arousal, msg.valence, msg.length_s, msg.seed, msg.gain_db, msg.priority,
            msg.owner, msg.intent_id) == (pytest.approx(0.8), 0.25, 1.5, -7, 1.0, 199, OWNER,
                                          'v1')


def test_sound_stop_carries_the_owner_and_no_priority():
    msg = sound_renderer.render(sound_action(core.CALL_SOUND_STOP, {'owner': OWNER}))
    assert type(msg).__name__ == 'StopSound_Request' and msg.owner == OWNER
    assert 'priority' not in msg.get_fields_and_field_types()


@pytest.mark.parametrize('changes, reason', [
    ({'name': '', 'pool': ''}, 'unknown name'),
    ({'name': 'sent1', 'pool': 'hum'}, 'unknown name'),
    ({'name': 'x' * 64}, 'name too long'),
    ({'name': '', 'pool': 'p' * 64}, 'name too long'),
    ({'owner': 'o' * 32}, 'owner too long'),
])
def test_sound_play_refusals_are_the_players_own_strings(changes, reason):
    with pytest.raises(RenderRefusal) as info:
        sound_renderer.render(sound_action(core.CALL_PLAY, play_request(**changes)))
    assert info.value.reason == reason


def test_sound_a_stop_with_a_long_owner_is_refused_and_an_unknown_call_too():
    with pytest.raises(RenderRefusal) as info:
        sound_renderer.render(sound_action(core.CALL_SOUND_STOP, {'owner': 'o' * 32}))
    assert info.value.reason == 'owner too long'
    with pytest.raises(RenderRefusal) as info:
        sound_renderer.render(sound_action(core.CALL_RUN, run_request(priority=100)))
    assert info.value.reason == 'unknown action'


@pytest.mark.parametrize('call, request_', [
    (core.CALL_PLAY, play_request(owner='')),
    (core.CALL_PLAY, play_request(priority=0)),
    (core.CALL_SOUND_STOP, {'owner': ''}),
    (core.CALL_PLAY, dict(play_request(), volume=3)),
])
def test_sound_a_request_without_owner_or_priority_or_a_stray_field_is_a_bug(call, request_):
    with pytest.raises(ValueError):
        sound_renderer.render(raw_action(call, request_, 'sound', 'sound'))


def test_sound_core_actions_render():
    arb = core.Arbiter(sound_cfg(), core.SOUND_CHANNELS)
    arb.set_catalogue(sound_catalogue(sound_list(synth=True)), 0.0)
    arb.sound_state({'state': 0}, 0.0)
    arb.mood('calm', 1.0, 0.0)
    actions = []
    for source, priority, action_, params in (
            ('manual', 199, 'play', {'name': 'sent1'}), ('manual', 199, 'play', {'pool': 'hum'}),
            ('manual', 199, 'vocalize', {'seed': 3}), ('manual', 199, 'stop', {})):
        actions += arb.intent(core.Intent('sound', source, priority, action_, params,
                                          interruptible=True), 0.1).actions
    actions += arb.claim('sound', 'op', 250, 10.0, 0.2).actions
    assert [a.call for a in actions] == [core.CALL_PLAY, core.CALL_PLAY, core.CALL_VOCALIZE,
                                         core.CALL_SOUND_STOP]
    for act in actions:
        msg = sound_renderer.render(act)
        assert getattr(msg, 'priority', 1) != 0 and msg.owner == OWNER
