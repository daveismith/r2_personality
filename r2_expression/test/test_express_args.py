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

"""The express argument parser and intent builder (no ROS)."""

import json
import math
import os
import sys
from types import SimpleNamespace

import pytest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

from r2_expression.core import DOME_ACTIONS, SOUND_ACTIONS  # noqa: E402
from r2_expression.express_cli import (  # noqa: E402
    build_intent, DOME_VERBS, format_clip, format_status, parse_args, pool_clips)


def intent(*argv):
    return build_intent(parse_args(list(argv)))


def params(*argv):
    text = intent(*argv)['params']
    return json.loads(text) if text else {}


def bad(capsys, *argv):
    with pytest.raises(SystemExit) as exc:
        parse_args(list(argv))
    assert exc.value.code == 2
    return capsys.readouterr().err


def test_defaults_are_manual_199_interruptible_with_a_generated_id():
    f = intent('effect', 'fld', 'fire')
    assert (f['source'], f['priority'], f['interruptible']) == ('manual', 199, True)
    assert f['channel'] == 'fld' and f['action'] == 'effect' and f['duration_ms'] == 0
    assert f['intent_id'].startswith('express-')
    assert f['intent_id'] != intent('effect', 'fld', 'fire')['intent_id']


def test_override_is_200_and_id_duration_hold_are_honoured():
    f = intent('effect', 'fld', 'fire', '--override', '--id', 'x1', '--duration', '2.5',
               '--hold')
    assert (f['priority'], f['intent_id'], f['duration_ms'], f['interruptible']) == \
        (200, 'x1', 2500, False)


def test_source_priorities():
    assert intent('stop', 'fld', '--source', 'idle')['priority'] == 10
    f = intent('stop', 'fld', '--source', 'behavior')
    assert (f['source'], f['priority']) == ('behavior', 100)
    assert intent('stop', 'fld', '--source', 'behavior', '--priority', '60')['priority'] == 60


def test_params_per_action():
    assert params('effect', 'fld', 'fire', '--rgb', 'ff2200', '--speed', '5') == \
        {'name': 'fire', 'rgb': 0xff2200, 'speed': 5}
    assert params('clip', 'rld', 'wave') == {'name': 'wave'}
    assert params('text', 'fld', 'hi', 'there', '--motion', 'loop', '--on-black') == \
        {'lines': ['hi', 'there'], 'motion': 'loop', 'on_black': True}
    assert params('configure', 'rld', '--drift', 'lively', '--brightness', '40') == \
        {'drift': 'lively', 'brightness': 40}
    assert params('sequence', 'magic', '44', '--end', 'leave') == \
        {'id': 44, 'repeat': 1, 'end': 'leave'}
    assert params('sequence', 'magic', 'sparkle', '--repeat', '3') == \
        {'name': 'sparkle', 'repeat': 3, 'end': 'blank'}
    assert params('stop', 'fld') == {}
    assert intent('stop', 'fld')['params'] == ''
    assert params('stop', 'magic', '--mode', 'freeze') == {'mode': 'freeze'}


def test_a_sequence_ends_blank_after_one_run_by_default():
    assert params('sequence', 'magic', '44') == {'id': 44, 'repeat': 1, 'end': 'blank'}


def test_dome_targets_the_sim_mock_by_default_never_the_real_dome():
    ns = parse_args(['dome', 'leds run rld rainbow'])
    assert ns.device == '/dome_mock'


def test_magic_configure_is_the_brightness_action():
    f = intent('configure', 'magic', '--brightness', '9')
    assert f['action'] == 'brightness' and json.loads(f['params']) == {'value': 9}


def test_claim_defaults_and_override():
    ns = parse_args(['claim', 'fld'])
    assert (ns.priority, ns.lease, ns.owner, ns.claim_service) == \
        (199, 30.0, 'express', '/expression/claim/led')
    assert parse_args(['claim', 'fld', '--override']).priority == 200
    assert parse_args(['release', 'fld', '--owner', 'me']).owner == 'me'


def test_claim_service_follows_the_channel():
    for channel in ('fld', 'tfld', 'bfld', 'rld', 'psi'):
        for verb in ('claim', 'release'):
            assert parse_args([verb, channel]).claim_service == '/expression/claim/led'
    for verb in ('claim', 'release'):
        assert parse_args([verb, 'magic']).claim_service == '/expression/claim/magic'
    assert parse_args(['claim', 'magic', '--claim-service', '/x']).claim_service == '/x'


def test_drift_is_accepted_on_the_logic_displays():
    for channel in ('fld', 'tfld', 'bfld', 'rld'):
        assert params('configure', channel, '--drift', 'calm') == {'drift': 'calm'}


def test_look_builds_a_dome_look_at_at_manual_199():
    f = intent('look', '0.8', '--profile', 'snappy')
    assert (f['channel'], f['source'], f['priority'], f['action']) == \
        ('dome', 'manual', 199, 'look_at')
    assert json.loads(f['params']) == {'bearing': 0.8, 'profile': 'snappy'}
    assert (f['duration_ms'], f['interruptible']) == (0, True)
    assert params('look', '-1.5') == {'bearing': -1.5}           # negative bearings parse
    assert params('look', str(math.pi)) == {'bearing': math.pi}
    assert params('look', '0') == {'bearing': 0.0}               # 0 is a bearing, not absent


def test_dome_verbs_are_core_actions():
    assert set(DOME_VERBS.values()) <= set(DOME_ACTIONS)


def test_glance_sends_only_what_was_given():
    assert intent('glance')['params'] == '' and intent('glance')['action'] == 'glance'
    assert params('glance', '--amplitude', '0.5', '--dwell', '1.5', '--side', 'right') == \
        {'amplitude': 0.5, 'dwell_s': 1.5, 'side': 'right'}
    assert params('glance', '--side', 'left') == {'side': 'left'}


def test_turn_and_rest():
    f = intent('turn', '-1.57')
    assert (f['action'], f['channel']) == ('turn', 'dome')
    assert json.loads(f['params']) == {'delta': -1.57}
    assert params('turn', str(2 * math.pi)) == {'delta': 2 * math.pi}
    assert params('turn', str(-2 * math.pi)) == {'delta': -2 * math.pi}
    assert intent('rest')['params'] == '' and intent('rest')['action'] == 'rest'
    assert params('rest', '0.35') == {'bearing': 0.35}
    assert params('rest', '0') == {'bearing': 0.0}


def test_dome_verbs_honour_override_source_hold_and_id():
    assert intent('turn', '1', '--override')['priority'] == 200
    f = intent('look', '0', '--source', 'behavior', '--hold', '--id', 'd1')
    assert (f['source'], f['priority'], f['interruptible'], f['intent_id']) == \
        ('behavior', 100, False, 'd1')
    assert intent('rest', '--source', 'idle')['priority'] == 10


def test_stop_and_claim_reach_the_dome_channel():
    f = intent('stop', 'dome')
    assert (f['channel'], f['action'], f['params'], f['priority']) == ('dome', 'stop', '', 199)
    assert parse_args(['claim', 'dome']).claim_service == '/expression/claim/dome'
    assert parse_args(['release', 'dome']).claim_service == '/expression/claim/dome'


def test_status_names_the_suppressed_state():
    msg = SimpleNamespace(channel='dome', state=7, owner='', priority=0,
                          lease_remaining_s=-1.0, active_action='', mood='', dropped=0,
                          refused=0)
    assert format_status(msg).startswith('dome  suppressed')


# --- sound (phase 3 plan 5.5) -----------------------------------------------------------------

def test_play_by_name_builds_a_sound_play_at_manual_199():
    f = intent('play', 'scream')
    assert (f['channel'], f['source'], f['priority'], f['action']) == (
        'sound', 'manual', 199, 'play')
    assert json.loads(f['params']) == {'name': 'scream'}
    assert (f['duration_ms'], f['interruptible']) == (0, True)       # a person's newest wins
    assert f['action'] in SOUND_ACTIONS


def test_play_gain_duration_pool_hold_override_and_source():
    f = intent('play', 'scream', '--gain', '-3.5', '--duration', '2', '--hold', '--id', 'p1')
    assert json.loads(f['params']) == {'name': 'scream', 'gain_db': -3.5}
    assert (f['duration_ms'], f['interruptible'], f['intent_id']) == (2000, False, 'p1')
    f = intent('play', '--pool', 'excited', '--gain', '0')
    assert json.loads(f['params']) == {'pool': 'excited', 'gain_db': 0.0}
    assert intent('play', 'x', '--override')['priority'] == 200
    f = intent('play', '--pool', 'hum', '--source', 'behavior')
    assert (f['source'], f['priority'], f['channel']) == ('behavior', 100, 'sound')
    assert intent('play', 'x', '--source', 'idle')['priority'] == 10


def test_vocalize_sends_only_what_was_given():
    f = intent('vocalize')
    assert (f['channel'], f['action'], f['params']) == ('sound', 'vocalize', '')
    assert params('vocalize', '--arousal', '0.8', '--valence', '0', '--length', '1.5',
                  '--seed', '-42', '--gain', '2') == {
        'arousal': 0.8, 'valence': 0.0, 'length_s': 1.5, 'seed': -42, 'gain_db': 2.0}
    assert params('vocalize', '--seed', '0') == {'seed': 0}


def test_stop_sound_and_the_sound_claim_service():
    f = intent('stop', 'sound')
    assert (f['channel'], f['action'], f['params'], f['priority']) == ('sound', 'stop', '', 199)
    for verb in ('claim', 'release'):
        assert parse_args([verb, 'sound']).claim_service == '/expression/claim/sound'
    assert parse_args(['claim', 'sound', '--claim-service', '/x']).claim_service == '/x'


def test_sounds_defaults_to_the_sound_player_and_reads_only():
    ns = parse_args(['sounds'])
    assert (ns.device, ns.pool) == ('/sound_player', None)
    ns = parse_args(['sounds', '--pool', 'hum', '--device', '/sound_player_t1'])
    assert (ns.device, ns.pool) == ('/sound_player_t1', 'hum')


def test_sounds_help_says_it_is_read_only(capsys):
    with pytest.raises(SystemExit):
        parse_args(['sounds', '--help'])
    out = ' '.join(capsys.readouterr().out.split())
    assert 'read-only' in out and 'nothing is played' in out


def test_the_catalogue_listing_filters_by_tag_and_formats_a_line():
    clips = [SimpleNamespace(name='a', tags=['hum', 'calm'], duration_ms=1500, gain_db=-2.0,
                             excluded=False),
             SimpleNamespace(name='cantina', tags=[], duration_ms=120000, gain_db=0.0,
                             excluded=True)]
    assert [c.name for c in pool_clips(clips)] == ['a', 'cantina']
    assert [c.name for c in pool_clips(clips, 'hum')] == ['a']
    assert pool_clips(clips, 'scream') == []
    line = format_clip(clips[0])
    assert line.startswith('a ') and '1.5s' in line and '-2.0 dB' in line
    assert line.endswith('hum,calm')
    assert format_clip(clips[1]).endswith('-  (manual only)')


def test_sound_status_line_shows_the_play_and_its_remaining_time():
    msg = SimpleNamespace(channel='sound', state=1, owner='manual', priority=199,
                          lease_remaining_s=4.5, active_source='manual', active_action='play',
                          active_remaining_ms=1800, mood='calm', dropped=1, refused=0)
    assert format_status(msg) == ('sound running  owner=manual prio=199 lease=4.5s '
                                  'active=manual/play 1800ms mood=calm dropped=1 refused=0')


@pytest.mark.parametrize('argv', [
    ['play'],
    ['play', 'scream', '--pool', 'hum'],
    ['play', '--pool', 'moo'],
    ['play', '--pool', 'excluded'],
    ['play', 'x' * 64],
    ['play', 'x', '--gain', '99'],
    ['play', 'x', '--gain', 'nan'],
    ['play', 'x', '--gain', 'loud'],
    ['play', 'x', '--duration', '-1'],
    ['play', 'x', '--priority', '50'],
    ['play', 'x', '--override', '--source', 'idle'],
    ['vocalize', '--arousal', '1.5'],
    ['vocalize', '--valence', '-0.1'],
    ['vocalize', '--length', '-1'],
    ['vocalize', '--seed', '1.5'],
    ['vocalize', '--seed', '9999999999'],
    ['vocalize', '--duration', '3'],
    ['sounds', '--pool', 'excluded-ish'],
    ['sounds', '--device', 'sound_player'],
    ['sounds', '--device', '/'],
    ['stop', 'sound', '--mode', 'freeze'],
    ['effect', 'sound', 'fire'],
    ['claim', 'sound', '--lease', 'abc'],
])
def test_sound_exit_2_validations(capsys, argv):
    assert bad(capsys, *argv)


@pytest.mark.parametrize('argv', [
    ['look'],
    ['look', 'abc'],
    ['look', 'nan'],
    ['look', 'inf'],
    ['look', '3.1416'],
    ['look', str(-math.pi)],                      # (-pi, pi]: -pi is pi
    ['look', '0', '--profile', 'a b'],
    ['look', '0', '--profile', 'x/y'],
    ['look', '0', '--profile', ''],
    ['look', '0', '--profile', 'x' * 64],
    ['look', '0', '--duration', '2'],
    ['look', '0', '--priority', '50'],
    ['look', '0', '--override', '--source', 'idle'],
    ['glance', '--amplitude', '0'],
    ['glance', '--amplitude', '3.2'],
    ['glance', '--dwell', '0'],
    ['glance', '--dwell', '-1'],
    ['glance', '--dwell', 'inf'],
    ['glance', '--side', 'up'],
    ['turn'],
    ['turn', '6.29'],
    ['turn', '-6.29'],
    ['turn', 'x'],
    ['rest', '4'],
    ['rest', '0.1', '0.2'],
    ['stop', 'dome', '--mode', 'freeze'],
    ['effect', 'dome', 'fire'],
    ['claim', 'dome', '--lease', 'abc'],
    ['effect', 'nope', 'fire'],
    ['effect', 'fld', 'fire', '--rgb', 'ff22'],
    ['effect', 'fld', 'fire', '--rgb', 'gg2200'],
    ['effect', 'fld', 'fire', '--speed', '0'],
    ['effect', 'fld', 'fire', '--speed', '10'],
    ['effect', 'fld', 'x' * 64],
    ['effect', 'fld', 'fire', '--duration', '-1'],
    ['effect', 'magic', 'fire'],
    ['configure', 'fld'],
    ['configure', 'fld', '--brightness', '101'],
    ['configure', 'magic', '--brightness', '16'],
    ['configure', 'fld', '--hue', '3'],
    ['configure', 'magic', '--drift', 'calm'],
    ['configure', 'psi', '--drift', 'calm'],
    ['text', 'fld', 'a', 'b', 'c'],
    ['text', 'rld', 'a', 'b'],
    ['clip', 'tfld', 'wave'],
    ['text', 'fld', 'x' * 64],
    ['text', 'fld', '\u00e9' * 40],          # 80 bytes in UTF-8, the device's strlen
    ['text', 'psi', 'hi'],
    ['sequence', 'fld', '44'],
    ['sequence', 'magic', '256'],
    ['stop', 'fld', '--mode', 'freeze'],
    ['stop', 'fld', '--override', '--source', 'idle'],
    ['stop', 'fld', '--priority', '50'],
    ['effect', 'fld', 'fire', '--source', 'manual'],
    ['claim', 'fld', '--lease', 'abc'],
    ['claim', 'tail'],
    [],
])
def test_exit_2_validations(capsys, argv):
    assert bad(capsys, *argv)
