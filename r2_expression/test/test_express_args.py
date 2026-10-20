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
import os
import sys

import pytest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

from r2_expression.express_cli import build_intent, parse_args  # noqa: E402


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
    assert params('sequence', 'magic', '44', '--end', 'blank') == \
        {'id': 44, 'repeat': 1, 'end': 'blank'}
    assert params('sequence', 'magic', 'sparkle', '--repeat', '3') == \
        {'name': 'sparkle', 'repeat': 3}
    assert params('stop', 'fld') == {}
    assert intent('stop', 'fld')['params'] == ''
    assert params('stop', 'magic', '--mode', 'freeze') == {'mode': 'freeze'}


def test_magic_configure_is_the_brightness_action():
    f = intent('configure', 'magic', '--brightness', '9')
    assert f['action'] == 'brightness' and json.loads(f['params']) == {'value': 9}


def test_claim_defaults_and_override():
    ns = parse_args(['claim', 'fld'])
    assert (ns.priority, ns.lease, ns.owner, ns.claim_service) == \
        (199, 30.0, 'express', '/expression/claim')
    assert parse_args(['claim', 'fld', '--override']).priority == 200
    assert parse_args(['release', 'fld', '--owner', 'me']).owner == 'me'


@pytest.mark.parametrize('argv', [
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
