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

import math

import pytest

from r2_stimulus.stimulate import build_message_fields, parse_args


def test_defaults():
    ns = parse_args(['bump'])
    assert ns.intensity == 1.0 and ns.confidence == 1.0
    assert ns.subject == '' and ns.ttl == 0.0 and ns.extra == ''
    assert ns.repeat == 1 and ns.every == 0.5 and ns.source == 'stimulate_cli'
    assert math.isnan(ns.bearing)
    assert math.isnan(build_message_fields(ns)['bearing_rad'])


def test_zero_bearing_is_kept():
    assert parse_args(['bump', '--bearing', '0']).bearing == 0.0


@pytest.mark.parametrize('argv', [
    [''], ['a b'], ['a\tb'],
    ['bump', '--intensity', '1.1'], ['bump', '--intensity', '-0.1'],
    ['bump', '--confidence', '2'], ['bump', '--confidence', '-1'],
    ['bump', '--repeat', '0'], ['bump', '--every', '-1'],
    ['bump', '--extra', '{bad'], ['bump', '--extra', '[1]'], ['bump', '--extra', '3'],
    ['bump', '--intensity', 'nan'],
])
def test_bad_input_exits_2(argv):
    with pytest.raises(SystemExit) as exc:
        parse_args(argv)
    assert exc.value.code == 2


def test_extra_round_trip():
    ns = parse_args(['button_sequence', '--extra', '{"taps":"LRR"}', '--subject', 'LRR',
                     '--ttl', '5', '--bearing', '1.2', '--source', 'x'])
    f = build_message_fields(ns)
    assert f['kind'] == 'button_sequence' and f['subject'] == 'LRR'
    assert f['ttl_s'] == 5.0 and f['source'] == 'x'
    assert f['bearing_rad'] == pytest.approx(1.2)
    import json
    assert json.loads(f['extra']) == {'taps': 'LRR'}


def test_unknown_kind_accepted():
    assert parse_args(['nonsense']).kind == 'nonsense'
