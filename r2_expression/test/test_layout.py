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

"""Structural checks of the Foxglove expression tab (plain JSON, no Foxglove)."""

import json
import os

import pytest

LAYOUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      'foxglove', 'expression-panels.json')
INTENT_FIELDS = {'header', 'channel', 'source', 'priority', 'action', 'params',
                 'duration_ms', 'interruptible', 'intent_id'}
CLAIM_FIELDS = {'channel', 'owner', 'priority', 'lease_s'}


def strict_json(text):
    def reject(token):
        raise ValueError('%s is not JSON' % token)
    return json.loads(text, parse_constant=reject)


@pytest.fixture(scope='module')
def layout():
    with open(LAYOUT) as f:
        return strict_json(f.read())


def panels(layout, kind):
    return {k: v for k, v in layout['configById'].items() if k.split('!')[0] == kind}


def leaves(node):
    if isinstance(node, str):
        return [node]
    assert node['direction'] in ('row', 'column') and 0 < node['splitPercentage'] < 100
    return leaves(node['first']) + leaves(node['second'])


def test_every_panel_is_placed_once(layout):
    placed = leaves(layout['layout'])
    assert sorted(placed) == sorted(layout['configById'])


def test_publish_panels_send_complete_manual_intents(layout):
    seen = {}
    for cfg in panels(layout, 'Publish').values():
        assert cfg['topicName'] == '/expression/intent'
        assert cfg['datatype'] == 'astromech_msgs/msg/ExpressionIntent'
        body = strict_json(cfg['value'])
        assert set(body) == INTENT_FIELDS
        assert body['source'] == 'manual' and body['priority'] == 199
        params = strict_json(body['params']) if body['params'] else {}
        seen[body['action'], body['channel']] = (params, body['duration_ms'])
    assert seen == {
        ('effect', 'fld'): ({'name': 'fire'}, 5000),
        ('sequence', 'magic'): ({'id': 44}, 3000),
        ('configure', 'rld'): ({'drift': 'lively'}, 0),
        ('stop', 'fld'): ({}, 0),
    }


def test_claim_and_release_panels(layout):
    calls = panels(layout, 'CallService').values()
    seen = {(c['serviceName'], strict_json(c['requestPayload'])['channel'],
             strict_json(c['requestPayload'])['lease_s']) for c in calls}
    assert seen == {('/expression/claim/led', 'fld', 30), ('/expression/claim/led', 'fld', 0),
                    ('/expression/claim/magic', 'magic', 30),
                    ('/expression/claim/magic', 'magic', 0)}
    bodies = [strict_json(c['requestPayload']) for c in calls]
    assert all(set(b) == CLAIM_FIELDS for b in bodies)
    assert all(b['owner'] == 'foxglove' and b['priority'] == 199 for b in bodies)


def test_state_and_sim_panels(layout):
    (st,) = panels(layout, 'StateTransitions').values()
    assert all(p['value'].startswith('/expression/status{channel==') and
               p['value'].endswith('.state') for p in st['paths'])
    assert len(st['paths']) == 6
    (raw,) = panels(layout, 'RawMessages').values()
    assert raw['topicPath'] == '/r2_dome/mock/picture'
    (img,) = panels(layout, 'Image').values()
    assert img['cameraTopic'] == '/r2_dome/mock/image'
