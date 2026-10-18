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

"""Structural checks of the Foxglove tuning layout (no Foxglove: plain JSON)."""

import json
import math
import os
import sys

import pytest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

from r2_affect.config import load_config  # noqa: E402
from r2_affect.model import AffectModel  # noqa: E402

LAYOUT = os.path.join(PKG_ROOT, 'foxglove', 'personality-tuning.json')
SHIPPED = os.path.join(PKG_ROOT, 'config', 'affect.yaml')

# Every field of astromech_msgs/msg/Stimulus, so the panel never relies on defaults.
STIMULUS_FIELDS = {'header', 'kind', 'source', 'intensity', 'confidence', 'subject',
                   'bearing_rad', 'ttl_s', 'extra'}
SETMOOD_FIELDS = {'mood', 'lock', 'duration_s', 'source'}


def strict_json(text):
    """Parse as JavaScript's JSON.parse does (the panels' parser): no NaN or Infinity tokens."""
    def reject(token):
        raise ValueError('%s is not JSON; JSON.parse in Foxglove rejects it' % token)
    return json.loads(text, parse_constant=reject)


@pytest.fixture(scope='module')
def layout():
    with open(LAYOUT) as f:
        return strict_json(f.read())


def panels(layout, panel_type):
    return {pid: cfg for pid, cfg in layout['configById'].items()
            if pid.split('!')[0] == panel_type}


def leaves(node):
    if isinstance(node, str):
        return [node]
    assert node['direction'] in ('row', 'column')
    assert 0 < node.get('splitPercentage', 50) < 100
    return leaves(node['first']) + leaves(node['second'])


def test_layout_is_an_exported_layout_with_every_panel_placed(layout):
    for key in ('configById', 'layout', 'globalVariables', 'playbackConfig'):
        assert key in layout
    placed = leaves(layout['layout'])
    assert sorted(placed) == sorted(layout['configById'])
    assert len(set(placed)) == len(placed)


def test_plot_shows_the_four_axes_and_strength_on_minus_one_to_one(layout):
    (plot,) = panels(layout, 'Plot').values()
    paths = [p['value'] for p in plot['paths']]
    assert paths == ['/mood/state.arousal', '/mood/state.valence', '/mood/state.curiosity',
                     '/mood/state.confidence', '/mood/state.mood_strength']
    assert (plot['minYValue'], plot['maxYValue']) == (-1, 1)


def test_label_raw_and_log_panels(layout):
    (transitions,) = panels(layout, 'StateTransitions').values()
    assert transitions['paths'][0]['value'] == '/mood/state.mood'
    (raw,) = panels(layout, 'RawMessages').values()
    assert raw['topicPath'] == '/mood/state'
    (log,) = panels(layout, 'RosOut').values()
    assert log['searchTerms'] == ['r2_affect'] and log['topicToRender'] == '/rosout'


def test_publish_panels_send_complete_stimuli(layout):
    pubs = panels(layout, 'Publish')
    bodies = {}
    for cfg in pubs.values():
        assert cfg['topicName'] == '/stimuli'
        assert cfg['datatype'] == 'astromech_msgs/msg/Stimulus'
        body = strict_json(cfg['value'])
        assert set(body) == STIMULUS_FIELDS
        assert body['source'] == 'foxglove' and body['confidence'] == 1
        assert body['ttl_s'] == 0 and body['extra'] == ''
        bodies[body['kind']] = body
    assert sorted(bodies) == ['bump', 'button_sequence', 'face_known', 'touched']
    assert bodies['bump']['intensity'] == 1
    assert (bodies['face_known']['subject'], bodies['face_known']['bearing_rad']) == ('dave', 0.8)
    assert bodies['button_sequence']['subject'] == 'LRR'


def test_unknown_bearing_is_infinite_and_r2_affect_reads_it_as_no_bearing(layout):
    # JSON cannot say NaN; the bodies say 1e999, which JSON.parse (and json.loads) read as
    # Infinity. The model treats any non-finite bearing as unknown, as it does NaN.
    model = AffectModel(load_config(SHIPPED))
    model.tick(0.0)
    for cfg in panels(layout, 'Publish').values():
        body = strict_json(cfg['value'])
        result = model.stimulus(body['kind'], body['intensity'], body['confidence'],
                                body['subject'], body['bearing_rad'], body['ttl_s'], 100.0)
        assert result.accepted, body['kind']
        if body['kind'] == 'face_known':
            assert result.state.focus_bearing == pytest.approx(0.8)
        else:
            assert math.isinf(body['bearing_rad']), body['kind']
            assert result.focus_updated == bool(body['subject']), body['kind']
            assert math.isnan(result.state.focus_bearing), body['kind']


def test_service_panels(layout):
    calls = {cfg['serviceName']: cfg for cfg in panels(layout, 'CallService').values()}
    assert sorted(calls) == ['/mood/reload_config', '/mood/set']
    request = strict_json(calls['/mood/set']['requestPayload'])
    assert request == {'mood': 'sleepy', 'lock': True, 'duration_s': 0, 'source': 'foxglove'}
    assert set(request) == SETMOOD_FIELDS
    assert strict_json(calls['/mood/reload_config']['requestPayload']) == {}
