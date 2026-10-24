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
Integration tests: ``sound_arbiter`` against the REAL ``sound_player`` (null backend).

``test_sound_node.py`` drives the arbiter against a fake player, which is the right tool for the
arbiter's own rules (claims, pre-emption, chatter, staleness, the offline path). The fake can
not show that the two halves agree, so these tests start ``ros2 run r2_sound sound_player``
with ``backend:=null`` over a temporary catalogue of synthetic WAVs (numpy and ``wave``, never
``/sounds``) and check what the pair does together: who owns a play, how it ends, the excluded
rule at 199, the duration cut, stale, and the graph (phase 3 plan rules 1 and 2).

Everything is test-private, as in ``test_sound_node.py``: the player runs as
``sound_player_t<tag>``, its dome level, ``/sound/level`` and ``/sound/playing`` are moved under
``/t<tag>``, and the arbiter's names are remapped as in the fake's tests.
"""

import os
import time
import wave

from astromech_msgs.srv import PlaySound
import numpy as np
import pytest
from test_nodes import Proc, ros_context, wait_until  # noqa: F401
from test_nodes import STATE_CLAIMED, STATE_IDLE, STATE_RUNNING, STATE_STALE
from test_sound_node import _names, OWNER, SoundRig
import yaml

RATE = 44100
STATE_PLAYING = 1
END_ENDED, END_DURATION, END_STOPPED = 1, 2, 3
CLIPS = (       # name, seconds, tags, excluded
    ('beep', 0.6, ['chatter'], False), ('longclip', 4.0, ['hum'], False),
    ('music', 1.5, ['excluded'], True))


def make_sounds(root):
    """
    Write the clips as constant-tone WAVs and a matching ``clips.yaml``; return both paths.

    They are built here because ``r2_expression`` has no test dependency on ``r2_sound``
    (whose ``test/fixtures.py`` has the same helpers).
    """
    sounds = os.path.join(str(root), 'sounds')
    os.makedirs(sounds, exist_ok=True)
    entries = []
    for name, seconds, tags, excluded in CLIPS:
        t = np.arange(int(seconds * RATE)) / RATE
        data = (0.25 * np.sin(2 * np.pi * 440.0 * t) * 32767).astype('<i2')
        with wave.open(os.path.join(sounds, name + '.wav'), 'wb') as f:
            f.setnchannels(1)
            f.setsampwidth(2)
            f.setframerate(RATE)
            f.writeframes(data.tobytes())
        entries.append({
            'name': name, 'set': 'Test', 'path': name + '.wav',
            'duration_ms': int(seconds * 1000), 'rms_dbfs': -16.0, 'peak_dbfs': -12.0,
            'clip_fraction': 0.0, 'set_gain_db': 0.0, 'gain_db': 0.0, 'tags': tags,
            'suggested_tag': tags[0], 'confidence': 1.0, 'excluded': excluded, 'label': ''})
    catalogue = os.path.join(str(root), 'clips.yaml')
    with open(catalogue, 'w') as f:
        yaml.safe_dump({
            'house_level_dbfs': -16.0, 'generated': '2026-10-24',
            'sets': {'Test': {'clips': len(entries), 'median_rms_dbfs': -16.0,
                              'set_gain_db': 0.0}},
            'clips': sorted(entries, key=lambda c: c['name'])}, f)
    return sounds, catalogue


class StackRig(SoundRig):
    """:class:`SoundRig` with the real player in place of the fake."""

    def __init__(self, tmp_path):
        self.sounds, self.catalogue = make_sounds(tmp_path)
        super().__init__(tmp_path)
        self.dome_topic = '/t%s/dome/audio/level' % self.tag

    def start_player(self, *extra):
        tag = '/t%s' % self.tag
        self.player = Proc([
            'ros2', 'run', 'r2_sound', 'sound_player', '--ros-args',
            '-r', '__node:=%s' % self.player_name,
            '-p', 'backend:=null', '-p', 'catalogue:=%s' % self.catalogue,
            '-p', 'sounds_dir:=%s' % self.sounds,
            '-p', 'dome_level_topic:=%s/dome/audio/level' % tag,
            '-r', '/sound/level:=%s/sound/level' % tag,
            '-r', '/sound/playing:=%s/sound/playing' % tag, *extra])
        return self.player

    def play_direct(self, name, priority):
        """Call the player's ``play`` as a bystander would (graph tests do not use this)."""
        client = self.node.create_client(PlaySound, '/%s/play' % self.player_name)
        wait_until(lambda: client.service_is_ready(), 10.0, 'play service', self.output)
        request = PlaySound.Request()
        request.name, request.priority, request.owner = name, priority, 'test'
        future = client.call_async(request)
        wait_until(future.done, 10.0, 'play reply', self.output)
        return future.result()

    def wait_idle(self, last_end, timeout=8.0):
        return wait_until(lambda: self.state is not None and self.state.state == 0 and
                          self.state.last_end == last_end and self.state, timeout,
                          'player idle with last_end %d' % last_end, self.output)


@pytest.fixture
def stack(tmp_path):
    made = []

    def make():
        made.append(StackRig(tmp_path))
        return made[-1]
    yield make
    for item in made:
        item.close()


def test_a_manual_play_runs_on_the_real_player_with_the_arbiters_owner_and_priority(stack):
    s = stack()
    s.start_mood('calm')
    s.intent('manual', 199, 'play', {'name': 'longclip'}, intent_id='m1')
    status = s.wait_status(lambda m: m.state == STATE_RUNNING, 8.0, 'running')
    assert (status.active_source, status.active_intent_id) == ('manual', 'm1')
    assert (status.owner, status.priority) == ('manual', 199)           # the auto-claim
    state = s.wait_playing('longclip')
    assert (state.owner, state.priority, state.intent_id) == (OWNER, 199, 'm1')
    assert state.backend == 'null'
    s.wait_status(lambda m: 0 < m.active_remaining_ms <= 4000, 4.0, 'remaining from the stream')


def test_a_natural_end_reports_last_end_1_and_the_channel_returns_to_idle(stack):
    s = stack()
    s.start_mood('calm')
    s.intent('manual', 199, 'play', {'name': 'beep'}, intent_id='m1')
    s.wait_playing('beep')
    s.wait_idle(END_ENDED)
    s.wait_status(lambda m: m.state in (STATE_CLAIMED, STATE_IDLE) and
                  m.active_remaining_ms == 0, 4.0, 'run over')
    s.wait_status(lambda m: m.state == STATE_IDLE, 10.0, 'idle after the manual hold')
    assert s.get_dropped() == 0


def test_after_a_manual_play_ends_the_channel_frees_then_behavior_and_chatter_run(stack):
    s = stack()
    s.start_mood('calm')
    s.intent('manual', 199, 'play', {'name': 'beep'}, intent_id='m1')
    s.wait_playing('beep')
    s.wait_idle(END_ENDED)
    s.wait_status(lambda m: m.state == STATE_IDLE, 10.0, 'idle after the manual hold')
    s.intent('behavior', 100, 'play', {'name': 'beep'}, intent_id='b1')
    state = s.wait_playing('beep')
    assert (state.priority, state.intent_id) == (100, 'b1') and s.get_dropped() == 0
    s.wait_idle(END_ENDED)
    s.start_mood('excited')                      # chatter every 1.5 s from the excited pools
    state = wait_until(lambda: s.state.state == STATE_PLAYING and s.state, 10.0,
                       'chatter on the real player', s.output)
    assert state.priority == 10 and state.intent_id.startswith('chatter-')


def test_a_manual_vocalize_ends_and_frees_the_channel(stack):
    s = stack()
    s.start_mood('calm')
    s.intent('manual', 199, 'vocalize', {'seed': 7, 'length_s': 0.8}, intent_id='v1')
    s.wait_playing('voice:7')
    s.wait_idle(END_ENDED)
    s.wait_status(lambda m: m.state == STATE_IDLE, 10.0, 'idle after the manual hold')


def test_an_excluded_clip_is_refused_below_199_and_accepted_at_199(stack):
    s = stack()
    s.start_mood('calm')
    low = s.play_direct('music', 198)                                    # the player's own rule
    assert not low.accepted and low.reason.startswith('excluded')
    s.intent('behavior', 100, 'play', {'name': 'music'}, intent_id='b1')   # the arbiter's rule
    time.sleep(1.0)
    assert s.state.state == 0 and s.state.name != 'music'
    s.intent('manual', 199, 'play', {'name': 'music'}, intent_id='m1')
    s.wait_playing('music')
    s.wait_idle(END_ENDED, 6.0)
    high = s.play_direct('music', 199)
    assert high.accepted and high.name == 'music'


def test_a_duration_cut_ends_the_play_early_with_last_end_2(stack):
    s = stack()
    s.start_mood('calm')
    s.intent('manual', 199, 'play', {'name': 'longclip'}, duration_ms=700, intent_id='m1')
    s.wait_playing('longclip')
    state = s.wait_idle(END_DURATION, 4.0)
    assert state.last_end == END_DURATION and state.elapsed_ms < 4000


def test_stale_stops_a_play_on_the_real_player(stack):
    s = stack()
    s.start_mood('calm')
    s.intent('idle', 10, 'play', {'name': 'longclip'}, intent_id='i1')
    s.wait_playing('longclip')
    s.stop_mood()
    s.wait_status(lambda m: m.state == STATE_STALE, 10.0, 'stale')
    s.wait_idle(END_STOPPED, 6.0)


def test_the_player_alone_publishes_the_dome_level_and_the_arbiter_alone_calls_it(stack):
    s = stack()
    node = s.node
    dev = '/%s' % s.player_name
    wait_until(lambda: node.get_publishers_info_by_topic(s.dome_topic), 10.0,
               'dome level publisher', s.output)
    pubs = [info.node_name for info in node.get_publishers_info_by_topic(s.dome_topic)]
    assert pubs == [s.player_name]
    controls = {dev + '/play', dev + '/vocalize', dev + '/stop'}
    arbiter = (s.arbiter_name, '/')
    wait_until(lambda: controls <= _names(node.get_client_names_and_types_by_node(*arbiter)),
               10.0, 'arbiter clients', s.output)
    users = [name for name, _ in node.get_node_names_and_namespaces()
             if name.endswith(s.tag)
             and controls & _names(node.get_client_names_and_types_by_node(name, '/'))]
    assert users == [s.arbiter_name]
