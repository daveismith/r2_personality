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

"""Run the express CLI against test-private topics and a fake claim service."""

import json
import subprocess
import time
import uuid

from astromech_msgs.msg import ChannelStatus, ExpressionIntent, SoundClip
from astromech_msgs.srv import ClaimChannel, ListSounds
from fixtures import shutdown_rclpy
import pytest
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    shutdown_rclpy()


@pytest.fixture
def harness():
    node = rclpy.create_node('test_express_pub_%s' % uuid.uuid4().hex[:6])
    tag = uuid.uuid4().hex[:8]
    h = type('H', (), {})()
    h.node, h.tag = node, tag
    h.intent_topic = '/test_express_%s/intent' % tag
    h.status_topic = '/test_express_%s/status' % tag
    h.claim_service = '/test_express_%s/claim_led' % tag
    h.claim_service_magic = '/test_express_%s/claim_magic' % tag
    h.claim_service_sound = '/test_express_%s/claim_sound' % tag
    h.player = '/test_express_%s_player' % tag
    h.console_topic = '/test_express_%s/mock/console' % tag
    h.calls, h.magic_calls, h.sound_calls, h.intents, h.consoles = [], [], [], [], []
    h.listed = 0
    h.accept, h.reason = True, 'held by someone'
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(ExpressionIntent, h.intent_topic, h.intents.append, qos)
    node.create_subscription(String, h.console_topic, h.consoles.append, qos)
    h.status_pub = node.create_publisher(ChannelStatus, h.status_topic, qos)

    def claim(request, response, calls=h.calls):
        calls.append(request)
        response.accepted = h.accept
        response.reason = '' if h.accept else h.reason
        response.status.channel = request.channel
        response.status.state = ChannelStatus.STATE_CLAIMED
        return response
    node.create_service(ClaimChannel, h.claim_service, claim)
    node.create_service(ClaimChannel, h.claim_service_magic,
                        lambda q, r: claim(q, r, h.magic_calls))
    node.create_service(ClaimChannel, h.claim_service_sound,
                        lambda q, r: claim(q, r, h.sound_calls))

    def list_sounds(request, response):
        h.listed += 1
        response.clips = [
            SoundClip(name='sent1', tags=['chatter'], duration_ms=1500, gain_db=-2.0),
            SoundClip(name='hum1', tags=['hum', 'calm'], duration_ms=4000),
            SoundClip(name='cantina', tags=[], duration_ms=120000, excluded=True)]
        response.catalogue, response.sample_rate, response.synth = 'fake', 44100, False
        return response
    node.create_service(ListSounds, h.player + '/list', list_sounds)
    yield h
    node.destroy_node()


def run(h, *argv, until=lambda: False, timeout=20.0):
    cmd = ['ros2', 'run', 'r2_expression', 'express', *argv, '--ros-args',
           '-r', '/expression/intent:=' + h.intent_topic,
           '-r', '/expression/status:=' + h.status_topic,
           '-r', '/expression/claim/led:=' + h.claim_service,
           '-r', '/expression/claim/magic:=' + h.claim_service_magic,
           '-r', '/expression/claim/sound:=' + h.claim_service_sound]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    end = time.monotonic() + timeout
    try:
        while time.monotonic() < end and proc.poll() is None:
            if until():
                h.status_pub.publish(ChannelStatus(channel='fld', state=1, owner='x'))
            rclpy.spin_once(h.node, timeout_sec=0.05)
        out, err = proc.communicate(timeout=5)
        return proc.returncode, out, err
    finally:
        if proc.poll() is None:
            proc.kill()


def accepted(out):
    # the zenoh scouting warnings share stdout, so look for the line
    return any(ln.startswith('accepted') for ln in out.splitlines())


def test_effect_publishes_a_manual_199_intent(harness):
    code, out, _ = run(harness, 'effect', 'fld', 'fire', '--rgb', 'ff2200', '--duration', '5',
                       '--id', 'abc')
    assert code == 0 and 'abc' in out, _
    (m,) = harness.intents
    assert (m.channel, m.source, m.priority, m.action) == ('fld', 'manual', 199, 'effect')
    assert json.loads(m.params) == {'name': 'fire', 'rgb': 0xff2200}
    assert (m.duration_ms, m.interruptible, m.intent_id) == (5000, True, 'abc')
    assert m.header.stamp.sec > 0


def test_override_and_hold(harness):
    code, _, _ = run(harness, 'stop', 'rld', '--override', '--hold')
    assert code == 0
    (m,) = harness.intents
    assert (m.priority, m.action, m.params, m.interruptible) == (200, 'stop', '', False)


def test_claim_accepted_and_refused_exit_codes(harness):
    code, out, _ = run(harness, 'claim', 'fld', '--owner', 'me', '--lease', '12')
    assert code == 0 and accepted(out)
    (r,) = harness.calls
    assert (r.channel, r.owner, r.priority, r.lease_s) == ('fld', 'me', 199, 12.0)
    harness.accept = False
    code, out, _ = run(harness, 'release', 'fld', '--owner', 'me')
    assert code == 1 and 'held by someone' in out
    assert harness.calls[-1].lease_s == 0.0


def test_release_by_a_non_holder_exits_1_and_prints_the_real_holder(harness):
    harness.accept, harness.reason = False, "no claim by 'express' on fld"
    code, out, _ = run(harness, 'release', 'fld')
    assert code == 1
    line = [ln for ln in out.splitlines() if ln.startswith('refused')]
    assert line and "no claim by 'express' on fld" in line[0]
    assert line[0].split(': ', 2)[2].startswith('fld')       # then the status line
    assert harness.calls[-1].owner == 'express' and harness.calls[-1].lease_s == 0.0


def test_magic_claim_and_release_use_the_magic_service(harness):
    code, out, _ = run(harness, 'claim', 'magic', '--owner', 'me')
    assert code == 0 and accepted(out)
    code, _, _ = run(harness, 'release', 'magic', '--owner', 'me')
    assert code == 0
    assert [(r.channel, r.lease_s) for r in harness.magic_calls] == [
        ('magic', 30.0), ('magic', 0.0)]
    assert harness.calls == []


def test_bad_arguments_exit_2_and_publish_nothing(harness):
    code, _, err = run(harness, 'effect', 'fld', 'fire', '--speed', '12', timeout=10)
    assert code == 2 and 'speed' in err
    assert harness.intents == []


def test_status_prints_one_line_per_channel(harness):
    code, out, _ = run(harness, 'status', until=lambda: True)
    assert code == 0
    # the zenoh router's own warnings share stdout, so look for the line
    assert [ln for ln in out.splitlines() if ln.startswith('fld   running')]


def test_dome_publishes_a_console_line_on_a_private_device(harness):
    device = '/test_express_%s' % harness.tag
    code, _, _ = run(harness, 'dome', 'leds run fld fire', '--device', device)
    assert code == 0
    assert [m.data for m in harness.consoles] == ['leds run fld fire']


def test_look_publishes_a_manual_199_dome_look_at(harness):
    code, out, err = run(harness, 'look', '-0.8', '--profile', 'snappy', '--id', 'lk')
    assert code == 0, err
    (m,) = harness.intents
    assert (m.channel, m.source, m.priority, m.action) == ('dome', 'manual', 199, 'look_at')
    assert json.loads(m.params) == {'bearing': -0.8, 'profile': 'snappy'}
    assert (m.duration_ms, m.interruptible, m.intent_id) == (0, True, 'lk')


def test_a_sequence_goes_out_with_end_blank_by_default(harness):
    code, _, _ = run(harness, 'sequence', 'magic', '44', '--duration', '3')
    assert code == 0
    (m,) = harness.intents
    assert json.loads(m.params) == {'id': 44, 'repeat': 1, 'end': 'blank'}


def test_no_listener_exits_1(harness):
    code, _, err = run(harness, 'stop', 'fld', '--ros-args', '-r',
                       '/expression/intent:=/test_express_nobody_%s' % harness.tag)
    assert code == 1 and 'nobody is listening' in err


def test_play_publishes_a_manual_199_sound_play(harness):
    code, out, err = run(harness, 'play', 'scream', '--gain', '-3', '--duration', '2',
                         '--id', 'pl')
    assert code == 0, err
    (m,) = harness.intents
    assert (m.channel, m.source, m.priority, m.action) == ('sound', 'manual', 199, 'play')
    assert json.loads(m.params) == {'name': 'scream', 'gain_db': -3.0}
    assert (m.duration_ms, m.interruptible, m.intent_id) == (2000, True, 'pl')


def test_play_pool_vocalize_and_stop_sound(harness):
    assert run(harness, 'play', '--pool', 'excited')[0] == 0
    assert run(harness, 'vocalize', '--seed', '7', '--length', '1.5')[0] == 0
    assert run(harness, 'stop', 'sound', '--override')[0] == 0
    pool, voice, stop = harness.intents
    assert (pool.action, json.loads(pool.params)) == ('play', {'pool': 'excited'})
    assert voice.action == 'vocalize'
    assert json.loads(voice.params) == {'length_s': 1.5, 'seed': 7}
    assert (stop.channel, stop.action, stop.priority) == ('sound', 'stop', 200)


def test_sound_claim_and_release_use_the_sound_service_only(harness):
    code, out, _ = run(harness, 'claim', 'sound', '--owner', 'me')
    assert code == 0 and accepted(out)
    assert run(harness, 'release', 'sound', '--owner', 'me')[0] == 0
    assert [(r.channel, r.owner, r.lease_s) for r in harness.sound_calls] == [
        ('sound', 'me', 30.0), ('sound', 'me', 0.0)]
    assert harness.calls == [] and harness.magic_calls == []


def test_sounds_lists_the_catalogue_through_the_players_list_service_and_plays_nothing(harness):
    code, out, err = run(harness, 'sounds', '--device', harness.player)
    assert code == 0, err
    lines = [ln for ln in out.splitlines() if ln.startswith(('sent1', 'hum1', 'cantina', '3 of'))]
    assert len(lines) == 4 and 'manual only' in lines[2]
    assert '3 of 3 clips; catalogue fake, 44100 Hz, synth no' in lines[3]
    code, out, _ = run(harness, 'sounds', '--device', harness.player, '--pool', 'hum')
    assert code == 0 and '1 of 3 clips tagged hum' in out and 'sent1' not in out
    assert harness.listed == 2 and harness.intents == []


def test_sounds_without_a_player_exits_1(harness):
    code, _, err = run(harness, 'sounds', '--device', harness.player + '_gone', timeout=30)
    assert code == 1 and 'not available' in err
