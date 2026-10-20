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
Integration tests: ``dome_mock`` as a subprocess, driven over ROS 2 services and topics.

Every name is test-private: the node is renamed (``-r __node:=dome_mock_t<pid>_<n>``, which moves
its services and state topics) and the absolute ``/dome/...`` topics are remapped, so a real
``/r2_dome`` and ``/dome/*`` on the graph are never touched.
"""

import os
import signal
import subprocess
import threading
import time

from astromech_msgs.msg import LedStateArray
from astromech_msgs.msg import MagicPanelState
from astromech_msgs.srv import ConfigureLedDisplay
from astromech_msgs.srv import ConfigureMagicPanel
from astromech_msgs.srv import ListLeds
from astromech_msgs.srv import ListMagicPanel
from astromech_msgs.srv import RunLedEffect
from astromech_msgs.srv import SayLedText
from astromech_msgs.srv import StartMagicSequence
from astromech_msgs.srv import StopLedEffect
from astromech_msgs.srv import StopMagicSequence
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import UInt8

CALL_TIMEOUT_S = 5.0
_counter = 0


class Harness:
    """A running dome_mock plus a test-side rclpy node."""

    def __init__(self, extra_args=()):
        global _counter
        _counter += 1
        tag = '%d_%d' % (os.getpid(), _counter)
        self.name = 'dome_mock_t%s' % tag
        self.text_topics = ['/%s/dome/text/line%d' % (self.name, i) for i in (0, 1)]
        self.level_topic = '/%s/dome/audio/level' % self.name
        remaps = ['-r', '__node:=%s' % self.name]
        self.proc = subprocess.Popen(
            ['ros2', 'run', 'r2_mocks', 'dome_mock', '--ros-args', '-p', 'state_hz:=10.0',
             *extra_args, *remaps],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            start_new_session=True)
        self.output = []
        threading.Thread(target=self._drain, daemon=True).start()

        self.node = rclpy.create_node('dome_mock_test_%s' % tag)
        self.latest = {}
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        for key, msg_type, topic in (
                ('leds', LedStateArray, '/%s/leds/states' % self.name),
                ('magic', MagicPanelState, '/%s/magic/states' % self.name),
                ('picture', String, '/%s/mock/picture' % self.name),
                ('image', Image, '/%s/mock/image' % self.name)):
            self.node.create_subscription(
                msg_type, topic, lambda msg, key=key: self.latest.__setitem__(key, msg),
                reliable)
        self.console = self.node.create_publisher(String, '/%s/mock/console' % self.name, reliable)
        self.text_pubs = [self.node.create_publisher(String, t, reliable)
                          for t in self.text_topics]
        self.level_pub = self.node.create_publisher(UInt8, self.level_topic, reliable)
        self.clients = {}
        for key, srv in (('leds/list', ListLeds), ('leds/run', RunLedEffect),
                         ('leds/say', SayLedText), ('leds/stop', StopLedEffect),
                         ('leds/configure', ConfigureLedDisplay),
                         ('magic/list', ListMagicPanel), ('magic/start', StartMagicSequence),
                         ('magic/stop', StopMagicSequence),
                         ('magic/configure', ConfigureMagicPanel)):
            self.clients[key] = self.node.create_client(srv, '/%s/%s' % (self.name, key))
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()
        for client in self.clients.values():
            assert client.wait_for_service(timeout_sec=30.0), ''.join(self.output)
        # A message published before the node's subscription is matched is lost.
        for pub in (self.console, self.level_pub, *self.text_pubs):
            assert self._wait(lambda pub=pub: pub.get_subscription_count() > 0, 30.0), \
                ''.join(self.output)

    def _drain(self):
        for line in self.proc.stdout:
            self.output.append(line)

    def close(self):
        """Stop the node (SIGINT, as a user would) and return its exit code."""
        code = None
        try:
            os.killpg(self.proc.pid, signal.SIGINT)
            code = self.proc.wait(timeout=10)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass
        try:
            os.killpg(self.proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.executor.shutdown()
        self.node.destroy_node()
        return code

    def call(self, key, request, timeout=CALL_TIMEOUT_S):
        """Call a service and return its response."""
        future = self.clients[key].call_async(request)
        assert self._wait(lambda: future.done(), timeout), 'no reply from %s' % key
        return future.result()

    @staticmethod
    def _wait(predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.02)
        return False

    def wait_for(self, key, predicate, timeout=10.0):
        """Return the first state message of ``key`` after now that satisfies ``predicate``."""
        found = []

        def check():
            msg = self.latest.get(key)
            if msg is not None and predicate(msg):
                found.append(msg)
            return bool(found)
        assert self._wait(check, timeout), 'timed out on %s\n%s' % (key, ''.join(self.output))
        return found[0]

    def say(self, line):
        """Send a console line and give the node a moment to take it."""
        self.console.publish(String(data=line))


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def mock():
    harness = Harness()
    yield harness
    harness.close()


def led(msg, kind, name):
    return next(x for x in getattr(msg, kind) if x.name == name)


def test_every_service_once_with_owner_and_priority(mock):
    listing = mock.call('leds/list', ListLeds.Request())
    assert [d.name for d in listing.displays] == ['rpsi', 'rld', 'bfld', 'tfld', 'fpsi']
    assert len(listing.surfaces) == 6 and len(listing.effects) == 14
    assert len(listing.effects) == len(listing.effect_summaries) and len(listing.clips) == 48
    assert list(listing.drift_presets) == ['calm', 'lively']

    run = mock.call('leds/run', RunLedEffect.Request(
        surface='fld', effect='fire', duration_ms=0, owner='r2_expression', priority=10))
    assert (run.accepted, run.reason) == (True, '')
    state = mock.wait_for('leds', lambda m: led(m, 'surfaces', 'fld').running)
    fld = led(state, 'surfaces', 'fld')
    assert (fld.effect, fld.source, fld.owner) == ('fire', 'ros', 'r2_expression')
    assert led(state, 'displays', 'tfld').mode == 5

    say = mock.call('leds/say', SayLedText.Request(
        surface='fld', lines=['hi', 'there'], motion=0, owner='r2_expression', priority=10))
    assert say.accepted
    assert led(mock.wait_for('leds', lambda m: led(m, 'surfaces', 'fld').effect == 'text'),
               'surfaces', 'fld').last_effect == 'fire'

    cfg = mock.call('leds/configure', ConfigureLedDisplay.Request(
        display='tfld', apply=ConfigureLedDisplay.Request.APPLY_BRIGHTNESS, brightness=55,
        owner='r2_expression', priority=10))
    assert cfg.accepted and cfg.result.name == 'tfld' and cfg.result.brightness == 55
    state = mock.wait_for('leds', lambda m: led(m, 'displays', 'tfld').brightness == 55)
    assert led(state, 'displays', 'tfld').changed_by == 'ros'
    assert led(state, 'displays', 'tfld').changed_by_owner == 'r2_expression'

    stop = mock.call('leds/stop', StopLedEffect.Request(surface='fld', priority=10))
    assert stop.accepted and list(stop.stopped) == ['fld']
    state = mock.wait_for('leds', lambda m: not led(m, 'surfaces', 'fld').running)
    assert led(state, 'surfaces', 'fld').last_end == 'stopped'

    magic = mock.call('magic/list', ListMagicPanel.Request())
    assert magic.panel.link == 1 and len(magic.sequences) == 58
    start = mock.call('magic/start', StartMagicSequence.Request(
        sequence_id=44, repeat=1, end=0, owner='r2_expression', priority=10))
    assert (start.accepted, start.confirmed, start.run_counter) == (True, True, 1)
    state = mock.wait_for('magic', lambda m: m.sequence_id == 44)
    assert (state.started_by, state.owner, state.ours, state.state, state.run_counter) == (
        'ros', 'r2_expression', True, MagicPanelState.STATE_RUNNING, 1)
    assert state.remaining_known and state.link == 1
    stop = mock.call('magic/stop', StopMagicSequence.Request(mode=0, priority=10))
    assert stop.accepted
    state = mock.wait_for('magic', lambda m: m.state == MagicPanelState.STATE_STOPPED)
    assert state.owner == 'r2_expression'
    conf = mock.call('magic/configure', ConfigureMagicPanel.Request(
        apply=ConfigureMagicPanel.Request.APPLY_BRIGHTNESS, brightness=4))
    assert conf.accepted and conf.result.brightness == 4
    refused = mock.call('magic/start', StartMagicSequence.Request(sequence_id=200, repeat=1))
    assert (refused.accepted, refused.reason) == (False, 'unknown sequence')


def test_console_writes_hold_and_ros_priority_is_refused(mock):
    mock.say('leds run fld fire -t 30')
    state = mock.wait_for('leds', lambda m: led(m, 'displays', 'tfld').held_by == 'console')
    fld = led(state, 'surfaces', 'fld')
    assert (fld.source, fld.owner, fld.effect) == ('console', '', 'fire')
    assert 0 < led(state, 'displays', 'tfld').hold_remaining_ms <= 5000
    assert led(state, 'displays', 'rld').held_by == ''

    refused = mock.call('leds/run', RunLedEffect.Request(
        surface='fld', effect='rainbow', owner='r2_expression', priority=10))
    assert (refused.accepted, refused.reason) == (False, 'held by console')
    override = mock.call('leds/run', RunLedEffect.Request(
        surface='fld', effect='rainbow', owner='r2_expression', priority=200))
    assert override.accepted
    unset = mock.call('leds/run', RunLedEffect.Request(surface='fld', effect='roam'))
    assert unset.accepted
    # the hold lapses after hold_ms, and a ROS write is then accepted again
    assert mock._wait(lambda: mock.call('leds/run', RunLedEffect.Request(
        surface='fld', effect='fire', owner='x', priority=10)).accepted, 8.0)

    mock.say('ui leds brightness rld 20')
    state = mock.wait_for('leds', lambda m: led(m, 'displays', 'rld').changed_by == 'ui')
    assert led(state, 'displays', 'rld').held_by == 'ui'


def test_sticky_manual_and_strip_enable_reach_the_state_topics(mock):
    mock.say('leds owner manual')
    state = mock.wait_for('leds', lambda m: m.manual)
    assert led(state, 'displays', 'rld').held_by == 'manual'
    assert led(state, 'displays', 'rld').hold_remaining_ms == 0xFFFFFFFF
    magic = mock.wait_for('magic', lambda m: m.manual)
    assert (magic.held_by, magic.hold_remaining_ms) == ('manual', 0xFFFFFFFF)
    refused = mock.call('magic/start', StartMagicSequence.Request(
        sequence_id=3, repeat=1, priority=10))
    assert refused.reason == 'held: manual'
    mock.say('leds owner auto')
    mock.wait_for('leds', lambda m: not m.manual)
    mock.say('leds enable off')
    mock.wait_for('leds', lambda m: not m.strip_enabled)
    dark = mock.call('leds/run', RunLedEffect.Request(surface='fld', effect='fire', priority=10))
    assert (dark.accepted, dark.reason) == (True, '')
    state = mock.wait_for('leds', lambda m: led(m, 'surfaces', 'fld').running)
    assert state.strip_enabled is False


def test_magic_console_and_rotary_attribution(mock):
    mock.say('magic_panel start Smile')
    state = mock.wait_for('magic', lambda m: m.sequence_id == 44 and m.started_by == 'console')
    assert (state.owner, state.ours, state.held_by) == ('', True, 'console')
    mock.say('rotary 3')
    state = mock.wait_for('magic', lambda m: m.takeovers == 1)
    assert (state.source, state.started_by, state.ours, state.sequence_id,
            state.rotary_code) == (3, 'other', False, 3, 3)
    mock.say('magic_panel start 56 -r 0')
    mock.say('magic_panel stop freeze')
    mock.wait_for('magic', lambda m: m.state == MagicPanelState.STATE_STOPPED)


def test_text_topic_level_and_rostext(mock):
    mock.text_pubs[0].publish(String(data='hello'))
    state = mock.wait_for('leds', lambda m: led(m, 'surfaces', 'fld').source == 'text_topic')
    fld = led(state, 'surfaces', 'fld')
    assert fld.effect == 'text' and led(state, 'displays', 'bfld').held_by == 'text_topic'
    mock.text_pubs[0].publish(String(data=''))
    mock.wait_for('leds', lambda m: not led(m, 'surfaces', 'fld').running)
    mock.level_pub.publish(UInt8(data=123))
    state = mock.wait_for('leds', lambda m: m.level_set)
    assert state.level == 123
    mock.say('leds rostext off')
    time.sleep(1.0)
    mock.text_pubs[1].publish(String(data='ignored'))
    time.sleep(1.0)
    assert not led(mock.latest['leds'], 'surfaces', 'fld').running


def test_picture_and_image_publish(mock):
    picture = mock.wait_for('picture', lambda m: 'surface fld' in m.data)
    assert picture.data.count('\nsurface') + 1 == 6 and 'display tfld: mode=random' in picture.data
    first = mock.wait_for('image', lambda m: m.height > 0)
    assert (first.encoding, first.step, len(first.data)) == (
        'rgb8', first.width * 3, first.height * first.width * 3)
    mock.say('leds mode all off')
    changed = mock.wait_for('image', lambda m: bytes(m.data) != bytes(first.data))
    assert changed.height == first.height
    mock.wait_for('picture', lambda m: 'display tfld: mode=off' in m.data)


def test_request_too_long_gets_no_reply_within_the_client_timeout(mock):
    future = mock.clients['leds/run'].call_async(
        RunLedEffect.Request(surface='x' * 97, effect='fire', priority=10))
    assert not mock._wait(lambda: future.done(), 5.0)
    assert mock._wait(lambda: future.done(), 3.0)
    assert future.result().accepted is False
    assert 'WARN' in ''.join(mock.output)
    # the node still answers, and nothing ran
    assert mock.call('leds/run', RunLedEffect.Request(surface='x' * 96, effect='fire')).reason \
        == 'unknown surface'
    long_line = mock.clients['leds/say'].call_async(
        SayLedText.Request(surface='fld', lines=['y' * 129]))
    assert not mock._wait(lambda: long_line.done(), 5.0)


def test_exits_cleanly_on_sigint():
    harness = Harness()
    assert harness.close() == 0
