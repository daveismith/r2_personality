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
Integration tests: both arbiters against ``dome_mock``, each as a subprocess.

Everything is test-private. The mock runs as ``dome_mock_t<pid>_<n>`` (which moves its services
and state topics) with its ``/dome/...`` topics remapped; the arbiters take ``device`` of that
name and every ``/expression/*`` and ``/mood/state`` name is remapped. The real ``/r2_dome``
and ``/dome/*`` on the graph are never touched: no test creates a client for ``/r2_dome/*`` or
publishes to ``/dome/*``, and the arbiters' ``device`` is always passed.
"""

import copy
import json
import os
import signal
import subprocess
import threading
import time

from astromech_msgs.msg import ChannelStatus
from astromech_msgs.msg import ExpressionIntent
from astromech_msgs.msg import LedStateArray
from astromech_msgs.msg import MagicPanelState
from astromech_msgs.msg import MoodState
from astromech_msgs.srv import ClaimChannel
from fixtures import BASE, shutdown_rclpy
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String
import yaml

STARTUP_S = 40.0
LED_CHANNELS = ('fld', 'rld', 'psi')
CHANNELS = {'led': LED_CHANNELS, 'magic': ('magic',)}
STATE_IDLE, STATE_RUNNING, STATE_CLAIMED, STATE_FOREIGN, STATE_STALE, STATE_OFFLINE, \
    STATE_DARK = range(7)
_counter = 0


def config_text(flourish=None, data=None, **defaults):
    """
    Return the YAML text of a config with short staleness windows and no flourishes.

    ``flourish`` (a dict) is put on the ``calm`` mood; ``data`` replaces BASE.
    """
    data = copy.deepcopy(BASE if data is None else data)
    data['defaults'].update(mood_stale_s=1.0, device_stale_s=2.0, status_hz=5.0)
    data['defaults'].update(defaults)
    for mood in data['moods'].values():
        mood.pop('flourish', None)
    if flourish:
        data['moods']['calm']['flourish'] = flourish
    return yaml.safe_dump(data)


class Proc:
    """A subprocess in its own session, its output collected."""

    def __init__(self, argv):
        self.proc = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, start_new_session=True)
        self.output = []
        threading.Thread(target=self._drain, daemon=True).start()

    def _drain(self):
        for line in self.proc.stdout:
            self.output.append(line)

    def text(self):
        return ''.join(self.output)

    def send(self, sig):
        try:
            os.killpg(self.proc.pid, sig)
        except ProcessLookupError:
            pass

    def close(self, sig=signal.SIGINT):
        """Stop the process as a user would; return its exit code (None if it had to die)."""
        code = None
        self.send(signal.SIGCONT)
        self.send(sig)
        try:
            code = self.proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            pass
        self.send(signal.SIGKILL)
        return code


def wait_until(predicate, timeout, what='condition', output=None):
    """Return the first truthy value of ``predicate()`` within ``timeout`` seconds."""
    deadline = time.monotonic() + timeout
    while True:
        value = predicate()
        if value:
            return value
        if time.monotonic() >= deadline:
            raise AssertionError('timed out on %s\n%s' % (what, output() if output else ''))
        time.sleep(0.02)


class Stack:
    """The mock, optionally the arbiters, and a test-side node, all with private names."""

    def __init__(self, tmp_path, kinds=('led', 'magic'), config=None, mock_args=(),
                 start_mock=True, arbiter_args=(), start_arbiters=True):
        global _counter
        _counter += 1
        self.tag = '%d_%d' % (os.getpid(), _counter)
        self.mock_name = 'dome_mock_t%s' % self.tag
        self.kinds = kinds
        self.config = config
        if config is None:
            self.config = str(tmp_path / 'expression.yaml')
            (tmp_path / 'expression.yaml').write_text(config_text())
        self.config_args = ['-p', 'config:=%s' % self.config] if config is not False else []
        self.arbiter_args = arbiter_args
        self.topics = {name: '/t%s%s' % (self.tag, name) for name in (
            '/expression/intent', '/mood/state', '/expression/status',
            '/expression/claim/led', '/expression/claim/magic')}
        self.mock_args = mock_args
        self.mock = Proc(self._mock_argv()) if start_mock else None
        self.arbiters = {}
        self.node = rclpy.create_node('arbiter_test_%s' % self.tag)
        self.lock = threading.Lock()
        self.status = {}
        self.history = []
        self.latest = {}
        reliable = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        sub = self.node.create_subscription
        sub(ChannelStatus, self.topics['/expression/status'], self._on_status, reliable)
        sub(LedStateArray, '/%s/leds/states' % self.mock_name,
            lambda m: self.latest.__setitem__('leds', m), reliable)
        sub(MagicPanelState, '/%s/magic/states' % self.mock_name,
            lambda m: self.latest.__setitem__('magic', m), reliable)
        self.intents = self.node.create_publisher(
            ExpressionIntent, self.topics['/expression/intent'], reliable)
        self.mood_pub = self.node.create_publisher(
            MoodState, self.topics['/mood/state'], reliable)
        self.console = self.node.create_publisher(
            String, '/%s/mock/console' % self.mock_name, reliable)
        self.claims = {kind: self.node.create_client(
            ClaimChannel, self.topics['/expression/claim/%s' % kind]) for kind in kinds}
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()
        self._mood = None
        self._mood_stop = threading.Event()
        self._mood_thread = None
        if start_arbiters:
            for kind in kinds:
                self.start_arbiter(kind)
            if start_mock:
                self.wait_ready()

    # --- processes ---------------------------------------------------------------------------

    def _mock_argv(self):
        return ['ros2', 'run', 'r2_mocks', 'dome_mock', '--ros-args', '-p', 'state_hz:=10.0',
                *self.mock_args, '-r', '__node:=%s' % self.mock_name]

    def start_mock(self):
        self.mock = Proc(self._mock_argv())

    def arbiter_name(self, kind):
        return '%s_arbiter_t%s' % (kind, self.tag)

    def arbiter_argv(self, kind, extra=()):
        remaps = []
        for name, target in self.topics.items():
            remaps += ['-r', '%s:=%s' % (name, target)]
        return [
            'ros2', 'run', 'r2_expression', '%s_arbiter' % kind, '--ros-args',
            '-r', '__node:=%s' % self.arbiter_name(kind), '-p', 'device:=/%s' % self.mock_name,
            *self.config_args, *self.arbiter_args, *extra, *remaps]

    def start_arbiter(self, kind, extra=()):
        self.arbiters[kind] = Proc(self.arbiter_argv(kind, extra))
        return self.arbiters[kind]

    def close(self):
        self.stop_mood()
        codes = {kind: proc.close() for kind, proc in self.arbiters.items()}
        if self.mock is not None:
            self.mock.close()
        self.executor.shutdown()
        self.node.destroy_node()
        return codes

    def output(self):
        parts = ['--- mock ---\n' + self.mock.text()] if self.mock else []
        parts += ['--- %s ---\n%s' % (kind, proc.text()) for kind, proc in self.arbiters.items()]
        return '\n'.join(parts)

    # --- test-side topics --------------------------------------------------------------------

    def _on_status(self, msg):
        with self.lock:
            self.status[msg.channel] = msg
            self.history.append((time.monotonic(), msg))

    def wait_ready(self):
        """Wait until every channel of every started arbiter is on the device (not OFFLINE)."""
        for kind in self.arbiters:
            for ch in CHANNELS[kind]:
                self.wait_status(ch, lambda s: s.state != STATE_OFFLINE, STARTUP_S,
                                 'ready %s' % ch)
        for pub in (self.intents, self.mood_pub):
            wait_until(lambda pub=pub: pub.get_subscription_count() >= len(self.arbiters), 30.0,
                       'subscriptions matched', self.output)
        wait_until(lambda: 'leds' in self.latest or 'led' not in self.kinds, 30.0, 'led state',
                   self.output)

    def wait_status(self, channel, predicate, timeout=8.0, what=''):
        def check():
            with self.lock:
                msg = self.status.get(channel)
            return msg if msg is not None and predicate(msg) else None
        return wait_until(check, timeout, '%s status %s' % (channel, what), self.output)

    def get_status(self, channel):
        with self.lock:
            return self.status.get(channel)

    def start_mood(self, label, strength=1.0):
        """Publish ``/mood/state`` at 5 Hz with ``label`` until :meth:`stop_mood`."""
        self._mood = (label, strength)
        if self._mood_thread is None or not self._mood_thread.is_alive():
            self._mood_stop.clear()
            self._mood_thread = threading.Thread(target=self._mood_loop, daemon=True)
            self._mood_thread.start()
        for kind in self.arbiters:
            for ch in CHANNELS[kind]:
                self.wait_status(ch, lambda m: m.mood == label, 10.0, 'mood %s' % label)

    def _mood_loop(self):
        while not self._mood_stop.wait(0.2):
            msg = MoodState()
            msg.mood, msg.mood_strength = self._mood
            self.mood_pub.publish(msg)

    def stop_mood(self):
        self._mood_stop.set()
        if self._mood_thread is not None:
            self._mood_thread.join(timeout=2.0)

    def intent(self, channel, source, priority, action, params=None, duration_ms=0,
               interruptible=True, intent_id=''):
        msg = ExpressionIntent()
        msg.channel, msg.source, msg.priority, msg.action = channel, source, priority, action
        msg.params = json.dumps(params) if params is not None else ''
        msg.duration_ms, msg.interruptible, msg.intent_id = duration_ms, interruptible, intent_id
        self.intents.publish(msg)

    def say(self, line):
        """Play the operator at the dome's console."""
        self.console.publish(String(data=line))

    def claim(self, kind, channel, owner='op', priority=150, lease_s=10.0):
        request = ClaimChannel.Request()
        request.channel, request.owner = channel, owner
        request.priority, request.lease_s = priority, lease_s
        future = self.claims[kind].call_async(request)
        wait_until(future.done, 10.0, 'claim reply', self.output)
        return future.result()

    # --- the mock's state --------------------------------------------------------------------

    def display(self, name):
        return next(d for d in self.latest['leds'].displays if d.name == name)

    def surface(self, name):
        return next(s for s in self.latest['leds'].surfaces if s.name == name)

    def wait_led(self, predicate, timeout=8.0, what='led state'):
        def check():
            msg = self.latest.get('leds')
            return msg if msg is not None and predicate(msg) else None
        return wait_until(check, timeout, what, self.output)


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    shutdown_rclpy()


@pytest.fixture
def stack(tmp_path):
    made = []

    def make(**kwargs):
        made.append(Stack(tmp_path, **kwargs))
        return made[-1]
    yield make
    for item in made:
        item.close()


# --- an intent runs and ends ------------------------------------------------------------------

def test_idle_intent_runs_and_ends_by_its_duration(stack):
    s = stack(kinds=('led',))
    s.start_mood('calm')
    s.intent('fld', 'idle', 10, 'effect', {'name': 'fire'}, duration_ms=1500,
             intent_id='idle-fire')
    status = s.wait_status('fld', lambda m: m.state == STATE_RUNNING)
    assert (status.active_source, status.active_action, status.active_intent_id) == (
        'idle', 'effect', 'idle-fire')
    surface = s.wait_led(lambda m: any(x.name == 'fld' and x.running for x in m.surfaces))
    fld = next(x for x in surface.surfaces if x.name == 'fld')
    assert (fld.effect, fld.source, fld.owner) == ('fire', 'ros', 'r2_expression')
    assert 0 < fld.duration_ms <= 1500
    s.wait_status('fld', lambda m: m.state == STATE_IDLE, 6.0, 'idle after the duration')
    s.wait_led(lambda m: not next(x for x in m.surfaces if x.name == 'fld').running)
    assert s.surface('fld').last_end == 'duration'


def test_manual_intent_preempts_and_holds_the_channel_for_five_seconds(stack):
    s = stack(kinds=('led',))
    s.start_mood('calm')
    s.intent('fld', 'idle', 10, 'effect', {'name': 'rainbow'}, duration_ms=20000)
    s.wait_status('fld', lambda m: m.active_source == 'idle')
    s.intent('fld', 'manual', 199, 'effect', {'name': 'fire'}, duration_ms=1000,
             intent_id='manual-fire')
    s.wait_status('fld', lambda m: m.active_source == 'manual', what='manual running')
    s.wait_led(lambda m: next(x for x in m.surfaces if x.name == 'fld').effect == 'fire')
    assert s.surface('fld').last_end == 'replaced'
    held = s.wait_status('fld', lambda m: m.state == STATE_CLAIMED, 6.0, 'claimed after run')
    assert (held.owner, held.priority) == ('manual', 199)
    assert 3.0 < held.lease_remaining_s <= 5.0
    dropped = held.dropped
    s.intent('fld', 'idle', 10, 'effect', {'name': 'rainbow'}, duration_ms=2000)
    assert s.wait_status('fld', lambda m: m.dropped == dropped + 1).state == STATE_CLAIMED
    assert not s.surface('fld').running


def test_manual_hold_ends_and_the_channel_is_idle_again(stack, tmp_path):
    config = tmp_path / 'fast.yaml'
    config.write_text(config_text(manual_hold_s=1.0))
    s = stack(kinds=('led',), config=str(config))
    s.start_mood('calm')
    s.intent('rld', 'manual', 199, 'effect', {'name': 'fire'}, duration_ms=500)
    s.wait_status('rld', lambda m: m.state == STATE_CLAIMED, 6.0, 'claimed')
    s.wait_status('rld', lambda m: m.state == STATE_IDLE, 6.0, 'idle after the hold')


def test_a_claim_refuses_a_lower_intent_and_counts_it(stack):
    s = stack(kinds=('led',))
    s.start_mood('calm')
    reply = s.claim('led', 'fld', 'op', 150, 10.0)
    assert reply.accepted and reply.status.state == STATE_CLAIMED
    assert (reply.status.owner, reply.status.priority) == ('op', 150)
    s.wait_status('fld', lambda m: m.state == STATE_CLAIMED)
    s.intent('fld', 'idle', 10, 'effect', {'name': 'fire'}, duration_ms=2000)
    s.intent('fld', 'behavior', 100, 'effect', {'name': 'fire'}, duration_ms=2000)
    status = s.wait_status('fld', lambda m: m.dropped == 2, what='two drops counted')
    assert status.state == STATE_CLAIMED
    assert not s.surface('fld').running
    other = s.claim('led', 'fld', 'someone', 120, 10.0)
    assert not other.accepted and "'op'" in other.reason
    assert s.claim('led', 'fld', 'op', 150, 0.0).accepted
    s.wait_status('fld', lambda m: m.state == STATE_IDLE, what='released')


# --- a person at the dome ---------------------------------------------------------------------

def test_a_console_run_makes_the_channel_foreign_and_the_arbiter_stops_reconfiguring(stack):
    s = stack(kinds=('led',), mock_args=('-p', 'hold_ms:=1500.0'))
    s.start_mood('calm')
    s.wait_led(lambda m: s.display('tfld').changed_by_owner == 'r2_expression')
    assert s.display('tfld').brightness == 60
    s.say('leds run fld fire -t 3')
    status = s.wait_status('fld', lambda m: m.state == STATE_FOREIGN, 3.0, 'foreign')
    assert status.owner == 'console'
    assert 0.0 <= status.lease_remaining_s <= 1.5
    s.start_mood('excited')                            # would raise brightness to 100
    s.wait_status('fld', lambda m: m.mood == 'excited')
    time.sleep(1.0)
    assert s.display('tfld').brightness == 60          # nothing was sent to the held channel
    assert s.surface('fld').source == 'console'
    # the person's run outlives the 1.5 s hold: still foreign, lease 0, until it ends
    time.sleep(1.0)
    held = s.get_status('fld')
    assert held.state == STATE_FOREIGN and held.lease_remaining_s == 0.0
    # it ends at 3 s: the palette returns
    s.wait_status('fld', lambda m: m.state == STATE_IDLE, 6.0, 'idle after the run')
    s.wait_led(lambda m: s.display('tfld').brightness == 100)
    assert s.display('tfld').changed_by == 'ros'


def test_the_palette_returns_when_a_hold_with_no_run_expires(stack):
    s = stack(kinds=('led',), mock_args=('-p', 'hold_ms:=1500.0'))
    s.start_mood('calm')
    s.wait_led(lambda m: s.display('rld').brightness == 60)
    s.say('leds brightness rld 10')
    s.wait_status('rld', lambda m: m.state == STATE_FOREIGN, 3.0, 'foreign')
    assert s.display('rld').brightness == 10
    s.wait_status('rld', lambda m: m.state == STATE_IDLE, 6.0, 'hold expired')
    s.wait_led(lambda m: s.display('rld').brightness == 60)


def test_a_held_by_console_refusal_is_counted_in_refused(stack):
    s = stack(kinds=('led',))
    s.start_mood('calm')
    s.say('leds run fld rainbow -t 20')
    s.wait_status('fld', lambda m: m.state == STATE_FOREIGN, 3.0, 'foreign')
    assert s.get_status('fld').refused == 0
    # an operator's intent below 200 is sent (the person's hold is the device's to refuse)
    s.intent('fld', 'manual', 199, 'effect', {'name': 'fire'}, duration_ms=2000)
    status = s.wait_status('fld', lambda m: m.refused == 1, 6.0, 'refusal counted')
    assert status.state == STATE_FOREIGN and status.owner == 'console'
    assert s.surface('fld').effect == 'rainbow'
    assert 'held by console' in s.arbiters['led'].text()
    # 200 is PRIORITY_OVERRIDE: it runs over the person
    s.intent('fld', 'manual', 200, 'effect', {'name': 'fire'}, duration_ms=2000)
    s.wait_led(lambda m: next(x for x in m.surfaces if x.name == 'fld').source == 'ros')
    assert s.surface('fld').effect == 'fire'


def test_owner_manual_holds_every_led_channel_forever_and_auto_releases(stack):
    s = stack(kinds=('led',))
    s.start_mood('calm')
    s.say('leds owner manual')
    for ch in LED_CHANNELS:
        status = s.wait_status(ch, lambda m: m.state == STATE_FOREIGN, 3.0, 'foreign')
        assert status.owner == 'manual' and status.lease_remaining_s == -1.0
    s.say('leds owner auto')
    for ch in LED_CHANNELS:
        s.wait_status(ch, lambda m: m.state == STATE_IDLE, 6.0, 'released')


def test_strip_off_reports_dark_and_on_restores(stack):
    s = stack(kinds=('led',))
    s.start_mood('calm')
    s.say('leds enable off')
    for ch in LED_CHANNELS:
        s.wait_status(ch, lambda m: m.state == STATE_DARK, 3.0, 'dark')
    s.intent('fld', 'idle', 10, 'effect', {'name': 'fire'}, duration_ms=1000)
    s.wait_status('fld', lambda m: m.dropped >= 1, what='idle dropped while dark')
    s.say('leds enable on')
    for ch in LED_CHANNELS:
        s.wait_status(ch, lambda m: m.state == STATE_IDLE, 6.0, 'back')


# --- the mood ---------------------------------------------------------------------------------

def palette_calls(output):
    """Return {display: number of palette configure calls} from the arbiter's debug log."""
    calls = {}
    for line in output.splitlines():
        if 'configure on' in line and '(palette)' in line:
            display = line.split('configure on ')[1].split()[0]
            calls[display] = calls.get(display, 0) + 1
    return calls


def test_a_new_mood_label_reconfigures_the_displays_once(stack):
    s = stack(kinds=('led',), arbiter_args=('--log-level', 'debug'))
    s.start_mood('calm')
    s.wait_led(lambda m: s.display('tfld').brightness == 60)
    time.sleep(1.0)                                    # five more mood messages
    shown = ('tfld', 'bfld', 'rld', 'fpsi', 'rpsi')
    out = s.arbiters['led'].text
    # Count from the first mood message on: on a slow graph the neutral palette may have been
    # sent before it arrived, and that is not a repeat. The node logs the mood line first.
    mark = out().index('mood: - -> calm')
    assert palette_calls(out()[mark:]) == {display: 1 for display in shown}, out()
    s.start_mood('excited')
    s.wait_led(lambda m: s.display('tfld').brightness == 100)
    assert s.display('tfld').drift == 'lively' and s.display('rld').drift == 'lively'
    time.sleep(1.5)                                    # seven more mood messages
    assert palette_calls(out()[mark:]) == {display: 2 for display in shown}


def test_a_silent_mood_makes_channels_stale_with_the_neutral_palette(stack):
    s = stack(kinds=('led',))
    s.start_mood('calm')
    s.wait_led(lambda m: s.display('tfld').brightness == 60)
    s.stop_mood()
    for ch in LED_CHANNELS:
        status = s.wait_status(ch, lambda m: m.state == STATE_STALE, 6.0, 'stale')
        assert status.mood == 'neutral'
    s.wait_led(lambda m: s.display('tfld').brightness == 40)     # neutral, from the config
    s.start_mood('excited')
    s.wait_status('fld', lambda m: m.state == STATE_IDLE and m.mood == 'excited', 6.0, 'back')
    s.wait_led(lambda m: s.display('tfld').brightness == 100)


def test_the_shipped_config_is_the_default_and_the_palette_follows_the_mood(stack):
    s = stack(kinds=('led',), config=False)
    s.start_mood('excited')
    s.wait_led(lambda m: s.display('tfld').brightness == 100, 10.0, 'excited brightness')
    assert s.display('tfld').drift == 'lively'
    assert s.display('tfld').changed_by_owner == 'r2_expression'


# --- local refusals ---------------------------------------------------------------------------

def test_a_long_effect_name_is_refused_locally_and_never_sent(stack):
    s = stack(kinds=('led',), arbiter_args=('--log-level', 'debug'))
    s.start_mood('calm')
    s.wait_led(lambda m: s.display('tfld').brightness == 60)

    def picture():
        return [(x.name, x.running, x.effect, x.last_effect, x.last_end, x.source)
                for x in s.latest['leds'].surfaces]
    before = picture()
    s.intent('fld', 'manual', 199, 'effect', {'name': 'x' * 70}, duration_ms=2000)
    wait_until(lambda: 'name too long' in s.arbiters['led'].text(), 6.0, 'local refusal',
               s.output)
    time.sleep(0.5)
    out = s.arbiters['led'].text()
    assert 'fld: run refused locally: name too long' in out
    assert 'run on fld' not in out                     # the debug line before every call
    assert picture() == before
    assert s.get_status('fld').state != STATE_RUNNING
    s.intent('fld', 'manual', 199, 'effect', {'name': 'x' * 40}, duration_ms=500)
    wait_until(lambda: 'run on fld (intent)' in s.arbiters['led'].text(), 6.0,
               'a good name is sent')


# --- the Magic Panel --------------------------------------------------------------------------

def test_the_magic_node_starts_a_sequence_and_stops_it_at_its_duration(stack):
    s = stack(kinds=('magic',))
    s.start_mood('calm')
    # 56 is a random show: it loops until stopped, so only the arbiter's stop ends it.
    s.intent('magic', 'manual', 199, 'sequence', {'id': 56, 'repeat': 0},
             duration_ms=2000, intent_id='show')
    status = s.wait_status('magic', lambda m: m.state == STATE_RUNNING, 6.0, 'running')
    assert (status.active_action, status.active_intent_id) == ('sequence', 'show')
    panel = wait_until(lambda: s.latest.get('magic') and s.latest['magic'].state == 1 and
                       s.latest['magic'], 6.0, 'panel running', s.output)
    assert (panel.started_by, panel.owner, panel.sequence_id) == ('ros', 'r2_expression', 56)
    stopped = wait_until(lambda: s.latest['magic'].state == 3, 10.0, 'panel stopped',
                         s.output)
    assert stopped
    s.wait_status('magic', lambda m: m.state == STATE_CLAIMED, 6.0, 'claimed after')


def test_the_magic_node_styles_the_panel_from_the_mood(stack):
    s = stack(kinds=('magic',))
    s.start_mood('excited')
    wait_until(lambda: s.latest.get('magic') and s.latest['magic'].live.brightness == 12, 8.0,
               'excited brightness', s.output)


# --- claims across the two nodes -----------------------------------------------------------------

def test_each_node_answers_the_claim_for_its_own_channels_and_refuses_the_others(stack):
    s = stack()
    own = s.claim('led', 'rld', 'op', 150, 5.0)
    assert own.accepted and own.status.channel == 'rld' and own.status.state == STATE_CLAIMED
    other = s.claim('led', 'magic', 'op', 150, 5.0)
    assert not other.accepted and other.reason == 'not my channel'
    own = s.claim('magic', 'magic', 'op', 150, 5.0)
    assert own.accepted and own.status.state == STATE_CLAIMED
    other = s.claim('magic', 'psi', 'op', 150, 5.0)
    assert not other.accepted and other.reason == 'not my channel'
    for _ in range(10):                                 # no routing lottery
        assert s.claim('led', 'rld', 'op', 150, 5.0).accepted
        assert s.claim('magic', 'magic', 'op', 150, 5.0).accepted
    assert s.claim('led', 'rld', 'op', 150, 0.0).accepted
    assert s.claim('magic', 'magic', 'op', 150, 0.0).accepted


# --- the graph ---------------------------------------------------------------------------------

def _names(entries):
    return {name for name, _ in entries}


def test_the_publisher_subscriber_and_client_sets_are_what_the_plan_lists(stack):
    s = stack()
    node = s.node
    dev = '/%s' % s.mock_name
    expected = {
        'led': {
            'clients': {dev + '/leds/' + x for x in ('list', 'run', 'say', 'stop', 'configure')},
            'subs': {s.topics['/expression/intent'], s.topics['/mood/state'],
                     dev + '/leds/states'},
            'claim': s.topics['/expression/claim/led']},
        'magic': {
            'clients': {dev + '/magic/' + x for x in ('list', 'start', 'stop', 'configure')},
            'subs': {s.topics['/expression/intent'], s.topics['/mood/state'],
                     dev + '/magic/states'},
            'claim': s.topics['/expression/claim/magic']}}
    for kind, want in expected.items():
        name = s.arbiter_name(kind)

        def graph(name=name):
            return (_names(node.get_publisher_names_and_types_by_node(name, '/')),
                    _names(node.get_subscriber_names_and_types_by_node(name, '/')),
                    _names(node.get_client_names_and_types_by_node(name, '/')),
                    _names(node.get_service_names_and_types_by_node(name, '/')))
        wait_until(lambda graph=graph: graph()[2] == want['clients'], 10.0,
                   '%s clients' % kind, s.output)
        pubs, subs, clients, services = graph()
        assert pubs == {s.topics['/expression/status'], '/rosout', '/parameter_events'}
        assert subs == want['subs']
        assert clients == want['clients']
        assert not any(c.startswith('/r2_dome') for c in clients)
        assert want['claim'] in services


# --- lifecycle ----------------------------------------------------------------------------------

def test_startup_parameters_are_refused_later(stack):
    s = stack(kinds=('led',))
    name = '/' + s.arbiter_name('led')
    for param, value in (('device', '/r2_dome'), ('config', '/tmp/other.yaml'),
                         ('use_sim_time', 'true')):
        out = subprocess.run(['ros2', 'param', 'set', name, param, value],
                             capture_output=True, text=True, timeout=30)
        assert 'failed' in out.stdout.lower() + out.stderr.lower(), out.stdout + out.stderr
        assert 'restart the node' in out.stdout + out.stderr


@pytest.mark.parametrize('sig', [signal.SIGINT, signal.SIGTERM])
def test_the_nodes_exit_cleanly_on_sigint_and_sigterm(stack, sig):
    s = stack()
    s.start_mood('calm')
    started = time.monotonic()
    codes = {kind: proc.close(sig) for kind, proc in s.arbiters.items()}
    assert time.monotonic() - started < 12.0
    assert set(codes.values()) <= {0, -signal.SIGTERM, -signal.SIGINT}, codes
    for proc in s.arbiters.values():
        assert 'Traceback' not in proc.text(), proc.text()
    s.arbiters.clear()


def test_the_arbiter_waits_for_a_device_that_starts_late(stack):
    s = stack(kinds=('led',), start_mock=False)
    s.wait_status('fld', lambda m: m.state == STATE_OFFLINE, STARTUP_S, 'offline')
    s.start_mock()
    s.start_mood('calm')
    s.wait_status('fld', lambda m: m.state == STATE_IDLE, 30.0, 'idle once the device is up')
    s.wait_led(lambda m: s.display('tfld').brightness == 60, 10.0)
    assert 'waiting for its list service' in s.arbiters['led'].text()


def test_a_device_that_stops_answering_is_offline_and_a_call_times_out(stack):
    s = stack(kinds=('led',))
    s.start_mood('calm')
    s.wait_led(lambda m: s.display('tfld').brightness == 60)
    s.mock.send(signal.SIGSTOP)
    s.wait_status('fld', lambda m: m.state == STATE_OFFLINE, 8.0, 'offline')
    s.intent('fld', 'manual', 199, 'effect', {'name': 'fire'}, duration_ms=1000)
    wait_until(lambda: 'no answer in 5 s' in s.arbiters['led'].text(), 12.0, 'timeout logged',
               s.output)
    s.mock.send(signal.SIGCONT)
    s.wait_status('fld', lambda m: m.state != STATE_OFFLINE, 15.0, 'back')


def test_a_restarted_arbiter_adopts_its_own_leftover_run(stack):
    s = stack(kinds=('led',))
    s.start_mood('calm')
    s.intent('rld', 'manual', 199, 'effect', {'name': 'fire'})
    s.wait_led(lambda m: next(x for x in m.surfaces if x.name == 'rld').running)
    s.arbiters['led'].close(signal.SIGKILL)
    s.status.clear()
    s.start_arbiter('led')
    s.wait_ready()
    status = s.wait_status('rld', lambda m: m.state == STATE_RUNNING, 10.0, 'adopted')
    assert status.active_source == 'adopted'
    assert 0 < status.active_remaining_ms <= 30000       # max_duration_ms from the restart
    assert s.surface('rld').running                       # not stopped on adoption
    assert s.get_status('fld').state != STATE_RUNNING


def test_a_stalled_device_queue_drops_stale_personality_actions_but_keeps_manual(stack):
    s = stack(kinds=('led',))
    s.start_mood('calm')
    s.wait_led(lambda m: all(s.display(n).brightness == 60 for n in ('tfld', 'rld', 'fpsi')),
               10.0, 'the calm palette applied')
    s.mock.send(signal.SIGSTOP)
    # The first call blocks the worker for its 5 s timeout; the rest queue up behind it.
    s.intent('fld', 'manual', 199, 'effect', {'name': 'fire'}, duration_ms=1000)
    for name in ('fire', 'rainbow', 'roam'):          # one target: only the newest is queued
        s.intent('rld', 'idle', 10, 'effect', {'name': name}, duration_ms=20000)
    s.intent('psi', 'manual', 199, 'effect', {'name': 'rainbow'}, duration_ms=20000)
    wait_until(lambda: 'no answer in 5 s' in s.arbiters['led'].text(), 12.0, 'timeout logged',
               s.output)
    s.mock.send(signal.SIGCONT)
    wait_until(lambda: 'dropped 1 stale device actions' in s.arbiters['led'].text(), 20.0,
               'stale burst reported', s.output)
    text = s.arbiters['led'].text()
    assert text.count('dropping queued personality actions') == 1     # once per burst
    s.wait_led(lambda m: next(x for x in m.surfaces if x.name == 'fpsi').running, 15.0,
               'the manual psi run was kept')
    assert s.surface('fpsi').effect == 'rainbow'
    assert not s.surface('rld').running                # the stale idle runs never played


def test_a_wedged_device_replays_at_most_one_run_per_target_when_it_recovers(stack):
    s = stack(kinds=('led',), arbiter_args=('--log-level', 'debug'))
    s.start_mood('calm')
    s.wait_led(lambda m: all(s.display(n).brightness == 60 for n in ('tfld', 'rld', 'fpsi')),
               10.0, 'the calm palette applied')
    s.mock.send(signal.SIGSTOP)
    s.intent('fld', 'manual', 199, 'effect', {'name': 'fire'}, duration_ms=1000)  # blocks 5 s
    for name in ('fire', 'rainbow', 'roam', 'wipe'):          # four manual runs, one target
        s.intent('rld', 'manual', 199, 'effect', {'name': name}, duration_ms=20000)
    s.intent('rld', 'manual', 199, 'stop')                     # a stop is never replaced
    s.intent('rld', 'manual', 199, 'effect', {'name': 'cycle'}, duration_ms=20000)
    wait_until(lambda: 'no answer in 5 s' in s.arbiters['led'].text(), 12.0, 'timeout logged',
               s.output)
    s.mock.send(signal.SIGCONT)
    s.wait_led(lambda m: s.surface('rld').effect.startswith('cycle'), 15.0, 'the newest played')
    time.sleep(1.0)
    text = s.arbiters['led'].text()
    played = [ln for ln in text.splitlines() if 'run on rld (intent)' in ln]
    assert len(played) == 1, text                              # one run, not five
    assert text.count('rld: stop on rld') >= 1, text            # the stop was kept


def test_a_second_arbiter_of_the_same_name_refuses_to_start(stack):
    s = stack(kinds=('led',))
    s.start_mood('calm')
    second = Proc(s.arbiter_argv('led'))
    try:
        code = second.proc.wait(timeout=40)
    except subprocess.TimeoutExpired:
        second.close()
        raise AssertionError('the duplicate did not exit\n' + second.text())
    assert code == 1, second.text()
    assert 'cannot start' in second.text() and '/' + s.arbiter_name('led') in second.text()
    s.wait_status('fld', lambda m: m.state != STATE_OFFLINE, 5.0, 'the first one is fine')
    allowed = Proc(s.arbiter_argv('led', ('-p', 'allow_duplicate:=true')))
    try:
        time.sleep(6.0)
        assert allowed.proc.poll() is None, allowed.text()
    finally:
        allowed.close()


def test_a_hostile_params_string_does_not_kill_the_node(stack):
    s = stack(kinds=('led',))
    s.start_mood('calm')
    msg = ExpressionIntent()
    msg.channel, msg.source, msg.priority, msg.action = 'rld', 'manual', 199, 'effect'
    msg.params, msg.intent_id = '[' * 200000, 'deep'
    s.intents.publish(msg)
    with s.lock:
        s.status.clear()
    s.wait_status('rld', lambda m: m.dropped >= 1, 10.0, 'the bomb dropped and counted')
    s.intent('rld', 'manual', 199, 'effect', {'name': 'fire'}, duration_ms=2000,
             intent_id='after')
    s.wait_status('rld', lambda m: m.state == STATE_RUNNING and
                  m.active_intent_id == 'after', 8.0, 'a later intent still runs')
    assert 'Traceback' not in s.arbiters['led'].text()
    assert s.arbiters['led'].proc.poll() is None


# --- palettes and flourishes (step 5) ---------------------------------------------------------

def flourish_config(tmp_path, picks, every_s=(0.5, 1.0)):
    path = tmp_path / 'flourish.yaml'
    path.write_text(config_text(flourish={'every_s': list(every_s), 'picks': picks}))
    return str(path)


def test_mood_strength_scales_the_brightness_between_min_and_max(stack):
    s = stack(kinds=('led',))
    s.start_mood('calm', 0.0)                                  # calm is [50, 60]
    s.wait_led(lambda m: s.display('tfld').brightness == 50, 10.0, 'strength 0')
    s.start_mood('excited', 0.5)                               # excited is [80, 100]
    s.wait_led(lambda m: s.display('tfld').brightness == 90, 10.0, 'strength 0.5')
    assert s.display('rld').brightness == 90 and s.display('fpsi').brightness == 90


def test_flourishes_fire_within_every_s_on_an_idle_channel_and_never_under_a_claim(
        stack, tmp_path):
    picks = [{'channel': 'fld', 'effect': 'rainbow', 'weight': 1, 'duration_ms': 300}]
    s = stack(kinds=('led',), config=flourish_config(tmp_path, picks))
    s.start_mood('calm')
    started = {}

    def fired():
        with s.lock:
            for t, m in s.history:
                if (m.channel == 'fld' and m.state == STATE_RUNNING and
                        m.active_source == 'idle'):
                    started.setdefault(m.active_intent_id, t)
        return len(started) >= 4
    wait_until(fired, 20.0, 'four flourishes', s.output)
    times = sorted(started.values())
    gaps = [b - a for a, b in zip(times, times[1:])]
    # every_s is 0.5-1.0 s from one flourish to the next; the status stream is 5 Hz
    assert all(0.3 <= g <= 1.0 + 0.5 for g in gaps), gaps
    claim = s.claim('led', 'fld', 'op', 150, 30.0)
    assert claim.accepted
    status = s.wait_status('fld', lambda m: m.state == STATE_CLAIMED)
    dropped, mark = status.dropped, time.monotonic()
    wait_until(lambda: s.get_status('fld').dropped >= dropped + 2, 8.0, 'flourishes dropped',
               s.output)
    with s.lock:
        under_claim = [m for t, m in s.history if t > mark and m.channel == 'fld']
    assert all(m.state == STATE_CLAIMED for m in under_claim)
    assert not s.surface('fld').running
    # the claim does not stop the other channels' styling: rld stays idle and styled
    assert s.get_status('rld').state == STATE_IDLE


def test_a_pick_naming_an_absent_clip_is_dropped_with_one_warn(stack, tmp_path):
    picks = [{'channel': 'fld', 'clip': 'nosuchclip', 'weight': 5, 'duration_ms': 300},
             {'channel': 'fld', 'effect': 'rainbow', 'weight': 1, 'duration_ms': 300}]
    s = stack(kinds=('led',), config=flourish_config(tmp_path, picks))
    s.start_mood('calm')
    wait_until(lambda: 'nosuchclip' in s.arbiters['led'].text(), 10.0, 'warned', s.output)
    s.wait_status('fld', lambda m: m.state == STATE_RUNNING and m.active_source == 'idle',
                  10.0, 'the surviving pick runs')
    s.start_mood('excited')
    s.start_mood('calm')                                       # the pick is validated again
    time.sleep(2.0)
    warns = [ln for ln in s.arbiters['led'].text().splitlines() if 'nosuchclip' in ln]
    assert len(warns) == 1 and 'WARN' in warns[0], warns
    assert not any(x.effect == 'nosuchclip' for x in s.latest['leds'].surfaces)


def test_the_magic_panel_brightness_follows_the_mood_but_not_while_the_panel_is_held(stack):
    s = stack(kinds=('magic',), mock_args=('-p', 'hold_ms:=2500.0'))

    def brightness():
        magic = s.latest.get('magic')
        return magic.live.brightness if magic else None
    s.start_mood('calm')
    wait_until(lambda: brightness() == 5, 8.0, 'calm brightness', s.output)
    s.say('magic_panel start 44')                              # a person at the dome
    s.wait_status('magic', lambda m: m.state == STATE_FOREIGN, 5.0, 'foreign')
    s.start_mood('excited')
    time.sleep(1.0)
    assert brightness() == 5                                   # not sent while held
    wait_until(lambda: brightness() == 12, 20.0, 'excited brightness after the hold',
               s.output)


def test_the_palette_arrives_after_the_device_stalls_through_the_change(stack):
    s = stack(kinds=('led',))
    s.start_mood('calm')
    s.wait_led(lambda m: s.display('tfld').brightness == 60)
    s.mock.send(signal.SIGSTOP)
    s.start_mood('excited')
    wait_until(lambda: 'no answer in 5 s' in s.arbiters['led'].text(), 12.0, 'timeout logged',
               s.output)
    s.mock.send(signal.SIGCONT)
    s.wait_led(lambda m: all(s.display(n).brightness == 100 for n in ('tfld', 'bfld', 'rld')) and
               all(s.display(n).drift == 'lively' for n in ('tfld', 'bfld', 'rld')), 25.0,
               'the excited palette arrived')
    assert s.display('fpsi').brightness == 100 and s.display('rpsi').brightness == 100


def test_a_psi_flourish_runs_on_both_psis(stack, tmp_path):
    picks = [{'channel': 'psi', 'clip': 'flash', 'rgb': 0xff2200, 'duration_ms': 800}]
    s = stack(kinds=('led',), config=flourish_config(tmp_path, picks))
    s.start_mood('calm')
    s.wait_led(lambda m: all(next(x for x in m.surfaces if x.name == n).running
                             for n in ('fpsi', 'rpsi')), 15.0, 'both PSIs flash')
    assert s.surface('fpsi').effect == s.surface('rpsi').effect == 'flash'
    assert s.surface('fpsi').owner == 'r2_expression'
