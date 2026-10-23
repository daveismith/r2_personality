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
Integration tests: ``dome_arbiter`` against the real converter and ``dome_drive_mock``.

Three subprocesses, as on the robot (phase 2 plan 5.4, step 5): ``dome_twist_converter``
(``r2_bringup``), ``dome_drive_mock`` (``r2_mocks``, the plant of plan 2.1) and the arbiter.
Every name is test-private: the arbiter's topics are its parameters, the converter's and the
mock's are remapped, and ``/expression/*`` and ``/mood/state`` are remapped as in
``test_nodes.py`` (whose ``Proc`` and ``wait_until`` are reused). The test plays the mood
publisher (5 Hz), the joystick and, where it needs one, the drive's velocity command.

Without ``r2_bringup`` the tests skip; with ``R2_PHASE2_FULL=1`` in the environment (the sim
acceptance sets it) they fail instead, and so does item 15's without ``r2_stimulus``.
"""

import json
import math
import os
import signal
import subprocess
import threading
import time

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import PackageNotFoundError
from astromech_msgs.msg import ChannelStatus
from astromech_msgs.msg import DomeCommandStatus
from astromech_msgs.msg import ExpressionIntent
from astromech_msgs.msg import MoodState
from astromech_msgs.msg import Stimulus
from astromech_msgs.srv import ClaimChannel
from fixtures import dome_data, MIN_SPEED
from geometry_msgs.msg import TwistStamped
import pytest
from r2_expression.motion import wrap
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from test_nodes import config_text, Proc, ros_context, STARTUP_S, wait_until  # noqa: F401
from test_nodes import STATE_CLAIMED, STATE_FOREIGN, STATE_IDLE, STATE_OFFLINE, STATE_RUNNING
from test_nodes import STATE_STALE
import yaml

STATE_SUPPRESSED = 7             # ChannelStatus.STATE_SUPPRESSED (astromech_msgs 2.2.0)
PERSONALITY_TIMEOUT_S = 0.5      # the converter's default
CONVERTER_TICK_S = 0.05
MOOD_STALE_S = 1.0
DEVICE_STALE_S = 2.0
BAND_RAD = 0.05                  # the smooth profile's settle band (fixtures.PROFILES)
FULL = os.environ.get('R2_PHASE2_FULL') == '1'
_counter = 0


def _installed(package):
    try:
        get_package_prefix(package)
    except PackageNotFoundError:
        return False
    return True


def require(package, what):
    """Skip without ``package``; fail instead under ``R2_PHASE2_FULL=1``."""
    if not _installed(package):
        message = '%s (%s) is not installed' % (package, what)
        if FULL:
            pytest.fail('R2_PHASE2_FULL=1: ' + message)
        pytest.skip(message)


@pytest.fixture(autouse=True)
def _converter():
    require('r2_bringup', 'dome_twist_converter')


def dome_config_text():
    """Return the test dome config: short staleness, a 1 s manual hold, no glances."""
    data = yaml.safe_load(config_text(data=dome_data(), manual_hold_s=1.0))
    assert (data['defaults']['mood_stale_s'], data['defaults']['device_stale_s']) == (
        MOOD_STALE_S, DEVICE_STALE_S)
    for mood in data['moods'].values():
        mood.get('dome', {}).pop('glance', None)
    data['moods']['sleepy']['dome'].pop('rest_rad', None)
    return yaml.safe_dump(data)


class DomeRig:
    """The converter, the mock and the arbiter, and a test node that plays everyone else."""

    def __init__(self, tmp_path, mock_args=(), debug=False, start_arbiter=True):
        global _counter
        _counter += 1
        self.tag = 'dn%d_%d' % (os.getpid(), _counter)
        t = '/t%s' % self.tag
        self.mock_name = 'dome_drive_mock_t%s' % self.tag
        self.arbiter_name = 'dome_arbiter_t%s' % self.tag
        self.converter_name = 'dome_twist_converter_t%s' % self.tag
        self.names = {
            'personality': t + '/dome/cmd/personality', 'status': t + '/dome/cmd/status',
            'commands': t + '/dome_controller/commands', 'joystick': t + '/cmd_vel_dome',
            'foxglove': t + '/cmd_vel_dome_foxglove', 'cmd_vel': t + '/diff_cont/cmd_vel',
            'joint_states': '/%s/joint_states' % self.mock_name, 'stimuli': t + '/stimuli'}
        self.topics = {name: t + name for name in (
            '/expression/intent', '/mood/state', '/expression/status', '/expression/claim/dome')}
        self.config = str(tmp_path / ('expression_%s.yaml' % self.tag))
        with open(self.config, 'w') as f:
            f.write(dome_config_text())
        self.arbiter_args = ('--log-level', '%s:=debug' % self.arbiter_name) if debug else ()
        self.converter = Proc([
            'ros2', 'run', 'r2_bringup', 'dome_twist_converter', '--ros-args',
            '-r', '__node:=%s' % self.converter_name,
            '-r', 'dome/cmd/personality:=%s' % self.names['personality'],
            '-r', 'dome/cmd/status:=%s' % self.names['status'],
            '-r', 'dome_controller/commands:=%s' % self.names['commands'],
            '-r', 'cmd_vel_dome:=%s' % self.names['joystick'],
            '-r', 'cmd_vel_dome_foxglove:=%s' % self.names['foxglove']])
        self.mock = Proc([
            'ros2', 'run', 'r2_mocks', 'dome_drive_mock', '--ros-args',
            '-r', '__node:=%s' % self.mock_name,
            '-r', '/dome_controller/commands:=%s' % self.names['commands'], *mock_args])
        self.arbiter = None

        self.node = rclpy.create_node('dome_test_%s' % self.tag)
        self.lock = threading.Lock()
        self.status = None
        self.history = []          # (time, ChannelStatus)
        self.sent = []             # (time, angular.z) on the personality topic
        self.commands = []         # (time, data[0]) on the controller topic
        self.sources = []          # (time, DomeCommandStatus.source)
        self.capped = []           # (time, DomeCommandStatus.capped)
        self.stimuli = []          # (time, Stimulus.kind)
        self.position = None
        self.positions = []        # (time, joint position)
        reliable = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        sub = self.node.create_subscription
        sub(ChannelStatus, self.topics['/expression/status'], self._on_status, reliable)
        sub(TwistStamped, self.names['personality'],
            lambda m: self._record(self.sent, m.twist.angular.z), reliable)
        sub(Float64MultiArray, self.names['commands'],
            lambda m: self._record(self.commands, m.data[0]), reliable)
        sub(DomeCommandStatus, self.names['status'], self._on_command_status, reliable)
        sub(Stimulus, self.names['stimuli'], lambda m: self._record(self.stimuli, m.kind),
            reliable)
        sub(JointState, self.names['joint_states'], self._on_joint, reliable)
        self.intents = self.node.create_publisher(
            ExpressionIntent, self.topics['/expression/intent'], reliable)
        self.mood_pub = self.node.create_publisher(
            MoodState, self.topics['/mood/state'], reliable)
        self.joystick = self.node.create_publisher(TwistStamped, self.names['joystick'], 10)
        self.cmd_vel_pub = self.node.create_publisher(TwistStamped, self.names['cmd_vel'], 10)
        self.claim_client = self.node.create_client(ClaimChannel,
                                                    self.topics['/expression/claim/dome'])
        self.extra = []            # more processes, closed with the rig
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()
        self._loops = {}
        if start_arbiter:
            self.start_arbiter()
            self.wait_ready()

    # --- processes ---------------------------------------------------------------------------

    def arbiter_argv(self, extra=()):
        remaps = []
        for name, target in self.topics.items():
            remaps += ['-r', '%s:=%s' % (name, target)]
        params = []
        for param, key in (('command_topic', 'personality'), ('status_topic', 'status'),
                           ('joint_states_topic', 'joint_states'),
                           ('cmd_vel_topic', 'cmd_vel')):
            params += ['-p', '%s:=%s' % (param, self.names[key])]
        return ['ros2', 'run', 'r2_expression', 'dome_arbiter', '--ros-args',
                '-r', '__node:=%s' % self.arbiter_name, '-p', 'config:=%s' % self.config,
                *params, *self.arbiter_args, *extra, *remaps]

    def start_arbiter(self):
        self.arbiter = Proc(self.arbiter_argv())
        return self.arbiter

    def wait_ready(self):
        """Wait for the dome channel to leave OFFLINE and the intent publisher to match."""
        self.wait_status(lambda s: s.state != STATE_OFFLINE, STARTUP_S, 'ready')
        wait_until(lambda: self.intents.get_subscription_count() >= 1 and
                   self.mood_pub.get_subscription_count() >= 1, 30.0, 'subscriptions matched',
                   self.output)

    def close(self):
        for name in list(self._loops):
            self.stop_loop(name)
        code = self.arbiter.close() if self.arbiter is not None else None
        for proc in self.extra:
            proc.close()
        self.mock.close()
        self.converter.close()
        self.executor.shutdown()
        self.node.destroy_node()
        return code

    def output(self):
        parts = [('converter', self.converter), ('mock', self.mock), ('arbiter', self.arbiter)]
        parts += [('extra %d' % i, proc) for i, proc in enumerate(self.extra)]
        return '\n'.join('--- %s ---\n%s' % (name, proc.text())
                         for name, proc in parts if proc is not None)

    def arbiter_pid(self):
        """Return the pid of the arbiter's Python process (``ros2 run`` is its parent)."""
        pid = self.arbiter.proc.pid
        with open('/proc/%d/task/%d/children' % (pid, pid)) as f:
            children = f.read().split()
        return int(children[0]) if children else pid

    # --- what the test hears -------------------------------------------------------------------

    def _record(self, log, value):
        with self.lock:
            log.append((time.monotonic(), value))

    def _on_command_status(self, msg):
        with self.lock:
            now = time.monotonic()
            self.sources.append((now, msg.source))
            self.capped.append((now, msg.capped))

    def _on_status(self, msg):
        if msg.channel != 'dome':
            return
        with self.lock:
            self.status = msg
            self.history.append((time.monotonic(), msg))

    def _on_joint(self, msg):
        self.position = msg.position[0]
        self._record(self.positions, msg.position[0])

    def position_at(self, t):
        """Return the first joint position published at or after ``t``."""
        with self.lock:
            return next(p for at, p in self.positions if at >= t)

    def wait_status(self, predicate, timeout=10.0, what=''):
        def check():
            with self.lock:
                msg = self.status
            return msg if msg is not None and predicate(msg) else None
        return wait_until(check, timeout, 'dome status %s' % what, self.output)

    def sent_since(self, since):
        with self.lock:
            return [(t, v) for t, v in self.sent if t >= since]

    def wait_moving(self, timeout=10.0):
        """Wait for a non-zero velocity on the personality topic; return when it was seen."""
        start = time.monotonic()
        return wait_until(lambda: next((t for t, v in self.sent_since(start) if v), None),
                          timeout, 'a non-zero personality velocity', self.output)

    def wait_quiet(self, quiet_s=1.0, timeout=20.0):
        """Wait until the personality topic has been silent for ``quiet_s``; return the log."""
        def check():
            with self.lock:
                last = self.sent[-1][0] if self.sent else 0.0
            return time.monotonic() - last >= quiet_s
        wait_until(check, timeout, 'the personality topic to fall silent', self.output)
        with self.lock:
            return list(self.sent)

    # --- what the test says ---------------------------------------------------------------

    def intent(self, source, priority, action, params=None, intent_id=''):
        msg = ExpressionIntent()
        msg.channel, msg.source, msg.priority, msg.action = 'dome', source, priority, action
        msg.params = json.dumps(params) if params is not None else ''
        msg.intent_id = intent_id
        self.intents.publish(msg)

    def _loop(self, name, hz, publish):
        stop = threading.Event()

        def run():
            while not stop.wait(1.0 / hz):
                publish()
        thread = threading.Thread(target=run, daemon=True)
        self._loops[name] = (stop, thread)
        thread.start()

    def stop_loop(self, name):
        stop, thread = self._loops.pop(name)
        stop.set()
        thread.join(timeout=2.0)

    def start_mood(self, label='calm'):
        """Publish ``/mood/state`` at 5 Hz until :meth:`stop_loop` ('mood')."""
        msg = MoodState()
        msg.mood, msg.mood_strength = label, 1.0
        self._loop('mood', 5.0, lambda: self.mood_pub.publish(msg))
        self.wait_status(lambda s: s.mood == label, 10.0, 'mood %s' % label)

    def start_joystick(self, value=0.6):
        msg = TwistStamped()
        msg.twist.linear.x = value
        self._loop('joystick', 20.0, lambda: self.joystick.publish(msg))

    def start_driving(self, speed):
        """Publish the drive's command at 20 Hz, as twist_mux does while someone drives."""
        msg = TwistStamped()
        msg.twist.linear.x = speed
        self._loop('drive', 20.0, lambda: self.cmd_vel_pub.publish(msg))

    def claim(self, owner, priority, lease_s):
        request = ClaimChannel.Request()
        request.channel, request.owner = 'dome', owner
        request.priority, request.lease_s = priority, lease_s
        wait_until(lambda: self.claim_client.wait_for_service(timeout_sec=0.1), 10.0,
                   'the claim service', self.output)
        future = self.claim_client.call_async(request)
        wait_until(future.done, 10.0, 'claim reply', self.output)
        return future.result()


@pytest.fixture
def rig(tmp_path):
    made = []

    def make(**kwargs):
        made.append(DomeRig(tmp_path, **kwargs))
        return made[-1]
    yield make
    for item in made:
        item.close()


def segments(log):
    """Return how many runs of non-zero velocities ``[(t, v)]`` holds."""
    count, previous = 0, 0.0
    for _, v in log:
        if v and not previous:
            count += 1
        previous = v
    return count


# --- (a) a look_at reaches the heading and ends with one 0 -------------------------------------

def test_a_look_at_reaches_the_heading_and_the_channel_returns_to_idle(rig):
    r = rig(mock_args=('-p', 'initial_rad:=0.3'))
    r.start_mood('calm')
    start = time.monotonic()
    r.intent('manual', 199, 'look_at', {'bearing': -2.0}, intent_id='look-a')
    status = r.wait_status(lambda s: s.state == STATE_RUNNING, 5.0, 'running')
    assert (status.active_source, status.active_action, status.active_intent_id) == (
        'manual', 'look_at', 'look-a')
    assert 0 < status.active_remaining_ms <= 8000            # max_move_s
    r.wait_status(lambda s: s.state == STATE_IDLE, 20.0, 'idle after the move and hold')
    log = [x for x in r.wait_quiet(1.5) if x[0] >= start]
    assert log[-1][1] == 0.0                                 # the move ended with a 0 ...
    time.sleep(1.0)
    assert len(r.sent_since(start)) == len(log)              # ... and nothing came after it
    assert all(v == 0.0 or MIN_SPEED - 1e-9 <= abs(v) <= 1.0 + 1e-9 for _, v in log)  # rule 4
    assert abs(wrap(r.position - 2.0)) <= BAND_RAD + 0.01, r.position  # joint = -bearing
    assert 'move over, coast scale' in r.arbiter.text()


# --- (b) a joystick takes the dome --------------------------------------------------------------

def test_b_a_joystick_mid_move_makes_the_channel_foreign_and_the_move_is_not_resumed(rig):
    r = rig(mock_args=('-p', 'initial_rad:=1.0'))
    r.start_mood('calm')
    r.intent('behavior', 100, 'turn', {'delta': 6.0}, intent_id='long-b')
    r.wait_moving()
    time.sleep(1.0)
    r.start_joystick(0.6)
    seen = wait_until(lambda: next((t for t, s in list(r.sources) if s == 'joystick'), None),
                      5.0, 'the converter on the joystick', r.output)
    status = r.wait_status(lambda s: s.state == STATE_FOREIGN, 2.0, 'foreign')
    assert status.owner == 'joystick' and status.active_action == ''
    after = r.sent_since(seen - 0.2)
    zero = next((t for t, v in after if v == 0.0), None)
    assert zero is not None and zero - seen < 0.25, after[:5]   # one tick and the transport
    assert all(v == 0.0 for t, v in r.sent_since(zero)), r.sent_since(zero)
    time.sleep(1.0)
    r.stop_loop('joystick')
    released = r.wait_status(lambda s: s.state == STATE_IDLE, 5.0, 'idle after release')
    assert released.active_action == ''
    since = time.monotonic()
    time.sleep(2.0)
    assert not any(v for _, v in r.sent_since(since))           # not resumed


# --- (c) the body drives: suppressed ------------------------------------------------------------

def test_c_a_drive_command_above_the_threshold_suppresses_drops_idle_and_lets_manual_run(rig):
    r = rig(mock_args=('-p', 'initial_rad:=1.0'))
    r.start_mood('calm')
    r.intent('behavior', 100, 'turn', {'delta': 6.0})
    r.wait_moving()
    started = time.monotonic()
    r.start_driving(0.4)                                # above body_speed_m_s 0.15
    status = r.wait_status(lambda s: s.state == STATE_SUPPRESSED, 3.0, 'suppressed')
    assert status.active_action == ''
    after = r.sent_since(started)
    zero = next((t for t, v in after if v == 0.0), None)
    assert zero is not None, after
    assert all(v == 0.0 for t, v in r.sent_since(zero))
    dropped = status.dropped
    r.intent('idle', 10, 'look_at', {'bearing': 0.5})
    r.wait_status(lambda s: s.dropped == dropped + 1, 3.0, 'idle intent dropped and counted')
    r.intent('manual', 199, 'turn', {'delta': 1.5}, intent_id='manual-c')
    running = r.wait_status(lambda s: s.active_intent_id == 'manual-c', 3.0, 'manual runs')
    assert running.state == STATE_SUPPRESSED and running.active_source == 'manual'
    r.wait_moving()
    stopped = time.monotonic()
    r.stop_loop('drive')                                # twist_mux falls silent
    r.wait_status(lambda s: s.state != STATE_SUPPRESSED, 5.0, 'clears once silent')
    assert time.monotonic() - stopped >= 0.5 + 1.0 - 0.2      # silence, then after_still_s


# --- (d) the arbiter dies mid-move: the converter stops the dome --------------------------------

def test_d_killing_the_arbiter_mid_move_stops_the_controller_within_the_timeout(rig):
    r = rig(mock_args=('-p', 'initial_rad:=1.0'))
    r.start_mood('calm')
    r.intent('manual', 199, 'turn', {'delta': 6.0})
    r.wait_moving()
    time.sleep(1.0)
    killed = time.monotonic()
    r.arbiter.send(signal.SIGKILL)

    def first_zero():
        with r.lock:
            return next((t for t, v in r.commands if t >= killed and v == 0.0), None)
    zero = wait_until(first_zero, 5.0, 'a 0 on the controller topic', r.output)
    assert zero - killed <= PERSONALITY_TIMEOUT_S + CONVERTER_TICK_S + 0.15, zero - killed
    with r.lock:
        moving_after = [t for t, v in r.commands if t > zero and v != 0.0]
    assert not moving_after
    wait_until(lambda: r.sources and r.sources[-1][1] == 'idle', 3.0, 'converter idle',
               r.output)
    r.arbiter.close()
    # Item 12's second half: restarted, it comes up IDLE and starts no move of its own.
    wait_until(lambda: r.arbiter_name not in r.node.get_node_names(), 10.0,
               'the killed arbiter to leave the graph', r.output)
    with r.lock:
        r.status = None
    restarted = time.monotonic()
    r.start_arbiter()
    r.wait_status(lambda s: True, STARTUP_S, 'the restarted arbiter')
    r.wait_status(lambda s: s.state == STATE_IDLE, 5.0, 'idle within 5 s of coming up')
    time.sleep(5.0)
    assert not any(v for _, v in r.sent_since(restarted)), r.sent_since(restarted)


# --- (d2) item 3 and rule 4 at the controller --------------------------------------------------

def test_d2_a_smooth_look_at_is_never_capped_and_every_command_is_0_or_at_the_floor(rig):
    r = rig(mock_args=('-p', 'initial_rad:=0.3'))
    r.start_mood('calm')
    start = time.monotonic()
    r.intent('manual', 199, 'look_at', {'bearing': -2.0, 'profile': 'smooth'})
    r.wait_moving()
    r.wait_status(lambda s: s.state != STATE_RUNNING, 20.0, 'the move over')
    r.wait_quiet(1.0)
    with r.lock:
        commands = [v for t, v in r.commands if t >= start]
        capped = [c for t, c in r.capped if t >= start]
    assert any(commands) and capped
    assert not any(capped), 'capped during a smooth look_at'
    assert all(v == 0.0 or MIN_SPEED - 1e-9 <= abs(v) <= 1.0 + 1e-9 for v in commands), \
        commands
    assert commands[-1] == 0.0


# --- (d3) item 6: a claim refuses a manual move, --override passes ------------------------------

def test_d3_a_claim_refuses_a_manual_look_and_an_override_runs(rig):
    r = rig()
    r.start_mood('calm')
    reply = r.claim('console', 200, 30.0)                  # express claim dome --override
    assert reply.accepted, reply.reason
    r.wait_status(lambda s: s.state == STATE_CLAIMED and s.owner == 'console', 3.0, 'claimed')
    dropped = r.status.dropped
    since = time.monotonic()
    r.intent('manual', 199, 'look_at', {'bearing': 1.0}, intent_id='refused-d3')
    r.wait_status(lambda s: s.dropped == dropped + 1, 3.0, 'manual 199 refused')
    time.sleep(0.5)
    assert not any(v for _, v in r.sent_since(since))
    r.intent('manual', 200, 'look_at', {'bearing': 1.0}, intent_id='override-d3')
    running = r.wait_status(lambda s: s.active_intent_id == 'override-d3', 3.0, 'override')
    assert running.active_source == 'manual'
    r.wait_moving()
    assert r.claim('console', 200, 0.0).accepted           # release
    r.wait_status(lambda s: s.state == STATE_IDLE, 20.0, 'idle after the release')


# --- (d4) item 15: dome_stimulus sees no push in a personality move ------------------------------

def test_d4_dome_stimulus_reports_no_push_during_a_look_at(rig):
    require('r2_stimulus', 'dome_stimulus')
    r = rig(mock_args=('-p', 'initial_rad:=0.3'))
    stimulus = Proc([
        'ros2', 'run', 'r2_stimulus', 'dome_stimulus', '--ros-args',
        '-r', '__node:=dome_stimulus_t%s' % r.tag,
        '-r', '/dome_state_broadcaster/joint_states:=%s' % r.names['joint_states'],
        '-r', '/dome_controller/commands:=%s' % r.names['commands'],
        '-r', '/stimuli:=%s' % r.names['stimuli']])
    r.extra.append(stimulus)
    wait_until(lambda: r.node.get_publishers_info_by_topic(r.names['stimuli']), STARTUP_S,
               'dome_stimulus on the graph', r.output)
    r.start_mood('calm')
    time.sleep(1.0)
    start = time.monotonic()
    for bearing in (-2.0, 1.0):
        r.intent('manual', 199, 'look_at', {'bearing': bearing, 'profile': 'snappy'})
        r.wait_moving()
        r.wait_status(lambda s: s.state != STATE_RUNNING, 20.0, 'the move over')
    r.wait_quiet(2.0)                                     # the settle_s after the last 0
    with r.lock:
        pushes = [(t, k) for t, k in r.stimuli if t >= start and k == 'dome_pushed']
    assert not pushes, pushes
    assert 'Traceback' not in stimulus.text(), stimulus.text()


# --- (e) a stale mood stops an idle move; manual still works ------------------------------------

def test_e_a_stale_mood_stops_an_idle_move_and_manual_still_runs(rig):
    r = rig(mock_args=('-p', 'initial_rad:=1.0'))
    r.start_mood('calm')
    r.intent('idle', 10, 'turn', {'delta': 6.0}, intent_id='idle-e')
    r.wait_status(lambda s: s.active_intent_id == 'idle-e', 3.0, 'idle move runs')
    r.wait_moving()
    stopped = time.monotonic()
    r.stop_loop('mood')
    status = r.wait_status(lambda s: s.state == STATE_STALE, MOOD_STALE_S + 2.0, 'stale')
    assert status.active_action == ''
    zero = next((t for t, v in r.sent_since(stopped) if v == 0.0), None)
    assert zero is not None
    assert all(v == 0.0 for _, v in r.sent_since(zero))
    r.intent('manual', 199, 'look_at', {'bearing': 0.0}, intent_id='manual-e')
    r.wait_status(lambda s: s.active_intent_id == 'manual-e', 3.0, 'manual runs while stale')
    r.wait_moving()


# --- (f) no joint state: offline -----------------------------------------------------------------

def test_f_missing_joint_states_make_the_channel_offline_and_it_comes_back(rig):
    r = rig()
    r.start_mood('calm')
    stopped = time.monotonic()
    r.mock.send(signal.SIGSTOP)
    r.wait_status(lambda s: s.state == STATE_OFFLINE, DEVICE_STALE_S + 2.0, 'offline')
    assert time.monotonic() - stopped >= DEVICE_STALE_S - 0.3
    dropped = r.status.dropped
    r.intent('manual', 199, 'look_at', {'bearing': 1.0})
    r.wait_status(lambda s: s.dropped == dropped + 1, 3.0, 'refused while offline')
    assert not any(v for _, v in r.sent_since(stopped))      # nothing sent while offline
    r.mock.send(signal.SIGCONT)
    r.wait_status(lambda s: s.state != STATE_OFFLINE, 5.0, 'back')


# --- (g) the graph: who publishes the dome's commands -------------------------------------------

def test_g_only_the_converter_publishes_the_controller_and_only_the_arbiter_the_personality(rig):
    r = rig()
    wait_until(lambda: r.node.get_publishers_info_by_topic(r.names['personality']), 10.0,
               'the personality publisher', r.output)
    controller = {i.node_name for i in r.node.get_publishers_info_by_topic(r.names['commands'])}
    personality = {i.node_name
                   for i in r.node.get_publishers_info_by_topic(r.names['personality'])}
    assert controller == {r.converter_name}
    assert personality == {r.arbiter_name}
    pubs = {name for name, _ in
            r.node.get_publisher_names_and_types_by_node(r.arbiter_name, '/')}
    assert pubs == {r.names['personality'], r.topics['/expression/status'], '/rosout',
                    '/parameter_events'}
    subs = {name for name, _ in
            r.node.get_subscriber_names_and_types_by_node(r.arbiter_name, '/')}
    assert subs == {r.names['joint_states'], r.names['status'], r.names['cmd_vel'],
                    r.topics['/expression/intent'], r.topics['/mood/state']}
    assert not r.node.get_client_names_and_types_by_node(r.arbiter_name, '/')
    services = {name for name, _ in
                r.node.get_service_names_and_types_by_node(r.arbiter_name, '/')}
    assert r.topics['/expression/claim/dome'] in services


# --- (h) track is not in this phase ------------------------------------------------------------

def test_h_track_is_refused_and_counted(rig):
    r = rig(debug=True)
    r.start_mood('calm')
    dropped = r.status.dropped
    r.intent('manual', 199, 'track', {})
    r.wait_status(lambda s: s.dropped == dropped + 1, 3.0, 'track dropped')
    wait_until(lambda: 'not in this phase' in r.arbiter.text(), 3.0, 'the reason logged',
               r.output)
    assert r.status.state != STATE_RUNNING
    out = subprocess.run(['ros2', 'param', 'set', '/' + r.arbiter_name, 'command_topic', '/x'],
                         capture_output=True, text=True, timeout=30)
    assert 'restart the node' in out.stdout + out.stderr


# --- (i) the coast learned on one move is used on the next -------------------------------------

def test_i_the_learned_coast_persists_across_two_moves(rig):
    # Gazebo's plant: no coast. The first move's first 0 comes short of the target by the
    # predicted coast, so it creeps the rest; the second knows better and its first 0 comes
    # at the target (or past it: under load a late sample may overshoot, never stop short).
    r = rig(mock_args=('-p', 'ideal:=true', '-p', 'initial_rad:=0.3'))
    r.start_mood('calm')

    def short_by(bearing):
        start = time.monotonic()
        r.intent('manual', 199, 'look_at', {'bearing': bearing})
        r.wait_moving()
        log = [x for x in r.wait_quiet(1.5) if x[0] >= start]
        moving = next(v for _, v in log if v)
        stop = next(t for t, v in log if t > start and v == 0.0)
        error = wrap(-bearing - r.position_at(stop + 0.3))      # joint = -bearing
        return error * math.copysign(1.0, moving), log          # > 0: short of the target
    first, first_log = short_by(-1.8)
    second, _ = short_by(-0.2)
    assert first > BAND_RAD and segments(first_log) == 2, (first, first_log)
    assert second <= BAND_RAD, second
    scales = [float(line.rsplit('coast scale ', 1)[1].split()[0])
              for line in r.arbiter.text().splitlines() if 'coast scale' in line]
    assert len(scales) == 2 and all(s < 0.3 for s in scales), scales
    assert abs(wrap(r.position - 0.2)) <= BAND_RAD + 0.01


# --- lifecycle, and the CPU ------------------------------------------------------------------

def test_sigint_twice_mid_move_sends_a_zero_and_exits_cleanly(rig):
    r = rig(mock_args=('-p', 'initial_rad:=1.0'))
    r.start_mood('calm')
    r.intent('manual', 199, 'turn', {'delta': 6.0})
    r.wait_moving()
    signalled = time.monotonic()
    r.arbiter.send(signal.SIGINT)                  # as a terminal and then launch both do
    time.sleep(0.01)
    code = r.arbiter.close(signal.SIGINT)
    assert code in (0, -signal.SIGINT), r.arbiter.text()
    assert 'Traceback' not in r.arbiter.text(), r.arbiter.text()
    assert 'stopped on shutdown' in r.arbiter.text()
    after = r.sent_since(signalled)
    assert after and after[-1][1] == 0.0, after
    r.arbiter = None


def test_idle_cpu_is_logged(rig, capsys):
    r = rig()
    r.start_mood('calm')
    time.sleep(2.0)
    pid = r.arbiter_pid()
    ticks = os.sysconf('SC_CLK_TCK')

    def cpu_s():
        with open('/proc/%d/stat' % pid) as f:
            fields = f.read().rsplit(')', 1)[1].split()
        return (int(fields[11]) + int(fields[12])) / ticks     # utime + stime

    def wakes():                  # the executor thread's sleeps: one per wake-up
        with open('/proc/%d/task/%d/status' % (pid, pid)) as f:
            line = next(x for x in f if x.startswith('voluntary_ctxt_switches'))
        return int(line.split()[1])
    before, woke, start = cpu_s(), wakes(), time.monotonic()
    time.sleep(20.0)
    used, wall = cpu_s() - before, time.monotonic() - start
    rate = (wakes() - woke) / wall
    with capsys.disabled():
        print('\ndome_arbiter idle CPU over %.0f s: %.2f %% of a core, executor thread '
              '%.1f wake-ups/s (pid %d)' % (wall, 100.0 * used / wall, rate, pid))
    assert r.status.state == STATE_IDLE
