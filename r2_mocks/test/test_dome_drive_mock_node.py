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
Node tests: ``dome_drive_mock`` and the real ``dome_twist_converter`` as subprocesses.

The test publishes a ``TwistStamped`` on the (remapped) personality topic; the converter turns
it into the (remapped) ``/dome_controller/commands`` the mock listens on, as on the robot.
Every name is test-private (a unique node name and namespace), so nothing on a real graph is
touched.
"""

import math
import os
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import PackageNotFoundError
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

STARTUP_S = 30.0


def _converter_installed():
    try:
        get_package_prefix('r2_bringup')
    except PackageNotFoundError:
        return False
    return True


# r2_bringup cannot be a test_depend (it exec_depends on this package), so say so when absent.
needs_converter = pytest.mark.skipif(
    not _converter_installed(), reason='r2_bringup (dome_twist_converter) is not installed')
_counter = 0


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.try_shutdown()


class Rig:
    """A mock, a converter, and a probe node that drives and records them."""

    def __init__(self, *mock_args):
        global _counter
        _counter += 1
        tag = 'ddm_%d_%d' % (os.getpid(), _counter)
        self.mock_name = '%s_mock' % tag
        self.commands_topic = '/%s/commands' % tag
        self.personality_topic = '/%s/personality' % tag
        self.procs = []
        self._start(['ros2', 'run', 'r2_bringup', 'dome_twist_converter', '--ros-args',
                     '-r', '__node:=%s_conv' % tag,
                     '-r', 'dome/cmd/personality:=%s' % self.personality_topic,
                     '-r', 'dome/cmd/status:=/%s/status' % tag,
                     '-r', 'dome_controller/commands:=%s' % self.commands_topic])
        self._start(['ros2', 'run', 'r2_mocks', 'dome_drive_mock', '--ros-args',
                     '-r', '__node:=%s' % self.mock_name,
                     '-r', '%s:=%s' % ('/dome_controller/commands', self.commands_topic),
                     *mock_args])
        self.node = rclpy.create_node('%s_probe' % tag)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.states = []        # (time, position)
        self.states_30hz = []
        self.odoms = []         # (time, linear.x)
        self.commands = []
        prefix = '/%s/' % self.mock_name
        self.node.create_subscription(JointState, prefix + 'joint_states', self._on_state, 10)
        self.node.create_subscription(
            JointState, prefix + 'joint_states_30hz', self._on_state_30hz, 10)
        self.node.create_subscription(Odometry, prefix + 'odom', self._on_odom, 10)
        self.node.create_subscription(
            Float64MultiArray, self.commands_topic, self._on_command, 10)
        self.personality = self.node.create_publisher(TwistStamped, self.personality_topic, 10)
        assert self.wait_for(self._wired, STARTUP_S), self.logs()

    def _start(self, argv):
        self.procs.append(subprocess.Popen(
            argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            start_new_session=True))

    def _wired(self):
        """Return whether the converter hears the test and the mock hears the converter."""
        listeners = self.node.get_subscriptions_info_by_topic(self.commands_topic)
        return (self.personality.get_subscription_count() > 0
                and any(i.node_name == self.mock_name for i in listeners)
                and self.states)

    def _on_state(self, msg):
        assert list(msg.name) == ['dome_joint'] and len(msg.position) == 1
        assert not msg.velocity and not msg.effort
        self.states.append((time.monotonic(), msg.position[0]))

    def _on_state_30hz(self, msg):
        self.states_30hz.append((time.monotonic(), msg.position[0]))

    def _on_odom(self, msg):
        self.odoms.append((time.monotonic(), msg.twist.twist.linear.x))

    def _on_command(self, msg):
        self.commands.append((time.monotonic(), msg.data[0]))

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.01)

    def wait_for(self, predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return True
            self.executor.spin_once(timeout_sec=0.02)
        return False

    def drive(self, rad_s, seconds):
        """Stream a personality velocity at 20 Hz for ``seconds``."""
        msg = TwistStamped()
        msg.twist.angular.z = rad_s
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.personality.publish(msg)
            self.spin(0.05)

    def stop_and_settle(self):
        """Let the converter time out (one final 0), then wait for the dome to stand still."""
        assert self.wait_for(lambda: self.commands[-1][1] == 0.0, 5.0)
        zero_at = self.states[-1][1]            # the last position published before the 0
        self.spin(1.5)
        before = self.states[-1][1]
        self.spin(0.5)
        assert self.states[-1][1] == before, 'the dome is still moving'
        return zero_at, before

    def logs(self):
        out = []
        for proc in self.procs:
            if proc.poll() is not None:
                out.append(proc.stdout.read())
        return '\n'.join(out) or '(still running)'

    def close(self):
        for proc in self.procs:
            try:
                os.killpg(proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for proc in self.procs:
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
            proc.stdout.close()
        self.executor.remove_node(self.node)
        self.node.destroy_node()


@pytest.fixture
def make_rig():
    rigs = []

    def make(*args):
        rigs.append(Rig(*args))
        return rigs[-1]
    yield make
    for rig in rigs:
        rig.close()


@needs_converter
def test_joint_states_at_10hz_named_dome_joint_position_only(make_rig):
    rig = make_rig('-p', 'initial_rad:=1.25')
    rig.states.clear()
    rig.spin(3.0)
    times = [t for t, _ in rig.states]
    rate = (len(times) - 1) / (times[-1] - times[0])
    assert 8.0 < rate < 12.0, rate
    assert all(p == pytest.approx(1.25) for _, p in rig.states)   # not commanded: stands still
    assert not rig.states_30hz and not rig.odoms                  # both are opt-in


@needs_converter
def test_the_dome_turns_the_commanded_way_and_coasts_after_the_final_zero(make_rig):
    rig = make_rig('-p', 'initial_rad:=1.0')
    rig.drive(2.0, 2.5)
    assert rig.states[-1][1] > 1.0 + 1.5            # about 3.7 rad after the lag, in 2.5 s
    zero_at, final = rig.stop_and_settle()
    assert rig.commands[-1][1] == 0.0
    assert final - zero_at > 0.4                    # coast 0.37 * ~1.95 - 0.07 = 0.65 rad
    assert 0.0 <= final < 2.0 * math.pi

    rig.drive(-2.0, 2.5)
    assert rig.states[-1][1] < final - 1.5          # negative turns the other way
    rig.stop_and_settle()


@needs_converter
def test_ideal_stops_at_once_with_no_coast(make_rig):
    rig = make_rig('-p', 'ideal:=true', '-p', 'initial_rad:=3.0')
    rig.drive(1.0, 2.0)                             # no lag: 1 rad/s at once, so about 2 rad
    assert rig.states[-1][1] == pytest.approx(5.0, abs=0.35)
    zero_at, final = rig.stop_and_settle()
    assert 0.0 <= final - zero_at < 0.2             # at most the 0.1 s the state lags


@needs_converter
def test_state_hz_30hz_topic_and_odometry_are_opt_in(make_rig):
    rig = make_rig('-p', 'state_hz:=5.0', '-p', 'publish_30hz:=true',
                   '-p', 'body_speed_m_s:=0.4')
    rig.states.clear()
    rig.states_30hz.clear()
    rig.odoms.clear()
    rig.spin(3.0)
    assert 10 <= len(rig.states) <= 17
    assert 70 <= len(rig.states_30hz) <= 100
    assert 25 <= len(rig.odoms) <= 35
    assert all(v == pytest.approx(0.4) for _, v in rig.odoms)


def test_a_bad_parameter_exits_with_1():
    proc = subprocess.run(
        ['ros2', 'run', 'r2_mocks', 'dome_drive_mock', '--ros-args', '-p', 'state_hz:=0.0',
         '-r', '__node:=ddm_bad_%d' % os.getpid()],
        capture_output=True, text=True, timeout=30)
    assert proc.returncode == 1 and 'state_hz' in proc.stdout + proc.stderr
