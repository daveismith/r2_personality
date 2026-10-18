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
Run the three adapter executables against private topic names and check their output.

The sim's real topics may be live on this graph, so every topic is remapped to a name
private to this test and every node gets a unique name.
"""

import json
import math
import os
import signal
import subprocess
import time

from astromech_msgs.msg import DomeButtonSequence
from astromech_msgs.msg import Stimulus
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

PID = os.getpid()
P = '/t7_%d' % PID
TOPICS = {
    '/diff_cont/odom': P + '/odom',
    '/dome_state_broadcaster/joint_states': P + '/dome_js',
    '/dome_controller/commands': P + '/dome_cmd',
    '/dome/buttons/sequence': P + '/buttons',
    '/stimuli': P + '/stimuli',
}
RELIABLE = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
ALLOWED_PUBLISHERS = {P + '/stimuli', '/rosout', '/parameter_events'}


def _start(executable):
    name = '%s_t%d' % (executable, PID)
    cmd = ['ros2', 'run', 'r2_stimulus', executable, '--ros-args', '-r', '__node:=' + name]
    for src, dst in TOPICS.items():
        cmd += ['-r', '%s:=%s' % (src, dst)]
    return name, subprocess.Popen(
        cmd, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, start_new_session=True)


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    node = rclpy.create_node('t7_harness_%d' % PID)
    procs = {}
    got = []
    node.create_subscription(Stimulus, TOPICS['/stimuli'], got.append, RELIABLE)
    pubs = {
        'odom': node.create_publisher(Odometry, TOPICS['/diff_cont/odom'], RELIABLE),
        'js': node.create_publisher(
            JointState, TOPICS['/dome_state_broadcaster/joint_states'], RELIABLE),
        'cmd': node.create_publisher(
            Float64MultiArray, TOPICS['/dome_controller/commands'], RELIABLE),
        'btn': node.create_publisher(
            DomeButtonSequence, TOPICS['/dome/buttons/sequence'], RELIABLE),
    }
    for exe in ('drive_stimulus', 'dome_stimulus', 'button_stimulus'):
        procs[exe] = _start(exe)
    try:
        # Wait until every adapter has subscribed to its private input.
        end = time.monotonic() + 20.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if all(p.get_subscription_count() >= 1 for p in pubs.values()):
                break
        else:
            pytest.fail('adapters did not come up')
        yield node, pubs, got, procs
    finally:
        # `ros2 run` is a wrapper: signal its whole process group so the node dies too.
        for _, proc in procs.values():
            os.killpg(proc.pid, signal.SIGINT)
        for _, proc in procs.values():
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
        node.destroy_node()
        rclpy.shutdown()


def _spin(node, seconds, until=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
        if until is not None and until():
            return


def _of(got, source):
    return [m for m in got if m.source == source]


def test_drive_adapter(ros):
    node, pubs, got, procs = ros
    name = procs['drive_stimulus'][0]
    got.clear()
    msg = Odometry()
    msg.twist.twist.linear.x = 0.5
    end = time.monotonic() + 1.5
    while time.monotonic() < end:
        msg.header.stamp = node.get_clock().now().to_msg()
        pubs['odom'].publish(msg)
        _spin(node, 0.05)
    msg.twist.twist.linear.x = 0.0
    end = time.monotonic() + 2.5
    while time.monotonic() < end:
        pubs['odom'].publish(msg)
        _spin(node, 0.05)
    mine = _of(got, name)
    kinds = [m.kind for m in mine]
    assert kinds.count('being_driven') == 2, kinds
    assert kinds.count('stopped') == 1 and kinds[-1] == 'stopped', kinds
    first = mine[0]
    assert abs(first.intensity - 0.5) < 1e-6
    assert math.isnan(first.bearing_rad) and first.confidence == 1.0 and first.ttl_s == 0.0
    assert first.header.stamp.sec > 0


def test_button_adapter(ros):
    node, pubs, got, procs = ros
    name = procs['button_stimulus'][0]
    got.clear()
    msg = DomeButtonSequence(value=4, taps='LRR', id=1, source=1)
    pubs['btn'].publish(msg)
    _spin(node, 5.0, until=lambda: _of(got, name))
    mine = _of(got, name)
    assert len(mine) == 1
    m = mine[0]
    assert m.kind == 'button_sequence' and m.subject == 'LRR'
    assert abs(m.intensity - 0.5) < 1e-6
    assert json.loads(m.extra) == {'value': 4, 'id': 1, 'source': 1}
    assert math.isnan(m.bearing_rad)


def _joint_state(node, pos):
    js = JointState()
    js.header.stamp = node.get_clock().now().to_msg()
    js.name = ['other_joint', 'dome_joint']
    js.position = [9.0, pos]
    return js


def _feed_dome(node, pubs, seconds, omega, command, start):
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        t = time.monotonic() - t0
        if command is not None:
            pubs['cmd'].publish(Float64MultiArray(data=[command]))
        pubs['js'].publish(_joint_state(node, (start + omega * t) % (2 * math.pi)))
        _spin(node, 0.1)
    return start + omega * seconds


def test_dome_adapter(ros):
    node, pubs, got, procs = ros
    name = procs['dome_stimulus'][0]
    got.clear()
    # Commanded motion: nothing.
    pos = _feed_dome(node, pubs, 3.0, 0.5, 0.5, 6.0)
    assert _of(got, name) == []
    # Command ends; the dome stands still for a while, then is pushed at 0.5 rad/s.
    pos = _feed_dome(node, pubs, 3.0, 0.0, 0.0, pos)      # past settle_s (1.5), at rest
    assert _of(got, name) == []
    _feed_dome(node, pubs, 2.5, 0.5, 0.0, pos)
    mine = _of(got, name)
    assert 1 <= len(mine) <= 4
    m = mine[0]
    assert m.kind == 'dome_pushed' and m.source == name
    assert abs(m.intensity - 0.5 / 6.65) < 0.04
    assert math.isnan(m.bearing_rad)


def test_publishers_are_only_stimuli(ros):
    node, _, _, procs = ros
    for exe, (name, _) in procs.items():
        end = time.monotonic() + 10.0
        topics = set()
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            topics = {t for t, _ in node.get_publisher_names_and_types_by_node(name, '/')}
            if P + '/stimuli' in topics:
                break
        assert P + '/stimuli' in topics, (exe, topics)
        assert topics <= ALLOWED_PUBLISHERS, (exe, topics)
        assert procs[exe][1].poll() is None


def _run_node(executable, *params):
    name = '%s_p%d' % (executable, PID)
    cmd = ['ros2', 'run', 'r2_stimulus', executable, '--ros-args', '-r', '__node:=' + name]
    for src, dst in TOPICS.items():
        cmd += ['-r', '%s:=%s' % (src, dst)]
    for key, value in params:
        cmd += ['-p', '%s:=%s' % (key, value)]
    return name, subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True,
        text=True)


def _stop(proc):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
    try:
        return proc.communicate(timeout=10)[0]
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        return proc.communicate()[0]


def test_integer_parameters_are_accepted_and_changes_refused():
    name, proc = _run_node('dome_stimulus', ('settle_s', '1'), ('pushed_omega', '1'))
    try:
        end = time.monotonic() + 15.0
        refused = None
        while time.monotonic() < end and refused is None:
            res = subprocess.run(
                ['ros2', 'param', 'set', '/' + name, 'settle_s', '2.0'],
                capture_output=True, text=True, timeout=20)
            if 'not found' not in (res.stdout + res.stderr).lower() and (
                    res.stdout + res.stderr).strip():
                refused = res.stdout + res.stderr
            else:
                time.sleep(0.3)
        assert proc.poll() is None, 'node died on integer parameters'
        assert refused and 'read at startup' in refused, refused
    finally:
        _stop(proc)


@pytest.mark.parametrize('bad', ['true', '-1', 'abc'])
def test_bad_parameter_exits_with_error(bad):
    name, proc = _run_node('drive_stimulus', ('moving_mps', bad))
    try:
        proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        pytest.fail('node did not exit on moving_mps:=%s' % bad)
    finally:
        out = _stop(proc)
    assert proc.returncode == 1, out
    assert 'cannot start' in out
