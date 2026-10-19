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
Run the single-process ``stimulus_adapters`` against private topic and node names.

The robot's real topics may be live on this graph, so every topic is remapped to a name
private to this test and every node is renamed per node (``drive_stimulus:__node:=...``).
"""

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

PID = os.getpid()
P = '/t8_%d' % PID
ODOM = P + '/odom'
STIMULI = P + '/stimuli'
RELIABLE = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
BASES = ('drive_stimulus', 'dome_stimulus', 'button_stimulus')
NAMES = {base: '%s_s%d' % (base, PID) for base in BASES}


def _spin(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.01)


@pytest.fixture(scope='module')
def ros():
    cmd = ['ros2', 'run', 'r2_stimulus', 'stimulus_adapters', '--ros-args']
    for base, name in NAMES.items():
        cmd += ['-r', '%s:__node:=%s' % (base, name)]
    for src, dst in (
            ('/diff_cont/odom', ODOM),
            ('/dome_state_broadcaster/joint_states', P + '/dome_js'),
            ('/dome_controller/commands', P + '/dome_cmd'),
            ('/dome/buttons/sequence', P + '/buttons'),
            ('/stimuli', STIMULI)):
        cmd += ['-r', '%s:=%s' % (src, dst)]
    proc = subprocess.Popen(
        cmd, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('t8_harness_%d' % PID)
    got = []
    node.create_subscription(Stimulus, STIMULI, got.append, RELIABLE)
    pub = node.create_publisher(Odometry, ODOM, RELIABLE)
    btn = node.create_publisher(DomeButtonSequence, P + '/buttons', RELIABLE)
    try:
        end = time.monotonic() + 20.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if pub.get_subscription_count() >= 1 and btn.get_subscription_count() >= 1:
                break
        else:
            pytest.fail('stimulus_adapters did not come up')
        yield node, (pub, btn), got, proc
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
        node.destroy_node()
        rclpy.shutdown()


def test_three_nodes_in_one_process(ros):
    node, _, _, proc = ros
    end = time.monotonic() + 10.0
    names = set()
    while time.monotonic() < end and not set(NAMES.values()) <= names:
        rclpy.spin_once(node, timeout_sec=0.05)
        names = set(node.get_node_names())
    assert set(NAMES.values()) <= names, names
    assert proc.poll() is None


def test_drive_detection_through_raw_path(ros):
    node, (pub, _), got, _ = ros
    name = NAMES['drive_stimulus']
    got.clear()
    msg = Odometry()
    msg.twist.twist.linear.x = 0.5
    for speed, seconds in ((0.5, 1.5), (0.0, 2.5)):
        msg.twist.twist.linear.x = speed
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            _spin(node, 1.0 / 30.0)
    kinds = [m.kind for m in got if m.source == name]
    assert kinds.count('being_driven') >= 1, kinds
    assert kinds.count('stopped') == 1 and kinds[-1] == 'stopped', kinds
    assert all(m.source in NAMES.values() for m in got)


def test_button_sequences_published_while_polling_are_not_lost(ros):
    node, (_, btn), got, _ = ros
    name = NAMES['button_stimulus']
    got.clear()
    for i in range(5):       # 10 ms apart: all five arrive inside one 100 ms poll period
        btn.publish(DomeButtonSequence(value=i, taps='LR', id=i, source=1))
        _spin(node, 0.01)
    end = time.monotonic() + 5.0
    while time.monotonic() < end and len([m for m in got if m.source == name]) < 5:
        _spin(node, 0.05)
    mine = [m for m in got if m.source == name]
    assert len(mine) == 5, len(mine)
