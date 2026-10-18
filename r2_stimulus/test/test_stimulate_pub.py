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

import math
import subprocess
import time

from astromech_msgs.msg import Stimulus
import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


def _run(extra_args):
    rclpy.init()
    node = rclpy.create_node('test_stimulate_pub_listener')
    got = []
    node.create_subscription(
        Stimulus, '/stimuli', got.append,
        QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
    cmd = ['ros2', 'run', 'r2_stimulus', 'stimulate', 'bump', '--intensity', '0.7',
           '--subject', 'dave', '--repeat', '3', '--every', '0.1'] + extra_args
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        end = time.monotonic() + 10.0
        while time.monotonic() < end and (len(got) < 3 or proc.poll() is None):
            rclpy.spin_once(node, timeout_sec=0.05)
        assert proc.poll() == 0, proc.communicate()
        return list(got)
    finally:
        if proc.poll() is None:
            proc.kill()
        node.destroy_node()
        rclpy.shutdown()


def test_bearing_given():
    got = _run(['--bearing', '1.2'])
    assert len(got) == 3
    for m in got:
        assert m.kind == 'bump' and m.subject == 'dave'
        assert abs(m.intensity - 0.7) < 1e-6
        assert abs(m.bearing_rad - 1.2) < 1e-6
        assert m.source == 'stimulate_cli'
        assert m.header.stamp.sec > 0


def test_bearing_omitted_is_nan():
    got = _run([])
    assert len(got) == 3
    assert all(math.isnan(m.bearing_rad) for m in got)
