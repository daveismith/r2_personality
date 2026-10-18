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

"""Shared plumbing of the three adapter nodes: they publish /stimuli and nothing else."""

import math
import sys

from astromech_msgs.msg import Stimulus
from r2_stimulus.stimulate import _try_shutdown
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

# Best effort matches both reliable and best-effort publishers of the drive topics.
SENSOR_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
EVENT_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


class AdapterNode(Node):
    """A node whose only publisher (besides rosout) is /stimuli."""

    def __init__(self, name):
        super().__init__(name)
        self._pub = self.create_publisher(
            Stimulus, '/stimuli', QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))

    def emit(self, kind, intensity, subject='', extra=''):
        """Publish one Stimulus (bearing unknown, confidence 1, default ttl) and log it."""
        msg = Stimulus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.kind = kind
        msg.source = self.get_name()
        msg.intensity = float(intensity)
        msg.confidence = 1.0
        msg.subject = subject
        msg.bearing_rad = math.nan
        msg.ttl_s = 0.0
        msg.extra = extra
        self._pub.publish(msg)
        self.get_logger().info('emit %s intensity=%.2f%s' % (
            kind, msg.intensity, ' subject=%s' % subject if subject else ''))


def run(node_class, args=None):
    """Spin node_class until SIGINT/SIGTERM; return the exit code."""
    rclpy.init(args=args)
    try:
        node = node_class()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            # spin_once with a timeout, so SIGINT/SIGTERM are noticed on an idle node.
            while rclpy.ok():
                executor.spin_once(timeout_sec=0.2)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        except RCLError:
            if rclpy.ok():
                raise
        finally:
            executor.shutdown(timeout_sec=0)
            node.destroy_node()
    finally:
        _try_shutdown()
    return 0


def main_for(node_class):
    """Exit with run()'s code."""
    sys.exit(run(node_class))
