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
import threading
import time

from astromech_msgs.msg import Stimulus
from r2_stimulus.stimulate import _try_shutdown
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

# Best effort matches both reliable and best-effort publishers of the drive topics. Depth 1:
# the nodes are polled, so only the latest sample of a stream is wanted at each poll.
SENSOR_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
EVENT_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


class AdapterNode(Node):
    """A node whose only publisher (besides rosout) is /stimuli."""

    def __init__(self, name):
        super().__init__(name)
        self.poll_period_s = 0.0
        self.handled = 0     # callbacks run so far; the polled drain stops when it is unchanged
        self._pub = self.create_publisher(
            Stimulus, '/stimuli', QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))

    def create_subscription(self, msg_type, topic, callback, qos_profile, **kwargs):
        """Create the subscription with a callback that counts its calls (see ``handled``)."""
        def counted(msg):
            self.handled += 1
            callback(msg)
        return super().create_subscription(msg_type, topic, counted, qos_profile, **kwargs)

    def declare_numbers(self, defaults, may_be_zero=()):
        """
        Declare numeric parameters (ints accepted) and return their values as floats.

        A bool or non-number, or a value that is not finite and > 0 (>= 0 for the names
        in ``may_be_zero``), raises ValueError. Call it once, in the constructor: the
        values are read at startup and any later ``ros2 param set`` is refused.
        """
        loose = ParameterDescriptor(dynamic_typing=True)
        values = {}
        for name, default in defaults.items():
            self.declare_parameter(name, default, loose)
            value = self.get_parameter(name).value
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError('%s must be a number, got %r' % (name, value))
            value = float(value)
            low_ok = name in may_be_zero
            if not math.isfinite(value) or value < 0.0 or (value == 0.0 and not low_ok):
                raise ValueError('%s must be %s, got %r' % (
                    name, '>= 0' if low_ok else '> 0', value))
            values[name] = value
        if 'poll_period_s' in values:
            self.poll_period_s = values.pop('poll_period_s')
        # Registered after TimeSource's callback, so it runs first and a refused
        # use_sim_time never reaches TimeSource.
        self.add_on_set_parameters_callback(self._refuse_changes)
        return values

    @staticmethod
    def _refuse_changes(params):
        return SetParametersResult(successful=False, reason='read at startup; restart the node')

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


# Blocking mode (poll_period_s 0) only: how long an idle spin_once waits, for signal handling.
SPIN_TIMEOUT_S = 0.5
# Polled mode: at most this many callbacks are drained per poll.
DRAIN_LIMIT = 32


def _spin(executor, nodes):
    """
    Run the executor until shutdown, polled or blocking per the nodes' poll_period_s.

    Polled: sleep ``poll_period_s``, then run ready callbacks with a zero timeout until a
    call runs none (at most DRAIN_LIMIT). A message costs no wake-up of its own; with
    depth-1 subscriptions only the newest sample of a stream is seen at each poll.
    """
    period = min(n.poll_period_s for n in nodes)
    try:
        while rclpy.ok():
            if period <= 0.0:
                executor.spin_once(timeout_sec=SPIN_TIMEOUT_S)
                continue
            time.sleep(period)
            for _ in range(DRAIN_LIMIT):
                before = sum(n.handled for n in nodes)
                executor.spin_once(timeout_sec=0.0)
                if sum(n.handled for n in nodes) == before:
                    break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if rclpy.ok():
            raise


def run_nodes(node_classes, args=None, threaded=False):
    """
    Spin one node per class until SIGINT/SIGTERM; return the exit code.

    With ``threaded`` each node gets its own SingleThreadedExecutor in its own thread: a
    shared executor rebuilds a wait set of every node's entities on every wake-up, which
    costs more than three processes (measured). Otherwise all nodes share one executor.
    """
    rclpy.init(args=args)
    nodes = []
    try:
        try:
            for node_class in node_classes:
                nodes.append(node_class())
        except ValueError as exc:
            rclpy.logging.get_logger(node_class.__name__).error('cannot start: %s' % exc)
            return 1
        groups = [[n] for n in nodes] if threaded else [nodes]
        executors = []
        for group in groups:
            executors.append(SingleThreadedExecutor())
            for node in group:
                executors[-1].add_node(node)
        try:
            if threaded:
                threads = [threading.Thread(target=_spin, args=(e, g))
                           for e, g in zip(executors, groups)]
                for t in threads:
                    t.start()
                try:
                    while rclpy.ok():
                        time.sleep(SPIN_TIMEOUT_S)
                except KeyboardInterrupt:
                    pass
                for t in threads:
                    t.join()
            else:
                _spin(executors[0], nodes)
        finally:
            for e in executors:
                e.shutdown(timeout_sec=0)
    finally:
        for node in nodes:
            node.destroy_node()
        _try_shutdown()
    return 0


def run(node_class, args=None):
    """Spin node_class until SIGINT/SIGTERM; return the exit code."""
    return run_nodes([node_class], args)


def main_for(node_class):
    """Exit with run()'s code."""
    sys.exit(run(node_class))
