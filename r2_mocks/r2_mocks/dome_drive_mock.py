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
``dome_drive_mock``: the dome's drive as the personality stack sees it, for development without it.

Subscribes ``/dome_controller/commands`` (``std_msgs/Float64MultiArray``, ``data[0]`` rad/s),
holds the last command (the forward controller does; the converter's final 0 is what stops the
dome), runs :class:`r2_mocks.dome_plant.DomePlant` at 30 Hz on the node clock and publishes
``~/joint_states`` (``dome_joint``, position only) at ``state_hz``, ``~/joint_states_30hz``
when ``publish_30hz`` is set, and ``~/odom`` (10 Hz) when ``body_speed_m_s`` is above 0, for
the suppression tests. It does not run the converter: node tests start the real one.
"""

import math
import sys

from nav_msgs.msg import Odometry
from r2_mocks.dome_plant import DomePlant
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINT = 'dome_joint'
PLANT_HZ = 30.0
ODOM_HZ = 10.0
MAX_STATE_HZ = PLANT_HZ
MAX_DT_S = 4.0 / PLANT_HZ        # a sim clock that jumps (it starts at 0) must not jump the dome
COMMAND_TOPIC = '/dome_controller/commands'


class DomeDriveMock(Node):
    """The mock drive node."""

    def __init__(self):
        super().__init__('dome_drive_mock')
        loose = ParameterDescriptor(dynamic_typing=True)
        for name, default in (('ideal', False), ('firmware_quirks', True),
                              ('publish_30hz', False), ('initial_rad', 0.0),
                              ('state_hz', 10.0), ('body_speed_m_s', 0.0)):
            self.declare_parameter(name, default, loose)
        ideal, quirks, both = (self._bool_param(n) for n in
                               ('ideal', 'firmware_quirks', 'publish_30hz'))
        initial_rad = self._number_param('initial_rad')
        state_hz = self._number_param('state_hz')
        body_speed = self._number_param('body_speed_m_s')
        if not 0.0 < state_hz <= MAX_STATE_HZ:
            raise ValueError('state_hz must be in (0, %g], got %r' % (MAX_STATE_HZ, state_hz))
        if body_speed < 0.0:
            raise ValueError('body_speed_m_s must be >= 0, got %r' % body_speed)
        self.plant = DomePlant(ideal=ideal, firmware_quirks=quirks, initial_rad=initial_rad)
        self._command = 0.0
        self._last_time = self.get_clock().now().nanoseconds / 1e9
        self._body_speed = body_speed

        self.create_subscription(
            Float64MultiArray, COMMAND_TOPIC, self._on_command,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self._state_pub = self.create_publisher(JointState, '~/joint_states', 10)
        self._state_30hz_pub = (
            self.create_publisher(JointState, '~/joint_states_30hz', 10) if both else None)
        self._odom_pub = (
            self.create_publisher(Odometry, '~/odom', 10) if body_speed > 0.0 else None)
        self.create_timer(1.0 / PLANT_HZ, self._on_plant)
        self.create_timer(1.0 / state_hz, self._on_state)
        if self._odom_pub is not None:
            self.create_timer(1.0 / ODOM_HZ, self._on_odom)
        self.get_logger().info(
            'dome drive mock up as %s (%s, quirks %s, state_hz %g, start %.3f rad, '
            'body speed %g m/s)' % (
                self.get_fully_qualified_name(), 'ideal' if ideal else 'real plant',
                quirks and not ideal, state_hz, initial_rad, body_speed))

    def _bool_param(self, name):
        value = self.get_parameter(name).value
        if not isinstance(value, bool):
            raise ValueError('%s must be a bool, got %r' % (name, value))
        return value

    def _number_param(self, name):
        value = self.get_parameter(name).value
        if isinstance(value, bool) or not isinstance(value, (int, float)) \
                or not math.isfinite(value):
            raise ValueError('%s must be a finite number, got %r' % (name, value))
        return float(value)

    def _on_command(self, msg):
        if msg.data:
            self._command = float(msg.data[0])

    def _on_plant(self):
        now = self.get_clock().now().nanoseconds / 1e9
        self.plant.step(self._command, min(now - self._last_time, MAX_DT_S))
        self._last_time = now
        if self._state_30hz_pub is not None:
            self._state_30hz_pub.publish(self._joint_state())

    def _on_state(self):
        self._state_pub.publish(self._joint_state())

    def _on_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.twist.twist.linear.x = self._body_speed
        self._odom_pub.publish(msg)

    def _joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [JOINT]
        msg.position = [self.plant.reported_rad]
        return msg


def _try_shutdown():
    """Shut rclpy down unless a signal handler already did."""
    try:
        rclpy.try_shutdown()
    except RCLError:
        pass


def main(args=None):
    """Run the node until SIGINT/SIGTERM; exit 1 on a bad parameter."""
    rclpy.init(args=args)
    try:
        try:
            node = DomeDriveMock()
        except ValueError as exc:
            rclpy.logging.get_logger('dome_drive_mock').error('cannot start: %s' % exc)
            return 1
        except KeyboardInterrupt:       # a signal while the node was still being built
            return 0
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


if __name__ == '__main__':
    sys.exit(main())
