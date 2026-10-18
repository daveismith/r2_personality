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

"""drive_stimulus: odometry speed in, being_driven and stopped out."""

import math

from nav_msgs.msg import Odometry
from r2_stimulus.adapter_base import AdapterNode
from r2_stimulus.adapter_base import main_for
from r2_stimulus.adapter_base import SENSOR_QOS
from r2_stimulus.detect import DriveDetector


class DriveStimulus(AdapterNode):
    """Listen to /diff_cont/odom only."""

    def __init__(self):
        super().__init__('drive_stimulus')
        params = self.declare_numbers({
            'moving_mps': 0.05, 'stop_after_s': 1.0, 'being_driven_period_s': 1.0,
            'max_speed_mps': 1.0, 'confirm_s': 0.25, 'wheel_separation': 0.572,
        }, may_be_zero=('confirm_s',))
        self._half_track = params.pop('wheel_separation') / 2.0
        self._detector = DriveDetector(**params)
        self.create_subscription(Odometry, '/diff_cont/odom', self._on_odom, SENSOR_QOS)

    def _on_odom(self, msg):
        twist = msg.twist.twist
        # A spin in place is driving too: the wheels move at |wz| * track / 2.
        speed = max(math.hypot(twist.linear.x, twist.linear.y),
                    abs(twist.angular.z) * self._half_track)
        now = self.get_clock().now().nanoseconds * 1e-9
        for kind, intensity in self._detector.update(speed, now):
            self.emit(kind, intensity)


def main():
    """Entry point."""
    main_for(DriveStimulus)


if __name__ == '__main__':
    main()
