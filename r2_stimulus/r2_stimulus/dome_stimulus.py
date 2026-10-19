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

"""dome_stimulus: dome position and command in, dome_pushed out."""

from r2_stimulus.adapter_base import AdapterNode
from r2_stimulus.adapter_base import main_for
from r2_stimulus.adapter_base import SENSOR_QOS
from r2_stimulus.detect import DomePushDetector
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

DOME_JOINT = 'dome_joint'


class DomeStimulus(AdapterNode):
    """Listen to the dome's joint state and velocity command only."""

    def __init__(self):
        super().__init__('dome_stimulus')
        params = self.declare_numbers({
            'pushed_omega': 0.3, 'commanded_eps': 0.02, 'pushed_after_s': 0.3,
            'max_omega': 6.65, 'command_timeout_s': 0.5, 'pushed_period_s': 1.0,
            'settle_s': 1.5, 'window_s': 0.4, 'min_span_s': 0.25, 'poll_period_s': 0.1,
        }, may_be_zero=('pushed_after_s', 'settle_s', 'poll_period_s'))
        self._detector = DomePushDetector(**params)
        self.create_subscription(
            JointState, '/dome_state_broadcaster/joint_states', self._on_state, SENSOR_QOS)
        self.create_subscription(
            Float64MultiArray, '/dome_controller/commands', self._on_command, SENSOR_QOS)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_command(self, msg):
        if msg.data:
            self._detector.command(float(msg.data[0]), self._now())

    def _on_state(self, msg):
        if DOME_JOINT not in msg.name:
            return
        i = msg.name.index(DOME_JOINT)
        if i >= len(msg.position):
            return
        for kind, intensity in self._detector.position(float(msg.position[i]), self._now()):
            self.emit(kind, intensity)


def main():
    """Entry point."""
    main_for(DomeStimulus)


if __name__ == '__main__':
    main()
