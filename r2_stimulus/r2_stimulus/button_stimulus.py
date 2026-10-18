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

"""button_stimulus: one button_sequence per DomeButtonSequence message."""

from astromech_msgs.msg import DomeButtonSequence
from r2_stimulus.adapter_base import AdapterNode
from r2_stimulus.adapter_base import EVENT_QOS
from r2_stimulus.adapter_base import main_for
from r2_stimulus.detect import button_stimulus


class ButtonStimulus(AdapterNode):
    """Listen to /dome/buttons/sequence only."""

    def __init__(self):
        super().__init__('button_stimulus')
        self.create_subscription(
            DomeButtonSequence, '/dome/buttons/sequence', self._on_sequence, EVENT_QOS)

    def _on_sequence(self, msg):
        kind, subject, intensity, extra = button_stimulus(
            msg.taps, msg.value, msg.id, msg.source)
        self.emit(kind, intensity, subject=subject, extra=extra)


def main():
    """Entry point."""
    main_for(ButtonStimulus)


if __name__ == '__main__':
    main()
