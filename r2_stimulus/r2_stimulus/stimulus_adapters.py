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

"""stimulus_adapters: the three adapter nodes in one process (own executor and thread each)."""

import sys

from r2_stimulus.adapter_base import run_nodes
from r2_stimulus.button_stimulus import ButtonStimulus
from r2_stimulus.dome_stimulus import DomeStimulus
from r2_stimulus.drive_stimulus import DriveStimulus


def main():
    """Entry point."""
    sys.exit(run_nodes([DriveStimulus, DomeStimulus, ButtonStimulus], threaded=True))


if __name__ == '__main__':
    main()
