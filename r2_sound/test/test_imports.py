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


"""The launch file and the parameters load no audio library (soundfile, PortAudio)."""

import os
import subprocess
import sys

import pytest

LAUNCH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'launch',
                      'sound_player.launch.py')
CHECK = """
import importlib.util, sys
{load}
loaded = [m for m in ('sounddevice', 'soundfile', '_sounddevice') if m in sys.modules]
print(','.join(loaded))
"""
LOAD_LAUNCH = """
spec = importlib.util.spec_from_file_location('sound_player_launch', {path!r})
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.generate_launch_description()
"""


@pytest.mark.parametrize('load', [
    'import r2_sound.parameters',
    'import r2_sound.backends, r2_sound.decode',
    LOAD_LAUNCH.format(path=LAUNCH)], ids=['parameters', 'backends-decode', 'launch'])
def test_no_audio_library_is_imported(load):
    result = subprocess.run([sys.executable, '-c', CHECK.format(load=load)],
                            capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == ''
