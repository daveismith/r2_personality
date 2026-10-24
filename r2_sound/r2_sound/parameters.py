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
``sound_player``'s parameters (plan 5.3): names, defaults, ranges and descriptions.

Pure Python with no third-party imports, so the launch file can declare its arguments
without importing the node (and with it ``soundfile``, ``sounddevice`` and PortAudio).
"""

NODE_NAME = 'sound_player'

# name: (default, description). Every one is read at startup; a later `ros2 param set` of
# any of them is refused.
_TABLE = {
    'catalogue': ('', "the clips.yaml to load ('' the installed one)"),
    'sounds_dir': ('/sounds', 'where the catalogue paths are relative to'),
    'backend': ('sounddevice', 'sounddevice | wav | null'),
    'device': ('', "output device name substring ('' the default)"),
    'require_device': (False, 'exit 1 when no device matches, instead of running silent '
                              'on the null backend'),
    'sample_rate': (44100, 'output rate, Hz'),
    'cache_mb': (64.0, 'decode cache cap, MB; a clip over half of it is not cached'),
    'state_hz': (5.0, '~/state rate while playing (1 Hz while idle)'),
    'level_hz': (30.0, 'level topics rate while playing'),
    'stream_linger_s': (2.0, 'the sounddevice stream closes this long after the last play'),
    'fade_ms': (20.0, 'fade-out on stop, replace and the duration cut'),
    'dome_level_topic': ('/dome/audio/level', "the dome's UInt8 audio level input"),
    'level_floor_dbfs': (-40.0, 'level mapped to 0 on the dome'),
    'level_ceiling_dbfs': (-6.0, 'level mapped to 255 on the dome'),
    'wav_dir': ('/ws/.audio/out', 'where the wav backend writes'),
    'allow_duplicate': (False, 'start even when another sound_player is on the graph'),
}
PARAMETERS = {name: default for name, (default, _) in _TABLE.items()}
DESCRIPTIONS = {name: text for name, (_, text) in _TABLE.items()}
# name: (low, high) for the numbers, both inclusive. ListSounds.sample_rate is a uint16.
RANGES = {
    'sample_rate': (8000, 48000), 'cache_mb': (0.0, 4096.0), 'state_hz': (0.1, 30.0),
    'level_hz': (1.0, 100.0), 'stream_linger_s': (0.0, 600.0), 'fade_ms': (0.0, 1000.0),
    'level_floor_dbfs': (-120.0, 0.0), 'level_ceiling_dbfs': (-120.0, 0.0),
}
