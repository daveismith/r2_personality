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
Launch ``sound_player`` (plan 5.7): every parameter of plan 5.3 is an argument.

``backend:=wav`` writes what would play under ``wav_dir`` (the sim and the agents);
``backend:=sounddevice`` plays through PortAudio (``device``: a name substring, '' the
default). ``catalogue`` '' is the installed ``clips.yaml``. The arguments come from
:mod:`r2_sound.parameters`, which imports nothing third-party: launching never loads
``soundfile`` or PortAudio.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from r2_sound.parameters import DESCRIPTIONS, NODE_NAME, PARAMETERS


def generate_launch_description():
    arguments = [DeclareLaunchArgument('use_sim_time', default_value='false',
                                       description='Use sim time (required with the sim)')]
    parameters = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                                 value_type=bool)}
    for name, default in PARAMETERS.items():
        text = str(default).lower() if isinstance(default, bool) else str(default)
        arguments.append(DeclareLaunchArgument(name, default_value=text,
                                               description=DESCRIPTIONS[name]))
        parameters[name] = ParameterValue(LaunchConfiguration(name),
                                          value_type=type(default))
    return LaunchDescription(arguments + [
        Node(package='r2_sound', executable='sound_player', name=NODE_NAME, output='screen',
             parameters=[parameters])])
