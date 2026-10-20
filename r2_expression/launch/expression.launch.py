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


"""Launch the LED and Magic Panel arbiters (the only clients of the dome's light services)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# With the sim, use_sim_time must be true. `device` is the dome node: /r2_dome on the droid,
# the mock's node name (for example /dome_mock) in the sim or when testing next to a real dome.
def generate_launch_description():
    parameters = [{
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        'config': ParameterValue(LaunchConfiguration('config'), value_type=str),
        'device': ParameterValue(LaunchConfiguration('device'), value_type=str),
    }]
    nodes = [Node(package='r2_expression', executable=kind, name=kind, output='screen',
                  parameters=parameters)
             for kind in ('led_arbiter', 'magic_arbiter')]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true (required with the sim)'),
        DeclareLaunchArgument(
            'config',
            default_value=os.path.join(
                get_package_share_directory('r2_expression'), 'config', 'expression.yaml'),
            description='Path of the expression.yaml to load (read at startup)'),
        DeclareLaunchArgument(
            'device',
            default_value='/r2_dome',
            description='The dome device node whose leds/* and magic/* services are used'),
        *nodes,
    ])
