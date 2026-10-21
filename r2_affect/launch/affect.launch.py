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

"""Launch the r2_affect node (the mood model: /stimuli in, /mood/state out)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# With the sim, use_sim_time must be true: the model's relax, dwell and lock timers then
# follow /clock and stand still while the sim is paused.
def generate_launch_description():
    node = Node(
        package='r2_affect',
        executable='r2_affect',
        name='r2_affect',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'config': ParameterValue(LaunchConfiguration('config'), value_type=str),
            'rate_hz': ParameterValue(LaunchConfiguration('rate_hz'), value_type=float),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true (required with the sim)'),
        DeclareLaunchArgument(
            'config',
            default_value=os.path.join(
                get_package_share_directory('r2_affect'), 'config', 'affect.yaml'),
            description='Path of the affect.yaml to load (reload with /mood/reload_config)'),
        DeclareLaunchArgument(
            'rate_hz',
            default_value='5.0',
            description='Tick and /mood/state publish rate'),
        node,
    ])
