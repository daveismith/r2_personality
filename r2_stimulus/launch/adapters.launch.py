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

"""Start the three stimulus adapters (drive, dome, buttons) with config/stimulus.yaml."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

NODES = ('drive_stimulus', 'dome_stimulus', 'button_stimulus')


def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    config = LaunchConfiguration('config')
    nodes = [
        Node(package='r2_stimulus', executable=name, name=name, output='screen',
             parameters=[config, {'use_sim_time': use_sim_time}])
        for name in NODES
    ]
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use sim time if true (required with the sim)'),
        DeclareLaunchArgument(
            'config',
            default_value=os.path.join(
                get_package_share_directory('r2_stimulus'), 'config', 'stimulus.yaml'),
            description='Path of the stimulus.yaml with the adapter thresholds'),
    ] + nodes)
