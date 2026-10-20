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

"""Launch dome_mock, the dome device's LED and Magic Panel ROS surface."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# With a real dome on the graph, give the mock another node_name (it moves every service and
# state topic with it); with the sim, use_sim_time must be true.
def generate_launch_description():
    node = Node(
        package='r2_mocks',
        executable='dome_mock',
        name=LaunchConfiguration('node_name'),
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'config': ParameterValue(LaunchConfiguration('config'), value_type=str),
            'state_hz': ParameterValue(LaunchConfiguration('state_hz'), value_type=float),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'config',
            default_value=os.path.join(
                get_package_share_directory('r2_mocks'), 'config', 'dome_mock.yaml'),
            description='Catalogue to serve (generate one with tools/gen_dome_mock_yaml.py)'),
        DeclareLaunchArgument(
            'state_hz',
            default_value='1.0',
            description='Rate of the leds/states and magic/states topics (above 0, up to 10)'),
        DeclareLaunchArgument(
            'node_name',
            default_value='r2_dome',
            description='Node name; the services are ~/leds/... under it'),
        node,
    ])
