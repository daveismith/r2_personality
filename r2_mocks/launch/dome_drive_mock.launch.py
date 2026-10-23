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

"""Launch dome_drive_mock, the dome's drive (velocity controller and motor) without the sim."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# (argument, type, default, description); each is also the node parameter of the same name.
ARGUMENTS = (
    ('use_sim_time', bool, 'false', 'Use sim time if true'),
    ('ideal', bool, 'false', 'A pure integrator like Gazebo: no dead band, lag, coast or quirks'),
    ('firmware_quirks', bool, 'true', 'The firmware reads 9 deg long across each crossing of 0'),
    ('initial_rad', float, '0.0', 'Starting position in rad'),
    ('state_hz', float, '10.0', 'Rate of ~/joint_states (above 0, up to 30)'),
    ('publish_30hz', bool, 'false', 'Also publish ~/joint_states_30hz'),
    ('body_speed_m_s', float, '0.0', 'Above 0: publish ~/odom with this linear speed'),
)


def generate_launch_description():
    node = Node(
        package='r2_mocks',
        executable='dome_drive_mock',
        name=LaunchConfiguration('node_name'),
        output='screen',
        parameters=[{name: ParameterValue(LaunchConfiguration(name), value_type=kind)
                     for name, kind, _, _ in ARGUMENTS}],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'node_name', default_value='dome_drive_mock',
            description='Node name; the topics are ~/joint_states and so on under it'),
        *(DeclareLaunchArgument(name, default_value=default, description=text)
          for name, _, default, text in ARGUMENTS),
        node,
    ])
