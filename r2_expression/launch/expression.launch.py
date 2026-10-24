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
Launch the arbiters: LED, Magic Panel, dome and sound.

The LED and Magic Panel arbiters are the only clients of the dome's light services.

``dome`` (true) adds ``dome_arbiter``, which drives the dome through ``dome_twist_converter``
(phase 2 plan 5.7); the topic arguments are its parameters. The body's motion comes from
``cmd_vel_topic`` (the drive's command, silent unless someone drives); set ``odom_topic`` to
read odometry instead (it also sees a base rolled by hand, at about 3 % of a Pi core).

``sound`` (true) adds ``sound_arbiter``, the only client of ``sound_player``'s play, vocalize and
stop services (phase 3 plan 5.4); ``sound_device`` is the player's node name.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from r2_expression.arbiter_node import DOME_PARAMETERS

# The dome arbiter's topics, as launch arguments (the defaults are the node's own).
DOME_TOPICS = (
    ('command_topic', "dome_twist_converter's personality input"),
    ('status_topic', "dome_twist_converter's DomeCommandStatus"),
    ('joint_states_topic', 'the dome joint position'),
    ('cmd_vel_topic', "the drive's TwistStamped command (the dome holds off while it drives)"),
    ('odom_topic', "the body odometry, used instead of cmd_vel_topic when set ('': off)"),
)


# With the sim, use_sim_time must be true. `device` is the dome node: /r2_dome on the droid,
# the mock's node name (for example /dome_mock) in the sim or when testing next to a real dome.
def generate_launch_description():
    parameters = [{
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        'config': ParameterValue(LaunchConfiguration('config'), value_type=str),
        'device': ParameterValue(LaunchConfiguration('device'), value_type=str),
    }]
    common = {key: parameters[0][key] for key in ('use_sim_time', 'config')}
    nodes = [Node(package='r2_expression', executable=kind, name=kind, output='screen',
                  parameters=parameters)
             for kind in ('led_arbiter', 'magic_arbiter')]
    nodes.append(Node(
        package='r2_expression', executable='dome_arbiter', name='dome_arbiter',
        output='screen', condition=IfCondition(LaunchConfiguration('dome')),
        parameters=[{**common, **{
            name: ParameterValue(LaunchConfiguration(name), value_type=str)
            for name, _ in DOME_TOPICS}}]))
    nodes.append(Node(
        package='r2_expression', executable='sound_arbiter', name='sound_arbiter',
        output='screen', condition=IfCondition(LaunchConfiguration('sound')),
        parameters=[{**common, 'device': ParameterValue(
            LaunchConfiguration('sound_device'), value_type=str)}]))
    dome_args = [DeclareLaunchArgument(name, default_value=DOME_PARAMETERS[name],
                                       description=text)
                 for name, text in DOME_TOPICS]

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
        DeclareLaunchArgument(
            'dome',
            default_value='true',
            description='Start dome_arbiter (the dome motion channel)'),
        DeclareLaunchArgument(
            'sound',
            default_value='true',
            description='Start sound_arbiter (the sound channel)'),
        DeclareLaunchArgument(
            'sound_device',
            default_value='/sound_player',
            description='The sound player node whose list, play, vocalize and stop services '
                        'and state topic are used'),
        *dome_args,
        *nodes,
    ])
