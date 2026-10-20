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


"""The launch file is importable and starts the two arbiters (it is not launched here)."""

import importlib.util
import os

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'launch',
                    'expression.launch.py')


def test_launch_description_has_the_arguments_and_both_nodes():
    spec = importlib.util.spec_from_file_location('expression_launch', PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    entities = module.generate_launch_description().entities
    args = {e.name for e in entities if isinstance(e, DeclareLaunchArgument)}
    assert args == {'use_sim_time', 'config', 'device'}
    nodes = [e for e in entities if isinstance(e, Node)]
    assert sorted(n.node_executable for n in nodes) == ['led_arbiter', 'magic_arbiter']
