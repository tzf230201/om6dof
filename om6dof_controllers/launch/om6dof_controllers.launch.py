# Copyright 2026 OM6DOF maintainers.
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
Load om6dof_controllers plugins into an already running controller_manager.

This launch file starts no hardware. om6dof_bringup owns ros2_control_node and
stays the only thing that opens U2D2; all this does is hand the controller
manager the parameters for these three controllers and ask it to load them.
"""

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# The controller_manager looks a controller's `type` up in its OWN parameters,
# and om6dof_bringup starts it with only its own controllers.yaml. Passing
# --param-file does not help: that lands on the controller's node, not on the
# manager. So the type has to be handed over explicitly, which is what
# --controller-type is for.
#
# The types are read back out of the param file rather than duplicated here. A
# hand-kept copy of that mapping is exactly the sort of thing that silently goes
# stale the first time a controller is added.
def _controller_types(param_file):
    try:
        with open(param_file) as handle:
            document = yaml.safe_load(handle) or {}
    except (OSError, yaml.YAMLError):
        return {}

    types = {}
    for node_params in document.values():
        if not isinstance(node_params, dict):
            continue
        manager = node_params.get('controller_manager', {})
        parameters = manager.get('ros__parameters', {}) if isinstance(manager, dict) else {}
        for name, spec in parameters.items():
            if isinstance(spec, dict) and 'type' in spec:
                types[name] = spec['type']
    return types


def _spawners(context, *args, **kwargs):
    controllers = LaunchConfiguration('controllers').perform(context).split()
    if not controllers:
        raise RuntimeError("launch argument 'controllers' is empty")

    manager = LaunchConfiguration('controller_manager').perform(context)
    param_file = LaunchConfiguration('param_file').perform(context)
    inactive = LaunchConfiguration('activate').perform(context).lower() not in ('true', '1')
    types = _controller_types(param_file)

    nodes = []
    for controller in controllers:
        arguments = [controller, '--controller-manager', manager, '--param-file', param_file]
        # A name the param file does not declare may still be loadable, if
        # whoever started the manager declared its type; let the manager decide.
        if controller in types:
            arguments += ['--controller-type', types[controller]]
        if inactive:
            arguments += ['--inactive']

        nodes.append(
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=arguments,
                output='screen',
            )
        )

    return nodes


def generate_launch_description():
    default_param_file = PathJoinSubstitution(
        [FindPackageShare('om6dof_controllers'), 'config', 'om6dof_controllers.yaml']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'controllers',
                default_value='om6dof_trajectory_controller',
                description=(
                    'Space-separated controller names to load. The effort-driven ones '
                    '(om6dof_gravity_compensation_controller, '
                    'om6dof_spring_actuator_controller) need bringup to be running the '
                    'current-mode description.'
                ),
            ),
            DeclareLaunchArgument(
                'controller_manager',
                default_value='/controller_manager',
                description='Controller manager to load into.',
            ),
            DeclareLaunchArgument(
                'param_file',
                default_value=default_param_file,
                description='Parameters for the controllers being loaded.',
            ),
            DeclareLaunchArgument(
                'activate',
                default_value='true',
                description='Activate on load, or leave configured but inactive.',
            ),
            OpaqueFunction(function=_spawners),
        ]
    )
