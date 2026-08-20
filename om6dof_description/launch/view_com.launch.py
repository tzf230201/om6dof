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
View the arm with each link's centre of mass drawn on it.

Same picture as view_robot.launch.py, plus the mass distribution the gravity
model actually integrates. Drag the joint sliders and watch the levers swing:
that is what g(q) is computing.
"""

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _joint_source(context, *args, **kwargs):
    """
    Pick something to put joint positions on /joint_states.

    Without one, robot_state_publisher emits no TF for the movable joints and
    RViz has nowhere to put the links. joint_state_publisher_gui is the nice
    answer but it is not installed everywhere, and failing the whole launch over
    a viewing convenience is not a good trade.
    """
    choice = LaunchConfiguration('joints').perform(context)

    if choice == 'auto':
        try:
            get_package_share_directory('joint_state_publisher_gui')
            choice = 'gui'
        except PackageNotFoundError:
            choice = 'static'

    if choice == 'external':
        return []

    if choice == 'gui':
        return [Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        )]

    if choice != 'static':
        raise RuntimeError(
            "launch argument 'joints' must be auto, gui, static or external, not %r" % choice)

    parameters = {'robot_description': _description()}
    pose = LaunchConfiguration('pose').perform(context).split()
    if pose:
        parameters['pose'] = [float(v) for v in pose]

    return [Node(
        package='om6dof_description',
        executable='static_joint_states.py',
        name='static_joint_states',
        parameters=[parameters],
        output='screen',
    )]


def _description():
    return ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([
                FindPackageShare('om6dof_description'), 'urdf', 'om6dof.urdf.xacro'
            ]),
        ]),
        value_type=str,
    )


def generate_launch_description():
    description = _description()
    rviz_config = PathJoinSubstitution([
        FindPackageShare('om6dof_description'), 'rviz', 'com.rviz'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'joints', default_value='auto',
            description=(
                'Where joint positions come from. "gui" uses '
                'joint_state_publisher_gui, "static" holds one pose with no extra '
                'packages, "external" assumes the live robot is publishing, and '
                '"auto" picks the gui when it is installed and static otherwise.'
            )),
        DeclareLaunchArgument(
            'pose', default_value='',
            description='Space-separated joint positions for the "static" source.'),
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Start RViz.'),
        DeclareLaunchArgument(
            'labels', default_value='true',
            description=(
                'Write each link name, mass and lever arm next to its centre of '
                'mass. The masses themselves are drawn by RViz, under the '
                "RobotModel display's Mass Properties. Text markers are the "
                'expensive thing to render here, so labels:=false if the view '
                'crawls.'
            )),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': description}],
            output='screen',
        ),
        OpaqueFunction(function=_joint_source),
        Node(
            package='om6dof_description',
            executable='com_markers.py',
            name='com_markers',
            parameters=[{'robot_description': description}],
            condition=IfCondition(LaunchConfiguration('labels')),
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
