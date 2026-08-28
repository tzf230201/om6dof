"""Visualise the friction-identification multisine in RViz without hardware."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([
                FindPackageShare('om6dof_description'), 'urdf', 'om6dof.urdf.xacro',
            ]),
        ]),
        value_type=str,
    )
    rviz_config = PathJoinSubstitution([
        FindPackageShare('om6dof_description'), 'rviz', 'com.rviz',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='conservative'),
        DeclareLaunchArgument('amplitude_scale', default_value='1.0'),
        DeclareLaunchArgument('frequency_scale', default_value='1.0'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': description}],
            output='screen',
        ),
        Node(
            package='om6dof_controllers',
            executable='multisine_identification.py',
            arguments=[
                '--simulate',
                '--profile', LaunchConfiguration('profile'),
                '--amplitude-scale', LaunchConfiguration('amplitude_scale'),
                '--frequency-scale', LaunchConfiguration('frequency_scale'),
            ],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
