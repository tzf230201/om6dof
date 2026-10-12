"""Focus RViz on the live TCP Jacobian and manipulability ellipsoid."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('om6dof_description')
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config', default_value=PathJoinSubstitution([share, 'rviz', 'jacobian.rviz']),
            description='RViz preset focused on the live TCP Jacobian.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                share, 'launch', 'view_dynamic.launch.py']))),
    ])
