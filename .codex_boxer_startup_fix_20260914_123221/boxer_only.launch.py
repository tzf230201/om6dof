"""Manual 2D prompts -> Meta BOXER 3D; no robot, DD-GNG, YOLO, or controller."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    runtime = Path.home() / 'ros2_ws/src/.boxer_runtime'
    share = Path(get_package_share_directory('om6dof_boxer'))
    script = share.parents[1] / 'lib/om6dof_boxer/boxer_preview.py'
    return LaunchDescription([
        DeclareLaunchArgument('boxer_python', default_value=str(runtime / 'venv/bin/python')),
        DeclareLaunchArgument('boxer_repo', default_value=str(runtime / 'boxer')),
        DeclareLaunchArgument('camera_serial', default_value='243222076197'),
        DeclareLaunchArgument('labels', default_value='bottle,keyboard'),
        DeclareLaunchArgument('device', default_value='cpu', choices=['cpu', 'cuda']),
        DeclareLaunchArgument('snapshot', default_value=''),
        DeclareLaunchArgument('output_dir', default_value=str(runtime / 'outputs')),
        DeclareLaunchArgument('launch_rviz', default_value='true', choices=['true', 'false']),
        ExecuteProcess(cmd=[LaunchConfiguration('boxer_python'), str(script),
                            '--repo', LaunchConfiguration('boxer_repo'),
                            '--serial', LaunchConfiguration('camera_serial'),
                            '--labels', LaunchConfiguration('labels'),
                            '--device', LaunchConfiguration('device'),
                            '--snapshot', LaunchConfiguration('snapshot'),
                            '--output-dir', LaunchConfiguration('output_dir')], output='screen'),
        Node(package='rviz2', executable='rviz2', name='boxer_rviz',
             arguments=['-d', str(share / 'rviz/boxer.rviz')],
             condition=IfCondition(LaunchConfiguration('launch_rviz'))),
    ])
