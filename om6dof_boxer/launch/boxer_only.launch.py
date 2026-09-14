"""Manual 2D prompts -> Meta BOXER 3D; no robot, DD-GNG, YOLO, or controller."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    runtime = Path.home() / 'ros2_ws/src/.boxer_runtime'
    share = Path(get_package_share_directory('om6dof_boxer'))
    script = share.parents[1] / 'lib/om6dof_boxer/boxer_preview.py'
    preview = ExecuteProcess(cmd=[LaunchConfiguration('boxer_python'), '-u', str(script),
                            '--repo', LaunchConfiguration('boxer_repo'),
                            '--serial', LaunchConfiguration('camera_serial'),
                            '--labels', LaunchConfiguration('labels'),
                            '--device', LaunchConfiguration('device'),
                            '--snapshot', LaunchConfiguration('snapshot'),
                            '--gravity-source', LaunchConfiguration('gravity_source'),
                            '--output-dir', LaunchConfiguration('output_dir')], output='both')
    description = Path(get_package_share_directory('om6dof_description')) / 'urdf/om6dof_v2.urdf.xacro'
    return LaunchDescription([
        DeclareLaunchArgument('boxer_python', default_value=str(runtime / 'venv/bin/python')),
        DeclareLaunchArgument('boxer_repo', default_value=str(runtime / 'boxer')),
        DeclareLaunchArgument('camera_serial', default_value='243222076197'),
        DeclareLaunchArgument('labels', default_value='bottle,keyboard'),
        DeclareLaunchArgument('device', default_value='cpu', choices=['cpu', 'cuda']),
        DeclareLaunchArgument('snapshot', default_value=''),
        DeclareLaunchArgument('output_dir', default_value=str(runtime / 'outputs')),
        DeclareLaunchArgument('launch_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('gravity_source', default_value='auto', choices=['auto', 'imu', 'tf']),
        DeclareLaunchArgument('publish_robot_state', default_value='true', choices=['true', 'false']),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='boxer_orientation_state_publisher', output='screen',
             parameters=[{'robot_description': ParameterValue(Command([FindExecutable(name='xacro'), ' ', str(description)]), value_type=str)}],
             remappings=[('/tf', '/boxer/tf'), ('/tf_static', '/boxer/tf_static'),
                         ('/robot_description', '/boxer/robot_description')],
             condition=IfCondition(LaunchConfiguration('publish_robot_state'))),
        RegisterEventHandler(OnProcessExit(target_action=preview, on_exit=[
            EmitEvent(event=Shutdown(reason='BOXER preview exited; closing its RViz'))])),
        preview,
        Node(package='rviz2', executable='rviz2', name='boxer_rviz',
             arguments=['-d', str(share / 'rviz/boxer.rviz')],
             condition=IfCondition(LaunchConfiguration('launch_rviz'))),
    ])
