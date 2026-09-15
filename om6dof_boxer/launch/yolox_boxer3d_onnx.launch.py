"""Automatic TensorRT YOLOX prompts -> Barath19 Boxer3D ONNX, standalone."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    runtime=Path.home()/'ros2_ws/src/.boxer3d_runtime'
    share=Path(get_package_share_directory('om6dof_boxer'))
    script=share.parents[1]/'lib/om6dof_boxer/barath_boxer3d_preview.py'
    rgb_topic='/boxer3d/rgb/image/compressed';detections_topic='/boxer3d/yolox/detections'
    preview=ExecuteProcess(cmd=[LaunchConfiguration('boxer_python'),'-u',str(script),
        '--model',LaunchConfiguration('boxer_model'),'--serial',LaunchConfiguration('camera_serial'),
        '--target-classes',LaunchConfiguration('target_classes'),'--yolo-min-score',LaunchConfiguration('yolo_min_score'),
        '--boxer-min-score',LaunchConfiguration('boxer_min_score'),'--max-boxes',LaunchConfiguration('max_boxes'),
        '--threads',LaunchConfiguration('onnx_threads'),'--gravity-source',LaunchConfiguration('gravity_source'),
        '--rgb-topic',rgb_topic,'--detections-topic',detections_topic,
        '--publish-pick-detections',LaunchConfiguration('publish_pick_detections'),
        '--camera-calibration-file',LaunchConfiguration('camera_calibration_file'),
        '--display-window',LaunchConfiguration('display_window'),
        '--output-dir',LaunchConfiguration('output_dir')],output='both')
    description=Path(get_package_share_directory('om6dof_description'))/'urdf/om6dof_v2.urdf.xacro'
    return LaunchDescription([
        DeclareLaunchArgument('boxer_python',default_value=str(runtime/'venv/bin/python')),
        DeclareLaunchArgument('boxer_model',default_value=str(runtime/'models/BoxerNet.onnx')),
        DeclareLaunchArgument('camera_serial',default_value='243222076197'),
        DeclareLaunchArgument('target_classes',default_value=''),
        DeclareLaunchArgument('yolo_min_score',default_value='0.25'),
        DeclareLaunchArgument('boxer_min_score',default_value='0.30'),
        DeclareLaunchArgument('max_boxes',default_value='3'),
        DeclareLaunchArgument('onnx_threads',default_value='8'),
        DeclareLaunchArgument('gravity_source',default_value='auto',choices=['auto','imu','tf']),
        DeclareLaunchArgument('publish_pick_detections',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('camera_calibration_file',default_value=''),
        DeclareLaunchArgument('display_window',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('output_dir',default_value=str(runtime/'outputs')),
        DeclareLaunchArgument('launch_rviz',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('publish_robot_state',default_value='true',choices=['true','false']),
        Node(package='robot_state_publisher',executable='robot_state_publisher',
             name='boxer3d_orientation_state_publisher',output='screen',
             parameters=[{'robot_description':ParameterValue(
                 Command([FindExecutable(name='xacro'),' ',str(description)]),value_type=str)}],
             remappings=[('/tf','/boxer/tf'),('/tf_static','/boxer/tf_static'),
                         ('/robot_description','/boxer3d/robot_description')],
             condition=IfCondition(LaunchConfiguration('publish_robot_state'))),
        Node(package='om6dof_dd_gng',executable='yolox_viewer_node',name='boxer3d_yolox',output='screen',
             parameters=[{'input_topic':rgb_topic,
                          'output_topic':'/boxer3d/yolox/debug_image/compressed',
                          'detections_topic':detections_topic,'display_window':False,
                          'confidence':LaunchConfiguration('yolo_min_score')}]),
        RegisterEventHandler(OnProcessExit(target_action=preview,on_exit=[
            EmitEvent(event=Shutdown(reason='Boxer3D preview exited'))])),
        preview,
        Node(package='rviz2',executable='rviz2',name='boxer3d_rviz',
             arguments=['-d',str(share/'rviz/boxer.rviz')],
             condition=IfCondition(LaunchConfiguration('launch_rviz'))),
    ])
