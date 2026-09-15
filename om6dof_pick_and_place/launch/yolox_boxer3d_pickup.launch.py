"""Independent YOLOX/Boxer3D centre pickup; existing hardware controllers only."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    share = Path(get_package_share_directory('om6dof_pick_and_place'))
    boxer = Path(get_package_share_directory('om6dof_boxer'))
    ddgng = Path(get_package_share_directory('om6dof_dd_gng'))
    helper = importlib.util.spec_from_file_location('boxer_pick_model_inputs', ddgng / 'launch/_model_inputs.py')
    module = importlib.util.module_from_spec(helper)
    helper.loader.exec_module(module)
    calibration = module.load_hand_eye_calibration(arg('camera_calibration_file'))
    if not calibration:
        raise RuntimeError('Pickup Boxer3D memerlukan camera_calibration_file yang sudah diverifikasi')
    serial = arg('camera_serial') or calibration['camera_calibration_serial']
    if serial != calibration['camera_calibration_serial']:
        raise RuntimeError('camera_serial tidak cocok dengan file kalibrasi pickup')
    description = Path(get_package_share_directory('om6dof_description'))
    config = (MoveItConfigsBuilder('om6dof_v2', package_name='om6dof_moveit_config')
              .robot_description(file_path=str(description / 'urdf/om6dof_v2.urdf.xacro'))
              .robot_description_semantic(file_path='config/om6dof.srdf')
              .robot_description_kinematics(file_path='config/kinematics.yaml')
              .joint_limits(file_path='config/joint_limits.yaml')
              .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
              .to_moveit_configs())
    urdf = config.robot_description['robot_description']
    model = ET.fromstring(urdf)
    semantic = ET.fromstring(config.robot_description_semantic['robot_description_semantic'])
    semantic.set('name', model.attrib['name'])
    links = {item.attrib['name'] for item in model.findall('link')}
    for pair in list(semantic.findall('disable_collisions')):
        if pair.get('link1') not in links or pair.get('link2') not in links or pair.get('reason') == 'Never':
            semantic.remove(pair)
    # Existing measured mechanical touch allowance, shared with graph grasp.
    if not any({p.get('link1'), p.get('link2')} == {'link2', 'link6'}
               for p in semantic.findall('disable_collisions')):
        ET.SubElement(semantic, 'disable_collisions', link1='link2', link2='link6', reason='User')
    config.robot_description_semantic['robot_description_semantic'] = ET.tostring(semantic, encoding='unicode')
    parameters = config.to_dict()
    parameters.update(moveit_manage_controllers=False,
                      disable_capabilities='move_group/MoveGroupExecuteTrajectoryAction move_group/MoveGroupMoveAction')
    private_tf = [('/tf', '/boxer/tf'), ('/tf_static', '/boxer/tf_static'), ('joint_states', '/joint_states')]
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(boxer / 'launch/yolox_boxer3d_onnx.launch.py')),
            launch_arguments={'camera_serial': serial, 'camera_calibration_file': arg('camera_calibration_file'),
                              'publish_pick_detections': 'true', 'target_classes': '',
                              'max_boxes': arg('max_boxes'), 'onnx_threads': arg('onnx_threads'),
                              'launch_rviz': 'false', 'display_window': arg('display_window'),
                              'publish_robot_state': 'true'}.items()),
        Node(package='moveit_ros_move_group', executable='move_group', name='move_group', namespace='boxer_pick',
             output='screen', parameters=[parameters, {'allow_trajectory_execution': False,
                  'publish_robot_description_semantic': True}], remappings=private_tf),
        Node(package='om6dof_pick_and_place', executable='boxer_pick_node', name='boxer_center_pick', output='screen',
             parameters=[str(share / 'config/boxer_pick.yaml'), {'robot_description': urdf,
                 'target_class': arg('target_class'), 'execution_enabled': arg('execution_enabled') == 'true',
                 'camera_serial': serial, 'camera_calibration_sha256': calibration['camera_calibration_sha256']}]),
        Node(package='om6dof_pick_and_place', executable='boxer_pick_gui', output='screen',
             condition=IfCondition(LaunchConfiguration('launch_gui'))),
        Node(package='rviz2', executable='rviz2', name='boxer_pick_rviz', output='log',
             arguments=['-d', str(share / 'config/boxer_pick.rviz')],
             parameters=[config.robot_description, config.robot_description_semantic,
                         config.robot_description_kinematics], remappings=private_tf,
             condition=IfCondition(LaunchConfiguration('launch_rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_calibration_file', default_value=str(Path.home() / '.config/om6dof/d435_hand_eye.yaml')),
        DeclareLaunchArgument('camera_serial', default_value=''),
        DeclareLaunchArgument('target_class', default_value='bottle'),
        DeclareLaunchArgument('execution_enabled', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('launch_gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('launch_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('display_window', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('max_boxes', default_value='3'),
        DeclareLaunchArgument('onnx_threads', default_value='8'),
        DeclareLaunchArgument('rmw_implementation', default_value='rmw_fastrtps_cpp'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', LaunchConfiguration('rmw_implementation')),
        OpaqueFunction(function=launch_setup),
    ])
