"""Fast YOLO+depth centre pickup: yolo_3d_box_node -> bridge -> boxer_pick_node.

Same MoveIt/boxer_pick_node/GUI/RViz plumbing as yolox_boxer3d_pickup.launch.py
(that file is untouched and still available as a fallback), but the upstream
3D-box source is om6dof_dd_gng's yolo_3d_box_node -- TensorRT YOLOX + direct
depth segmentation, ~20 ms/frame -- instead of a separate YOLOX viewer process
feeding a ~1 s/cycle ONNX Boxer3D orientation network. PickupPlanner only ever
needs a target CENTRE point (it searches its own yaw/roll grasp candidates via
IK; see boxer_pick_planning.py's plan()), so the missing per-object yaw
estimate costs nothing here -- only a slightly looser axis-aligned collision
box, not the ability to pick a centre point.

yolo3d_pickup_bridge.py (om6dof_perception) does the one thing that differs from
the ONNX path: transform yolo_3d_box_node's camera-frame detections into the
world frame and republish them in the same
om6dof.boxer3d_detections.v1 schema boxer_pick_node.py already validates.
boxer_pick_node.py itself is completely unmodified -- it does not know or
care which detector produced its input.
"""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    share = Path(get_package_share_directory('om6dof_pick_and_place'))
    boxer = Path(get_package_share_directory('om6dof_perception'))
    ddgng = Path(get_package_share_directory('om6dof_dd_gng'))
    helper = importlib.util.spec_from_file_location(
        'boxer_pick_model_inputs', ddgng / 'launch/_model_inputs.py')
    module = importlib.util.module_from_spec(helper)
    helper.loader.exec_module(module)
    calibration = module.load_hand_eye_calibration(arg('camera_calibration_file'))
    if not calibration:
        raise RuntimeError('Pickup memerlukan camera_calibration_file yang sudah diverifikasi')
    serial = arg('camera_serial') or calibration['camera_calibration_serial']
    if serial != calibration['camera_calibration_serial']:
        raise RuntimeError('camera_serial tidak cocok dengan file kalibrasi pickup')
    description_share = Path(get_package_share_directory('om6dof_description'))
    description = description_share / 'urdf/om6dof_v2.urdf.xacro'
    config = (MoveItConfigsBuilder('om6dof_v2', package_name='om6dof_moveit')
              .robot_description(file_path=str(description))
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
        # Own v2-consistent forward kinematics on a private /boxer/tf, exactly
        # like yolox_boxer3d_onnx.launch.py's boxer3d_orientation_state_publisher:
        # the live stack's real /tf carries the v1 URDF, whose joint3->link4
        # datum sits 4 mm / 90 deg from v2's, so anything planning against v2
        # geometry (this whole pickup pipeline) must compute its own v2 TF
        # rather than trust the real /tf.
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='yolo3d_pickup_orientation_state_publisher', output='screen',
             parameters=[{'robot_description': ParameterValue(
                 Command([FindExecutable(name='xacro'), ' ', str(description)]), value_type=str)}],
             remappings=[('/tf', '/boxer/tf'), ('/tf_static', '/boxer/tf_static'),
                         ('/robot_description', '/boxer3d/robot_description')]),
        # Same private v2 TF the bridge and the planner read, so the world
        # coordinates drawn on the preview window are the very numbers the arm
        # is commanded against -- reading them off a different tree would make
        # the display quietly disagree with where the gripper actually goes.
        Node(package='om6dof_dd_gng', executable='yolo_3d_box_node', name='yolo3d_pickup_perception',
             output='screen',
             parameters=[{'camera_serial': serial, 'enable_dd_gng': False,
                          'display_window': arg('display_window') == 'true',
                          'confidence': float(arg('confidence')),
                          'nms_threshold': float(arg('nms_threshold'))}],
             remappings=[('/tf', '/boxer/tf'), ('/tf_static', '/boxer/tf_static')]),
        Node(package='om6dof_perception', executable='yolo3d_pickup_bridge.py', name='yolo3d_pickup_bridge',
             output='screen',
             parameters=[{'camera_serial': serial, 'camera_calibration_file': arg('camera_calibration_file')}],
             remappings=[('/tf', '/boxer/tf'), ('/tf_static', '/boxer/tf_static')]),
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
        DeclareLaunchArgument('confidence', default_value='0.35'),
        DeclareLaunchArgument('nms_threshold', default_value='0.5'),
        DeclareLaunchArgument('rmw_implementation', default_value='rmw_fastrtps_cpp'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', LaunchConfiguration('rmw_implementation')),
        OpaqueFunction(function=launch_setup),
    ])
