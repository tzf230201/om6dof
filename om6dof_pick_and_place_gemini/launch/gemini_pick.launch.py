"""Classification run: pick the best object, ask Gemini what it is, sort it.

    ros2 launch om6dof_pick_and_place_gemini gemini_pick.launch.py
    ros2 service call /gemini_pick/run std_srvs/srv/Trigger

Bring up the OM6DOF hardware stack first and make sure nothing else holds the
wrist RealSense. This launch starts MoveGroup itself unless start_moveit=false.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (EnvironmentVariable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PACKAGE = "om6dof_pick_and_place_gemini"


def generate_launch_description():
    fastdds = SetEnvironmentVariable(
        name="RMW_IMPLEMENTATION",
        value="rmw_fastrtps_cpp",
    )
    default_params = os.path.join(get_package_share_directory(PACKAGE),
                                  "config", "gemini_pick.yaml")
    args = [
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="parameter file for gemini_pick"),
        DeclareLaunchArgument("backend", default_value="graspnet",
                              description=("grasp backend: analytic | graspnet "
                                           "| anygrasp")),
        DeclareLaunchArgument(
            "graspnet_repo_path",
            default_value=EnvironmentVariable(
                "GRASPNET_REPO_PATH",
                default_value=("/mnt/agx_nvme/om6dof-graspnet-jp622/src/"
                               "graspnet-baseline"))),
        DeclareLaunchArgument(
            "graspnet_checkpoint",
            default_value=EnvironmentVariable(
                "GRASPNET_CHECKPOINT",
                default_value=("/mnt/agx_nvme/om6dof-graspnet-jp622/"
                               "checkpoint-rs.tar"))),
        DeclareLaunchArgument(
            "anygrasp_runtime_dir",
            default_value=("/home/kublab/ros2_ws/src/anygrasp_sdk/"
                           "grasp_detection")),
        DeclareLaunchArgument(
            "anygrasp_checkpoint",
            default_value=PathJoinSubstitution([
                LaunchConfiguration("anygrasp_runtime_dir"),
                "checkpoint_detection.tar",
            ])),
        DeclareLaunchArgument(
            "anygrasp_license_dir",
            default_value=PathJoinSubstitution([
                LaunchConfiguration("anygrasp_runtime_dir"), "license",
            ])),
        DeclareLaunchArgument("anygrasp_max_width", default_value="0.065"),
        DeclareLaunchArgument("anygrasp_gripper_height", default_value="0.058"),
        DeclareLaunchArgument("anygrasp_dense_grasp", default_value="false"),
        DeclareLaunchArgument(
            "anygrasp_collision_detection", default_value="true"),
        DeclareLaunchArgument("execute_motion", default_value="false",
                              description="false = real MoveIt plan-only; no motion"),
        DeclareLaunchArgument("auto", default_value="false",
                              description="start the sequence after launch"),
        DeclareLaunchArgument("place_enabled", default_value="false"),
        DeclareLaunchArgument("calibration_validated", default_value="false",
                              description="explicit physical camera/TCP/table sign-off"),
        DeclareLaunchArgument("gripper_width_at_open_pos", default_value="-1.0"),
        DeclareLaunchArgument("gripper_width_at_close_pos", default_value="-1.0"),
        DeclareLaunchArgument(
            "gripper_calibration_validated", default_value="false"),
        DeclareLaunchArgument("place_poses_validated", default_value="false"),
        DeclareLaunchArgument(
            "dynamixel_health_topic",
            default_value="/dynamixel_hardware_interface/health"),
        DeclareLaunchArgument(
            "dynamixel_health_timeout_s", default_value="0.30"),
        DeclareLaunchArgument(
            "dynamixel_health_clean_window_s", default_value="60.0"),
        DeclareLaunchArgument("start_moveit", default_value="true",
                              description="start MoveGroup with OMPL + Pilz LIN"),
        DeclareLaunchArgument("start_rviz", default_value="false"),
        DeclareLaunchArgument("top_k", default_value="50",
                              description=("learned-backend candidates kept "
                                           "before filtering")),
        DeclareLaunchArgument(
            "max_prevalidation_candidates", default_value="5",
            description="ranked grasps fully plan-checked before one physical attempt"),
    ]
    node = Node(
        package=PACKAGE,
        executable="gemini_pick_node",
        name="gemini_pick",
        output="screen",
        emulate_tty=True,
        # Give the node time to keep retrying physical controller cancellation
        # instead of launch escalating to SIGKILL after the default few seconds.
        sigterm_timeout="120.0",
        sigkill_timeout="120.0",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "mode": "classify",
                "grasp_backend": LaunchConfiguration("backend"),
                "graspnet_repo_path": LaunchConfiguration(
                    "graspnet_repo_path"),
                "graspnet_checkpoint": LaunchConfiguration(
                    "graspnet_checkpoint"),
                "anygrasp_runtime_dir": LaunchConfiguration(
                    "anygrasp_runtime_dir"),
                "anygrasp_checkpoint": LaunchConfiguration(
                    "anygrasp_checkpoint"),
                "anygrasp_license_dir": LaunchConfiguration(
                    "anygrasp_license_dir"),
                "anygrasp_max_width": ParameterValue(
                    LaunchConfiguration("anygrasp_max_width"),
                    value_type=float),
                "anygrasp_gripper_height": ParameterValue(
                    LaunchConfiguration("anygrasp_gripper_height"),
                    value_type=float),
                "anygrasp_dense_grasp": ParameterValue(
                    LaunchConfiguration("anygrasp_dense_grasp"),
                    value_type=bool),
                "anygrasp_collision_detection": ParameterValue(
                    LaunchConfiguration("anygrasp_collision_detection"),
                    value_type=bool),
                "execute_motion": ParameterValue(
                    LaunchConfiguration("execute_motion"), value_type=bool),
                "auto_run": ParameterValue(
                    LaunchConfiguration("auto"), value_type=bool),
                "place_enabled": ParameterValue(
                    LaunchConfiguration("place_enabled"), value_type=bool),
                "calibration_validated": ParameterValue(
                    LaunchConfiguration("calibration_validated"), value_type=bool),
                "gripper_width_at_open_pos": ParameterValue(
                    LaunchConfiguration("gripper_width_at_open_pos"),
                    value_type=float),
                "gripper_width_at_close_pos": ParameterValue(
                    LaunchConfiguration("gripper_width_at_close_pos"),
                    value_type=float),
                "gripper_calibration_validated": ParameterValue(
                    LaunchConfiguration("gripper_calibration_validated"),
                    value_type=bool),
                "place_poses_validated": ParameterValue(
                    LaunchConfiguration("place_poses_validated"), value_type=bool),
                "dynamixel_health_topic": LaunchConfiguration(
                    "dynamixel_health_topic"),
                "dynamixel_health_timeout_s": ParameterValue(
                    LaunchConfiguration("dynamixel_health_timeout_s"),
                    value_type=float),
                "dynamixel_health_clean_window_s": ParameterValue(
                    LaunchConfiguration("dynamixel_health_clean_window_s"),
                    value_type=float),
                "top_k": ParameterValue(
                    LaunchConfiguration("top_k"), value_type=int),
                "max_prevalidation_candidates": ParameterValue(
                    LaunchConfiguration("max_prevalidation_candidates"),
                    value_type=int),
            },
        ],
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("om6dof_moveit_config"),
            "launch", "om6dof_moveit.launch.py",
        ])),
        launch_arguments={"start_rviz": LaunchConfiguration("start_rviz")}.items(),
        condition=IfCondition(LaunchConfiguration("start_moveit")),
    )
    return LaunchDescription(args + [fastdds, moveit, node])
