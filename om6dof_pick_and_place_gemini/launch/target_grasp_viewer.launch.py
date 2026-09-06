"""Launch the separate vision-only Gemini target + learned-grasp viewer."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("target", default_value="the object on the table"),
        DeclareLaunchArgument(
            "backend", default_value="graspnet",
            description="grasp backend: graspnet | anygrasp"),
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
        DeclareLaunchArgument("camera_serial", default_value="427622271962"),
        DeclareLaunchArgument("camera_width", default_value="640"),
        DeclareLaunchArgument("camera_height", default_value="480"),
        DeclareLaunchArgument("camera_fps", default_value="15"),
        DeclareLaunchArgument("camera_timeout_ms", default_value="3000"),
        DeclareLaunchArgument(
            "top_k", default_value="50",
            description="learned-backend candidates kept before filtering"),
        DeclareLaunchArgument("graspnet_sampling_seed", default_value="0"),
        DeclareLaunchArgument("table_z", default_value="0.0"),
        DeclareLaunchArgument("gripper_max_width_m", default_value="0.065"),
        DeclareLaunchArgument(
            "gripper_width_at_open_pos", default_value="-1.0",
            description=("measured clear open aperture in metres; negative "
                         "uses max width for non-executable preview only")),
        DeclareLaunchArgument("grasp_max_tilt_rad", default_value="1.50"),
        DeclareLaunchArgument("target_crop_pad_px", default_value="4.0"),
        DeclareLaunchArgument(
            "target_depth_tolerance_m", default_value="0.05"),
        DeclareLaunchArgument(
            "target_component_voxel_m", default_value="0.008"),
        DeclareLaunchArgument(
            "target_component_min_points", default_value="30"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
    ]
    viewer = Node(
        package="om6dof_pick_and_place_gemini",
        executable="target_grasp_viewer",
        name="target_grasp_viewer",
        output="screen",
        parameters=[{
            "target": ParameterValue(LaunchConfiguration("target"),
                                     value_type=str),
            "grasp_backend": LaunchConfiguration("backend"),
            "anygrasp_runtime_dir": LaunchConfiguration(
                "anygrasp_runtime_dir"),
            "anygrasp_checkpoint": LaunchConfiguration(
                "anygrasp_checkpoint"),
            "anygrasp_license_dir": LaunchConfiguration(
                "anygrasp_license_dir"),
            "anygrasp_max_width": ParameterValue(
                LaunchConfiguration("anygrasp_max_width"), value_type=float),
            "anygrasp_gripper_height": ParameterValue(
                LaunchConfiguration("anygrasp_gripper_height"),
                value_type=float),
            "anygrasp_dense_grasp": ParameterValue(
                LaunchConfiguration("anygrasp_dense_grasp"), value_type=bool),
            "anygrasp_collision_detection": ParameterValue(
                LaunchConfiguration("anygrasp_collision_detection"),
                value_type=bool),
            # Numeric-only D405 serials must never be converted to YAML ints.
            "camera_serial": ParameterValue(
                LaunchConfiguration("camera_serial"), value_type=str),
            "camera_width": ParameterValue(
                LaunchConfiguration("camera_width"), value_type=int),
            "camera_height": ParameterValue(
                LaunchConfiguration("camera_height"), value_type=int),
            "camera_fps": ParameterValue(
                LaunchConfiguration("camera_fps"), value_type=int),
            "camera_timeout_ms": ParameterValue(
                LaunchConfiguration("camera_timeout_ms"), value_type=int),
            "top_k": ParameterValue(LaunchConfiguration("top_k"),
                                    value_type=int),
            "graspnet_sampling_seed": ParameterValue(
                LaunchConfiguration("graspnet_sampling_seed"),
                value_type=int),
            "table_z": ParameterValue(LaunchConfiguration("table_z"),
                                      value_type=float),
            "gripper_max_width_m": ParameterValue(
                LaunchConfiguration("gripper_max_width_m"),
                value_type=float),
            "gripper_width_at_open_pos": ParameterValue(
                LaunchConfiguration("gripper_width_at_open_pos"),
                value_type=float),
            "grasp_max_tilt_rad": ParameterValue(
                LaunchConfiguration("grasp_max_tilt_rad"),
                value_type=float),
            "target_crop_pad_px": ParameterValue(
                LaunchConfiguration("target_crop_pad_px"), value_type=float),
            "target_depth_tolerance_m": ParameterValue(
                LaunchConfiguration("target_depth_tolerance_m"),
                value_type=float),
            "target_component_voxel_m": ParameterValue(
                LaunchConfiguration("target_component_voxel_m"),
                value_type=float),
            "target_component_min_points": ParameterValue(
                LaunchConfiguration("target_component_min_points"),
                value_type=int),
        }],
    )
    rviz = Node(
        package="rviz2", executable="rviz2", name="target_grasp_rviz",
        condition=IfCondition(LaunchConfiguration("start_rviz")),
        output="screen",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("om6dof_pick_and_place_gemini"), "config",
            "target_grasp_viewer.rviz",
        ])],
    )
    return LaunchDescription(arguments + [viewer, rviz])
