"""Launch the vision-only RGB-D centre-pixel/world-coordinate viewer."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("camera_source", default_value="realsense"),
        DeclareLaunchArgument("camera_serial", default_value="427622271962"),
        DeclareLaunchArgument("camera_width", default_value="640"),
        DeclareLaunchArgument("camera_height", default_value="480"),
        DeclareLaunchArgument("camera_fps", default_value="15"),
        DeclareLaunchArgument("camera_timeout_ms", default_value="3000"),
        DeclareLaunchArgument("base_frame", default_value="world"),
        DeclareLaunchArgument("camera_optical_frame",
                              default_value="d405_depth_optical_frame"),
        DeclareLaunchArgument("max_display_depth_m", default_value="1.0"),
    ]
    viewer = Node(
        package="om6dof_pick_and_place_gemini",
        executable="rgbd_viewer",
        name="rgbd_viewer",
        output="screen",
        parameters=[{
            "camera_source": ParameterValue(
                LaunchConfiguration("camera_source"), value_type=str),
            # A numeric-only RealSense serial is otherwise parsed as an
            # integer by the generated YAML parameter file.
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
            "base_frame": ParameterValue(
                LaunchConfiguration("base_frame"), value_type=str),
            "camera_optical_frame": ParameterValue(
                LaunchConfiguration("camera_optical_frame"), value_type=str),
            "max_display_depth_m": ParameterValue(
                LaunchConfiguration("max_display_depth_m"), value_type=float),
        }],
    )
    return LaunchDescription(arguments + [viewer])
