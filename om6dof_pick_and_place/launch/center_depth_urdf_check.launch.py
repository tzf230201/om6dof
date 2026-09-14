"""Camera-centre world-coordinate check using read-only V2 URDF TF."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description = Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([
            FindPackageShare("om6dof_description"), "urdf",
            "om6dof_v2.urdf.xacro"]),
    ])
    isolated_tf = [
        ("/tf", "/om6dof_dd_gng/v2/tf"),
        ("/tf_static", "/om6dof_dd_gng/v2/tf_static"),
        ("/robot_description", "/om6dof_dd_gng/v2/robot_description"),
    ]
    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_serial", default_value="243222076197",
            description="D435i serial; kept as a string"),
        DeclareLaunchArgument("show_window", default_value="true"),
        DeclareLaunchArgument("width", default_value="640"),
        DeclareLaunchArgument("height", default_value="480"),
        DeclareLaunchArgument("fps", default_value="30"),
        DeclareLaunchArgument("patch_radius", default_value="4"),
        DeclareLaunchArgument(
            "world_z_offset", default_value="-0.0185",
            description="Explicit diagnostic correction added to URDF world Z"),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="center_depth_v2_state_publisher", output="screen",
            parameters=[{
                "robot_description": ParameterValue(description, value_type=str),
            }],
            remappings=isolated_tf,
        ),
        Node(
            package="om6dof_pick_and_place",
            executable="center_depth_urdf_check",
            name="center_depth_urdf_check",
            output="screen", emulate_tty=True,
            parameters=[{
                "camera_serial": ParameterValue(
                    LaunchConfiguration("camera_serial"), value_type=str),
                "width": ParameterValue(LaunchConfiguration("width"), value_type=int),
                "height": ParameterValue(LaunchConfiguration("height"), value_type=int),
                "fps": ParameterValue(LaunchConfiguration("fps"), value_type=int),
                "patch_radius": ParameterValue(
                    LaunchConfiguration("patch_radius"), value_type=int),
                "world_z_offset": ParameterValue(
                    LaunchConfiguration("world_z_offset"), value_type=float),
                "camera_frame": "d435_color_optical_frame",
                "world_frame": "world",
                "show_window": ParameterValue(
                    LaunchConfiguration("show_window"), value_type=bool),
            }],
            remappings=isolated_tf,
        ),
    ])
