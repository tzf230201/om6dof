"""Read-only V2 TF plus the sole RealSense owner for hand-eye capture.

This launch never starts ros2_control, MoveIt, or a motion node.  Stop the
DD-GNG camera launch first, then move the arm only with the separately
commissioned operator controls.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial = LaunchConfiguration("camera_serial")
    description = Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([
            FindPackageShare("om6dof_description"), "urdf", "om6dof_v2.urdf.xacro"]),
    ])
    isolated_tf = [
        ("/tf", "/om6dof_dd_gng/v2/tf"),
        ("/tf_static", "/om6dof_dd_gng/v2/tf_static"),
        ("/robot_description", "/om6dof_dd_gng/v2/robot_description"),
    ]
    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_serial", default_value="",
            description="Required D435i serial; use `rs-enumerate-devices -s`"),
        DeclareLaunchArgument("tag_id", default_value="1"),
        DeclareLaunchArgument("tag_size", default_value="0.04"),
        DeclareLaunchArgument("show_window", default_value="true"),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="hand_eye_v2_state_publisher", output="screen",
            parameters=[{"robot_description": ParameterValue(description, value_type=str)}],
            remappings=isolated_tf,
        ),
        Node(
            package="om6dof_pick_and_place", executable="apriltag_detector",
            name="hand_eye_apriltag_detector", output="screen", emulate_tty=True,
            parameters=[{
                "serial": ParameterValue(serial, value_type=str),
                "camera_frame": "d435_color_optical_frame",
                "tag_id": ParameterValue(LaunchConfiguration("tag_id"), value_type=int),
                "tag_size": ParameterValue(LaunchConfiguration("tag_size"), value_type=float),
                "use_depth": False,
                "show_window": ParameterValue(
                    LaunchConfiguration("show_window"), value_type=bool),
                "publish_debug": True,
            }],
        ),
    ])
