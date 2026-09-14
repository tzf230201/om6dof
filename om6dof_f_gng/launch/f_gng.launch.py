"""RealSense and world-frame F-GNG in RViz; uses the robot's existing TF tree."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("om6dof_f_gng"))
    params_file = LaunchConfiguration("params_file")
    world_frame = LaunchConfiguration("world_frame")
    camera_frame = LaunchConfiguration("camera_frame")
    inputs = [
        ("depth/image_raw", LaunchConfiguration("depth_topic")),
        ("depth/camera_info", LaunchConfiguration("camera_info_topic")),
    ]

    camera = Node(
        package="om6dof_f_gng",
        executable="realsense_depth_node",
        namespace="om6dof_f_gng",
        name="realsense_depth",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_camera")),
        parameters=[params_file, {
            "camera_frame": ParameterValue(camera_frame, value_type=str),
            "serial": ParameterValue(LaunchConfiguration("camera_serial"), value_type=str),
        }],
        remappings=inputs,
    )
    mapper = Node(
        package="om6dof_f_gng",
        executable="f_gng_node",
        namespace="om6dof_f_gng",
        name="f_gng",
        output="screen",
        emulate_tty=True,
        parameters=[params_file, {
            "world_frame": ParameterValue(world_frame, value_type=str),
            "camera_frame": ParameterValue(camera_frame, value_type=str),
        }],
        remappings=inputs,
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="f_gng_rviz",
        output="screen",
        arguments=["-d", str(share / "rviz" / "f_gng.rviz"), "-f", world_frame],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    declarations = [
        DeclareLaunchArgument("use_rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument(
            "start_camera", default_value="true", choices=["true", "false"],
            description="Open RealSense directly; false subscribes to existing ROS depth topics."),
        DeclareLaunchArgument(
            "depth_topic", default_value="/om6dof_f_gng/depth/image_raw",
            description="Rectified depth image topic (32FC1 metres or 16UC1 millimetres)."),
        DeclareLaunchArgument(
            "camera_info_topic", default_value="/om6dof_f_gng/depth/camera_info",
            description="CameraInfo matching the depth image resolution and optical frame."),
        DeclareLaunchArgument("world_frame", default_value="world"),
        DeclareLaunchArgument("camera_frame", default_value="d435_depth_optical_frame"),
        DeclareLaunchArgument("camera_serial", default_value=""),
        DeclareLaunchArgument(
            "params_file", default_value=str(share / "config" / "f_gng.yaml"),
            description="ROS parameter file for the camera and mapper nodes."),
    ]
    # Closing RViz also releases the SDK camera. A failed component terminates
    # the session, instead of leaving an apparently live but frozen display.
    handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason=reason))],
        ))
        for process, reason in [
            (camera, "F-GNG camera process exited"),
            (mapper, "F-GNG mapper process exited"),
            (rviz, "F-GNG RViz window closed"),
        ]
    ]
    return LaunchDescription(declarations + handlers + [camera, mapper, rviz])
