"""DD-GNG on depth-derived 3D world points, using the existing robot TF tree."""

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
        name="dd_gng",
        output="screen",
        emulate_tty=True,
        parameters=[params_file, {
            "algorithm": "ddgng",
            "world_frame": ParameterValue(world_frame, value_type=str),
            "camera_frame": ParameterValue(camera_frame, value_type=str),
            "max_nodes": ParameterValue(LaunchConfiguration("max_nodes"), value_type=int),
            "memory_mode": ParameterValue(LaunchConfiguration("memory_mode"), value_type=str),
            "edge_support_radius": ParameterValue(
                LaunchConfiguration("edge_support_radius"), value_type=float),
            "dd_updates": ParameterValue(LaunchConfiguration("dd_updates"), value_type=int),
        }],
        remappings=inputs,
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="dd_gng_rviz",
        output="screen",
        arguments=["-d", str(share / "rviz" / "dd_gng.rviz"), "-f", world_frame],
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
        DeclareLaunchArgument("max_nodes", default_value="768",
                              description="Maximum number of graph nodes."),
        DeclareLaunchArgument(
            "memory_mode", default_value="world", choices=["world", "current"],
            description="world replays bounded world voxel memory; current learns from this frame."),
        DeclareLaunchArgument(
            "edge_support_radius", default_value="0.0",
            description="Exported edge surface support radius in metres; 0 disables this filter."),
        DeclareLaunchArgument("dd_updates", default_value="500",
                              description="Native DD-GNG updates per frame epoch."),
        DeclareLaunchArgument(
            "params_file", default_value=str(share / "config" / "dd_gng.yaml"),
            description="ROS parameter file for the camera and DD-GNG mapper nodes."),
    ]
    # One launch owns the camera and the shared RViz topics. Stop the session
    # when a component exits so a stopped mapper cannot look like a live map.
    handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason=reason))],
        ))
        for process, reason in [
            (camera, "DD-GNG camera process exited"),
            (mapper, "DD-GNG mapper process exited"),
            (rviz, "DD-GNG RViz window closed"),
        ]
    ]
    return LaunchDescription(declarations + handlers + [camera, mapper, rviz])
