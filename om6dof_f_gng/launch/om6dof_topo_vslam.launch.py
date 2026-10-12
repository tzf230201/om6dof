"""om6dof_topo_vslam stage 1: robot model plus live RealSense depth in RViz.

No GNG and no topological map yet. This launch only shows the arm and the
world-frame points its wrist camera measures, which is what the later mapping
stages will consume.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = Path(get_package_share_directory("om6dof_f_gng"))
    params_file = LaunchConfiguration("params_file")
    world_frame = LaunchConfiguration("world_frame")
    camera_frame = LaunchConfiguration("camera_frame")
    inputs = [
        ("depth/image_raw", LaunchConfiguration("depth_topic")),
        ("depth/camera_info", LaunchConfiguration("camera_info_topic")),
    ]
    color_outputs = [
        ("color/image_raw", LaunchConfiguration("color_topic")),
        ("color/camera_info", LaunchConfiguration("color_info_topic")),
    ]

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([
                FindPackageShare("om6dof_description"), "urdf",
                LaunchConfiguration("urdf_file"),
            ]),
        ]),
        value_type=str,
    )
    # Leave this off when om6dof_bringup already publishes the model, or the
    # TF tree gets two publishers for the same links.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="topo_vslam_robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        condition=IfCondition(LaunchConfiguration("start_robot_state_publisher")),
    )
    # Without the hardware there are no joint states, so no world -> camera
    # pose and no cloud. The GUI stands in for the arm when it is not running.
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="topo_vslam_joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_joint_gui")),
    )
    camera = Node(
        package="om6dof_f_gng",
        executable="realsense_depth_node",
        namespace="om6dof_topo_vslam",
        name="realsense_depth",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_camera")),
        parameters=[params_file, {
            "camera_frame": ParameterValue(camera_frame, value_type=str),
            "serial": ParameterValue(LaunchConfiguration("camera_serial"), value_type=str),
            "enable_color": ParameterValue(LaunchConfiguration("enable_color"), value_type=bool),
            "color_frame": ParameterValue(LaunchConfiguration("color_frame"), value_type=str),
        }],
        remappings=inputs + color_outputs,
    )
    projector = Node(
        package="om6dof_f_gng",
        executable="om6dof_topo_vslam",
        namespace="om6dof_topo_vslam",
        name="topo_vslam",
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
        name="topo_vslam_rviz",
        output="screen",
        arguments=["-d", str(share / "rviz" / "om6dof_topo_vslam.rviz"), "-f", world_frame],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    declarations = [
        DeclareLaunchArgument("use_rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument(
            "start_camera", default_value="true", choices=["true", "false"],
            description="Open RealSense directly; false subscribes to existing ROS depth topics."),
        DeclareLaunchArgument(
            "start_robot_state_publisher", default_value="true", choices=["true", "false"],
            description="Set false when om6dof_bringup already publishes robot_description."),
        DeclareLaunchArgument(
            "start_joint_gui", default_value="false", choices=["true", "false"],
            description="Slider GUI for joint states when the arm is not running."),
        DeclareLaunchArgument(
            "urdf_file", default_value="om6dof_v2.urdf.xacro",
            description="Model in om6dof_description/urdf to display."),
        DeclareLaunchArgument(
            "depth_topic", default_value="/om6dof_topo_vslam/depth/image_raw",
            description="Rectified depth image topic (32FC1 metres or 16UC1 millimetres)."),
        DeclareLaunchArgument(
            "camera_info_topic", default_value="/om6dof_topo_vslam/depth/camera_info",
            description="CameraInfo matching the depth image resolution and optical frame."),
        DeclareLaunchArgument(
            "enable_color", default_value="true", choices=["true", "false"],
            description="Also stream RGB from the same device for the RViz camera window."),
        DeclareLaunchArgument(
            "color_topic", default_value="/om6dof_topo_vslam/color/image_raw",
            description="RGB8 viewing image; not aligned to depth."),
        DeclareLaunchArgument(
            "color_info_topic", default_value="/om6dof_topo_vslam/color/camera_info"),
        DeclareLaunchArgument("color_frame", default_value="d435_color_optical_frame"),
        DeclareLaunchArgument("world_frame", default_value="world"),
        DeclareLaunchArgument("camera_frame", default_value="d435_depth_optical_frame"),
        DeclareLaunchArgument("camera_serial", default_value=""),
        DeclareLaunchArgument(
            "params_file", default_value=str(share / "config" / "om6dof_topo_vslam.yaml"),
            description="ROS parameter file for the camera and projector nodes."),
    ]
    # One launch owns the camera and the RViz topics. Stop the session when a
    # component exits so a frozen cloud cannot look like a live measurement.
    handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason=reason))],
        ))
        for process, reason in [
            (camera, "topo_vslam camera process exited"),
            (projector, "topo_vslam depth projector exited"),
            (rviz, "topo_vslam RViz window closed"),
        ]
    ]
    return LaunchDescription(declarations + handlers + [
        robot_state_publisher, joint_state_publisher_gui, camera, projector, rviz])
