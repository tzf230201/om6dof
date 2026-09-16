"""Paper-faithful ATC-DT mapper configured for Intel RealSense D435."""

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
    inputs = [("depth/image_raw", LaunchConfiguration("depth_topic")),
              ("depth/camera_info", LaunchConfiguration("camera_info_topic"))]
    camera = Node(package="om6dof_f_gng", executable="realsense_depth_node",
                  namespace="om6dof_f_gng", name="realsense_depth", output="screen",
                  condition=IfCondition(LaunchConfiguration("start_camera")), parameters=[params_file, {
                      "camera_frame": ParameterValue(camera_frame, value_type=str),
                      "serial": ParameterValue(LaunchConfiguration("camera_serial"), value_type=str)}],
                  remappings=inputs)
    mapper = Node(package="om6dof_f_gng", executable="f_gng_node", namespace="om6dof_f_gng",
                  name="atc_dt", output="screen", parameters=[params_file, {
                      "algorithm": "atcdt", "memory_mode": "current",
                      "world_frame": ParameterValue(world_frame, value_type=str),
                      "camera_frame": ParameterValue(camera_frame, value_type=str),
                      "max_nodes": ParameterValue(LaunchConfiguration("max_nodes"), value_type=int),
                      "atc_vigilance": ParameterValue(LaunchConfiguration("vigilance"), value_type=float)}],
                  remappings=inputs)
    rviz = Node(package="rviz2", executable="rviz2", name="atc_dt_rviz", output="screen",
                arguments=["-d", str(share / "rviz" / "dd_gng.rviz"), "-f", world_frame],
                condition=IfCondition(LaunchConfiguration("use_rviz")))
    declarations = [
        DeclareLaunchArgument("use_rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("start_camera", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("depth_topic", default_value="/om6dof_f_gng/depth/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/om6dof_f_gng/depth/camera_info"),
        DeclareLaunchArgument("world_frame", default_value="world"),
        DeclareLaunchArgument("camera_frame", default_value="d435_depth_optical_frame"),
        DeclareLaunchArgument("camera_serial", default_value=""),
        DeclareLaunchArgument("max_nodes", default_value="1000"),
        DeclareLaunchArgument("vigilance", default_value="0.01",
                              description="ATC-DT V_thr in metres; larger means a coarser graph."),
        DeclareLaunchArgument("params_file", default_value=str(share / "config" / "atc_dt_d435.yaml")),
    ]
    handlers = [RegisterEventHandler(OnProcessExit(target_action=process,
        on_exit=[EmitEvent(event=Shutdown(reason=reason))])) for process, reason in [
            (camera, "D435 camera process exited"), (mapper, "ATC-DT mapper process exited"),
            (rviz, "ATC-DT RViz window closed")]]
    return LaunchDescription(declarations + handlers + [camera, mapper, rviz])
