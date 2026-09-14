"""RViz + semantic planning-target GUI; no standalone 2D YOLO window."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("om6dof_dd_gng"))
    pick_share = Path(get_package_share_directory("om6dof_pick_and_place"))
    return LaunchDescription([
        DeclareLaunchArgument("execution_enabled", default_value="false"),
        DeclareLaunchArgument("camera_calibration_file", default_value=""),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / "topo_gng_v2.launch.py")),
            launch_arguments={
                "launch_reachability": "true",
                "launch_rviz": "true",
                "camera_calibration_file": LaunchConfiguration("camera_calibration_file"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pick_share / "launch" / "graph_pick.launch.py")),
            launch_arguments={"execution_enabled": LaunchConfiguration("execution_enabled")}.items(),
        ),
        Node(
            package="om6dof_dd_gng",
            executable="semantic_target_gui.py",
            name="semantic_target_gui",
            output="screen",
        ),
    ])
