"""Desktop analysis: one D435i owner, YOLOX 2D window, and RViz clusters."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("om6dof_dd_gng"))
    return LaunchDescription([
        DeclareLaunchArgument("display_window", default_value="true"),
        DeclareLaunchArgument("launch_reachability", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(share / "launch" / "topo_gng_v2.launch.py")),
            launch_arguments={
                "launch_reachability": LaunchConfiguration("launch_reachability"),
                "launch_rviz": "true",
            }.items(),
        ),
        # This node subscribes to V2 raw RGB; it never opens a second camera.
        Node(
            package="om6dof_dd_gng",
            executable="yolox_viewer_node",
            name="yolox_2d_viewer",
            output="screen",
            parameters=[{
                "input_topic": "/om6dof_topo_gng_v2/rgb/image/compressed",
                "output_topic": "/om6dof_yolox/debug_image/compressed",
                "display_window": LaunchConfiguration("display_window"),
            }],
        ),
    ])
