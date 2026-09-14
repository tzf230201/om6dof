"""Preview-only V2 reachability; no camera ownership or robot motion."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model_version", default_value="v2", choices=["v2"]),
        DeclareLaunchArgument("publish_robot_state", default_value="true", choices=["true", "false"]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(__file__).with_name("reachability_graph.launch.py"))),
            launch_arguments={
                "model_version": "v2",
                "publish_robot_state": LaunchConfiguration("publish_robot_state"),
            }.items(),
        ),
    ])
