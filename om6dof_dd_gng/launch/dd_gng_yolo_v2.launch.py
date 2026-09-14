"""V2/D435i camera-space DD-GNG; optional serial, desktop preview by default."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model_version", default_value="v2", choices=["v2"]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(__file__).with_name("dd_gng_yolo.launch.py"))),
            launch_arguments={"model_version": "v2"}.items(),
        ),
    ])
