"""Camera-space segmentation preview/web stream; no robot or motor commands."""

from pathlib import Path
import sys

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _launch_setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    script = Path(get_package_prefix("om6dof_dd_gng")) / "lib/om6dof_dd_gng/dd_gng_yolo.py"
    command = [
        sys.executable, str(script),
        "--model-version", value("model_version"),
        "--camera-model", value("camera_model"),
        "--camera-fps", value("camera_fps"),
        "--ros-topic", value("image_topic"),
        "--labels-topic", value("labels_topic"),
    ]
    # An omitted serial means auto-select one matching camera. Do not require
    # users to pass camera_serial:="" (the ROS launch CLI rejects that token).
    serial = value("camera_serial").strip()
    if serial:
        command.extend(["--camera-serial", serial])
    if value("headless").lower() == "true":
        command.append("--headless")
    # This argparse-based script accepts its own flags, not Node's --ros-args.
    return [ExecuteProcess(cmd=command, output="screen", emulate_tty=True)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model_version", default_value="v1", choices=["v1", "v2"]),
        DeclareLaunchArgument("camera_model", default_value="auto",
                              choices=["auto", "D405", "D435", "D435i"],
                              description="auto: V1 D405; V2 D435i"),
        DeclareLaunchArgument("camera_serial", default_value="",
                              description="Optional; omit for one matching camera"),
        DeclareLaunchArgument("camera_fps", default_value="30"),
        DeclareLaunchArgument("headless", default_value="false", choices=["true", "false"],
                              description="Disable the desktop OpenCV window"),
        DeclareLaunchArgument("image_topic", default_value="/application_web_monitor/ddgng/image/compressed"),
        DeclareLaunchArgument("labels_topic", default_value="/application_web_monitor/ddgng/labels"),
        OpaqueFunction(function=_launch_setup),
    ])
