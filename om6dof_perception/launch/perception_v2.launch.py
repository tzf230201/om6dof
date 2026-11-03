"""D435 perception with a read-only V2 URDF coordinate preview, no robot control."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model_version", default_value="v2", choices=["v2"],
                              description="This dedicated launch always uses the V2/D435 model"),
        DeclareLaunchArgument("publish_world_point", default_value="true",
                              choices=["true", "false"]),
        DeclareLaunchArgument("camera_model", default_value="D435i",
                              choices=["D435", "D435i"]),
        DeclareLaunchArgument("camera_serial", default_value=""),
        DeclareLaunchArgument("target_class", default_value="bottle"),
        DeclareLaunchArgument("target_description", default_value="bottle"),
        DeclareLaunchArgument("detector_backend", default_value="yoloe",
                              choices=["yolox", "yoloe"]),
        DeclareLaunchArgument(
            "yoloe_model_path",
            default_value="~/.cache/om6dof_perception/yoloe-26n-seg.pt"),
        DeclareLaunchArgument("yoloe_classes", default_value='["bottle", "cup", "person"]'),
        DeclareLaunchArgument("yoloe_imgsz", default_value="640"),
        DeclareLaunchArgument("yoloe_device", default_value=""),
        DeclareLaunchArgument("segmentation_rate_hz", default_value="10.0"),
        # YOLOX-S takes roughly 0.5 s per CPU inference on this host. Refresh
        # as soon as the previous result can finish instead of idling 1.5 s.
        DeclareLaunchArgument("reacquire_period_sec", default_value="0.5"),
        # Leave CPU headroom for YOLO/CSRT and halve browser JPEG bandwidth.
        DeclareLaunchArgument("frame_rate_hz", default_value="15.0"),
        DeclareLaunchArgument("web_stream_max_width", default_value="480"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("om6dof_perception") +
                "/launch/perception.launch.py"),
            launch_arguments={
                "model_version": "v2",
                "publish_world_point": LaunchConfiguration("publish_world_point"),
                "camera_model": LaunchConfiguration("camera_model"),
                "camera_serial": LaunchConfiguration("camera_serial"),
                "target_class": LaunchConfiguration("target_class"),
                "target_description": LaunchConfiguration("target_description"),
                "detector_backend": LaunchConfiguration("detector_backend"),
                "yoloe_model_path": LaunchConfiguration("yoloe_model_path"),
                "yoloe_classes": LaunchConfiguration("yoloe_classes"),
                "yoloe_imgsz": LaunchConfiguration("yoloe_imgsz"),
                "yoloe_device": LaunchConfiguration("yoloe_device"),
                "segmentation_rate_hz": LaunchConfiguration("segmentation_rate_hz"),
                "reacquire_period_sec": LaunchConfiguration("reacquire_period_sec"),
                "frame_rate_hz": LaunchConfiguration("frame_rate_hz"),
                "web_stream_max_width": LaunchConfiguration("web_stream_max_width"),
            }.items(),
        ),
    ])
