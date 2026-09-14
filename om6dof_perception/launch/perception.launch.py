import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _nodes(context):
    version = LaunchConfiguration("model_version").perform(context)
    world = LaunchConfiguration("publish_world_point").perform(context).lower() == "true"
    if version not in ("v1", "v2"):
        raise ValueError("model_version must be v1 or v2")
    if world and version != "v2":
        raise ValueError("publish_world_point is a V2 nominal URDF preview; "
                         "V1 uses the existing external calibrated picker")
    nodes = [Node(
        package="om6dof_perception", executable="perception_node",
        name="om6dof_perception", output="screen",
        parameters=[{
            "model_version": version,
            "camera_model": ParameterValue(LaunchConfiguration("camera_model"), value_type=str),
            "camera_serial": ParameterValue(LaunchConfiguration("camera_serial"), value_type=str),
            "target_description": ParameterValue(LaunchConfiguration("target_description"), value_type=str),
            "target_class": ParameterValue(LaunchConfiguration("target_class"), value_type=str),
            "ee_mode": LaunchConfiguration("ee_mode"),
            "ee_left_pixel": LaunchConfiguration("ee_left_pixel"),
            "ee_right_pixel": LaunchConfiguration("ee_right_pixel"),
            "ee_pixels_calibrated": ParameterValue(LaunchConfiguration("ee_pixels_calibrated"), value_type=bool),
            "yolo_model_path": LaunchConfiguration("yolo_model_path"),
            "yolo_confidence": LaunchConfiguration("yolo_confidence"),
            "reacquire_period_sec": ParameterValue(
                LaunchConfiguration("reacquire_period_sec"), value_type=float),
            "frame_rate_hz": ParameterValue(
                LaunchConfiguration("frame_rate_hz"), value_type=float),
            "web_stream_topic": LaunchConfiguration("web_stream_topic"),
            "web_stream_max_width": ParameterValue(
                LaunchConfiguration("web_stream_max_width"), value_type=int),
            "publish_debug_image": True,
        }],
    )]
    if world:
        nodes.append(Node(
            package="om6dof_perception", executable="world_projection_node",
            name="om6dof_perception_projection", output="screen",
            parameters=[{"model_version": version}],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model_version", default_value="v1", choices=["v1", "v2"],
                              description="v1=D405, v2=D435; never mixed automatically"),
        DeclareLaunchArgument("camera_serial", default_value="",
                              description="SDK serial string; required if multiple matching cameras"),
        DeclareLaunchArgument("camera_model", default_value="auto",
                              choices=["auto", "D405", "D435", "D435i"],
                              description="auto preserves V1=D405/V2=D435; select D435i explicitly"),
        DeclareLaunchArgument("publish_world_point", default_value="false", choices=["true", "false"],
                              description="V2 read-only nominal FK preview; requires fresh joint_states"),
        DeclareLaunchArgument("ee_pixels_calibrated", default_value="false", choices=["true", "false"],
                              description="Acknowledge newly measured jaw pixels before enabling them on V2"),
        DeclareLaunchArgument(
            "target_description",
            default_value="glass jar with a black lid on the floor",
        ),
        DeclareLaunchArgument(
            "target_class",
            default_value="bottle",
        ),
        DeclareLaunchArgument(
            "ee_mode",
            default_value="auto",
            description="auto: V1 fixed_jaw_pixels, V2 disabled; or fixed_jaw_pixels/yolo/disabled",
        ),
        DeclareLaunchArgument(
            "ee_left_pixel", default_value="[200, 430]",
        ),
        DeclareLaunchArgument(
            "ee_right_pixel", default_value="[540, 400]",
        ),
        DeclareLaunchArgument(
            "yolo_model_path",
            default_value=(
                os.path.expanduser(
                    "~/.cache/om6dof_perception/yolox_s.onnx"
                )
            ),
        ),
        DeclareLaunchArgument(
            "yolo_confidence",
            default_value="0.35",
        ),
        DeclareLaunchArgument(
            "reacquire_period_sec", default_value="2.0",
            description="Seconds between full YOLO refreshes; tracking fills intermediate frames",
        ),
        DeclareLaunchArgument(
            "frame_rate_hz", default_value="20.0",
            description="Maximum capture/tracking/publishing loop rate",
        ),
        DeclareLaunchArgument(
            "web_stream_topic",
            default_value=(
                "/application_web_monitor/perception/image/compressed"
            ),
        ),
        DeclareLaunchArgument(
            "web_stream_max_width", default_value="0",
            description="Resize browser preview to this width; 0 preserves capture width",
        ),
        OpaqueFunction(function=_nodes),
    ])
