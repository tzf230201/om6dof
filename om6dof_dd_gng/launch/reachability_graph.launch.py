"""Launch the preview-only OM6DOF end-effector reachability roadmap.

This launch does not start a camera, move_group, ros2_control, or a hardware
controller. It loads the URDF/SRDF locally for FK and collision checks.
"""

import importlib.util
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_spec = importlib.util.spec_from_file_location(
    "dd_gng_model_inputs", Path(__file__).with_name("_model_inputs.py"))
_model_inputs = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_model_inputs)


def _launch_setup(context, *, moveit_share, description_share, dd_gng_share):
    """Resolve every model input once, hash the exact bytes, then launch."""
    inputs = _model_inputs.resolve_model_inputs(
        context, moveit_share=moveit_share, description_share=description_share,
        dd_gng_share=dd_gng_share)
    rviz_path = LaunchConfiguration("rviz_config").perform(context) or os.path.join(
        dd_gng_share, "rviz", "topo_gng_v2.rviz" if inputs["version"] == "v2" else "topo_gng.rviz")

    graph_method = LaunchConfiguration("graph_method")
    sample_count = LaunchConfiguration("sample_count")
    halton_start_index = LaunchConfiguration("halton_start_index")
    sample_stream_seed = LaunchConfiguration("sample_stream_seed")
    gng_guard_fraction = LaunchConfiguration("gng_guard_fraction")
    query_mode = LaunchConfiguration("query_mode")

    return [
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name=f"dd_gng_{inputs['version']}_state_publisher", output="screen",
            parameters=[{"robot_description": inputs["model_parameters"]["robot_description"]}],
            remappings=inputs["remappings"],
            condition=IfCondition(LaunchConfiguration("publish_robot_state")),
        ),
        Node(
            package="om6dof_dd_gng",
            executable="reachability_graph_node",
            name="reachability_graph_node",
            output="screen",
            parameters=[
                inputs["params_path"],
                {
                    **inputs["model_parameters"],
                    "graph_method": graph_method,
                    "sample_count": ParameterValue(sample_count, value_type=int),
                    "halton_start_index": ParameterValue(
                        halton_start_index, value_type=int
                    ),
                    "sample_stream_seed": ParameterValue(
                        sample_stream_seed, value_type=int
                    ),
                    "gng_guard_fraction": ParameterValue(
                        gng_guard_fraction, value_type=float
                    ),
                    "query_mode": ParameterValue(query_mode, value_type=bool),
                },
            ],
            remappings=inputs["remappings"],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_reachability",
            output="screen",
            arguments=["-d", rviz_path],
            remappings=inputs["remappings"],
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
        ),
    ]


def generate_launch_description():
    share_dir = get_package_share_directory("om6dof_dd_gng")
    moveit_share = get_package_share_directory("om6dof_moveit_config")

    description_share = get_package_share_directory("om6dof_description")

    return LaunchDescription([
        DeclareLaunchArgument("model_version", default_value="v1", choices=["v1", "v2"]),
        DeclareLaunchArgument(
            "params_file", default_value="",
            description="Empty selects topo_gng.yaml (V1) or topo_gng_v2.yaml (V2)",
        ),
        DeclareLaunchArgument(
            "publish_robot_state", default_value="false", choices=["true", "false"],
            description="Read-only state publisher; V2 TF is isolated from hardware TF",
        ),
        DeclareLaunchArgument(
            "launch_rviz", default_value="true",
            description="Start RViz with the reachability display",
        ),
        DeclareLaunchArgument(
            "rviz_config", default_value="",
            description="Empty selects the model-specific RViz preset",
        ),
        DeclareLaunchArgument(
            "graph_method", default_value="gng",
            description="Reachability backend: gng, guarded_gng, or halton_prm",
        ),
        DeclareLaunchArgument(
            "sample_count", default_value="800",
            description="Matched roadmap node budget",
        ),
        DeclareLaunchArgument(
            "halton_start_index", default_value="17",
            description="Deterministic index offset within the sample stream",
        ),
        DeclareLaunchArgument(
            "sample_stream_seed", default_value="0",
            description="Zero for legacy Halton; positive for digit-permuted Halton",
        ),
        DeclareLaunchArgument(
            "gng_guard_fraction", default_value="0.25",
            description="Raw deterministic sample fraction reserved by guarded_gng",
        ),
        DeclareLaunchArgument(
            "query_mode", default_value="false",
            description="Use atomic preview-only benchmark queries instead of live inputs",
        ),
        OpaqueFunction(
            function=_launch_setup,
            kwargs={
                "moveit_share": moveit_share,
                "description_share": description_share,
                "dd_gng_share": share_dir,
            },
        ),
    ])
