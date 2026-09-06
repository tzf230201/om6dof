"""Launch the preview-only OM6DOF end-effector reachability roadmap.

This launch does not start a camera, move_group, ros2_control, or a hardware
controller. It loads the URDF/SRDF locally for FK and collision checks.
"""

import hashlib
import os
import shutil
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _sha256(payload):
    return hashlib.sha256(payload).hexdigest()


def _launch_setup(context, *, moveit_share, description_share):
    """Resolve every model input once, hash the exact bytes, then launch."""
    params_path = Path(
        LaunchConfiguration("params_file").perform(context)
    ).expanduser().resolve(strict=True)
    srdf_path = Path(moveit_share) / "config" / "om6dof.srdf"
    xacro_path = Path(description_share) / "urdf" / "om6dof.urdf.xacro"
    xacro_executable = shutil.which("xacro")
    if not xacro_executable:
        raise RuntimeError("could not resolve the xacro executable")

    expanded = subprocess.run(
        [xacro_executable, os.fspath(xacro_path)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if expanded.returncode != 0:
        detail = expanded.stderr.decode("utf-8", errors="replace").strip()
        raise RuntimeError(f"xacro expansion failed: {detail}")

    # Hash the same byte sequences whose decoded strings/path are handed to the
    # node.  Reading the parameter file here also binds provenance to the
    # actually resolved launch argument, not merely the package default.
    urdf_bytes = expanded.stdout
    srdf_bytes = srdf_path.read_bytes()
    params_bytes = params_path.read_bytes()
    robot_description = urdf_bytes.decode("utf-8")
    robot_description_semantic = srdf_bytes.decode("utf-8")

    graph_method = LaunchConfiguration("graph_method")
    sample_count = LaunchConfiguration("sample_count")
    halton_start_index = LaunchConfiguration("halton_start_index")
    sample_stream_seed = LaunchConfiguration("sample_stream_seed")
    gng_guard_fraction = LaunchConfiguration("gng_guard_fraction")
    query_mode = LaunchConfiguration("query_mode")

    return [
        Node(
            package="om6dof_dd_gng",
            executable="reachability_graph_node",
            name="reachability_graph_node",
            output="screen",
            parameters=[
                os.fspath(params_path),
                {
                    "robot_description": robot_description,
                    "robot_description_semantic": robot_description_semantic,
                    "expanded_urdf_sha256": _sha256(urdf_bytes),
                    "srdf_sha256": _sha256(srdf_bytes),
                    "reachability_parameters_sha256": _sha256(params_bytes),
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
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_reachability",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
        ),
    ]


def generate_launch_description():
    share_dir = get_package_share_directory("om6dof_dd_gng")
    moveit_share = get_package_share_directory("om6dof_moveit_config")
    default_params = os.path.join(share_dir, "config", "topo_gng.yaml")
    default_rviz = os.path.join(share_dir, "rviz", "topo_gng.rviz")

    description_share = get_package_share_directory("om6dof_description")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file", default_value=default_params,
            description="Shared topo/reachability parameter file",
        ),
        DeclareLaunchArgument(
            "launch_rviz", default_value="true",
            description="Start RViz with the reachability display",
        ),
        DeclareLaunchArgument(
            "rviz_config", default_value=default_rviz,
            description="RViz configuration",
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
            },
        ),
    ])
