"""Launch topo_gng_node (+ RViz by default) with its default parameter file.

Does NOT bring up robot_state_publisher or the D405 -- run this against an
already-running bringup (topo_gng_node needs TF from world down to
d405_depth_optical_frame, and exclusive access to the RealSense: stop
om6dof-dd-gng.service / om6dof-perception.service first).

RViz needs a real X display: run this from a terminal inside the session you
want it to appear in (e.g. a NoMachine session, DISPLAY=:1002 here), or pass
launch_rviz:=false for headless use (the RViz-on-another-machine workflow
described in TopoVLA/CONNECTING_TO_MSI_AND_AGX.md).
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


def _sha256(payload):
    return hashlib.sha256(payload).hexdigest()


def _launch_setup(context, *, moveit_share, description_share):
    """Resolve the reachability node's model/parameter inputs and hash them.

    reachability_graph_node refuses to start unless expanded_urdf_sha256,
    srdf_sha256, and reachability_parameters_sha256 are valid SHA-256 digests
    of the exact bytes it was handed -- mirrors reachability_graph.launch.py.
    """
    params_path = Path(
        LaunchConfiguration('params_file').perform(context)
    ).expanduser().resolve(strict=True)
    srdf_path = Path(moveit_share) / 'config' / 'om6dof.srdf'
    xacro_path = Path(description_share) / 'urdf' / 'om6dof.urdf.xacro'
    xacro_executable = shutil.which('xacro')
    if not xacro_executable:
        raise RuntimeError('could not resolve the xacro executable')

    expanded = subprocess.run(
        [xacro_executable, os.fspath(xacro_path)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if expanded.returncode != 0:
        detail = expanded.stderr.decode('utf-8', errors='replace').strip()
        raise RuntimeError(f'xacro expansion failed: {detail}')

    urdf_bytes = expanded.stdout
    srdf_bytes = srdf_path.read_bytes()
    params_bytes = params_path.read_bytes()
    robot_description = urdf_bytes.decode('utf-8')
    robot_description_semantic = srdf_bytes.decode('utf-8')

    return [
        Node(
            package='om6dof_dd_gng',
            executable='reachability_graph_node',
            name='reachability_graph_node',
            output='screen',
            parameters=[
                os.fspath(params_path),
                {
                    'robot_description': robot_description,
                    'robot_description_semantic': robot_description_semantic,
                    'expanded_urdf_sha256': _sha256(urdf_bytes),
                    'srdf_sha256': _sha256(srdf_bytes),
                    'reachability_parameters_sha256': _sha256(params_bytes),
                },
            ],
            condition=IfCondition(LaunchConfiguration('launch_reachability')),
        ),
    ]


def generate_launch_description():
    share_dir = get_package_share_directory('om6dof_dd_gng')
    default_params = os.path.join(share_dir, 'config', 'topo_gng.yaml')
    default_rviz = os.path.join(share_dir, 'rviz', 'topo_gng.rviz')
    params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    launch_rviz = LaunchConfiguration('launch_rviz')

    moveit_share = get_package_share_directory('om6dof_moveit_config')
    description_share = get_package_share_directory('om6dof_description')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='YAML parameters for topo_gng_node and reachability_graph_node',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Also start RViz2 pre-configured for environment_graph/robot_graph',
        ),
        DeclareLaunchArgument(
            'launch_reachability',
            default_value='true',
            description='Start the preview-only end-effector reachability graph node',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='RViz config used when launch_rviz:=true',
        ),
        Node(
            package='om6dof_dd_gng',
            executable='topo_gng_node',
            name='topo_gng_node',
            output='screen',
            parameters=[params_file],
        ),
        OpaqueFunction(
            function=_launch_setup,
            kwargs={
                'moveit_share': moveit_share,
                'description_share': description_share,
            },
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(launch_rviz),
        ),
    ])
