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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    share_dir = get_package_share_directory('om6dof_dd_gng')
    default_params = os.path.join(share_dir, 'config', 'topo_gng.yaml')
    default_rviz = os.path.join(share_dir, 'rviz', 'topo_gng.rviz')
    params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    launch_rviz = LaunchConfiguration('launch_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='YAML parameters for topo_gng_node',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Also start RViz2 pre-configured for environment_graph/robot_graph',
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
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(launch_rviz),
        ),
    ])
