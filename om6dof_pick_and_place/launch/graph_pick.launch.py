"""Start the semantic DD-GNG pickup coordinator.

This launch assumes V2 perception and reachability are already running. It
does not open a camera, launch a controller, enable torque, or move the robot.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("om6dof_pick_and_place"),
            "config",
            "graph_pick.yaml",
        ]),
    )
    execution = DeclareLaunchArgument(
        "execution_enabled",
        default_value="false",
        description=(
            "Expose explicit physical execution after all live guards pass. "
            "Planning and preview remain available when false."
        ),
    )
    middleware = DeclareLaunchArgument(
        "rmw_implementation", default_value="rmw_fastrtps_cpp",
        description="DDS implementation used by the hardware controller manager",
    )
    coordinator = Node(
        package="om6dof_pick_and_place",
        executable="graph_pick_node",
        name="graph_pick",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {"execution_enabled": LaunchConfiguration("execution_enabled")},
        ],
        additional_env={"RMW_IMPLEMENTATION": LaunchConfiguration("rmw_implementation")},
        emulate_tty=True,
    )
    return LaunchDescription([config, execution, middleware, coordinator])
