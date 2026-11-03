"""Load the experimental plugin INACTIVE into an existing controller manager.

Does not open hardware, switch motor mode, move to READY, or activate a controller.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller_manager', default_value='/controller_manager'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('om6dof_controllers'), 'config', 'position_impedance.yaml'])),
        DeclareLaunchArgument('force_params_file', default_value=PathJoinSubstitution([
            FindPackageShare('om6dof_controllers'), 'config', 'cartesian_force.yaml'])),
        Node(package='controller_manager', executable='spawner', output='screen', arguments=[
            'om6dof_position_impedance_controller', '--inactive',
            '--controller-manager', LaunchConfiguration('controller_manager'),
            '--controller-type', 'om6dof_controllers/PositionImpedanceController',
            '--param-file', LaunchConfiguration('params_file'),
        ]),
        Node(package='om6dof_controllers', executable='impedance_target', output='screen',
             parameters=[{'controller_manager': LaunchConfiguration('controller_manager')}]),
        # Safe while disabled: publishes a single zero only.  A reviewed
        # nonzero wrench profile still needs explicit Mode-0 stack activation.
        Node(package='om6dof_controllers', executable='cartesian_force_bridge', output='screen',
             parameters=[LaunchConfiguration('force_params_file')]),
        # F710 owns the direction input in force mode. It publishes only
        # signs; cartesian_force_bridge applies the configured wrench limits.
        Node(package='om6dof_teleop', executable='teleop_node', name='om6dof_teleop',
             output='screen', parameters=[{'input_source': 'gamepad'}]),
    ])
