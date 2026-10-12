"""View dynamics using the running hardware model, TF and measured motor data."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    dynamic_share = FindPackageShare('om6dof_description')

    return LaunchDescription([
        DeclareLaunchArgument(
            'joints', default_value='external', choices=['external'],
            description='Measured feedback from the running hardware stack.'),
        DeclareLaunchArgument('labels', default_value='true', description='Show link mass labels.'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz.'),
        DeclareLaunchArgument(
            'rviz_config', default_value=PathJoinSubstitution([dynamic_share, 'rviz', 'dynamic.rviz']),
            description='RViz configuration file.'),
        DeclareLaunchArgument('jacobian', default_value='true', description='Show TCP Jacobian.'),
        DeclareLaunchArgument(
            'tip_link', default_value='end_effector_link', description='TCP link for the Jacobian.'),
        DeclareLaunchArgument(
            'jacobian_scale', default_value='0.4', description='Display scale for linear Jacobian columns.'),
        DeclareLaunchArgument(
            'description_topic', default_value='/robot_description',
            description='Robot description published by the running hardware stack.'),
        DeclareLaunchArgument(
            'dynamic_state_topic', default_value='/dynamic_joint_states',
            description='Measured joint and motor state interfaces from joint_state_broadcaster.'),
        DeclareLaunchArgument(
            'vector_mode', default_value='gravity', choices=['gravity', 'com_offset'],
            description='Weight from each CoM, or link-origin to CoM position vectors.'),
        DeclareLaunchArgument(
            'force_scale', default_value='0.05', description='Arrow display scale [m/N].'),
        DeclareLaunchArgument(
            'config', default_value=PathJoinSubstitution([
                dynamic_share, 'config', 'cog_vectors.yaml']),
            description='Vector node parameter file; vector_mode and force_scale override it.'),
        # The hardware stack owns the model, joint states and TF. This viewer
        # only subscribes; it never starts a hardware or joint-state publisher.
        Node(
            package='om6dof_description', executable='cog_vectors', name='om6dof_dynamic',
            parameters=[LaunchConfiguration('config'), {
                'robot_description': '',
                'description_topic': LaunchConfiguration('description_topic'),
                'dynamic_state_topic': LaunchConfiguration('dynamic_state_topic'),
                'enable_motor_feedback': True,
                'require_motor_feedback': True,
                'show_jacobian': ParameterValue(LaunchConfiguration('jacobian'), value_type=bool),
                'tip_link': LaunchConfiguration('tip_link'),
                'jacobian_scale': ParameterValue(LaunchConfiguration('jacobian_scale'), value_type=float),
                'show_mass_labels': ParameterValue(LaunchConfiguration('labels'), value_type=bool),
                'vector_mode': LaunchConfiguration('vector_mode'),
                'force_scale': ParameterValue(LaunchConfiguration('force_scale'), value_type=float),
            }], output='screen'),
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
