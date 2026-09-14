"""Combined trajectory/joystick reference preview; starts no hardware owner."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('target_gui', default_value='true'),
        DeclareLaunchArgument('gamepad', default_value='false',
                              description='Read Logitech for preview; stop legacy teleop first.'),
        DeclareLaunchArgument('trajectory_topic', default_value=
                              '/om6dof_topo_gng_v2/graph_pick/trajectory_preview'),
        Node(package='om6dof_controller', executable='combined_preview_node',
             output='screen', parameters=[{'trajectory_topic': LaunchConfiguration('trajectory_topic')}]),
        Node(package='om6dof_teleop', executable='combined_preview_gamepad',
             output='screen', condition=IfCondition(LaunchConfiguration('gamepad'))),
        Node(package='om6dof_dd_gng', executable='semantic_target_gui.py',
             name='combined_preview_target_gui', parameters=[{'preview_only': True}],
             condition=IfCondition(LaunchConfiguration('target_gui')), output='screen'),
        Node(package='rviz2', executable='rviz2', name='combined_preview_rviz',
             arguments=['-d', PathJoinSubstitution([
                 FindPackageShare('om6dof_controller'), 'rviz', 'combined_preview.rviz'])],
             remappings=[('/tf', '/om6dof_dd_gng/v2/tf'),
                         ('/tf_static', '/om6dof_dd_gng/v2/tf_static')],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
