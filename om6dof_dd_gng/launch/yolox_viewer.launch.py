from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'input_topic', default_value='/om6dof_topo_gng_v2/rgb/image/compressed'),
        DeclareLaunchArgument(
            'output_topic', default_value='/om6dof_yolox/debug_image/compressed'),
        DeclareLaunchArgument('display_window', default_value='false'),
        Node(
            package='om6dof_dd_gng', executable='yolox_viewer_node', name='yolox_viewer',
            output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'display_window': LaunchConfiguration('display_window'),
            }]),
    ])
