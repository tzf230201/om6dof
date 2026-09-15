"""Green graph + front insertion, treating only the selected object as graspable.

Owns perception through the ordinary experiment launch; never starts hardware
controllers. Motion still requires execution_enabled and the GUI Execute click.
Other object components and robot self-collision remain checked.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('camera_calibration_file', default_value=''),
        DeclareLaunchArgument('execution_enabled', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('launch_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('launch_yolo_2d', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('exact_replan_budget', default_value='256'),
        DeclareLaunchArgument('rmw_implementation', default_value='rmw_fastrtps_cpp'),
    ]
    forwarded = {argument.name: LaunchConfiguration(argument.name) for argument in arguments}
    forwarded.update({'pickup': 'true', 'sample_selection': 'green'})
    return LaunchDescription(arguments + [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('om6dof_dd_gng'), 'launch',
            'ddgng_experiment_reachability.launch.py'])),
        launch_arguments=forwarded.items())])
