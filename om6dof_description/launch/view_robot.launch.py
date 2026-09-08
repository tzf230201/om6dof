from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([
                FindPackageShare("om6dof_description"), "urdf", "om6dof.urdf.xacro"
            ]),
        ]),
        value_type=str,
    )
    # Bare rviz2 opens its default config: no RobotModel display, and a fixed
    # frame of "map" that this URDF never publishes. Both have to come from a
    # config file or the window comes up empty.
    rviz_config = PathJoinSubstitution([
        FindPackageShare("om6dof_description"), "rviz", "view_robot.rviz"
    ])
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": description}],
            output="screen",
        ),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
