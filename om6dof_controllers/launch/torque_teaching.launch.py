"""Pure-current (Dynamixel mode 0) gravity-compensated teaching launch.

Put the arm in its supported ready pose with the normal position launcher,
then stop that launcher before starting this one.  Mode 0 has no position
holding: while the controller is starting, physically support the arm.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    port_name = LaunchConfiguration("port_name")
    baud_rate = LaunchConfiguration("baud_rate")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    startup_delay = LaunchConfiguration("startup_delay")

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("om6dof_bringup"), "launch", "hardware.launch.py",
        ])),
        launch_arguments={
            "port_name": port_name,
            "baud_rate": baud_rate,
            "use_fake_hardware": use_fake_hardware,
            "current_control": "true",
            "arm_operating_mode": "0",
            # A trajectory/position controller is incompatible with mode 0.
            "start_arm_controller": "false",
        }.items(),
    )

    torque_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "om6dof_torque_teaching_controller",
            "--controller-manager", "/controller_manager",
            "--controller-type", "om6dof_controllers/GravityCompensationController",
            "--param-file", PathJoinSubstitution([
                FindPackageShare("om6dof_controllers"), "config",
                "om6dof_controllers.yaml",
            ]),
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "port_name",
            default_value="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT5NUUIQ-if00-port0",
        ),
        DeclareLaunchArgument("baud_rate", default_value="1000000"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("startup_delay", default_value="3.0"),
        hardware,
        TimerAction(period=startup_delay, actions=[torque_controller]),
    ])
