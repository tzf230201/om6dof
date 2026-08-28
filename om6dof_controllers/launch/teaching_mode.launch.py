"""Bring up a safe gravity-compensated teaching session for the real OM6DOF.

This launch owns the Dynamixel port.  Stop any existing hardware/bringup
launch before starting it.  The arm is placed in current-based position mode
(Dynamixel operating mode 5), then only the leader-arm controller is allowed
to claim its position and effort command interfaces.
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler,
                            TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    port_name = LaunchConfiguration("port_name")
    baud_rate = LaunchConfiguration("baud_rate")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    startup_delay = LaunchConfiguration("startup_delay")
    ready_speed = LaunchConfiguration("ready_speed")

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("om6dof_bringup"), "launch", "hardware.launch.py",
        ])),
        launch_arguments={
            "port_name": port_name,
            "baud_rate": baud_rate,
            "use_fake_hardware": use_fake_hardware,
            # Mode 5 exposes effort commands while the motor retains its
            # internal position loop.  LeaderArmController needs both.
            "current_control": "true",
            # This controller owns the ready-pose move.  It is stopped only
            # after that move succeeds, before the leader takes ownership.
            "start_arm_controller": "true",
        }.items(),
    )

    leader_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "om6dof_leader_arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-type", "om6dof_controllers/LeaderArmController",
            "--inactive",
            "--param-file", PathJoinSubstitution([
                FindPackageShare("om6dof_controllers"), "config",
                "om6dof_controllers.yaml",
            ]),
        ],
        output="screen",
    )

    # Keep the switch in the same process chain as the move: ``&&`` means a
    # rejected/timed-out ready-pose action cannot accidentally enable teaching
    # mode.  The ready-speed is passed as a positional shell argument rather
    # than interpolated into the command string.
    move_ready_then_activate = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "ros2 run om6dof_controllers multisine_identification.py "
            "--execute --ready-only --ready-speed \"$1\" && "
            "ros2 control switch_controllers "
            "--controller-manager /controller_manager "
            "--deactivate arm_controller "
            "--activate om6dof_leader_arm_controller --strict",
            "teaching_mode", ready_speed,
        ],
        output="screen",
    )
    move_after_leader_is_loaded = RegisterEventHandler(
        OnProcessExit(
            target_action=leader_spawner,
            on_exit=[move_ready_then_activate],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "port_name",
            default_value="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT5NUUIQ-if00-port0",
        ),
        DeclareLaunchArgument("baud_rate", default_value="1000000"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument(
            "ready_speed", default_value="0.12",
            description="maximum ready-pose speed in rad/s",
        ),
        # The spawner itself waits for controller_manager; this small delay
        # also leaves time for the hardware and joint-state broadcaster to
        # complete their initialisation before command interfaces are claimed.
        DeclareLaunchArgument("startup_delay", default_value="3.0"),
        bringup,
        TimerAction(period=startup_delay, actions=[leader_spawner]),
        move_after_leader_is_loaded,
    ])
