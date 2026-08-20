import os
import time
"""Canonical runtime: hardware owner -> command converter -> input adapter."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare



# The dashboard needs a way to bring the stack up in current-control mode
# without editing a systemd unit it cannot write. A flag file is the whole
# mechanism: the GUI writes it, then triggers the restart it is already
# allowed to trigger, and this reads it. Missing file means position mode.
CURRENT_CONTROL_FLAG = os.path.expanduser("~/.config/om6dof/current_control")

# The flag expires, so position mode is what every restart lands in unless the
# GUI asked for current control moments earlier. A latching flag meant a leader
# session enabled once kept every later boot -- including a reboot days on --
# in current-based mode, silently inheriting a setting nobody remembered
# choosing. Expiry rather than delete-on-read because both this file and
# full_stack.launch.py read the flag during one launch, and a consuming read
# would give whichever ran second the opposite answer.
CURRENT_CONTROL_TTL_S = 180.0


def current_control_default() -> str:
    try:
        with open(CURRENT_CONTROL_FLAG) as handle:
            requested = handle.read().strip().lower() == "true"
        age = time.time() - os.path.getmtime(CURRENT_CONTROL_FLAG)
    except OSError:
        return "false"
    return "true" if requested and age <= CURRENT_CONTROL_TTL_S else "false"

def generate_launch_description():
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("om6dof_bringup"), "launch", "hardware.launch.py",
        ])),
        launch_arguments={
            "port_name": LaunchConfiguration("port_name"),
            "baud_rate": LaunchConfiguration("baud_rate"),
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "current_control": LaunchConfiguration("current_control"),
        }.items(),
    )
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("om6dof_controller"), "launch", "controller.launch.py",
        ])),
        launch_arguments={
            "remote_enabled_on_start": LaunchConfiguration(
                "remote_enabled_on_start"
            ),
        }.items(),
    )
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("om6dof_teleop"), "launch", "teleop.launch.py",
        ])),
        launch_arguments={
            "joint_velocity": LaunchConfiguration("joint_velocity"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_go2w_teleop")),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "port_name",
            default_value="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT5NUUIQ-if00-port0",
        ),
        DeclareLaunchArgument("baud_rate", default_value="1000000"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        # Opt-in: loads the effort-capable description so gravity
        # compensation can reach the motors. The arm is softer with it on.
        DeclareLaunchArgument("current_control",
                              default_value=current_control_default()),
        DeclareLaunchArgument("joint_velocity", default_value="0.5"),
        DeclareLaunchArgument(
            "start_go2w_teleop",
            default_value="true",
            description="Start the Go2W wireless-controller adapter.",
        ),
        DeclareLaunchArgument("remote_enabled_on_start", default_value="false"),
        hardware,
        controller,
        teleop,
    ])
