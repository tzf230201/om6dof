"""Start one selected human-input adapter for an existing OM6DOF controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    joint_velocity = LaunchConfiguration("joint_velocity")
    input_source = LaunchConfiguration("input_source")

    teleop = Node(
        package="om6dof_teleop",
        executable="teleop_node",
        name="om6dof_teleop",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "joint_velocity": ParameterValue(joint_velocity, value_type=float),
            "input_source": input_source,
        }],
    )

    stop_stack_if_gateway_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=teleop,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "joint_velocity",
            default_value="0.5",
            description="Remote JOINT-mode speed in rad/s.",
        ),
        DeclareLaunchArgument(
            "input_source",
            default_value="go2w",
            choices=["go2w", "keyboard", "gamepad", "airbus"],
            description="Exactly one source: Unitree remote, terminal, Logitech F710, or Airbus TCA.",
        ),
        teleop,
        stop_stack_if_gateway_exits,
    ])
