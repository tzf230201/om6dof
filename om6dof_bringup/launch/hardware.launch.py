import os
import time
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    port_name = LaunchConfiguration("port_name")
    baud_rate = LaunchConfiguration("baud_rate")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    current_control = LaunchConfiguration("current_control")
    arm_operating_mode = LaunchConfiguration("arm_operating_mode")
    start_arm_controller = LaunchConfiguration("start_arm_controller")
    package_share = FindPackageShare("om6dof_bringup")

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([package_share, "urdf", "om6dof.urdf.xacro"]),
            " port_name:=", port_name,
            " baud_rate:=", baud_rate,
            " use_fake_hardware:=", use_fake_hardware,
            " current_control:=", current_control,
            " arm_operating_mode:=", arm_operating_mode,
        ]),
        value_type=str,
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[PathJoinSubstitution([package_share, "config", "controllers.yaml"])],
        remappings=[("~/robot_description", "/robot_description")],
        output="screen",
    )

    joint_state_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    arm_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        condition=IfCondition(start_arm_controller),
    )
    gripper_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )
    # Only exists in current-control mode: it claims the effort command
    # interfaces, which the position description does not declare. Loaded
    # inactive, so bringing the stack up in current mode still leaves the arm
    # under the position loop until something deliberately activates this.
    # Without it the gravity compensation node published to a controller that
    # was not there, and the arm never felt a thing.
    forward_effort_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=[
            "forward_effort_controller",
            "--controller-manager", "/controller_manager",
            "--inactive",
        ],
        condition=IfCondition(current_control),
    )
    forward_position_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=[
            "forward_position_controller",
            "--controller-manager", "/controller_manager",
            "--inactive",
        ],
    )
    start_motion_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=[arm_spawner, gripper_spawner, forward_position_spawner,
                     forward_effort_spawner],
        )
    )
    stop_launch_if_hardware_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=control_node,
            on_exit=[EmitEvent(event=Shutdown(
                reason="ros2_control hardware owner exited",
            ))],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("port_name", default_value="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT5NUUIQ-if00-port0"),
        DeclareLaunchArgument("baud_rate", default_value="1000000"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        # Off by default. True loads the description with an effort command
        # interface and the servos in current-based position mode, which is
        # what lets gravity compensation reach a motor. The arm goes softer
        # as soon as it comes up, so it is opt-in.
        DeclareLaunchArgument("current_control",
                              default_value=current_control_default()),
        DeclareLaunchArgument("arm_operating_mode", default_value="5"),
        # A teaching/leader session supplies its own controller, which claims
        # the same arm command interfaces as arm_controller.  Keep the
        # conventional controller enabled for every normal hardware launch.
        DeclareLaunchArgument("start_arm_controller", default_value="true"),
        state_publisher,
        control_node,
        joint_state_spawner,
        start_motion_controllers,
        stop_launch_if_hardware_exits,
    ])
