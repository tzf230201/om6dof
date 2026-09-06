"""Run the production Dynamixel loop with a driver-enforced torque-off invariant."""

import shutil
import subprocess
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


HARDWARE_SERVICE = "om6dof-hardware.service"
SUPPORTED_UPDATE_RATES_HZ = (10, 20, 50, 100)


def _query_service_state(service_name=HARDWARE_SERVICE):
    """Return (LoadState, ActiveState); uncertainty is intentionally unsafe."""
    try:
        result = subprocess.run(
            [
                "systemctl",
                "show",
                service_name,
                "--property=LoadState",
                "--property=ActiveState",
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=3.0,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return "unavailable", "unknown", str(exc)

    fields = {}
    for line in result.stdout.splitlines():
        key, separator, value = line.partition("=")
        if separator:
            fields[key.strip()] = value.strip()
    detail = result.stderr.strip() or f"systemctl exit={result.returncode}"
    return (
        fields.get("LoadState", "unknown"),
        fields.get("ActiveState", "unknown"),
        detail,
    )


def _validate_rendered_description(description):
    """Require the real plugin and its torque-off contract in rendered XML."""
    try:
        root = ET.fromstring(description)
    except ET.ParseError as exc:
        raise RuntimeError(f"Diagnostic xacro produced invalid XML: {exc}") from exc

    systems = root.findall(".//ros2_control")
    if len(systems) != 1:
        raise RuntimeError(
            f"Expected exactly one ros2_control system, found {len(systems)}."
        )
    hardware = systems[0].find("hardware")
    plugin = None if hardware is None else hardware.findtext("plugin")
    if plugin is None or plugin.strip() != (
        "dynamixel_hardware_interface/DynamixelHardware"
    ):
        raise RuntimeError(
            f"Diagnostic xacro did not select the real Dynamixel plugin: {plugin!r}."
        )

    parameters = {
        element.get("name"): (element.text or "").strip()
        for element in hardware.findall("param")
    }
    diagnostic_value = parameters.get("torque_off_diagnostic_mode", "")
    if diagnostic_value.casefold() not in ("true", "1"):
        raise RuntimeError(
            "Rendered robot description does not enable torque_off_diagnostic_mode; "
            f"observed {diagnostic_value!r}."
        )
    init_value = parameters.get("disable_torque_at_init", "")
    if init_value.casefold() not in ("true", "1"):
        raise RuntimeError(
            "Rendered robot description does not request disable_torque_at_init; "
            f"observed {init_value!r}."
        )
    read_transport = parameters.get("read_transport_mode", "")
    if read_transport != "sequential_single_sync":
        raise RuntimeError(
            "Rendered robot description does not select the commissioned "
            "sequential_single_sync read transport; "
            f"observed {read_transport!r}."
        )


def _diagnostic_update_rate(context):
    """Return the whitelisted controller-manager rate or fail before port open."""
    raw_value = LaunchConfiguration("diagnostic_update_rate_hz").perform(context)
    try:
        update_rate = int(raw_value)
    except (TypeError, ValueError) as exc:
        raise RuntimeError(
            "diagnostic_update_rate_hz must be one of "
            f"{SUPPORTED_UPDATE_RATES_HZ}; observed {raw_value!r}."
        ) from exc
    if str(update_rate) != raw_value.strip() or update_rate not in SUPPORTED_UPDATE_RATES_HZ:
        raise RuntimeError(
            "diagnostic_update_rate_hz must be one of "
            f"{SUPPORTED_UPDATE_RATES_HZ}; observed {raw_value!r}."
        )
    return update_rate


def _render_diagnostic_description(context):
    """Render once, validate once, and return the exact XML given to both nodes."""
    executable = shutil.which("xacro")
    if not executable:
        raise RuntimeError("Cannot find xacro executable for diagnostic preflight.")
    package_share = FindPackageShare("om6dof_bringup").perform(context)
    xacro_file = str(PathJoinSubstitution(
        [package_share, "urdf", "om6dof.urdf.xacro"]
    ).perform(context))
    command = [
        executable,
        xacro_file,
        f"port_name:={LaunchConfiguration('port_name').perform(context)}",
        f"baud_rate:={LaunchConfiguration('baud_rate').perform(context)}",
        "use_fake_hardware:=false",
        "current_control:=false",
        "torque_off_diagnostic_mode:=true",
    ]
    try:
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=10.0,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise RuntimeError(f"Unable to render diagnostic xacro: {exc}") from exc
    if result.returncode != 0:
        raise RuntimeError(
            "Diagnostic xacro rendering failed before hardware open: "
            f"{result.stderr.strip()}"
        )
    _validate_rendered_description(result.stdout)
    return result.stdout


def _preflight(context):
    """Abort before opening U2D2 unless physical support and ownership are safe."""
    supported = LaunchConfiguration("arm_supported").perform(context)
    if supported.strip().lower() != "true":
        raise RuntimeError(
            "Refusing torque-off diagnostic: mechanically support the arm, keep "
            "emergency power removal reachable, then explicitly pass arm_supported:=true."
        )

    update_rate = _diagnostic_update_rate(context)

    load_state, active_state, detail = _query_service_state()
    if load_state != "loaded" or active_state != "inactive":
        raise RuntimeError(
            f"Refusing torque-off diagnostic: {HARDWARE_SERVICE} must be conclusively "
            f"loaded+inactive before U2D2 is opened; observed LoadState={load_state}, "
            f"ActiveState={active_state} ({detail})."
        )

    verified_description = _render_diagnostic_description(context)

    return [
        SetLaunchConfiguration(
            "verified_robot_description", verified_description
        ),
        LogInfo(
            msg=(
                "TORQUE-OFF DIAGNOSTIC MODE: arm support acknowledged; normal hardware "
                "service is inactive; only joint_state_broadcaster will be started; "
                f"controller update rate={update_rate} Hz"
            )
        )
    ]


def generate_launch_description():
    package_share = FindPackageShare("om6dof_bringup")

    # The preflight renders and validates this exact value before either node
    # starts.  It is not a launch argument an operator can override.
    robot_description = ParameterValue(
        LaunchConfiguration("verified_robot_description"),
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
        parameters=[
            PathJoinSubstitution([package_share, "config", "controllers.yaml"]),
            {
                # Applied after controllers.yaml, so only this diagnostic
                # process changes cadence; the production 100 Hz file remains
                # untouched.
                "update_rate": ParameterValue(
                    LaunchConfiguration("diagnostic_update_rate_hz"),
                    value_type=int,
                )
            },
        ],
        remappings=[("~/robot_description", "/robot_description")],
        output="screen",
    )
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    stop_launch_if_hardware_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=control_node,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="torque-off diagnostic hardware owner exited"
                    )
                )
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port_name",
                default_value=(
                    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_"
                    "FT5NUUIQ-if00-port0"
                ),
            ),
            DeclareLaunchArgument("baud_rate", default_value="1000000"),
            DeclareLaunchArgument(
                "diagnostic_update_rate_hz",
                default_value="100",
                description=(
                    "Torque-off controller cadence; intentionally restricted to "
                    "10, 20, 50, or 100 Hz."
                ),
            ),
            DeclareLaunchArgument(
                "arm_supported",
                default_value="false",
                description=(
                    "Operator assertion required before torque is removed; must be true."
                ),
            ),
            OpaqueFunction(function=_preflight),
            stop_launch_if_hardware_exits,
            state_publisher,
            control_node,
            joint_state_spawner,
        ]
    )
