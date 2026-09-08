"""Selectable human-input adapter for canonical OM6DOF command topics.

This node has no kinematics, controller-manager client, or final joint-position
publisher. It converts Go2W, keyboard, Logitech F710, or Airbus TCA input into
``/om6dof/operation_mode``, ``/om6dof/control_cmd``, and
``/om6dof/gripper_cmd``. It never controls an
action server or hardware interface itself.
"""

from __future__ import annotations

import glob
import math
import os
import select
import struct
import sys
import termios
import threading
import time
import tty
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Float64MultiArray, String
from unitree_go.msg import LowState, WirelessController

from om6dof_controller.control_math import (
    MODE_AUTONOMOUS,
    MODE_CARTESIAN,
    MODE_CYLINDRICAL,
    MODE_JOINT,
    MODE_READY,
    MODE_REST,
    MODE_STARTUP,
)


BTN_R1 = 1 << 0
BTN_L1 = 1 << 1
BTN_START = 1 << 2
BTN_SELECT = 1 << 3
BTN_R2 = 1 << 4
BTN_L2 = 1 << 5
BTN_F1 = 1 << 6
BTN_F3 = 1 << 7
BTN_A = 1 << 8
BTN_B = 1 << 9
BTN_X = 1 << 10
BTN_Y = 1 << 11
BTN_UP = 1 << 12
BTN_RIGHT = 1 << 13
BTN_DOWN = 1 << 14
BTN_LEFT = 1 << 15


def _next_teleop_mode(mode: str) -> str:
    """Expose only the documented three velocity frames."""
    modes = (MODE_JOINT, MODE_CARTESIAN, MODE_CYLINDRICAL)
    try:
        return modes[(modes.index(mode) + 1) % len(modes)]
    except ValueError:
        return MODE_JOINT


# Linux joystick API constants. This deliberately uses only the standard
# library: the robot image does not need python-evdev just to read a USB pad.
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JSIOCGAXES = 0x80016A11
JSIOCGBUTTONS = 0x80016A12
TCA_NAME_HINTS = ("thrustmaster", "tca", "airbus")
GAMEPAD_NAME_HINTS = (
    "logitech", "logicool", "f710", "gamepad", "xbox", "x-box",
)


class _LinuxJoystick:
    """Small non-blocking /dev/input/js* reader selected by USB name."""

    def __init__(self, name_hints: Sequence[str]) -> None:
        self.name_hints = tuple(hint.lower() for hint in name_hints)
        self.fd: Optional[int] = None
        self.axes: List[int] = []
        self.buttons: List[bool] = []
        self.last_scan = 0.0

    def _close(self) -> None:
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
        self.fd = None
        self.axes = []
        self.buttons = []

    @staticmethod
    def _name(fd: int) -> str:
        try:
            import fcntl
            data = bytearray(128)
            fcntl.ioctl(fd, 0x80006A13 + (len(data) << 16), data)
            return bytes(data).split(b"\0", 1)[0].decode("utf-8", "replace")
        except (ImportError, OSError):
            return ""

    @staticmethod
    def _count(fd: int, request: int) -> int:
        try:
            import fcntl
            data = bytearray(1)
            fcntl.ioctl(fd, request, data)
            return int(data[0])
        except (ImportError, OSError):
            return 0

    def _scan(self) -> None:
        self.last_scan = time.monotonic()
        self._close()
        for path in sorted(glob.glob("/dev/input/js*")):
            try:
                fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
            except OSError:
                continue
            name = self._name(fd).lower()
            if any(hint in name for hint in self.name_hints):
                self.fd = fd
                self.axes = [0] * self._count(fd, JSIOCGAXES)
                self.buttons = [False] * self._count(fd, JSIOCGBUTTONS)
                return
            os.close(fd)

    def snapshot(self) -> Tuple[bool, List[float], set[int]]:
        if self.fd is None or time.monotonic() - self.last_scan > 3.0:
            self._scan()
        if self.fd is not None:
            while True:
                try:
                    packet = os.read(self.fd, 8)
                except BlockingIOError:
                    break
                except OSError:
                    self._close()
                    break
                if len(packet) != 8:
                    break
                _when, value, kind, index = struct.unpack("IhBB", packet)
                kind &= ~JS_EVENT_INIT
                if kind == JS_EVENT_AXIS and index < len(self.axes):
                    self.axes[index] = value
                elif kind == JS_EVENT_BUTTON and index < len(self.buttons):
                    self.buttons[index] = bool(value)
        return (
            self.fd is not None,
            [value / 32767.0 for value in self.axes],
            {index for index, down in enumerate(self.buttons) if down},
        )


def _decode_lowstate_remote(
    wireless_remote: Sequence[int],
) -> Tuple[int, Tuple[float, float, float, float]]:
    """Decode Unitree's 40-byte controller payload as keys and lx/ly/rx/ry."""
    raw = bytes(wireless_remote)
    if len(raw) != 40:
        raise ValueError("wireless_remote must contain exactly 40 bytes")
    keys = int(raw[2]) | (int(raw[3]) << 8)
    axes = (
        struct.unpack_from("<f", raw, 4)[0],
        struct.unpack_from("<f", raw, 20)[0],
        struct.unpack_from("<f", raw, 8)[0],
        struct.unpack_from("<f", raw, 12)[0],
    )
    if not all(math.isfinite(value) for value in axes):
        raise ValueError("wireless_remote contains a non-finite joystick axis")
    return keys, axes


class TeleopNode(Node):
    """Translate one selected human-input source into canonical arm topics."""

    def __init__(self) -> None:
        super().__init__("om6dof_teleop")

        self.declare_parameter("lowstate_topic", "/lowstate")
        self.declare_parameter(
            "wirelesscontroller_topic", "/wirelesscontroller"
        )
        self.declare_parameter(
            "operation_mode_topic", "/om6dof/operation_mode"
        )
        self.declare_parameter("control_cmd_topic", "/om6dof/control_cmd")
        self.declare_parameter(
            "operation_mode_state_topic", "/om6dof/operation_mode/state"
        )
        self.declare_parameter(
            "remote_enabled_state_topic", "/om6dof/remote_enabled/state"
        )
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("input_source", "go2w")
        self.declare_parameter("keyboard_pulse_seconds", 0.15)
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("speed_scale_min", 0.15)
        self.declare_parameter("speed_scale_max", 5.0)
        self.declare_parameter("speed_scale_step", 0.10)
        self.declare_parameter("speed_repeat_seconds", 0.20)
        self.declare_parameter("remote_command_timeout_seconds", 0.5)
        self.declare_parameter("lowstate_preference_timeout_seconds", 0.1)
        self.declare_parameter("remote_action_debounce_seconds", 0.3)
        self.declare_parameter("mode_request_timeout_seconds", 2.0)
        self.declare_parameter("joint_axis_deadzone", 0.08)
        self.declare_parameter("joint_velocity", 0.5)
        self.declare_parameter("cartesian_linear_speed", 0.05)
        self.declare_parameter("cartesian_angular_speed", 0.5)
        self.declare_parameter("cylindrical_theta_speed", 0.25)
        self.declare_parameter(
            "button_pairs", [15, 13, 14, 12, 8, 11, 10, 9, -1, -1, 0, 4]
        )
        self.declare_parameter("joint_axes", [-1, -1, -1, -1, 3, -1])
        self.declare_parameter("joint_signs", [1.0] * 6)

        self.declare_parameter("gripper_command_topic", "/om6dof/gripper_cmd")

        self.rate = float(self.get_parameter("publish_rate_hz").value)
        self.input_source = str(
            self.get_parameter("input_source").value
        ).lower()
        self.keyboard_pulse_seconds = float(
            self.get_parameter("keyboard_pulse_seconds").value
        )
        self.speed_scale = float(self.get_parameter("speed_scale").value)
        self.speed_scale_min = float(
            self.get_parameter("speed_scale_min").value
        )
        self.speed_scale_max = float(
            self.get_parameter("speed_scale_max").value
        )
        self.speed_scale_step = float(
            self.get_parameter("speed_scale_step").value
        )
        self.speed_repeat_seconds = float(
            self.get_parameter("speed_repeat_seconds").value
        )
        self.remote_timeout = float(
            self.get_parameter("remote_command_timeout_seconds").value
        )
        self.lowstate_preference_timeout = float(
            self.get_parameter("lowstate_preference_timeout_seconds").value
        )
        self.action_debounce = float(
            self.get_parameter("remote_action_debounce_seconds").value
        )
        self.mode_request_timeout = float(
            self.get_parameter("mode_request_timeout_seconds").value
        )
        self.deadzone = float(self.get_parameter("joint_axis_deadzone").value)
        self.joint_velocity = float(self.get_parameter("joint_velocity").value)
        self.cartesian_linear_speed = float(
            self.get_parameter("cartesian_linear_speed").value
        )
        self.cartesian_angular_speed = float(
            self.get_parameter("cartesian_angular_speed").value
        )
        self.cylindrical_theta_speed = float(
            self.get_parameter("cylindrical_theta_speed").value
        )
        scalar_values = (
            self.rate,
            self.remote_timeout,
            self.lowstate_preference_timeout,
            self.action_debounce,
            self.mode_request_timeout,
            self.deadzone,
            self.joint_velocity,
            self.cartesian_linear_speed,
            self.cartesian_angular_speed,
            self.cylindrical_theta_speed,
            self.keyboard_pulse_seconds,
            self.speed_scale,
            self.speed_scale_min,
            self.speed_scale_max,
            self.speed_scale_step,
            self.speed_repeat_seconds,
        )
        if (
            not all(math.isfinite(value) for value in scalar_values)
            or self.rate <= 0.0
            or self.input_source not in (
                "go2w", "keyboard", "gamepad", "airbus"
            )
            or self.keyboard_pulse_seconds <= 0.0
            or self.speed_scale_min <= 0.0
            or self.speed_scale_max < self.speed_scale_min
            or not self.speed_scale_min <= self.speed_scale <= self.speed_scale_max
            or self.speed_scale_step <= 0.0
            or self.speed_repeat_seconds <= 0.0
            or self.remote_timeout <= 0.0
            or self.lowstate_preference_timeout < 0.0
            or self.action_debounce < 0.0
            or self.mode_request_timeout <= 0.0
            or not 0.0 <= self.deadzone < 1.0
            or self.joint_velocity < 0.0
            or self.cartesian_linear_speed < 0.0
            or self.cartesian_angular_speed < 0.0
            or self.cylindrical_theta_speed < 0.0
        ):
            raise RuntimeError("invalid remote mapping parameters")
        self.dt = 1.0 / self.rate

        packed_pairs = [int(value) for value in self.get_parameter(
            "button_pairs"
        ).value]
        if len(packed_pairs) != 12:
            raise RuntimeError("button_pairs must contain 12 integers")
        self.button_pairs = [
            (packed_pairs[2 * index], packed_pairs[2 * index + 1])
            for index in range(6)
        ]
        self.joint_axes = [
            int(value) for value in self.get_parameter("joint_axes").value
        ]
        self.joint_signs = [
            float(value) for value in self.get_parameter("joint_signs").value
        ]
        if len(self.joint_axes) != 6 or len(self.joint_signs) != 6:
            raise RuntimeError(
                "joint_axes and joint_signs must have six values"
            )

        self.lock = threading.RLock()
        self.remote_enabled = False
        self.control_mode = MODE_JOINT
        self.f1_destination: Optional[str] = None
        self.mode_request_pending_until = 0.0
        self.remote_waiting_for_neutral = True
        self.keys = 0
        self.axes = (0.0, 0.0, 0.0, 0.0)
        self.lowstate_keys = 0
        self.lowstate_axes = (0.0, 0.0, 0.0, 0.0)
        self.event_keys = 0
        self.event_axes = (0.0, 0.0, 0.0, 0.0)
        self.last_lowstate_remote = 0.0
        self.last_event_remote = 0.0
        self.last_remote_action: Dict[int, float] = {}
        self.keyboard_command = [0.0] * 6
        self.keyboard_command_until = 0.0
        self.keyboard_thread: Optional[threading.Thread] = None
        self.keyboard_running = False
        self.gamepad = _LinuxJoystick(GAMEPAD_NAME_HINTS)
        self.airbus = _LinuxJoystick(TCA_NAME_HINTS)
        self.stick_buttons: set[int] = set()
        self.stick_primed = False
        self.stick_speed_direction = 0
        self.stick_speed_next_at = 0.0

        event_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        lowstate_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        mode_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        command_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.operation_pub = self.create_publisher(
            String,
            str(self.get_parameter("operation_mode_topic").value),
            mode_qos,
        )
        self.control_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("control_cmd_topic").value),
            command_qos,
        )
        self.gripper_pub = self.create_publisher(
            String,
            str(self.get_parameter("gripper_command_topic").value),
            10,
        )
        self.create_subscription(
            LowState,
            str(self.get_parameter("lowstate_topic").value),
            self._on_lowstate,
            lowstate_qos,
        )
        self.create_subscription(
            WirelessController,
            str(self.get_parameter("wirelesscontroller_topic").value),
            self._on_wireless_event,
            event_qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("remote_enabled_state_topic").value),
            self._on_remote_state,
            state_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("operation_mode_state_topic").value),
            self._on_operation_state,
            state_qos,
        )

        self.create_timer(self.dt, self._tick)
        self.parameter_callback = self.add_on_set_parameters_callback(
            self._on_set_parameters
        )
        if self.input_source == "keyboard":
            self._start_keyboard_reader()
        self.get_logger().info(
            f"teleop adapter ready: source={self.input_source} -> "
            "/om6dof/operation_mode + /om6dof/control_cmd; "
            "kinematics live in om6dof_controller"
        )

    def _deadzone(self, value: float) -> float:
        if abs(value) < self.deadzone:
            return 0.0
        sign = 1.0 if value > 0.0 else -1.0
        return sign * min(
            1.0, (abs(value) - self.deadzone) / (1.0 - self.deadzone)
        )

    def _remote_joint_velocity(
        self, keys: int, axes: Sequence[float]
    ) -> List[float]:
        result = [0.0] * 6
        for index in range(6):
            sign = self.joint_signs[index]
            axis_index = self.joint_axes[index]
            if 0 <= axis_index < len(axes):
                result[index] = (
                    self.joint_velocity
                    * sign
                    * self._deadzone(float(axes[axis_index]))
                )
                continue
            decrease, increase = self.button_pairs[index]
            if 0 <= increase < 16 and keys & (1 << increase):
                result[index] += self.joint_velocity * sign
            if 0 <= decrease < 16 and keys & (1 << decrease):
                result[index] -= self.joint_velocity * sign
        return result

    def _remote_coordinate_velocity(
        self, keys: int, axes: Sequence[float]
    ) -> List[float]:
        linear = self.cartesian_linear_speed
        angular = self.cartesian_angular_speed
        first = linear if keys & BTN_Y else 0.0
        if keys & BTN_A:
            first -= linear
        second_speed = (
            self.cylindrical_theta_speed
            if self.control_mode == MODE_CYLINDRICAL
            else linear
        )
        second = second_speed if keys & BTN_LEFT else 0.0
        if keys & BTN_RIGHT:
            second -= second_speed
        vertical = linear if keys & BTN_UP else 0.0
        if keys & BTN_DOWN:
            vertical -= linear
        roll = angular if keys & BTN_X else 0.0
        if keys & BTN_B:
            roll -= angular
        pitch = angular * (
            self._deadzone(float(axes[3])) if len(axes) > 3 else 0.0
        )
        yaw = angular if keys & BTN_R1 else 0.0
        if keys & BTN_R2:
            yaw -= angular
        return [first, second, vertical, roll, pitch, yaw]

    def _remote_motion_active(
        self, keys: int, axes: Sequence[float]
    ) -> bool:
        motion_buttons = (
            BTN_R1
            | BTN_R2
            | BTN_X
            | BTN_B
            | BTN_Y
            | BTN_A
            | BTN_UP
            | BTN_DOWN
            | BTN_LEFT
            | BTN_RIGHT
        )
        return bool(keys & motion_buttons) or (
            len(axes) > 3
            and abs(self._deadzone(float(axes[3]))) > 0.0
        )

    def _selected_remote_state_locked(self, now: float):
        lowstate_age = (
            now - self.last_lowstate_remote
            if self.last_lowstate_remote else math.inf
        )
        event_age = (
            now - self.last_event_remote
            if self.last_event_remote else math.inf
        )
        if lowstate_age <= self.lowstate_preference_timeout:
            return (
                self.lowstate_keys, self.lowstate_axes,
                self.last_lowstate_remote,
            )
        if event_age <= self.remote_timeout:
            return self.event_keys, self.event_axes, self.last_event_remote
        return 0, (0.0, 0.0, 0.0, 0.0), 0.0

    def _on_lowstate(self, msg: LowState) -> None:
        if self.input_source != "go2w":
            return
        try:
            keys, axes = _decode_lowstate_remote(msg.wireless_remote)
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(
                f"invalid /lowstate wireless_remote payload: {exc}",
                throttle_duration_sec=2.0,
            )
            return
        self._process_remote_sample(keys, axes, "lowstate")

    def _on_wireless_event(self, msg: WirelessController) -> None:
        if self.input_source != "go2w":
            return
        self._process_remote_sample(
            int(msg.keys),
            (float(msg.lx), float(msg.ly), float(msg.rx), float(msg.ry)),
            "event",
        )

    def _process_remote_sample(
        self, keys: int, axes: Sequence[float], source: str
    ) -> None:
        operation_request = None
        gripper = None
        with self.lock:
            now = time.monotonic()
            keys = int(keys) & 0xFFFF
            axes = tuple(float(value) for value in axes)
            if len(axes) != 4 or not all(
                math.isfinite(value) for value in axes
            ):
                self.get_logger().warn(f"invalid {source} remote axes ignored")
                return

            if source == "lowstate":
                previous_source_keys = self.lowstate_keys
                self.lowstate_keys = keys
                self.lowstate_axes = axes
                self.last_lowstate_remote = now
            elif source == "event":
                stale = (
                    not self.last_event_remote
                    or now - self.last_event_remote > self.remote_timeout
                )
                previous_source_keys = 0 if stale else self.event_keys
                self.event_keys = keys
                self.event_axes = axes
                self.last_event_remote = now
            else:
                raise ValueError(f"unknown remote source: {source}")

            self.keys, self.axes, _ = self._selected_remote_state_locked(now)
            rising = (~previous_source_keys) & keys
            if (
                source == "event"
                and self.last_lowstate_remote
                and now - self.last_lowstate_remote
                <= self.lowstate_preference_timeout
            ):
                rising &= ~self.lowstate_keys
            elif (
                source == "lowstate"
                and self.last_event_remote
                and now - self.last_event_remote <= self.remote_timeout
            ):
                rising &= ~self.event_keys

            def accepted_rising(button: int) -> bool:
                if not rising & button:
                    return False
                previous = float(self.last_remote_action.get(button, 0.0))
                if previous and now - previous < self.action_debounce:
                    return False
                self.last_remote_action[button] = now
                return True

            ownership_rising = accepted_rising(BTN_F3)
            if ownership_rising:
                if now < self.mode_request_pending_until:
                    self.get_logger().warn(
                        "F3 ignored: previous ownership request is pending"
                    )
                else:
                    operation_request = (
                        MODE_AUTONOMOUS if self.remote_enabled else MODE_JOINT
                    )
                    self.mode_request_pending_until = (
                        now + self.mode_request_timeout
                    )
                    if operation_request == MODE_JOINT:
                        self.remote_waiting_for_neutral = True

            elif accepted_rising(BTN_SELECT):
                if self.remote_enabled:
                    self.control_mode = _next_teleop_mode(self.control_mode)
                    operation_request = self.control_mode
                    self.remote_waiting_for_neutral = True
                else:
                    self.get_logger().warn(
                        "Select ignored: enable remote control with F3 first"
                    )

            elif accepted_rising(BTN_F1):
                if not self.remote_enabled:
                    self.get_logger().warn(
                        "F1 ignored: enable remote control with F3 first"
                    )
                else:
                    operation_request = (
                        MODE_STARTUP
                        if self.f1_destination == MODE_READY
                        else MODE_READY
                    )
                    self.control_mode = MODE_JOINT
                    self.remote_waiting_for_neutral = True

            if self.remote_enabled and accepted_rising(BTN_L1):
                gripper = "open"
            if self.remote_enabled and accepted_rising(BTN_L2):
                gripper = "close"

            if self.remote_enabled and self.remote_waiting_for_neutral:
                if not self._remote_motion_active(self.keys, self.axes):
                    self.remote_waiting_for_neutral = False
                    self.get_logger().info(
                        "remote motion armed after neutral input"
                    )

        if operation_request is not None:
            self.operation_pub.publish(String(data=operation_request))
            self.get_logger().info(
                f"remote operation request -> {operation_request}"
            )
        if gripper is not None:
            self._publish_gripper(gripper)

    def _on_remote_state(self, msg: Bool) -> None:
        with self.lock:
            previous = self.remote_enabled
            self.remote_enabled = bool(msg.data)
            self.mode_request_pending_until = 0.0
            if self.remote_enabled and not previous:
                self.control_mode = MODE_JOINT
                self.f1_destination = MODE_READY
                self.remote_waiting_for_neutral = True

    def _on_operation_state(self, msg: String) -> None:
        mode = msg.data.strip().upper()
        with self.lock:
            if mode == MODE_AUTONOMOUS:
                self.remote_enabled = False
                self.mode_request_pending_until = 0.0
            elif mode in (MODE_JOINT, MODE_CARTESIAN, MODE_CYLINDRICAL):
                changed = mode != self.control_mode
                self.control_mode = mode
                if changed:
                    self.remote_waiting_for_neutral = True
            elif mode == MODE_READY:
                self.control_mode = MODE_JOINT
                self.f1_destination = MODE_READY
                self.remote_waiting_for_neutral = True
            elif mode == MODE_STARTUP:
                self.control_mode = MODE_JOINT
                self.f1_destination = MODE_STARTUP
                self.remote_waiting_for_neutral = True

    def _publish_gripper(self, command: str) -> None:
        if command not in ("open", "close"):
            self.get_logger().warn(f"gripper command '{command}' rejected")
            return
        self.gripper_pub.publish(String(data=command))

    def _request_ownership_locked(self) -> Optional[str]:
        """Request the opposite owner; state feedback remains authoritative."""
        now = time.monotonic()
        if now < self.mode_request_pending_until:
            return None
        requested = MODE_AUTONOMOUS if self.remote_enabled else MODE_JOINT
        self.mode_request_pending_until = now + self.mode_request_timeout
        if requested == MODE_JOINT:
            self.remote_waiting_for_neutral = True
        return requested

    def _adjust_speed_locked(self, direction: int) -> None:
        self.speed_scale = max(
            self.speed_scale_min,
            min(
                self.speed_scale_max,
                self.speed_scale + direction * self.speed_scale_step,
            ),
        )
        self.get_logger().info(f"teleop speed: {self.speed_scale:.0%}")

    def _start_keyboard_reader(self) -> None:
        if self.keyboard_thread is not None and self.keyboard_thread.is_alive():
            if not self.keyboard_running:
                self.keyboard_thread.join(timeout=0.2)
            if self.keyboard_thread.is_alive():
                return
        self.keyboard_running = True
        self.keyboard_thread = threading.Thread(
            target=self._keyboard_loop, name="om6dof-keyboard", daemon=True
        )
        self.keyboard_thread.start()

    def _on_set_parameters(self, parameters) -> SetParametersResult:
        """Hot-switch input only after discarding all state from its predecessor."""
        requested = self.input_source
        for parameter in parameters:
            if parameter.name == "input_source":
                requested = str(parameter.value).lower()
        if requested not in ("go2w", "keyboard", "gamepad", "airbus"):
            return SetParametersResult(
                successful=False,
                reason="input_source must be go2w, keyboard, gamepad, or airbus",
            )
        if requested == self.input_source:
            return SetParametersResult(successful=True)

        old_source = self.input_source
        with self.lock:
            self.input_source = requested
            self.keys = 0
            self.axes = (0.0, 0.0, 0.0, 0.0)
            self.lowstate_keys = 0
            self.lowstate_axes = (0.0, 0.0, 0.0, 0.0)
            self.event_keys = 0
            self.event_axes = (0.0, 0.0, 0.0, 0.0)
            self.last_lowstate_remote = 0.0
            self.last_event_remote = 0.0
            self.keyboard_command = [0.0] * 6
            self.keyboard_command_until = 0.0
            self.stick_buttons = set()
            self.stick_primed = False
            self.stick_speed_direction = 0
            self.stick_speed_next_at = 0.0
            self.remote_waiting_for_neutral = True

        if old_source == "keyboard":
            self.keyboard_running = False
        if requested == "keyboard":
            self._start_keyboard_reader()
        self.get_logger().info(
            f"teleop input source changed: {old_source} -> {requested}; "
            "waiting for neutral input"
        )
        return SetParametersResult(successful=True)

    def _keyboard_loop(self) -> None:
        """ROBOTIS-style terminal reader adapted to velocity rather than trajectory."""
        if not sys.stdin.isatty():
            self.get_logger().error(
                "keyboard source requires an interactive terminal"
            )
            return
        fd = sys.stdin.fileno()
        try:
            old = termios.tcgetattr(fd)
        except termios.error:
            self.get_logger().error(
                "could not configure terminal for keyboard teleop"
            )
            return
        self.get_logger().info(
            "keyboard: g=enable/disable, m=mode, r=READY/STARTUP, "
            "1/q..6/y=joint +/-; Cartesian: w/s x, a/d y, r/f z, "
            "u/o roll, i/k pitch, j/l yaw, +/- speed, [/]=gripper, Esc=quit"
        )
        try:
            tty.setcbreak(fd)
            while self.keyboard_running and rclpy.ok():
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if ready:
                    self._handle_keyboard_key(sys.stdin.read(1))
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def _handle_keyboard_key(self, key: str) -> None:
        operation = None
        gripper = None
        with self.lock:
            if key == "\x1b":
                self.keyboard_running = False
                return
            if key == "g":
                operation = self._request_ownership_locked()
            elif key in ("+", "="):
                self._adjust_speed_locked(1)
            elif key == "-":
                self._adjust_speed_locked(-1)
            elif key == "m" and self.remote_enabled:
                self.control_mode = _next_teleop_mode(self.control_mode)
                self.remote_waiting_for_neutral = False
                operation = self.control_mode
            elif key == "r" and self.remote_enabled:
                operation = (
                    MODE_STARTUP if self.f1_destination == MODE_READY
                    else MODE_READY
                )
                self.control_mode = MODE_JOINT
                self.remote_waiting_for_neutral = True
            elif key == "[":
                gripper = "open"
            elif key == "]":
                gripper = "close"
            elif self.remote_enabled:
                vector = [0.0] * 6
                if self.control_mode == MODE_JOINT:
                    pairs = (("1", "q"), ("2", "w"), ("3", "e"),
                             ("4", "r"), ("5", "t"), ("6", "y"))
                    for index, (positive, negative) in enumerate(pairs):
                        if key == positive:
                            vector[index] = self.joint_velocity
                        elif key == negative:
                            vector[index] = -self.joint_velocity
                else:
                    keys = {"w": (0, 1), "s": (0, -1), "a": (1, 1),
                            "d": (1, -1), "r": (2, 1), "f": (2, -1),
                            "u": (3, 1), "o": (3, -1), "i": (4, 1),
                            "k": (4, -1), "j": (5, 1), "l": (5, -1)}
                    if key in keys:
                        index, sign = keys[key]
                        speed = (self.cylindrical_theta_speed if index == 1
                                 and self.control_mode == MODE_CYLINDRICAL
                                 else self.cartesian_linear_speed if index < 3
                                 else self.cartesian_angular_speed)
                        vector[index] = sign * speed
                if any(vector):
                    self.keyboard_command = vector
                    self.keyboard_command_until = (
                        time.monotonic() + self.keyboard_pulse_seconds
                    )
                    self.remote_waiting_for_neutral = False
        if operation:
            self.operation_pub.publish(String(data=operation))
        if gripper and self.remote_enabled:
            self._publish_gripper(gripper)

    def _stick_velocity_locked(
        self,
    ) -> Tuple[List[float], Optional[str], Optional[str]]:
        """Return selected USB velocity, mode request, and gripper command."""
        stick = self.gamepad if self.input_source == "gamepad" else self.airbus
        connected, axes, buttons = stick.snapshot()
        axes.extend([0.0] * max(0, 8 - len(axes)))
        if not self.stick_primed:
            # Treat everything held at connection/switch time as the baseline,
            # never as an edge-triggered ownership or gripper command.
            self.stick_buttons = buttons if connected else set()
            self.stick_primed = connected
            return [0.0] * 6, None, None
        new = buttons - self.stick_buttons
        self.stick_buttons = buttons if connected else set()
        if self.input_source == "gamepad":
            dpad = -1 if axes[7] <= -0.5 else (1 if axes[7] >= 0.5 else 0)
            now = time.monotonic()
            if dpad and (
                dpad != self.stick_speed_direction
                or now >= self.stick_speed_next_at
            ):
                self._adjust_speed_locked(1 if dpad < 0 else -1)
                self.stick_speed_next_at = now + self.speed_repeat_seconds
            self.stick_speed_direction = dpad
        else:
            # Airbus TCA's lift lever is a continuous speed throttle.
            lever = max(-1.0, min(1.0, axes[3]))
            self.speed_scale = (
                self.speed_scale_max if lever <= -1.0 else
                self.speed_scale_min if lever >= 1.0 else
                self.speed_scale_min + (1.0 - lever) *
                (self.speed_scale_max - self.speed_scale_min) / 2.0
            )
        operation = None
        gripper = None
        toggle_button = 7
        cycle_button = 8
        back_button = 6
        if back_button in new and self.remote_enabled:
            # Safe one-way exit: controller performs REST fully before
            # releasing ownership to MoveIt/autonomous control.
            self.remote_waiting_for_neutral = True
            operation = MODE_REST
        elif toggle_button in new:
            operation = self._request_ownership_locked()
        elif cycle_button in new and self.remote_enabled:
            self.control_mode = _next_teleop_mode(self.control_mode)
            self.remote_waiting_for_neutral = True
            operation = self.control_mode
        if self.input_source == "gamepad":
            if 5 in new:
                gripper = "close"
            elif 4 in new:
                gripper = "open"
            # F710 is spatial: protect against a misleading JOINT mapping.
            if self.control_mode not in (MODE_CARTESIAN, MODE_CYLINDRICAL):
                return [0.0] * 6, operation, gripper

            def trigger(index: int) -> float:
                return max(0.0, min(1.0, (axes[index] + 1.0) / 2.0))

            second = 0.20 if self.control_mode == MODE_CYLINDRICAL else 0.03
            pitch = float((1 if 0 in buttons else 0) -
                          (1 if 3 in buttons else 0))
            values = [
                -self._deadzone(axes[1]) * 0.03,
                -self._deadzone(axes[0]) * second,
                -self._deadzone(axes[4]) * 0.03,
                self._deadzone(axes[3]) * 0.35,
                pitch * 0.35,
                (trigger(5) - trigger(2)) * 0.35,
            ]
        else:
            if 3 in new:
                gripper = "close"
            elif 2 in new:
                gripper = "open"
            if self.control_mode not in (MODE_CARTESIAN, MODE_CYLINDRICAL):
                return [0.0] * 6, operation, gripper
            z = float((1 if 1 in buttons else 0) -
                      (1 if 0 in buttons else 0))
            second = 0.20 if self.control_mode == MODE_CYLINDRICAL else 0.03
            values = [
                -self._deadzone(axes[5]) * 0.03,
                -self._deadzone(axes[4]) * second,
                z * 0.03,
                self._deadzone(axes[2]) * 0.35,
                self._deadzone(axes[1]) * 0.35,
                self._deadzone(axes[0]) * 0.35,
            ]
        return values, operation, gripper

    def _tick(self) -> None:
        operation = None
        gripper = None
        with self.lock:
            now = time.monotonic()
            if self.input_source in ("gamepad", "airbus"):
                velocity, operation, gripper = self._stick_velocity_locked()
                if not self.remote_enabled:
                    velocity = [0.0] * 6
                elif self.remote_waiting_for_neutral:
                    if not any(abs(value) > 0.0 for value in velocity):
                        self.remote_waiting_for_neutral = False
                    velocity = [0.0] * 6
            elif not self.remote_enabled:
                return
            elif self.input_source == "keyboard":
                velocity = (
                    self.keyboard_command
                    if now <= self.keyboard_command_until else [0.0] * 6
                )
            else:
                keys, axes, stamp = self._selected_remote_state_locked(now)
                fresh = bool(
                    not self.remote_waiting_for_neutral and stamp
                    and now - stamp <= max(
                        self.remote_timeout, self.lowstate_preference_timeout
                    )
                )
                if not fresh:
                    velocity = [0.0] * 6
                elif self.control_mode == MODE_JOINT:
                    velocity = self._remote_joint_velocity(keys, axes)
                else:
                    velocity = self._remote_coordinate_velocity(keys, axes)
        self.control_pub.publish(
            Float64MultiArray(data=[value * self.speed_scale for value in velocity])
        )
        if operation:
            self.operation_pub.publish(String(data=operation))
        if gripper and self.remote_enabled:
            self._publish_gripper(gripper)

    def destroy_node(self):
        self.keyboard_running = False
        if (self.keyboard_thread is not None
                and self.keyboard_thread.is_alive()):
            self.keyboard_thread.join(timeout=0.3)
        self.gamepad._close()
        self.airbus._close()
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
