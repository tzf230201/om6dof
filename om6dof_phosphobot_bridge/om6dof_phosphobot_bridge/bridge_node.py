"""Expose the OM6DOF arm to phosphobot over HTTP, without handing it the bus.

Why a bridge instead of a direct driver
---------------------------------------
phosphobot's manipulator drivers open the servo bus themselves and speak
Dynamixel. Doing that for the OM6DOF would mean fighting
``om6dof-hardware.service`` for ``/dev/ttyUSB0`` -- only one process may
hold it -- and discarding everything ``om6dof_controller`` provides: IK,
velocity ceilings, pose profiles, and the JOINT/CARTESIAN/CYLINDRICAL/
SEMI_CYLINDRICAL
modes.

So phosphobot talks HTTP to this node, and this node talks ROS. The
controller keeps sole ownership of the hardware and every guard stays in
force.

The position loop
-----------------
There is an impedance mismatch worth stating plainly: phosphobot writes
*absolute joint positions*, while ``om6dof_controller`` only accepts
*velocities* on ``/om6dof/control_cmd`` -- ``/om6dof/target_cmd`` exists
on the publisher side but nothing subscribes to it.

Rather than bypass the controller by publishing straight to
``/forward_position_controller/commands`` -- which would skip exactly the
safety we are trying to keep -- this node closes the loop itself: a
proportional controller drives measured position toward the requested
target and emits bounded velocities. Targets expire, so a dead client
stops the arm rather than leaving it running.
"""
from __future__ import annotations

import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String

DEFAULT_PORT = 8021
JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
GRIPPER_JOINT = "gripper_left_joint"

# Proportional gain turning position error into joint velocity, and the
# ceiling applied afterwards. The ceiling stays below the controller's own
# limits so this node is never the thing that saturates them.
POSITION_GAIN = 2.0
MAX_JOINT_VELOCITY = 0.25          # rad/s
POSITION_TOLERANCE = 0.005         # rad; inside this the joint is "there"
TARGET_TIMEOUT_S = 1.0             # stop if no fresh target arrives
CONTROL_PERIOD_S = 0.02            # 50 Hz
STATE_STALE_S = 0.5


class OM6DOFBridge(Node):
    """ROS side: watches joint state, drives the controller, serves state."""

    def __init__(self) -> None:
        super().__init__("om6dof_phosphobot_bridge")

        self.declare_parameter("http_port", DEFAULT_PORT)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("control_cmd_topic", "/om6dof/control_cmd")
        self.declare_parameter("operation_mode_topic", "/om6dof/operation_mode")
        self.declare_parameter(
            "operation_mode_state_topic", "/om6dof/operation_mode/state"
        )
        self.declare_parameter(
            "remote_enabled_state_topic", "/om6dof/remote_enabled/state"
        )
        self.declare_parameter("gripper_cmd_topic", "/om6dof_teleop/gripper_cmd")

        self._lock = threading.Lock()
        self._positions: Dict[str, float] = {}
        self._efforts: Dict[str, float] = {}
        self._joint_state_time = 0.0
        self._mode: str = ""
        self._remote_enabled: Optional[bool] = None

        self._target: Optional[List[float]] = None
        self._target_time = 0.0

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.pub_cmd = self.create_publisher(
            Float64MultiArray, self._p("control_cmd_topic"), 10
        )
        self.pub_mode = self.create_publisher(
            String, self._p("operation_mode_topic"), 10
        )
        self.pub_gripper = self.create_publisher(
            String, self._p("gripper_cmd_topic"), 10
        )

        self.create_subscription(
            JointState, self._p("joint_state_topic"), self._on_joint_state, 20
        )
        self.create_subscription(
            String, self._p("operation_mode_state_topic"), self._on_mode, latched
        )
        self.create_subscription(
            Bool, self._p("remote_enabled_state_topic"), self._on_remote, latched
        )

        self.create_timer(CONTROL_PERIOD_S, self._tick)
        self.get_logger().info(
            f"phosphobot bridge listening on port {self._p('http_port')}"
        )

    def _p(self, name: str):
        return self.get_parameter(name).value

    # -- ROS callbacks ----------------------------------------------------
    def _on_joint_state(self, msg: JointState) -> None:
        with self._lock:
            for index, name in enumerate(msg.name):
                if index < len(msg.position):
                    self._positions[name] = float(msg.position[index])
                if index < len(msg.effort):
                    self._efforts[name] = float(msg.effort[index])
            self._joint_state_time = time.monotonic()

    def _on_mode(self, msg: String) -> None:
        with self._lock:
            self._mode = str(msg.data).strip().upper()

    def _on_remote(self, msg: Bool) -> None:
        with self._lock:
            self._remote_enabled = bool(msg.data)

    # -- control loop -----------------------------------------------------
    def _tick(self) -> None:
        """Drive measured position toward the target, or command a stop."""
        with self._lock:
            target = self._target
            fresh = (time.monotonic() - self._target_time) < TARGET_TIMEOUT_S
            positions = [self._positions.get(n) for n in JOINT_NAMES]
            state_fresh = (
                time.monotonic() - self._joint_state_time
            ) < STATE_STALE_S

        if target is None or not fresh:
            # A client that stops asking must not leave the arm moving.
            if target is not None and not fresh:
                with self._lock:
                    self._target = None
                self.pub_cmd.publish(Float64MultiArray(data=[0.0] * 6))
            return

        if not state_fresh or any(p is None for p in positions):
            self.pub_cmd.publish(Float64MultiArray(data=[0.0] * 6))
            return

        velocities: List[float] = []
        for measured, wanted in zip(positions, target):
            error = wanted - float(measured)
            if abs(error) < POSITION_TOLERANCE:
                velocities.append(0.0)
                continue
            v = POSITION_GAIN * error
            velocities.append(max(-MAX_JOINT_VELOCITY, min(MAX_JOINT_VELOCITY, v)))
        self.pub_cmd.publish(Float64MultiArray(data=velocities))

    # -- API used by the HTTP layer ---------------------------------------
    def snapshot(self) -> dict:
        with self._lock:
            age = time.monotonic() - self._joint_state_time
            return {
                "joints": [self._positions.get(n) for n in JOINT_NAMES],
                "joint_names": JOINT_NAMES,
                "gripper": self._positions.get(GRIPPER_JOINT),
                "efforts": [self._efforts.get(n) for n in JOINT_NAMES],
                "mode": self._mode,
                "remote_enabled": self._remote_enabled,
                "joint_state_age_s": round(age, 3) if self._joint_state_time else None,
                "connected": bool(self._joint_state_time) and age < STATE_STALE_S,
            }

    def set_target(self, joints: List[float]) -> None:
        with self._lock:
            self._target = list(joints)
            self._target_time = time.monotonic()

    def clear_target(self) -> None:
        with self._lock:
            self._target = None
        self.pub_cmd.publish(Float64MultiArray(data=[0.0] * 6))

    def set_mode(self, mode: str) -> None:
        self.pub_mode.publish(String(data=mode.strip().upper()))

    def set_gripper(self, command: str) -> None:
        self.pub_gripper.publish(String(data=command.strip().lower()))


def make_handler(node: OM6DOFBridge):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args):  # keep the ROS log readable
            pass

        def _send(self, payload: dict, code: int = 200) -> None:
            body = json.dumps(payload).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _read_json(self) -> dict:
            length = int(self.headers.get("Content-Length", 0) or 0)
            if length <= 0 or length > 8192:
                return {}
            try:
                return json.loads(self.rfile.read(length) or b"{}")
            except (ValueError, UnicodeDecodeError):
                return {}

        def do_GET(self) -> None:
            if self.path.split("?")[0] in ("/state", "/"):
                return self._send(node.snapshot())
            self._send({"error": "not found"}, 404)

        def do_POST(self) -> None:
            path = self.path.split("?")[0]
            body = self._read_json()

            if path == "/positions":
                joints = body.get("joints")
                if (
                    not isinstance(joints, list)
                    or len(joints) != 6
                    or not all(isinstance(v, (int, float)) for v in joints)
                    or not all(math.isfinite(float(v)) for v in joints)
                ):
                    return self._send(
                        {"error": "joints must be 6 finite numbers (radians)"}, 400
                    )
                node.set_target([float(v) for v in joints])
                return self._send({"ok": True})

            if path == "/stop":
                node.clear_target()
                return self._send({"ok": True})

            if path == "/mode":
                mode = str(body.get("mode", "")).strip().upper()
                allowed = {
                    "JOINT", "CARTESIAN", "CYLINDRICAL", "SEMI_CYLINDRICAL",
                    "READY", "STARTUP", "AUTONOMOUS", "TOGGLE_REST_READY",
                }
                if mode not in allowed:
                    return self._send(
                        {"error": f"mode must be one of {sorted(allowed)}"}, 400
                    )
                node.set_mode(mode)
                return self._send({"ok": True, "mode": mode})

            if path == "/gripper":
                command = str(body.get("command", "")).strip().lower()
                if command not in ("open", "close"):
                    return self._send({"error": "command must be open or close"}, 400)
                node.set_gripper(command)
                return self._send({"ok": True, "command": command})

            self._send({"error": "not found"}, 404)

    return Handler


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OM6DOFBridge()
    port = int(node.get_parameter("http_port").value)

    # Bound to localhost: this endpoint commands a real arm and has no
    # authentication, so it must not be reachable from the network.
    server = ThreadingHTTPServer(("127.0.0.1", port), make_handler(node))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.clear_target()
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
