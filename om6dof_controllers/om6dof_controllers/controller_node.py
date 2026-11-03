"""OM6DOF operator channel: converts human jog input into an additive offset.

Two command channels reach every arm joint, and the hardware adds them before
the motor write (see DynamixelHardware::write):

  * ``position``        - autonomous motion. Owned by ``arm_controller``
    (MoveIt, graph_pick, and the absolute poses this node requests as
    FollowJointTrajectory goals).
  * ``position_offset`` - the human operator. Owned by this node alone.

This node therefore **never publishes an absolute joint position**. It only
ever writes the offset channel, so teleop has a dedicated path that cannot
collide with, duplicate, or preempt the autonomous one. Both controllers stay
active permanently; there is no ownership, no controller switching, and no
arbitration anywhere in the stack.

Absolute motions this node still offers (READY, STARTUP, REST, and absolute
web targets) are sent to ``arm_controller`` as trajectory goals, because they
belong on the autonomous channel by definition. The trajectory controller does
the time profiling, so this node carries no position servo of its own.
"""

from __future__ import annotations

import json
import math
import threading
import time
from typing import List, Optional, Sequence

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory, GripperCommand
from control_msgs.msg import JointTolerance, JointTrajectoryControllerState
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .control_math import (
    COORDINATE_MODES,
    MODE_AUTONOMOUS,
    MODE_CARTESIAN,
    MODE_CYLINDRICAL,
    MODE_FLOAT,
    MODE_FORCE,
    MODE_JOINT,
    MODE_READY,
    MODE_REST,
    MODE_SEMI_CYLINDRICAL,
    MODE_STARTUP,
    MOTION_MODES,
    clamp_positions,
    cartesian_rpy_to_tip_rotation,
    tip_rotation_to_cartesian_rpy,
    limit_norm,
    normalize_operation_mode,
    rotation_from_zyx,
    rotation_to_zyx,
    step_toward,
    validated_control_command,
    validated_joint_positions,
)
from .target_planner import (
    plan_pose, path_is_clear, pose_error, POSITION_TOLERANCE,
    ORIENTATION_TOLERANCE, JOINT_TOLERANCE,
)
from .admittance_target import AdmittanceTargetMixin


DEFAULT_READY_JOINT_POSITIONS_DEG = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
DEFAULT_TRANSITION_JOINT_POSITIONS_DEG = (0.0, 0.0, -90.0, 0.0, 0.0, 0.0)
DEFAULT_JOINT_LOWER = (
    -math.pi * 0.9,
    -math.pi * 0.65,
    -math.pi * 0.60,
    -math.pi * 0.9,
    -math.pi * 0.63,
    -math.pi * 0.9,
)
DEFAULT_JOINT_UPPER = (
    math.pi * 0.9,
    math.pi * 0.67,
    math.pi * 0.68,
    math.pi * 0.9,
    math.pi * 0.67,
    math.pi * 0.9,
)


class OM6DOFController(AdmittanceTargetMixin, Node):
    """Turn canonical six-axis jog commands into a bounded joint offset."""

    # Class-level defaults so the attributes exist on every instance,
    # including the bare nodes the tests construct without __init__.
    last_coordinate_mode = MODE_CARTESIAN
    semi_pitch_nudge = 0.0
    semi_yaw_nudge = 0.0
    _target_generation = 0
    _target_send_pending = False
    _target_goal_handle = None
    _target_stop_requested = False
    _target_cancel_pending = False
    _target_result_pending = False
    _target_action_succeeded = False
    _ready_generation = 0
    _ready_pending = False
    _ready_succeeded = False
    _custom_z_target = None

    def __init__(self) -> None:
        super().__init__("om6dof_controller")

        self.declare_parameter(
            "joint_names", [f"joint{index}" for index in range(1, 7)]
        )
        self.declare_parameter("arm_controller", "arm_controller")
        # The operator channel. Claims joint*/position_offset, which the
        # hardware adds to joint*/position; it is never an absolute pose.
        self.declare_parameter(
            "offset_command_topic", "/forward_offset_controller/commands"
        )
        self.declare_parameter(
            "arm_action", "/arm_controller/follow_joint_trajectory"
        )
        self.declare_parameter(
            "arm_state_topic", "/arm_controller/controller_state"
        )
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter(
            "operation_mode_topic", "/om6dof/operation_mode"
        )
        self.declare_parameter("control_cmd_topic", "/om6dof/control_cmd")
        self.declare_parameter("target_command_topic", "/om6dof/target_cmd")
        self.declare_parameter("target_status_topic", "/om6dof/target_status")
        self.declare_parameter("gripper_command_topic", "/om6dof/gripper_cmd")
        self.declare_parameter("gripper_state_topic", "/om6dof/gripper_state")
        self.declare_parameter(
            "gripper_action", "/gripper_controller/gripper_cmd"
        )
        self.declare_parameter("gripper_open_target", 0.019)
        self.declare_parameter("gripper_close_target", -0.010)
        self.declare_parameter(
            "operation_mode_state_topic", "/om6dof/operation_mode/state"
        )
        self.declare_parameter(
            "remote_enabled_state_topic", "/om6dof/remote_enabled/state"
        )

        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("joint_state_timeout_seconds", 1.0)
        self.declare_parameter("control_cmd_timeout_seconds", 0.3)
        self.declare_parameter("max_joint_command_velocity", 1.4)
        # Lead-through follow rate. A hand pushes the arm slowly; an arm
        # falling under its own weight accelerates past this, and the
        # command then stops chasing it so the servo catches the arm
        # instead of following it all the way down.
        self.declare_parameter("float_follow_velocity", 0.35)
        self.declare_parameter("joint_limit_margin", 0.02)
        self.declare_parameter(
            "ready_pose_degrees", list(DEFAULT_READY_JOINT_POSITIONS_DEG)
        )
        self.declare_parameter(
            "transition_pose_degrees",
            list(DEFAULT_TRANSITION_JOINT_POSITIONS_DEG),
        )
        self.declare_parameter("pose_profile_duration_seconds", 4.0)
        self.declare_parameter("target_reach_duration_seconds", 5.0)

        self.declare_parameter("ik_enabled", True)
        self.declare_parameter("ik_base_link", "world")
        self.declare_parameter("ik_tip_link", "end_effector_link")
        self.declare_parameter("ik_urdf_pkg", "om6dof_description")
        self.declare_parameter("ik_xacro_rel", "urdf/om6dof_v2.urdf.xacro")
        self.declare_parameter("ik_damping", 0.05)
        self.declare_parameter("max_cartesian_linear_velocity", 0.17)
        self.declare_parameter("max_cartesian_angular_velocity", 1.95)
        self.declare_parameter("max_cylindrical_theta_velocity", 1.10)
        self.declare_parameter("cylindrical_origin_xy", [0.012, 0.0])
        self.declare_parameter("cylindrical_min_radius", 0.03)
        self.declare_parameter("ik_tool_frame_rotation", True)
        self.declare_parameter("ik_manipulability_warning_threshold", 1.0e-6)
        self.declare_parameter("ik_self_collision", True)
        self.declare_parameter("ik_collision_radius", 0.025)

        self.joint_names = [
            str(value) for value in self.get_parameter("joint_names").value
        ]
        if len(self.joint_names) != 6:
            raise RuntimeError("joint_names must contain the six arm joints")
        self.arm_controller = str(self.get_parameter("arm_controller").value)
        self.rate = float(self.get_parameter("publish_rate_hz").value)
        self.joint_state_timeout = float(
            self.get_parameter("joint_state_timeout_seconds").value
        )
        self.control_cmd_timeout = float(
            self.get_parameter("control_cmd_timeout_seconds").value
        )
        self.max_joint_velocity = float(
            self.get_parameter("max_joint_command_velocity").value
        )
        if (
            not all(math.isfinite(value) for value in (
                self.rate,
                self.joint_state_timeout,
                self.control_cmd_timeout,
                self.max_joint_velocity,
            ))
            or self.rate <= 0.0
            or self.joint_state_timeout <= 0.0
            or self.control_cmd_timeout <= 0.0
            or self.max_joint_velocity <= 0.0
        ):
            raise RuntimeError("invalid controller rate/timeout/velocity")
        self.nominal_dt = 1.0 / self.rate

        lower, upper = self._load_urdf_joint_limits()
        margin = float(self.get_parameter("joint_limit_margin").value)
        if (
            not math.isfinite(margin)
            or margin < 0.0
            or any(lo + margin >= hi - margin for lo, hi in zip(lower, upper))
        ):
            raise RuntimeError("joint_limit_margin is incompatible with limits")
        self.joint_lower = [value + margin for value in lower]
        self.joint_upper = [value - margin for value in upper]
        self.ready_pose = clamp_positions(
            [
                math.radians(value) for value in validated_joint_positions(
                    self.get_parameter("ready_pose_degrees").value,
                    "ready_pose_degrees",
                )
            ],
            self.joint_lower,
            self.joint_upper,
        )
        self.transition_pose = clamp_positions(
            [
                math.radians(value) for value in validated_joint_positions(
                    self.get_parameter("transition_pose_degrees").value,
                    "transition_pose_degrees",
                )
            ],
            self.joint_lower,
            self.joint_upper,
        )
        self.pose_profile_duration = float(
            self.get_parameter("pose_profile_duration_seconds").value
        )
        self.target_reach_duration = float(
            self.get_parameter("target_reach_duration_seconds").value
        )

        self.max_cartesian_linear_velocity = float(
            self.get_parameter("max_cartesian_linear_velocity").value
        )
        self.max_cartesian_angular_velocity = float(
            self.get_parameter("max_cartesian_angular_velocity").value
        )
        self.max_cylindrical_theta_velocity = float(
            self.get_parameter("max_cylindrical_theta_velocity").value
        )
        self.cylindrical_origin_xy = np.asarray(
            self.get_parameter("cylindrical_origin_xy").value, dtype=float
        )
        self.cylindrical_min_radius = float(
            self.get_parameter("cylindrical_min_radius").value
        )
        self.ik_tool_frame_rotation = bool(
            self.get_parameter("ik_tool_frame_rotation").value
        )
        self.ik_manipulability_warning_threshold = float(
            self.get_parameter("ik_manipulability_warning_threshold").value
        )
        self.ik_self_collision = bool(
            self.get_parameter("ik_self_collision").value
        )
        self.ik_collision_radius = float(
            self.get_parameter("ik_collision_radius").value
        )
        scalars = np.asarray([
            self.max_cartesian_linear_velocity,
            self.max_cartesian_angular_velocity,
            self.max_cylindrical_theta_velocity,
            self.cylindrical_min_radius,
            self.ik_manipulability_warning_threshold,
            self.ik_collision_radius,
            self.pose_profile_duration,
            self.target_reach_duration,
        ])
        if (
            not np.all(np.isfinite(scalars))
            or self.max_cartesian_linear_velocity <= 0.0
            or self.max_cartesian_angular_velocity <= 0.0
            or self.max_cylindrical_theta_velocity <= 0.0
            or self.cylindrical_min_radius < 0.0
            or self.ik_manipulability_warning_threshold < 0.0
            or self.ik_collision_radius <= 0.0
            or self.pose_profile_duration <= 0.0
            or self.target_reach_duration <= 0.0
            or self.cylindrical_origin_xy.shape != (2,)
            or not np.all(np.isfinite(self.cylindrical_origin_xy))
        ):
            raise RuntimeError("invalid pose/Cartesian/cylindrical parameters")

        self.lock = threading.RLock()
        self.motion_mode = MODE_JOINT
        self.last_coordinate_mode = MODE_SEMI_CYLINDRICAL

        # The one thing this node owns and publishes.
        self.offset = np.zeros(6)
        self.offset_blocked = False

        self.joint_positions: dict[str, float] = {}
        self.last_joint_state = 0.0
        self.startup_pose: Optional[list[float]] = None
        self.control_velocity = np.zeros(6)
        self.last_control_cmd = 0.0
        self.last_tick = time.monotonic()
        self._custom_z_target = None

        # Autonomous reference, straight from the trajectory controller. Only
        # FLOAT needs it: to make the total command track the measured arm the
        # offset has to cancel whatever arm_controller is holding.
        self.arm_reference: Optional[list[float]] = None
        self.seen_goal_ids: set[bytes] = set()

        self.f1_destination = MODE_READY
        self.target_active = False
        self.target_mode = ""
        self.target_goal: Optional[list[float]] = None
        self.target_command_joints: Optional[list[float]] = None
        self.target_request_id = ""
        self.target_approximate = False
        self.target_status_state = "idle"
        self.target_status_message = "no target yet"
        self.last_target_status_publish = 0.0

        self.ik = None

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

        self.offset_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("offset_command_topic").value),
            10,
        )
        self.operation_state_pub = self.create_publisher(
            String,
            str(self.get_parameter("operation_mode_state_topic").value),
            state_qos,
        )
        self.remote_state_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("remote_enabled_state_topic").value),
            state_qos,
        )
        self.gripper_state_pub = self.create_publisher(
            String,
            str(self.get_parameter("gripper_state_topic").value),
            state_qos,
        )
        self.target_status_pub = self.create_publisher(
            String,
            str(self.get_parameter("target_status_topic").value),
            state_qos,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._on_joint_state,
            20,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("operation_mode_topic").value),
            self._on_operation_mode,
            mode_qos,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("control_cmd_topic").value),
            self._on_control_cmd,
            command_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("target_command_topic").value),
            self._on_target_command,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("gripper_command_topic").value),
            self._on_gripper_command,
            10,
        )
        self.create_subscription(
            JointTrajectoryControllerState,
            str(self.get_parameter("arm_state_topic").value),
            self._on_arm_state,
            10,
        )
        # Every autonomous goal -- ours, MoveIt's, graph_pick's -- shows up
        # here. A new one means the planner just replanned from the measured
        # pose, which already contains the operator's offset, so keeping that
        # offset would apply it twice. Rebase to zero instead.
        arm_action = str(self.get_parameter("arm_action").value)
        self.create_subscription(
            GoalStatusArray,
            arm_action.rstrip("/") + "/_action/status",
            self._on_arm_goal_status,
            10,
        )

        self.arm_client = ActionClient(self, FollowJointTrajectory, arm_action)
        self.gripper_client = ActionClient(
            self, GripperCommand,
            str(self.get_parameter("gripper_action").value),
        )

        self._initialize_ik()
        if self.ik is not None:
            # The per-tick joint-4 guard still holds motion until feedback and
            # the commanded joint position are near zero.
            self.motion_mode = MODE_SEMI_CYLINDRICAL
        self._initialize_admittance()
        self.create_timer(self.nominal_dt, self._tick)
        self._publish_state()
        self._publish_target_status_locked("idle", "no target yet")
        self.get_logger().info(
            "controller ready: /om6dof/control_cmd -> "
            f"{self.offset_pub.topic_name} (additive operator channel); "
            "absolute poses go to arm_controller as trajectory goals"
        )

    # ---------------------------------------------------------------- state

    def _reported_mode_locked(self) -> str:
        return self.motion_mode

    def _publish_state(self) -> None:
        self.operation_state_pub.publish(
            String(data=self._reported_mode_locked())
        )
        # There is no ownership any more: the operator channel is live for as
        # long as this node is. Kept so existing dashboards keep parsing.
        self.remote_state_pub.publish(Bool(data=True))

    def _initialize_ik(self) -> None:
        if not bool(self.get_parameter("ik_enabled").value):
            self.get_logger().warn(
                "IK disabled; only JOINT operation mode is available"
            )
            return
        try:
            from .ik_solver import IKSolver

            self.ik = IKSolver(
                base_link=str(self.get_parameter("ik_base_link").value),
                tip_link=str(self.get_parameter("ik_tip_link").value),
                urdf_pkg=str(self.get_parameter("ik_urdf_pkg").value),
                damping=float(self.get_parameter("ik_damping").value),
                xacro_rel=str(self.get_parameter("ik_xacro_rel").value),
            )
            self.ik.q_min = np.asarray(self.joint_lower)
            self.ik.q_max = np.asarray(self.joint_upper)
            self.get_logger().info(
                f"IK ready: {self.ik.base_link} -> {self.ik.tip_link}"
            )
        except Exception as exc:
            self.ik = None
            self.get_logger().error(
                f"IK initialization failed; coordinate modes disabled: {exc}"
            )

    def _load_urdf_joint_limits(self) -> tuple[list[float], list[float]]:
        """Load the one authoritative joint range used by both URDF and jog."""
        from .ik_solver import joint_limits_from_urdf

        try:
            lower, upper = joint_limits_from_urdf(
                self.joint_names,
                base_link=str(self.get_parameter("ik_base_link").value),
                tip_link=str(self.get_parameter("ik_tip_link").value),
                urdf_pkg=str(self.get_parameter("ik_urdf_pkg").value),
                xacro_rel=str(self.get_parameter("ik_xacro_rel").value),
            )
        except Exception as exc:
            raise RuntimeError(
                "could not load finite joint limits from the OM6DOF URDF"
            ) from exc
        self.get_logger().info(
            "joint limits loaded from URDF: "
            + ", ".join(
                f"{name}=[{lo:.3f}, {hi:.3f}]"
                for name, lo, hi in zip(self.joint_names, lower, upper)
            )
        )
        return lower, upper

    def _joint_vector_locked(self) -> Optional[list[float]]:
        if len(self.joint_positions) != 6:
            return None
        return [self.joint_positions[name] for name in self.joint_names]

    def _joint_state_fresh_locked(self, now: float) -> bool:
        return bool(self.last_joint_state) and (
            now - self.last_joint_state <= self.joint_state_timeout
        )

    def _on_joint_state(self, msg: JointState) -> None:
        indices = {name: index for index, name in enumerate(msg.name)}
        positions: dict[str, float] = {}
        for name in self.joint_names:
            index = indices.get(name)
            if index is None or index >= len(msg.position):
                continue
            value = float(msg.position[index])
            if math.isfinite(value):
                positions[name] = value
        if len(positions) != 6:
            return
        with self.lock:
            self.joint_positions = positions
            now = time.monotonic()
            self.last_joint_state = now
            if self.startup_pose is None:
                self.startup_pose = [positions[n] for n in self.joint_names]
                self.get_logger().info(
                    'startup pose captured from joint feedback (rad): '
                    + ', '.join(f'{value:.5f}' for value in self.startup_pose))
            self._try_finish_ready_locked()
            # Keep the web form's "Use current position" feedback fresh
            # without making this status topic another high-rate joint stream.
            if now - self.last_target_status_publish >= 0.2:
                self._publish_target_status_locked(
                    self.target_status_state, self.target_status_message
                )

    def _on_arm_state(self, msg: JointTrajectoryControllerState) -> None:
        point = msg.reference if msg.reference.positions else msg.desired
        if not point.positions:
            return
        index = {name: i for i, name in enumerate(msg.joint_names)}
        values = []
        for name in self.joint_names:
            i = index.get(name)
            if i is None or i >= len(point.positions):
                return
            values.append(float(point.positions[i]))
        if not all(math.isfinite(value) for value in values):
            return
        with self.lock:
            self.arm_reference = values
            self._arm_state_updated = time.monotonic()
            self._on_admittance_arm_state(msg)
            self._try_finish_ready_locked()

    def _try_finish_ready_locked(self) -> None:
        if not self._ready_pending or not self._ready_succeeded:
            return
        feedback = self._joint_vector_locked()
        if feedback is None or not self._joint_state_fresh_locked(time.monotonic()):
            return
        if self.ik is None:
            self._ready_pending = False
            self._set_motion_mode_locked(MODE_JOINT, 'READY complete')
            return
        reference = self._autonomous_reference_locked(feedback)
        if abs(feedback[3]) > 0.01 or abs(reference[3] + self.offset[3]) > 0.01:
            return
        self._ready_pending = False
        self._set_motion_mode_locked(MODE_SEMI_CYLINDRICAL, 'READY complete')

    def _ready_goal_response(self, generation, future) -> None:
        with self.lock:
            if generation != self._ready_generation or not self._ready_pending:
                return
            try:
                handle = future.result()
                if not handle.accepted:
                    raise ValueError('goal rejected')
                handle.get_result_async().add_done_callback(
                    lambda result: self._ready_goal_result(generation, result))
            except Exception as exc:
                self._ready_pending = False
                self.get_logger().warn(f'READY failed; CUSTOM not selected: {exc}')

    def _ready_goal_result(self, generation, future) -> None:
        with self.lock:
            if generation != self._ready_generation or not self._ready_pending:
                return
            try:
                outcome = future.result()
                if (outcome.status != GoalStatus.STATUS_SUCCEEDED or
                        outcome.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
                    raise ValueError('trajectory did not succeed')
            except Exception as exc:
                self._ready_pending = False
                self.get_logger().warn(f'READY failed; CUSTOM not selected: {exc}')
                return
            self._ready_succeeded = True
            self._try_finish_ready_locked()

    def _on_arm_goal_status(self, msg: GoalStatusArray) -> None:
        rebase = False
        with self.lock:
            self._arm_action_active = any(status.status in (
                GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                GoalStatus.STATUS_CANCELING) for status in msg.status_list)
            for status in msg.status_list:
                if status.status not in (
                    GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING
                ):
                    continue
                key = bytes(status.goal_info.goal_id.uuid)
                if key in self.seen_goal_ids:
                    continue
                self.seen_goal_ids.add(key)
                rebase = True
            if len(self.seen_goal_ids) > 256:
                self.seen_goal_ids.clear()
            if rebase and self._admittance_target_active:
                self._admittance_preempted_locked()
            if rebase:
                self._custom_z_target = None
            if rebase and np.any(np.abs(self.offset) > 1.0e-9):
                self.offset = np.zeros(6)
                self.get_logger().info(
                    "manual offset rebased to zero: a new autonomous "
                    "trajectory planned from the current measured pose"
                )

    # -------------------------------------------------------------- offsets

    def _float_follow_velocity(self) -> float:
        """How fast the lead-through offset may chase the arm.

        Read live rather than cached at startup: finding a value that feels
        light without following a sag means trying a few while holding the
        arm, and a restart between attempts loses both the pose and the feel.
        """
        try:
            value = float(self.get_parameter("float_follow_velocity").value)
        except (TypeError, ValueError):
            return 0.35
        if not math.isfinite(value) or value <= 0.0:
            return 0.35
        return min(value, self.max_joint_velocity)

    def _cartesian_twist_locked(
        self, feedback: Sequence[float], velocity: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """Map the six jog channels onto a base-frame twist for the tip."""
        if self.motion_mode == MODE_CARTESIAN:
            linear = limit_norm(velocity[:3], self.max_cartesian_linear_velocity)
        else:
            # Cylindrical channels are (radial, theta rate, vertical) around
            # the configured column, resolved at wherever the tip actually is.
            position, _ = self.ik.fk_pose(np.asarray(feedback, dtype=float))
            dx = float(position[0] - self.cylindrical_origin_xy[0])
            dy = float(position[1] - self.cylindrical_origin_xy[1])
            radius = math.hypot(dx, dy)
            if radius < max(self.cylindrical_min_radius, 1.0e-6):
                # On the column the tangential direction is undefined; radial
                # and vertical still are, so keep those rather than refuse.
                radial = np.zeros(3)
                tangential = np.zeros(3)
            else:
                radial = np.array([dx / radius, dy / radius, 0.0])
                tangential = np.array([-dy / radius, dx / radius, 0.0])
            radial_speed = max(
                -self.max_cartesian_linear_velocity,
                min(self.max_cartesian_linear_velocity, float(velocity[0])),
            )
            theta_rate = max(
                -self.max_cylindrical_theta_velocity,
                min(self.max_cylindrical_theta_velocity, float(velocity[1])),
            )
            vertical = max(
                -self.max_cartesian_linear_velocity,
                min(self.max_cartesian_linear_velocity, float(velocity[2])),
            )
            linear = (
                radial * radial_speed
                + tangential * (theta_rate * radius)
                + np.array([0.0, 0.0, vertical])
            )
            linear = limit_norm(linear, self.max_cartesian_linear_velocity)

        if self.motion_mode == MODE_SEMI_CYLINDRICAL:
            # Only roll steers the wrist here; pitch and yaw are direct joint
            # nudges applied by the caller, so leaving them on the twist as
            # well would have IK and the nudge pulling the same joints apart.
            angular = limit_norm(
                np.array([float(velocity[3]), 0.0, 0.0]),
                self.max_cartesian_angular_velocity,
            )
        else:
            angular = limit_norm(
                velocity[3:], self.max_cartesian_angular_velocity
            )
        if (
            self.ik_tool_frame_rotation
            and float(np.linalg.norm(angular)) > 1.0e-9
        ):
            angular = self.ik.ee_to_base_angular(
                np.asarray(feedback, dtype=float), angular
            )
        return linear, angular

    def _offset_delta_locked(
        self, feedback: Sequence[float], velocity: np.ndarray, dt: float
    ) -> np.ndarray:
        """Joint-space increment the operator is asking for this tick."""
        if self.motion_mode == MODE_JOINT:
            return np.clip(
                velocity, -self.max_joint_velocity, self.max_joint_velocity
            ) * dt
        if self.ik is None:
            return np.zeros(6)
        try:
            if self.motion_mode == MODE_SEMI_CYLINDRICAL:
                q = np.asarray(feedback, dtype=float)
                reference = self._autonomous_reference_locked(feedback)
                if abs(q[3]) > 0.01 or abs(reference[3] + self.offset[3]) > 0.01:
                    self._custom_z_target = None
                    self.get_logger().warn(
                        'CUSTOM held: joint 4 or its command is not near zero',
                        throttle_duration_sec=2.0)
                    return np.zeros(6)
                coordinates = self.ik.semi_cylindrical_fk(q, self.cylindrical_origin_xy)
                if coordinates[0] < self.cylindrical_min_radius:
                    self._custom_z_target = None
                    raise ValueError('radius is too small for semi-cylindrical IK')
                z = float(coordinates[2])
                vertical = float(np.clip(velocity[2], -self.max_cartesian_linear_velocity,
                                         self.max_cartesian_linear_velocity))
                if not math.isfinite(z):
                    raise ValueError('IK produced non-finite height')
                if getattr(self, '_arm_action_active', False):
                    self._custom_z_target = None
                elif abs(vertical) > 1e-6:
                    # While the operator moves Z, follow the measured height;
                    # releasing the input makes the current height the target.
                    self._custom_z_target = z
                elif self._custom_z_target is None:
                    self._custom_z_target = z
                else:
                    error = self._custom_z_target - z
                    # Do not chase large external displacement or a blocked arm.
                    if abs(error) > 0.03:
                        self._custom_z_target = z
                    elif abs(error) > 0.001:
                        vertical = float(np.clip(2.0 * error, -0.01, 0.01))
                rates = np.array([
                    np.clip(velocity[0], -self.max_cartesian_linear_velocity, self.max_cartesian_linear_velocity),
                    np.clip(velocity[1], -self.max_cylindrical_theta_velocity, self.max_cylindrical_theta_velocity),
                    vertical,
                    np.clip(velocity[3], -self.max_cartesian_angular_velocity, self.max_cartesian_angular_velocity),
                    np.clip(velocity[4], -self.max_cartesian_angular_velocity, self.max_cartesian_angular_velocity),
                ])
                joint_velocity = self.ik.semi_cylindrical_velocity_ik(
                    q, rates, self.cylindrical_origin_xy)
                if not np.all(np.isfinite(joint_velocity)):
                    raise ValueError('IK produced non-finite joint velocity')
                peak = float(np.max(np.abs(joint_velocity)))
                if peak > self.max_joint_velocity:
                    joint_velocity *= self.max_joint_velocity / peak
                delta = joint_velocity * dt
                delta[3] = np.clip(-reference[3] - self.offset[3],
                                   -self.max_joint_velocity * dt, self.max_joint_velocity * dt)
                return delta
            linear, angular = self._cartesian_twist_locked(feedback, velocity)
            q = np.asarray(feedback, dtype=float)
            joint_velocity = np.zeros(6)
            if float(np.linalg.norm(np.concatenate([linear, angular]))) > 1.0e-9:
                joint_velocity = self.ik.velocity_ik_priority(q, linear, angular)
                if not np.all(np.isfinite(joint_velocity)):
                    raise ValueError("IK produced non-finite joint velocity")
                peak = float(np.max(np.abs(joint_velocity)))
                if peak > self.max_joint_velocity:
                    joint_velocity = joint_velocity * (
                        self.max_joint_velocity / peak
                    )
            delta = joint_velocity * dt
            return delta
        except Exception as exc:
            if self.motion_mode == MODE_SEMI_CYLINDRICAL:
                self._custom_z_target = None
            self.get_logger().error(
                f"coordinate jog failed; holding offset: {exc}",
                throttle_duration_sec=2.0,
            )
            return np.zeros(6)

    def _autonomous_reference_locked(
        self, feedback: Sequence[float]
    ) -> np.ndarray:
        """The pose arm_controller is commanding -- what the offset adds to.

        The motors are driven `reference + offset`, so every limit belongs on
        that sum. The measured pose is NOT a substitute: it already contains
        the offset, so using it would count the operator's contribution twice
        and halve the travel they actually get.
        """
        if self.arm_reference is not None:
            return np.asarray(self.arm_reference, dtype=float)
        # No controller_state stream to read it from. Backing our own offset
        # out of the measurement would be exact only while the arm is keeping
        # up -- if it is blocked, that estimate drifts and the offset could
        # grow without ever tripping the limit. Fall back to the measurement
        # itself: it double-counts the offset and so clamps early, costing
        # travel, which is the safe direction to be wrong in.
        self.get_logger().warn(
            "no arm_controller state; limiting jog against the measured pose",
            throttle_duration_sec=10.0,
        )
        return np.asarray(feedback, dtype=float)

    def _apply_offset_locked(
        self, feedback: Sequence[float], delta: np.ndarray
    ) -> None:
        """Accept `delta` only if the resulting total pose is safe."""
        if not np.any(np.abs(delta) > 0.0):
            return
        candidate = self.offset + delta
        # No separate ceiling on the offset itself: the only limit that means
        # anything is the URDF joint range applied to the sum the motors
        # actually see, which is reference + offset.
        reference = self._autonomous_reference_locked(feedback)
        total = np.asarray(
            clamp_positions(
                reference + candidate, self.joint_lower, self.joint_upper,
            ),
            dtype=float,
        )
        candidate = total - reference
        if self.ik is not None and self.ik_self_collision:
            try:
                if self.ik.self_collides(total, self.ik_collision_radius):
                    # Only refuse a jog that *enters* the boundary. This model
                    # is conservative enough to flag poses the arm rests in
                    # perfectly safely, and refusing from inside one would trap
                    # the operator there with no way to jog back out.
                    entering = not self.ik.self_collides(
                        np.asarray(feedback, dtype=float),
                        self.ik_collision_radius,
                    )
                    if entering:
                        if not self.offset_blocked:
                            self.get_logger().warn(
                                "manual jog blocked by self-collision boundary"
                            )
                        self.offset_blocked = True
                        self._custom_z_target = None
                        return
            except Exception as exc:
                self.get_logger().error(
                    f"self-collision check failed; holding offset: {exc}",
                    throttle_duration_sec=5.0,
                )
                return
        self.offset_blocked = False
        self.offset = candidate

    # ----------------------------------------------------------- operations

    def _set_motion_mode_locked(self, mode: str, source: str) -> bool:
        if self.target_active:
            self.get_logger().warn(f"{source} rejected: an owned target is still active")
            return False
        # FLOAT is a jog mode for this node even though control_math keeps it
        # out of MOTION_MODES (it is not a frame you jog *in*).
        if mode not in MOTION_MODES and mode not in (MODE_FLOAT, MODE_FORCE):
            return False
        if mode in COORDINATE_MODES and self.ik is None:
            self.get_logger().warn(
                f"{source} request rejected: IK is unavailable"
            )
            return False
        if mode == MODE_SEMI_CYLINDRICAL:
            feedback = self._joint_vector_locked()
            if (feedback is None or not self._joint_state_fresh_locked(time.monotonic())
                    or abs(feedback[3]) > 0.01
                    or abs(self._autonomous_reference_locked(feedback)[3] + self.offset[3]) > 0.01):
                self.get_logger().warn(
                    'CUSTOM rejected: set joint 4 and its command near zero first')
                return False
        self.motion_mode = mode
        self._custom_z_target = None
        if mode == MODE_FORCE:
            # Wrench control is owned exclusively by the Mode-0 effort stack.
            # Never retain a position-offset command while it is selected.
            self.offset = np.zeros(6)
        if mode in COORDINATE_MODES:
            self.last_coordinate_mode = mode
        self.control_velocity = np.zeros(6)
        self.last_control_cmd = 0.0
        self._publish_state()
        self.get_logger().info(f"operation mode -> {mode} ({source})")
        return True

    def _trajectory_goal(
        self, waypoints: Sequence[Sequence[float]], duration: float
    ) -> FollowJointTrajectory.Goal:
        """One goal for arm_controller; it owns the time profiling."""
        trajectory = JointTrajectory()
        trajectory.joint_names = list(self.joint_names)
        elapsed = 0.0
        for waypoint in waypoints:
            elapsed += duration
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in waypoint]
            point.velocities = [0.0] * 6
            point.time_from_start = DurationMsg(
                sec=int(elapsed),
                nanosec=int(round((elapsed - int(elapsed)) * 1e9)),
            )
            trajectory.points.append(point)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        # The JTC default position tolerance is zero (= disabled). A stationary
        # arm that never followed READY must not be reported as a successful
        # trajectory merely because its reference reached the final timestamp.
        goal.goal_tolerance = [
            JointTolerance(name=name, position=0.03)
            for name in self.joint_names
        ]
        return goal

    def _send_pose_locked(
        self,
        operation: str,
        waypoints: Sequence[Sequence[float]],
        duration: Optional[float] = None,
    ) -> bool:
        if self.target_active:
            self.get_logger().warn(
                f"{operation} rejected: an owned target is still pending or active"
            )
            return False
        now = time.monotonic()
        if not self._joint_state_fresh_locked(now):
            self.get_logger().warn(
                f"{operation} rejected: /joint_states is stale"
            )
            return False
        if not self.arm_client.server_is_ready():
            self.get_logger().warn(
                f"{operation} rejected: arm_controller action is unavailable"
            )
            return False
        if operation in (MODE_STARTUP, MODE_REST) and any(
            value < lower or value > upper
            for value, lower, upper in zip(waypoints[-1], self.joint_lower, self.joint_upper)
        ):
            self.get_logger().warn(
                f'{operation} rejected: captured startup pose is outside joint limits')
            return False
        clamped = [
            clamp_positions(waypoint, self.joint_lower, self.joint_upper)
            for waypoint in waypoints
        ]
        goal = self._trajectory_goal(
            clamped, self.pose_profile_duration if duration is None else duration
        )
        if operation == "TARGET":
            # An absolute pose rebases the operator correction. Freeze at zero
            # throughout this owned action so a measured-position cancel hold
            # cannot have the same correction added a second time.
            self.offset = np.zeros(6)
            self.control_velocity = np.zeros(6)
            self.last_control_cmd = 0.0
            self._target_generation += 1
            generation = self._target_generation
            self._target_send_pending = True
            self._target_goal_handle = None
            self._target_stop_requested = False
            self._target_cancel_pending = False
            self._target_result_pending = False
            self._target_action_succeeded = False
            self.target_active = True
            self._publish_target_status_locked(
                "requesting", "Waiting for arm_controller to accept the target."
            )
            try:
                future = self.arm_client.send_goal_async(goal)
                future.add_done_callback(
                    lambda result: self._on_target_goal_response(generation, result)
                )
            except Exception as exc:
                # A transport exception is not evidence that the server never
                # received the goal. Keep motion inputs locked until resolved.
                self._publish_target_status_locked(
                    "blocked", f"Target acceptance unknown ({exc}); no automatic retry."
                )
        else:
            if operation == MODE_READY:
                self._ready_generation += 1
                generation = self._ready_generation
                self._ready_pending = True
                self._ready_succeeded = False
                self.control_velocity = np.zeros(6)
                self.last_control_cmd = 0.0
                try:
                    future = self.arm_client.send_goal_async(goal)
                    future.add_done_callback(
                        lambda result: self._ready_goal_response(generation, result))
                except Exception as exc:
                    self.get_logger().error(f'READY acceptance unknown; holding jog: {exc}')
                    return False
            else:
                self.arm_client.send_goal_async(goal)
            self.offset = np.zeros(6)
        self.get_logger().info(
            f"{operation} sent to arm_controller: {len(clamped)} waypoint(s)"
        )
        return True

    def _on_operation_mode(self, msg: String) -> None:
        with self.lock:
            if self.target_active:
                self.get_logger().warn(
                    "operation mode rejected: wait for the owned target result or Stop confirmation"
                )
                return
            self._ready_generation += 1
            self._ready_pending = False
            self._ready_succeeded = False
        raw_mode = str(msg.data).strip().upper()
        if raw_mode == "TOGGLE_REST_READY":
            with self.lock:
                if self.startup_pose is None:
                    self.get_logger().warn(
                        "REST/READY toggle rejected: startup pose has not "
                        "been captured"
                    )
                    return
                vector = self._joint_vector_locked()
                if vector is None:
                    self.get_logger().warn(
                        "REST/READY toggle rejected: /joint_states is stale"
                    )
                    return
                rest_distance = float(np.linalg.norm(
                    np.asarray(vector) - np.asarray(self.startup_pose)
                ))
                ready_distance = float(np.linalg.norm(
                    np.asarray(vector) - np.asarray(self.ready_pose)
                ))
                if rest_distance <= ready_distance:
                    self._send_pose_locked(
                        MODE_READY, [self.transition_pose, self.ready_pose]
                    )
                else:
                    self._send_pose_locked(
                        MODE_STARTUP, [self.transition_pose, self.startup_pose]
                    )
            return
        try:
            mode = normalize_operation_mode(msg.data)
        except ValueError as exc:
            self.get_logger().warn(f"operation mode rejected: {exc}")
            return
        with self.lock:
            if mode in MOTION_MODES:
                self._set_motion_mode_locked(mode, "operation_mode")
                return
            if mode == MODE_FLOAT:
                if self._set_motion_mode_locked(MODE_FLOAT, "operation_mode"):
                    self.get_logger().warn(
                        "FLOAT: the arm is now free to push by hand. It holds "
                        "where friction holds it, which is not everywhere."
                    )
                return
            if mode == MODE_FORCE:
                self._set_motion_mode_locked(MODE_FORCE, "operation_mode")
                self.get_logger().warn(
                    "FORCE: position-offset output is zero; only the separately armed "
                    "Cartesian wrench stack may command effort."
                )
                return
            if mode == MODE_READY:
                self.f1_destination = MODE_READY
                self._send_pose_locked(
                    MODE_READY, [self.transition_pose, self.ready_pose]
                )
                return
            if mode in (MODE_STARTUP, MODE_REST):
                if self.startup_pose is None:
                    self.get_logger().warn(
                        f"{mode} rejected: startup pose has not been captured"
                    )
                    return
                self.f1_destination = MODE_STARTUP
                self._send_pose_locked(
                    mode, [self.transition_pose, self.startup_pose]
                )
                self._set_motion_mode_locked(MODE_JOINT, mode)
                return
            if mode == MODE_AUTONOMOUS:
                # This used to mean "hand the hardware back". Nothing to hand
                # back any more: both channels are always live. Zeroing the
                # operator's offset is all the request can still mean.
                self.offset = np.zeros(6)
                self.get_logger().info(
                    "manual offset cleared; the arm now follows the "
                    "autonomous channel alone"
                )
                return
            self.get_logger().warn(f"operation mode '{mode}' not handled")

    def _ready_resume_mode_locked(self) -> str:
        resume = self.last_coordinate_mode
        if resume not in COORDINATE_MODES:
            resume = MODE_CARTESIAN
        return resume if self.ik is not None else MODE_JOINT

    # ------------------------------------------------------------- commands

    def _on_control_cmd(self, msg: Float64MultiArray) -> None:
        try:
            values = validated_control_command(msg.data)
        except ValueError as exc:
            self.get_logger().warn(f"control_cmd rejected: {exc}")
            return
        with self.lock:
            if self.target_active or self._ready_pending:
                self._custom_z_target = None
                self.control_velocity = np.zeros(6)
                self.last_control_cmd = 0.0
                return
            if self.motion_mode == MODE_FORCE:
                # Keep receiving the canonical six direction channels for the
                # wrench bridge, but never interpret them as a position jog.
                self.control_velocity = np.zeros(6)
                self.last_control_cmd = 0.0
                return
            self.control_velocity = values
            self.last_control_cmd = time.monotonic()

    def _on_gripper_command(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if command == "open":
            target = float(self.get_parameter("gripper_open_target").value)
        elif command == "close":
            target = float(self.get_parameter("gripper_close_target").value)
        else:
            self.get_logger().warn(f"gripper command '{command}' rejected")
            return
        if not self.gripper_client.server_is_ready():
            self.get_logger().warn("gripper action server is not ready")
            return
        goal = GripperCommand.Goal()
        goal.command.position = target
        goal.command.max_effort = 0.0
        self.gripper_client.send_goal_async(goal)
        self.gripper_state_pub.publish(
            String(data=f"{command.upper()} requested")
        )

    # --------------------------------------------------------- web targets

    def _on_target_goal_response(self, generation, future) -> None:
        with self.lock:
            if generation != self._target_generation or not self._target_send_pending:
                return
            try:
                handle = future.result()
                accepted = handle.accepted
            except Exception as exc:
                self._publish_target_status_locked(
                    "blocked", f"Target acceptance unknown ({exc}); motion may continue."
                )
                return
            self._target_send_pending = False
            if not accepted:
                self.target_active = False
                self._publish_target_status_locked(
                    "stopped" if self._target_stop_requested else "rejected",
                    "arm_controller rejected the target; no owned trajectory was accepted.",
                )
                return
            self._target_goal_handle = handle
            self._publish_target_status_locked(
                "stopping" if self._target_stop_requested else "running",
                "Target accepted; cancellation requested." if self._target_stop_requested
                else "Target accepted by arm_controller.",
            )
            self._watch_target_result_locked(generation)
            if self.target_active and self._target_stop_requested:
                self._cancel_target_locked(generation)

    def _watch_target_result_locked(self, generation) -> None:
        if self._target_result_pending or self._target_goal_handle is None:
            return
        self._target_result_pending = True
        try:
            future = self._target_goal_handle.get_result_async()
            future.add_done_callback(
                lambda result: self._on_target_result(generation, result)
            )
        except Exception as exc:
            self._target_result_pending = False
            self._publish_target_status_locked(
                "blocked", f"Target result unavailable ({exc}); motion may continue."
            )

    def _cancel_target_locked(self, generation) -> None:
        handle = self._target_goal_handle
        if handle is None or self._target_cancel_pending or not self.target_active:
            return
        self._target_cancel_pending = True
        try:
            # Exact owned handle only: never cancel all goals on the shared
            # controller, including READY, MoveIt and other clients' goals.
            future = handle.cancel_goal_async()
            future.add_done_callback(
                lambda result: self._on_target_cancel_response(generation, handle, result)
            )
        except Exception as exc:
            self._target_cancel_pending = False
            self._publish_target_status_locked(
                "blocked", f"Target cancellation failed ({exc}); motion may continue. Retry Stop."
            )

    def _on_target_cancel_response(self, generation, handle, future) -> None:
        with self.lock:
            if (generation != self._target_generation or not self.target_active
                    or handle is not self._target_goal_handle):
                return
            try:
                response = future.result()
                accepted = response.return_code == 0 and any(
                    bytes(info.goal_id.uuid) == bytes(handle.goal_id.uuid)
                    for info in response.goals_canceling
                )
            except Exception as exc:
                accepted = False
                detail = str(exc)
            else:
                detail = f"response code {response.return_code}"
            if accepted:
                self._publish_target_status_locked(
                    "stopping", "Cancellation accepted; waiting for the terminal target result."
                )
            else:
                self._target_cancel_pending = False
                self._publish_target_status_locked(
                    "blocked", f"Cancellation not confirmed ({detail}); motion may continue. Retry Stop."
                )

    def _on_target_result(self, generation, future) -> None:
        with self.lock:
            if generation != self._target_generation or not self.target_active:
                return
            self._target_result_pending = False
            try:
                result = future.result()
                status = result.status
                error_code = result.result.error_code
                error_string = result.result.error_string
            except Exception as exc:
                self._publish_target_status_locked(
                    "blocked", f"Target result unknown ({exc}); motion may continue. Retry Stop."
                )
                return
            if status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
                              GoalStatus.STATUS_ABORTED):
                self._publish_target_status_locked(
                    "blocked", f"Nonterminal target result ({status}); motion may continue."
                )
                return
            self.target_active = False
            self._target_goal_handle = None
            self._target_cancel_pending = False
            self.control_velocity = np.zeros(6)
            self.last_control_cmd = 0.0
            if status == GoalStatus.STATUS_CANCELED:
                self._publish_target_status_locked(
                    "stopped", "Target cancellation confirmed by arm_controller; this is not torque-off."
                )
            elif status == GoalStatus.STATUS_SUCCEEDED and error_code == 0:
                self._target_action_succeeded = True
                self._publish_target_status_locked(
                    "holding", "Trajectory completed; waiting for measured target confirmation."
                )
            else:
                self._publish_target_status_locked(
                    "blocked", f"Target ended without success (status {status}, code {error_code}): {error_string}"
                )

    def _stop_target_locked(self, request_id: str) -> None:
        if self._admittance_target_active:
            self._request_admittance_stop_locked(request_id)
            return
        self.target_request_id = request_id
        self.control_velocity = np.zeros(6)
        self.last_control_cmd = 0.0
        # Preserve the additive offset: dropping it would itself be a new
        # position command. Owned targets freeze it at zero from submission.
        if not self.target_active:
            self._publish_target_status_locked(
                "stopped", "No owned target trajectory to cancel; other clients' motion is unchanged."
            )
            return
        self._target_stop_requested = True
        self._publish_target_status_locked(
            "stopping", "Stop requested; waiting for target cancellation confirmation."
        )
        if self._target_goal_handle is not None:
            self._watch_target_result_locked(self._target_generation)
            self._cancel_target_locked(self._target_generation)

    def _target_current_locked(self) -> dict[str, list[float]]:
        """Return current arm pose in controller SI units for target feedback."""
        joint = self._joint_vector_locked()
        if joint is None:
            return {}
        current = {"joint": [float(value) for value in joint]}
        if self.ik is None:
            return current
        try:
            position, rotation = self.ik.fk_pose(np.asarray(joint, dtype=float))
            roll, pitch, yaw = rotation_to_zyx(rotation)
            current["cartesian"] = [float(v) for v in position] + list(
                tip_rotation_to_cartesian_rpy(rotation)
            )
            dx = float(position[0] - self.cylindrical_origin_xy[0])
            dy = float(position[1] - self.cylindrical_origin_xy[1])
            current["cylindrical"] = [
                math.hypot(dx, dy), math.atan2(dy, dx), float(position[2]),
                roll, pitch, yaw,
            ]
            forward_roll, forward_pitch, _ = tip_rotation_to_cartesian_rpy(rotation)
            current["semi_cylindrical"] = current["cylindrical"][:3] + [
                forward_roll, forward_pitch,
            ]
        except Exception as exc:
            self.get_logger().warn(
                f"target feedback FK unavailable: {exc}",
                throttle_duration_sec=5.0,
            )
        return current

    def _publish_target_status_locked(self, state: str, message: str) -> None:
        current = self._target_current_locked()
        errors = {}
        command_joints = getattr(self, "target_command_joints", None)
        if command_joints is not None and "joint" in current:
            errors["joint_max_deg"] = math.degrees(max(
                abs(a - b) for a, b in zip(command_joints, current["joint"])
            ))
        if (
            self.target_goal is not None
            and self.target_mode in (MODE_CARTESIAN, MODE_CYLINDRICAL, MODE_SEMI_CYLINDRICAL)
            and self.ik is not None
        ):
            try:
                goal = self.target_goal
                if self.target_mode == MODE_SEMI_CYLINDRICAL:
                    actual = np.asarray(current['semi_cylindrical'])
                    error = self.ik._semi_error(np.asarray(goal), actual)
                    ep = math.hypot(float(error[0]), float(error[2]))
                    ep = math.hypot(ep, float(goal[0] * error[1]))
                    er = float(np.linalg.norm(error[3:]))
                    errors.update(position_mm=1000.0 * ep,
                                  orientation_deg=math.degrees(er))
                elif self.target_mode == MODE_CARTESIAN:
                    position = np.asarray(goal[:3])
                    rotation = cartesian_rpy_to_tip_rotation(*goal[3:])
                else:
                    position = np.array([
                        self.cylindrical_origin_xy[0] + goal[0] * math.cos(goal[1]),
                        self.cylindrical_origin_xy[1] + goal[0] * math.sin(goal[1]),
                        goal[2],
                    ])
                    rotation = rotation_from_zyx(*goal[3:])
                if self.target_mode != MODE_SEMI_CYLINDRICAL and "joint" in current:
                    ep, er = pose_error(
                        self.ik, current["joint"], position, rotation
                    )
                    errors.update(
                        position_mm=1000.0 * ep, orientation_deg=math.degrees(er)
                    )
            except Exception as exc:
                self.get_logger().warn(
                    f"Target error feedback unavailable: {exc}",
                    throttle_duration_sec=5.0,
                )
        if self._target_action_succeeded and state in ("holding", "reached", "approximate"):
            joints_reached = errors.get("joint_max_deg", math.inf) <= math.degrees(
                JOINT_TOLERANCE
            )
            pose_reached = self.target_mode == MODE_JOINT or (
                errors.get("position_mm", math.inf) <= POSITION_TOLERANCE * 1000
                and errors.get("orientation_deg", math.inf)
                <= math.degrees(ORIENTATION_TOLERANCE)
            )
            fresh = self._joint_state_fresh_locked(time.monotonic())
            if fresh and joints_reached and pose_reached:
                state, message = (
                    "reached", "Target reached within feedback tolerance."
                )
            elif fresh and joints_reached and self.target_approximate:
                state, message = (
                    "approximate",
                    "Approximate IK goal reached; requested pose differs.",
                )
            else:
                state, message = (
                    "holding", "Trajectory completed; measured target is not confirmed."
                )
            if self._target_stop_requested:
                message = "Trajectory completed before cancellation. " + message
            if "position_mm" in errors:
                message += (
                    f" Error: {errors['position_mm']:.2f} mm / "
                    f"{errors['orientation_deg']:.2f} deg."
                )
        self.target_status_state = state
        self.target_status_message = message
        self.last_target_status_publish = time.monotonic()
        payload = {
            "state": state,
            "active": bool(self.target_active),
            "mode": self.target_mode,
            "request_id": self.target_request_id,
            "goal": self.target_goal,
            "approximate": bool(self.target_approximate),
            "current": current,
            "command_joints": command_joints,
            "errors": errors,
            "feedback_fresh": self._joint_state_fresh_locked(time.monotonic()),
            "message": message,
        }
        payload.update(self._admittance_status_locked())
        self.target_status_pub.publish(String(data=json.dumps(
            payload, separators=(",", ":"), allow_nan=False
        )))

    def _on_target_command(self, msg: String) -> None:
        """Accept one validated absolute target from the web monitor.

        The wire protocol deliberately uses SI units: metre/radian. The web
        UI converts its mm/degree form values before publishing this message.
        """
        try:
            request = json.loads(msg.data)
            if not isinstance(request, dict):
                raise ValueError("command must be an object")
            action = str(request.get("action", "")).strip().lower()
            request_id = str(request.get("request_id", ""))[:80]
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f"target command rejected: {exc}")
            return

        with self.lock:
            if action == "stop":
                self._stop_target_locked(request_id)
                return
            if self._ready_pending:
                self._publish_target_status_locked(
                    'rejected', 'Target rejected: wait for READY to finish.')
                return
            if self.target_active:
                self._publish_target_status_locked(
                    self.target_status_state,
                    "New target rejected: wait for the owned target result or Stop confirmation.",
                )
                return
            if action != "move":
                self._publish_target_status_locked(
                    "rejected", "Target rejected: action must be move or stop."
                )
                return

            # Echo even rejected requests so the web client can distinguish
            # this response from a delayed status for an older command.
            self.target_request_id = request_id
            if self.motion_mode == MODE_FLOAT:
                self._publish_target_status_locked(
                    "rejected", "Target rejected: select a coordinate mode before leaving FLOAT."
                )
                return

            mode = str(request.get("mode", "")).strip().upper()
            if mode in ('SEMI', 'SEMI_CYL', 'SEMI_CYLINDER', 'SEMI_CYLINDRICAL', 'SEMI_SILINDER'):
                mode = MODE_SEMI_CYLINDRICAL
            try:
                values = [float(value) for value in request.get("values", [])]
                reach_duration = float(
                    request.get("reach_duration", self.target_reach_duration)
                )
            except (TypeError, ValueError):
                values = []
                reach_duration = self.target_reach_duration
            if (
                mode not in (MODE_JOINT, MODE_CARTESIAN, MODE_CYLINDRICAL, MODE_SEMI_CYLINDRICAL)
                or len(values) != (5 if mode == MODE_SEMI_CYLINDRICAL else 6)
                or not all(math.isfinite(value) for value in values)
            ):
                self._publish_target_status_locked(
                    "rejected", "Target rejected: invalid mode or value count (semi: five; others: six)."
                )
                return
            if not math.isfinite(reach_duration) or not 0.5 <= reach_duration <= 5.0:
                self._publish_target_status_locked(
                    "rejected", "Target rejected: reach time must be 0.5-5 seconds."
                )
                return
            feedback = self._joint_vector_locked()
            if feedback is None or not self._joint_state_fresh_locked(
                time.monotonic()
            ):
                self._publish_target_status_locked(
                    "rejected", "Target rejected: joint feedback is unavailable."
                )
                return

            target_joint: Optional[list[float]] = None
            approximate = False
            if mode == MODE_JOINT:
                if any(
                    value < lower or value > upper
                    for value, lower, upper in zip(
                        values, self.joint_lower, self.joint_upper
                    )
                ):
                    self._publish_target_status_locked(
                        "rejected",
                        "Target rejected: joint target exceeds URDF limits.",
                    )
                    return
                target_joint = values
            elif self.ik is None:
                self._publish_target_status_locked(
                    "rejected", "Target rejected: inverse kinematics is unavailable."
                )
                return
            elif mode == MODE_SEMI_CYLINDRICAL:
                try:
                    if values[0] < self.cylindrical_min_radius:
                        raise ValueError('cylindrical radius is too small')
                    if abs(feedback[3]) > 0.01:
                        raise ValueError('set joint 4 near zero first')
                    solution, converged = self.ik.solve_semi_cylindrical_ik(
                        np.asarray(feedback), np.asarray(values), self.cylindrical_origin_xy)
                    if (not converged or not np.all(np.isfinite(solution)) or
                            np.any(solution < self.joint_lower) or np.any(solution > self.joint_upper)):
                        raise ValueError('no valid five-coordinate solution')
                    target_joint = solution.tolist()
                except Exception as exc:
                    self._publish_target_status_locked(
                        'rejected', f'Target rejected: pose planning failed ({exc}).')
                    return
            else:
                if mode == MODE_CARTESIAN:
                    target_position = np.asarray(values[:3], dtype=float)
                    target_rotation = cartesian_rpy_to_tip_rotation(*values[3:])
                else:
                    radius, theta, z, roll, pitch, yaw = values
                    if radius < self.cylindrical_min_radius:
                        self._publish_target_status_locked(
                            "rejected",
                            "Target rejected: cylindrical radius is too small.",
                        )
                        return
                    target_position = np.asarray([
                        self.cylindrical_origin_xy[0] + radius * math.cos(theta),
                        self.cylindrical_origin_xy[1] + radius * math.sin(theta),
                        z,
                    ])
                    target_rotation = rotation_from_zyx(roll, pitch, yaw)
                try:
                    plan = plan_pose(
                        self.ik, feedback, target_position, target_rotation,
                        self.joint_lower, self.joint_upper, self.ready_pose,
                        self.ik_collision_radius if self.ik_self_collision else None,
                    )
                except Exception as exc:
                    self._publish_target_status_locked(
                        "rejected", f"Target rejected: pose planning failed ({exc})."
                    )
                    return
                approximate = plan.approximate
                target_joint = plan.joints.tolist()

            if self.ik_self_collision and self.ik is not None:
                try:
                    if not path_is_clear(
                        self.ik, np.asarray(feedback), np.asarray(target_joint),
                        self.ik_collision_radius,
                    ):
                        raise ValueError("joint path is blocked")
                except Exception as exc:
                    self._publish_target_status_locked(
                        "blocked", f"Target blocked: {exc}"
                    )
                    return
            travel = max(abs(a - b) for a, b in zip(target_joint, feedback))
            peak_velocity = 2.0 * travel / reach_duration
            if peak_velocity > self.max_joint_velocity:
                minimum = 2.0 * travel / self.max_joint_velocity
                self._publish_target_status_locked(
                    "rejected",
                    f"Target requires at least {minimum:.2f} s to respect "
                    "joint velocity limits.",
                )
                return

            self.target_mode = mode
            self.target_goal = values
            self.target_command_joints = list(target_joint)
            self.target_request_id = request_id
            self.target_approximate = approximate
            if self._admittance_enabled:
                self._start_admittance_target_locked(feedback)
                return
            if not self._send_pose_locked("TARGET", [target_joint], reach_duration):
                self.target_active = False
                self._publish_target_status_locked(
                    "rejected", "Target rejected: arm_controller did not accept it."
                )

    # ------------------------------------------------------------------ loop

    def _tick(self) -> None:
        now = time.monotonic()
        actual_dt = now - self.last_tick
        dt = max(0.0, min(0.1, actual_dt))
        self.last_tick = now
        with self.lock:
            if self._admittance_target_active:
                # The observer/admittance watchdog must see a real timer gap;
                # the legacy jog clamp must not disguise missed control ticks.
                self._tick_admittance_locked(now, actual_dt)
            if self.target_active or self._ready_pending:
                # Do not let a cached/new jog or FLOAT offset alter the target
                # while acceptance/cancellation/result is still unresolved.
                self._custom_z_target = None
                self.control_velocity = np.zeros(6)
                self.last_control_cmd = 0.0
                command = self.offset.tolist()
            elif not self._joint_state_fresh_locked(now):
                self._custom_z_target = None
                # Without feedback there is no safe way to bound the sum, so
                # stop adding to the offset. The existing one is held, not
                # dropped: releasing it would move the arm by exactly the
                # amount the operator dialled in, with no one asking.
                self.get_logger().warn(
                    "/joint_states stale; holding the manual offset",
                    throttle_duration_sec=2.0,
                )
                command = self.offset.tolist()
            else:
                feedback = self._joint_vector_locked()
                if feedback is None:
                    command = self.offset.tolist()
                elif self.motion_mode == MODE_FORCE:
                    self.offset = np.zeros(6)
                    command = self.offset.tolist()
                elif self.motion_mode == MODE_FLOAT:
                    # Chase the measurement so the servo's position error, and
                    # so its holding torque, stay near zero. Rate limited: a
                    # hand moves the arm slowly, a fall does not, and refusing
                    # to chase the fast case is what makes the servo catch the
                    # arm rather than follow it to the bench.
                    reference = self.arm_reference or feedback
                    # Bounded by the arm itself: it chases a pose the arm is
                    # already measured to be in, so it cannot ask for one the
                    # joints cannot reach.
                    wanted = np.asarray(feedback) - np.asarray(reference)
                    self.offset = np.asarray(step_toward(
                        self.offset.tolist(), wanted.tolist(),
                        self._float_follow_velocity() * dt,
                    ))
                    command = self.offset.tolist()
                else:
                    fresh = bool(
                        self.last_control_cmd
                        and now - self.last_control_cmd <= self.control_cmd_timeout
                    )
                    if fresh and dt > 0.0:
                        self._apply_offset_locked(
                            feedback,
                            self._offset_delta_locked(
                                feedback, self.control_velocity, dt
                            ),
                        )
                    elif not fresh:
                        self._custom_z_target = None
                    command = self.offset.tolist()
        self.offset_pub.publish(Float64MultiArray(data=command))

    def destroy_node(self):
        # Leave the operator channel at zero: whatever starts next must not
        # inherit an offset nobody is holding a stick for.
        try:
            if rclpy.ok():
                self.offset_pub.publish(Float64MultiArray(data=[0.0] * 6))
        except Exception:
            pass
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = OM6DOFController()
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
