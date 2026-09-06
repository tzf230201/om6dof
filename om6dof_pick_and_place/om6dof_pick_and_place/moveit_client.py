"""Thin synchronous wrapper around the MoveGroup and GripperCommand actions.

`pymoveit2` is not available on PyPI for our Python/arch combination and the
official `moveit_py` bindings only ship from Iron onward. So we drive
move_group directly through the standard `moveit_msgs/action/MoveGroup`
goal + `control_msgs/action/GripperCommand` goal.

This wrapper is intentionally small — enough to:
  - send a *named target* goal (group_state from the SRDF, e.g. "home", "rest")
  - send a *joint values* goal (list of 6 floats for the arm)
  - send a *position* or *pose* goal (geometry_msgs/Pose in `world` frame)
  - command the gripper to an absolute prismatic position

Async control flow: every method blocks until the action finishes; result is
returned as a bool (success/failure) so the caller's state machine can branch.
"""

from __future__ import annotations

import copy
import math
import threading
import time
from dataclasses import dataclass
from typing import Iterable, List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Pose, Quaternion
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rcl_interfaces.srv import GetParameters
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    DisplayTrajectory,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    WorkspaceParameters,
)
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


DEFAULT_ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


@dataclass
class JointTarget:
    """A named pose (from SRDF group_state) or explicit joint values."""
    name: Optional[str] = None
    values: Optional[List[float]] = None


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class MoveItClient:
    """Synchronous helpers driving move_group and the gripper action."""

    def __init__(
        self,
        node: Node,
        arm_group: str = "arm",
        gripper_group: str = "gripper",
        ee_link: str = "end_effector_link",
        reference_frame: str = "world",
        arm_joint_names: Iterable[str] = DEFAULT_ARM_JOINTS,
        planning_time: float = 5.0,
        num_planning_attempts: int = 10,
        max_velocity_scaling: float = 0.3,
        max_acceleration_scaling: float = 0.3,
        position_tolerance: float = 0.03,
        orientation_tolerance: float = 0.20,
        joint_tolerance: float = 0.01,
        arm_action_name: str = "/arm_controller/follow_joint_trajectory",
        gripper_action_name: str = "/gripper_controller/gripper_cmd",
        display_trajectory_topic: str = "/display_planned_path",
        display_model_id: str = "om6dof",
    ) -> None:
        self.node = node
        self.arm_group = arm_group
        self.gripper_group = gripper_group
        self.ee_link = ee_link
        self.reference_frame = reference_frame
        self.arm_joint_names = list(arm_joint_names)
        self.planning_time = planning_time
        self.num_planning_attempts = num_planning_attempts
        self.vel_scale = max_velocity_scaling
        self.acc_scale = max_acceleration_scaling
        self.pos_tol = position_tolerance
        self.ori_tol = orientation_tolerance
        self.joint_tol = joint_tolerance

        self._move_client = ActionClient(node, MoveGroup, "/move_action")
        self._arm_controller_client = ActionClient(
            node, FollowJointTrajectory, arm_action_name)
        self._grip_client = ActionClient(node, GripperCommand, gripper_action_name)
        self._arm_cancel_client = node.create_client(
            CancelGoal, f"{arm_action_name.rstrip('/')}/_action/cancel_goal")
        self._gripper_cancel_client = node.create_client(
            CancelGoal, f"{gripper_action_name.rstrip('/')}/_action/cancel_goal")
        self._scene_client = node.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self._move_group_params_client = node.create_client(
            GetParameters, "/move_group/get_parameters"
        )
        # MoveGroup publishes each planning result separately.  This publisher
        # is used only when a caller explicitly captures a complete dry-run
        # chain, allowing RViz to replay pregrasp -> grasp -> lift/retreat as a
        # single coherent DisplayTrajectory message.
        self._display_trajectory_pub = node.create_publisher(
            DisplayTrajectory, display_trajectory_topic, 10)
        self._display_model_id = str(display_model_id)
        self._plan_only_display_active = False
        self._plan_only_display_start = None
        self._plan_only_display_segments = []
        self._goal_lock = threading.Lock()
        self._current_move_goal = None
        self._current_move_result_future = None
        self._move_goal_pending = False
        self._pending_move_send_future = None
        self._cancel_move_on_accept = False
        self._physical_move_goal = False
        self._current_gripper_goal = None
        self._current_gripper_result_future = None
        self._gripper_goal_pending = False
        self._pending_gripper_send_future = None
        self._cancel_gripper_on_accept = False
        self._physical_gripper_goal = False
        self._controller_cancel_futures = {}
        self._controller_cancel_last_sent = {}
        self._plan_only_start_state = None
        self._motion_faulted = False
        self._commands_stopped = False

    # ---------------- lifecycle ----------------
    def wait_for_move_server(self, timeout_sec: float = 30.0) -> bool:
        if not self._move_client.wait_for_server(timeout_sec=timeout_sec):
            self.node.get_logger().error("MoveGroup action server not available")
            return False
        return True

    def wait_for_servers(self, timeout_sec: float = 30.0) -> bool:
        if not self.wait_for_move_server(timeout_sec=timeout_sec):
            return False
        if not self._arm_controller_client.wait_for_server(
                timeout_sec=timeout_sec):
            self.node.get_logger().error(
                "Arm FollowJointTrajectory action server not available")
            return False
        if not self._grip_client.wait_for_server(timeout_sec=timeout_sec):
            self.node.get_logger().error("GripperCommand action server not available")
            return False
        return True

    def wait_for_scene_server(self, timeout_sec: float = 10.0) -> bool:
        if not self._scene_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().error(
                "ApplyPlanningScene service not available"
            )
            return False
        return True

    def verify_planning_pipeline(self, pipeline_id: str,
                                 timeout_sec: float = 10.0) -> bool:
        """Fail closed unless MoveGroup advertises the required pipeline."""
        if not self._move_group_params_client.wait_for_service(
                timeout_sec=timeout_sec):
            self.node.get_logger().error(
                "MoveGroup parameter service not available")
            return False
        request = GetParameters.Request()
        request.names = ["planning_pipelines"]
        future = self._move_group_params_client.call_async(request)
        if not self._wait_future(future, timeout_sec=timeout_sec):
            self.node.get_logger().error(
                "Timed out querying MoveGroup planning pipelines")
            return False
        response = future.result()
        pipelines = (list(response.values[0].string_array_value)
                     if response is not None and response.values else [])
        if pipeline_id not in pipelines:
            self.node.get_logger().error(
                f"MoveGroup does not advertise required pipeline "
                f"'{pipeline_id}' (available: {pipelines})")
            return False
        return True

    # ---------------- helpers ----------------
    def _workspace(self) -> WorkspaceParameters:
        ws = WorkspaceParameters()
        ws.header.frame_id = self.reference_frame
        ws.min_corner.x = -1.0
        ws.min_corner.y = -1.0
        ws.min_corner.z = -0.5
        ws.max_corner.x = 1.0
        ws.max_corner.y = 1.0
        ws.max_corner.z = 1.5
        return ws

    def _new_plan_request(
        self,
        group: str,
        pipeline_id: str = "",
        planner_id: str = "",
    ) -> MotionPlanRequest:
        req = MotionPlanRequest()
        req.workspace_parameters = self._workspace()
        req.group_name = group
        req.pipeline_id = str(pipeline_id)
        req.planner_id = str(planner_id)
        req.num_planning_attempts = self.num_planning_attempts
        req.allowed_planning_time = self.planning_time
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale
        # Empty + is_diff means "use the current monitored state". Leaving an
        # entirely default RobotState makes MoveIt print conversion errors even
        # though it eventually falls back to the current state.
        req.start_state.is_diff = True
        return req

    def _wait_future(self, future: Future, timeout_sec: float = 60.0) -> bool:
        """Block until `future` is done. Must be called from a thread that is
        NOT the executor's main spin thread. Works under MultiThreadedExecutor
        because other threads keep processing rclpy callbacks (action client
        responses) while we sleep here."""
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                return False
            time.sleep(0.02)
        return future.done()

    def _send_move_goal(
        self, plan_req: MotionPlanRequest, *, plan_only: bool = False
    ) -> bool:
        if plan_only and self._plan_only_start_state is not None:
            plan_req.start_state = copy.deepcopy(self._plan_only_start_state)
        elif not plan_only:
            self._plan_only_start_state = None
        goal = MoveGroup.Goal()
        goal.request = plan_req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = bool(plan_only)
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.replan_attempts = 1

        self.node.get_logger().info(
            f"{'Planning only' if plan_only else 'Planning/executing'} "
            f"{plan_req.group_name} (pipeline={plan_req.pipeline_id or 'default'} "
            f"planner={plan_req.planner_id or 'default'} "
            f"vel={self.vel_scale} acc={self.acc_scale})"
        )
        # Check the stop latch and transmit while holding the same lock used by
        # cancel_current_goal(). Thus no goal can be sent after /stop returns.
        with self._goal_lock:
            if self._motion_faulted:
                self.node.get_logger().error(
                    "Arm goals are locked after an action timeout; verify the "
                    "robot state and restart this node before continuing"
                )
                return False
            if self._commands_stopped:
                self.node.get_logger().warn(
                    "Refusing arm goal after stop; start a new sequence first"
                )
                return False
            if self._move_goal_pending or self._current_move_goal is not None:
                self.node.get_logger().error(
                    "Refusing a new arm goal while the previous goal is active"
                )
                return False
            self._move_goal_pending = True
            self._cancel_move_on_accept = False
            self._physical_move_goal = not plan_only
            try:
                send_future = self._move_client.send_goal_async(goal)
            except Exception:  # noqa: BLE001 - preserve client exception
                self._move_goal_pending = False
                self._pending_move_send_future = None
                self._physical_move_goal = False
                raise
            self._pending_move_send_future = send_future
        if not self._wait_future(send_future, timeout_sec=10.0):
            with self._goal_lock:
                cancel_on_accept = self._cancel_move_on_accept
                self._motion_faulted = True
                self._move_goal_pending = False
            if not cancel_on_accept:
                send_future.add_done_callback(
                    lambda future: self._cancel_late_goal(
                        future, "MoveGroup"))
            self.node.get_logger().error(
                "Timed out waiting for MoveGroup goal acceptance; the goal is "
                "in-doubt and further commands are locked"
            )
            return False
        handle = send_future.result()
        with self._goal_lock:
            cancel_on_accept = self._cancel_move_on_accept
            self._move_goal_pending = False
            if not cancel_on_accept:
                self._pending_move_send_future = None
            self._cancel_move_on_accept = False
            accepted = handle is not None and handle.accepted
            # Make accepted -> active an atomic handoff. Otherwise /stop can
            # arrive after pending is cleared but before the handle is visible.
            if accepted and not cancel_on_accept:
                self._current_move_goal = handle
            elif not accepted:
                self._physical_move_goal = False
        if not accepted:
            self.node.get_logger().error("MoveGroup goal rejected")
            return False

        if cancel_on_accept:
            self.node.get_logger().warn(
                "MoveGroup goal was accepted after cancellation was requested"
            )
            return False

        result_future = handle.get_result_async()
        with self._goal_lock:
            self._current_move_result_future = result_future
        if not self._wait_future(result_future, timeout_sec=120.0):
            self.node.get_logger().error(
                "Timed out waiting for MoveGroup; requesting cancellation and "
                "locking further arm/gripper goals until this node is restarted"
            )
            with self._goal_lock:
                self._motion_faulted = True
            if self.physical_action_in_flight:
                self.cancel_controller_goals()
            try:
                cancel_future = handle.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001 - direct fallback is active
                self.node.get_logger().error(
                    f"MoveGroup cancellation request raised: {exc}")
            else:
                if not self._wait_future(cancel_future, timeout_sec=5.0):
                    self.node.get_logger().error(
                        "MoveGroup cancellation acknowledgement timed out"
                    )
            # Do not overlap a recovery goal with a trajectory that may still
            # be decelerating. Waiting briefly improves diagnostics, but the
            # fault latch remains set even if a late terminal result arrives.
            self._wait_future(result_future, timeout_sec=10.0)
            if result_future.done():
                try:
                    late_result = result_future.result()
                except Exception as exc:  # noqa: BLE001 - state stays uncertain
                    late_result = None
                    self.node.get_logger().error(
                        f"MoveGroup terminal result raised: {exc}")
                self._finalize_action_result(
                    "MoveGroup", handle, late_result)
            return False
        try:
            result = result_future.result()
        except Exception as exc:  # noqa: BLE001 - preserve uncertain ownership
            result = None
            self.node.get_logger().error(
                f"MoveGroup result raised: {exc}")
        if not self._finalize_action_result("MoveGroup", handle, result):
            return False
        # MoveItErrorCodes.SUCCESS = 1
        code = result.result.error_code.val
        ok = (result.status == GoalStatus.STATUS_SUCCEEDED and code == 1)
        if not ok:
            self.node.get_logger().error(
                f"MoveGroup failed, action status {result.status}, "
                f"error code {code}"
            )
        elif plan_only:
            self._remember_planned_end_state(result.result)
            self._record_plan_only_display(result.result)
        return ok

    def _remember_planned_end_state(self, move_result) -> None:
        """Use one plan's endpoint as the next plan-only request's start.

        Without this, a dry-run would plan every segment from the live robot
        state and could not meaningfully validate pregrasp -> grasp -> lift.
        """
        state = copy.deepcopy(move_result.trajectory_start)
        trajectory = move_result.planned_trajectory.joint_trajectory
        if trajectory.points:
            final = trajectory.points[-1]
            positions = dict(zip(trajectory.joint_names, final.positions))
            names = list(state.joint_state.name)
            values = list(state.joint_state.position)
            index = {name: idx for idx, name in enumerate(names)}
            for name, value in positions.items():
                if name in index:
                    values[index[name]] = float(value)
                else:
                    names.append(name)
                    values.append(float(value))
            state.joint_state.name = names
            state.joint_state.position = values
            state.joint_state.velocity = []
            state.joint_state.effort = []
        state.is_diff = False
        self._plan_only_start_state = state

    def reset_plan_only_state(self) -> None:
        """Start a new plan-only chain from MoveIt's current monitored state."""
        self._plan_only_start_state = None

    def begin_plan_only_display(self) -> None:
        """Begin collecting successful dry-run segments for one RViz replay.

        Capture is deliberately explicit.  Physical-mode prevalidation also
        uses ``plan_only=True`` internally, but those speculative plans must
        not be presented as the chosen pick trajectory.
        """
        self._plan_only_display_active = True
        self._plan_only_display_start = None
        self._plan_only_display_segments = []

    def _record_plan_only_display(self, move_result) -> None:
        """Append one successful MoveGroup result to an active capture."""
        if not bool(getattr(self, "_plan_only_display_active", False)):
            return
        trajectory = move_result.planned_trajectory
        joint_points = trajectory.joint_trajectory.points
        multi_dof_points = trajectory.multi_dof_joint_trajectory.points
        if not joint_points and not multi_dof_points:
            return
        if self._plan_only_display_start is None:
            self._plan_only_display_start = copy.deepcopy(
                move_result.trajectory_start)
        self._plan_only_display_segments.append(copy.deepcopy(trajectory))

    def publish_plan_only_display(self) -> bool:
        """Publish the captured chain once, then close the capture.

        A visualization error is fail-soft: the already validated planning
        result remains valid and no controller behavior changes.
        """
        active = bool(getattr(self, "_plan_only_display_active", False))
        start = getattr(self, "_plan_only_display_start", None)
        segments = list(getattr(
            self, "_plan_only_display_segments", []))
        self.discard_plan_only_display()
        if not active or start is None or not segments:
            return False
        message = DisplayTrajectory()
        message.model_id = str(getattr(
            self, "_display_model_id", "om6dof"))
        message.trajectory_start = copy.deepcopy(start)
        message.trajectory = copy.deepcopy(segments)
        try:
            self._display_trajectory_pub.publish(message)
        except Exception as exc:  # noqa: BLE001 - visualization is optional
            self.node.get_logger().warn(
                "Could not publish combined plan-only trajectory "
                f"(motion unaffected): {exc}")
            return False
        self.node.get_logger().info(
            f"Published combined plan-only trajectory with "
            f"{len(message.trajectory)} segment(s)")
        return True

    def discard_plan_only_display(self) -> None:
        """Drop an incomplete/aborted visualization capture."""
        self._plan_only_display_active = False
        self._plan_only_display_start = None
        self._plan_only_display_segments = []

    def set_plan_only_joint_state(self, joint_names: Iterable[str],
                                  value: float) -> bool:
        """Update non-arm joints in the cached endpoint of a dry-run chain."""
        if self._plan_only_start_state is None:
            self.node.get_logger().error(
                "Cannot update plan-only joints before the first arm plan"
            )
            return False
        state = self._plan_only_start_state
        names = list(state.joint_state.name)
        values = list(state.joint_state.position)
        index = {name: idx for idx, name in enumerate(names)}
        for name in joint_names:
            name = str(name)
            if name in index:
                values[index[name]] = float(value)
            else:
                index[name] = len(names)
                names.append(name)
                values.append(float(value))
        state.joint_state.name = names
        state.joint_state.position = values
        state.joint_state.velocity = []
        state.joint_state.effort = []
        return True

    @property
    def motion_faulted(self) -> bool:
        """Whether an arm timeout made continued automatic motion unsafe."""
        with self._goal_lock:
            return self._motion_faulted

    @property
    def action_in_flight(self) -> bool:
        """Whether an action can still accept, execute, or return a result."""
        with self._goal_lock:
            return bool(
                self._move_goal_pending
                or self._gripper_goal_pending
                or self._current_move_goal is not None
                or self._current_gripper_goal is not None
                or self._pending_move_send_future is not None
                or self._pending_gripper_send_future is not None
                or any(self._controller_cancel_futures.values())
            )

    @property
    def physical_action_in_flight(self) -> bool:
        """Whether this client can still own a physical controller goal."""
        with self._goal_lock:
            return self._physical_action_in_flight_unlocked()

    @property
    def controller_cancel_in_flight(self) -> bool:
        """Whether a direct physical-controller cancel request lacks an ACK."""
        with self._goal_lock:
            return any(self._controller_cancel_futures.values())

    def _physical_action_in_flight_unlocked(self) -> bool:
        move_in_flight = (
            self._move_goal_pending
            or self._current_move_goal is not None
            or self._pending_move_send_future is not None
        )
        gripper_in_flight = (
            self._gripper_goal_pending
            or self._current_gripper_goal is not None
            or self._pending_gripper_send_future is not None
        )
        return bool(
            (self._physical_move_goal and move_in_flight)
            or (self._physical_gripper_goal and gripper_in_flight)
        )

    def begin_sequence(self) -> bool:
        """Clear an operator stop latch for one explicitly requested run."""
        with self._goal_lock:
            if self._motion_faulted:
                return False
            if (self._move_goal_pending or self._gripper_goal_pending
                    or self._current_move_goal is not None
                    or self._current_gripper_goal is not None
                    or self._pending_move_send_future is not None
                    or self._pending_gripper_send_future is not None
                    or any(self._controller_cancel_futures.values())):
                return False
            self._commands_stopped = False
            return True

    def _cancel_late_goal(self, future: Future, label: str) -> None:
        """Best-effort cancellation when goal acceptance arrived after timeout."""
        accepted = False
        needs_controller_stop = False
        try:
            handle = future.result()
            if handle is not None and handle.accepted:
                accepted = True
                with self._goal_lock:
                    if label == "MoveGroup":
                        self._current_move_goal = handle
                        needs_controller_stop = self._physical_move_goal
                    else:
                        self._current_gripper_goal = handle
                        needs_controller_stop = self._physical_gripper_goal
                if needs_controller_stop:
                    self.cancel_controller_goals()
                try:
                    handle.cancel_goal_async()
                except Exception as exc:  # noqa: BLE001 - fallback already sent
                    self.node.get_logger().error(
                        f"Late {label} cancellation request raised: {exc}")
                try:
                    result_future = handle.get_result_async()
                except Exception as exc:  # noqa: BLE001 - retain ownership
                    with self._goal_lock:
                        self._motion_faulted = True
                    self.node.get_logger().error(
                        f"Late {label} result request raised: {exc}; "
                        "ownership remains locked")
                else:
                    with self._goal_lock:
                        if label == "MoveGroup":
                            self._current_move_result_future = result_future
                        else:
                            self._current_gripper_result_future = result_future
                    result_future.add_done_callback(
                        lambda done, action_label=label, goal_handle=handle:
                        self._clear_late_goal(
                            action_label, goal_handle, done))
                self.node.get_logger().warn(
                    f"Late {label} acceptance arrived; cancellation requested"
                )
        except Exception as exc:  # noqa: BLE001 - asynchronous best effort
            self.node.get_logger().error(
                f"Could not cancel late {label} goal: {exc}"
            )
        finally:
            with self._goal_lock:
                if (label == "MoveGroup"
                        and self._pending_move_send_future is future):
                    self._pending_move_send_future = None
                    self._move_goal_pending = False
                    if not accepted:
                        self._physical_move_goal = False
                elif (label == "GripperCommand"
                      and self._pending_gripper_send_future is future):
                    self._pending_gripper_send_future = None
                    self._gripper_goal_pending = False
                    if not accepted:
                        self._physical_gripper_goal = False

    def _clear_late_goal(self, label: str, handle,
                         result_future: Future) -> None:
        try:
            result = result_future.result()
        except Exception as exc:  # noqa: BLE001 - state remains uncertain
            result = None
            self.node.get_logger().error(
                f"Late {label} result raised: {exc}")
        self._finalize_action_result(label, handle, result)

    def _finalize_action_result(self, label: str, handle, result) -> bool:
        """Release ownership only after the action reports a terminal status."""
        status = (GoalStatus.STATUS_UNKNOWN if result is None
                  else int(getattr(result, "status", GoalStatus.STATUS_UNKNOWN)))
        terminal = status in {
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_ABORTED,
        }
        still_current = False
        needs_controller_stop = False
        with self._goal_lock:
            if label == "MoveGroup" and self._current_move_goal is handle:
                still_current = True
                if terminal:
                    self._current_move_goal = None
                    self._current_move_result_future = None
                    self._physical_move_goal = False
                else:
                    self._motion_faulted = True
                    needs_controller_stop = self._physical_move_goal
            elif (label == "GripperCommand"
                  and self._current_gripper_goal is handle):
                still_current = True
                if terminal:
                    self._current_gripper_goal = None
                    self._current_gripper_result_future = None
                    self._physical_gripper_goal = False
                else:
                    self._motion_faulted = True
                    needs_controller_stop = self._physical_gripper_goal

        if still_current and not terminal:
            self.node.get_logger().error(
                f"{label} returned non-terminal/unknown action status {status}; "
                "ownership remains locked")
            if needs_controller_stop:
                self.cancel_controller_goals()
            try:
                handle.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001 - direct fallback may be active
                self.node.get_logger().error(
                    f"{label} re-cancellation request raised: {exc}")
        return terminal

    def cancel_current_goal(self) -> bool:
        """Request cancellation of any in-flight arm or gripper action.

        Cancellation is asynchronous: callers should still treat the current
        robot state as unknown until the action result arrives.
        """
        requested = False
        pending_futures = []
        with self._goal_lock:
            self._commands_stopped = True
            needs_controller_stop = self._physical_action_in_flight_unlocked()
            handles = (self._current_move_goal, self._current_gripper_goal)
            if self._move_goal_pending:
                self._cancel_move_on_accept = True
                self._motion_faulted = True
                requested = True
                if self._pending_move_send_future is not None:
                    pending_futures.append(
                        (self._pending_move_send_future, "MoveGroup"))
            if self._gripper_goal_pending:
                self._cancel_gripper_on_accept = True
                self._motion_faulted = True
                requested = True
                if self._pending_gripper_send_future is not None:
                    pending_futures.append((
                        self._pending_gripper_send_future,
                        "GripperCommand"))
        for future, label in pending_futures:
            future.add_done_callback(
                lambda done, action_label=label: self._cancel_late_goal(
                    done, action_label))
        if needs_controller_stop:
            self.cancel_controller_goals()
            requested = True
        for handle in handles:
            if handle is not None:
                try:
                    handle.cancel_goal_async()
                except Exception as exc:  # noqa: BLE001 - direct fallback sent
                    self.node.get_logger().error(
                        f"Action cancellation request raised: {exc}")
                requested = True
        if requested:
            self.node.get_logger().warn("Cancellation requested for active action")
        return requested

    def cancel_controller_goals(self) -> int:
        """Directly cancel all arm/gripper controller goals as a stop fallback.

        MoveGroup normally propagates cancellation to the trajectory controller.
        Sending the standard cancel-all request directly to each controller
        currently owned by this client prevents a stuck or disappearing
        MoveGroup process from leaving a physical trajectory running after
        this node is stopped. Plan-only and unrelated controller endpoints are
        deliberately excluded.
        """
        pending_callbacks = []
        now = time.monotonic()
        with self._goal_lock:
            move_owned = bool(
                self._physical_move_goal
                and (self._move_goal_pending
                     or self._current_move_goal is not None
                     or self._pending_move_send_future is not None))
            gripper_owned = bool(
                self._physical_gripper_goal
                and (self._gripper_goal_pending
                     or self._current_gripper_goal is not None
                     or self._pending_gripper_send_future is not None))
            targets = []
            if move_owned:
                targets.append(("arm controller", self._arm_cancel_client))
            if gripper_owned:
                targets.append((
                    "gripper controller", self._gripper_cancel_client))

            for label, client in targets:
                outstanding = self._controller_cancel_futures.get(label, [])
                last_sent = self._controller_cancel_last_sent.get(label, 0.0)
                # A response that acknowledged zero goals can be followed by a
                # late controller goal. Retry per endpoint, without allowing a
                # stuck arm request to suppress gripper cancellation (or vice
                # versa). Keep every outstanding request tracked so a new run
                # cannot start before an old cancel request is accounted for.
                if outstanding and now - last_sent < 0.5:
                    continue
                if not client.service_is_ready():
                    self.node.get_logger().error(
                        f"{label} cancel service is unavailable")
                    continue
                try:
                    future = client.call_async(CancelGoal.Request())
                except Exception as exc:  # noqa: BLE001 - safety best effort
                    self.node.get_logger().error(
                        f"could not request {label} cancellation: {exc}")
                    continue
                self._controller_cancel_futures.setdefault(label, []).append(
                    future)
                self._controller_cancel_last_sent[label] = now
                pending_callbacks.append((future, label))
        for future, label in pending_callbacks:
            future.add_done_callback(
                lambda done, action_label=label:
                self._controller_cancel_done(done, action_label))
        requests = len(pending_callbacks)
        if requests:
            self.node.get_logger().warn(
                "Direct cancellation requested from physical controllers")
        return requests

    def _controller_cancel_done(self, future: Future, label: str) -> None:
        try:
            response = future.result()
            if (response is None
                    or response.return_code != CancelGoal.Response.ERROR_NONE):
                code = None if response is None else response.return_code
                self.node.get_logger().error(
                    f"{label} cancel-all request failed (code={code})")
            else:
                self.node.get_logger().info(
                    f"{label} cancel-all acknowledged "
                    f"({len(response.goals_canceling)} goal(s))")
        except Exception as exc:  # noqa: BLE001 - asynchronous safety report
            self.node.get_logger().error(
                f"{label} cancel-all request raised: {exc}")
        finally:
            with self._goal_lock:
                outstanding = self._controller_cancel_futures.get(label, [])
                outstanding = [item for item in outstanding
                               if item is not future]
                if outstanding:
                    self._controller_cancel_futures[label] = outstanding
                else:
                    self._controller_cancel_futures.pop(label, None)
                    self._controller_cancel_last_sent.pop(label, None)

    # ---------------- public API ----------------
    def move_to_named_pose(
        self,
        name: str,
        named_poses: Optional[dict] = None,
        group: Optional[str] = None,
    ) -> bool:
        """Move to an SRDF group_state by name. Caller must supply the
        `named_poses` dict {state_name: {joint_name: value, ...}} because the
        MoveGroup action server does NOT resolve SRDF names server-side."""
        if not named_poses or name not in named_poses:
            self.node.get_logger().error(
                f"Named pose '{name}' not in supplied named_poses dict "
                f"({list((named_poses or {}).keys())})"
            )
            return False
        joint_map = named_poses[name]
        names = list(joint_map.keys())
        values = [float(joint_map[n]) for n in names]
        return self.move_to_joint_values(values, joint_names=names, group=group)

    def move_to_joint_values(
        self,
        values: List[float],
        joint_names: Optional[List[str]] = None,
        group: Optional[str] = None,
        plan_only: bool = False,
        pipeline_id: str = "",
        planner_id: str = "",
    ) -> bool:
        group = group or self.arm_group
        names = joint_names if joint_names is not None else self.arm_joint_names
        if len(values) != len(names):
            raise ValueError(
                f"values has {len(values)} entries but {len(names)} joint names"
            )
        req = self._new_plan_request(
            group, pipeline_id=pipeline_id, planner_id=planner_id
        )
        gc = Constraints()
        for n, v in zip(names, values):
            jc = JointConstraint()
            jc.joint_name = n
            jc.position = float(v)
            jc.tolerance_above = self.joint_tol
            jc.tolerance_below = self.joint_tol
            jc.weight = 1.0
            gc.joint_constraints.append(jc)
        req.goal_constraints.append(gc)
        return self._send_move_goal(req, plan_only=plan_only)

    def move_to_pose(
        self,
        pose: Pose,
        ee_link: Optional[str] = None,
        position_tolerance: Optional[float] = None,
        orientation_tolerance: Optional[float] = None,
        plan_only: bool = False,
        pipeline_id: str = "",
        planner_id: str = "",
    ) -> bool:
        link = ee_link or self.ee_link
        pos_tol = self.pos_tol if position_tolerance is None else float(position_tolerance)
        ori_tol = self.ori_tol if orientation_tolerance is None else float(orientation_tolerance)
        req = self._new_plan_request(
            self.arm_group, pipeline_id=pipeline_id, planner_id=planner_id
        )
        gc = Constraints()

        # position constraint = small sphere at the target position
        pc = PositionConstraint()
        pc.header.frame_id = self.reference_frame
        pc.link_name = link
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [pos_tol]
        pc.constraint_region.primitives.append(sphere)
        pc.constraint_region.primitive_poses.append(pose)
        pc.weight = 1.0
        gc.position_constraints.append(pc)

        # orientation constraint
        oc = OrientationConstraint()
        oc.header.frame_id = self.reference_frame
        oc.link_name = link
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = ori_tol
        oc.absolute_y_axis_tolerance = ori_tol
        oc.absolute_z_axis_tolerance = ori_tol
        oc.weight = 1.0
        gc.orientation_constraints.append(oc)

        req.goal_constraints.append(gc)
        return self._send_move_goal(req, plan_only=plan_only)

    def move_linear_to_pose(
        self,
        pose: Pose,
        ee_link: Optional[str] = None,
        position_tolerance: Optional[float] = None,
        orientation_tolerance: Optional[float] = None,
        plan_only: bool = False,
    ) -> bool:
        """Plan a straight Cartesian tool path with the Pilz LIN planner."""
        return self.move_to_pose(
            pose,
            ee_link=ee_link,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
            plan_only=plan_only,
            pipeline_id="pilz_industrial_motion_planner",
            planner_id="LIN",
        )

    def move_to_position(
        self,
        pose: Pose,
        ee_link: Optional[str] = None,
        position_tolerance: Optional[float] = None,
        plan_only: bool = False,
    ) -> bool:
        """Move the tool point without constraining its orientation.

        This is useful for a staged grasp: first place the gripper in front
        of the object, then issue a second pose goal that aligns orientation
        at exactly the same position.
        """
        link = ee_link or self.ee_link
        pos_tol = self.pos_tol if position_tolerance is None else float(position_tolerance)
        req = self._new_plan_request(self.arm_group)
        gc = Constraints()

        pc = PositionConstraint()
        pc.header.frame_id = self.reference_frame
        pc.link_name = link
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [pos_tol]
        pc.constraint_region.primitives.append(sphere)
        pc.constraint_region.primitive_poses.append(pose)
        pc.weight = 1.0
        gc.position_constraints.append(pc)

        req.goal_constraints.append(gc)
        return self._send_move_goal(req, plan_only=plan_only)

    def move_to_xyz_rpy(
        self,
        x: float, y: float, z: float,
        roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
        position_tolerance: Optional[float] = None,
        orientation_tolerance: Optional[float] = None,
        plan_only: bool = False,
    ) -> bool:
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = z
        p.orientation = quat_from_rpy(roll, pitch, yaw)
        return self.move_to_pose(
            p,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
            plan_only=plan_only,
        )

    def apply_collision_box(
        self,
        object_id: str,
        size_xyz: Iterable[float],
        pose: Pose,
        timeout_sec: float = 10.0,
    ) -> bool:
        """Add or replace one axis-aligned box in MoveIt's planning scene."""
        size = [float(value) for value in size_xyz]
        if len(size) != 3 or any(value <= 0.0 for value in size):
            raise ValueError("collision box size must contain three positive values")
        if not self.wait_for_scene_server(timeout_sec=timeout_sec):
            return False

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = size
        obj = CollisionObject()
        obj.header.frame_id = self.reference_frame
        obj.id = str(object_id)
        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.world.collision_objects.append(obj)
        future = self._scene_client.call_async(request)
        if not self._wait_future(future, timeout_sec=timeout_sec):
            self.node.get_logger().error(
                f"Timed out applying collision object '{object_id}'"
            )
            return False
        response = future.result()
        ok = response is not None and bool(response.success)
        if not ok:
            self.node.get_logger().error(
                f"MoveIt rejected collision object '{object_id}'"
            )
        return ok

    def set_gripper(self, position: float, max_effort: float = 5.0) -> bool:
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
        self.node.get_logger().info(f"Gripper → {position:+.4f}")
        with self._goal_lock:
            if self._motion_faulted:
                self.node.get_logger().error(
                    "Refusing gripper goal while action state is uncertain "
                    "after timeout"
                )
                return False
            if self._commands_stopped:
                self.node.get_logger().warn(
                    "Refusing gripper goal after stop; start a new sequence first"
                )
                return False
            if (self._gripper_goal_pending
                    or self._current_gripper_goal is not None):
                self.node.get_logger().error(
                    "Refusing a new gripper goal while the previous goal is active"
                )
                return False
            self._gripper_goal_pending = True
            self._cancel_gripper_on_accept = False
            self._physical_gripper_goal = True
            try:
                send_future = self._grip_client.send_goal_async(goal)
            except Exception:  # noqa: BLE001 - preserve client exception
                self._gripper_goal_pending = False
                self._pending_gripper_send_future = None
                self._physical_gripper_goal = False
                raise
            self._pending_gripper_send_future = send_future
        if not self._wait_future(send_future, timeout_sec=5.0):
            with self._goal_lock:
                cancel_on_accept = self._cancel_gripper_on_accept
                self._motion_faulted = True
                self._gripper_goal_pending = False
            if not cancel_on_accept:
                send_future.add_done_callback(
                    lambda future: self._cancel_late_goal(
                        future, "GripperCommand"))
            self.node.get_logger().error(
                "Timed out waiting for Gripper goal acceptance; the goal is "
                "in-doubt and further commands are locked"
            )
            return False
        handle = send_future.result()
        with self._goal_lock:
            cancel_on_accept = self._cancel_gripper_on_accept
            self._gripper_goal_pending = False
            if not cancel_on_accept:
                self._pending_gripper_send_future = None
            self._cancel_gripper_on_accept = False
            accepted = handle is not None and handle.accepted
            # Keep the accepted -> active handoff atomic with /stop.
            if accepted and not cancel_on_accept:
                self._current_gripper_goal = handle
            elif not accepted:
                self._physical_gripper_goal = False
        if not accepted:
            self.node.get_logger().error("GripperCommand goal rejected")
            return False
        if cancel_on_accept:
            self.node.get_logger().warn(
                "Gripper goal was accepted after cancellation was requested"
            )
            return False
        result_future = handle.get_result_async()
        with self._goal_lock:
            self._current_gripper_result_future = result_future
        if not self._wait_future(result_future, timeout_sec=10.0):
            self.node.get_logger().error(
                "Timed out waiting for Gripper result; requesting cancellation "
                "and locking further goals"
            )
            with self._goal_lock:
                self._motion_faulted = True
            if self.physical_action_in_flight:
                self.cancel_controller_goals()
            try:
                cancel_future = handle.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001 - direct fallback is active
                self.node.get_logger().error(
                    f"Gripper cancellation request raised: {exc}")
            else:
                if not self._wait_future(cancel_future, timeout_sec=5.0):
                    self.node.get_logger().error(
                        "Gripper cancellation acknowledgement timed out"
                    )
            self._wait_future(result_future, timeout_sec=5.0)
            if result_future.done():
                try:
                    late_result = result_future.result()
                except Exception as exc:  # noqa: BLE001 - state stays uncertain
                    late_result = None
                    self.node.get_logger().error(
                        f"Gripper terminal result raised: {exc}")
                self._finalize_action_result(
                    "GripperCommand", handle, late_result)
            return False
        try:
            result = result_future.result()
        except Exception as exc:  # noqa: BLE001 - preserve uncertain ownership
            result = None
            self.node.get_logger().error(
                f"GripperCommand result raised: {exc}")
        if not self._finalize_action_result(
                "GripperCommand", handle, result):
            return False
        # The reached_goal flag isn't strictly required for success (the gripper
        # may stall at an object before reaching the commanded position).
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().error(
                f"GripperCommand failed, action status {result.status}"
            )
            return False
        return True
