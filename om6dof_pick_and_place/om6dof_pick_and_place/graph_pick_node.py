"""Safety-gated pickup coordinator for the semantic DD-GNG graph.

The reachability node remains the sole graph planner.  This node consumes its
exact-collision-validated ``ReachabilityPlan``, binds the selected semantic
environment node to the same-class DD-GNG component, and freezes a reviewable
joint trajectory.  Planning and execution are deliberately separate services.
No motion is sent unless ``execution_enabled`` was set at launch *and* an
operator subsequently calls ``/execute_graph_pick``.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, GripperCommand
from controller_manager_msgs.srv import ListControllers
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from om6dof_dd_gng.msg import EnvironmentGraph, ReachabilityPlan
try:
    from om6dof_dd_gng.srv import ValidateGraspExecution
except ImportError:
    # Legacy move-to-target installs can import this module; pickup requires
    # the rebuilt validator interface and fails closed during initialization.
    ValidateGraspExecution = None
from .additive_control_guard import additive_controller_blockers


# The `shortcut` variants have passed the same exact FCL validation; they only
# contain fewer joint waypoints than the raw roadmap route.
TARGET_PROTECTED_PLAN_REASONS = frozenset({
    "path_ready_exact_validated_target_protected_preview_only",
    "path_ready_exact_validated_target_protected_shortcut_preview_only",
})
VALIDATED_PLAN_REASONS = TARGET_PROTECTED_PLAN_REASONS | frozenset({
    "path_ready_exact_validated_preview_only",
    "path_ready_exact_validated_shortcut_preview_only",
})
GRASP_VALIDATED_PLAN_REASON = "path_ready_exact_validated_grasp_preview_only"


def _duration_seconds(duration) -> float:
    return float(duration.sec) + 1.0e-9 * float(duration.nanosec)


def _diagnostic_level(value) -> int:
    """Normalize ROS uint8 values represented as either int or one byte."""
    if isinstance(value, (bytes, bytearray)):
        if len(value) != 1:
            raise ValueError("diagnostic level must contain exactly one byte")
        return int(value[0])
    return int(value)


def _set_duration(duration, seconds: float) -> None:
    nanoseconds = max(0, int(round(float(seconds) * 1.0e9)))
    duration.sec = nanoseconds // 1_000_000_000
    duration.nanosec = nanoseconds % 1_000_000_000


def _point_tuple(point: Point) -> Tuple[float, float, float]:
    return float(point.x), float(point.y), float(point.z)


def _distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def _normalize(vector: Sequence[float]) -> Optional[Tuple[float, float, float]]:
    if len(vector) != 3 or not all(math.isfinite(float(value)) for value in vector):
        return None
    norm = math.sqrt(sum(float(value) ** 2 for value in vector))
    if norm <= 1.0e-9:
        return None
    return tuple(float(value) / norm for value in vector)


def rotate_vector_by_quaternion(vector: Sequence[float], quaternion) -> Tuple[float, float, float]:
    """Rotate a 3-vector by a geometry_msgs quaternion."""
    x, y, z = (float(value) for value in vector)
    qx = float(quaternion.x)
    qy = float(quaternion.y)
    qz = float(quaternion.z)
    qw = float(quaternion.w)
    qnorm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if qnorm <= 1.0e-9:
        raise ValueError("goal orientation quaternion has zero norm")
    qx, qy, qz, qw = qx / qnorm, qy / qnorm, qz / qnorm, qw / qnorm
    # v' = v + 2 * q.xyz x (q.xyz x v + q.w * v)
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + qy * tz - qz * ty,
        y + qw * ty + qz * tx - qx * tz,
        z + qw * tz + qx * ty - qy * tx,
    )


def approach_alignment(goal_pose, target_position: Sequence[float],
                       tool_axis: Sequence[float]) -> float:
    axis = _normalize(tool_axis)
    if axis is None:
        raise ValueError("tool_approach_axis must be a nonzero finite 3-vector")
    world_axis = _normalize(rotate_vector_by_quaternion(axis, goal_pose.orientation))
    goal = _point_tuple(goal_pose.position)
    direction = _normalize(tuple(float(target_position[i]) - goal[i] for i in range(3)))
    if world_axis is None or direction is None:
        return -1.0
    return sum(world_axis[i] * direction[i] for i in range(3))


def approach_axis_endpoint(goal_pose, target_position: Sequence[float],
                           tool_axis: Sequence[float]) -> Tuple[float, float, float]:
    """End of the RViz approach arrow: physical tool axis, never a world axis."""
    origin = _point_tuple(goal_pose.position)
    axis = _normalize(rotate_vector_by_quaternion(tool_axis, goal_pose.orientation))
    if axis is None:
        raise ValueError("tool_approach_axis cannot be transformed by goal orientation")
    length = _distance(origin, target_position)
    return tuple(origin[index] + length * axis[index] for index in range(3))


def semantic_component(environment: EnvironmentGraph, target_node_id: int):
    """Return the same-class edge-connected component containing target."""
    nodes = {int(node.id): node for node in environment.nodes}
    target = nodes.get(int(target_node_id))
    if target is None:
        raise ValueError("planned target node is absent from current environment graph")
    class_id = int(target.class_id)
    if class_id < 0:
        raise ValueError("planned target node is no longer a selected semantic label")
    adjacency: Dict[int, List[int]] = {}
    for edge in environment.edges:
        source = nodes.get(int(edge.source_id))
        destination = nodes.get(int(edge.target_id))
        if source is None or destination is None:
            continue
        if int(source.class_id) != class_id or int(destination.class_id) != class_id:
            continue
        adjacency.setdefault(int(source.id), []).append(int(destination.id))
        adjacency.setdefault(int(destination.id), []).append(int(source.id))
    pending = [int(target.id)]
    visited = set()
    while pending:
        node_id = pending.pop()
        if node_id in visited:
            continue
        visited.add(node_id)
        pending.extend(adjacency.get(node_id, ()))
    component = [nodes[node_id] for node_id in sorted(visited)]
    weights = [max(1.0e-6, float(node.confidence)) for node in component]
    weight_sum = sum(weights)
    centroid = tuple(
        sum(weights[index] * _point_tuple(node.position)[axis]
            for index, node in enumerate(component)) / weight_sum
        for axis in range(3)
    )
    return class_id, component, centroid


def component_bounding_center(component: Sequence[EnvironmentNode]) -> Tuple[float, float, float]:
    """Physical target reference shared by every node of one object cluster."""
    if not component:
        raise ValueError("semantic component is empty")
    positions = [_point_tuple(node.position) for node in component]
    return tuple(
        (min(position[axis] for position in positions) +
         max(position[axis] for position in positions)) * 0.5
        for axis in range(3))


@dataclass(frozen=True)
class ObjectTrackObservation:
    track_id: int
    class_id: int
    class_name: str
    centroid: Tuple[float, float, float]
    tracked_centroid: Tuple[float, float, float]
    node_ids: Tuple[int, ...]


def parse_object_tracks(data: str) -> List[ObjectTrackObservation]:
    """Parse the versioned object-cluster JSON while rejecting bad geometry."""
    try:
        value = json.loads(data)
    except (TypeError, json.JSONDecodeError):
        return []
    if not isinstance(value, list):
        return []
    tracks = []
    for item in value:
        try:
            centroid_value = item["centroid"]
            tracked_value = item.get("tracked_centroid", centroid_value)
            centroid = tuple(float(centroid_value[key]) for key in ("x", "y", "z"))
            tracked = tuple(float(tracked_value[key]) for key in ("x", "y", "z"))
            track = ObjectTrackObservation(
                track_id=int(item["track_id"]),
                class_id=int(item["class_id"]),
                class_name=str(item["class"]),
                centroid=centroid,
                tracked_centroid=tracked,
                node_ids=tuple(int(node_id) for node_id in item.get("node_ids", [])),
            )
        except (KeyError, TypeError, ValueError, OverflowError):
            continue
        if track.track_id <= 0 or track.class_id < 0:
            continue
        if not all(math.isfinite(value) for value in centroid + tracked):
            continue
        tracks.append(track)
    return tracks


def matching_object_track(history, target_node_id: int, class_id: int,
                          component_centroid: Sequence[float],
                          plan_received: float, now: float, max_age: float,
                          max_association_distance: float,
                          max_future_skew: float = 0.05):
    """Bind a changing DD-GNG node to a stable class-and-centroid track."""
    nearest = None
    nearest_distance = math.inf
    for received, observations in reversed(history):
        if received > plan_received + max_future_skew or now - received > max_age:
            continue
        for observation in observations:
            if observation.class_id != int(class_id):
                continue
            if int(target_node_id) in observation.node_ids:
                return observation
            distance = _distance(observation.centroid, component_centroid)
            if distance < nearest_distance:
                nearest = observation
                nearest_distance = distance
    if nearest is not None and nearest_distance <= float(max_association_distance):
        return nearest
    return None


def matching_environment_snapshot(history, target_node_id: int,
                                  plan_received: float, now: float,
                                  max_age: float,
                                  max_future_skew: float = 0.05):
    """Match an async plan to a recent environment that contains its target."""
    for received, candidate in reversed(history):
        if received > plan_received + max_future_skew or now - received > max_age:
            continue
        if any(int(node.id) == int(target_node_id) and int(node.class_id) >= 0
               for node in candidate.nodes):
            return candidate
    return None


def pickup_target_observation(history, plan, now: float, max_age: float,
                              max_shift: float):
    """Resolve a frozen target without treating a detector ID as physical identity.

    A replacement ID must be the unique nearby same-class observation in three
    consecutive fresh frames spanning 0.1 seconds. Compare raw AND smoothed
    centres with the frozen reference so smoothing cannot hide object motion.
    The planner still validates the physical component centre and the entire
    frozen segment against the newest 3-D scene before any action is sent.
    """
    if not history or now - history[-1][0] > max_age:
        return None, "object_tracks_missing_or_stale"

    def resolve(observations):
        same_class = [item for item in observations
                      if item.class_id == plan.target_class_id]

        def close(item):
            return (_distance(item.centroid, plan.component_centroid) <= max_shift
                    and _distance(item.tracked_centroid, plan.tracked_centroid) <= max_shift)

        original = [item for item in same_class if item.track_id == plan.target_track_id]
        if any(not close(item) for item in original):
            return None, "semantic_object_moved_during_pickup"
        candidates = [item for item in same_class if close(item)]
        if len(original) > 1 or len(candidates) > 1:
            return None, "pickup_target_identity_ambiguous"
        if not candidates:
            return None, "frozen_object_track_not_current"
        return candidates[0], None

    observation, rejection = resolve(history[-1][1])
    if rejection is not None or observation.track_id == plan.target_track_id:
        return observation, rejection
    frames = 0
    newest = history[-1][0]
    previous_received = math.inf
    for received, observations in reversed(history):
        if now - received > max_age or received >= previous_received:
            break
        previous_received = received
        candidate, rejection = resolve(observations)
        if rejection is not None or candidate.track_id != observation.track_id:
            break
        frames += 1
        if frames >= 3 and newest - received >= 0.1:
            return observation, None
    return None, "pickup_target_reacquiring"


def ordered_joint_positions(message: JointState,
                            joint_names: Sequence[str]) -> Optional[List[float]]:
    if len(message.name) != len(message.position):
        return None
    positions = {str(name): float(value)
                 for name, value in zip(message.name, message.position)}
    if any(name not in positions or not math.isfinite(positions[name])
           for name in joint_names):
        return None
    return [positions[name] for name in joint_names]


def validate_trajectory(trajectory: JointTrajectory,
                        joint_names: Sequence[str]) -> Optional[str]:
    if list(trajectory.joint_names) != list(joint_names):
        return "joint_path_names_do_not_match_configured_arm"
    if len(trajectory.points) < 2:
        return "joint_path_has_fewer_than_two_points"
    previous_time = -1.0
    for index, point in enumerate(trajectory.points):
        if len(point.positions) != len(joint_names):
            return f"joint_path_point_{index}_has_wrong_dimension"
        if not all(math.isfinite(float(value)) for value in point.positions):
            return f"joint_path_point_{index}_contains_nonfinite_position"
        point_time = _duration_seconds(point.time_from_start)
        if index == 0 and abs(point_time) > 1.0e-6:
            return "joint_path_first_point_does_not_start_at_zero"
        if point_time <= previous_time:
            return f"joint_path_time_is_not_strictly_increasing_at_{index}"
        previous_time = point_time
    return None


def retime_trajectory(source: JointTrajectory, velocity: float,
                      minimum_segment_time: float) -> JointTrajectory:
    """Copy a graph trajectory and retime it from joint deltas."""
    output = JointTrajectory()
    output.header = copy.deepcopy(source.header)
    output.joint_names = list(source.joint_names)
    elapsed = 0.0
    previous = None
    for source_point in source.points:
        point = JointTrajectoryPoint()
        point.positions = list(source_point.positions)
        if previous is not None:
            maximum_delta = max(abs(float(a) - float(b))
                                for a, b in zip(point.positions, previous))
            elapsed += max(float(minimum_segment_time),
                           maximum_delta / max(float(velocity), 1.0e-6))
        _set_duration(point.time_from_start, elapsed)
        output.points.append(point)
        previous = point.positions
    return output


def reverse_trajectory(source: JointTrajectory, velocity: float,
                       minimum_segment_time: float) -> JointTrajectory:
    reversed_source = JointTrajectory()
    reversed_source.header = copy.deepcopy(source.header)
    reversed_source.joint_names = list(source.joint_names)
    for source_point in reversed(source.points):
        point = JointTrajectoryPoint()
        point.positions = list(source_point.positions)
        reversed_source.points.append(point)
    return retime_trajectory(reversed_source, velocity, minimum_segment_time)


def split_grasp_trajectory(source: JointTrajectory, pregrasp_count: int):
    """Retain the validated graph/insertion boundary and rebase segment time."""
    if pregrasp_count < 2 or pregrasp_count >= len(source.points):
        raise ValueError("grasp_pregrasp_waypoint_count_invalid")
    graph = copy.deepcopy(source)
    graph.points = graph.points[:pregrasp_count]
    approach = copy.deepcopy(source)
    approach.points = approach.points[pregrasp_count - 1:]
    offset = _duration_seconds(approach.points[0].time_from_start)
    for point in approach.points:
        _set_duration(point.time_from_start,
                      _duration_seconds(point.time_from_start) - offset)
    return graph, approach


def validate_grasp_contract(plan, snapshot_center, gripper_open: float,
                            gripper_close: float, max_target_shift: float,
                            max_position_error: float,
                            tcp_to_pinch: Sequence[float] = (0.0, 0.0, 0.0),
                            selected_target_excluded_from_collision: bool = False):
    """Reject old pregrasp-only plans instead of closing far from an object."""
    if (str(plan.reason) != GRASP_VALIDATED_PLAN_REASON or
            not bool(getattr(plan, "grasp_approach_valid", False))):
        raise ValueError("pickup_requires_validated_final_approach")
    if bool(getattr(plan, "selected_target_excluded_from_collision", False)) != bool(
            selected_target_excluded_from_collision):
        raise ValueError("grasp_selected_target_collision_policy_mismatch")
    count = int(getattr(plan, "pregrasp_waypoint_count", 0))
    if count < 2 or count >= len(plan.joint_path_preview.points):
        raise ValueError("grasp_pregrasp_waypoint_count_invalid")
    if len(plan.end_effector_path.poses) != len(plan.joint_path_preview.points):
        raise ValueError("grasp_joint_and_pose_path_size_mismatch")
    target = getattr(plan, "grasp_target_position", None)
    if target is None:
        raise ValueError("grasp_target_position_missing")
    target_position = _point_tuple(target)
    if not all(math.isfinite(value) for value in target_position):
        raise ValueError("grasp_target_position_nonfinite")
    if _distance(target_position, snapshot_center) > max_target_shift:
        raise ValueError("grasp_target_snapshot_mismatch")
    error = float(getattr(plan, "grasp_position_error", math.nan))
    final_pose = plan.end_effector_path.poses[-1].pose
    pinch_offset = rotate_vector_by_quaternion(tcp_to_pinch, final_pose.orientation)
    pinch_position = tuple(a + b for a, b in zip(
        _point_tuple(final_pose.position), pinch_offset))
    actual_error = _distance(pinch_position, target_position)
    if (not math.isfinite(error) or error < 0.0 or error > max_position_error or
            not math.isfinite(actual_error) or actual_error > max_position_error):
        raise ValueError("grasp_endpoint_position_error_invalid")
    for field, expected in (("gripper_open_position", gripper_open),
                            ("gripper_close_position", gripper_close)):
        value = float(getattr(plan, field, math.nan))
        if not math.isfinite(value) or abs(value - expected) > 1.0e-6:
            raise ValueError(f"grasp_collision_model_{field}_mismatch")
    return count, target_position


def gripper_result_state(result, opening: bool, requested: float,
                         close_position: float, tolerance: float,
                         measured_position: Optional[float] = None,
                         allow_stalled_abort: bool = False) -> str:
    """Classify actuator feedback; a closed aperture alone is not a grasp."""
    if result is None:
        return "action_failed"
    feedback = getattr(result, "result", None)
    stalled = bool(getattr(feedback, "stalled", False))
    if result.status != GoalStatus.STATUS_SUCCEEDED and not (
            allow_stalled_abort and not opening and stalled and
            result.status == GoalStatus.STATUS_ABORTED):
        return "action_failed"
    reported = float(getattr(feedback, "position", math.nan))
    position = reported if measured_position is None else float(measured_position)
    if not math.isfinite(position) or not math.isfinite(reported):
        return "position_invalid"
    reached = bool(getattr(feedback, "reached_goal", False))
    if opening:
        return ("open_reached" if reached and not stalled and
                abs(position - requested) <= tolerance else "open_not_reached")
    if stalled and position > close_position + tolerance:
        return "contact_detected"
    if reached and abs(position - requested) <= tolerance:
        return "closed_unconfirmed"
    return "close_not_confirmed"


def gripper_stationary(history, earliest_time: float, duration: float,
                       tolerance: float) -> bool:
    """Require a full interval of encoder observations with bounded variation."""
    samples = [(stamp, value) for stamp, value in history if stamp >= earliest_time]
    if len(samples) < 2:
        return False
    cutoff = samples[-1][0] - duration
    anchor = next((index for index in range(len(samples) - 1, -1, -1)
                   if samples[index][0] <= cutoff), None)
    if anchor is None:
        return False
    values = [value for _, value in samples[anchor:]]
    return all(math.isfinite(value) for value in values) and max(values) - min(values) <= tolerance


def _grasp_fingerprint_payload(plan) -> dict:
    if not bool(getattr(plan, "grasp_approach_valid", False)):
        return {}
    target = getattr(plan, "grasp_target_position", None)
    return {
        "grasp_approach_valid": True,
        "pregrasp_waypoint_count": int(getattr(plan, "pregrasp_waypoint_count", 0)),
        "grasp_target_position": (list(_point_tuple(target)) if target is not None else None),
        "gripper_open_position": float(getattr(plan, "gripper_open_position", math.nan)),
        "gripper_close_position": float(getattr(plan, "gripper_close_position", math.nan)),
        "selected_target_excluded_from_collision": bool(getattr(
            plan, "selected_target_excluded_from_collision", False)),
    }


def trajectory_fingerprint(plan: ReachabilityPlan) -> str:
    payload = {
        "graph_revision": int(plan.graph_revision),
        "target": int(plan.target_environment_node_id),
        "nodes": [int(value) for value in plan.reachability_node_ids],
        "joints": list(plan.joint_path_preview.joint_names),
        # Point zero is the measured start and naturally jitters between the
        # 2 Hz reachability updates. Bind the frozen plan to graph waypoints;
        # current joints are checked separately immediately before execution.
        "points": [[round(float(value), 7) for value in point.positions]
                   for point in plan.joint_path_preview.points[1:]],
    }
    payload.update(_grasp_fingerprint_payload(plan))
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()


def motion_fingerprint(plan: ReachabilityPlan) -> str:
    """Identify the reviewed motion while allowing semantic node-ID churn."""
    payload = {
        "graph_revision": int(plan.graph_revision),
        "nodes": [int(value) for value in plan.reachability_node_ids],
        "joints": list(plan.joint_path_preview.joint_names),
        "points": [[round(float(value), 7) for value in point.positions]
                   for point in plan.joint_path_preview.points[1:]],
    }
    payload.update(_grasp_fingerprint_payload(plan))
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()


def validate_goal_alignment(task_mode, alignment, minimum_alignment):
    if task_mode not in {"move_to_target", "pickup"}:
        raise ValueError(f"unsupported_task_mode:{task_mode}")
    if task_mode == "pickup" and alignment < minimum_alignment:
        raise ValueError(
            f"gripper_approach_misaligned:{alignment:.6f}<{minimum_alignment:.6f}")


@dataclass
class FrozenPickPlan:
    created_monotonic: float
    fingerprint: str
    motion_fingerprint: str
    graph_revision: int
    target_track_id: int
    target_node_id: int
    target_class_id: int
    target_class_name: str
    target_position: Tuple[float, float, float]
    component_centroid: Tuple[float, float, float]
    tracked_centroid: Tuple[float, float, float]
    component_size: int
    target_distance: float
    alignment: float
    trajectory: JointTrajectory
    path: Path
    blockers: List[str]
    bus_read_failures: int
    bus_write_failures: int
    task_mode: str = "pickup"
    graph_trajectory: Optional[JointTrajectory] = None
    approach_trajectory: Optional[JointTrajectory] = None
    pregrasp_waypoint_count: int = 0
    selected_target_excluded_from_collision: bool = False
    gripper_open_position: float = math.nan
    gripper_close_position: float = math.nan


class GraphPickNode(Node):
    def __init__(self) -> None:
        super().__init__("graph_pick")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("task_mode", "move_to_target")
        self.declare_parameter(
            "arm_joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )
        self.declare_parameter(
            "environment_topic", "/om6dof_topo_gng_v2/environment_graph_data")
        self.declare_parameter(
            "reachability_plan_topic", "/om6dof_topo_gng_v2/reachability_plan")
        self.declare_parameter("perception_status_topic", "/om6dof_topo_gng_v2/status")
        self.declare_parameter("labels_topic", "/om6dof_topo_gng_v2/labels")
        self.declare_parameter(
            "object_clusters_topic", "/om6dof_topo_gng_v2/object_clusters")
        self.declare_parameter(
            "target_classes_topic", "/om6dof_topo_gng_v2/set_target_classes")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter(
            "controller_manager_service", "/controller_manager/list_controllers")
        self.declare_parameter("arm_controller_name", "arm_controller")
        self.declare_parameter("offset_controller_name", "forward_offset_controller")
        self.declare_parameter("controller_state_timeout_sec", 1.5)
        self.declare_parameter(
            "dynamixel_health_topic", "/dynamixel_hardware_interface/health")
        self.declare_parameter(
            "dynamixel_health_status_name",
            "dynamixel_hardware_interface/BusHealth")
        self.declare_parameter("dynamixel_health_timeout_sec", 0.30)
        # The exact-collision planner itself can take close to one second on
        # this AGX. Two seconds admits one completed planning cycle while the
        # target ID and current-joint checks below still prevent stale motion.
        self.declare_parameter("max_input_age_sec", 2.0)
        self.declare_parameter("max_joint_state_age_sec", 0.5)
        self.declare_parameter("max_plan_hold_sec", 5.0)
        self.declare_parameter("max_start_joint_error_rad", 0.08)
        self.declare_parameter("arm_endpoint_verification_timeout_sec", 2.0)
        self.declare_parameter("max_target_distance_m", 0.055)
        self.declare_parameter("minimum_target_distance_m", 0.01)
        self.declare_parameter("minimum_target_component_nodes", 3)
        self.declare_parameter("max_object_track_association_m", 0.08)
        self.declare_parameter("max_object_track_shift_m", 0.02)
        self.declare_parameter("target_reacquisition_timeout_sec", 1.0)
        self.declare_parameter("max_grasp_target_snapshot_shift_m", 0.01)
        self.declare_parameter("max_grasp_position_error_m", 0.005)
        self.declare_parameter("grasp_tcp_to_pinch", [0.0, 0.0, 0.0])
        self.declare_parameter("selected_target_excluded_from_collision", False)
        # Frame-specific convention; the isolated grasp launch overrides this
        # to the physical forward axis verified for its URDF/model.
        self.declare_parameter("tool_approach_axis", [1.0, 0.0, 0.0])
        self.declare_parameter("minimum_approach_alignment", 0.30)
        self.declare_parameter("require_calibration_verified", True)
        self.declare_parameter("execution_enabled", False)
        self.declare_parameter(
            "arm_action_name", "/arm_controller/follow_joint_trajectory")
        self.declare_parameter(
            "gripper_action_name", "/gripper_controller/gripper_cmd")
        self.declare_parameter("gripper_open", 0.019)
        self.declare_parameter("gripper_close", -0.010)
        self.declare_parameter("gripper_max_effort", 0.0)
        self.declare_parameter("gripper_joint_name", "gripper_left_joint")
        self.declare_parameter("gripper_position_tolerance", 0.0015)
        self.declare_parameter("gripper_result_match_tolerance", 0.002)
        self.declare_parameter("gripper_contact_position_margin", 0.002)
        self.declare_parameter("gripper_stationary_time_sec", 0.25)
        self.declare_parameter("gripper_stationary_tolerance", 0.0005)
        self.declare_parameter("gripper_verification_timeout_sec", 3.0)
        self.declare_parameter("grasp_validation_service",
                               "/om6dof_topo_gng_v2/validate_grasp_execution")
        self.declare_parameter("grasp_validation_timeout_sec", 30.0)
        self.declare_parameter("action_timeout_sec", 120.0)
        self.declare_parameter("retreat_after_grasp", False)
        self.declare_parameter("retreat_joint_velocity", 0.20)
        self.declare_parameter("minimum_segment_time_sec", 0.10)

        self.task_mode = str(self.get_parameter("task_mode").value)
        validate_goal_alignment(self.task_mode, 1.0, 0.3)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.joint_names = [str(value) for value in
                            self.get_parameter("arm_joint_names").value]
        self.execution_enabled = bool(self.get_parameter("execution_enabled").value)
        axis = [float(value) for value in
                self.get_parameter("tool_approach_axis").value]
        if _normalize(axis) is None:
            raise ValueError("tool_approach_axis must be a nonzero finite 3-vector")
        self.tool_axis = axis
        self.gripper_joint_name = str(self.get_parameter("gripper_joint_name").value)
        self.tcp_to_pinch = [float(value) for value in
                             self.get_parameter("grasp_tcp_to_pinch").value]
        if len(self.tcp_to_pinch) != 3 or not all(
                math.isfinite(value) for value in self.tcp_to_pinch):
            raise ValueError("grasp_tcp_to_pinch must be a finite 3-vector")
        for parameter_name in (
                "max_object_track_association_m", "max_object_track_shift_m",
                "target_reacquisition_timeout_sec",
                "max_grasp_target_snapshot_shift_m", "max_grasp_position_error_m",
                "gripper_position_tolerance",
                "gripper_result_match_tolerance", "gripper_contact_position_margin",
                "gripper_stationary_time_sec", "gripper_stationary_tolerance",
                "gripper_verification_timeout_sec",
                "arm_endpoint_verification_timeout_sec",
                "grasp_validation_timeout_sec",
                "controller_state_timeout_sec"):
            value = float(self.get_parameter(parameter_name).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{parameter_name} must be positive and finite")
        gripper_open = float(self.get_parameter("gripper_open").value)
        gripper_close = float(self.get_parameter("gripper_close").value)
        if (not math.isfinite(gripper_open) or not math.isfinite(gripper_close) or
                gripper_open <= gripper_close):
            raise ValueError("gripper_open must be finite and greater than gripper_close")
        if self.task_mode == "pickup" and bool(
                self.get_parameter("retreat_after_grasp").value):
            raise ValueError("pickup_retreat_requires_payload_validated_plan")
        if self.task_mode == "pickup" and ValidateGraspExecution is None:
            raise ValueError("pickup_validator_interface_unavailable; rebuild om6dof_dd_gng")

        self._lock = threading.RLock()
        self._environment = None
        self._environment_time = 0.0
        self._environment_history = deque(maxlen=64)
        self._reachability_plan = None
        self._reachability_plan_time = 0.0
        self._perception_status = {}
        self._perception_status_time = 0.0
        self._labels_by_node: Dict[int, str] = {}
        self._labels_time = 0.0
        self._object_track_history = deque(maxlen=64)
        self._selected_target = ""
        self._joint_state = None
        self._joint_state_time = 0.0
        self._gripper_measurements = deque(maxlen=512)
        self._controller_snapshot = None
        self._controller_snapshot_time = 0.0
        self._controller_query_future = None
        self._controller_query_started = 0.0
        self._bus_health = None
        self._bus_health_time = 0.0
        self._frozen: Optional[FrozenPickPlan] = None
        self._state = "waiting_for_inputs"
        self._message = "waiting for semantic and reachability graph inputs"
        self._busy = False
        self._holding_object = False
        self._faulted = False
        self._fault_reason = ""
        self._cancel = threading.Event()
        self._arm_goal_handle = None
        self._gripper_goal_handle = None
        self._last_gripper_result_state = "not_commanded"
        self._failure_stage = ""
        self._observed_target_track_id = None

        cb = ReentrantCallbackGroup()
        input_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            EnvironmentGraph, str(self.get_parameter("environment_topic").value),
            self._on_environment, input_qos, callback_group=cb)
        self.create_subscription(
            ReachabilityPlan,
            str(self.get_parameter("reachability_plan_topic").value),
            self._on_reachability_plan, input_qos, callback_group=cb)
        self.create_subscription(
            String, str(self.get_parameter("perception_status_topic").value),
            self._on_perception_status, input_qos, callback_group=cb)
        self.create_subscription(
            String, str(self.get_parameter("labels_topic").value),
            self._on_labels, input_qos, callback_group=cb)
        self.create_subscription(
            String, str(self.get_parameter("object_clusters_topic").value),
            self._on_object_clusters, input_qos, callback_group=cb)
        self.create_subscription(
            String, str(self.get_parameter("target_classes_topic").value),
            self._on_target_classes, input_qos, callback_group=cb)
        self.create_subscription(
            JointState, str(self.get_parameter("joint_state_topic").value),
            self._on_joint_state, input_qos, callback_group=cb)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            DiagnosticArray, str(self.get_parameter("dynamixel_health_topic").value),
            self._on_bus_health, state_qos, callback_group=cb)

        self._status_pub = self.create_publisher(
            String, "/om6dof_topo_gng_v2/graph_pick/status", latched_qos)
        self._trajectory_pub = self.create_publisher(
            JointTrajectory,
            "/om6dof_topo_gng_v2/graph_pick/trajectory_preview", latched_qos)
        self._path_pub = self.create_publisher(
            Path, "/om6dof_topo_gng_v2/graph_pick/path_preview", latched_qos)
        self._target_pub = self.create_publisher(
            PointStamped, "/om6dof_topo_gng_v2/graph_pick/target", latched_qos)
        self._marker_pub = self.create_publisher(
            MarkerArray, "/om6dof_topo_gng_v2/graph_pick/markers", latched_qos)

        self._arm_client = ActionClient(
            self, FollowJointTrajectory,
            str(self.get_parameter("arm_action_name").value), callback_group=cb)
        self._gripper_client = ActionClient(
            self, GripperCommand,
            str(self.get_parameter("gripper_action_name").value), callback_group=cb)

        self.create_service(
            Trigger, "/plan_graph_pick", self._on_plan, callback_group=cb)
        self.create_service(
            Trigger, "/execute_graph_pick", self._on_execute, callback_group=cb)
        self.create_service(
            Trigger, "/graph_pick_status", self._on_status, callback_group=cb)
        self.create_service(
            Trigger, "/cancel_graph_pick", self._on_cancel, callback_group=cb)
        self._controller_client = self.create_client(
            ListControllers, str(self.get_parameter("controller_manager_service").value),
            callback_group=cb)
        self._grasp_validate_client = (self.create_client(
            ValidateGraspExecution,
            str(self.get_parameter("grasp_validation_service").value),
            callback_group=cb) if self.task_mode == "pickup" else None)
        self.create_timer(0.5, self._poll_controllers, callback_group=cb)
        self.create_timer(0.5, self._publish_status, callback_group=cb)
        self.create_timer(0.1, self._monitor_execution_interlocks, callback_group=cb)
        self._publish_status()
        self.get_logger().info(
            "DD-GNG graph-pick coordinator ready; execution_enabled="
            f"{str(self.execution_enabled).lower()}; planning never sends a "
            "robot command")

    def _on_environment(self, message: EnvironmentGraph) -> None:
        received = time.monotonic()
        # rclpy hands this callback a completed message object and does not
        # mutate it afterwards. Retain that immutable snapshot: deep-copying
        # thousands of graph nodes/edges here can starve the reachability
        # callback and can itself make a fresh plan exceed the age gate.
        with self._lock:
            self._environment = message
            self._environment_time = received
            self._environment_history.append((received, message))

    def _on_reachability_plan(self, message: ReachabilityPlan) -> None:
        with self._lock:
            self._reachability_plan = copy.deepcopy(message)
            self._reachability_plan_time = time.monotonic()

    def _on_perception_status(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError("status is not an object")
        except (ValueError, TypeError, json.JSONDecodeError):
            value = {"accepted": False, "state": "invalid_status_json"}
        with self._lock:
            self._perception_status = value
            self._perception_status_time = time.monotonic()

    def _on_labels(self, message: String) -> None:
        labels: Dict[int, str] = {}
        try:
            value = json.loads(message.data)
            for item in value if isinstance(value, list) else []:
                labels[int(item["node_id"])] = str(item["class"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            labels = {}
        with self._lock:
            self._labels_by_node = labels
            self._labels_time = time.monotonic()

    def _on_object_clusters(self, message: String) -> None:
        observations = parse_object_tracks(message.data)
        with self._lock:
            self._object_track_history.append((time.monotonic(), observations))

    def _on_target_classes(self, message: String) -> None:
        selected = message.data.strip().lower() or "all"
        publish = False
        with self._lock:
            previous = self._selected_target
            self._selected_target = selected
            if previous and selected != previous and self._frozen is not None:
                self._frozen = None
                self._state = "target_changed"
                self._message = "planning target changed; request a new preview"
                publish = True
        if publish:
            self._publish_status()

    def _on_joint_state(self, message: JointState) -> None:
        received = time.monotonic()
        gripper = ordered_joint_positions(message, [self.gripper_joint_name])
        with self._lock:
            self._joint_state = copy.deepcopy(message)
            self._joint_state_time = received
            if gripper is not None:
                self._gripper_measurements.append((received, gripper[0]))

    def _poll_controllers(self) -> None:
        """Query topology without switching controllers or sending a command."""
        now = time.monotonic()
        timeout = float(self.get_parameter("controller_state_timeout_sec").value)
        with self._lock:
            pending = self._controller_query_future
            if pending is not None:
                if now - self._controller_query_started < timeout:
                    return
                # Invalidate identity before cancellation; late replies cannot
                # overwrite a newer snapshot or make an old state look fresh.
                self._controller_query_future = None
                self._controller_snapshot = None
                self._controller_client.remove_pending_request(pending)
                pending.cancel()
            if not self._controller_client.service_is_ready():
                self._controller_snapshot = None
                return
            try:
                future = self._controller_client.call_async(ListControllers.Request())
            except Exception:
                self._controller_snapshot = None
                return
            self._controller_query_future = future
            self._controller_query_started = now
            future.add_done_callback(self._on_controllers)

    def _on_controllers(self, future) -> None:
        with self._lock:
            if future is not self._controller_query_future:
                return
            self._controller_query_future = None
            try:
                self._controller_snapshot = list(future.result().controller)
                # Count age from request, not receipt of a potentially delayed reply.
                self._controller_snapshot_time = self._controller_query_started
            except Exception:
                self._controller_snapshot = None

    def _on_bus_health(self, message: DiagnosticArray) -> None:
        expected = str(self.get_parameter("dynamixel_health_status_name").value)
        health = None
        for status in message.status:
            if status.name != expected:
                continue
            values = {str(item.key): str(item.value) for item in status.values}
            try:
                health = {
                    "level": _diagnostic_level(status.level),
                    "message": str(status.message),
                    "read_failure_count": int(values.get("read_failure_count", "-1")),
                    "write_failure_count": int(values.get("write_failure_count", "-1")),
                    "consecutive_read_failures": int(
                        values.get("consecutive_read_failures", "-1")),
                    "consecutive_write_failures": int(
                        values.get("consecutive_write_failures", "-1")),
                    "current_read_error": int(values.get("current_read_error", "-1")),
                    "current_write_error": int(values.get("current_write_error", "-1")),
                    "fail_safe_triggered": values.get(
                        "fail_safe_triggered", "true").lower() == "true",
                    "torque_all_enabled": values.get(
                        "torque_all_enabled", "false").lower() == "true",
                    "hardware_error_mask": int(values.get("hardware_error_mask", "-1")),
                }
            except ValueError:
                health = None
            break
        with self._lock:
            self._bus_health = health
            self._bus_health_time = time.monotonic()

    def _input_snapshot(self):
        with self._lock:
            return (
                [(received, message)
                 for received, message in self._environment_history],
                copy.deepcopy(self._reachability_plan), self._reachability_plan_time,
                dict(self._perception_status), self._perception_status_time,
                dict(self._labels_by_node), self._labels_time,
                [(received, list(observations))
                 for received, observations in self._object_track_history],
                copy.deepcopy(self._joint_state), self._joint_state_time,
            )

    def _interlock_blockers(self, plan: Optional[FrozenPickPlan] = None) -> List[str]:
        blockers = []
        health_topic = str(self.get_parameter("dynamixel_health_topic").value)
        try:
            if self.count_publishers(health_topic) != 1:
                blockers.append("dynamixel_health_publisher_count_not_one")
        except Exception:
            blockers.append("controller_publisher_count_unavailable")
        with self._lock:
            controllers = copy.deepcopy(self._controller_snapshot)
            controller_time = self._controller_snapshot_time
            health = copy.deepcopy(self._bus_health)
            health_time = self._bus_health_time
        controller_timeout = float(self.get_parameter("controller_state_timeout_sec").value)
        if controllers is None or time.monotonic() - controller_time > controller_timeout:
            blockers.append("controller_state_missing_or_stale")
        else:
            blockers.extend(additive_controller_blockers(
                controllers, self.joint_names,
                str(self.get_parameter("arm_controller_name").value),
                str(self.get_parameter("offset_controller_name").value)))
        timeout = float(self.get_parameter("dynamixel_health_timeout_sec").value)
        if health is None or time.monotonic() - health_time > timeout:
            blockers.append("dynamixel_health_missing_or_stale")
            return blockers
        if (health["level"] != 0 or health["consecutive_read_failures"] != 0
                or health["consecutive_write_failures"] != 0
                or health["current_read_error"] != 0
                or health["current_write_error"] != 0
                or health["fail_safe_triggered"]
                or not health["torque_all_enabled"]
                or health["hardware_error_mask"] != 0):
            blockers.append(f"dynamixel_bus_not_healthy:{health['message']}")
        if plan is not None and (
                health["read_failure_count"] > plan.bus_read_failures
                or health["write_failure_count"] > plan.bus_write_failures):
            blockers.append("dynamixel_failure_counter_advanced_since_preview")
        return blockers

    def _bus_failure_counts(self) -> Tuple[int, int]:
        with self._lock:
            health = copy.deepcopy(self._bus_health)
        if health is None:
            return -1, -1
        return health["read_failure_count"], health["write_failure_count"]

    def _build_plan(self) -> FrozenPickPlan:
        (environment_history, reachability, reachability_time,
         perception, perception_time, labels, labels_time,
         object_track_history,
         joint_state, joint_time) = self._input_snapshot()
        now = time.monotonic()
        max_age = float(self.get_parameter("max_input_age_sec").value)
        if reachability is None or now - reachability_time > max_age:
            raise ValueError("reachability_plan_missing_or_stale")
        # A rejected reachability message carries sentinel target IDs. Report
        # its real reason before trying to correlate that sentinel with an
        # environment snapshot, otherwise the GUI hides the actionable cause
        # behind a misleading snapshot-mismatch message.
        if reachability.header.frame_id != self.world_frame:
            raise ValueError("reachability_plan_frame_mismatch")
        if not bool(reachability.valid):
            raise ValueError(f"reachability_plan_invalid:{reachability.reason}")
        if not bool(reachability.exact_collision_valid):
            raise ValueError("reachability_plan_lacks_exact_collision_validation")
        if self.task_mode == "move_to_target" and reachability.reason not in \
                TARGET_PROTECTED_PLAN_REASONS:
            raise ValueError("planner_target_collision_check_required")
        if self.task_mode == "pickup" and (
                reachability.reason != GRASP_VALIDATED_PLAN_REASON or
                not bool(getattr(reachability, "grasp_approach_valid", False))):
            raise ValueError("pickup_requires_validated_final_approach")
        if reachability.reason not in VALIDATED_PLAN_REASONS | {
                GRASP_VALIDATED_PLAN_REASON}:
            raise ValueError(f"unexpected_reachability_reason:{reachability.reason}")
        target_id = int(reachability.target_environment_node_id)
        environment = matching_environment_snapshot(
            environment_history, target_id, reachability_time, now, max_age)
        if environment is None:
            raise ValueError("matching_environment_snapshot_missing_or_stale")
        if not perception or now - perception_time > max_age:
            raise ValueError("perception_status_missing_or_stale")
        if not bool(perception.get("accepted", False)):
            raise ValueError(
                f"perception_not_accepted:{perception.get('state', 'unknown')}")
        if environment.header.frame_id != self.world_frame:
            raise ValueError("environment_frame_mismatch")
        trajectory_error = validate_trajectory(
            reachability.joint_path_preview, self.joint_names)
        if trajectory_error:
            raise ValueError(trajectory_error)
        max_joint_age = float(self.get_parameter("max_joint_state_age_sec").value)
        if joint_state is None or now - joint_time > max_joint_age:
            raise ValueError("joint_state_missing_or_stale")
        ordered = ordered_joint_positions(joint_state, self.joint_names)
        if ordered is None:
            raise ValueError("joint_state_incomplete")
        start = list(reachability.joint_path_preview.points[0].positions)
        start_error = max(abs(a - b) for a, b in zip(ordered, start))
        if start_error > float(self.get_parameter("max_start_joint_error_rad").value):
            raise ValueError(f"joint_path_start_mismatch:{start_error:.6f}")
        class_id, component, centroid = semantic_component(environment, target_id)
        minimum_component = int(
            self.get_parameter("minimum_target_component_nodes").value)
        if len(component) < minimum_component:
            raise ValueError(
                f"semantic_target_component_too_small:{len(component)}<{minimum_component}")
        track = matching_object_track(
            object_track_history, target_id, class_id, centroid,
            reachability_time, now, max_age,
            float(self.get_parameter("max_object_track_association_m").value),
        )
        if track is None:
            raise ValueError("matching_object_track_missing_or_stale")
        class_name = track.class_name
        selected = str(perception.get("semantic_target_classes", "")).strip().lower()
        if selected and selected != "all" and class_name.lower() != selected:
            raise ValueError(
                f"tracked_object_class_mismatch:{class_name}!={selected}")
        if not reachability.end_effector_path.poses:
            raise ValueError("end_effector_path_is_empty")
        if reachability.end_effector_path.header.frame_id != self.world_frame:
            raise ValueError("end_effector_path_frame_mismatch")
        for point in reachability.end_effector_path.poses:
            if (point.header.frame_id and point.header.frame_id != self.world_frame):
                raise ValueError("end_effector_pose_frame_mismatch")
            orientation = point.pose.orientation
            values = _point_tuple(point.pose.position) + (
                orientation.x, orientation.y, orientation.z, orientation.w)
            if not all(math.isfinite(float(value)) for value in values):
                raise ValueError("end_effector_path_nonfinite_pose")
            if sum(float(value)**2 for value in values[3:]) <= 1.0e-18:
                raise ValueError("end_effector_path_zero_quaternion")
        # target_id identifies the cluster for temporal matching.  It is not a
        # grasp point: the planner and coordinator both use the 3-D centre of
        # that connected semantic cluster, so a dense top/side surface cannot
        # pull the goal away from the object's middle.
        target_position = component_bounding_center(component)
        pregrasp_count = 0
        gripper_open = float(self.get_parameter("gripper_open").value)
        gripper_close = float(self.get_parameter("gripper_close").value)
        if self.task_mode == "pickup":
            pregrasp_count, target_position = validate_grasp_contract(
                reachability, target_position, gripper_open, gripper_close,
                float(self.get_parameter("max_grasp_target_snapshot_shift_m").value),
                float(self.get_parameter("max_grasp_position_error_m").value),
                self.tcp_to_pinch,
                bool(self.get_parameter("selected_target_excluded_from_collision").value))
        goal_index = pregrasp_count - 1 if pregrasp_count else -1
        goal_pose = reachability.end_effector_path.poses[goal_index].pose
        target_distance = (float(reachability.target_distance) if not pregrasp_count else
                           _distance(_point_tuple(goal_pose.position), target_position))
        if not math.isfinite(target_distance) or target_distance < 0.0:
            raise ValueError("target_distance_invalid")
        minimum_distance = float(self.get_parameter("minimum_target_distance_m").value)
        if target_distance < minimum_distance:
            raise ValueError(f"target_too_close_to_graph_goal:{target_distance:.6f}")
        if target_distance > float(self.get_parameter("max_target_distance_m").value):
            raise ValueError(f"target_too_far_from_graph_goal:{target_distance:.6f}")
        alignment = approach_alignment(goal_pose, target_position, self.tool_axis)
        minimum_alignment = float(
            self.get_parameter("minimum_approach_alignment").value)
        validate_goal_alignment(self.task_mode, alignment, minimum_alignment)
        blockers = []
        if bool(self.get_parameter("require_calibration_verified").value) \
                and not bool(perception.get("calibration_verified", False)):
            reason = str(perception.get("calibration_reason", "unknown")).strip()
            blockers.append(f"camera_calibration_not_verified:{reason}")
        if not self.execution_enabled:
            blockers.append("execution_disabled_at_launch")
        label_name = labels.get(target_id, "") if now - labels_time <= max_age else ""
        if label_name and label_name != class_name:
            raise ValueError(
                f"tracked_object_label_mismatch:{label_name}!={class_name}")
        trajectory = retime_trajectory(
            reachability.joint_path_preview,
            velocity=0.20,
            minimum_segment_time=float(
                self.get_parameter("minimum_segment_time_sec").value),
        )
        graph_trajectory, approach_trajectory = (None, None)
        if pregrasp_count:
            graph_trajectory, approach_trajectory = split_grasp_trajectory(
                trajectory, pregrasp_count)
        bus_read_failures, bus_write_failures = self._bus_failure_counts()
        return FrozenPickPlan(
            created_monotonic=now,
            fingerprint=trajectory_fingerprint(reachability),
            motion_fingerprint=motion_fingerprint(reachability),
            graph_revision=int(reachability.graph_revision),
            target_track_id=track.track_id,
            target_node_id=target_id,
            target_class_id=class_id,
            target_class_name=class_name,
            target_position=target_position,
            component_centroid=centroid,
            tracked_centroid=track.tracked_centroid,
            component_size=len(component),
            target_distance=target_distance,
            alignment=alignment,
            trajectory=trajectory,
            path=copy.deepcopy(reachability.end_effector_path),
            blockers=blockers,
            bus_read_failures=bus_read_failures,
            bus_write_failures=bus_write_failures,
            task_mode=self.task_mode,
            graph_trajectory=graph_trajectory,
            approach_trajectory=approach_trajectory,
            pregrasp_waypoint_count=pregrasp_count,
            selected_target_excluded_from_collision=bool(getattr(
                reachability, "selected_target_excluded_from_collision", False)),
            gripper_open_position=gripper_open,
            gripper_close_position=gripper_close,
        )

    def _plan_payload(self, plan: Optional[FrozenPickPlan] = None) -> dict:
        with self._lock:
            plan = plan or self._frozen
            state = self._state
            message = self._message
            busy = self._busy
            holding = self._holding_object
            faulted = self._faulted
            fault_reason = self._fault_reason
        payload = {
            "schema": "om6dof.graph_pick_status.v1",
            "task_mode": self.task_mode,
            "state": state,
            "message": message,
            "execution_enabled": self.execution_enabled,
            "busy": busy,
            "holding_object": holding,
            "motion_faulted": faulted,
            "motion_fault_reason": fault_reason,
            "failure_stage": getattr(self, "_failure_stage", ""),
            "observed_target_track_id": getattr(self, "_observed_target_track_id", None),
            "plan_ready": plan is not None,
        }
        if plan is not None:
            age = time.monotonic() - plan.created_monotonic
            blockers = list(plan.blockers)
            blockers.extend(self._interlock_blockers(plan))
            if faulted:
                blockers.append("motion_faulted")
            if age > float(self.get_parameter("max_plan_hold_sec").value):
                blockers.append("frozen_plan_stale")
            blockers = list(dict.fromkeys(blockers))
            payload.update({
                "plan_id": plan.fingerprint[:16],
                "motion_id": plan.motion_fingerprint[:16],
                "plan_age_sec": round(age, 3),
                "graph_revision": plan.graph_revision,
                "target_track_id": plan.target_track_id,
                "target_node_id": plan.target_node_id,
                "target_class_id": plan.target_class_id,
                "target_class": plan.target_class_name,
                "target_position": list(plan.target_position),
                "component_centroid": list(plan.component_centroid),
                "tracked_centroid": list(plan.tracked_centroid),
                "component_nodes": plan.component_size,
                "target_distance_m": plan.target_distance,
                "approach_alignment": plan.alignment,
                "grasp_orientation_required": plan.task_mode == "pickup",
                "final_approach_validated": plan.approach_trajectory is not None,
                "pregrasp_waypoint_count": plan.pregrasp_waypoint_count,
                "selected_target_excluded_from_collision":
                    plan.selected_target_excluded_from_collision,
                "approach_points": (len(plan.approach_trajectory.points)
                                    if plan.approach_trajectory is not None else 0),
                "trajectory_points": len(plan.trajectory.points),
                "reachability_edges_used": max(0, len(plan.trajectory.points) - 2),
                "execution_blockers": blockers,
                "executable": not blockers,
            })
        return payload

    def _publish_status(self) -> None:
        self._status_pub.publish(String(data=json.dumps(
            self._plan_payload(), separators=(",", ":"), sort_keys=True)))

    def _publish_plan(self, plan: FrozenPickPlan) -> None:
        self._trajectory_pub.publish(copy.deepcopy(plan.trajectory))
        self._path_pub.publish(copy.deepcopy(plan.path))
        target = PointStamped()
        target.header = copy.deepcopy(plan.path.header)
        target.header.frame_id = self.world_frame
        target.point.x, target.point.y, target.point.z = plan.target_position
        self._target_pub.publish(target)

        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        clear.header = copy.deepcopy(target.header)
        markers.markers.append(clear)
        sphere = Marker()
        sphere.header = copy.deepcopy(target.header)
        sphere.ns = "graph_pick_target"
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = copy.deepcopy(target.point)
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.025
        sphere.color.r = 1.0
        sphere.color.g = 0.55
        sphere.color.b = 0.05
        sphere.color.a = 0.95
        markers.markers.append(sphere)
        if plan.path.poses:
            goal_pose = plan.path.poses[
                plan.pregrasp_waypoint_count - 1 if plan.pregrasp_waypoint_count else -1].pose
            endpoint = approach_axis_endpoint(
                goal_pose, plan.target_position, self.tool_axis)
            arrow = Marker()
            arrow.header = copy.deepcopy(target.header)
            arrow.ns = "graph_pick_approach"
            arrow.id = 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            # Draw the configured physical gripper approach direction. The endpoint is
            # intentionally derived from FK orientation, rather than forced
            # to the target point, so an alignment error stays visible.
            arrow.points = [copy.deepcopy(goal_pose.position), Point(
                x=endpoint[0], y=endpoint[1], z=endpoint[2])]
            arrow.scale.x = 0.006
            arrow.scale.y = 0.012
            arrow.scale.z = 0.018
            arrow.color.g = 1.0
            arrow.color.a = 0.95
            markers.markers.append(arrow)
        self._marker_pub.publish(markers)
        self._publish_status()

    def _on_plan(self, _request, response):
        with self._lock:
            if self._busy:
                response.success = False
                response.message = "graph pickup is busy"
                return response
        try:
            plan = self._build_plan()
        except (ValueError, StopIteration) as error:
            with self._lock:
                self._frozen = None
                self._state = "plan_rejected"
                self._message = str(error)
            self._publish_status()
            response.success = False
            response.message = f"graph trajectory rejected: {error}"
            return response
        with self._lock:
            self._frozen = plan
            self._state = "preview_ready"
            self._message = (
                f"preview ready for {plan.target_class_name}; "
                f"{len(plan.trajectory.points)} joint points")
        self._publish_plan(plan)
        response.success = True
        response.message = json.dumps(self._plan_payload(plan), sort_keys=True)
        return response

    def _execution_rejection(self, plan: FrozenPickPlan) -> Optional[str]:
        if self._faulted:
            reason = getattr(self, "_fault_reason", "") or "reason unavailable"
            return (
                f"motion_faulted:{reason}; restart the graph-pick node "
                "after checking hardware")
        if not self.execution_enabled:
            return "execution_disabled_at_launch"
        age = time.monotonic() - plan.created_monotonic
        if age > float(self.get_parameter("max_plan_hold_sec").value):
            return f"frozen_plan_stale:{age:.3f}s"
        if plan.blockers:
            return ",".join(plan.blockers)
        interlock_blockers = self._interlock_blockers(plan)
        if interlock_blockers:
            return ",".join(interlock_blockers)
        try:
            current = self._build_plan()
        except (ValueError, StopIteration) as error:
            return f"live_revalidation_failed:{error}"
        if current.blockers:
            return "live_revalidation_blocked:" + ",".join(current.blockers)
        if current.target_class_id != plan.target_class_id:
            return "semantic_target_class_changed; request a new preview"
        if current.target_track_id != plan.target_track_id:
            return "semantic_object_track_changed; request a new preview"
        track_shift = _distance(current.tracked_centroid, plan.tracked_centroid)
        max_shift = float(
            self.get_parameter("max_object_track_shift_m").value)
        if track_shift > max_shift:
            return f"semantic_object_moved:{track_shift:.6f}m>{max_shift:.6f}m"
        if current.motion_fingerprint != plan.motion_fingerprint:
            return "live_motion_path_changed; request a new preview"
        return None

    def _wait_future(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and not self._cancel.is_set():
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.02)
        return future.done() and rclpy.ok() and not self._cancel.is_set()

    def _latch_motion_fault(self, reason: str) -> None:
        """Retain the first execution fault until this coordinator restarts."""
        normalized = str(reason).strip() or "unknown motion fault"
        with self._lock:
            first_fault = not self._faulted
            self._faulted = True
            if first_fault or not self._fault_reason:
                self._fault_reason = normalized
        if first_fault:
            self.get_logger().error(f"motion fault latched: {normalized}")

    def _execution_interlocks_ready(self, plan: FrozenPickPlan) -> bool:
        if self._cancel.is_set() or self._faulted:
            self._message = "execution cancelled or faulted"
            return False
        blockers = self._interlock_blockers(plan)
        if not blockers:
            return True
        self._message = "execution interlock tripped: " + ",".join(blockers)
        self._latch_motion_fault(self._message)
        self._cancel.set()
        return False

    def _segment_start_rejection(self, trajectory: JointTrajectory) -> Optional[str]:
        """Compare measured joints with this segment, including after graph motion."""
        with self._lock:
            joint_state = copy.deepcopy(self._joint_state)
            received = self._joint_state_time
        if joint_state is None or time.monotonic() - received > float(
                self.get_parameter("max_joint_state_age_sec").value):
            return "joint_state_missing_or_stale"
        ordered = ordered_joint_positions(joint_state, self.joint_names)
        if ordered is None:
            return "joint_state_incomplete"
        if not trajectory.points or len(trajectory.points[0].positions) != len(ordered):
            return "arm_segment_start_invalid"
        error = max(abs(a - b) for a, b in zip(
            ordered, trajectory.points[0].positions))
        if not math.isfinite(error) or error > float(
                self.get_parameter("max_start_joint_error_rad").value):
            return f"arm_segment_start_mismatch:{error:.6f}"
        return None

    def _pickup_target_rejection(self, plan: FrozenPickPlan) -> Optional[str]:
        """Check current sensing/identity without rebuilding a plan from its old start."""
        now = time.monotonic()
        max_age = float(self.get_parameter("max_input_age_sec").value)
        with self._lock:
            perception = dict(self._perception_status)
            perception_time = self._perception_status_time
            history = list(self._object_track_history)
        if now - perception_time > max_age or not perception.get("accepted", False):
            return "perception_missing_stale_or_rejected"
        if bool(self.get_parameter("require_calibration_verified").value) and not \
                perception.get("calibration_verified", False):
            return "camera_calibration_not_verified"
        observation, rejection = pickup_target_observation(
            history, plan, now, max_age,
            float(self.get_parameter("max_object_track_shift_m").value))
        if observation is not None:
            with self._lock:
                self._observed_target_track_id = observation.track_id
        return rejection

    def _arm_segment_ready(self, plan: FrozenPickPlan,
                           trajectory: JointTrajectory) -> bool:
        # Pause at the segment boundary for a brief detector dropout. Never
        # send motion during reacquisition, and never wait through joint drift,
        # hardware faults, moved objects, or ambiguous identity.
        deadline = None
        while True:
            if not self._execution_interlocks_ready(plan):
                return False
            rejection = self._segment_start_rejection(trajectory)
            if rejection is not None:
                self._message = rejection
                return False
            if plan.task_mode != "pickup":
                return True
            rejection = self._pickup_target_rejection(plan)
            if rejection is None:
                return True
            self._message = rejection
            if rejection not in {"frozen_object_track_not_current", "pickup_target_reacquiring"}:
                return False
            if deadline is None:
                deadline = time.monotonic() + float(
                    self.get_parameter("target_reacquisition_timeout_sec").value)
                self._publish_status()
            if time.monotonic() >= deadline or self._cancel.wait(0.02):
                return False

    def _cancel_late_goal(self, future) -> None:
        """Cancel a goal whose acceptance arrives after timeout/cancellation."""
        try:
            handle = future.result()
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
        except Exception:
            # A canceled/exceptional acceptance has no usable goal handle.
            pass

    def _validate_pickup_segment(self, plan: FrozenPickPlan,
                                 trajectory: JointTrajectory,
                                 include_closure: bool) -> bool:
        """Ask the planner to check this frozen segment against its newest scene."""
        if not self._arm_segment_ready(plan, trajectory):
            return False
        client = self._grasp_validate_client
        if ValidateGraspExecution is None or client is None or not client.wait_for_service(
                timeout_sec=1.0):
            self._message = "grasp_execution_validator_unavailable"
            return False
        if self._cancel.is_set():
            self._message = "grasp_execution_validation_cancelled"
            return False
        request = ValidateGraspExecution.Request()
        request.trajectory = copy.deepcopy(trajectory)
        request.trajectory.header.frame_id = self.world_frame
        request.trajectory.header.stamp = self.get_clock().now().to_msg()
        request.target_position = Point(
            x=plan.target_position[0], y=plan.target_position[1], z=plan.target_position[2])
        request.target_class_id = plan.target_class_id
        request.target_environment_node_id = plan.target_node_id
        request.gripper_open_position = plan.gripper_open_position
        request.gripper_close_position = plan.gripper_close_position
        request.include_closure = include_closure
        try:
            future = client.call_async(request)
            if not self._wait_future(future, float(
                    self.get_parameter("grasp_validation_timeout_sec").value)):
                client.remove_pending_request(future)
                future.cancel()
                self._message = "grasp_execution_validation_timed_out_or_cancelled"
                return False
            result = future.result()
        except Exception as error:
            self._message = f"grasp_execution_validation_failed:{error}"
            return False
        if result is None or not result.valid:
            self._message = "grasp_execution_validation_rejected:" + str(
                getattr(result, "reason", "missing_response"))
            return False
        # The service may have taken long enough for encoders, health, or
        # target observations to change. Recheck these before sending motion.
        return self._arm_segment_ready(plan, trajectory)

    def _confirm_gripper_result(self, result, opening: bool,
                                requested: float, action_started: float,
                                deadline: float) -> bool:
        """Action success can precede physical arrival with legacy loose tolerances."""
        result_received = time.monotonic()
        feedback = getattr(result, "result", None)
        if gripper_result_state(result, opening, requested, requested,
                                float(self.get_parameter("gripper_position_tolerance").value),
                                allow_stalled_abort=True) in {"action_failed", "position_invalid"}:
            self._last_gripper_result_state = "action_failed"
            return False
        deadline = min(deadline, result_received + float(
            self.get_parameter("gripper_verification_timeout_sec").value))
        self._last_gripper_result_state = "measured_gripper_unconfirmed"
        while rclpy.ok() and not self._cancel.is_set() and time.monotonic() < deadline:
            now = time.monotonic()
            with self._lock:
                measurements = list(self._gripper_measurements)
            if (not measurements or measurements[-1][0] < result_received or
                    now - measurements[-1][0] > float(
                        self.get_parameter("max_joint_state_age_sec").value)):
                time.sleep(0.02)
                continue
            position = measurements[-1][1]
            state = gripper_result_state(
                result, opening, requested, requested,
                float(self.get_parameter("gripper_position_tolerance").value),
                measured_position=position, allow_stalled_abort=True)
            if state == "open_reached":
                self._last_gripper_result_state = state
                return True
            reported = float(getattr(feedback, "position", math.nan))
            matches = abs(position - reported) <= float(
                self.get_parameter("gripper_result_match_tolerance").value)
            stationary = gripper_stationary(
                measurements, action_started,
                float(self.get_parameter("gripper_stationary_time_sec").value),
                float(self.get_parameter("gripper_stationary_tolerance").value))
            if not opening and matches and stationary:
                if state == "contact_detected" and position > requested + float(
                        self.get_parameter("gripper_contact_position_margin").value):
                    self._last_gripper_result_state = state
                    return True
                if state == "closed_unconfirmed":
                    self._last_gripper_result_state = state
                    return True
            time.sleep(0.02)
        return False

    def _send_gripper(self, position: float, label: str) -> bool:
        self._last_gripper_result_state = "not_confirmed"
        if self._cancel.is_set():
            self._message = f"{label} cancelled before command"
            return False
        if not self._gripper_client.wait_for_server(timeout_sec=3.0):
            self._message = "gripper action server unavailable"
            return False
        if self._cancel.is_set():
            self._message = f"{label} cancelled before command"
            return False
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(
            self.get_parameter("gripper_max_effort").value)
        action_started = time.monotonic()
        future = self._gripper_client.send_goal_async(goal)
        if not self._wait_future(future, 5.0):
            future.add_done_callback(self._cancel_late_goal)
            self._message = f"{label} goal acceptance timed out"
            self._latch_motion_fault(self._message)
            return False
        handle = future.result()
        if handle is None or not handle.accepted:
            self._message = f"{label} goal rejected"
            return False
        with self._lock:
            self._gripper_goal_handle = handle
        if self._cancel.is_set():
            handle.cancel_goal_async()
            self._message = f"{label} cancelled before result"
            return False
        result_future = handle.get_result_async()
        timeout = float(self.get_parameter("action_timeout_sec").value)
        if not self._wait_future(result_future, timeout):
            handle.cancel_goal_async()
            self._message = f"{label} action timed out or was cancelled"
            self._latch_motion_fault(self._message)
            return False
        result = result_future.result()
        with self._lock:
            self._gripper_goal_handle = None
        if self._cancel.is_set():
            self._message = f"{label} cancelled"
            return False
        verified = self._confirm_gripper_result(
            result, opening=label == "open gripper", requested=position,
            action_started=action_started, deadline=action_started + timeout)
        self._message = f"{label}: {self._last_gripper_result_state}"
        return verified

    def _send_arm(self, trajectory: JointTrajectory, label: str) -> bool:
        if self._cancel.is_set():
            self._message = f"{label} cancelled before command"
            return False
        if not self._arm_client.wait_for_server(timeout_sec=3.0):
            self._message = "arm FollowJointTrajectory server unavailable"
            return False
        if self._cancel.is_set():
            self._message = f"{label} cancelled before command"
            return False
        rejection = self._segment_start_rejection(trajectory)
        if rejection is not None:
            self._message = rejection
            return False
        command = copy.deepcopy(trajectory)
        command.header.stamp = self.get_clock().now().to_msg()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = command
        future = self._arm_client.send_goal_async(goal)
        if not self._wait_future(future, 5.0):
            future.add_done_callback(self._cancel_late_goal)
            self._message = f"{label} goal acceptance timed out"
            self._latch_motion_fault(self._message)
            return False
        handle = future.result()
        if handle is None or not handle.accepted:
            self._message = f"{label} goal rejected"
            return False
        with self._lock:
            self._arm_goal_handle = handle
        if self._cancel.is_set():
            handle.cancel_goal_async()
            self._message = f"{label} cancelled before result"
            return False
        result_future = handle.get_result_async()
        timeout = float(self.get_parameter("action_timeout_sec").value)
        if not self._wait_future(result_future, timeout):
            handle.cancel_goal_async()
            self._message = f"{label} action timed out or was cancelled"
            self._latch_motion_fault(self._message)
            return False
        result = result_future.result()
        with self._lock:
            self._arm_goal_handle = None
        succeeded = bool(
            not self._cancel.is_set() and result is not None
            and result.status == GoalStatus.STATUS_SUCCEEDED
            and int(result.result.error_code) == 0
        )
        if not succeeded:
            self._message = f"{label} action failed or was cancelled"
            return False
        terminal = copy.deepcopy(trajectory)
        terminal.points = terminal.points[-1:]
        result_received = time.monotonic()
        deadline = result_received + float(
            self.get_parameter("arm_endpoint_verification_timeout_sec").value)
        while rclpy.ok() and not self._cancel.is_set() and time.monotonic() < deadline:
            with self._lock:
                received = self._joint_state_time
            if received >= result_received and self._segment_start_rejection(terminal) is None:
                return True
            time.sleep(0.02)
        self._message = f"{label} measured endpoint not reached or cancelled"
        return False

    def _execute_worker(self, plan: FrozenPickPlan) -> None:
        success = False
        self._failure_stage = ""
        try:
            if plan.task_mode == "move_to_target":
                self._state = "following_graph_path"
                self._message = "following robot graph edges to target; gripper unchanged"
                self._publish_status()
                if not self._execution_interlocks_ready(plan) or not self._send_arm(
                        plan.trajectory, "graph move to target"):
                    return
                success = True
                self._state = "target_reached"
                self._message = (
                    f"reached robot graph goal near {plan.target_class_name}; gripper unchanged")
                return
            if plan.task_mode != "pickup":
                raise ValueError(f"unsupported_task_mode:{plan.task_mode}")
            if (plan.graph_trajectory is None or plan.approach_trajectory is None or
                    not math.isfinite(plan.gripper_open_position) or
                    not math.isfinite(plan.gripper_close_position)):
                self._message = "pickup_requires_validated_final_approach"
                return
            self._state = "opening_gripper"
            self._message = "opening gripper before graph traversal"
            self._publish_status()
            if not self._validate_pickup_segment(plan, plan.graph_trajectory, False) or not self._send_gripper(
                    plan.gripper_open_position, "open gripper"):
                return
            self._state = "following_graph_path"
            self._message = (
                f"following {len(plan.graph_trajectory.points)} validated pregrasp points")
            self._publish_status()
            if not self._arm_segment_ready(plan, plan.graph_trajectory) \
                    or not self._send_arm(plan.graph_trajectory, "graph pregrasp"):
                return
            self._state = "following_final_approach"
            self._message = "following validated insertion from pregrasp to cluster center"
            self._publish_status()
            if not self._validate_pickup_segment(plan, plan.approach_trajectory, True) \
                    or not self._send_arm(plan.approach_trajectory, "final grasp approach"):
                return
            self._state = "closing_gripper"
            self._message = f"closing gripper on {plan.target_class_name}"
            self._publish_status()
            terminal = copy.deepcopy(plan.approach_trajectory)
            terminal.points = terminal.points[-1:]
            if not self._validate_pickup_segment(plan, terminal, True) or not self._send_gripper(
                    plan.gripper_close_position, "close gripper"):
                return
            if self._cancel.is_set():
                self._message = "pickup cancelled after gripper action"
                return
            success = True
            contact = self._last_gripper_result_state == "contact_detected"
            with self._lock:
                self._holding_object = contact
            if contact:
                self._state = "grasp_contact_detected"
                self._message = (
                    f"gripper contact detected at {plan.target_class_name}; lift not performed")
            else:
                self._state = "gripper_closed_unconfirmed"
                self._message = "gripper reached closed position; object holding is unconfirmed"
        finally:
            with self._lock:
                self._busy = False
                self._arm_goal_handle = None
                self._gripper_goal_handle = None
                if not success:
                    self._failure_stage = self._state
                    self._state = "motion_failed"
            self._publish_status()

    def _on_execute(self, _request, response):
        with self._lock:
            plan = copy.deepcopy(self._frozen)
            if self._busy:
                response.success = False
                response.message = "graph pickup is already active"
                return response
        if plan is None:
            response.success = False
            response.message = "no frozen plan; call /plan_graph_pick first"
            return response
        rejection = self._execution_rejection(plan)
        if rejection:
            response.success = False
            response.message = f"execution rejected: {rejection}"
            return response
        with self._lock:
            if self._busy:
                response.success = False
                response.message = "graph pickup is already active"
                return response
            self._busy = True
            self._holding_object = False
            self._cancel.clear()
            self._state = "execution_starting"
            self._message = f"starting {plan.task_mode} plan {plan.fingerprint[:16]}"
        threading.Thread(
            target=self._execute_worker, args=(plan,), daemon=True).start()
        self._publish_status()
        response.success = True
        response.message = self._message
        return response

    def _on_cancel(self, _request, response):
        self._cancel.set()
        requested = 0
        with self._lock:
            handles = [self._arm_goal_handle, self._gripper_goal_handle]
        for handle in handles:
            if handle is not None:
                handle.cancel_goal_async()
                requested += 1
        with self._lock:
            self._state = "cancel_requested"
            self._message = f"cancellation requested for {requested} active action(s)"
        self._publish_status()
        response.success = True
        response.message = self._message
        return response

    def _monitor_execution_interlocks(self) -> None:
        with self._lock:
            busy = self._busy
            plan = copy.deepcopy(self._frozen)
        if not busy or plan is None or self._cancel.is_set():
            return
        blockers = self._interlock_blockers(plan)
        if not blockers:
            return
        fault_reason = "execution interlock tripped: " + ",".join(blockers)
        self._latch_motion_fault(fault_reason)
        self._cancel.set()
        with self._lock:
            handles = [self._arm_goal_handle, self._gripper_goal_handle]
            self._state = "execution_interlock_tripped"
            self._message = fault_reason
        for handle in handles:
            if handle is not None:
                handle.cancel_goal_async()
        self._publish_status()

    def _on_status(self, _request, response):
        payload = self._plan_payload()
        response.success = bool(payload.get("plan_ready", False))
        response.message = json.dumps(payload, sort_keys=True)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = GraphPickNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
