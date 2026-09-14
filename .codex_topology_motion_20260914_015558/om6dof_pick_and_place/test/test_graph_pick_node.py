import json
import math
import threading
import time
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from om6dof_dd_gng.msg import (EnvironmentGraph, EnvironmentNode,
                               ReachabilityPlan, TopologyEdge)
from om6dof_pick_and_place.graph_pick_node import (
    GraphPickNode, _diagnostic_level, approach_alignment, matching_object_track,
    motion_fingerprint,
    ordered_joint_positions, reverse_trajectory, matching_environment_snapshot,
    parse_object_tracks, semantic_component, trajectory_fingerprint,
    validate_trajectory)


JOINTS = [f"joint{index}" for index in range(1, 7)]


def test_diagnostic_level_accepts_ros_uint8_byte_and_integer_forms():
    assert _diagnostic_level(b"\x00") == 0
    assert _diagnostic_level(bytearray([2])) == 2
    assert _diagnostic_level(1) == 1
    with pytest.raises(ValueError, match="exactly one byte"):
        _diagnostic_level(b"")


def _node(node_id, class_id, confidence, xyz):
    node = EnvironmentNode()
    node.id = node_id
    node.class_id = class_id
    node.confidence = confidence
    node.position.x, node.position.y, node.position.z = xyz
    return node


def _edge(source, target):
    edge = TopologyEdge()
    edge.source_id = source
    edge.target_id = target
    edge.cost = 1.0
    return edge


def _trajectory(points):
    trajectory = JointTrajectory()
    trajectory.joint_names = JOINTS
    for seconds, values in points:
        point = JointTrajectoryPoint()
        point.positions = list(values)
        point.time_from_start.sec = int(seconds)
        point.time_from_start.nanosec = int(round(
            (seconds - int(seconds)) * 1.0e9))
        trajectory.points.append(point)
    return trajectory


def test_semantic_component_uses_same_class_edges_and_weighted_centroid():
    environment = EnvironmentGraph()
    environment.nodes = [
        _node(10, 5, 1.0, (0.0, 0.0, 0.0)),
        _node(11, 5, 3.0, (0.4, 0.0, 0.0)),
        _node(12, 5, 1.0, (9.0, 0.0, 0.0)),
        _node(13, -1, 1.0, (0.2, 0.0, 0.0)),
    ]
    environment.edges = [_edge(10, 11), _edge(11, 13), _edge(12, 13)]

    class_id, component, centroid = semantic_component(environment, 10)

    assert class_id == 5
    assert [node.id for node in component] == [10, 11]
    assert centroid == pytest.approx((0.3, 0.0, 0.0))


def test_semantic_component_rejects_unselected_target():
    environment = EnvironmentGraph()
    environment.nodes = [_node(10, -1, 0.0, (0.0, 0.0, 0.0))]
    with pytest.raises(ValueError, match="no longer a selected semantic"):
        semantic_component(environment, 10)


def test_matching_environment_uses_recent_snapshot_containing_plan_target():
    matched = EnvironmentGraph()
    matched.nodes = [_node(10, 5, 0.9, (0.0, 0.0, 0.0))]
    newest = EnvironmentGraph()
    newest.nodes = [_node(11, 5, 0.9, (0.0, 0.0, 0.0))]
    history = [(9.2, matched), (9.9, newest)]
    assert matching_environment_snapshot(
        history, 10, plan_received=10.0, now=10.0, max_age=1.0) is matched
    assert matching_environment_snapshot(
        history, 10, plan_received=10.0, now=10.3, max_age=1.0) is None


def test_approach_alignment_uses_goal_orientation_and_tool_axis():
    goal = Pose()
    goal.orientation.w = 1.0
    assert approach_alignment(goal, (0.0, 0.0, 0.1), (0.0, 0.0, 1.0)) \
        == pytest.approx(1.0)
    assert approach_alignment(goal, (0.0, 0.0, -0.1), (0.0, 0.0, 1.0)) \
        == pytest.approx(-1.0)


def test_joint_ordering_requires_complete_finite_state():
    message = JointState()
    message.name = list(reversed(JOINTS))
    message.position = [6.0, 5.0, 4.0, 3.0, 2.0, 1.0]
    assert ordered_joint_positions(message, JOINTS) == pytest.approx(
        [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    message.position[-1] = math.nan
    assert ordered_joint_positions(message, JOINTS) is None


def test_trajectory_validation_and_reverse_retime():
    trajectory = _trajectory([
        (0.0, [0.0] * 6),
        (1.0, [0.2] * 6),
        (2.0, [0.5] * 6),
    ])
    assert validate_trajectory(trajectory, JOINTS) is None

    retreat = reverse_trajectory(
        trajectory, velocity=0.25, minimum_segment_time=0.1)
    assert retreat.points[0].positions == pytest.approx([0.5] * 6)
    assert retreat.points[-1].positions == pytest.approx([0.0] * 6)
    assert retreat.points[0].time_from_start.sec == 0
    assert retreat.points[1].time_from_start.sec == 1
    assert retreat.points[2].time_from_start.sec == 2
    assert retreat.points[2].time_from_start.nanosec == 0


def test_trajectory_validation_rejects_nonincreasing_time():
    trajectory = _trajectory([
        (0.0, [0.0] * 6),
        (0.0, [0.1] * 6),
    ])
    assert "not_strictly_increasing" in validate_trajectory(
        trajectory, JOINTS)


def test_trajectory_fingerprint_binds_revision_target_nodes_and_joints():
    plan = ReachabilityPlan()
    plan.graph_revision = 7
    plan.target_environment_node_id = 42
    plan.reachability_node_ids = [1, 4, 9]
    plan.joint_path_preview = _trajectory([
        (0.0, [0.0] * 6),
        (1.0, [0.1] * 6),
    ])
    original = trajectory_fingerprint(plan)
    plan.joint_path_preview.points[-1].positions[-1] = 0.2
    assert trajectory_fingerprint(plan) != original


def test_motion_fingerprint_allows_semantic_node_churn_only():
    plan = ReachabilityPlan()
    plan.graph_revision = 7
    plan.target_environment_node_id = 42
    plan.reachability_node_ids = [1, 4, 9]
    plan.joint_path_preview = _trajectory([
        (0.0, [0.0] * 6),
        (1.0, [0.1] * 6),
    ])
    original = motion_fingerprint(plan)
    plan.target_environment_node_id = 84
    assert motion_fingerprint(plan) == original
    plan.reachability_node_ids[-1] = 10
    assert motion_fingerprint(plan) != original


def test_object_track_parser_and_node_match_survive_ddgng_id_churn():
    observations = parse_object_tracks(json.dumps([{
        "track_id": 12,
        "class_id": 39,
        "class": "bottle",
        "centroid": {"x": 0.30, "y": 0.01, "z": 0.20},
        "tracked_centroid": {"x": 0.31, "y": 0.01, "z": 0.20},
        "node_ids": [100, 101],
    }]))
    matched = matching_object_track(
        [(9.9, observations)], 101, 39, (0.30, 0.01, 0.20),
        plan_received=10.0, now=10.0, max_age=1.0,
        max_association_distance=0.08)
    assert matched.track_id == 12

    # A replacement DD-GNG node still binds to the same physical track by
    # class and component centroid.
    matched = matching_object_track(
        [(9.9, observations)], 999, 39, (0.32, 0.01, 0.20),
        plan_received=10.0, now=10.0, max_age=1.0,
        max_association_distance=0.08)
    assert matched.track_id == 12


def test_object_track_match_rejects_wrong_class_or_large_jump():
    observations = parse_object_tracks(json.dumps([{
        "track_id": 12,
        "class_id": 39,
        "class": "bottle",
        "centroid": {"x": 0.30, "y": 0.01, "z": 0.20},
        "node_ids": [100],
    }]))
    assert matching_object_track(
        [(9.9, observations)], 999, 41, (0.30, 0.01, 0.20),
        plan_received=10.0, now=10.0, max_age=1.0,
        max_association_distance=0.08) is None
    assert matching_object_track(
        [(9.9, observations)], 999, 39, (0.60, 0.01, 0.20),
        plan_received=10.0, now=10.0, max_age=1.0,
        max_association_distance=0.08) is None


def test_execution_revalidation_accepts_node_churn_but_rejects_motion_change():
    node = object.__new__(GraphPickNode)
    node._faulted = False
    node.execution_enabled = True
    parameters = {
        "max_plan_hold_sec": 5.0,
        "max_object_track_shift_m": 0.02,
    }
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    node._interlock_blockers = lambda _plan: []
    frozen = SimpleNamespace(
        created_monotonic=time.monotonic(), blockers=[], target_class_id=39,
        target_track_id=7, tracked_centroid=(0.30, 0.0, 0.20),
        motion_fingerprint="same-motion", fingerprint="node-100")
    current = SimpleNamespace(
        blockers=[], target_class_id=39, target_track_id=7,
        tracked_centroid=(0.305, 0.0, 0.20),
        motion_fingerprint="same-motion", fingerprint="node-999")
    node._build_plan = lambda: current
    assert GraphPickNode._execution_rejection(node, frozen) is None

    current.motion_fingerprint = "different-motion"
    assert GraphPickNode._execution_rejection(node, frozen) == \
        "live_motion_path_changed; request a new preview"


def test_execution_revalidation_rejects_track_change_and_motion_over_tolerance():
    node = object.__new__(GraphPickNode)
    node._faulted = False
    node.execution_enabled = True
    parameters = {
        "max_plan_hold_sec": 5.0,
        "max_object_track_shift_m": 0.02,
    }
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    node._interlock_blockers = lambda _plan: []
    frozen = SimpleNamespace(
        created_monotonic=time.monotonic(), blockers=[], target_class_id=39,
        target_track_id=7, tracked_centroid=(0.30, 0.0, 0.20),
        motion_fingerprint="same-motion")
    current = SimpleNamespace(
        blockers=[], target_class_id=39, target_track_id=8,
        tracked_centroid=(0.30, 0.0, 0.20),
        motion_fingerprint="same-motion")
    node._build_plan = lambda: current
    assert GraphPickNode._execution_rejection(node, frozen) == \
        "semantic_object_track_changed; request a new preview"

    current.target_track_id = 7
    current.tracked_centroid = (0.33, 0.0, 0.20)
    assert GraphPickNode._execution_rejection(node, frozen).startswith(
        "semantic_object_moved:")


def test_execution_rejection_preserves_latched_motion_fault_reason():
    node = object.__new__(GraphPickNode)
    node._faulted = True
    node._fault_reason = "open gripper goal acceptance timed out"

    rejection = GraphPickNode._execution_rejection(node, object())

    assert rejection == (
        "motion_faulted:open gripper goal acceptance timed out; restart the "
        "graph-pick node after checking hardware")


def _interlock_node():
    node = object.__new__(GraphPickNode)
    node._lock = threading.RLock()
    node._operation_mode = "CARTESIAN"
    node._remote_enabled = True
    node.joint_names = JOINTS
    node._controller_snapshot_time = time.monotonic()
    node._controller_snapshot = [
        SimpleNamespace(name="arm_controller", state="active",
                        type="joint_trajectory_controller/JointTrajectoryController",
                        claimed_interfaces=[f"{j}/position" for j in JOINTS]),
        SimpleNamespace(name="forward_offset_controller", state="active",
                        type="forward_command_controller/ForwardCommandController",
                        claimed_interfaces=[f"{j}/position_offset" for j in JOINTS]),
    ]
    node._controller_query_future = None
    node._controller_query_started = 0.0
    node._bus_health_time = time.monotonic()
    node._bus_health = {
        "level": 0,
        "message": "OK",
        "read_failure_count": 4,
        "write_failure_count": 2,
        "consecutive_read_failures": 0,
        "consecutive_write_failures": 0,
        "current_read_error": 0,
        "current_write_error": 0,
        "fail_safe_triggered": False,
        "torque_all_enabled": True,
        "hardware_error_mask": 0,
    }
    parameters = {
        "arm_controller_name": "arm_controller",
        "offset_controller_name": "forward_offset_controller",
        "controller_state_timeout_sec": 1.5,
        "dynamixel_health_topic": "/health",
        "dynamixel_health_timeout_sec": 0.3,
    }
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    node.count_publishers = lambda _topic: 1
    return node


def test_additive_execution_accepts_cartesian_and_remote_enabled():
    node = _interlock_node()
    plan = SimpleNamespace(bus_read_failures=4, bus_write_failures=2)
    assert GraphPickNode._interlock_blockers(node, plan) == []


@pytest.mark.parametrize("state", ["inactive", "unconfigured"])
def test_additive_execution_requires_active_offset_controller(state):
    node = _interlock_node()
    node._controller_snapshot[1].state = state
    assert "required_controller_not_active:forward_offset_controller" in node._interlock_blockers()


def test_additive_execution_rejects_absolute_interface_on_offset_channel():
    node = _interlock_node()
    node._controller_snapshot[1].claimed_interfaces = [f"{j}/position" for j in JOINTS]
    blockers = node._interlock_blockers()
    assert "controller_command_interfaces_mismatch:forward_offset_controller" in blockers
    assert "controller_command_interfaces_conflict:arm_controller" in blockers


@pytest.mark.parametrize("missing", [True, False])
def test_additive_execution_rejects_unavailable_topology(missing):
    node = _interlock_node()
    if missing:
        node._controller_snapshot = None
    else:
        node._controller_snapshot_time -= 2.0
    assert "controller_state_missing_or_stale" in node._interlock_blockers()


def test_additive_execution_still_rejects_bad_bus():
    node = _interlock_node()
    node._bus_health["torque_all_enabled"] = False
    assert any(b.startswith("dynamixel_bus_not_healthy:") for b in node._interlock_blockers())


@pytest.mark.parametrize("mutation, expected", [
    ("missing", "required_controller_not_active:arm_controller"),
    ("duplicate", "controller_snapshot_ambiguous"),
    ("wrong_type", "controller_type_mismatch:arm_controller"),
    ("missing_joint", "controller_command_interfaces_mismatch:arm_controller"),
])
def test_additive_execution_rejects_invalid_topology(mutation, expected):
    node = _interlock_node()
    if mutation == "missing":
        node._controller_snapshot.pop(0)
    elif mutation == "duplicate":
        node._controller_snapshot.append(node._controller_snapshot[0])
    elif mutation == "wrong_type":
        node._controller_snapshot[0].type = "other"
    else:
        node._controller_snapshot[0].claimed_interfaces.pop()
    assert expected in node._interlock_blockers()


def test_controller_query_timeout_and_late_reply_cannot_refresh_snapshot():
    from concurrent.futures import Future
    node = _interlock_node()
    old, new = Future(), Future()
    removed = []
    node._controller_client = SimpleNamespace(
        service_is_ready=lambda: True, call_async=lambda _: new,
        remove_pending_request=lambda future: removed.append(future))
    node._controller_query_future = old
    node._controller_query_started = time.monotonic() - 2.0
    node._poll_controllers()
    assert old.cancelled() and removed == [old]
    assert node._controller_snapshot is None
    assert node._controller_query_future is new
    node._on_controllers(old)
    assert node._controller_query_future is new
    new.set_result(SimpleNamespace(controller=[]))
    assert node._controller_snapshot == []
    assert node._controller_query_future is None


def test_slow_controller_response_is_not_treated_as_fresh():
    from concurrent.futures import Future
    node = _interlock_node()
    future = Future()
    node._controller_query_future = future
    node._controller_query_started = time.monotonic() - 2.0
    future.set_result(SimpleNamespace(controller=node._controller_snapshot))
    node._on_controllers(future)
    assert "controller_state_missing_or_stale" in node._interlock_blockers()


def test_controller_poll_has_only_one_outstanding_request():
    from concurrent.futures import Future
    node = _interlock_node()
    node._controller_query_future = Future()
    node._controller_query_started = time.monotonic()
    node._poll_controllers()  # No client needed: must return before a second query.


def test_execution_interlock_rejects_bus_failure_after_preview():
    node = _interlock_node()
    plan = SimpleNamespace(bus_read_failures=3, bus_write_failures=2)
    assert "dynamixel_failure_counter_advanced_since_preview" in \
        GraphPickNode._interlock_blockers(node, plan)
