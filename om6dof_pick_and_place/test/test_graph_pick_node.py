import copy
import json
import math
import threading
import time
from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from om6dof_dd_gng.msg import (EnvironmentGraph, EnvironmentNode,
                               ReachabilityPlan, TopologyEdge)
from om6dof_pick_and_place.graph_pick_node import (
    GraphPickNode, _diagnostic_level, approach_alignment, matching_object_track,
    motion_fingerprint,
    ordered_joint_positions, reverse_trajectory, matching_environment_snapshot,
    parse_object_tracks, semantic_component, component_bounding_center, trajectory_fingerprint,
    validate_trajectory, split_grasp_trajectory, validate_grasp_contract,
    validate_goal_alignment,
    gripper_result_state, GRASP_VALIDATED_PLAN_REASON)


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


def test_component_bounding_center_is_object_center_not_one_surface_node():
    component = [
        _node(10, 39, 1.0, (0.1, -0.1, 0.0)),
        _node(11, 39, 1.0, (0.3, 0.2, 0.4)),
        _node(12, 39, 1.0, (0.2, 0.0, 0.2)),
    ]
    assert component_bounding_center(component) == pytest.approx((0.2, 0.05, 0.2))


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


def test_approach_arrow_uses_tool_x_not_world_vertical_direction():
    from om6dof_pick_and_place.graph_pick_node import approach_axis_endpoint
    goal = Pose()
    goal.orientation.w = 1.0
    endpoint = approach_axis_endpoint(goal, (0.0, 0.0, -0.10), (1.0, 0.0, 0.0))
    assert endpoint == pytest.approx((0.10, 0.0, 0.0))


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


def test_move_to_target_does_not_require_grasp_orientation():
    from om6dof_pick_and_place.graph_pick_node import validate_goal_alignment
    validate_goal_alignment("move_to_target", 0.208489, 0.3)
    with pytest.raises(ValueError, match="gripper_approach_misaligned"):
        validate_goal_alignment("pickup", 0.208489, 0.3)
    with pytest.raises(ValueError, match="unsupported_task_mode"):
        validate_goal_alignment("unknown", 1.0, 0.3)


def test_shortcut_plan_reason_is_still_exact_target_protected():
    from om6dof_pick_and_place.graph_pick_node import (
        TARGET_PROTECTED_PLAN_REASONS, VALIDATED_PLAN_REASONS)
    assert "path_ready_exact_validated_target_protected_shortcut_preview_only" \
        in TARGET_PROTECTED_PLAN_REASONS
    assert "path_ready_exact_validated_shortcut_preview_only" in VALIDATED_PLAN_REASONS


@pytest.mark.parametrize("allowed, arm_success", [(True, True), (False, True), (True, False)])
def test_move_to_target_follows_frozen_trajectory_without_gripper_actions(allowed, arm_success):
    node = object.__new__(GraphPickNode)
    node._lock = threading.RLock()
    node._busy = True
    node._holding_object = False
    node._publish_status = lambda: None
    node._execution_interlocks_ready = lambda _: allowed
    sent = []
    def arm(trajectory, label):
        sent.append(trajectory)
        return arm_success
    node._send_arm = arm
    node._send_gripper = lambda *_: pytest.fail("move-to-target must not command gripper")
    plan = SimpleNamespace(task_mode="move_to_target", trajectory=object(), target_class_name="bottle")
    node._execute_worker(plan)
    assert sent == ([plan.trajectory] if allowed else [])
    assert node._state == ("target_reached" if allowed and arm_success else "motion_failed")
    assert not node._busy and not node._holding_object


def _grasp_message():
    path = Path()
    path.header.frame_id = "world"
    for x in (0.0, 0.10, 0.20, 0.25, 0.30):
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = x
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return SimpleNamespace(
        reason=GRASP_VALIDATED_PLAN_REASON, grasp_approach_valid=True,
        selected_target_excluded_from_collision=False,
        pregrasp_waypoint_count=3, grasp_target_position=Point(x=0.30),
        grasp_position_error=0.0, gripper_open_position=0.019,
        gripper_close_position=-0.010, end_effector_path=path,
        joint_path_preview=_trajectory([(0.0, [0.0] * 6), (1.0, [0.1] * 6),
                                       (2.0, [0.2] * 6), (2.5, [0.25] * 6),
                                       (3.0, [0.3] * 6)]))


def _validate_grasp(message, center=(0.30, 0.0, 0.0), offset=(0.0, 0.0, 0.0),
                    target_excluded=False):
    return validate_grasp_contract(
        message, center, 0.019, -0.010, 0.01, 0.005, offset, target_excluded)


@pytest.mark.parametrize("expected,reported", [(False, False), (True, True)])
def test_grasp_contract_accepts_only_matching_selected_target_policy(expected, reported):
    message = _grasp_message()
    message.selected_target_excluded_from_collision = reported
    assert _validate_grasp(message, target_excluded=expected)[0] == 3
    with pytest.raises(ValueError, match="selected_target_collision_policy_mismatch"):
        _validate_grasp(message, target_excluded=not expected)


def test_grasp_exclusion_requires_explicit_new_planner_contract():
    message = _grasp_message()
    del message.selected_target_excluded_from_collision
    # Missing means legacy protected behavior, not permission to exclude a target.
    assert _validate_grasp(message)[0] == 3
    with pytest.raises(ValueError, match="selected_target_collision_policy_mismatch"):
        _validate_grasp(message, target_excluded=True)


@pytest.mark.parametrize("fingerprint", [trajectory_fingerprint, motion_fingerprint])
def test_grasp_fingerprint_binds_selected_target_collision_policy(fingerprint):
    message = _grasp_message()
    message.graph_revision = 4
    message.target_environment_node_id = 10
    message.reachability_node_ids = [1, 2, 3]
    original = fingerprint(message)
    message.selected_target_excluded_from_collision = True
    assert fingerprint(message) != original


def test_grasp_split_shares_boundary_rebases_timing_and_preserves_full_path():
    source = _grasp_message().joint_path_preview
    original = copy.deepcopy(source)
    graph, approach = split_grasp_trajectory(source, 3)
    assert graph.points[-1].positions == approach.points[0].positions
    assert len(graph.points) == 3 and len(approach.points) == 3
    assert validate_trajectory(graph, JOINTS) is None
    assert validate_trajectory(approach, JOINTS) is None
    assert approach.points[0].time_from_start.sec == 0
    assert approach.points[-1].time_from_start.sec == 1
    assert source == original


def test_grasp_prefix_can_include_validated_bridge_beyond_recorded_graph_nodes():
    message = _grasp_message()
    # Three archived graph positions followed by two bridge positions, then
    # insertion. The split is at the true aligned pregrasp, not the last node ID.
    positions = [(0., 0.), (.1, .05), (.17, .05), (.185, .025),
                 (.2, 0.), (.25, 0.), (.3, 0.)]
    message.end_effector_path.poses = []
    for x, y in positions:
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.w = 1.0
        message.end_effector_path.poses.append(pose)
    message.joint_path_preview = _trajectory(
        [(float(index), [x] * 6) for index, (x, _) in enumerate(positions)])
    message.reachability_node_ids = [4, 9]
    message.pregrasp_waypoint_count = 5
    count, target = _validate_grasp(message)
    assert count > len(message.reachability_node_ids) + 1
    graph, insertion = split_grasp_trajectory(message.joint_path_preview, count)
    assert len(graph.points) == 5 and len(insertion.points) == 3
    assert graph.points[-1].positions == insertion.points[0].positions
    assert validate_trajectory(graph, JOINTS) is None
    assert validate_trajectory(insertion, JOINTS) is None
    # The old archived anchor is misaligned; the bridge reaches a valid pose.
    anchor = message.end_effector_path.poses[2].pose
    pregrasp = message.end_effector_path.poses[count - 1].pose
    assert approach_alignment(anchor, target, [1., 0., 0.]) < .95
    validate_goal_alignment("pickup", approach_alignment(pregrasp, target, [1., 0., 0.]), .95)


@pytest.mark.parametrize("count", [0, 1, 5, 8])
def test_grasp_split_rejects_missing_graph_or_missing_insertion(count):
    with pytest.raises(ValueError, match="waypoint_count_invalid"):
        split_grasp_trajectory(_grasp_message().joint_path_preview, count)


def test_grasp_contract_binds_authoritative_center_not_refreshed_snapshot():
    message = _grasp_message()
    count, target = _validate_grasp(message, center=(0.305, 0.0, 0.0))
    assert count == 3
    assert target == (0.30, 0.0, 0.0)
    with pytest.raises(ValueError, match="snapshot_mismatch"):
        _validate_grasp(message, center=(0.32, 0.0, 0.0))


def test_grasp_contract_applies_rotated_tcp_to_pinch_offset():
    message = _grasp_message()
    final = message.end_effector_path.poses[-1].pose
    final.position.x = 0.28
    final.orientation.y = math.sin(math.pi / 4)
    final.orientation.w = math.cos(math.pi / 4)
    # A 20 mm local +Z offset rotates into world +X.
    assert _validate_grasp(message, offset=(0.0, 0.0, 0.02))[0] == 3
    with pytest.raises(ValueError, match="endpoint_position_error"):
        _validate_grasp(message)


@pytest.mark.parametrize("field,value,error", [
    ("reason", "path_ready_exact_validated_target_protected_preview_only", "requires_validated"),
    ("grasp_approach_valid", False, "requires_validated"),
    ("pregrasp_waypoint_count", 0, "waypoint_count"),
    ("grasp_position_error", float("nan"), "position_error"),
    ("grasp_position_error", 0.015, "position_error"),
    ("gripper_open_position", 0.03, "gripper_open_position_mismatch"),
    ("gripper_close_position", -0.02, "gripper_close_position_mismatch"),
])
def test_grasp_contract_rejects_unsafe_or_legacy_payload(field, value, error):
    message = _grasp_message()
    setattr(message, field, value)
    with pytest.raises(ValueError, match=error):
        _validate_grasp(message)


def test_grasp_contract_fails_closed_with_old_message_fields():
    message = _grasp_message()
    del message.grasp_approach_valid
    with pytest.raises(ValueError, match="requires_validated"):
        _validate_grasp(message)


def test_grasp_contract_checks_endpoint_instead_of_trusting_reported_error():
    message = _grasp_message()
    message.end_effector_path.poses[-1].pose.position.x = 0.27
    with pytest.raises(ValueError, match="endpoint_position_error"):
        _validate_grasp(message)
    message.end_effector_path.poses.pop()
    with pytest.raises(ValueError, match="path_size_mismatch"):
        _validate_grasp(message)


@pytest.mark.parametrize("opening,reached,stalled,position,status,expected", [
    (True, True, False, 0.019, GoalStatus.STATUS_SUCCEEDED, "open_reached"),
    (True, False, True, 0.010, GoalStatus.STATUS_SUCCEEDED, "open_not_reached"),
    (True, True, True, 0.019, GoalStatus.STATUS_SUCCEEDED, "open_not_reached"),
    (True, True, False, 0.0, GoalStatus.STATUS_SUCCEEDED, "open_not_reached"),
    (False, False, True, 0.005, GoalStatus.STATUS_SUCCEEDED, "contact_detected"),
    (False, True, False, -0.010, GoalStatus.STATUS_SUCCEEDED, "closed_unconfirmed"),
    (False, False, False, 0.005, GoalStatus.STATUS_SUCCEEDED, "close_not_confirmed"),
    (False, False, True, -0.010, GoalStatus.STATUS_SUCCEEDED, "close_not_confirmed"),
    (False, True, False, float("nan"), GoalStatus.STATUS_SUCCEEDED, "position_invalid"),
    (False, False, True, 0.005, GoalStatus.STATUS_ABORTED, "action_failed"),
])
def test_gripper_feedback_distinguishes_opening_contact_and_empty_closure(
        opening, reached, stalled, position, status, expected):
    result = SimpleNamespace(status=status, result=SimpleNamespace(
        position=position, reached_goal=reached, stalled=stalled))
    assert gripper_result_state(result, opening, 0.019 if opening else -0.010,
                                -0.010, 0.001) == expected


def _pickup_worker_node(gripper_result="contact_detected", arm_failure=None,
                        cancel_after=None):
    node = object.__new__(GraphPickNode)
    node._lock = threading.RLock()
    node._cancel = threading.Event()
    node._busy = True
    node._holding_object = False
    node._faulted = False
    node._publish_status = lambda: None
    node._execution_interlocks_ready = lambda _: not node._cancel.is_set()
    node._pickup_target_rejection = lambda _: None
    node._validate_pickup_segment = lambda plan, trajectory, closure: node._arm_segment_ready(
        plan, trajectory)
    node.joint_names = JOINTS
    node._joint_state = JointState(name=JOINTS, position=[0.0] * 6)
    node._joint_state_time = time.monotonic()
    parameters = {"max_joint_state_age_sec": 0.5, "max_start_joint_error_rad": 0.01}
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    calls = []

    def arm(trajectory, label):
        calls.append(label)
        if label == arm_failure:
            return False
        node._joint_state.position = list(trajectory.points[-1].positions)
        node._joint_state_time = time.monotonic()
        if label == cancel_after:
            node._cancel.set()
        return True

    def gripper(position, label):
        calls.append(label)
        node._last_gripper_result_state = ("open_reached" if label == "open gripper"
                                            else gripper_result)
        return True

    node._send_arm = arm
    node._send_gripper = gripper
    message = _grasp_message()
    graph, approach = split_grasp_trajectory(message.joint_path_preview, 3)
    plan = SimpleNamespace(task_mode="pickup", graph_trajectory=graph,
                           approach_trajectory=approach, gripper_open_position=0.019,
                           gripper_close_position=-0.010, target_class_name="bottle",
                           target_position=(0.3, 0.0, 0.0), target_class_id=39,
                           target_node_id=10)
    return node, plan, calls


def test_pickup_executes_graph_then_insertion_then_close_with_new_segment_start():
    node, plan, calls = _pickup_worker_node()
    node._execute_worker(plan)
    assert calls == ["open gripper", "graph pregrasp", "final grasp approach", "close gripper"]
    assert node._state == "grasp_contact_detected"
    assert node._holding_object
    assert not node._busy
    # The original start is now wrong, yet insertion proceeded from its own start.
    assert node._segment_start_rejection(plan.graph_trajectory).startswith("arm_segment_start_mismatch")


@pytest.mark.parametrize("stage", ["graph pregrasp", "final grasp approach"])
def test_pickup_arm_failure_never_closes_or_retreats(stage):
    node, plan, calls = _pickup_worker_node(arm_failure=stage)
    node._execute_worker(plan)
    assert "close gripper" not in calls
    assert not node._holding_object
    assert node._state == "motion_failed"


@pytest.mark.parametrize("stage", ["graph pregrasp", "final grasp approach"])
def test_pickup_cancellation_never_closes_even_if_arm_returns_success(stage):
    node, plan, calls = _pickup_worker_node(cancel_after=stage)
    node._execute_worker(plan)
    assert "close gripper" not in calls
    assert not node._holding_object


def test_pickup_reaching_empty_closed_position_does_not_claim_holding():
    node, plan, calls = _pickup_worker_node(gripper_result="closed_unconfirmed")
    node._execute_worker(plan)
    assert calls[-1] == "close gripper"
    assert node._state == "gripper_closed_unconfirmed"
    assert not node._holding_object


def test_pickup_stale_joint_state_stops_before_opening():
    node, plan, calls = _pickup_worker_node()
    node._joint_state_time -= 1.0
    node._execute_worker(plan)
    assert not calls
    assert node._message == "joint_state_missing_or_stale"


def test_pickup_drift_at_new_segment_start_stops_before_insertion():
    node, plan, calls = _pickup_worker_node()
    send_arm = node._send_arm

    def disturbed(trajectory, label):
        result = send_arm(trajectory, label)
        node._joint_state.position[0] += 0.10
        return result

    node._send_arm = disturbed
    node._execute_worker(plan)
    assert calls == ["open gripper", "graph pregrasp"]
    assert node._message.startswith("arm_segment_start_mismatch")


def test_cancelled_completed_future_does_not_count_as_success(monkeypatch):
    from concurrent.futures import Future
    import om6dof_pick_and_place.graph_pick_node as module
    monkeypatch.setattr(module.rclpy, "ok", lambda: True)
    node = object.__new__(GraphPickNode)
    node._cancel = threading.Event()
    node._cancel.set()
    future = Future()
    future.set_result(object())
    assert not node._wait_future(future, 1.0)


def _measured_gripper_node(monkeypatch, measurement, refresh=True):
    from collections import deque
    import om6dof_pick_and_place.graph_pick_node as module
    clock = [10.0]
    node = object.__new__(GraphPickNode)
    node._lock = threading.RLock()
    node._cancel = threading.Event()
    node._gripper_measurements = deque([(9.5, measurement), (9.75, measurement),
                                       (9.99, measurement)], maxlen=512)
    parameters = {
        "gripper_position_tolerance": 0.0015,
        "gripper_result_match_tolerance": 0.002,
        "gripper_contact_position_margin": 0.002,
        "gripper_stationary_time_sec": 0.25,
        "gripper_stationary_tolerance": 0.0005,
        "gripper_verification_timeout_sec": 0.5,
        "max_joint_state_age_sec": 0.1,
    }
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    monkeypatch.setattr(module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])

    def advance(seconds):
        clock[0] += seconds
        if refresh:
            node._gripper_measurements.append((clock[0], measurement))

    monkeypatch.setattr(module.time, "sleep", advance)
    return node, clock


def test_legacy_early_open_success_waits_for_fresh_measured_open_aperture(monkeypatch):
    node, clock = _measured_gripper_node(monkeypatch, 0.019)
    result = SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED,
                             result=SimpleNamespace(position=0.010,
                                                    reached_goal=True, stalled=False))
    assert node._confirm_gripper_result(result, True, 0.019, 9.0, 11.0)
    assert clock[0] > 10.0  # A result callback alone cannot confirm physical opening.
    assert node._last_gripper_result_state == "open_reached"


def test_reported_open_goal_with_inadequate_actual_aperture_cannot_start_arm(monkeypatch):
    node, _ = _measured_gripper_node(monkeypatch, 0.010)
    result = SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED,
                             result=SimpleNamespace(position=0.019,
                                                    reached_goal=True, stalled=False))
    assert not node._confirm_gripper_result(result, True, 0.019, 9.0, 11.0)


@pytest.mark.parametrize("status", [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED])
def test_stalled_close_requires_independent_stationary_encoder_contact(monkeypatch, status):
    node, _ = _measured_gripper_node(monkeypatch, 0.005)
    result = SimpleNamespace(status=status, result=SimpleNamespace(
        position=0.005, reached_goal=False, stalled=True))
    assert node._confirm_gripper_result(result, False, -0.010, 9.0, 11.0)
    assert node._last_gripper_result_state == "contact_detected"


@pytest.mark.parametrize("mutation", ["stale", "disagreement", "no_stall", "moving"])
def test_aborted_close_without_independent_contact_evidence_is_rejected(monkeypatch, mutation):
    node, clock = _measured_gripper_node(monkeypatch,
                                        0.009 if mutation == "disagreement" else 0.005,
                                        refresh=mutation != "stale")
    result = SimpleNamespace(status=GoalStatus.STATUS_ABORTED, result=SimpleNamespace(
        position=0.005, reached_goal=False, stalled=mutation != "no_stall"))
    if mutation == "moving":
        import om6dof_pick_and_place.graph_pick_node as module

        def advance(seconds):
            clock[0] += seconds
            measured = 0.005 + (0.001 if len(node._gripper_measurements) % 2 else 0.0)
            node._gripper_measurements.append((clock[0], measured))

        monkeypatch.setattr(module.time, "sleep", advance)
        node._gripper_measurements.clear()
    assert not node._confirm_gripper_result(result, False, -0.010, 9.0, 11.0)


def test_goal_accepted_after_timeout_is_cancelled():
    from concurrent.futures import Future
    node = object.__new__(GraphPickNode)
    cancellations = []
    handle = SimpleNamespace(accepted=True,
                             cancel_goal_async=lambda: cancellations.append(True))
    future = Future()
    future.add_done_callback(node._cancel_late_goal)
    future.set_result(handle)
    assert cancellations == [True]


def _service_validator_node(monkeypatch, valid=True):
    from builtin_interfaces.msg import Time
    from concurrent.futures import Future
    import om6dof_pick_and_place.graph_pick_node as module
    node, plan, _ = _pickup_worker_node()
    del node._validate_pickup_segment
    node.world_frame = "world"
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time))
    node.get_parameter = lambda name: SimpleNamespace(value=30.0)
    monkeypatch.setattr(module, "ValidateGraspExecution", SimpleNamespace(Request=SimpleNamespace))
    node._arm_segment_ready = lambda *_: not node._cancel.is_set()
    node._wait_future = lambda future, timeout: future.done()
    requests = []

    def call(request):
        requests.append(request)
        future = Future()
        future.set_result(SimpleNamespace(valid=valid, reason="blocked_by_new_obstacle"))
        return future

    node._grasp_validate_client = SimpleNamespace(
        wait_for_service=lambda **_: True, call_async=call)
    return node, plan, requests


def test_stage_revalidation_sends_frozen_segment_target_and_closure_flag(monkeypatch):
    node, plan, requests = _service_validator_node(monkeypatch)
    assert node._validate_pickup_segment(plan, plan.approach_trajectory, True)
    request = requests[0]
    assert request.include_closure
    assert request.target_position.x == 0.3 and request.target_class_id == 39
    assert request.target_environment_node_id == 10
    assert request.gripper_open_position == 0.019
    assert request.gripper_close_position == -0.010
    assert request.trajectory.points == plan.approach_trajectory.points
    assert request.trajectory.header.frame_id == "world"


def test_stage_revalidation_rejects_new_collision_without_sending_motion(monkeypatch):
    node, plan, requests = _service_validator_node(monkeypatch, valid=False)
    assert not node._validate_pickup_segment(plan, plan.approach_trajectory, True)
    assert "blocked_by_new_obstacle" in node._message
    assert len(requests) == 1


def test_stage_revalidation_rechecks_start_after_service_reply(monkeypatch):
    node, plan, _ = _service_validator_node(monkeypatch)
    readiness = iter([True, False])
    node._arm_segment_ready = lambda *_: next(readiness)
    assert not node._validate_pickup_segment(plan, plan.approach_trajectory, True)


def test_missing_stage_validator_cannot_fall_back_to_preview_only(monkeypatch):
    node, plan, requests = _service_validator_node(monkeypatch)
    node._grasp_validate_client.wait_for_service = lambda **_: False
    assert not node._validate_pickup_segment(plan, plan.graph_trajectory, False)
    assert not requests
    assert node._message == "grasp_execution_validator_unavailable"


def test_pickup_revalidates_graph_insertion_and_final_closure_separately():
    node, plan, _ = _pickup_worker_node()
    validations = []

    def validate(plan, trajectory, closure):
        validations.append((len(trajectory.points), closure))
        return node._arm_segment_ready(plan, trajectory)

    node._validate_pickup_segment = validate
    node._execute_worker(plan)
    assert validations == [(3, False), (3, True), (1, True)]


def _observed_target(track_id=7, x=0.3, tracked_x=None, class_id=39):
    from om6dof_pick_and_place.graph_pick_node import ObjectTrackObservation
    return ObjectTrackObservation(track_id, class_id, 'bottle', (x, 0.0, 0.2),
                                  (x if tracked_x is None else tracked_x, 0.0, 0.2), (10,))


def _identity_plan():
    return SimpleNamespace(target_track_id=7, target_class_id=39,
                           component_centroid=(0.3, 0.0, 0.2),
                           tracked_centroid=(0.3, 0.0, 0.2))


def _resolve_pickup(history, plan=None):
    from om6dof_pick_and_place.graph_pick_node import pickup_target_observation
    return pickup_target_observation(history, plan or _identity_plan(), 10.0, 2.0, 0.02)


def test_reacquired_id_requires_three_consecutive_fresh_observations():
    observed = _observed_target(track_id=54, x=0.301)
    frames = [(9.7, [observed]), (9.8, [observed]), (9.9, [observed])]
    assert _resolve_pickup(frames) == (observed, None)
    assert _resolve_pickup(frames[1:]) == (None, 'pickup_target_reacquiring')
    assert _resolve_pickup([(9.85, [observed]), (9.87, [observed]), (9.9, [observed])])[1] == \
        'pickup_target_reacquiring'
    assert _resolve_pickup(frames[:1] + [(9.8, [])] + frames[2:])[1] == \
        'pickup_target_reacquiring'


def test_original_id_can_continue_but_not_hide_motion_with_smoothed_position():
    observed = _observed_target(x=0.301)
    assert _resolve_pickup([(9.9, [observed])]) == (observed, None)
    moved = _observed_target(x=0.34, tracked_x=0.301)
    assert _resolve_pickup([(9.9, [moved])])[1] == 'semantic_object_moved_during_pickup'
    # Do not switch to a different nearby bottle while the original moves away.
    assert _resolve_pickup([(9.9, [moved, _observed_target(track_id=8)])])[1] == \
        'semantic_object_moved_during_pickup'


@pytest.mark.parametrize('observations,reason', [
    ([], 'frozen_object_track_not_current'),
    ([_observed_target(class_id=1)], 'frozen_object_track_not_current'),
    ([_observed_target(track_id=8, x=0.34)], 'frozen_object_track_not_current'),
    ([_observed_target(), _observed_target(track_id=8)], 'pickup_target_identity_ambiguous'),
    ([_observed_target(), _observed_target()], 'pickup_target_identity_ambiguous'),
])
def test_reacquisition_rejects_missing_wrong_class_moved_and_ambiguous_targets(observations, reason):
    assert _resolve_pickup([(9.9, observations)])[1] == reason


def test_reacquisition_needs_latest_observation_not_just_a_recent_old_match():
    observed = _observed_target(track_id=8)
    frames = [(9.6, [observed]), (9.7, [observed]), (9.8, [observed])]
    assert _resolve_pickup(frames + [(9.9, [])])[1] == 'frozen_object_track_not_current'
    assert _resolve_pickup([(7.9, [observed])])[1] == 'object_tracks_missing_or_stale'
    assert _resolve_pickup([])[1] == 'object_tracks_missing_or_stale'
    # Repeated processing of the same frame is not three observations.
    assert _resolve_pickup([(9.9, [observed])] * 3)[1] == 'pickup_target_reacquiring'


def test_pregrasp_track_id_churn_still_executes_insertion_and_closure():
    node, plan, calls = _pickup_worker_node()
    plan.target_track_id = 7
    plan.component_centroid = plan.tracked_centroid = (0.3, 0.0, 0.2)
    parameters = {'max_joint_state_age_sec': 0.5, 'max_start_joint_error_rad': 0.01,
                  'max_input_age_sec': 2.0, 'require_calibration_verified': True,
                  'max_object_track_shift_m': 0.02, 'target_reacquisition_timeout_sec': 0.1}
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    del node._pickup_target_rejection
    node._perception_status = {'accepted': True, 'calibration_verified': True}
    node._perception_status_time = time.monotonic()
    node._object_track_history = [(time.monotonic(), [_observed_target()])]
    send_arm = node._send_arm

    def arm_with_track_churn(trajectory, label):
        result = send_arm(trajectory, label)
        if label == 'graph pregrasp':
            now = time.monotonic()
            observed = _observed_target(track_id=54, x=0.301)
            node._object_track_history = [(now - 0.2, [observed]),
                                          (now - 0.1, [observed]), (now, [observed])]
        return result

    node._send_arm = arm_with_track_churn
    node._execute_worker(plan)
    assert calls == ['open gripper', 'graph pregrasp', 'final grasp approach', 'close gripper']
    assert node._observed_target_track_id == 54
    assert node._state == 'grasp_contact_detected'


def test_reacquisition_wait_still_checks_joint_drift_and_cancellation():
    node, plan, calls = _pickup_worker_node()
    parameters = {'max_joint_state_age_sec': 0.5, 'max_start_joint_error_rad': 0.01,
                  'target_reacquisition_timeout_sec': 0.1}
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    def missing(_):
        node._joint_state.position[0] = 0.5
        return 'frozen_object_track_not_current'
    node._pickup_target_rejection = missing
    assert not node._arm_segment_ready(plan, plan.graph_trajectory)
    assert node._message.startswith('arm_segment_start_mismatch')
    assert not calls
    node._joint_state.position[0] = 0.0
    def cancelled(_):
        node._cancel.set()
        return 'pickup_target_reacquiring'
    node._pickup_target_rejection = cancelled
    assert not node._arm_segment_ready(plan, plan.graph_trajectory)
    assert not calls


def test_missing_target_timeout_records_stage_and_never_sends_insertion_or_closure():
    node, plan, calls = _pickup_worker_node()
    parameters = {'max_joint_state_age_sec': 0.5, 'max_start_joint_error_rad': 0.01,
                  'target_reacquisition_timeout_sec': 0.025}
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    node._pickup_target_rejection = lambda _: ('frozen_object_track_not_current'
        if 'graph pregrasp' in calls else None)
    node._execute_worker(plan)
    assert calls == ['open gripper', 'graph pregrasp']
    assert node._state == 'motion_failed'
    assert node._failure_stage == 'following_final_approach'
    assert node._message == 'frozen_object_track_not_current'


def test_captured_stationary_bottle_survives_detector_track_reset():
    from pathlib import Path
    from om6dof_pick_and_place.graph_pick_node import pickup_target_observation
    fixture = json.loads((Path(__file__).parent / 'data/pickup_track_reacquisition.json').read_text())
    plan = SimpleNamespace(**fixture['frozen'])
    history = [(f['received'], parse_object_tracks(json.dumps(f['clusters'])))
               for f in fixture['frames']]
    assert not any(o.track_id == plan.target_track_id for o in history[-1][1])
    observation, rejection = pickup_target_observation(
        history, plan, history[-1][0] + 0.05, 2.0, 0.02)
    assert rejection is None
    assert observation.track_id == 105


def test_brief_detection_dropout_waits_at_boundary_and_recovers():
    node, plan, calls = _pickup_worker_node()
    parameters = {'max_joint_state_age_sec': 0.5, 'max_start_joint_error_rad': 0.01,
                  'target_reacquisition_timeout_sec': 0.2}
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    responses = iter(['frozen_object_track_not_current', 'pickup_target_reacquiring', None])
    node._pickup_target_rejection = lambda _: next(responses)
    assert node._arm_segment_ready(plan, plan.graph_trajectory)
    assert not calls


def test_health_failure_during_reacquisition_never_sends_motion():
    node, plan, calls = _pickup_worker_node()
    parameters = {'max_joint_state_age_sec': 0.5, 'max_start_joint_error_rad': 0.01,
                  'target_reacquisition_timeout_sec': 0.2}
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    healthy = iter([True, False])
    node._execution_interlocks_ready = lambda _: next(healthy)
    node._pickup_target_rejection = lambda _: 'frozen_object_track_not_current'
    assert not node._arm_segment_ready(plan, plan.graph_trajectory)
    assert not calls
