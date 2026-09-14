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
    GraphPickNode, _diagnostic_level, approach_alignment,
    ordered_joint_positions, reverse_trajectory, matching_environment_snapshot,
    semantic_component, trajectory_fingerprint, validate_trajectory)


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


def _interlock_node():
    node = object.__new__(GraphPickNode)
    node._lock = threading.RLock()
    node._operation_mode = "AUTONOMOUS"
    node._remote_enabled = False
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
        "operation_mode_state_topic": "/mode",
        "remote_enabled_state_topic": "/remote",
        "dynamixel_health_topic": "/health",
        "dynamixel_health_timeout_sec": 0.3,
    }
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    node.count_publishers = lambda _topic: 1
    return node


def test_execution_interlocks_require_autonomous_remote_off_and_clean_bus():
    node = _interlock_node()
    plan = SimpleNamespace(bus_read_failures=4, bus_write_failures=2)
    assert GraphPickNode._interlock_blockers(node, plan) == []

    node._remote_enabled = True
    assert "remote_control_not_disabled:True" in \
        GraphPickNode._interlock_blockers(node, plan)


def test_execution_interlock_rejects_bus_failure_after_preview():
    node = _interlock_node()
    plan = SimpleNamespace(bus_read_failures=3, bus_write_failures=2)
    assert "dynamixel_failure_counter_advanced_since_preview" in \
        GraphPickNode._interlock_blockers(node, plan)
