"""Node-level helpers that need no camera, no arm and no network."""

import threading
import time
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip("rclpy")

import numpy as np  # noqa: E402
from rclpy.clock import Clock, ClockType  # noqa: E402
from visualization_msgs.msg import Marker  # noqa: E402

import om6dof_pick_and_place_gemini.gemini_pick_node as pick_node_module  # noqa: E402
from om6dof_pick_and_place_gemini.gemini_client import Localization  # noqa: E402
from om6dof_pick_and_place_gemini.gemini_pick_node import (  # noqa: E402
    GeminiPickNode, ee_position_for_tcp, gripper_calibration_error,
    gripper_position_for_width, rotation_distance)
from om6dof_pick_and_place_gemini.grasp_backends import (  # noqa: E402
    GraspCandidate, GraspScene)
from om6dof_pick_and_place_gemini.grasp_filter import (  # noqa: E402
    FilterConfig, NearMiss, Rejection)
from om6dof_pick_and_place_gemini.transforms import (  # noqa: E402
    rpy_to_matrix, tool_rotation)

LIMITS = dict(open_pos=0.019, close_pos=-0.010,
              width_at_open_pos=0.065, width_at_close_pos=0.010)


def command(width, bias):
    return gripper_position_for_width(width, bias=bias, **LIMITS)


def test_zero_bias_stops_at_the_measured_width():
    # 37.5 mm is halfway between the independently measured 10 and 65 mm
    # apertures, hence halfway through the joint travel.
    assert command(0.0375, 0.0) == pytest.approx(0.0045)


def test_measured_aperture_endpoints_map_exactly_to_joint_endpoints():
    assert command(0.065, 0.0) == pytest.approx(LIMITS["open_pos"])
    assert command(0.010, 0.0) == pytest.approx(LIMITS["close_pos"])


def test_legacy_max_width_mapping_remains_available_for_plan_only_callers():
    assert gripper_position_for_width(
        0.0325, open_pos=0.019, close_pos=-0.010,
        max_width=0.065, bias=0.0) == pytest.approx(0.0045)


def test_full_bias_closes_all_the_way():
    assert command(0.019, 1.0) == pytest.approx(LIMITS["close_pos"])


def test_more_bias_always_means_more_squeeze():
    commands = [command(0.025, bias) for bias in (0.0, 0.3, 0.6, 1.0)]
    assert commands == sorted(commands, reverse=True)


def test_a_wider_object_leaves_the_jaws_further_open():
    assert command(0.030, 0.6) > command(0.012, 0.6)


def test_the_command_never_leaves_the_joint_range():
    for width in (0.0, 0.005, 0.02, 0.065, 0.5):
        for bias in (0.0, 0.5, 1.0):
            value = command(width, bias)
            assert LIMITS["close_pos"] <= value <= LIMITS["open_pos"]


def test_a_bias_outside_0_1_is_clamped():
    assert command(0.02, 5.0) == pytest.approx(command(0.02, 1.0))
    assert command(0.02, -3.0) == pytest.approx(command(0.02, 0.0))


def test_unmeasured_gripper_aperture_is_rejected():
    error = gripper_calibration_error(
        open_pos=0.019, close_pos=-0.010,
        width_at_open_pos=-1.0, width_at_close_pos=-1.0,
        min_width=0.010, max_width=0.065)
    assert error is not None
    assert "unmeasured" in error


@pytest.mark.parametrize(
    ("open_width", "close_width", "min_width", "max_width", "message"),
    [
        (0.010, 0.010, 0.010, 0.0101, "must be greater"),
        (0.065, 0.012, 0.010, 0.065, "below the calibrated closed"),
        (0.060, 0.005, 0.010, 0.065, "exceeds the calibrated open"),
    ])
def test_gripper_calibration_must_contain_the_candidate_width_interval(
        open_width, close_width, min_width, max_width, message):
    error = gripper_calibration_error(
        open_pos=0.019, close_pos=-0.010,
        width_at_open_pos=open_width, width_at_close_pos=close_width,
        min_width=min_width, max_width=max_width)
    assert error is not None
    assert message in error


def test_plan_only_uses_documented_legacy_mapping_when_unmeasured():
    warnings = []
    values = {
        "execute_motion": False,
        "gripper_open_pos": 0.019,
        "gripper_close_pos": -0.010,
        "gripper_width_at_open_pos": -1.0,
        "gripper_width_at_close_pos": -1.0,
        "gripper_calibration_validated": False,
        "min_width": 0.010,
        "max_width": 0.065,
    }
    node = object.__new__(GeminiPickNode)
    node._param = values.__getitem__
    node.get_logger = lambda: SimpleNamespace(
        warn=warnings.append, error=pytest.fail)

    assert node._gripper_width_mapping() == pytest.approx((0.065, 0.0))
    assert warnings and "plan-only" in warnings[0]


def test_physical_mapping_requires_separate_gripper_acknowledgement():
    errors = []
    values = {
        "execute_motion": True,
        "gripper_open_pos": 0.019,
        "gripper_close_pos": -0.010,
        "gripper_width_at_open_pos": 0.065,
        "gripper_width_at_close_pos": 0.005,
        "gripper_calibration_validated": False,
        "min_width": 0.010,
        "max_width": 0.065,
    }
    node = object.__new__(GeminiPickNode)
    node._param = values.__getitem__
    node.get_logger = lambda: SimpleNamespace(
        warn=pytest.fail, error=errors.append)

    assert node._gripper_width_mapping() is None
    assert errors and "gripper_calibration_validated is false" in errors[0]
    values["gripper_calibration_validated"] = True
    assert node._gripper_width_mapping() == pytest.approx((0.065, 0.005))


def test_motion_preflight_stops_before_moveit_without_gripper_signoff():
    errors = []
    values = {
        "execute_motion": True,
        "base_frame": "world",
        "ik_base_link": "world",
        "joint_state_topic": "/joint_states",
        "observe_pose": [0.0] * 6,
        "home_pose": [0.0] * 6,
        "gripper_open_pos": 0.019,
        "gripper_close_pos": -0.010,
        "gripper_width_at_open_pos": 0.065,
        "gripper_width_at_close_pos": 0.005,
        "gripper_calibration_validated": False,
        "min_width": 0.010,
        "max_width": 0.065,
    }
    node = object.__new__(GeminiPickNode)
    node._param = values.__getitem__
    node.base_frame = "world"
    node.mode = "classify"
    node.arm_joints = [f"joint{i}" for i in range(1, 7)]
    node.backend = SimpleNamespace(available=lambda: True, name="test")
    node.moveit = SimpleNamespace(
        motion_faulted=False,
        wait_for_servers=lambda **_kwargs: pytest.fail(
            "MoveIt must not be contacted before gripper sign-off"))
    node._current_joints = lambda: np.zeros(6)
    node._tcp_offset = lambda: np.zeros(3)
    node.get_logger = lambda: SimpleNamespace(error=errors.append)

    assert not node._motion_preflight()
    assert errors and "gripper_calibration_validated:=true" in errors[0]


class _PlanOnlyMoveItRecorder:
    def __init__(self, linear_results, *, pose_result=True,
                 close_result=True):
        self.linear_results = list(linear_results)
        self.pose_result = pose_result
        self.close_result = close_result
        self.calls = []
        self.motion_faulted = False

    @staticmethod
    def _xyz(pose):
        return np.array([
            pose.position.x, pose.position.y, pose.position.z], dtype=float)

    def reset_plan_only_state(self):
        self.calls.append(("reset",))

    def move_to_pose(self, pose, **kwargs):
        self.calls.append(("pose", self._xyz(pose), kwargs))
        if isinstance(self.pose_result, Exception):
            raise self.pose_result
        return self.pose_result

    def move_linear_to_pose(self, pose, **kwargs):
        self.calls.append(("linear", self._xyz(pose), kwargs))
        return self.linear_results.pop(0)

    def set_plan_only_joint_state(self, names, value):
        self.calls.append(("close", list(names), float(value)))
        return self.close_result

    def set_gripper(self, *_args, **_kwargs):
        pytest.fail("prevalidation must never issue a physical gripper goal")


def _prevalidation_node(linear_results, *, pose_result=True,
                        close_result=True):
    values = {
        "execute_motion": True,
        "pregrasp_standoff": 0.10,
        "lift_height": 0.08,
        "grasp_position_tolerance": 0.008,
        "linear_orientation_tolerance": 0.10,
        "gripper_joint_names": [
            "gripper_left_joint", "gripper_right_joint"],
        "gripper_open_pos": 0.019,
        "gripper_close_pos": -0.010,
        "gripper_width_at_open_pos": 0.065,
        "gripper_width_at_close_pos": 0.005,
        "gripper_calibration_validated": True,
        "gripper_close_bias": 0.6,
        "min_width": 0.010,
        "max_width": 0.065,
        "max_prevalidation_candidates": 5,
    }
    node = object.__new__(GeminiPickNode)
    node._param = values.__getitem__
    node._ee_target = lambda point, _rotation: np.asarray(point, dtype=float)
    node._pose = lambda point, rotation: GeminiPickNode._pose(
        node, point, rotation)
    node._motion_command_allowed = lambda: True
    node._should_stop = lambda: False
    node._publish_status = lambda _message: None
    node._set_stage = lambda _stage: None
    node.get_logger = lambda: SimpleNamespace(
        error=lambda _message: None, warn=lambda _message: None)
    node.moveit = _PlanOnlyMoveItRecorder(
        linear_results, pose_result=pose_result, close_result=close_result)
    candidate = GraspCandidate(
        position=np.array([0.30, 0.0, 0.10]),
        approach=np.array([0.0, 0.0, -1.0]),
        closing=np.array([0.0, 1.0, 0.0]),
        width=0.020, score=0.8)
    return node, candidate


def test_full_chain_prevalidation_is_plan_only_and_prefers_vertical_lift():
    node, candidate = _prevalidation_node([True, True])

    assert node._prevalidate_candidate_chain(candidate)

    calls = node.moveit.calls
    assert [call[0] for call in calls] == [
        "reset", "pose", "linear", "close", "linear", "reset"]
    assert calls[1][1] == pytest.approx([0.30, 0.0, 0.20])
    assert calls[2][1] == pytest.approx([0.30, 0.0, 0.10])
    assert calls[4][1] == pytest.approx([0.30, 0.0, 0.18])
    assert calls[1][2]["plan_only"] is True
    assert calls[2][2] == {
        "position_tolerance": 0.008,
        "orientation_tolerance": 0.10,
        "plan_only": True,
    }
    assert calls[4][2] == {
        "orientation_tolerance": 0.10, "plan_only": True}
    assert calls[3][1] == ["gripper_left_joint", "gripper_right_joint"]
    assert calls[3][2] == pytest.approx(-0.0071)
    assert candidate.extras["post_grasp_mode"] == "vertical_lift"
    assert candidate.extras["moveit_chain_validated"] == 1.0


def test_candidate_motion_targets_use_anygrasp_tcp_not_grasp_centre():
    node = object.__new__(GeminiPickNode)
    node._param = lambda name: {
        "pregrasp_standoff": 0.10,
        "lift_height": 0.08,
    }[name]
    node._ee_target = lambda point, _rotation: np.asarray(point, dtype=float)
    candidate = GraspCandidate(
        position=np.array([0.30, 0.0, 0.10]),
        approach=np.array([1.0, 0.0, 0.0]),
        closing=np.array([0.0, 1.0, 0.0]),
        width=0.020, score=0.8,
        extras={"tcp_position": np.array([0.34, 0.0, 0.10])})

    _rotation, pregrasp, grasp, lift = \
        GeminiPickNode._candidate_motion_targets(node, candidate)

    assert pregrasp == pytest.approx([0.24, 0.0, 0.10])
    assert grasp == pytest.approx([0.34, 0.0, 0.10])
    assert lift == pytest.approx([0.34, 0.0, 0.18])
    assert candidate.position == pytest.approx([0.30, 0.0, 0.10])


def test_full_chain_prevalidation_falls_back_to_exact_reverse_retreat():
    node, candidate = _prevalidation_node([True, False, True])

    assert node._prevalidate_candidate_chain(candidate)

    linear_targets = [call[1] for call in node.moveit.calls
                      if call[0] == "linear"]
    np.testing.assert_allclose(linear_targets, [
        [0.30, 0.0, 0.10], [0.30, 0.0, 0.18], [0.30, 0.0, 0.20]])
    assert candidate.extras["post_grasp_mode"] == "reverse_to_pregrasp"
    assert [candidate.extras[f"post_grasp_target_{axis}"]
            for axis in "xyz"] == pytest.approx([0.30, 0.0, 0.20])


@pytest.mark.parametrize(
    ("pose_result", "linear_results", "close_result", "expected"),
    [
        (False, [], True, ["reset", "pose", "reset"]),
        (True, [False], True, ["reset", "pose", "linear", "reset"]),
        (True, [True], False,
         ["reset", "pose", "linear", "close", "reset"]),
        (True, [True, False, False], True,
         ["reset", "pose", "linear", "close", "linear", "linear", "reset"]),
    ])
def test_full_chain_prevalidation_stops_safely_at_failed_stage(
        pose_result, linear_results, close_result, expected):
    node, candidate = _prevalidation_node(
        linear_results, pose_result=pose_result, close_result=close_result)

    assert not node._prevalidate_candidate_chain(candidate)
    assert [call[0] for call in node.moveit.calls] == expected
    assert candidate.extras["moveit_chain_validated"] == 0.0


def test_full_chain_prevalidation_resets_cached_state_after_exception():
    node, candidate = _prevalidation_node(
        [], pose_result=RuntimeError("planner transport failed"))

    with pytest.raises(RuntimeError, match="planner transport failed"):
        node._prevalidate_candidate_chain(candidate)

    assert [call[0] for call in node.moveit.calls] == [
        "reset", "pose", "reset"]


def test_candidate_scan_resets_each_chain_and_selects_first_complete_one():
    node, first = _prevalidation_node([True, False, False, True, True])
    second = GraspCandidate(
        position=np.array([0.32, 0.0, 0.10]),
        approach=np.array([0.0, 0.0, -1.0]),
        closing=np.array([0.0, 1.0, 0.0]),
        width=0.020, score=0.7)

    selected = node._select_prevalidated_candidate([first, second])

    assert selected is second
    assert sum(call[0] == "reset" for call in node.moveit.calls) == 6
    pose_targets = [call[1] for call in node.moveit.calls
                    if call[0] == "pose"]
    np.testing.assert_allclose(
        pose_targets, [[0.30, 0.0, 0.20], [0.32, 0.0, 0.20]])


def test_candidate_scan_rejects_an_unbounded_configuration():
    node, candidate = _prevalidation_node([])
    original_param = node._param
    node._param = lambda name: (
        21 if name == "max_prevalidation_candidates" else original_param(name))
    errors = []
    node.get_logger = lambda: SimpleNamespace(error=errors.append)

    assert node._select_prevalidated_candidate([candidate]) is None
    assert errors and "must be in [1, 20]" in errors[0]
    assert [call[0] for call in node.moveit.calls] == ["reset", "reset"]


def test_execute_grasp_refuses_unvalidated_physical_candidate_before_motion():
    node, candidate = _prevalidation_node([])
    errors = []
    node.get_logger = lambda: SimpleNamespace(error=errors.append)
    node._move_pose = lambda *_args, **_kwargs: pytest.fail(
        "unvalidated candidate must not start motion")

    assert not node.execute_grasp(candidate)
    assert errors and "no complete MoveIt" in errors[0]


def test_execute_grasp_uses_the_exact_validated_reverse_target():
    node, candidate = _prevalidation_node([])
    candidate.extras["moveit_chain_validated"] = 1.0
    candidate.extras["post_grasp_mode"] = "reverse_to_pregrasp"
    candidate.extras["post_grasp_target_x"] = 0.30
    candidate.extras["post_grasp_target_y"] = 0.0
    candidate.extras["post_grasp_target_z"] = 0.20
    linear_targets = []
    stages = []
    node._move_pose = lambda *_args, **_kwargs: True
    node._move_linear_pose = lambda target, *_args, **_kwargs: (
        linear_targets.append(np.asarray(target).copy()) or True)
    node._close_on = lambda _candidate: True
    node._set_stage = stages.append
    node._stop_requested = False
    node.moveit.motion_faulted = False

    assert node.execute_grasp(candidate)
    np.testing.assert_allclose(
        linear_targets, [[0.30, 0.0, 0.10], [0.30, 0.0, 0.20]])
    assert stages[-1] == "postgrasp_retreat"


def _sequence_node(*, execute_motion, accepted):
    values = {
        "execute_motion": execute_motion,
        "observe_pose": [0.0] * 6,
        "max_grasp_attempts": 3,
        "place_enabled": False,
    }
    node = object.__new__(GeminiPickNode)
    node._param = values.__getitem__
    node._motion_preflight = lambda: True
    node._should_stop = lambda: False
    node._move_joints = lambda *_args: True
    node._open_gripper = lambda: True
    node.capture_scene = lambda: object()
    node.select_grasp = lambda _scene: (accepted[0], accepted)
    node._publish_status = lambda _message: None
    node._set_stage = lambda stage: setattr(node, "_stage", stage)
    node._stop_requested = False
    node._last_recovery_ok = True
    node._last_pick = {}
    node.moveit = SimpleNamespace(
        reset_plan_only_state=lambda: None,
        begin_plan_only_display=lambda: None,
        publish_plan_only_display=lambda: None,
        discard_plan_only_display=lambda: None,
    )
    return node


def test_physical_sequence_executes_only_the_one_prevalidated_candidate():
    first = grasp_at([0.30, 0.0, 0.10])
    second = grasp_at([0.32, 0.0, 0.10])
    node = _sequence_node(execute_motion=True, accepted=[first, second])
    node._select_prevalidated_candidate = lambda _candidates: second
    executed = []
    node.execute_grasp = lambda candidate: executed.append(candidate) or False

    node._run_sequence_impl()

    assert executed == [second]
    assert node._stage == "failed"


def test_global_plan_only_bypasses_the_extra_candidate_prevalidation():
    first = grasp_at([0.30, 0.0, 0.10])
    node = _sequence_node(execute_motion=False, accepted=[first])
    node._select_prevalidated_candidate = lambda _candidates: pytest.fail(
        "global plan-only must retain its existing chained execution")
    node.execute_grasp = lambda candidate: candidate is first
    node._classify_pick = lambda _scene, _candidate: "object"

    node._run_sequence_impl()

    assert node._stage == "idle"
    assert node._last_pick["label"] == "object"


def test_global_plan_only_publishes_capture_only_after_grasp_chain_succeeds():
    candidate = grasp_at([0.30, 0.0, 0.10])
    node = _sequence_node(execute_motion=False, accepted=[candidate])
    events = []
    node.moveit.begin_plan_only_display = lambda: events.append("begin")
    node.moveit.publish_plan_only_display = lambda: events.append("publish")
    node.moveit.discard_plan_only_display = lambda: events.append("discard")
    node.execute_grasp = lambda selected: (
        events.append(("execute", selected)) or True)
    node._classify_pick = lambda _scene, _candidate: "object"

    node._run_sequence_impl()

    assert events == [
        "begin", ("execute", candidate), "publish", "discard"]
    assert node._stage == "idle"


def test_global_plan_only_discards_failed_capture_without_publishing():
    candidate = grasp_at([0.30, 0.0, 0.10])
    node = _sequence_node(execute_motion=False, accepted=[candidate])
    events = []
    node.moveit.begin_plan_only_display = lambda: events.append("begin")
    node.moveit.publish_plan_only_display = lambda: events.append("publish")
    node.moveit.discard_plan_only_display = lambda: events.append("discard")
    node.execute_grasp = lambda selected: (
        events.append(("execute", selected)) or False)

    node._run_sequence_impl()

    assert events == ["begin", ("execute", candidate), "discard"]
    assert node._stage == "failed"


def test_global_plan_only_discards_capture_when_stop_interrupts_candidate():
    candidate = grasp_at([0.30, 0.0, 0.10])
    node = _sequence_node(execute_motion=False, accepted=[candidate])
    events = []
    node.moveit.begin_plan_only_display = lambda: events.append("begin")
    node.moveit.publish_plan_only_display = lambda: events.append("publish")
    node.moveit.discard_plan_only_display = lambda: events.append("discard")

    def stop_during_grasp(selected):
        events.append(("execute", selected))
        node._stop_requested = True
        # Even if the final planner call reports success concurrently with a
        # stop, the aborted chain must never be presented as completed.
        return True

    node.execute_grasp = stop_during_grasp

    node._run_sequence_impl()

    assert events == ["begin", ("execute", candidate), "discard"]


def test_physical_prevalidation_never_begins_app_display_capture():
    candidate = grasp_at([0.30, 0.0, 0.10])
    node = _sequence_node(execute_motion=True, accepted=[candidate])
    events = []
    node._select_prevalidated_candidate = lambda _candidates: (
        events.append("prevalidate") or candidate)
    node.moveit.reset_plan_only_state = lambda: events.append("reset")
    node.moveit.begin_plan_only_display = lambda: pytest.fail(
        "physical candidate prevalidation must not begin app display capture")
    node.moveit.publish_plan_only_display = lambda: pytest.fail(
        "physical execution must not publish app display capture")
    node.moveit.discard_plan_only_display = lambda: pytest.fail(
        "physical execution must not own an app display capture")
    node.execute_grasp = lambda selected: (
        events.append(("execute", selected)) or False)

    node._run_sequence_impl()

    assert events == ["prevalidate", "reset", ("execute", candidate)]


def test_rotation_distance_handles_the_180_degree_case():
    half_turn_x = np.diag([1.0, -1.0, -1.0])
    assert rotation_distance(np.eye(3), half_turn_x) == pytest.approx(np.pi)


def test_parameter_snapshot_wins_over_a_late_live_parameter_commit():
    node = object.__new__(GeminiPickNode)
    node._active_parameters = {"execute_motion": False}
    node.get_parameter = lambda name: SimpleNamespace(value=True)

    assert node._param("execute_motion") is False


def test_parameter_snapshot_uses_the_humble_compatible_prefix_api():
    node = object.__new__(GeminiPickNode)
    source = {"execute_motion": False, "camera_xyz": [1.0, 2.0, 3.0]}
    parameters = {
        name: SimpleNamespace(value=value) for name, value in source.items()
    }
    node.get_parameters_by_prefix = lambda prefix: parameters

    snapshot = node._snapshot_parameters()

    assert snapshot == source
    snapshot["camera_xyz"][0] = 99.0
    assert source["camera_xyz"][0] == 1.0


def test_auto_run_cancels_only_its_one_shot_timer():
    class Timer:
        def __init__(self):
            self.cancelled = False

        def cancel(self):
            self.cancelled = True

    node = object.__new__(GeminiPickNode)
    auto_timer = Timer()
    safety_timer = Timer()
    node._auto_run_timer = auto_timer
    node._interlock_timer = safety_timer
    node._run_sequence = object()
    node._busy = lambda: False
    starts = []
    node._start = lambda *args, **kwargs: starts.append((args, kwargs))

    node._auto_run_once()

    assert auto_timer.cancelled
    assert node._auto_run_timer is None
    assert not safety_timer.cancelled
    assert len(starts) == 1


def test_shutdown_latch_rejects_a_queued_start_before_begin_sequence():
    node = object.__new__(GeminiPickNode)
    node._worker_lock = threading.Lock()
    node._shutdown_requested = True
    node.moveit = SimpleNamespace(begin_sequence=lambda: pytest.fail(
        "shutdown must reject before clearing the MoveIt stop latch"))
    response = SimpleNamespace(success=None, message="")

    result = node._start(lambda: None, "late_run", response,
                         motion_sequence=True)

    assert result is response
    assert not response.success
    assert response.message == "node is shutting down"


def test_stop_monitor_repeats_direct_cancel_while_physical_action_is_in_doubt():
    requests = []
    node = object.__new__(GeminiPickNode)
    node._worker_lock = threading.Lock()
    node._motion_sequence_active = False
    node._stop_requested = True
    node.moveit = SimpleNamespace(
        physical_action_in_flight=True,
        motion_faulted=False,
        cancel_controller_goals=lambda: requests.append(True),
    )
    node._param = lambda name: False

    node._monitor_execution_interlocks()

    assert requests == [True]


def test_fault_monitor_retries_direct_cancel_without_an_operator_stop():
    requests = []
    node = object.__new__(GeminiPickNode)
    node._worker_lock = threading.Lock()
    node._motion_sequence_active = False
    node._stop_requested = False
    node.moveit = SimpleNamespace(
        physical_action_in_flight=True,
        motion_faulted=True,
        cancel_controller_goals=lambda: requests.append(True),
    )
    node._param = lambda name: False

    node._monitor_execution_interlocks()

    assert requests == [True]


def test_tcp_offset_is_rotated_from_tool_axes_into_world():
    quarter_turn_z = np.array([[0.0, -1.0, 0.0],
                               [1.0, 0.0, 0.0],
                               [0.0, 0.0, 1.0]])
    ee = ee_position_for_tcp([0.0, 0.0, 0.0], quarter_turn_z,
                             [0.10, 0.0, 0.0])
    assert ee == pytest.approx([0.0, -0.10, 0.0])


def test_ik_acceptance_uses_fk_residual_not_the_solver_boolean():
    class ExactButStrictIK:
        def solve_pose_ik(self, seed, position, rotation):
            self.position = np.asarray(position)
            self.rotation = np.asarray(rotation)
            return np.asarray(seed), False

        def fk_pose(self, _solution):
            return self.position.copy(), self.rotation.copy()

    fake_node = type("FakeNode", (), {})()
    fake_node.ik = ExactButStrictIK()
    fake_node._param = lambda name: {
        "ik_position_tolerance": 0.02,
        "ik_orientation_tolerance": 0.20,
    }[name]
    solution, ok, pos_error, ori_error = GeminiPickNode._solve_ik_checked(
        fake_node, np.zeros(6), np.array([0.2, 0.0, 0.1]), np.eye(3))
    assert solution == pytest.approx(np.zeros(6))
    assert ok
    assert pos_error == pytest.approx(0.0)
    assert ori_error == pytest.approx(0.0)


def _reachable_chain_node(results):
    node = object.__new__(GeminiPickNode)
    node._current_joints = lambda: np.zeros(6)
    node._plan_only = lambda: False
    node._param = lambda name: {"lift_height": 0.08}[name]
    node._ee_target = lambda position, _rotation: np.asarray(
        position, dtype=float)
    calls = []

    def solve(seed, position, rotation):
        calls.append((np.asarray(seed).copy(), np.asarray(position).copy(),
                      np.asarray(rotation).copy()))
        return results.pop(0)

    node._solve_ik_checked = solve
    return node, calls


def test_reachable_chain_checks_vertical_lift_with_chained_ik_seeds():
    q_pre = np.ones(6)
    q_grasp = np.full(6, 2.0)
    q_lift = np.full(6, 3.0)
    node, calls = _reachable_chain_node([
        (q_pre, True, 0.001, 0.01),
        (q_grasp, True, 0.002, 0.02),
        (q_lift, True, 0.003, 0.03),
    ])
    candidate = grasp_at([0.30, 0.0, 0.10])
    pregrasp = np.array([0.20, 0.0, 0.10])
    grasp = candidate.position.copy()

    ok, detail = GeminiPickNode._reachable_chain(
        node, pregrasp, grasp, candidate)

    assert ok
    assert "lift=3.0 mm" in detail
    assert len(calls) == 3
    assert calls[0][0] == pytest.approx(np.zeros(6))
    assert calls[1][0] == pytest.approx(q_pre)
    assert calls[2][0] == pytest.approx(q_grasp)
    assert calls[0][1] == pytest.approx(pregrasp)
    assert calls[1][1] == pytest.approx(grasp)
    assert calls[2][1] == pytest.approx(grasp + [0.0, 0.0, 0.08])
    assert all(call[2] == pytest.approx(calls[0][2]) for call in calls)
    assert candidate.extras["post_grasp_mode"] == "vertical_lift"
    assert [candidate.extras[f"post_grasp_target_{axis}"]
            for axis in "xyz"] == pytest.approx([0.30, 0.0, 0.18])
    assert candidate.extras["ik_post_grasp_position_error"] == \
        pytest.approx(0.003)


def test_reachable_chain_falls_back_to_exact_reverse_pregrasp_target():
    q_pre = np.ones(6)
    q_grasp = np.full(6, 2.0)
    node, calls = _reachable_chain_node([
        (q_pre, True, 0.001, 0.01),
        (q_grasp, True, 0.002, 0.02),
        (np.full(6, 3.0), False, 0.025, 0.30),
        (np.full(6, 4.0), True, 0.004, 0.04),
    ])
    candidate = grasp_at([0.30, 0.0, 0.10])
    pregrasp = np.array([0.20, 0.0, 0.10])

    ok, detail = GeminiPickNode._reachable_chain(
        node, pregrasp, candidate.position, candidate)

    assert ok
    assert "vertical lift failed (25.0 mm)" in detail
    assert "reverse retreat=4.0 mm" in detail
    assert len(calls) == 4
    assert calls[2][0] == pytest.approx(q_grasp)
    assert calls[3][0] == pytest.approx(q_grasp)
    assert calls[2][1] == pytest.approx([0.30, 0.0, 0.18])
    assert calls[3][1] == pytest.approx(pregrasp)
    assert calls[3][2] == pytest.approx(calls[2][2])
    assert candidate.extras["post_grasp_mode"] == "reverse_to_pregrasp"
    assert [candidate.extras[f"post_grasp_target_{axis}"]
            for axis in "xyz"] == pytest.approx(pregrasp)
    assert candidate.extras["ik_lift_position_error"] == pytest.approx(0.025)
    assert candidate.extras["ik_post_grasp_position_error"] == \
        pytest.approx(0.004)


def test_reachable_chain_rejects_when_lift_and_reverse_are_unreachable():
    node, _calls = _reachable_chain_node([
        (np.ones(6), True, 0.001, 0.01),
        (np.full(6, 2.0), True, 0.002, 0.02),
        (np.full(6, 3.0), False, 0.025, 0.30),
        (np.full(6, 4.0), False, 0.030, 0.40),
    ])
    candidate = grasp_at([0.30, 0.0, 0.10])

    ok, detail = GeminiPickNode._reachable_chain(
        node, np.array([0.20, 0.0, 0.10]), candidate.position, candidate)

    assert not ok
    assert "lift residual 25.0 mm" in detail
    assert "reverse retreat residual 30.0 mm" in detail
    assert "post_grasp_mode" not in candidate.extras


def test_stop_never_starts_an_automatic_recovery_motion():
    class Logger:
        def warn(self, _message):
            pass

    fake_node = type("FakeNode", (), {})()
    fake_node._stop_requested = True
    fake_node.get_logger = lambda: Logger()
    fake_node._publish_status = lambda _message: pytest.fail(
        "recovery should stop before publishing a motion sequence")
    fake_node._open_gripper = lambda: pytest.fail(
        "stop must not issue a new gripper goal")
    fake_node._move_linear_pose = lambda *_args: pytest.fail(
        "stop must not issue a retreat goal")

    assert not GeminiPickNode._recover_failed_grasp(
        fake_node, np.zeros(3), np.eye(3))


def test_capture_joint_lookup_uses_the_nearest_timestamp_and_fails_stale():
    fake_node = type("FakeNode", (), {})()
    fake_node._lock = threading.Lock()
    fake_node._joint_state = np.zeros(6)
    fake_node._joint_stamp = time.monotonic()
    fake_node._joint_history = deque([
        (10.0, np.ones(6)),
        (10.2, np.full(6, 2.0)),
    ])
    fake_node._param = lambda name: {
        "joint_capture_tolerance_s": 0.15,
        "joint_state_timeout_s": 5.0,
    }[name]

    joints = GeminiPickNode._joints_for_capture(fake_node, 10.18)
    assert joints == pytest.approx(np.full(6, 2.0))
    assert GeminiPickNode._joints_for_capture(fake_node, 11.0) is None


def test_malformed_joint_state_is_ignored_without_replacing_last_state():
    class Logger:
        def __init__(self):
            self.warnings = []

        def warn(self, message):
            self.warnings.append(message)

    fake_node = object.__new__(GeminiPickNode)
    fake_node.arm_joints = [f"joint{i}" for i in range(1, 7)]
    fake_node._lock = threading.Lock()
    fake_node._joint_state = np.ones(6)
    fake_node._joint_stamp = 123.0
    fake_node._joint_history = deque(maxlen=10)
    fake_node._last_bad_joint_state_warning = 0.0
    logger = Logger()
    fake_node.get_logger = lambda: logger
    malformed = SimpleNamespace(
        name=fake_node.arm_joints,
        position=[0.0, 0.0],
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=1, nanosec=0)),
    )

    GeminiPickNode._on_joint_state(fake_node, malformed)

    assert fake_node._joint_state == pytest.approx(np.ones(6))
    assert fake_node._joint_stamp == 123.0
    assert not fake_node._joint_history
    assert len(logger.warnings) == 1


def test_camera_tf_lookup_uses_the_rgbd_capture_timestamp():
    class Buffer:
        def lookup_transform(self, target, source, stamp, timeout):
            self.target = target
            self.source = source
            self.stamp = stamp
            return SimpleNamespace(transform=SimpleNamespace(
                translation=SimpleNamespace(x=0.1, y=0.2, z=0.3),
                rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))

    fake_node = type("FakeNode", (), {})()
    fake_node.base_frame = "world"
    fake_node._param = lambda name: {
        "camera_optical_frame": "d405_depth_optical_frame",
    }[name]
    fake_node._tf_buffer = Buffer()
    fake_node._warned_tf_fallback = False
    fake_node.get_clock = lambda: Clock(clock_type=ClockType.SYSTEM_TIME)

    position, rotation, source = GeminiPickNode._camera_pose(
        fake_node, np.zeros(6), "d405_depth_optical_frame", 123.456)

    assert fake_node._tf_buffer.stamp.nanoseconds == 123456000000
    assert position == pytest.approx([0.1, 0.2, 0.3])
    assert rotation == pytest.approx(np.eye(3))
    assert source == "TF(d405_depth_optical_frame)"


def test_fallback_camera_rpy_is_already_the_resolved_urdf_optical_rotation():
    camera_rpy = [-0.436, 0.0, -1.571]
    values = {
        "mode": "target", "base_frame": "world",
        "arm_joint_names": [f"joint{i}" for i in range(1, 7)],
        "camera_xyz": [-0.087, 0.0, -0.074],
        "camera_rpy": camera_rpy,
        "camera_optical_frame": "",
        "min_width": 0.010, "max_width": 0.065, "table_z": 0.0,
        "min_clearance": 0.010, "max_tilt": 1.2,
        "workspace_min": [-1.0, -1.0, -1.0],
        "workspace_max": [1.0, 1.0, 1.0],
        "pregrasp_standoff": 0.08, "min_score": 0.0,
        "place_poses_file": "", "place_categories": [],
    }
    fake_node = object.__new__(GeminiPickNode)
    fake_node._param = values.__getitem__
    fake_node._load_place_poses = lambda _path: {}
    fake_node.ik = SimpleNamespace(
        fk_pose=lambda _joints: (np.zeros(3), np.eye(3)))

    GeminiPickNode._read_parameters(fake_node)
    position, rotation, source = GeminiPickNode._camera_pose(
        fake_node, np.zeros(6))

    # The shipped RPY is the full end_effector_link -> optical transform
    # resolved from the URDF, not a camera-body RPY awaiting another optical
    # axis conversion.
    expected = rpy_to_matrix(*camera_rpy)
    assert position == pytest.approx(values["camera_xyz"])
    assert rotation == pytest.approx(expected)
    assert rotation[:, 2] == pytest.approx(
        [0.422317, -0.000086, 0.906448], abs=1e-6)
    assert source == "fallback params"


def test_capture_scene_keeps_capture_time_tool_orientation(monkeypatch):
    capture_joints = np.arange(6, dtype=float)
    tool_rotation_at_capture = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ])

    class IK:
        def __init__(self):
            self.fk_joints = None

        def fk_pose(self, joints):
            self.fk_joints = np.asarray(joints).copy()
            return np.zeros(3), tool_rotation_at_capture.copy()

    frame = SimpleNamespace(
        depth=np.ones((1, 1), dtype=np.uint16),
        color=np.zeros((1, 1, 3), dtype=np.uint8),
        intrinsics=(100.0, 100.0, 0.0, 0.0), depth_scale=0.001,
        stamp=12.5, frame_id="camera_optical")
    fake_node = type("FakeNode", (), {})()
    fake_node.base_frame = "world"
    fake_node.camera = SimpleNamespace(capture=lambda warmup: frame)
    fake_node._camera_lock = threading.Lock()
    fake_node.ik = IK()
    fake_node._current_joints = lambda: np.zeros(6)
    fake_node._joints_for_capture = lambda stamp: capture_joints.copy()
    fake_node._camera_pose = lambda joints, frame_id, stamp: (
        np.zeros(3), np.eye(3), "test TF")
    fake_node._publish_status = lambda _message: None
    published_clouds = []
    fake_node.pub_world_cloud = SimpleNamespace(
        publish=published_clouds.append)
    fake_node.get_clock = lambda: SimpleNamespace(
        now=lambda: pick_node_module.Time(nanoseconds=12_500_000_000))
    fake_node._param = lambda name: {
        "camera_warmup_frames": 1,
        "joint_state_topic": "/joint_states",
        "joint_capture_tolerance_s": 0.15,
        "cloud_stride": 1,
        "cloud_z_min": 0.05,
        "cloud_z_max": 0.80,
        "self_exclusion_radius_m": 0.09,
    }[name]
    monkeypatch.setattr(
        pick_node_module, "point_cloud",
        lambda *args, **kwargs: (
            np.array([[0.3, 0.0, 0.4]]),
            np.array([[1, 2, 3]], dtype=np.uint8),
            np.array([[0.0, 0.0]])))

    captured = GeminiPickNode.capture_scene(fake_node)

    assert fake_node.ik.fk_joints == pytest.approx(capture_joints)
    assert captured.tool_rotation_base == pytest.approx(
        tool_rotation_at_capture)
    assert np.array_equal(captured.source_indices, [0])
    assert len(published_clouds) == 1
    assert published_clouds[0].header.frame_id == "world"


def test_target_detection_segments_and_collision_checks_the_wider_scene(
        monkeypatch):
    full_scene = scene()
    full_scene.points_optical = np.zeros((40, 3))
    full_scene.points_base = np.zeros((40, 3))
    full_scene.pixels = np.zeros((40, 2))
    target_scene = scene()
    target_scene.points_optical = np.zeros((35, 3))
    target_scene.points_base = np.zeros((35, 3))
    target_scene.pixels = np.zeros((35, 2))
    target = Localization(
        pixel=(320.0, 240.0), box=(300.0, 220.0, 340.0, 260.0),
        confidence=0.9)
    calls = {}

    def segment(source, bbox, pixel, **kwargs):
        calls["segment"] = (source, bbox, pixel, kwargs)
        return target_scene

    class Backend:
        name = "graspnet"

        def detect(self, network_scene, collision_scene=None,
                   sampling_seed=None):
            calls["detect"] = (
                network_scene, collision_scene, sampling_seed)
            return [grasp_at([0.0, 0.0, 0.4])]

    class Logger:
        def error(self, message):
            calls.setdefault("errors", []).append(message)

    fake_node = object.__new__(GeminiPickNode)
    fake_node.backend = Backend()
    fake_node._target_description = "red cube"
    fake_node._locate_target = lambda _scene: target
    fake_node._set_stage = lambda stage: calls.setdefault("stages", []).append(stage)
    fake_node._publish_status = lambda message: calls.setdefault(
        "status", []).append(message)
    fake_node._publish_target_cloud = lambda target: calls.update(
        published_target=target)
    fake_node.get_logger = lambda: Logger()
    fake_node._param = lambda name: {
        "target_support_enabled": False,
        "target_crop_pad_px": 4.0,
        "target_seed_radius_px": 14.0,
        "target_depth_tolerance_m": 0.05,
        "target_component_voxel_m": 0.008,
        "target_component_min_points": 30,
        "graspnet_sampling_attempts": 1,
        "graspnet_sampling_seed": 7,
        "table_z": 0.0,
        "target_table_margin_m": 0.006,
    }[name]
    monkeypatch.setattr(pick_node_module, "segment_target_component", segment)
    monkeypatch.setattr(
        pick_node_module, "crop_to_workspace",
        lambda source, _low, _high: source)
    fake_node.filter_cfg = FilterConfig(
        workspace_min=[-1.0, -1.0, -1.0],
        workspace_max=[1.0, 1.0, 1.0])

    candidates, segmented, pixel = GeminiPickNode._detect_in_target(
        fake_node, full_scene)

    assert len(candidates) == 1
    assert segmented is target_scene
    assert calls["published_target"] is target_scene
    assert pixel == target.pixel
    # Regression: Gemini segmentation is not the GraspNet input.
    assert calls["detect"] == (full_scene, full_scene, 7)
    assert calls["segment"][0] is full_scene
    assert calls["segment"][1:3] == (target.box, target.pixel)
    assert calls["segment"][3] == {
        "pad_px": 4.0,
        "seed_radius_px": 14.0,
        "depth_tolerance": 0.05,
        "voxel_size": 0.008,
        "min_points": 30,
        "table_z": 0.0,
        "table_margin": 0.006,
    }


def test_anygrasp_target_detection_uses_full_scene_and_exact_region_mask(
        monkeypatch):
    points = np.array([
        [0.10, 0.0, 0.30],
        [0.20, 0.0, 0.30],
        [0.30, 0.0, 0.30],
        [0.40, 0.0, 0.30],
    ])
    full_scene = GraspScene(
        points_optical=points.copy(), points_base=points.copy(),
        pixels=np.array([[10, 10], [20, 20], [30, 30], [40, 40]],
                        dtype=float),
        colors=None, p_wc=np.zeros(3), R_wc=np.eye(3), intrinsics=INTR,
        color_image=np.zeros((480, 640, 3), np.uint8),
        source_indices=np.array([101, 102, 103, 104]))
    target_scene = GraspScene(
        points_optical=points[[3, 1]].copy(),
        points_base=points[[3, 1]].copy(),
        pixels=np.array([[40, 40], [20, 20]], dtype=float),
        colors=None, p_wc=np.zeros(3), R_wc=np.eye(3), intrinsics=INTR,
        color_image=full_scene.color_image,
        source_indices=np.array([104, 102]))
    target = Localization(
        pixel=(20.0, 20.0), box=(5.0, 5.0, 45.0, 45.0),
        confidence=0.9)
    calls = {}

    class Backend:
        name = "anygrasp"
        supports_region_steering = True

        def detect(self, network_scene, collision_scene=None,
                   region_mask=None):
            calls["detect"] = (
                network_scene, collision_scene, region_mask.copy())
            return [grasp_at([0.20, 0.0, 0.30])]

    fake_node = object.__new__(GeminiPickNode)
    fake_node.backend = Backend()
    fake_node._target_description = "red cube"
    fake_node._locate_target = lambda _scene: target
    fake_node._set_stage = lambda _stage: None
    fake_node._publish_status = lambda _message: None
    fake_node._publish_target_cloud = lambda value: calls.update(
        published_target=value)
    fake_node.get_logger = lambda: SimpleNamespace(error=pytest.fail)
    fake_node._param = lambda name: {
        "target_support_enabled": False,
        "target_crop_pad_px": 4.0,
        "target_seed_radius_px": 14.0,
        "target_depth_tolerance_m": 0.05,
        "target_component_voxel_m": 0.008,
        "target_component_min_points": 1,
        "table_z": 0.0,
        "target_table_margin_m": 0.006,
    }[name]
    monkeypatch.setattr(
        pick_node_module, "segment_target_component",
        lambda source, *_args, **_kwargs: (
            target_scene if source is full_scene else pytest.fail(
                "segmentation must use the full capture")))
    monkeypatch.setattr(
        pick_node_module, "crop_to_workspace",
        lambda *_args, **_kwargs: pytest.fail(
            "AnyGrasp target mode must not crop the inference scene"))

    candidates, segmented, pixel = GeminiPickNode._detect_in_target(
        fake_node, full_scene)

    assert len(candidates) == 1
    assert segmented is target_scene
    assert pixel == target.pixel
    assert calls["detect"][0] is full_scene
    assert calls["detect"][1] is full_scene
    assert np.array_equal(
        calls["detect"][2], [False, True, False, True])
    assert calls["published_target"] is target_scene


def test_graspnet_multi_seed_pools_candidates_without_skipping_filters():
    calls = []

    class Backend:
        def detect(self, network_scene, collision_scene=None,
                   sampling_seed=None):
            calls.append((network_scene, collision_scene, sampling_seed))
            return [grasp_at([0.01 * sampling_seed, 0.0, 0.4])]

    fake_node = object.__new__(GeminiPickNode)
    fake_node.backend = Backend()
    fake_node._param = lambda name: {
        "graspnet_sampling_attempts": 3,
        "graspnet_sampling_seed": 4,
    }[name]
    statuses = []
    fake_node._publish_status = statuses.append
    target = scene()
    collision = scene()

    candidates = GeminiPickNode._detect_graspnet_multi_seed(
        fake_node, target, collision_scene=collision)

    assert len(candidates) == 3
    assert [call[2] for call in calls] == [4, 5, 6]
    assert all(call[:2] == (target, collision) for call in calls)
    assert "combined=3 candidates" in statuses[-1]


def test_target_selection_uses_bounds_and_capture_orientation_before_ik():
    full_scene = scene()
    object_points = np.array([
        [x, y, z]
        for x in (-0.01, 0.0, 0.01)
        for y in (-0.01, 0.0, 0.01)
        for z in (0.39, 0.40, 0.41)
    ])
    target_scene = GraspScene(
        points_optical=object_points.copy(), points_base=object_points.copy(),
        pixels=np.tile([320.0, 240.0], (len(object_points), 1)), colors=None,
        p_wc=np.zeros(3), R_wc=np.eye(3), intrinsics=INTR,
        color_image=np.zeros((480, 640, 3), np.uint8))

    tilt = 1.30
    current_like = grasp_at([0.0, 0.0, 0.4])
    current_like.score = 0.90
    current_like.approach = np.array([np.sin(tilt), 0.0, -np.cos(tilt)])
    current_like.closing = np.array([0.0, -1.0, 0.0])
    higher_score_vertical = grasp_at([0.005, 0.0, 0.4])
    higher_score_vertical.score = 0.99
    off_target = grasp_at([0.20, 0.0, 0.4])
    off_target.score = 1.0
    below_support = grasp_at([0.0, 0.0, 0.375])
    below_support.score = 0.95
    reference = tool_rotation(
        current_like.approach, np.array([0.0, 1.0, 0.0]))
    full_scene.tool_rotation_base = reference
    calls = {"ik_closing": {}, "rejected": []}

    fake_node = object.__new__(GeminiPickNode)
    fake_node.mode = "target"
    fake_node.filter_cfg = FilterConfig(
        min_width=0.01, max_width=0.065, table_z=0.0,
        min_clearance=0.005, max_tilt=1.50,
        workspace_min=(-1.0, -1.0, -1.0),
        workspace_max=(1.0, 1.0, 1.0), pregrasp_standoff=0.10)
    fake_node._detect_in_target = lambda _scene: (
        [off_target, below_support, higher_score_vertical, current_like],
        target_scene, (320.0, 240.0))
    fake_node._param = lambda name: {
        "target_support_enabled": True,
        "target_support_z": 0.38,
        "table_z": 0.0,
        "target_bounds_margin_m": 0.020,
        "check_reachability": True,
        "gripper_scene_collision_enabled": False,
        "selection_score_slack": 0.15,
        "selection_tilt_slack_rad": np.deg2rad(10.0),
    }[name]
    fake_node._set_stage = lambda _stage: None
    fake_node._publish_status = lambda _message: None
    fake_node._publish_target_cloud = lambda _scene: None
    fake_node._publish_markers = lambda accepted, rejected: calls.update(
        rejected=rejected)
    fake_node._reachable_chain = lambda pregrasp, grasp, candidate: (
        calls["ik_closing"].__setitem__(id(candidate), candidate.closing.copy())
        or (True, "ok"))

    class Logger:
        def info(self, _message):
            pass

    fake_node.get_logger = lambda: Logger()

    selected, accepted = GeminiPickNode.select_grasp(fake_node, full_scene)

    assert selected is current_like
    assert accepted[0] is current_like
    assert all(candidate is not off_target for candidate in accepted)
    assert any(candidate is off_target for candidate in calls["rejected"])
    assert all(candidate is not below_support for candidate in accepted)
    assert any(candidate is below_support for candidate in calls["rejected"])
    assert calls["ik_closing"][id(current_like)] == pytest.approx(
        [0.0, 1.0, 0.0])
    assert selected.extras["orientation_delta_rad"] == pytest.approx(0.0)
    assert selected.extras["closing_axis_flipped"] == 1.0


def test_target_pick_launch_exposes_segmentation_and_table_overrides():
    launch = (Path(__file__).parents[1] / "launch" /
              "gemini_target_pick.launch.py").read_text()
    for argument in (
            "table_z", "target_support_enabled", "target_support_z",
            "target_support_collision_size_x",
            "target_support_collision_size_y",
            "target_support_collision_size_z",
            "target_support_collision_center_x",
            "target_support_collision_center_y",
            "target_crop_pad_px", "target_seed_radius_px",
            "target_depth_tolerance_m", "target_component_voxel_m",
            "target_component_min_points", "target_table_margin_m",
            "target_bounds_margin_m", "selection_score_slack",
            "selection_tilt_slack_rad"):
        assert f'DeclareLaunchArgument("{argument}"' in launch or \
            f'DeclareLaunchArgument(\n            "{argument}"' in launch


@pytest.mark.parametrize(
    "filename", ["gemini_pick.launch.py", "gemini_target_pick.launch.py"])
def test_pick_launches_pin_fastdds_before_moveit_and_gemini(filename):
    launch = (Path(__file__).parents[1] / "launch" / filename).read_text()

    assert 'name="RMW_IMPLEMENTATION"' in launch
    assert 'value="rmw_fastrtps_cpp"' in launch
    assert "args + [fastdds, moveit, node]" in launch


@pytest.mark.parametrize(
    "filename", ["gemini_pick.launch.py", "gemini_target_pick.launch.py"])
def test_pick_launches_expose_the_fail_closed_gripper_calibration(filename):
    launch = (Path(__file__).parents[1] / "launch" / filename).read_text()
    for argument in (
            "gripper_width_at_open_pos", "gripper_width_at_close_pos",
            "gripper_calibration_validated",
            "max_prevalidation_candidates"):
        assert (f'DeclareLaunchArgument("{argument}"' in launch
                or f'DeclareLaunchArgument(\n            "{argument}"' in launch)
        assert f'LaunchConfiguration("{argument}")' in launch
    assert launch.count('default_value="-1.0"') >= 2


def test_srdf_attaches_the_gripper_group_to_its_real_parent_link():
    path = (Path(__file__).parents[2] / "om6dof_moveit_config" /
            "config" / "om6dof.srdf")
    root = ET.parse(path).getroot()
    end_effector = root.find("./end_effector[@name='linear_gripper']")
    assert end_effector is not None
    assert end_effector.attrib["parent_link"] == "link7"


def test_tcp_frame_has_no_unmeasured_visual_or_collision_cube():
    path = (Path(__file__).parents[2] / "om6dof_description" /
            "urdf" / "om6dof.urdf.xacro")
    root = ET.parse(path).getroot()
    link = root.find("./link[@name='end_effector_link']")
    assert link is not None
    assert link.find("visual") is None
    assert link.find("collision") is None
    # Preserve the frame's inertial role and therefore the existing payload
    # and camera chain; only the phantom 10 mm geometry was removed.
    assert link.find("inertial") is not None


def test_target_surface_is_independent_from_the_main_table():
    fake_node = type("FakeNode", (), {})()
    values = {
        "target_support_enabled": False,
        "target_support_z": 0.24,
        "table_z": 0.01,
    }
    fake_node._param = values.__getitem__

    assert GeminiPickNode._target_surface_z(fake_node) == pytest.approx(0.01)
    values["target_support_enabled"] = True
    assert GeminiPickNode._target_surface_z(fake_node) == pytest.approx(0.24)


def test_preflight_adds_separate_measured_table_and_support_boxes():
    boxes = []

    class MoveIt:
        def apply_collision_box(self, name, size, pose):
            boxes.append((name, list(size), pose))
            return True

    class Logger:
        def error(self, _message):
            pytest.fail("valid measured collision geometry was rejected")

        def warn(self, _message):
            pass

    values = {
        "table_collision_enabled": True,
        "table_collision_size": [0.60, 0.80, 0.05],
        "table_collision_center_xy": [0.35, 0.0],
        "table_z": 0.01,
        "target_support_enabled": True,
        "target_support_z": 0.21,
        "target_support_collision_size_x": 0.30,
        "target_support_collision_size_y": 0.20,
        "target_support_collision_size_z": 0.20,
        "target_support_collision_center_x": 0.32,
        "target_support_collision_center_y": -0.04,
    }
    fake_node = object.__new__(GeminiPickNode)
    fake_node._param = values.__getitem__
    fake_node.moveit = MoveIt()
    fake_node.get_logger = lambda: Logger()

    assert GeminiPickNode._apply_table_collision(fake_node)
    assert [box[0] for box in boxes] == ["pick_table", "target_support"]
    assert boxes[0][1] == pytest.approx([0.60, 0.80, 0.05])
    assert [boxes[0][2].position.x, boxes[0][2].position.y,
            boxes[0][2].position.z] == pytest.approx([0.35, 0.0, -0.015])
    assert boxes[1][1] == pytest.approx([0.30, 0.20, 0.20])
    assert [boxes[1][2].position.x, boxes[1][2].position.y,
            boxes[1][2].position.z] == pytest.approx([0.32, -0.04, 0.11])


def test_enabled_support_with_unmeasured_dimensions_fails_closed():
    errors = []
    values = {
        "table_collision_enabled": False,
        "target_support_enabled": True,
        "target_support_z": 0.21,
        "target_support_collision_size_x": 0.0,
        "target_support_collision_size_y": 0.0,
        "target_support_collision_size_z": 0.0,
        "target_support_collision_center_x": 0.0,
        "target_support_collision_center_y": 0.0,
    }
    fake_node = object.__new__(GeminiPickNode)
    fake_node._param = values.__getitem__
    fake_node.moveit = SimpleNamespace(
        apply_collision_box=lambda *_args: pytest.fail(
            "invalid support must not reach MoveIt"))
    fake_node.get_logger = lambda: SimpleNamespace(
        warn=lambda _message: None, error=errors.append)

    assert not GeminiPickNode._apply_table_collision(fake_node)
    assert errors and "positive finite dimensions" in errors[0]


# --- projecting a grasp back into the image, for the classification crop -----

INTR = (600.0, 600.0, 320.0, 240.0)


def scene(p_wc=(0.0, 0.0, 0.0), R_wc=None):
    return GraspScene(points_optical=np.zeros((0, 3)),
                      points_base=np.zeros((0, 3)), pixels=np.zeros((0, 2)),
                      colors=None, p_wc=np.array(p_wc, dtype=float),
                      R_wc=np.eye(3) if R_wc is None else R_wc,
                      intrinsics=INTR,
                      color_image=np.zeros((480, 640, 3), np.uint8))


def grasp_at(position):
    return GraspCandidate(position=np.array(position, dtype=float),
                          approach=np.array([0.0, 0.0, -1.0]),
                          closing=np.array([0.0, 1.0, 0.0]),
                          width=0.02, score=0.5)


def _near_miss_marker_node(*, enabled=True):
    published = []
    node = object.__new__(GeminiPickNode)
    node.base_frame = "world"
    node.pub_near_miss_markers = SimpleNamespace(publish=published.append)
    node._param = lambda name: {
        "near_miss_markers_enabled": enabled,
    }[name]
    node.get_clock = lambda: Clock(clock_type=ClockType.SYSTEM_TIME)
    return node, published


def _marker_xyz(marker):
    return np.asarray([[point.x, point.y, point.z]
                       for point in marker.points], dtype=float)


def test_near_miss_marker_shows_one_gripper_exact_tcp_path_and_literal_reason():
    node, published = _near_miss_marker_node()
    candidate = grasp_at([0.30, 0.0, 0.20])
    candidate.approach = np.array([1.0, 0.0, 0.0])
    # AnyGrasp's commanded tip is intentionally different from its geometric
    # centre. The diagnostic arrow must describe the path that motion would use.
    candidate.extras["tcp_position"] = np.array([0.34, 0.0, 0.20])
    detail = "3 scene points occupy the swept gripper envelope (limit 2)"
    near_miss = NearMiss(
        candidate=candidate,
        rejection=Rejection("scene_collision", detail),
        gate_progress=9, violation=1.0,
        collision_mask=np.array([False, True, False, True]))
    scene_points = np.array([
        [9.0, 9.0, 9.0],
        [0.31, 0.02, 0.20],
        [8.0, 8.0, 8.0],
        [0.32, -0.02, 0.20],
    ])

    GeminiPickNode._publish_near_miss_markers(
        node, near_miss, scene_points, pregrasp_standoff=0.10)

    assert len(published) == 1
    markers = published[0].markers
    assert markers[0].action == markers[0].DELETEALL
    grippers = [marker for marker in markers if marker.type == marker.LINE_LIST]
    arrows = [marker for marker in markers if marker.type == marker.ARROW]
    texts = [marker for marker in markers
             if marker.type == marker.TEXT_VIEW_FACING]
    collision_points = [marker for marker in markers
                        if marker.type == marker.POINTS]
    assert len(grippers) == 1, "a near miss is one proposal, not candidate clutter"
    assert grippers[0].scale.x == pytest.approx(0.010)
    assert grippers[0].color.a == pytest.approx(1.0)
    assert len(arrows) == 1
    assert _marker_xyz(arrows[0]) == pytest.approx(np.vstack((
        candidate.pregrasp(0.10), candidate.motion_position())))
    assert len(texts) == 1
    assert detail in texts[0].text, \
        "the operator needs the filter's literal rejection evidence"
    assert len(collision_points) == 1
    assert _marker_xyz(collision_points[0]) == pytest.approx(
        scene_points[near_miss.collision_mask])


@pytest.mark.parametrize("enabled,near_miss", [
    (False, NearMiss(
        candidate=grasp_at([0.30, 0.0, 0.20]),
        rejection=Rejection("tilt", "one degree over the limit"),
        gate_progress=3, violation=0.01)),
    (True, None),
])
def test_near_miss_marker_clear_and_disabled_publish_deleteall_only(
        enabled, near_miss):
    node, published = _near_miss_marker_node(enabled=enabled)

    GeminiPickNode._publish_near_miss_markers(
        node, near_miss, None, pregrasp_standoff=0.10)

    assert len(published) == 1
    assert len(published[0].markers) == 1
    assert published[0].markers[0].action == \
        published[0].markers[0].DELETEALL


def test_near_miss_collision_points_require_a_mask_aligned_to_the_scene():
    node, published = _near_miss_marker_node()
    candidate = grasp_at([0.30, 0.0, 0.20])
    near_miss = NearMiss(
        candidate=candidate,
        rejection=Rejection("scene_collision", "collision detail"),
        gate_progress=9, violation=1.0,
        collision_mask=np.array([True, False]))

    GeminiPickNode._publish_near_miss_markers(
        node, near_miss, np.zeros((3, 3)), pregrasp_standoff=0.10)

    marker_types = [marker.type for marker in published[0].markers]
    assert marker_types.count(Marker.POINTS) == 0
    assert marker_types.count(Marker.LINE_LIST) == 1
    assert marker_types.count(Marker.ARROW) == 1
    assert marker_types.count(Marker.TEXT_VIEW_FACING) == 1


def test_near_miss_safe_wrapper_never_changes_control_flow(monkeypatch):
    node = object.__new__(GeminiPickNode)
    warnings = []
    node.get_logger = lambda: SimpleNamespace(warn=warnings.append)
    candidate = grasp_at([0.30, 0.0, 0.20])
    near_miss = NearMiss(
        candidate=candidate, rejection=Rejection("tilt", "too tilted"),
        gate_progress=3, violation=0.1)

    def fail_publish(*_args, **_kwargs):
        raise RuntimeError("synthetic RViz failure")

    monkeypatch.setattr(node, "_publish_near_miss_markers", fail_publish)

    GeminiPickNode._publish_near_miss_markers_safely(
        node, near_miss, np.zeros((1, 3)), pregrasp_standoff=0.10)

    assert len(warnings) == 1
    assert "synthetic RViz failure" in warnings[0]
    assert "motion unaffected" in warnings[0]


def test_moveit_rviz_enables_the_non_executable_near_miss_layer():
    config = (Path(__file__).parents[2] / "om6dof_moveit_config" /
              "config" / "moveit.rviz").read_text()
    assert "Name: Best Near Miss (NOT EXECUTABLE)" in config
    assert "Value: /gemini_pick/near_miss_markers" in config


def test_moveit_rviz_makes_the_planned_path_unmistakably_visible():
    config = (Path(__file__).parents[2] / "om6dof_moveit_config" /
              "config" / "moveit.rviz").read_text()
    planned_path = config.split("      Planned Path:\n", 1)[1].split(
        "      Planning Metrics:\n", 1)[0]
    settings = {}
    for line in planned_path.splitlines():
        if (line.startswith("        ") and not line.startswith("          ")
                and ": " in line):
            name, value = line.strip().split(": ", 1)
            settings[name] = value

    assert settings["Color Enabled"] == "true"
    assert settings["Interrupt Display"] == "true"
    assert settings["Loop Animation"] == "true"
    assert float(settings["Robot Alpha"]) >= 0.9
    assert settings["Show Trail"] == "true"
    assert settings["State Display Time"] == "0.15 s"
    assert int(settings["Trail Step Size"]) >= 8
    assert settings["Trajectory Topic"] == "/display_planned_path"


def test_moveit_rviz_keeps_the_world_cloud_from_hiding_planned_motion():
    config = (Path(__file__).parents[2] / "om6dof_moveit_config" /
              "config" / "moveit.rviz").read_text()
    world_cloud = config.split(
        "      Name: RealSense World Cloud\n", 1)[0].rsplit(
            "    - Alpha: ", 1)[1]
    assert float(world_cloud.splitlines()[0]) <= 0.5
    assert "    Background Color: 48; 48; 48" in config


def test_debug_grasp_markers_show_every_nonselected_candidate_opaque():
    published = []
    node = object.__new__(GeminiPickNode)
    node.base_frame = "world"
    node.pub_debug_markers = SimpleNamespace(publish=published.append)
    node._param = lambda name: {
        "debug_grasp_markers_enabled": True,
    }[name]
    node.get_clock = lambda: Clock(clock_type=ClockType.SYSTEM_TIME)
    selected = grasp_at([0.10, 0.0, 0.20])
    valid_unselected = grasp_at([0.20, 0.0, 0.20])
    collision = grasp_at([0.30, 0.0, 0.20])
    unreachable = grasp_at([0.40, 0.0, 0.20])

    GeminiPickNode._publish_debug_markers(
        node, [selected, valid_unselected], [
            (collision, Rejection("scene_collision", "obstacle")),
            (unreachable, Rejection("reachability", "IK failed")),
        ])

    assert len(published) == 1
    markers = published[0].markers
    assert markers[0].action == markers[0].DELETEALL
    glyphs = markers[1:]
    assert [marker.ns for marker in glyphs] == [
        "debug_valid_unselected",
        "debug_scene_collision",
        "debug_reachability",
    ]
    assert all(marker.type == marker.LINE_LIST for marker in glyphs)
    assert all(len(marker.points) == 8 for marker in glyphs)
    assert all(marker.color.a == pytest.approx(1.0) for marker in glyphs)
    assert all(marker.scale.x == pytest.approx(0.006) for marker in glyphs)
    # The selected grasp is intentionally absent: it stays exclusively on the
    # thick red, safety-filtered ~/grasp_markers topic.
    debug_x = [marker.points[0].x for marker in glyphs]
    assert not any(abs(value - 0.10) < 0.02 for value in debug_x)


def test_selected_grasp_marker_keeps_the_bold_red_tutorial_style():
    published = []
    node = object.__new__(GeminiPickNode)
    node.base_frame = "world"
    node.pub_markers = SimpleNamespace(publish=published.append)
    node.get_clock = lambda: Clock(clock_type=ClockType.SYSTEM_TIME)

    GeminiPickNode._publish_markers(
        node, [grasp_at([0.20, 0.0, 0.20])], [])

    glyph = published[0].markers[1]
    assert glyph.ns == "parallel_gripper_selected"
    assert glyph.type == glyph.LINE_LIST
    assert glyph.scale.x == pytest.approx(0.014)
    assert (glyph.color.r, glyph.color.g, glyph.color.b, glyph.color.a) == \
        pytest.approx((1.0, 0.05, 0.05, 1.0))


@pytest.mark.parametrize(("reason", "expected_rgb"), [
    ("valid_unselected", (0.05, 1.0, 0.10)),
    ("scene_collision", (1.0, 0.05, 0.85)),
    ("reachability", (1.0, 0.45, 0.02)),
    ("tilt", (0.05, 0.75, 1.0)),
    ("width", (0.55, 0.20, 1.0)),
    ("workspace", (0.10, 0.35, 1.0)),
    ("off_target", (0.10, 0.35, 1.0)),
    ("clearance", (1.0, 0.85, 0.05)),
    ("unknown_reason", (0.80, 0.80, 0.80)),
])
def test_debug_grasp_reason_colors_are_stable_and_opaque(
        reason, expected_rgb):
    color = GeminiPickNode._debug_grasp_color(reason)

    assert (color.r, color.g, color.b) == pytest.approx(expected_rgb)
    assert color.a == pytest.approx(1.0)


def test_all_rejected_live_case_publishes_thirteen_debug_grippers_only():
    debug_messages = []
    selected_messages = []
    node = object.__new__(GeminiPickNode)
    node.base_frame = "world"
    node.pub_debug_markers = SimpleNamespace(publish=debug_messages.append)
    node.pub_markers = SimpleNamespace(publish=selected_messages.append)
    node._param = lambda name: {
        "debug_grasp_markers_enabled": True,
    }[name]
    node.get_clock = lambda: Clock(clock_type=ClockType.SYSTEM_TIME)
    candidates = [grasp_at([0.10 + index * 0.01, 0.0, 0.20])
                  for index in range(13)]
    rejected = [
        (candidate, Rejection(
            "scene_collision" if index < 11 else
            "reachability" if index == 11 else "tilt",
            "debug rejection"))
        for index, candidate in enumerate(candidates)
    ]

    GeminiPickNode._publish_markers(node, [], candidates)
    GeminiPickNode._publish_debug_markers(node, [], rejected)

    assert len(selected_messages[0].markers) == 1
    assert selected_messages[0].markers[0].action == \
        selected_messages[0].markers[0].DELETEALL
    debug = debug_messages[0].markers
    assert len(debug) == 14  # one DELETEALL plus thirteen gripper glyphs
    namespaces = [marker.ns for marker in debug[1:]]
    assert namespaces.count("debug_scene_collision") == 11
    assert namespaces.count("debug_reachability") == 1
    assert namespaces.count("debug_tilt") == 1
    assert all(marker.action == marker.ADD for marker in debug[1:])
    assert all(marker.color.a == pytest.approx(1.0) for marker in debug[1:])


def test_disabled_debug_layer_clears_stale_markers():
    published = []
    node = object.__new__(GeminiPickNode)
    node.base_frame = "world"
    node.pub_debug_markers = SimpleNamespace(publish=published.append)
    node._param = lambda name: {
        "debug_grasp_markers_enabled": False,
    }[name]

    GeminiPickNode._publish_debug_markers(
        node, [], [(grasp_at([0.2, 0.0, 0.2]),
                    Rejection("off_target", "outside"))])

    assert len(published[0].markers) == 1
    assert published[0].markers[0].action == \
        published[0].markers[0].DELETEALL


def test_debug_publisher_failure_never_changes_pipeline_control_flow():
    warnings = []
    node = object.__new__(GeminiPickNode)
    node._publish_debug_markers = lambda *_args: (_ for _ in ()).throw(
        RuntimeError("RViz publisher unavailable"))
    node.get_logger = lambda: SimpleNamespace(warn=warnings.append)

    GeminiPickNode._publish_debug_markers_safely(node, [], [])

    assert warnings
    assert "motion unaffected" in warnings[0]


def test_moveit_rviz_enables_the_separate_non_safe_debug_topic():
    config = (Path(__file__).parents[2] / "om6dof_moveit_config" /
              "config" / "moveit.rviz").read_text()
    assert "Name: Non-selected Grasps (DEBUG ONLY)" in config
    assert "Value: /gemini_pick/debug_grasp_markers" in config


def test_a_point_on_the_optical_axis_projects_to_the_principal_point():
    pixel = GeminiPickNode._project_into_image(scene(), grasp_at([0.0, 0.0, 0.4]))
    assert pixel == pytest.approx((320.0, 240.0))


def test_the_camera_translation_is_subtracted_before_projecting():
    pixel = GeminiPickNode._project_into_image(
        scene(p_wc=(0.1, 0.0, 0.0)), grasp_at([0.1, 0.0, 0.4]))
    assert pixel == pytest.approx((320.0, 240.0))


def test_a_grasp_behind_the_camera_has_no_pixel():
    assert GeminiPickNode._project_into_image(
        scene(), grasp_at([0.0, 0.0, -0.4])) is None


def test_a_grasp_outside_the_frame_has_no_pixel():
    assert GeminiPickNode._project_into_image(
        scene(), grasp_at([2.0, 0.0, 0.4])) is None
