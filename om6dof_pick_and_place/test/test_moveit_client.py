"""Pure request-building tests for the synchronous MoveIt wrapper."""

import threading

from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
from rclpy.task import Future  # noqa: E402

from action_msgs.msg import GoalStatus  # noqa: E402
from action_msgs.srv import CancelGoal  # noqa: E402
from moveit_msgs.msg import RobotState, RobotTrajectory  # noqa: E402
from trajectory_msgs.msg import JointTrajectoryPoint  # noqa: E402

from om6dof_pick_and_place.moveit_client import MoveItClient  # noqa: E402


def bare_client():
    client = object.__new__(MoveItClient)
    client.reference_frame = "world"
    client.num_planning_attempts = 3
    client.planning_time = 2.0
    client.vel_scale = 0.2
    client.acc_scale = 0.2
    client._plan_only_start_state = None
    client._controller_cancel_futures = {}
    client._controller_cancel_last_sent = {}
    client._physical_move_goal = False
    client._physical_gripper_goal = False
    return client


def test_request_carries_explicit_pilz_lin_selection():
    request = bare_client()._new_plan_request(
        "arm", pipeline_id="pilz_industrial_motion_planner", planner_id="LIN")
    assert request.group_name == "arm"
    assert request.pipeline_id == "pilz_industrial_motion_planner"
    assert request.planner_id == "LIN"
    assert request.start_state.is_diff


def test_plan_only_chain_remembers_the_planned_endpoint():
    client = bare_client()
    start = RobotState()
    start.joint_state.name = ["joint1", "joint2"]
    start.joint_state.position = [0.0, 0.0]
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = ["joint1", "joint2"]
    point = JointTrajectoryPoint()
    point.positions = [0.4, -0.2]
    trajectory.joint_trajectory.points = [point]
    result = SimpleNamespace(trajectory_start=start,
                             planned_trajectory=trajectory)

    client._remember_planned_end_state(result)

    assert client._plan_only_start_state.joint_state.position \
        == pytest.approx([0.4, -0.2])
    # Caching must not mutate the response object supplied by MoveIt.
    assert start.joint_state.position == pytest.approx([0.0, 0.0])


def test_plan_only_gripper_update_is_carried_into_the_next_arm_request():
    client = bare_client()
    client._plan_only_start_state = RobotState()
    client._plan_only_start_state.joint_state.name = ["joint1"]
    client._plan_only_start_state.joint_state.position = [0.2]

    assert client.set_plan_only_joint_state(
        ["gripper_left_joint", "gripper_right_joint"], -0.01)
    state = client._plan_only_start_state.joint_state
    positions = dict(zip(state.name, state.position))
    assert positions["joint1"] == pytest.approx(0.2)
    assert positions["gripper_left_joint"] == pytest.approx(-0.01)
    assert positions["gripper_right_joint"] == pytest.approx(-0.01)


def plan_result(start_position, end_position):
    start = RobotState()
    start.joint_state.name = ["joint1"]
    start.joint_state.position = [float(start_position)]
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = ["joint1"]
    point = JointTrajectoryPoint()
    point.positions = [float(end_position)]
    trajectory.joint_trajectory.points = [point]
    return SimpleNamespace(
        trajectory_start=start, planned_trajectory=trajectory)


def display_client():
    published = []
    client = bare_client()
    logger = SimpleNamespace(
        info=lambda _message: None, warn=lambda _message: None)
    client.node = SimpleNamespace(get_logger=lambda: logger)
    client._display_trajectory_pub = SimpleNamespace(publish=published.append)
    client._display_model_id = "om6dof"
    client._plan_only_display_active = False
    client._plan_only_display_start = None
    client._plan_only_display_segments = []
    return client, published


def test_plan_only_display_retains_first_start_and_order_as_deep_copies():
    client, published = display_client()
    first = plan_result(0.1, 0.2)
    second = plan_result(0.2, 0.3)

    client.begin_plan_only_display()
    client._record_plan_only_display(first)
    client._record_plan_only_display(second)
    first.trajectory_start.joint_state.position[0] = 9.0
    first.planned_trajectory.joint_trajectory.points[0].positions[0] = 8.0
    second.planned_trajectory.joint_trajectory.points[0].positions[0] = 7.0
    assert client.publish_plan_only_display()
    assert not client.publish_plan_only_display()

    assert len(published) == 1
    display = published[0]
    assert display.model_id == "om6dof"
    assert display.trajectory_start.joint_state.position == pytest.approx([0.1])
    assert [segment.joint_trajectory.points[0].positions[0]
            for segment in display.trajectory] == pytest.approx([0.2, 0.3])


def test_plan_only_display_ignores_results_until_capture_is_active():
    client, published = display_client()

    client._record_plan_only_display(plan_result(0.1, 0.2))
    assert not client.publish_plan_only_display()

    assert published == []


def test_plan_only_display_with_no_segments_does_not_publish():
    client, published = display_client()

    client.begin_plan_only_display()
    assert not client.publish_plan_only_display()

    assert published == []


def test_discard_plan_only_display_clears_the_buffer():
    client, published = display_client()

    client.begin_plan_only_display()
    client._record_plan_only_display(plan_result(0.1, 0.2))
    client.discard_plan_only_display()
    assert not client.publish_plan_only_display()

    assert published == []


def test_cancel_pending_goal_cancels_it_when_acceptance_arrives():
    warnings = []
    logger = SimpleNamespace(
        warn=warnings.append,
        error=lambda message: None,
    )
    client = bare_client()
    client.node = SimpleNamespace(get_logger=lambda: logger)
    client._goal_lock = threading.Lock()
    client._motion_faulted = False
    client._move_goal_pending = True
    client._pending_move_send_future = Future()
    client._cancel_move_on_accept = False
    client._current_move_goal = None
    client._gripper_goal_pending = False
    client._pending_gripper_send_future = None
    client._cancel_gripper_on_accept = False
    client._current_gripper_goal = None

    cancelled = []
    result_future = Future()
    handle = SimpleNamespace(
        accepted=True,
        cancel_goal_async=lambda: cancelled.append(True),
        get_result_async=lambda: result_future,
    )

    assert client.cancel_current_goal()
    assert client.motion_faulted
    client._pending_move_send_future.set_result(handle)

    assert cancelled == [True]
    assert client.action_in_flight
    assert any("Late MoveGroup acceptance" in item for item in warnings)

    # The client remains drainable until the action reaches a terminal result,
    # then releases both the accepted handle and its send future.
    result_future.set_result(SimpleNamespace(
        status=GoalStatus.STATUS_CANCELED))
    assert not client.action_in_flight


def test_direct_controller_stop_tracks_both_cancel_all_acknowledgements():
    class CancelClient:
        def __init__(self):
            self.future = Future()

        def service_is_ready(self):
            return True

        def call_async(self, request):
            assert request.goal_info.goal_id.uuid.tolist() == [0] * 16
            return self.future

    logger = SimpleNamespace(
        warn=lambda _message: None,
        error=lambda _message: None,
        info=lambda _message: None,
    )
    client = bare_client()
    client.node = SimpleNamespace(get_logger=lambda: logger)
    client._goal_lock = threading.Lock()
    client._move_goal_pending = False
    client._gripper_goal_pending = False
    client._current_move_goal = None
    client._current_gripper_goal = None
    client._pending_move_send_future = None
    client._pending_gripper_send_future = None
    client._arm_cancel_client = CancelClient()
    client._gripper_cancel_client = CancelClient()
    client._move_goal_pending = True
    client._gripper_goal_pending = True
    client._physical_move_goal = True
    client._physical_gripper_goal = True

    assert client.cancel_controller_goals() == 2
    # Model both owned actions reaching terminal state; the outstanding direct
    # cancel service calls alone must still keep the client non-restartable.
    client._move_goal_pending = False
    client._gripper_goal_pending = False
    client._physical_move_goal = False
    client._physical_gripper_goal = False
    assert client.action_in_flight

    response = CancelGoal.Response()
    response.return_code = CancelGoal.Response.ERROR_NONE
    client._arm_cancel_client.future.set_result(response)
    assert client.action_in_flight
    client._gripper_cancel_client.future.set_result(response)
    assert not client.action_in_flight


def test_begin_sequence_waits_for_old_controller_cancel_acknowledgement():
    client = bare_client()
    client._goal_lock = threading.Lock()
    client._motion_faulted = False
    client._commands_stopped = True
    client._move_goal_pending = False
    client._gripper_goal_pending = False
    client._current_move_goal = None
    client._current_gripper_goal = None
    client._pending_move_send_future = None
    client._pending_gripper_send_future = None
    client._controller_cancel_futures = {"arm controller": [Future()]}

    assert not client.begin_sequence()
    assert client._commands_stopped

    client._controller_cancel_futures.clear()
    assert client.begin_sequence()
    assert not client._commands_stopped


def test_direct_stop_targets_only_the_controller_owned_by_this_client():
    class CancelClient:
        def __init__(self):
            self.calls = 0

        def service_is_ready(self):
            return True

        def call_async(self, _request):
            self.calls += 1
            return Future()

    logger = SimpleNamespace(
        warn=lambda _message: None,
        error=lambda _message: None,
        info=lambda _message: None,
    )
    client = bare_client()
    client.node = SimpleNamespace(get_logger=lambda: logger)
    client._goal_lock = threading.Lock()
    client._move_goal_pending = True
    client._gripper_goal_pending = False
    client._current_move_goal = None
    client._current_gripper_goal = None
    client._pending_move_send_future = Future()
    client._pending_gripper_send_future = None
    client._physical_move_goal = True
    client._physical_gripper_goal = False
    client._arm_cancel_client = CancelClient()
    client._gripper_cancel_client = CancelClient()

    assert client.cancel_controller_goals() == 1
    assert client._arm_cancel_client.calls == 1
    assert client._gripper_cancel_client.calls == 0


def test_stuck_arm_cancel_does_not_suppress_a_new_gripper_cancel_round():
    class CancelClient:
        def __init__(self):
            self.futures = []

        def service_is_ready(self):
            return True

        def call_async(self, _request):
            future = Future()
            self.futures.append(future)
            return future

    logger = SimpleNamespace(
        warn=lambda _message: None,
        error=lambda _message: None,
        info=lambda _message: None,
    )
    client = bare_client()
    client.node = SimpleNamespace(get_logger=lambda: logger)
    client._goal_lock = threading.Lock()
    client._move_goal_pending = True
    client._gripper_goal_pending = True
    client._current_move_goal = None
    client._current_gripper_goal = None
    client._pending_move_send_future = Future()
    client._pending_gripper_send_future = Future()
    client._physical_move_goal = True
    client._physical_gripper_goal = True
    client._arm_cancel_client = CancelClient()
    client._gripper_cancel_client = CancelClient()

    assert client.cancel_controller_goals() == 2
    response = CancelGoal.Response()
    response.return_code = CancelGoal.Response.ERROR_REJECTED
    client._gripper_cancel_client.futures[0].set_result(response)

    # Arm remains unacknowledged, but the independently cleared gripper
    # endpoint must be eligible for another round immediately.
    assert client.cancel_controller_goals() == 1
    assert len(client._arm_cancel_client.futures) == 1
    assert len(client._gripper_cancel_client.futures) == 2


def test_stop_directly_cancels_physical_pending_acceptance_after_timeout():
    client = bare_client()
    client._goal_lock = threading.Lock()
    client._commands_stopped = False
    client._motion_faulted = True
    client._move_goal_pending = False
    client._gripper_goal_pending = False
    client._current_move_goal = None
    client._current_gripper_goal = None
    client._pending_move_send_future = Future()
    client._pending_gripper_send_future = None
    client._cancel_move_on_accept = False
    client._cancel_gripper_on_accept = False
    client._physical_move_goal = True
    calls = []
    client.cancel_controller_goals = lambda: calls.append(True) or 1
    client.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(
        warn=lambda _message: None, error=lambda _message: None))

    assert client.cancel_current_goal()
    assert calls == [True]


def test_action_cancel_exception_does_not_skip_direct_controller_fallback():
    errors = []
    handle = SimpleNamespace(cancel_goal_async=lambda: (_ for _ in ()).throw(
        RuntimeError("cancel transport failed")))
    client = bare_client()
    client._goal_lock = threading.Lock()
    client._commands_stopped = False
    client._motion_faulted = False
    client._move_goal_pending = False
    client._gripper_goal_pending = False
    client._current_move_goal = handle
    client._current_gripper_goal = None
    client._pending_move_send_future = None
    client._pending_gripper_send_future = None
    client._cancel_move_on_accept = False
    client._cancel_gripper_on_accept = False
    client._physical_move_goal = True
    calls = []
    client.cancel_controller_goals = lambda: calls.append(True) or 1
    client.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(
        warn=lambda _message: None, error=errors.append))

    assert client.cancel_current_goal()
    assert calls == [True]
    assert any("cancel transport failed" in message for message in errors)


def test_unknown_result_keeps_physical_ownership_and_fault_latched():
    cancellations = []
    handle = SimpleNamespace(
        cancel_goal_async=lambda: cancellations.append(True))
    client = bare_client()
    client._goal_lock = threading.Lock()
    client._motion_faulted = False
    client._move_goal_pending = False
    client._gripper_goal_pending = False
    client._current_move_goal = handle
    client._current_move_result_future = Future()
    client._current_gripper_goal = None
    client._pending_move_send_future = None
    client._pending_gripper_send_future = None
    client._physical_move_goal = True
    direct_calls = []
    client.cancel_controller_goals = lambda: direct_calls.append(True) or 1
    client.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(
        error=lambda _message: None))

    terminal = client._finalize_action_result(
        "MoveGroup", handle,
        SimpleNamespace(status=GoalStatus.STATUS_UNKNOWN))

    assert not terminal
    assert client._current_move_goal is handle
    assert client._physical_move_goal
    assert client.motion_faulted
    assert direct_calls == [True]
    assert cancellations == [True]


def test_late_acceptance_keeps_handle_when_get_result_request_raises():
    cancelled = []
    handle = SimpleNamespace(
        accepted=True,
        cancel_goal_async=lambda: cancelled.append(True),
        get_result_async=lambda: (_ for _ in ()).throw(
            RuntimeError("result transport failed")),
    )
    send_future = Future()
    client = bare_client()
    client._goal_lock = threading.Lock()
    client._motion_faulted = False
    client._move_goal_pending = True
    client._gripper_goal_pending = False
    client._current_move_goal = None
    client._current_move_result_future = None
    client._current_gripper_goal = None
    client._pending_move_send_future = send_future
    client._pending_gripper_send_future = None
    client._physical_move_goal = True
    direct_calls = []
    client.cancel_controller_goals = lambda: direct_calls.append(True) or 1
    client.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(
        warn=lambda _message: None, error=lambda _message: None))

    send_future.add_done_callback(
        lambda done: client._cancel_late_goal(done, "MoveGroup"))
    send_future.set_result(handle)

    assert client._current_move_goal is handle
    assert client._current_move_result_future is None
    assert client._physical_move_goal
    assert client.physical_action_in_flight
    assert client.motion_faulted
    assert direct_calls == [True]
    assert cancelled == [True]
