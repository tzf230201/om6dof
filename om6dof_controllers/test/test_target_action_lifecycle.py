"""Offline regression tests for ownership of web-target trajectory actions.

No ROS node or action server is started. Controlled futures exercise responses
in different orders, including a Stop pressed before the goal is accepted.
"""

import json
from types import SimpleNamespace

import numpy as np
import pytest
from action_msgs.msg import GoalStatus

from test_controller_node import _controller, _jog, _target
from om6dof_controllers.control_math import MODE_JOINT, MODE_READY


class _ControlledFuture:
    def __init__(self):
        self.callbacks = []
        self.completed = False
        self.value = None
        self.error = None

    def add_done_callback(self, callback):
        self.callbacks.append(callback)
        if self.completed:
            callback(self)

    def result(self):
        if self.error is not None:
            raise self.error
        assert self.completed, "Test resolved an incomplete future"
        return self.value

    def resolve(self, value=None, error=None):
        assert not self.completed
        self.completed = True
        self.value = value
        self.error = error
        for callback in list(self.callbacks):
            callback(self)

    def replay_callbacks(self):
        """Simulate a late/duplicated callback after a new generation starts."""
        assert self.completed
        for callback in list(self.callbacks):
            callback(self)


class _GoalHandle:
    def __init__(self, accepted=True, identity=1):
        self.accepted = accepted
        self.goal_id = SimpleNamespace(uuid=[identity] * 16)
        self.result_future = _ControlledFuture()
        self.cancel_futures = []
        self.cancel_error = None
        self.result_error = None

    def get_result_async(self):
        if self.result_error is not None:
            raise self.result_error
        return self.result_future

    def cancel_goal_async(self):
        if self.cancel_error is not None:
            raise self.cancel_error
        future = _ControlledFuture()
        self.cancel_futures.append(future)
        return future

    def acknowledge_cancel(self, accepted=True, index=-1):
        self.cancel_futures[index].resolve(SimpleNamespace(
            return_code=0 if accepted else 1,
            goals_canceling=[SimpleNamespace(goal_id=self.goal_id)]
            if accepted else [],
        ))

    def finish(self, status, error_code=0, error_string=""):
        self.result_future.resolve(SimpleNamespace(
            status=status,
            result=SimpleNamespace(
                error_code=error_code, error_string=error_string,
            ),
        ))


class _ControlledActionClient:
    def __init__(self):
        self.goals = []
        self.response_futures = []
        self.ready = True
        self.send_error = None

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal):
        if self.send_error is not None:
            raise self.send_error
        self.goals.append(goal)
        future = _ControlledFuture()
        self.response_futures.append(future)
        return future

    def cancel_all_goals_async(self):
        raise AssertionError("Stop target must never cancel unrelated goals")

    def accept(self, accepted=True, index=-1):
        handle = _GoalHandle(accepted=accepted, identity=len(self.goals))
        self.response_futures[index].resolve(handle)
        return handle


def _node():
    node = _controller()
    node.arm_client = _ControlledActionClient()
    return node


def _move(node, request_id="move-1"):
    return _target(
        node, action="move", mode=MODE_JOINT,
        values=[0.2, -0.3, 0.2, 0.1, 0.4, 0.0],
        reach_duration=5.0, request_id=request_id,
    )


def _status(node):
    return json.loads(node.target_status_pub.messages[-1].data)


def test_target_is_not_running_until_action_server_accepts():
    node = _node()
    submitted = _move(node)
    assert submitted["active"]
    assert submitted["state"] not in {"running", "reached", "stopped"}
    assert len(node.arm_client.goals) == 1

    node.arm_client.accept()
    assert _status(node)["state"] == "running"
    assert _status(node)["active"]


def test_rejected_action_goal_never_becomes_running():
    node = _node()
    _move(node)
    handle = node.arm_client.accept(accepted=False)
    assert not _status(node)["active"]
    assert _status(node)["state"] == "rejected"
    assert not handle.cancel_futures


def test_stop_pending_acceptance_cancels_the_eventual_handle_only():
    node = _node()
    _move(node)
    stopping = _target(node, action="stop", request_id="stop-1")
    assert stopping["active"]
    assert stopping["state"] != "stopped"

    handle = node.arm_client.accept()
    assert len(handle.cancel_futures) == 1
    assert _status(node)["state"] != "running"
    handle.acknowledge_cancel()
    assert _status(node)["active"]
    assert _status(node)["state"] != "stopped"

    handle.finish(GoalStatus.STATUS_CANCELED)
    assert _status(node)["state"] == "stopped"
    assert not _status(node)["active"]


def test_stop_accepted_target_waits_for_canceled_result_not_cancel_ack():
    node = _node()
    _move(node)
    handle = node.arm_client.accept()
    _target(node, action="stop", request_id="stop-1")
    assert len(handle.cancel_futures) == 1
    handle.acknowledge_cancel()
    assert _status(node)["active"]
    assert _status(node)["state"] != "stopped"

    handle.finish(GoalStatus.STATUS_CANCELED)
    assert _status(node)["state"] == "stopped"
    assert not _status(node)["active"]


def test_cancel_rejection_preserves_goal_and_allows_stop_retry():
    node = _node()
    _move(node)
    handle = node.arm_client.accept()
    _target(node, action="stop")
    handle.acknowledge_cancel(accepted=False)
    assert _status(node)["active"]
    assert _status(node)["state"] != "stopped"

    _target(node, action="stop", request_id="retry")
    assert len(handle.cancel_futures) == 2
    handle.acknowledge_cancel()
    handle.finish(GoalStatus.STATUS_CANCELED)
    assert _status(node)["state"] == "stopped"


@pytest.mark.parametrize("status,error_code", [
    (GoalStatus.STATUS_ABORTED, -4),
    (GoalStatus.STATUS_SUCCEEDED, -5),
])
def test_unsuccessful_result_never_reports_reached(status, error_code):
    node = _node()
    _move(node)
    handle = node.arm_client.accept()
    handle.finish(status, error_code, "trajectory tolerance violated")
    result = _status(node)
    assert not result["active"]
    assert result["state"] not in {"running", "reached", "stopped"}
    assert "tolerance" in result["message"].lower()


def test_feedback_alone_does_not_release_target_action_ownership():
    node = _node()
    _move(node)
    handle = node.arm_client.accept()
    node.joint_positions = dict(zip(node.joint_names, node.target_command_joints))
    node._publish_target_status_locked("running", "Tracking")
    assert _status(node)["active"]
    _target(node, action="stop")
    assert len(handle.cancel_futures) == 1


def test_successful_result_releases_target_ownership():
    node = _node()
    _move(node)
    handle = node.arm_client.accept()
    node.joint_positions = dict(zip(node.joint_names, node.target_command_joints))
    handle.finish(GoalStatus.STATUS_SUCCEEDED)
    assert not _status(node)["active"]
    assert _status(node)["state"] == "reached"


def test_late_result_from_previous_generation_does_not_overwrite_new_target():
    node = _node()
    _move(node)
    first = node.arm_client.accept()
    first.finish(GoalStatus.STATUS_SUCCEEDED)
    _move(node, request_id="move-2")
    second = node.arm_client.accept()
    before = _status(node)

    first.result_future.replay_callbacks()
    assert _status(node) == before
    assert _status(node)["request_id"] == "move-2"
    assert not second.cancel_futures


def test_late_cancel_ack_does_not_modify_new_target():
    node = _node()
    _move(node)
    first = node.arm_client.accept()
    _target(node, action="stop")
    first.finish(GoalStatus.STATUS_CANCELED)

    _move(node, request_id="move-2")
    second = node.arm_client.accept()
    before = _status(node)
    first.acknowledge_cancel()
    assert _status(node) == before
    assert not second.cancel_futures


def test_target_stop_does_not_cancel_non_target_ready_trajectory():
    node = _node()
    node._send_pose_locked(MODE_READY, [node.transition_pose, node.ready_pose])
    ready_handle = node.arm_client.accept()
    _target(node, action="stop")
    assert not ready_handle.cancel_futures
    assert len(node.arm_client.goals) == 1


@pytest.mark.parametrize("phase", ["pending", "running", "canceling"])
def test_owned_target_blocks_jog_offsets_in_every_nonterminal_phase(phase):
    node = _node()
    _move(node)
    if phase != "pending":
        node.arm_client.accept()
    if phase == "canceling":
        _target(node, action="stop")

    _jog(node, [0.5, 0, 0, 0, 0, 0], dt=0.1, repeats=2)
    assert node.offset == pytest.approx(np.zeros(6))
    assert node.offset_pub.messages[-1].data == pytest.approx([0.0] * 6)


def test_second_move_does_not_replace_unfinished_owned_goal():
    node = _node()
    _move(node)
    handle = node.arm_client.accept()
    _move(node, request_id="move-2")
    assert len(node.arm_client.goals) == 1
    assert _status(node)["active"]
    _target(node, action="stop")
    assert len(handle.cancel_futures) == 1


def test_send_exception_is_fail_closed_without_fake_running_target():
    node = _node()
    node.arm_client.send_error = RuntimeError("send failed")
    result = _move(node)
    assert result["active"]
    assert result["state"] not in {"running", "reached"}


def test_cancel_exception_keeps_target_owned_for_retry():
    node = _node()
    _move(node)
    handle = node.arm_client.accept()
    handle.cancel_error = RuntimeError("cancel transport unavailable")
    result = _target(node, action="stop")
    assert result["active"]
    assert result["state"] != "stopped"
    handle.cancel_error = None
    _target(node, action="stop")
    assert len(handle.cancel_futures) == 1


def test_goal_response_exception_retains_uncertain_ownership():
    node = _node()
    _move(node)
    node.arm_client.response_futures[-1].resolve(
        error=RuntimeError("goal response lost")
    )
    assert _status(node)["active"]
    assert _status(node)["state"] not in {"running", "stopped", "reached"}
    _move(node, request_id="move-2")
    assert len(node.arm_client.goals) == 1


def test_result_future_exception_cannot_claim_motion_stopped():
    node = _node()
    _move(node)
    handle = node.arm_client.accept()
    handle.result_future.resolve(error=RuntimeError("result disconnected"))
    assert _status(node)["active"]
    assert _status(node)["state"] not in {"running", "stopped", "reached"}


def test_failure_to_subscribe_result_still_retains_handle_for_cancel():
    node = _node()
    _move(node)
    handle = _GoalHandle()
    handle.result_error = RuntimeError("result subscription failed")
    node.arm_client.response_futures[-1].resolve(handle)
    assert _status(node)["active"]
    assert _status(node)["state"] not in {"running", "stopped", "reached"}
    _target(node, action="stop")
    assert len(handle.cancel_futures) == 1


def test_unrelated_goal_uuid_does_not_confirm_target_cancellation():
    node = _node()
    _move(node)
    handle = node.arm_client.accept()
    _target(node, action="stop")
    handle.cancel_futures[-1].resolve(SimpleNamespace(
        return_code=0,
        goals_canceling=[SimpleNamespace(
            goal_id=SimpleNamespace(uuid=[99] * 16)
        )],
    ))
    assert _status(node)["active"]
    assert _status(node)["state"] not in {"running", "stopped", "reached"}
    _target(node, action="stop")
    assert len(handle.cancel_futures) == 2


def test_stop_without_owned_goal_preserves_existing_offset():
    node = _node()
    _jog(node, [0.5, 0, 0, 0, 0, 0], dt=0.1)
    before = node.offset.copy()
    _target(node, action="stop")
    assert node.offset == pytest.approx(before)
    assert not node.arm_client.goals


@pytest.mark.parametrize("phase", ["pending", "running", "canceling"])
def test_ready_command_does_not_preempt_unfinished_target(phase):
    from std_msgs.msg import String

    node = _node()
    _move(node)
    if phase != "pending":
        node.arm_client.accept()
    if phase == "canceling":
        _target(node, action="stop")
    node._on_operation_mode(String(data=MODE_READY))
    assert len(node.arm_client.goals) == 1
    assert node.motion_mode == MODE_JOINT


def test_stop_pending_goal_rejection_is_terminal_without_canceling():
    node = _node()
    _move(node)
    _target(node, action="stop")
    handle = node.arm_client.accept(accepted=False)
    assert _status(node)["state"] == "stopped"
    assert not _status(node)["active"]
    assert not handle.cancel_futures


def test_already_completed_send_future_is_not_overwritten_as_requesting():
    node = _node()
    handle = _GoalHandle()
    response = _ControlledFuture()
    response.resolve(handle)

    def send_completed(goal):
        node.arm_client.goals.append(goal)
        return response

    node.arm_client.send_goal_async = send_completed
    result = _move(node)
    assert result["state"] == "running"
    assert result["active"]
    _target(node, action="stop")
    assert len(handle.cancel_futures) == 1


def test_result_completing_during_stop_acceptance_is_not_canceled_again():
    node = _node()
    _move(node)
    _target(node, action="stop")
    handle = _GoalHandle()
    handle.finish(GoalStatus.STATUS_SUCCEEDED)
    node.arm_client.response_futures[-1].resolve(handle)
    assert not _status(node)["active"]
    assert _status(node)["state"] not in {"running", "requesting", "stopping"}
    assert not handle.cancel_futures
