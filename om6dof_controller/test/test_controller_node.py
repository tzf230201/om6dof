"""Tests for the additive operator channel.

The node under test owns exactly one output: a bounded joint-space offset that
the hardware adds to whatever ``arm_controller`` is commanding. Every test here
is ultimately about one of two things -- that the offset tracks the operator,
and that it can never grow into something the arm should not be asked to do.
"""

import json
import math
import threading
import time

import numpy as np
import pytest
from action_msgs.msg import GoalStatus, GoalStatusArray
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectoryPoint

from om6dof_controller.control_math import (
    MODE_AUTONOMOUS,
    MODE_CARTESIAN,
    MODE_CYLINDRICAL,
    MODE_FLOAT,
    MODE_JOINT,
    MODE_READY,
    MODE_REST,
    MODE_SEMI_CYLINDRICAL,
    MODE_STARTUP,
)
from om6dof_controller.controller_node import (
    DEFAULT_JOINT_LOWER,
    DEFAULT_JOINT_UPPER,
    DEFAULT_READY_JOINT_POSITIONS_DEG,
    DEFAULT_TRANSITION_JOINT_POSITIONS_DEG,
    OM6DOFController,
)


class _Logger:
    def __init__(self):
        self.infos = []
        self.warnings = []
        self.errors = []

    def info(self, message, **_kwargs):
        self.infos.append(str(message))

    def warn(self, message, **_kwargs):
        self.warnings.append(str(message))

    def error(self, message, **_kwargs):
        self.errors.append(str(message))


class _Publisher:
    def __init__(self, topic_name="/fake"):
        self.messages = []
        self.topic_name = topic_name

    def publish(self, message):
        self.messages.append(message)


class _Future:
    def add_done_callback(self, _callback):
        return None


class _ActionClient:
    def __init__(self, ready=True):
        self.ready = ready
        self.goals = []

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal):
        self.goals.append(goal)
        return _Future()


class _IdentityIK:
    """Tip position is the first three joints; orientation is identity."""

    def __init__(self):
        self.collides = False

    def fk_pose(self, q):
        values = np.asarray(q, dtype=float)
        return values[:3].copy(), np.eye(3)

    def velocity_ik_priority(self, _q, linear, angular):
        return np.concatenate([linear, angular])

    def ee_to_base_angular(self, _q, angular):
        return np.asarray(angular, dtype=float)

    def self_collides(self, _q, _radius):
        return self.collides

    def manipulability(self, _q):
        return 1.0


INITIAL = [0.1, -0.4, 0.3, 0.2, 0.5, -0.1]


def _controller(mode=MODE_JOINT, ik=True):
    node = object.__new__(OM6DOFController)
    node.lock = threading.RLock()
    node.joint_names = [f"joint{index}" for index in range(1, 7)]
    node.arm_controller = "arm_controller"
    node.motion_mode = mode
    node.last_coordinate_mode = MODE_CARTESIAN

    node.offset = np.zeros(6)
    node.offset_blocked = False

    node.joint_positions = dict(zip(node.joint_names, INITIAL))
    node.last_joint_state = time.monotonic()
    node.startup_pose = list(INITIAL)
    node.control_velocity = np.zeros(6)
    node.last_control_cmd = 0.0
    node.last_tick = time.monotonic() - 0.02
    node.arm_reference = None
    node.seen_goal_ids = set()

    node.joint_state_timeout = 1.0
    node.control_cmd_timeout = 0.3
    node.max_joint_velocity = 1.2
    node.joint_lower = [value + 0.02 for value in DEFAULT_JOINT_LOWER]
    node.joint_upper = [value - 0.02 for value in DEFAULT_JOINT_UPPER]
    node.ready_pose = [
        math.radians(value) for value in DEFAULT_READY_JOINT_POSITIONS_DEG
    ]
    node.transition_pose = [
        math.radians(value) for value in DEFAULT_TRANSITION_JOINT_POSITIONS_DEG
    ]
    node.pose_profile_duration = 4.0
    node.target_reach_duration = 5.0

    node.max_cartesian_linear_velocity = 0.1
    node.max_cartesian_angular_velocity = 1.0
    node.max_cylindrical_theta_velocity = 0.5
    node.cylindrical_origin_xy = np.array([0.012, 0.0])
    node.cylindrical_min_radius = 0.03
    node.ik_tool_frame_rotation = True
    node.ik_manipulability_warning_threshold = 1.0e-6
    node.ik_self_collision = True
    node.ik_collision_radius = 0.025
    node.ik = _IdentityIK() if ik else None

    node.f1_destination = MODE_READY
    node.target_active = False
    node.target_mode = ""
    node.target_goal = None
    node.target_command_joints = None
    node.target_request_id = ""
    node.target_approximate = False
    node.target_status_state = "idle"
    node.target_status_message = "no target yet"
    node.last_target_status_publish = 0.0

    node.offset_pub = _Publisher("/forward_offset_controller/commands")
    node.operation_state_pub = _Publisher()
    node.remote_state_pub = _Publisher()
    node.gripper_state_pub = _Publisher()
    node.target_status_pub = _Publisher()
    node.arm_client = _ActionClient()
    node.gripper_client = _ActionClient()
    node._logger = _Logger()
    node.get_logger = lambda: node._logger
    node.get_parameter = lambda name: type(
        "P", (), {"value": {"float_follow_velocity": 0.35}[name]}
    )()
    return node


def _jog(node, velocity, dt=0.02, repeats=1):
    """Feed one fresh jog command and run `repeats` ticks of `dt` each."""
    for _ in range(repeats):
        node._on_control_cmd(Float64MultiArray(data=list(velocity)))
        node.last_tick = time.monotonic() - dt
        node._tick()


def _goal_status(uuid_byte, status=GoalStatus.STATUS_ACCEPTED):
    message = GoalStatusArray()
    entry = GoalStatus()
    entry.status = status
    entry.goal_info.goal_id.uuid = [uuid_byte] * 16
    message.status_list.append(entry)
    return message


# --------------------------------------------------------------- the output

def test_the_node_only_ever_publishes_an_offset_never_a_pose():
    """The whole point of the split: this channel is additive, so a zero
    command must mean 'no operator contribution', not 'go to zero'."""
    node = _controller()
    node._tick()
    assert node.offset_pub.messages[-1].data == pytest.approx([0.0] * 6)
    # An untouched arm sitting far from zero still publishes zeros.
    assert INITIAL != [0.0] * 6


def test_joint_jog_integrates_into_the_offset():
    node = _controller()
    _jog(node, [0.5, 0, 0, 0, 0, 0], dt=0.1)
    assert node.offset[0] == pytest.approx(0.05, abs=1e-6)
    assert node.offset[1:] == pytest.approx([0.0] * 5)
    assert node.offset_pub.messages[-1].data == pytest.approx(node.offset.tolist())


def test_joint_jog_accumulates_across_ticks():
    node = _controller()
    _jog(node, [0.5, 0, 0, 0, 0, 0], dt=0.1, repeats=3)
    assert node.offset[0] == pytest.approx(0.15, abs=1e-6)


def test_jog_velocity_is_clamped_to_the_joint_ceiling():
    node = _controller()
    _jog(node, [99.0, 0, 0, 0, 0, 0], dt=0.1)
    assert node.offset[0] == pytest.approx(node.max_joint_velocity * 0.1)


# ------------------------------------------------------------------ bounds

def test_offset_travel_is_not_halved_by_a_servo_that_follows():
    """Regression: the limit belongs on reference + offset.

    The measured pose already contains the offset, so clamping against it
    counted the operator's contribution twice and stopped the arm at roughly
    half its travel -- an invisible wall that moved as the offset grew. This
    fake arm settles on the command each tick, the way a real servo does.
    """
    node = _controller()
    reference = node.joint_positions["joint1"]        # arm_controller holds
    node.arm_reference = list(INITIAL)
    for _ in range(400):
        node._on_control_cmd(Float64MultiArray(data=[1.0, 0, 0, 0, 0, 0]))
        node.last_tick = time.monotonic() - 0.02
        node._tick()
        # The servo catches up: measured = what the motors were commanded.
        node.joint_positions["joint1"] = reference + node.offset[0]
    assert reference + node.offset[0] == pytest.approx(node.joint_upper[0])


def test_the_urdf_joint_range_is_the_only_bound_on_the_offset():
    """No separate offset ceiling: jogging for a long time must run the joint
    all the way to its URDF limit rather than stopping short of it."""
    node = _controller()
    start = node.joint_positions["joint1"]
    _jog(node, [1.0, 0, 0, 0, 0, 0], dt=0.1, repeats=200)
    assert start + node.offset[0] == pytest.approx(node.joint_upper[0])


def test_offset_is_bounded_so_the_total_stays_inside_joint_limits():
    node = _controller()
    # Park joint1 one hair below its upper limit, then push past it.
    near_limit = node.joint_upper[0] - 0.01
    node.joint_positions["joint1"] = near_limit
    _jog(node, [1.0, 0, 0, 0, 0, 0], dt=0.1, repeats=20)
    total = near_limit + node.offset[0]
    assert total <= node.joint_upper[0] + 1e-9
    assert node.offset[0] == pytest.approx(0.01, abs=1e-6)


def test_self_collision_blocks_a_jog_that_enters_the_boundary():
    node = _controller()

    class _EntersOnMove(_IdentityIK):
        def self_collides(self, q, _radius):
            # Clear where the arm is; colliding anywhere it would move to.
            return not np.allclose(np.asarray(q, dtype=float), INITIAL)

    node.ik = _EntersOnMove()
    _jog(node, [0.5, 0, 0, 0, 0, 0], dt=0.1)
    assert node.offset == pytest.approx(np.zeros(6))
    assert any("self-collision" in msg for msg in node._logger.warnings)


def test_an_arm_already_inside_the_boundary_can_still_be_jogged_out():
    """The collision model flags poses the real arm rests in safely, so
    refusing from inside one would trap the operator with no way back out."""
    node = _controller()
    node.ik.collides = True          # every pose, including where it sits
    _jog(node, [0.5, 0, 0, 0, 0, 0], dt=0.1)
    assert node.offset[0] == pytest.approx(0.05, abs=1e-6)


# ------------------------------------------------------------- staleness

def test_stale_jog_stream_freezes_the_offset_without_dropping_it():
    node = _controller()
    _jog(node, [0.5, 0, 0, 0, 0, 0], dt=0.1)
    held = node.offset.copy()
    node.last_control_cmd = time.monotonic() - 5.0
    node.last_tick = time.monotonic() - 0.1
    node._tick()
    # Held, not released: dropping it would move the arm by exactly the
    # amount the operator dialled in, with nobody asking for it.
    assert node.offset == pytest.approx(held)
    assert node.offset_pub.messages[-1].data == pytest.approx(held.tolist())


def test_stale_joint_feedback_holds_the_offset():
    node = _controller()
    _jog(node, [0.5, 0, 0, 0, 0, 0], dt=0.1)
    held = node.offset.copy()
    node.last_joint_state = time.monotonic() - 10.0
    node._on_control_cmd(Float64MultiArray(data=[0.5, 0, 0, 0, 0, 0]))
    node.last_tick = time.monotonic() - 0.1
    node._tick()
    assert node.offset == pytest.approx(held)
    assert any("stale" in msg for msg in node._logger.warnings)


# --------------------------------------------------------------- rebasing

def test_a_new_autonomous_goal_rebases_the_offset_to_zero():
    """MoveIt replans from the measured pose, which already contains the
    offset; keeping it would apply the operator's correction twice."""
    node = _controller()
    _jog(node, [0.5, 0, 0, 0, 0, 0], dt=0.1)
    assert node.offset[0] != pytest.approx(0.0)
    node._on_arm_goal_status(_goal_status(7))
    assert node.offset == pytest.approx(np.zeros(6))


def test_the_same_goal_reported_twice_rebases_only_once():
    node = _controller()
    node._on_arm_goal_status(_goal_status(7))
    _jog(node, [0.5, 0, 0, 0, 0, 0], dt=0.1)
    moved = node.offset.copy()
    node._on_arm_goal_status(_goal_status(7))
    assert node.offset == pytest.approx(moved)


def test_a_finished_goal_does_not_rebase():
    node = _controller()
    _jog(node, [0.5, 0, 0, 0, 0, 0], dt=0.1)
    moved = node.offset.copy()
    node._on_arm_goal_status(_goal_status(9, GoalStatus.STATUS_SUCCEEDED))
    assert node.offset == pytest.approx(moved)


# ------------------------------------------------------------------- FLOAT

def _arm_state(positions):
    message = JointTrajectoryControllerState()
    message.joint_names = [f"joint{i}" for i in range(1, 7)]
    point = JointTrajectoryPoint()
    point.positions = list(positions)
    message.reference = point
    return message


def test_float_offset_cancels_the_autonomous_reference():
    """Total command = reference + offset, so tracking the measured arm means
    the offset has to be exactly the servo's standing error."""
    node = _controller(mode=MODE_FLOAT)
    node._on_arm_state(_arm_state(INITIAL))
    hand_moved = list(INITIAL)
    hand_moved[0] += 0.10
    node.joint_positions["joint1"] = hand_moved[0]
    for _ in range(40):
        node.last_tick = time.monotonic() - 0.02
        node._tick()
    assert node.offset[0] == pytest.approx(0.10, abs=1e-3)


def test_float_does_not_chase_a_falling_joint():
    node = _controller(mode=MODE_FLOAT)
    node._on_arm_state(_arm_state(INITIAL))
    node.joint_positions["joint2"] = INITIAL[1] - 1.0   # a drop, not a push
    node.last_tick = time.monotonic() - 0.02
    node._tick()
    # A hand moves the arm slowly and a fall does not, so one tick may only
    # close float_follow_velocity * dt of that metre-scale gap -- the servo
    # catches the arm instead of following it to the bench. Generous on the
    # upper bound because the real dt is whatever the clock says it was.
    assert abs(node.offset[1]) <= 0.35 * 0.05
    assert abs(node.offset[1]) < 0.1


# ------------------------------------------------------------ coordinates

def test_cartesian_jog_goes_through_ik_into_the_offset():
    node = _controller(mode=MODE_CARTESIAN)
    _jog(node, [0.05, 0, 0, 0, 0, 0], dt=0.1)
    # _IdentityIK maps a linear x twist straight onto joint1.
    assert node.offset[0] == pytest.approx(0.005, abs=1e-6)


def test_cartesian_jog_is_inert_without_ik():
    node = _controller(mode=MODE_CARTESIAN, ik=False)
    _jog(node, [0.05, 0, 0, 0, 0, 0], dt=0.1)
    assert node.offset == pytest.approx(np.zeros(6))


def test_cylindrical_theta_sweeps_around_the_column():
    node = _controller(mode=MODE_CYLINDRICAL)
    _jog(node, [0.0, 0.2, 0, 0, 0, 0], dt=0.1)
    # Tangential motion is perpendicular to the radius, so it cannot be zero
    # on an arm parked off the column.
    assert float(np.linalg.norm(node.offset)) > 0.0


def test_semi_cylindrical_pitch_and_yaw_nudge_the_wrist_joints():
    node = _controller(mode=MODE_SEMI_CYLINDRICAL)
    _jog(node, [0, 0, 0, 0, 0.5, 0.25], dt=0.1)
    assert node.offset[4] == pytest.approx(0.05, abs=1e-6)
    assert node.offset[5] == pytest.approx(0.025, abs=1e-6)


# ------------------------------------------------------------ absolute poses

def test_ready_sends_a_trajectory_goal_and_clears_the_offset():
    node = _controller()
    _jog(node, [0.5, 0, 0, 0, 0, 0], dt=0.1)
    assert node.offset[0] != pytest.approx(0.0)
    node._on_operation_mode(String(data=MODE_READY))
    assert len(node.arm_client.goals) == 1
    goal = node.arm_client.goals[0]
    assert list(goal.trajectory.joint_names) == node.joint_names
    # Transition pose first, then READY itself.
    assert len(goal.trajectory.points) == 2
    assert list(goal.trajectory.points[-1].positions) == pytest.approx(
        node.ready_pose
    )
    assert node.offset == pytest.approx(np.zeros(6))


def test_rest_and_startup_target_the_captured_startup_pose():
    for mode in (MODE_REST, MODE_STARTUP):
        node = _controller()
        node._on_operation_mode(String(data=mode))
        goal = node.arm_client.goals[-1]
        assert list(goal.trajectory.points[-1].positions) == pytest.approx(
            node.startup_pose
        )


def test_rest_is_refused_until_a_startup_pose_exists():
    node = _controller()
    node.startup_pose = None
    node._on_operation_mode(String(data=MODE_REST))
    assert not node.arm_client.goals
    assert any("startup pose" in msg for msg in node._logger.warnings)


def test_a_pose_is_refused_when_the_arm_action_is_down():
    node = _controller()
    node.arm_client.ready = False
    node._on_operation_mode(String(data=MODE_READY))
    assert not node.arm_client.goals
    assert any("unavailable" in msg for msg in node._logger.warnings)


def test_autonomous_request_only_clears_the_offset():
    """Nothing to hand back any more -- both channels are always live."""
    node = _controller()
    _jog(node, [0.5, 0, 0, 0, 0, 0], dt=0.1)
    node._on_operation_mode(String(data=MODE_AUTONOMOUS))
    assert node.offset == pytest.approx(np.zeros(6))
    assert not node.arm_client.goals


def test_mode_selection_switches_the_jog_frame():
    node = _controller()
    node._on_operation_mode(String(data=MODE_CARTESIAN))
    assert node.motion_mode == MODE_CARTESIAN
    assert node.last_coordinate_mode == MODE_CARTESIAN


def test_coordinate_mode_is_refused_without_ik():
    node = _controller(ik=False)
    node._on_operation_mode(String(data=MODE_CARTESIAN))
    assert node.motion_mode == MODE_JOINT
    assert any("IK is unavailable" in msg for msg in node._logger.warnings)


# ----------------------------------------------------------------- targets

def _target(node, **payload):
    node._on_target_command(String(data=json.dumps(payload)))
    return json.loads(node.target_status_pub.messages[-1].data)


def test_absolute_joint_target_sends_one_trajectory_goal():
    node = _controller()
    goal_positions = [0.2, -0.3, 0.2, 0.1, 0.4, 0.0]
    status = _target(
        node, action="move", mode=MODE_JOINT, values=goal_positions,
        reach_duration=5.0, request_id="abc",
    )
    assert status["state"] == "running"
    assert len(node.arm_client.goals) == 1
    points = node.arm_client.goals[0].trajectory.points
    assert len(points) == 1                      # straight there, no detour
    assert list(points[0].positions) == pytest.approx(goal_positions)


def test_target_clears_the_operator_offset():
    node = _controller()
    _jog(node, [0.5, 0, 0, 0, 0, 0], dt=0.1)
    _target(
        node, action="move", mode=MODE_JOINT,
        values=[0.2, -0.3, 0.2, 0.1, 0.4, 0.0], reach_duration=5.0,
    )
    assert node.offset == pytest.approx(np.zeros(6))


def test_target_outside_joint_limits_is_rejected():
    node = _controller()
    status = _target(
        node, action="move", mode=MODE_JOINT,
        values=[99.0, 0, 0, 0, 0, 0], reach_duration=5.0,
    )
    assert status["state"] == "rejected"
    assert not node.arm_client.goals


def test_target_too_fast_for_the_joint_ceiling_is_rejected():
    node = _controller()
    status = _target(
        node, action="move", mode=MODE_JOINT,
        values=[1.4, -0.4, 0.3, 0.2, 0.5, -0.1], reach_duration=0.5,
    )
    assert status["state"] == "rejected"
    assert "velocity" in status["message"]


def test_target_stop_clears_the_offset():
    node = _controller()
    _jog(node, [0.5, 0, 0, 0, 0, 0], dt=0.1)
    status = _target(node, action="stop")
    assert status["state"] == "stopped"
    assert node.offset == pytest.approx(np.zeros(6))


def test_malformed_target_is_rejected_not_crashed():
    node = _controller()
    node._on_target_command(String(data="{not json"))
    status = _target(node, action="move", mode="NONSENSE", values=[])
    assert status["state"] == "rejected"
