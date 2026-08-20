import math
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest
from controller_manager_msgs.srv import SwitchController
from std_msgs.msg import Float64MultiArray, String

from om6dof_controller.control_math import (
    MODE_FLOAT,
    MODE_SEMI_CYLINDRICAL,
    SEMI_PITCH_LIMIT,
    rotation_error,
    rotation_from_zyx,
    rotation_to_zyx,
    semi_cylindrical_rotation,
    wrap_angle,
    MODE_AUTONOMOUS,
    MODE_CARTESIAN,
    MODE_CYLINDRICAL,
    MODE_JOINT,
    MODE_READY,
    MODE_STARTUP,
)
from om6dof_controller.controller_node import (
    DEFAULT_JOINT_LOWER,
    DEFAULT_JOINT_UPPER,
    DEFAULT_READY_JOINT_POSITIONS,
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
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Future:
    def __init__(self):
        self.callback = None
        self.response = None

    def add_done_callback(self, callback):
        self.callback = callback

    def result(self):
        return self.response

    def finish(self, response):
        self.response = response
        self.callback(self)


class _SwitchClient:
    def __init__(self, ready=True):
        self.ready = ready
        self.requests = []
        self.futures = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        future = _Future()
        self.futures.append(future)
        return future


class _UnavailableListClient:
    def service_is_ready(self):
        return False


class _IdentityIK:
    def fk_pose(self, q):
        values = np.asarray(q, dtype=float)
        return values[:3].copy(), np.eye(3)

    def velocity_ik_priority(self, _q, linear, angular):
        return np.concatenate([linear, angular])

    def ee_to_base_angular(self, _q, angular):
        return np.asarray(angular, dtype=float)

    def self_collides(self, _q, _radius):
        return False

    def manipulability(self, _q):
        return 1.0


def _controller(remote_enabled=False):
    node = object.__new__(OM6DOFController)
    node.lock = threading.RLock()
    node.joint_names = [f"joint{index}" for index in range(1, 7)]
    node.arm_controller = "arm_controller"
    node.remote_controller = "forward_position_controller"
    node.motion_mode = MODE_JOINT if remote_enabled else MODE_AUTONOMOUS
    node.remote_enabled = remote_enabled
    node.arm_controller_active = not remote_enabled
    node.remote_controller_active = remote_enabled
    node.switch_in_progress = False
    node.switch_target = None
    node.pending_manual_mode = MODE_JOINT
    node.remote_enabled_on_start = False
    node.startup_switch_attempted = False
    node.controller_list_future = None
    node.next_controller_poll = 0.0

    initial = [0.1, -1.9, 1.8, 0.2, 2.0, -0.1]
    node.joint_positions = dict(zip(node.joint_names, initial))
    node.last_joint_state = time.monotonic()
    node.startup_pose = list(initial)
    node.command_positions = list(initial) if remote_enabled else None
    node.control_velocity = np.zeros(6)
    node.last_control_cmd = 0.0
    node.last_tick = time.monotonic() - 0.02

    node.pose_target = None
    node.pose_operation = None
    node.pose_target_until = 0.0
    node.post_pose_mode = MODE_JOINT
    node.ready_pending_on_enable = False
    node.ready_pending_mode = MODE_JOINT

    node.joint_state_timeout = 1.0
    node.control_cmd_timeout = 0.3
    node.max_joint_velocity = 1.2
    node.joint_lower = [value + 0.02 for value in DEFAULT_JOINT_LOWER]
    node.joint_upper = [value - 0.02 for value in DEFAULT_JOINT_UPPER]
    node.ready_pose = list(DEFAULT_READY_JOINT_POSITIONS)
    node.pose_target_velocity = 0.5
    node.pose_target_tolerance = 0.01
    node.pose_target_timeout = 20.0

    node.max_cartesian_linear_velocity = 0.1
    node.max_cartesian_angular_velocity = 1.0
    node.max_cylindrical_theta_velocity = 0.5
    node.cylindrical_origin_xy = np.array([0.012, 0.0])
    node.cylindrical_min_radius = 0.03
    node.ik_position_gain = 4.0
    node.ik_rotation_gain = 3.0
    node.ik_tool_frame_rotation = True
    node.ik_max_target_lead = 0.04
    node.ik_max_joint_following_error = 0.30
    node.ik_manipulability_warning_threshold = 1.0e-6
    node.ik_self_collision = False
    node.ik_collision_radius = 0.025
    node.ik_collision_blocked = False
    node.ik = _IdentityIK()
    node.ik_target_pos = None
    node.ik_target_rotation = None
    node.cylindrical_theta_hint = None

    node.command_pub = _Publisher()
    node.operation_state_pub = _Publisher()
    node.remote_state_pub = _Publisher()
    node.switch_client = _SwitchClient()
    node.list_client = _UnavailableListClient()
    node._logger = _Logger()
    node.get_logger = lambda: node._logger
    node.get_parameter = lambda name: SimpleNamespace(
        value={"switch_timeout_seconds": 2.0}[name]
    )
    return node


def test_joint_mode_switches_only_after_success_and_schedules_ready():
    node = _controller(remote_enabled=False)

    node._on_operation_mode(String(data="JOINT"))

    assert node.remote_enabled is False
    request = node.switch_client.requests[-1]
    assert request.activate_controllers == ["forward_position_controller"]
    assert request.deactivate_controllers == ["arm_controller"]
    assert request.strictness == SwitchController.Request.STRICT
    node.switch_client.futures[-1].finish(SimpleNamespace(ok=True))
    assert node.remote_enabled is True
    assert node.pose_operation == MODE_READY
    assert node.pose_target == pytest.approx(DEFAULT_READY_JOINT_POSITIONS)
    assert node.command_pub.messages[-1].data == pytest.approx(node.startup_pose)


def test_failed_switch_never_arms_ready():
    node = _controller(remote_enabled=False)
    node._on_operation_mode(String(data="JOINT"))
    node.switch_client.futures[-1].finish(SimpleNamespace(ok=False))
    assert node.remote_enabled is False
    assert node.pose_target is None
    assert node._logger.errors


def test_coordinate_mode_requires_manual_ownership():
    node = _controller(remote_enabled=False)
    node._on_operation_mode(String(data="CARTESIAN"))
    assert node.motion_mode == MODE_AUTONOMOUS
    assert not node.switch_client.requests
    assert any("request JOINT first" in msg for msg in node._logger.warnings)


def test_mode_change_clears_old_command():
    node = _controller(remote_enabled=True)
    node.control_velocity = np.ones(6)
    node.last_control_cmd = time.monotonic()
    node._on_operation_mode(String(data="CARTESIAN"))
    assert node.motion_mode == MODE_CARTESIAN
    assert node.last_control_cmd == 0.0
    assert node.control_velocity == pytest.approx(np.zeros(6))


def test_joint_velocity_stream_integrates_then_watchdog_holds_feedback():
    node = _controller(remote_enabled=True)
    before = list(node.command_positions)
    node._on_control_cmd(Float64MultiArray(data=[0.5, 0, 0, 0, 0, 0]))
    node.last_tick = time.monotonic() - 0.02
    node._tick()
    moved = node.command_pub.messages[-1].data
    assert moved[0] > before[0]

    node.last_control_cmd = time.monotonic() - 1.0
    node.last_tick = time.monotonic() - 0.02
    node._tick()
    assert node.command_pub.messages[-1].data == pytest.approx(node.startup_pose)


def test_cartesian_velocity_resolves_to_joint_positions():
    node = _controller(remote_enabled=True)
    node.motion_mode = MODE_CARTESIAN
    node._seed_ik_anchor_locked(node.command_positions)
    before = list(node.command_positions)
    node._on_control_cmd(Float64MultiArray(data=[0.02, 0, 0, 0, 0, 0]))
    node.last_tick = time.monotonic() - 0.02
    node._tick()
    assert node.command_pub.messages[-1].data[0] > before[0]


def test_cylindrical_velocity_resolves_to_joint_positions():
    node = _controller(remote_enabled=True)
    node.motion_mode = MODE_CYLINDRICAL
    node._seed_ik_anchor_locked(node.command_positions)
    before = list(node.command_positions)
    node._on_control_cmd(Float64MultiArray(data=[0.02, 0, 0, 0, 0, 0]))
    node.last_tick = time.monotonic() - 0.02
    node._tick()
    assert node.command_pub.messages[-1].data[0] > before[0]


def test_ready_and_startup_are_transient_joint_pose_operations():
    node = _controller(remote_enabled=True)
    node._on_operation_mode(String(data="READY"))
    assert node.pose_operation == MODE_READY
    assert node.motion_mode == MODE_JOINT
    node._on_operation_mode(String(data="STARTUP"))
    assert node.pose_target == pytest.approx(node.startup_pose)
    assert node.motion_mode == MODE_JOINT


def test_ready_hands_back_a_coordinate_mode_not_joint():
    """READY must leave the arm drivable by coordinate-space interfaces.

    Handing back JOINT locked the gamepad out after every READY, because
    its mapping is Cartesian and it refuses JOINT. TOGGLE_REST_READY
    already resumed the remembered mode; plain READY has to match it.
    """
    node = _controller(remote_enabled=True)
    node.last_coordinate_mode = MODE_CYLINDRICAL
    node._on_operation_mode(String(data="READY"))
    assert node.pose_operation == MODE_READY
    assert node.post_pose_mode == MODE_CYLINDRICAL

    # An unset or non-coordinate memory falls back to CARTESIAN rather
    # than passing JOINT through and reintroducing the lockout.
    node = _controller(remote_enabled=True)
    node.last_coordinate_mode = MODE_JOINT
    node._on_operation_mode(String(data="READY"))
    assert node.post_pose_mode == MODE_CARTESIAN


def test_rest_ready_toggle_selects_the_opposite_nearest_pose():
    node = _controller(remote_enabled=True)
    node._set_motion_mode_locked(MODE_CYLINDRICAL, "test")
    node.joint_positions = dict(zip(node.joint_names, node.startup_pose))
    node._on_operation_mode(String(data="TOGGLE_REST_READY"))
    assert node.pose_operation == MODE_READY
    assert node.pose_target == pytest.approx(node.ready_pose)
    assert node.post_pose_mode == MODE_CYLINDRICAL

    node.joint_positions = dict(zip(node.joint_names, node.ready_pose))
    node._on_operation_mode(String(data="TOGGLE_REST_READY"))
    assert node.pose_operation == MODE_STARTUP
    assert node.pose_target == pytest.approx(node.startup_pose)
    assert node.post_pose_mode == MODE_JOINT


def test_joint_commands_do_not_interrupt_guarded_pose_profile():
    node = _controller(remote_enabled=True)
    node._on_operation_mode(String(data="READY"))

    node._on_control_cmd(Float64MultiArray(data=[0.0] * 6))
    assert node.pose_operation == MODE_READY
    assert node.last_control_cmd == 0.0

    node._on_control_cmd(Float64MultiArray(data=[0.2, 0, 0, 0, 0, 0]))
    assert node.pose_target is not None
    assert node.pose_operation == MODE_READY
    assert node.motion_mode == MODE_JOINT
    assert node.control_velocity == pytest.approx([0.0] * 6)
    assert node.last_control_cmd == 0.0
    assert any("ignores control_cmd" in message for message in node._logger.warnings)


def test_autonomous_restores_trajectory_controller():
    node = _controller(remote_enabled=True)
    node._on_operation_mode(String(data="AUTONOMOUS"))
    request = node.switch_client.requests[-1]
    assert request.activate_controllers == ["arm_controller"]
    assert request.deactivate_controllers == ["forward_position_controller"]
    node.switch_client.futures[-1].finish(SimpleNamespace(ok=True))
    assert node.remote_enabled is False
    assert node.motion_mode == MODE_AUTONOMOUS


@pytest.mark.parametrize(
    "values",
    ([0.0] * 5, [0.0] * 7, [0, 0, np.nan, 0, 0, 0]),
)
def test_invalid_control_command_is_rejected(values):
    node = _controller(remote_enabled=True)
    node._on_control_cmd(Float64MultiArray(data=values))
    assert node.last_control_cmd == 0.0
    assert node._logger.warnings


class _FollowingIK(_IdentityIK):
    """A wrist that reaches whatever was last asked of it.

    Needed to exercise limits that only bite when the arm is keeping up --
    with a lagging wrist the anti-windup clamp stops the angles first.
    """

    def __init__(self, node):
        self.node = node

    def fk_pose(self, q):
        values = np.asarray(q, dtype=float)
        rotation = self.node.ik_target_rotation
        if rotation is None:
            rotation = np.eye(3)
        return values[:3].copy(), np.asarray(rotation, dtype=float).copy()

    def ee_to_base_angular(self, q, angular):
        # The real IK rotates a tool-frame rate into the base frame.
        # _IdentityIK returns it unchanged, which cannot tell a tool-frame
        # command apart from a base-frame one -- exactly the distinction the
        # stick-axis test exists to check.
        rotation = self.node.ik_target_rotation
        if rotation is None:
            rotation = np.eye(3)
        return np.asarray(rotation, dtype=float) @ np.asarray(angular, dtype=float)


class _TiltedIK(_IdentityIK):
    """An IK whose tool is not axis-aligned, so seeding has to do real work."""

    ROTATION = rotation_from_zyx(0.35, -0.42, 1.1)

    def fk_pose(self, q):
        values = np.asarray(q, dtype=float)
        return values[:3].copy(), self.ROTATION.copy()


def test_entering_semi_cylindrical_does_not_move_the_wrist():
    """Seeding must reproduce the pose the arm is already holding.

    The mode drives absolute wrist angles. If they started at zero instead
    of at the current orientation, selecting the mode would snap the wrist
    on a real arm before the operator touched anything.
    """
    node = _controller(remote_enabled=True)
    node.ik = _TiltedIK()
    assert node._seed_ik_anchor_locked(node.command_positions)

    rebuilt = semi_cylindrical_rotation(
        node.cylindrical_theta_hint,
        node.semi_roll,
        node.semi_pitch,
        node.semi_yaw_offset,
    )
    assert np.allclose(rebuilt, _TiltedIK.ROTATION, atol=1e-9)


def test_semi_cylindrical_yaw_offset_is_measured_from_theta():
    """Yaw is stored relative to theta, which is what makes it follow."""
    node = _controller(remote_enabled=True)
    node.ik = _TiltedIK()
    node._seed_ik_anchor_locked(node.command_positions)

    _, _, absolute_yaw = rotation_to_zyx(_TiltedIK.ROTATION)
    expected = math.remainder(
        absolute_yaw - node.cylindrical_theta_hint, 2.0 * math.pi
    )
    assert math.isclose(node.semi_yaw_offset, expected, abs_tol=1e-9)


def test_semi_cylindrical_pitch_is_clamped_when_seeding_near_vertical():
    """Entering the mode from a near-vertical wrist must not seed a singularity.

    This is the only way pitch can reach the limit now. The stick cannot get
    there: pitch and yaw became joint 5 / joint 6 offsets, and roll is a
    rotation about the body X axis, which by construction changes only the
    roll term of a Z-Y-X decomposition and leaves pitch untouched -- measured,
    not assumed.
    """
    class _NearVerticalIK(_IdentityIK):
        ROTATION = rotation_from_zyx(0.2, math.pi / 2 - 0.001, 0.4)

        def fk_pose(self, q):
            values = np.asarray(q, dtype=float)
            return values[:3].copy(), self.ROTATION.copy()

    node = _controller(remote_enabled=True)
    node.ik = _NearVerticalIK()
    node._set_motion_mode_locked(MODE_SEMI_CYLINDRICAL, "test")
    assert node._seed_ik_anchor_locked(node.command_positions)

    assert math.isfinite(node.semi_pitch)
    assert math.isfinite(node.semi_roll)
    assert math.isfinite(node.semi_yaw_offset)
    assert abs(node.semi_pitch) <= SEMI_PITCH_LIMIT + 1e-9, (
        f"seeded pitch {node.semi_pitch:+.4f} sits past the limit"
    )


def test_semi_cylindrical_roll_does_not_disturb_pitch():
    """Rolling must leave the wrist's tilt alone, so the tool spins in place."""
    node = _semi_node()
    pitch_before = node.semi_pitch
    roll_before = node.semi_roll
    _drive(node, 3, 5.0, steps=300)
    assert not node._logger.errors, node._logger.errors
    # Non-vacuous: roll really turned, so "pitch held" is a statement about
    # the decomposition and not about nothing happening.
    # 0.2 rad is roughly where the fixture's joint limits stop the roll; well
    # clear of zero, which is all this guard needs to rule out.
    assert abs(wrap_angle(node.semi_roll - roll_before)) > 0.2
    assert math.isclose(node.semi_pitch, pitch_before, abs_tol=1e-6)


def test_semi_cylindrical_sweep_carries_yaw_and_leaves_the_tilt_alone():
    """The defining behaviour, end to end through the control step.

    Commanding only theta must rotate the wrist target's heading by the same
    amount and leave pitch and roll exactly where the operator put them.
    """
    node = _controller(remote_enabled=True)
    node.ik = _TiltedIK()
    node._set_motion_mode_locked(MODE_SEMI_CYLINDRICAL, "test")
    node._seed_ik_anchor_locked(node.command_positions)

    roll_before, pitch_before, yaw_before = rotation_to_zyx(
        node.ik_target_rotation
    )
    theta_before = node.cylindrical_theta_hint

    # theta only: index 1 of the coordinate velocity is the angular sweep.
    node.control_velocity = np.array([0.0, 0.4, 0.0, 0.0, 0.0, 0.0])
    node.last_control_cmd = time.monotonic()
    for _ in range(25):
        node._coordinate_step_locked(
            node.command_positions, node.control_velocity, 0.02
        )

    assert not node._logger.errors, node._logger.errors
    roll_after, pitch_after, yaw_after = rotation_to_zyx(
        node.ik_target_rotation
    )
    swept = node.cylindrical_theta_hint - theta_before
    assert abs(swept) > 1e-3, "theta did not actually move"
    assert math.isclose(roll_after, roll_before, abs_tol=1e-6)
    assert math.isclose(pitch_after, pitch_before, abs_tol=1e-6)
    assert math.isclose(
        math.remainder(yaw_after - yaw_before, 2.0 * math.pi),
        math.remainder(swept, 2.0 * math.pi),
        abs_tol=1e-6,
    )


def test_semi_cylindrical_angles_cannot_wind_up_past_the_arm():
    """The stored angles must not run away from what the arm is holding.

    They are rebuilt into the target every cycle, so the orientation-error
    clamp that bounds the incremental modes was being discarded here. The
    angles then integrated freely, pitch reached its own limit while the
    wrist had barely moved, and the stick went dead -- which is what "roll
    and pitch keep getting stuck" actually was.
    """
    node = _controller(remote_enabled=True)
    node.ik = _TiltedIK()  # orientation never changes: a wrist that cannot keep up
    node._set_motion_mode_locked(MODE_SEMI_CYLINDRICAL, "test")
    node._seed_ik_anchor_locked(node.command_positions)

    node.control_velocity = np.array([0.0, 0.0, 0.0, 0.0, 1.0, 0.0])
    node.last_control_cmd = time.monotonic()
    for _ in range(120):
        node._coordinate_step_locked(
            node.command_positions, node.control_velocity, 0.02
        )

    achieved_pitch = rotation_to_zyx(_TiltedIK.ROTATION)[1]
    lead = abs(node.semi_pitch - achieved_pitch)
    # The same 0.35 rad ceiling the incremental modes get, plus one step of
    # slack for the integration that happens before the clamp is applied.
    assert lead <= 0.35 + 0.05, f"pitch led the arm by {lead:.3f} rad"
    assert abs(node.semi_pitch) < SEMI_PITCH_LIMIT - 0.1, (
        "pitch reached its travel limit while the wrist never moved"
    )


def test_semi_cylindrical_stick_turns_the_tool_like_cartesian_does():
    """The stick must rotate about the tool's axis, not the world vertical.

    Absolute angles describe where the wrist is *held*; they should not also
    dictate what the stick turns around. Interpreting the rates as base-frame
    Euler rates put the yaw axis 60 degrees away from the Cartesian one on a
    typical downward-pointing wrist, so pressing yaw swept the tool through a
    cone instead of spinning it in place.
    """
    node = _controller(remote_enabled=True)
    node.ik = _TiltedIK()
    node._set_motion_mode_locked(MODE_SEMI_CYLINDRICAL, "test")
    node._seed_ik_anchor_locked(node.command_positions)
    node.ik = _FollowingIK(node)

    before = node.ik_target_rotation.copy()
    # Roll is the only channel left steering the wrist: pitch and yaw became
    # joint 5 / joint 6 offsets.
    node.control_velocity = np.array([0.0, 0.0, 0.0, 0.35, 0.0, 0.0])
    node.last_control_cmd = time.monotonic()
    node._coordinate_step_locked(
        node.command_positions, node.control_velocity, 0.02
    )
    produced = rotation_error(node.ik_target_rotation, before) / 0.02
    expected = before @ np.array([0.35, 0.0, 0.0])

    produced_axis = produced / np.linalg.norm(produced)
    expected_axis = expected / np.linalg.norm(expected)
    assert np.allclose(produced_axis, expected_axis, atol=1e-6), (
        f"axis {produced_axis} is not the tool axis {expected_axis}"
    )


def _semi_node():
    node = _controller(remote_enabled=True)
    node.ik = _TiltedIK()
    node._set_motion_mode_locked(MODE_SEMI_CYLINDRICAL, "test")
    node._seed_ik_anchor_locked(node.command_positions)
    node.ik = _FollowingIK(node)
    node.last_control_cmd = time.monotonic()
    return node


def _drive(node, index, rate, steps=30, dt=0.02):
    before = list(node.command_positions)
    velocity = np.zeros(6)
    velocity[index] = rate
    node.control_velocity = velocity
    for _ in range(steps):
        node.command_positions = node._coordinate_step_locked(
            node.command_positions, node.control_velocity, dt
        )
    return np.asarray(node.command_positions) - np.asarray(before)


def test_semi_cylindrical_pitch_stick_offsets_joint5_only():
    """Y/A nudge joint 5 directly instead of steering the wrist through IK."""
    node = _semi_node()
    delta = _drive(node, 4, 0.5)
    assert not node._logger.errors, node._logger.errors
    assert abs(delta[4]) > 0.05, f"joint5 barely moved: {delta[4]:+.4f}"
    others = [abs(delta[i]) for i in range(6) if i != 4]
    assert max(others) < 1e-6, f"other joints moved: {np.round(delta, 5)}"


def test_semi_cylindrical_yaw_stick_offsets_joint6_only():
    """LT/RT nudge joint 6 directly."""
    node = _semi_node()
    delta = _drive(node, 5, 0.5)
    assert not node._logger.errors, node._logger.errors
    assert abs(delta[5]) > 0.05, f"joint6 barely moved: {delta[5]:+.4f}"
    others = [abs(delta[i]) for i in range(6) if i != 5]
    assert max(others) < 1e-6, f"other joints moved: {np.round(delta, 5)}"


def test_semi_cylindrical_joint_nudges_are_not_undone_by_ik():
    """The offset has to stick once the stick is released.

    Without re-seeding the IK anchor, the next cycles would read the nudge as
    an orientation error and drive it straight back out.
    """
    node = _semi_node()
    _drive(node, 4, 0.5)
    held = node.command_positions[4]
    _drive(node, 4, 0.0, steps=50)  # let go and let the loop settle
    assert abs(node.command_positions[4] - held) < 1e-6, (
        "IK pulled joint 5 back to where it was"
    )


def test_semi_cylindrical_joint_nudges_respect_joint_limits():
    node = _semi_node()
    _drive(node, 4, 5.0, steps=400)
    _drive(node, 5, 5.0, steps=400)
    assert node.command_positions[4] <= node.joint_upper[4] + 1e-9
    assert node.command_positions[5] <= node.joint_upper[5] + 1e-9


def _float_node(follow=0.35):
    node = _controller(remote_enabled=True)
    node._float_follow_velocity = lambda: follow
    node._on_operation_mode(String(data="FLOAT"))
    return node


def test_float_requires_remote_ownership():
    """Handing the arm to a person must not bypass taking control of it."""
    node = _controller(remote_enabled=False)
    node._float_follow_velocity = lambda: 0.35
    node._on_operation_mode(String(data="FLOAT"))
    assert node.motion_mode != MODE_FLOAT


def test_float_is_entered_by_name_and_by_alias():
    assert _float_node().motion_mode == MODE_FLOAT
    node = _controller(remote_enabled=True)
    node._float_follow_velocity = lambda: 0.35
    node._on_operation_mode(String(data="teach"))
    assert node.motion_mode == MODE_FLOAT


def test_float_command_follows_a_hand_moving_the_arm():
    """Pushed slowly, the command tracks the arm so the servo stops resisting."""
    node = _float_node()
    start = list(node.command_positions)
    # A hand nudges joint 2 by 2 degrees; feedback reports the new place.
    moved = list(start)
    moved[1] += math.radians(2.0)
    node.joint_positions = dict(zip(node.joint_names, moved))
    node.last_joint_state = time.monotonic()

    for _ in range(60):
        node.last_tick = time.monotonic() - 0.02
        node._tick()

    assert node.command_positions[1] == pytest.approx(moved[1], abs=1e-4)


def test_float_does_not_chase_a_falling_joint():
    """A fall outruns the follow rate, so the command lags and the servo holds.

    Chasing it would mean the goal descends with the arm and nothing ever
    catches it.
    """
    node = _float_node()
    start = list(node.command_positions)
    dropped = list(start)
    dropped[1] += 1.2  # far more than a hand moves in one tick
    node.joint_positions = dict(zip(node.joint_names, dropped))
    node.last_joint_state = time.monotonic()

    elapsed = 0.02
    node.last_tick = time.monotonic() - elapsed
    node._tick()

    moved = abs(node.command_positions[1] - start[1])
    # dt is measured inside the tick, so allow for it being a shade over the
    # sleep; the point is that 1.2 rad of fall produced millimetres of chase.
    assert moved <= 0.35 * elapsed * 1.5, (
        f"command chased {moved:.4f} rad in one tick"
    )
    assert moved > 0.0
    assert moved < 0.05, "the command tracked most of the fall"


def test_float_follow_rate_takes_effect_without_a_restart():
    """The rate must be tunable live, or the advice to tune it is useless.

    It was cached at startup, so `ros2 param set` changed the parameter and
    nothing else -- the arm kept the value it booted with.
    """
    steps = {}
    for label, follow in (("slow", 0.2), ("fast", 1.2)):
        node = _float_node(follow=follow)
        start = list(node.command_positions)
        moved = list(start)
        moved[1] += 1.0
        node.joint_positions = dict(zip(node.joint_names, moved))
        node.last_joint_state = time.monotonic()
        node.last_tick = time.monotonic() - 0.02
        node._tick()
        steps[label] = abs(node.command_positions[1] - start[1])
    assert steps["fast"] > steps["slow"] * 3, steps
