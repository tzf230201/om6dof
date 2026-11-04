import pytest
import math

from om6dof_controllers.cartesian_force_bridge import directional_wrench
from om6dof_controllers.cartesian_force_bridge import custom_directional_wrench, CartesianForceBridge
from om6dof_controllers.control_math import cartesian_rpy_to_tip_rotation, MODE_FORCE
from types import SimpleNamespace
import numpy as np
import time


def test_direction_only_selects_configured_wrench_magnitude():
    assert directional_wrench([.01, -.7, 0, 3, -2, 0], [2, 3, 4, .1, .2, .3]) == [2, -3, 0, .1, -.2, 0]


def test_wrench_limits_reject_negative_value():
    with pytest.raises(ValueError, match='nonnegative'):
        directional_wrench([0] * 6, [1, 1, 1, -1, 1, 1])


def test_directional_wrench_never_uses_deflection_as_force_magnitude():
    assert directional_wrench([0.0001, -999.0, 0, 0, 0, 0], [2, 3, 4, .1, .2, .3]) == [2, -3, 0, 0, 0, 0]


@pytest.mark.parametrize('direction', [[1,0,0,0,0,0], [1,-1,0,0,0,0], [1,1,1,0,0,0]])
def test_force_profile_requests_one_newton_resultant_including_diagonals(direction):
    wrench = directional_wrench(direction, [1,1,1,.2,.2,.2], 1.0)
    assert math.sqrt(sum(v*v for v in wrench[:3])) == pytest.approx(1.0)


def test_force_profile_preserves_axis_caps_and_release():
    assert directional_wrench([1,0,0,0,0,0], [.5,2,2,0,0,0], 2) == [.5,0,0,0,0,0]
    assert directional_wrench([0]*6, [2,2,2,.2,.2,.2], 2) == [0]*6


@pytest.mark.parametrize('direction,expected', [
    ([1,0,0,0,0,0], [0,5,0]), ([0,1,0,0,0,0], [-5,0,0]),
    ([0,0,1,0,0,0], [0,0,5]), ([0,-1,0,0,0,0], [5,0,0]),
])
def test_custom_force_rotates_radial_and_tangential_axes_with_robot(direction, expected):
    result = custom_directional_wrench(direction, [5,5,5,.2,.2,.2], 5.,
        [.012,.2,.1], cartesian_rpy_to_tip_rotation(0,0,0))
    assert result[:3] == pytest.approx(expected)


def test_custom_force_preserves_five_newtons_for_diagonal_and_uses_gripper_roll_pitch():
    rotation = cartesian_rpy_to_tip_rotation(0,0,math.pi/2)
    result = custom_directional_wrench([1,1,1,1,0,1], [5,5,5,.2,.2,.2], 5.,
                                     [.212,.2,.1], rotation)
    assert np.linalg.norm(result[:3]) == pytest.approx(5.)
    assert result[3:] == pytest.approx([0,.2,0], abs=1e-12)
    result = custom_directional_wrench([0,0,0,0,1,0], [5,5,5,.2,.2,.2], 5.,
                                     [.212,.2,.1], rotation)
    assert result[3:] == pytest.approx([-.2,0,0], abs=1e-12)


def test_custom_force_stops_when_joint_feedback_expires():
    bridge = object.__new__(CartesianForceBridge)
    bridge.mode = MODE_FORCE
    bridge.input_frame = 'custom'
    bridge.timeout = .2
    bridge.direction_at = time.monotonic()
    bridge.joints = [0.] * 6
    bridge.joints_at = time.monotonic() - 1.
    bridge.sent_stop = False
    sent = []
    bridge.publisher = SimpleNamespace(publish=lambda msg: sent.append(msg.data))
    bridge._tick()
    assert list(sent[0]) == [0.] * 6
    bridge._tick()
    assert len(sent) == 1


def test_custom_force_rejects_undefined_radius():
    with pytest.raises(ValueError, match='undefined'):
        custom_directional_wrench([1,0,0,0,0,0], [5,5,5,.2,.2,.2], 5.,
                                 [.012,0.,.1], cartesian_rpy_to_tip_rotation(0,0,0))
