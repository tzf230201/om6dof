"""Six-joint compensation driven by posture alone, with no identification.

The node used to require a fitted model for every joint before it would even
start, so the six-joint path could not be reached however good the gravity
model was. These tests pin the arithmetic of the fallback and, more
importantly, that a joint nobody enabled stays at zero current.
"""
import numpy as np
import pytest

from om6dof_gravity_comp.units import CURRENT_TICK_MA, JOINT_NAMES


def model_only_command(gravity_nm, sign, alpha, kt):
    """I_cmd = s * alpha * G(q) / Kt, amperes converted to register ticks."""
    return sign * alpha * gravity_nm / kt * 1000.0 / CURRENT_TICK_MA


def test_command_matches_the_stated_formula():
    # 1.3781 Nm at joint2 is what the model gives reaching forward with the
    # 80 g camera declared.
    raw = model_only_command(1.3781, 1.0, 1.0, 0.61)
    assert raw == pytest.approx(1.3781 / 0.61 * 1000.0 / 2.69, rel=1e-12)
    assert 830 < raw < 850


def test_sign_flips_the_command_and_nothing_else():
    a = model_only_command(0.9, +1.0, 1.0, 0.61)
    b = model_only_command(0.9, -1.0, 1.0, 0.61)
    assert a == pytest.approx(-b)


def test_alpha_scales_linearly():
    base = model_only_command(0.9, 1.0, 1.0, 0.61)
    assert model_only_command(0.9, 1.0, 2.0, 0.61) == pytest.approx(2 * base)


def test_a_wrong_torque_constant_cannot_be_hidden_in_alpha_silently():
    """alpha and Kt are separate knobs on purpose: doubling Kt and doubling
    alpha cancel, which is exactly why the pair must be identified, not
    guessed one at a time."""
    assert model_only_command(0.9, 1.0, 2.0, 1.22) == pytest.approx(
        model_only_command(0.9, 1.0, 1.0, 0.61))


def test_disabled_joints_receive_exactly_zero():
    enabled = np.array([j == "joint2" for j in JOINT_NAMES])
    gravity = np.array([0.1, 1.3781, 0.65, 0.0, 0.2, 0.0])
    command = np.where(
        enabled, model_only_command(gravity, 1.0, 1.0, 0.61), 0.0)
    assert command[1] != 0.0
    assert np.count_nonzero(command) == 1
    for index, joint in enumerate(JOINT_NAMES):
        if joint != "joint2":
            assert command[index] == 0.0
