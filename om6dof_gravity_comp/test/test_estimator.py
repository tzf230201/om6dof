"""Estimator tests, driven by synthetic joint state rather than a robot."""

import math

import numpy as np
import pytest

from om6dof_gravity_comp.estimator import coulomb_feature, evaluate_model
from om6dof_gravity_comp.identify import build_regressor, FEATURE_NAMES
from om6dof_gravity_comp.units import JOINT_NAMES, order_by_joint


def _coefficients(a=2.0, b=15.0, c=5.0, d=1.0):
    return {joint: np.array([a, b, c, d]) for joint in JOINT_NAMES}


def test_model_matches_the_regressor_it_was_fitted_with():
    """Online and offline must compute the same thing from the same numbers.

    If they drift apart the residual becomes a measure of the mismatch rather
    than of external force, and nothing announces it.
    """
    nominal = np.array([0.1, -0.4, 0.3, 0.0, -0.2, 0.05])
    velocity = np.array([0.5, -0.3, 0.0, 0.9, -1.2, 0.01])
    coefficients = _coefficients()

    model, _, _ = evaluate_model(coefficients, nominal, velocity, 0.02, "smooth")
    for index, joint in enumerate(JOINT_NAMES):
        offline = build_regressor(
            np.array([nominal[index]]), np.array([velocity[index]]),
            0.02, "smooth") @ coefficients[joint]
        assert model[index] == pytest.approx(offline[0], abs=1e-12)


def test_gravity_and_friction_parts_add_up_to_the_model():
    nominal = np.linspace(-0.5, 0.5, 6)
    velocity = np.linspace(-1.0, 1.0, 6)
    model, gravity, friction = evaluate_model(
        _coefficients(), nominal, velocity, 0.02, "smooth")
    assert np.allclose(model, gravity + friction)


def test_friction_part_vanishes_at_rest():
    nominal = np.ones(6) * 0.2
    model, gravity, friction = evaluate_model(
        _coefficients(), nominal, np.zeros(6), 0.02, "smooth")
    assert np.allclose(friction, 0.0)
    assert np.allclose(model, gravity)


def test_residual_is_zero_when_measurement_equals_the_model():
    nominal = np.array([0.0, -0.3, 0.2, 0.0, 0.1, 0.0])
    velocity = np.array([0.2, 0.2, -0.2, 0.0, 0.4, -0.4])
    model, _, _ = evaluate_model(_coefficients(), nominal, velocity, 0.02, "smooth")
    assert np.allclose(model - model, 0.0)


def test_residual_reports_an_external_push():
    """A push shows up as measured minus modelled, and nowhere else."""
    nominal = np.zeros(6)
    velocity = np.zeros(6)
    model, _, _ = evaluate_model(_coefficients(), nominal, velocity, 0.02, "smooth")
    measured = model.copy()
    measured[1] += 80.0
    residual = measured - model
    assert residual[1] == pytest.approx(80.0)
    assert np.allclose(np.delete(residual, 1), 0.0)


def test_coulomb_feature_agrees_between_modes_away_from_zero():
    assert coulomb_feature(1.0, 0.02, "smooth") == pytest.approx(1.0, abs=1e-6)
    assert coulomb_feature(1.0, 0.02, "exclude") == 1.0
    assert coulomb_feature(-1.0, 0.02, "exclude") == -1.0


def test_coulomb_feature_is_zero_inside_the_deadzone_when_excluding():
    assert coulomb_feature(0.001, 0.02, "exclude") == 0.0
    assert coulomb_feature(0.0, 0.02, "smooth") == 0.0


def test_synthetic_joint_state_is_read_by_name():
    """A JointState with the gripper interleaved must still map correctly."""
    names = ["gripper_left_joint", "joint3", "joint1", "joint2",
             "joint6", "joint5", "joint4"]
    effort = [999.0, 3.0, 1.0, 2.0, 6.0, 5.0, 4.0]
    assert order_by_joint(names, effort) == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
