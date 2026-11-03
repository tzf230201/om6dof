"""Pure offline admittance invariants; no ROS node or actuator access."""

from dataclasses import FrozenInstanceError, replace
import math

import pytest

from om6dof_controllers.admittance import AdmittanceConfig, JointAdmittance


ZERO = (0.0,) * 6


def _config(**kwargs):
    return AdmittanceConfig(lower=(-1.0,) * 6, upper=(1.0,) * 6,
                            raw_per_nm=(200.0,) * 6, **kwargs)


def _node(**kwargs):
    node = JointAdmittance(_config(**kwargs))
    node.reset(ZERO)
    return node


def _step(node, force=0.0, target=ZERO, valid=True, dt=0.01, model=ZERO, measured=None):
    # A positive model-minus-motor residual is a positive external push.
    current = tuple((model[i] - force) * node.config.raw_per_nm[i] for i in range(6))
    # Ideal servo follows the previous command unless the test blocks it.
    if measured is None:
        measured = tuple(node._command) if node._initialized else ZERO
    return node.step(target, measured, ZERO, current, model, dt, valid=valid)


@pytest.mark.parametrize("direction", [-1.0, 1.0])
def test_push_yields_in_correct_direction_then_returns_original_target(direction):
    node = _node()
    for _ in range(500):
        result = _step(node, direction * 0.4)
    assert all(result.contact)
    assert all(direction * x > 0.05 for x in result.offset)
    assert all(direction * x > 0.05 for x in result.command)
    for _ in range(1500):
        result = _step(node)
    assert not any(result.contact)
    assert max(map(abs, result.offset)) < 1e-5
    assert max(map(abs, result.command)) < 1e-4


def test_absolute_target_is_not_replaced_by_displaced_measurement():
    node = _node()
    target = (0.25,) * 6
    for _ in range(1000):
        result = _step(node, 0.4, target)
    assert max(result.nominal) < 0.01  # Progress pauses while the push persists.
    for _ in range(1500):
        result = _step(node, target=target)
    assert max(abs(x - 0.25) for x in result.command) < 0.001


def test_frozen_bias_removes_known_model_error_without_learning_sustained_push():
    node = _node(torque_bias=(0.12,) * 6)
    for _ in range(100):
        result = _step(node, 0.12)
    assert result.offset == ZERO
    assert not any(result.contact)
    for _ in range(2000):
        result = _step(node, 0.52)
    assert all(result.contact)
    assert min(result.offset) > 0.05
    assert node.config.torque_bias == (0.12,) * 6
    with pytest.raises(FrozenInstanceError):
        node.config.torque_bias = ZERO


def test_below_threshold_noise_and_short_spikes_never_drift():
    node = _node()
    for index in range(1000):
        result = _step(node, 0.05 * math.sin(index))
    assert result.offset == ZERO
    for _ in range(3):
        result = _step(node, 0.3)
    assert not any(result.contact)
    assert result.offset == ZERO


def test_contact_release_requires_hysteresis_and_dwell():
    node = _node()
    for _ in range(20):
        result = _step(node, 0.4)
    assert all(result.contact)
    for _ in range(100):
        result = _step(node, 0.10)
    assert all(result.contact)  # Between the off/on thresholds retains contact.
    for _ in range(5):
        result = _step(node)
    assert all(result.contact)
    for _ in range(100):
        result = _step(node)
    assert not any(result.contact)


def test_unavailable_feedback_freezes_command_displacement_and_stops_velocity():
    node = _node()
    for _ in range(100):
        before = _step(node, 0.4)
    for _ in range(100):
        after = _step(node, valid=False)
        assert not after.valid
        assert after.command == before.command
        assert after.nominal == before.nominal
        assert after.offset == before.offset
        assert after.offset_velocity == ZERO


@pytest.mark.parametrize("dt", [0.0, -0.01, 0.2, float("nan"), float("inf"), True, "0.01"])
def test_bad_control_interval_freezes(dt):
    node = _node()
    before = _step(node, 0.4)
    after = _step(node, dt=dt)
    assert not after.valid
    assert after.command == before.command


@pytest.mark.parametrize("argument", ["q_nominal", "q_measured", "q_velocity", "raw_current", "model_torque"])
@pytest.mark.parametrize("bad", [float("nan"), float("inf"), True, "0.0", None])
def test_bad_telemetry_freezes_all_motion(argument, bad):
    node = _node()
    kwargs = dict(q_nominal=ZERO, q_measured=ZERO, q_velocity=ZERO,
                  raw_current=ZERO, model_torque=ZERO, dt=0.01)
    kwargs[argument] = (bad,) * 6
    result = node.step(**kwargs)
    assert not result.valid
    assert result.command == ZERO


def test_joint_limits_clip_displacement_and_unwind_after_release():
    node = JointAdmittance(_config())
    pose = (0.98,) * 6
    node.reset(pose)
    for _ in range(1000):
        result = _step(node, 10.0, pose)
    assert max(result.command) <= 1.0
    assert max(result.offset) <= 0.0200000001
    assert result.offset_velocity == ZERO
    for _ in range(1500):
        result = _step(node, target=pose)
    assert max(abs(x - 0.98) for x in result.command) < 0.001


def test_max_displacement_and_command_velocity_acceleration_remain_bounded():
    node = _node()
    previous = ZERO
    previous_velocity = ZERO
    for _ in range(1500):
        result = _step(node, 100.0, target=(0.5,) * 6)
        velocity = tuple((a - b) / 0.01 for a, b in zip(result.command, previous))
        acceleration = tuple((a - b) / 0.01 for a, b in zip(velocity, previous_velocity))
        assert max(map(abs, result.offset)) <= 0.15 + 1e-10
        assert max(map(abs, velocity)) <= 0.10 + 1e-9
        assert max(map(abs, acceleration)) <= 0.30 + 1e-8
        previous, previous_velocity = result.command, velocity


def test_sustained_contact_pauses_nominal_path_then_resumes_original_target():
    node = _node()
    target = (0.6,) * 6
    for _ in range(150):
        result = _step(node, 0.4, target)
    paused = result.nominal
    for _ in range(800):
        result = _step(node, 0.4, target)
    assert max(abs(a - b) for a, b in zip(paused, result.nominal)) < 1e-8
    assert max(result.nominal) < 0.01
    for _ in range(2500):
        result = _step(node, target=target)
    assert max(abs(x - 0.6) for x in result.command) < 1e-4


def test_blocked_measurement_caps_reference_lead_without_contact_detection():
    node = _node()
    for _ in range(3000):
        result = _step(node, target=(0.8,) * 6, measured=ZERO)
        assert max(result.command) <= 0.05 + 1e-9
        assert max(result.nominal) <= 0.05 + 1e-9
    assert result.tracking_limited
    assert not any(result.contact)


def test_measurement_jump_prioritizes_command_rate_over_impossible_lead_bound():
    node = _node()
    previous = ZERO
    previous_velocity = ZERO
    for index in range(100):
        measured = ZERO if index < 30 else (-0.4,) * 6
        result = _step(node, target=(0.8,) * 6, measured=measured)
        velocity = tuple((a - b) / 0.01 for a, b in zip(result.command, previous))
        acceleration = tuple((a - b) / 0.01 for a, b in zip(velocity, previous_velocity))
        assert max(map(abs, velocity)) <= 0.1 + 1e-9
        assert max(map(abs, acceleration)) <= 0.3 + 1e-8
        if index == 30:
            assert result.tracking_limited
            assert min(result.command) > -0.35  # No instantaneous jump to moving lead bound.
        previous, previous_velocity = result.command, velocity


@pytest.mark.parametrize("parameter,value", [
    ("raw_per_nm", (0.0,) * 6), ("inertia", (-1.0,) * 6),
    ("damping", (float("nan"),) * 6), ("stiffness", ZERO),
    ("max_displacement", (True,) * 6), ("torque_off", (0.16,) * 6),
    ("torque_on", (0.1,) * 5), ("contact_dwell", 0.0),
    ("release_dwell", float("inf")), ("max_dt", 0.2),
])
def test_invalid_configuration_is_rejected(parameter, value):
    with pytest.raises(ValueError):
        replace(_config(), **{parameter: value})


def test_reset_requires_valid_in_limit_pose_and_step_requires_reset():
    node = JointAdmittance(_config())
    with pytest.raises(RuntimeError):
        _step(node)
    with pytest.raises(ValueError):
        node.reset((2.0,) * 6)
    with pytest.raises(ValueError):
        node.reset((0.0,) * 5)
