"""Tests for the identification pipeline that need no robot and no ROS graph."""

import math
import os
import tempfile

import numpy as np
import pytest

from om6dof_gravity_comp import units
from om6dof_gravity_comp.identify import (
    stribeck_feature,
    feature_names,
    FEATURE_NAMES,
    build_regressor,
    fit_least_squares,
    friction_features,
    load_csv,
    load_yaml,
    metrics,
    save_yaml,
    stack_joint_arrays,
    time_split,
)
from om6dof_gravity_comp.units import JOINT_NAMES, ma_to_raw, order_by_joint, raw_to_ma

CSV_TEXT = """# om6dof identification dataset
# written_at_iso: 2026-01-01T00:00:00+09:00
# joint_order: joint1,joint2,joint3,joint4,joint5,joint6
# current_unit: raw_dynamixel_ticks
# current_tick_ma: 2.69
# sample_rate_hz: 100.0
t_wall,t_ros,operation_mode,remote_enabled,{q},{qd},{iraw},{ima}
1.0,0.1,JOINT,1,{qv},{qdv},{iv},{imv}
2.0,0.2,JOINT,1,{qv},{qdv},{iv},{imv}
""".format(
    q=",".join(f"q_{n}" for n in JOINT_NAMES),
    qd=",".join(f"qd_{n}" for n in JOINT_NAMES),
    iraw=",".join(f"i_raw_{n}" for n in JOINT_NAMES),
    ima=",".join(f"i_ma_{n}" for n in JOINT_NAMES),
    qv=",".join("0.1" for _ in JOINT_NAMES),
    qdv=",".join("0.2" for _ in JOINT_NAMES),
    iv=",".join("10" for _ in JOINT_NAMES),
    imv=",".join("26.9" for _ in JOINT_NAMES),
)


# -- 1. CSV parsing --------------------------------------------------------
def test_csv_parsing_keeps_metadata_and_columns():
    with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False) as handle:
        handle.write(CSV_TEXT)
        path = handle.name
    try:
        columns, meta = load_csv(path)
        assert meta["current_unit"] == "raw_dynamixel_ticks"
        assert float(meta["current_tick_ma"]) == pytest.approx(2.69)
        assert columns["q_joint1"].tolist() == [0.1, 0.1]
        assert columns["i_raw_joint3"].tolist() == [10.0, 10.0]
    finally:
        os.unlink(path)


def test_csv_parsing_rejects_an_empty_dataset():
    with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False) as handle:
        handle.write("# only metadata\n# current_unit: raw_dynamixel_ticks\n")
        path = handle.name
    try:
        with pytest.raises(ValueError):
            load_csv(path)
    finally:
        os.unlink(path)


def test_stacking_reports_a_missing_joint_rather_than_guessing():
    columns = {f"q_{n}": np.zeros(3) for n in JOINT_NAMES[:-1]}
    with pytest.raises(ValueError, match="joint6"):
        stack_joint_arrays(columns, "q_")


# -- 2. joint-name mapping -------------------------------------------------
def test_joint_mapping_follows_names_not_positions():
    """/joint_states order is not stable, so positional reads are wrong."""
    scrambled = ["joint4", "joint1", "gripper_left_joint", "joint6",
                 "joint2", "joint5", "joint3"]
    values = [4.0, 1.0, 99.0, 6.0, 2.0, 5.0, 3.0]
    assert order_by_joint(scrambled, values) == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_joint_mapping_reports_absence_as_none():
    assert order_by_joint(["joint1"], [1.0])[1] is None


# -- 3. current-unit conversion --------------------------------------------
def test_current_conversion_matches_the_hardware_description():
    # The xacro comment states 45 raw ticks * 2.69 = 121.05 mA.
    assert raw_to_ma(45) == pytest.approx(121.05)
    assert ma_to_raw(121.05) == pytest.approx(45)
    assert units.CURRENT_TICK_MA == pytest.approx(2.69)


def test_servo_model_table_matches_the_hardware():
    assert units.JOINT_SERVO_MODELS["joint4"] == "XM430-W210"
    assert units.JOINT_SERVO_MODELS["joint6"] == "XM430-W210"
    assert units.JOINT_SERVO_MODELS["joint2"] == "XM430-W350"


# -- 4. friction features --------------------------------------------------
def test_friction_features_saturate_away_from_zero():
    velocity = np.array([-5.0, -0.5, 0.5, 5.0])
    coulomb, viscous = friction_features(velocity, 0.02, "smooth")
    assert np.allclose(np.abs(coulomb), 1.0, atol=1e-6)
    assert np.sign(coulomb).tolist() == [-1, -1, 1, 1]
    assert viscous.tolist() == velocity.tolist()


def test_smooth_and_hard_sign_agree_away_from_the_deadzone():
    velocity = np.array([-1.0, 1.0])
    smooth, _ = friction_features(velocity, 0.02, "smooth")
    hard, _ = friction_features(velocity, 0.02, "exclude")
    assert np.allclose(smooth, hard, atol=1e-6)


def test_unknown_deadzone_mode_is_rejected():
    with pytest.raises(ValueError):
        friction_features(np.zeros(3), 0.02, "whatever")


# -- 5. zero-velocity handling ---------------------------------------------
def test_coulomb_feature_is_continuous_through_zero():
    """A step at zero would chatter on quantisation noise."""
    velocity = np.linspace(-0.05, 0.05, 101)
    coulomb, _ = friction_features(velocity, 0.02, "smooth")
    assert abs(coulomb[len(coulomb) // 2]) < 1e-9
    assert np.max(np.abs(np.diff(coulomb))) < 0.2


def test_hard_sign_is_discontinuous_which_is_why_smooth_is_the_default():
    velocity = np.array([-1e-9, 1e-9])
    coulomb, _ = friction_features(velocity, 0.02, "exclude")
    assert abs(coulomb[1] - coulomb[0]) == pytest.approx(2.0)


# -- 6. regression on synthetic data ---------------------------------------
def test_fit_recovers_known_coefficients():
    rng = np.random.default_rng(0)
    n = 4000
    gravity = np.sin(np.linspace(0, 12 * math.pi, n))
    velocity = 0.8 * np.sin(np.linspace(0, 5 * math.pi, n))
    truth = np.array([37.0, 12.0, 4.0, -3.0])
    regressor = build_regressor(gravity, velocity, 0.02, "smooth")
    target = regressor @ truth + rng.normal(0.0, 0.5, n)

    fitted = fit_least_squares(regressor, target)
    assert np.allclose(fitted, truth, atol=0.5), fitted
    assert metrics(target, regressor @ fitted)["r2"] > 0.99


def test_ridge_shrinks_coefficients_but_leaves_the_bias_alone():
    rng = np.random.default_rng(1)
    n = 500
    gravity = rng.normal(size=n)
    velocity = rng.normal(size=n)
    regressor = build_regressor(gravity, velocity, 0.02, "smooth")
    target = regressor @ np.array([50.0, 20.0, 5.0, 8.0])

    plain = fit_least_squares(regressor, target, ridge=0.0)
    shrunk = fit_least_squares(regressor, target, ridge=500.0)
    assert abs(shrunk[0]) < abs(plain[0])
    # The bias column is unpenalised on purpose: penalising it would push the
    # offset into the other coefficients.
    assert abs(shrunk[3]) > abs(plain[3]) * 0.5


def test_time_split_does_not_interleave_neighbouring_samples():
    train, validate = time_split(1000, 0.3)
    assert train.max() < validate.min()
    assert len(validate) == 300


def test_metrics_are_exact_on_a_perfect_fit():
    values = np.array([1.0, 2.0, 3.0])
    score = metrics(values, values)
    assert score["rmse"] == 0.0 and score["mae"] == 0.0
    assert score["r2"] == pytest.approx(1.0)


# -- 7. YAML round trip ----------------------------------------------------
def test_yaml_round_trip_preserves_coefficients_and_units():
    results = {
        joint: {
            "coefficients": dict(zip(FEATURE_NAMES, [1.5, 2.5, 3.5, 4.5])),
            "train": {"rmse": 1.0},
            "validation": {"rmse": 2.0},
        }
        for joint in JOINT_NAMES
    }
    with tempfile.TemporaryDirectory() as directory:
        path = os.path.join(directory, "model.yaml")
        save_yaml(path, results, {"dataset": "x.csv", "deadzone_mode": "smooth",
                                  "velocity_deadzone": 0.02, "ridge": 0.0})
        loaded = load_yaml(path)
        assert loaded["current_unit"] == units.CURRENT_UNIT_RAW
        assert loaded["joint_order"] == list(JOINT_NAMES)
        assert loaded["joints"]["joint2"]["coefficients"]["coulomb"] == 2.5
        assert loaded["features"] == list(FEATURE_NAMES)


def test_stribeck_feature_fades_with_speed_and_follows_direction():
    """Extra friction just after breakaway, gone once the joint is moving."""
    assert stribeck_feature(np.array([0.0]), 0.15)[0] == 0.0
    slow = stribeck_feature(np.array([0.02]), 0.15)[0]
    fast = stribeck_feature(np.array([0.5]), 0.15)[0]
    assert slow > fast > 0
    assert stribeck_feature(np.array([-0.02]), 0.15)[0] == pytest.approx(-slow)


def test_a_falling_friction_curve_forces_negative_viscous_without_stribeck():
    """The reason the coefficient kept coming out impossible.

    Friction measured on this arm falls with speed. Fitting b*sign + c*qd to
    a falling curve can only match it with c negative, which is physically
    impossible; the Stribeck column lets c be positive again.
    """
    rng = np.random.default_rng(5)
    velocity = np.concatenate([
        rng.uniform(0.02, 0.5, 2000), -rng.uniform(0.02, 0.5, 2000)])
    # Friction that falls from 50 to 40 across the speed range, as measured.
    truth = np.sign(velocity) * (40.0 + 10.0 * np.exp(-np.abs(velocity) / 0.15))
    gravity = np.zeros_like(velocity)

    plain = fit_least_squares(
        build_regressor(gravity, velocity, 0.02, "smooth"), truth)
    assert plain[2] < 0, "the falling curve did not force a negative viscous"

    with_stribeck = fit_least_squares(
        build_regressor(gravity, velocity, 0.02, "smooth", stribeck=True),
        truth)
    assert with_stribeck[2] > -1e-6, "viscous still negative with Stribeck"
    assert with_stribeck[4] > 0, "Stribeck term should be positive"


def test_feature_names_track_the_regressor_width():
    velocity = np.linspace(-1, 1, 10)
    gravity = np.zeros(10)
    for stribeck in (False, True):
        design = build_regressor(gravity, velocity, 0.02, "smooth", stribeck)
        assert design.shape[1] == len(feature_names(stribeck))
