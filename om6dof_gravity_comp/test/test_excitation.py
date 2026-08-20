"""Excitation planner tests. No ROS, no hardware."""

import math

import numpy as np
import pytest

from om6dof_gravity_comp.excitation import (
    FREQUENCY_SPREAD,
    plan_many,
    LEAD_IN_S,
    trajectory_times,
    DEFAULT_ACCELERATION_LIMIT,
    DEFAULT_VELOCITY_LIMIT,
    URDF_LIMITS,
    default_components,
    plan,
    ramp,
)


def test_ramp_starts_and_ends_at_rest():
    assert ramp(0.0, 60.0, 3.0) == pytest.approx(0.0)
    assert ramp(60.0, 60.0, 3.0) == pytest.approx(0.0)
    assert ramp(30.0, 60.0, 3.0) == pytest.approx(1.0)


def test_plan_stays_inside_the_conservative_band():
    result = plan("joint2", centre=-0.5, duration=40.0, sample_rate=20.0,
                  base_frequency=0.05)
    low, high = result["conservative_band"]
    assert result["q"].min() >= low - 1e-9
    assert result["q"].max() <= high + 1e-9
    assert not result["violations"], result["violations"]


def test_plan_uses_only_a_fraction_of_the_hard_range():
    """The hard stops leave no margin for tracking error, so they are not
    what the planner aims at."""
    result = plan("joint2", centre=0.0, duration=40.0, sample_rate=20.0,
                  base_frequency=0.05, range_fraction=0.35)
    hard_low, hard_high = URDF_LIMITS["joint2"]
    span = result["q"].max() - result["q"].min()
    assert span < 0.5 * (hard_high - hard_low)


def test_plan_flags_a_range_that_would_hit_the_stops():
    result = plan("joint2", centre=0.0, duration=40.0, sample_rate=20.0,
                  base_frequency=0.05, range_fraction=1.6)
    assert result["violations"], "a range wider than the joint was accepted"


def test_plan_flags_a_frequency_that_would_move_too_fast():
    result = plan("joint1", centre=0.0, duration=30.0, sample_rate=100.0,
                  base_frequency=2.0)
    assert any("velocity" in v or "acceleration" in v
               for v in result["violations"]), result["violations"]


def test_trajectory_begins_and_ends_at_the_centre():
    """Starting anywhere else would step the arm the moment it is sent."""
    result = plan("joint3", centre=0.2, duration=30.0, sample_rate=20.0,
                  base_frequency=0.05)
    assert result["q"][0] == pytest.approx(0.2, abs=1e-6)
    assert result["q"][-1] == pytest.approx(0.2, abs=0.02)


def test_components_do_not_repeat_quickly():
    """Frequency ratios are chosen so the pattern does not close early; a
    short repeat would revisit the same handful of configurations."""
    components = default_components(0.05)
    ratios = [c.frequency / components[0].frequency for c in components]
    assert ratios[0] == pytest.approx(1.0)
    for ratio in ratios[1:]:
        assert abs(ratio - round(ratio)) > 0.1, f"{ratio} is nearly an integer"


def test_unknown_joint_is_rejected():
    with pytest.raises(ValueError):
        plan("elbow", 0.0, 10.0, 20.0, 0.05)


def test_limits_table_matches_six_joints():
    assert len(URDF_LIMITS) == 6
    for lower, upper in URDF_LIMITS.values():
        assert lower < upper


def test_first_waypoint_is_strictly_in_the_future():
    """time_from_start = 0 makes JointTrajectoryController drop the whole
    trajectory, and drop it silently -- the publisher sees success while the
    arm never moves."""
    times = np.arange(0.0, 5.0, 0.05)
    stamps = trajectory_times(times)
    assert stamps[0] > 0.0
    assert stamps[0] == pytest.approx(LEAD_IN_S)


def test_waypoint_times_strictly_increase():
    stamps = trajectory_times(np.arange(0.0, 3.0, 0.05))
    assert all(b > a for a, b in zip(stamps, stamps[1:]))


def test_duplicate_times_are_rejected_rather_than_sent():
    with pytest.raises(ValueError, match="strictly increase"):
        trajectory_times([0.0, 0.1, 0.1, 0.2])


def test_a_non_positive_lead_in_is_rejected():
    with pytest.raises(ValueError, match="future"):
        trajectory_times([0.0, 0.1], lead_in=0.0)


def test_trajectory_ends_at_rest():
    """arm_controller sets allow_nonzero_velocity_at_trajectory_end: false,
    so a trajectory that still has speed at its last point is rejected --
    silently, with the arm simply never moving."""
    result = plan("joint2", centre=-0.69, duration=60.0, sample_rate=20.0,
                  base_frequency=0.05)
    assert result["qd"][-1] == 0.0
    assert result["qd"][0] == 0.0


def test_last_waypoint_lands_on_the_requested_duration():
    """Stopping one step short left the window open at the final point."""
    result = plan("joint3", centre=0.0, duration=30.0, sample_rate=20.0,
                  base_frequency=0.05)
    assert result["times"][-1] == pytest.approx(30.0)


def test_trajectory_starts_and_ends_at_the_centre_exactly():
    result = plan("joint5", centre=0.4, duration=20.0, sample_rate=20.0,
                  base_frequency=0.05)
    assert result["q"][0] == pytest.approx(0.4, abs=1e-12)
    assert result["q"][-1] == pytest.approx(0.4, abs=1e-12)


def test_multi_joint_plan_covers_every_requested_joint():
    joints = ["joint2", "joint3", "joint5"]
    centres = {"joint2": -0.69, "joint3": 1.39, "joint5": 0.91}
    result = plan_many(joints, centres, 60.0, 20.0, 0.05)
    assert set(result["plans"]) == set(joints)
    assert not result["violations"], result["violations"]
    for joint in joints:
        assert len(result["plans"][joint]["q"]) == len(result["times"])


def test_multi_joint_sweeps_do_not_move_in_lockstep():
    """Sweeping in phase revisits the same arm configurations, which is the
    thing simultaneous excitation exists to avoid."""
    joints = ["joint2", "joint3", "joint5"]
    centres = {j: 0.0 for j in joints}
    result = plan_many(joints, centres, 60.0, 20.0, 0.05)
    a = result["plans"]["joint2"]["q"]
    b = result["plans"]["joint3"]["q"]
    correlation = float(np.corrcoef(a, b)[0, 1])
    assert abs(correlation) < 0.9, f"joints move together (r={correlation:.2f})"


def test_frequency_spread_has_no_repeats():
    assert len(set(FREQUENCY_SPREAD)) == len(FREQUENCY_SPREAD)


def test_multi_joint_plan_reports_violations_per_joint():
    result = plan_many(["joint2", "joint3"], {"joint2": 0.0, "joint3": 0.0},
                       60.0, 20.0, 0.05, range_fraction=1.6)
    assert result["violations"]
    assert any(v.startswith("joint2:") for v in result["violations"])
