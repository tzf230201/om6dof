"""Static sweep planning, tested without a robot."""

import math

import numpy as np
import pytest

from om6dof_gravity_comp.excitation import URDF_LIMITS
from om6dof_gravity_comp.static_sweep import (
    GRAVITY_JOINTS,
    check_poses,
    estimate_duration,
    plan_visits,
    pose_grid,
)
from om6dof_gravity_comp.units import JOINT_NAMES

CENTRES = {j: 0.0 for j in JOINT_NAMES}


def test_grid_covers_only_the_joints_gravity_can_load():
    """The other three have axes along gravity; moving them teaches nothing
    and only lengthens the run."""
    poses = pose_grid(CENTRES, steps=3)
    for joint in JOINT_NAMES:
        values = {round(p[joint], 6) for p in poses}
        if joint in GRAVITY_JOINTS:
            assert len(values) == 3, f"{joint} should vary"
        else:
            assert len(values) == 1, f"{joint} should be held"


def test_grid_size_is_the_product_of_the_axes():
    assert len(pose_grid(CENTRES, steps=4)) == 4 ** len(GRAVITY_JOINTS)


def test_every_generated_pose_is_inside_the_joint_limits():
    for steps in (2, 3, 5):
        assert check_poses(pose_grid(CENTRES, steps=steps)) == []


def test_poses_outside_the_limits_are_reported():
    bad = [{**CENTRES, "joint2": URDF_LIMITS["joint2"][1] + 0.5}]
    problems = check_poses(bad)
    assert problems and "joint2" in problems[0]


def test_each_pose_is_visited_from_both_directions():
    """Friction flips sign with approach direction; gravity does not. Without
    both visits the two cannot be separated at rest."""
    poses = pose_grid(CENTRES, steps=2)
    visits = plan_visits(poses)
    labels = [label for label, _ in visits]
    assert labels.count("from_above") == len(poses)
    assert labels.count("from_below") == len(poses)


def test_approach_backs_off_on_opposite_sides():
    pose = {**CENTRES, "joint2": 0.5}
    visits = plan_visits([pose], approach=0.12)
    above = visits[0][1]["joint2"]
    below = visits[2][1]["joint2"]
    assert above > 0.5 and below < 0.5


def test_approach_points_stay_inside_the_limits():
    """Backing off must not push the stand-off past a stop."""
    lower, upper = URDF_LIMITS["joint2"]
    edge = {**CENTRES, "joint2": upper - 0.03}
    visits = plan_visits([edge], approach=0.5)
    for _, pose in visits:
        assert lower <= pose["joint2"] <= upper


def test_duration_grows_with_dwell_and_pose_count():
    poses = pose_grid(CENTRES, steps=2)
    visits = plan_visits(poses)
    short = estimate_duration(visits, 1.0, 0.25, CENTRES)
    long = estimate_duration(visits, 5.0, 0.25, CENTRES)
    assert long > short
    # Two dwells per pose, so the extra time is 2 * poses * extra dwell.
    assert long - short == pytest.approx(2 * len(poses) * 4.0, rel=0.01)
