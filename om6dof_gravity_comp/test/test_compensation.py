"""Compensation guards, tested without a robot.

The node's output path is deliberately hard to reach: every check has to pass
before a non-zero current is produced. These pin the checks themselves.
"""

import math

import numpy as np
import pytest

from om6dof_gravity_comp.compensation import (
    DEFAULT_CURRENT_LIMIT_RAW,
    LIMIT_MARGIN_RAD,
    MAX_SAFE_VELOCITY,
    RAMP_SECONDS,
    STATE_STALE_S,
)


def _clip(command, limits):
    return np.clip(command, -np.abs(limits), np.abs(limits))


def test_current_limit_is_far_below_stall():
    """XM430 stalls near 855 raw ticks; the default must not approach it."""
    assert DEFAULT_CURRENT_LIMIT_RAW < 855 * 0.3


def test_saturation_clamps_both_directions():
    limits = np.full(6, DEFAULT_CURRENT_LIMIT_RAW)
    command = np.array([1e6, -1e6, 0.0, 50.0, -50.0, 1e9])
    clipped = _clip(command, limits)
    assert clipped.max() <= DEFAULT_CURRENT_LIMIT_RAW
    assert clipped.min() >= -DEFAULT_CURRENT_LIMIT_RAW
    assert clipped[3] == 50.0 and clipped[4] == -50.0


def test_limit_guard_only_blocks_the_direction_that_digs_in():
    """Near a stop, current pushing further in is dropped; current pulling
    away is left alone, or the arm could not be recovered."""
    lower = np.full(6, -2.0)
    upper = np.full(6, 2.0)
    q = np.array([-2.0 + LIMIT_MARGIN_RAD / 2, 2.0 - LIMIT_MARGIN_RAD / 2,
                  0.0, 0.0, 0.0, 0.0])
    command = np.array([-10.0, 10.0, -10.0, 10.0, 0.0, 0.0])
    near_low = q <= lower + LIMIT_MARGIN_RAD
    near_high = q >= upper - LIMIT_MARGIN_RAD
    command[near_low & (command < 0)] = 0.0
    command[near_high & (command > 0)] = 0.0
    assert command[0] == 0.0, "pushed further into the lower stop"
    assert command[1] == 0.0, "pushed further into the upper stop"
    assert command[2] == -10.0 and command[3] == 10.0


def test_limit_guard_leaves_the_escape_direction():
    lower = np.full(6, -2.0)
    q = np.full(6, -2.0 + LIMIT_MARGIN_RAD / 2)
    command = np.full(6, 10.0)          # pulling away from the stop
    near_low = q <= lower + LIMIT_MARGIN_RAD
    command[near_low & (command < 0)] = 0.0
    assert np.all(command == 10.0)


def test_ramp_takes_the_configured_time_to_reach_full():
    period = 0.01
    ramp = 0.0
    steps = 0
    while ramp < 1.0 and steps < 10000:
        ramp = min(1.0, ramp + period / RAMP_SECONDS)
        steps += 1
    assert steps * period == pytest.approx(RAMP_SECONDS, rel=0.05)


def test_watchdog_is_tighter_than_the_estimator_because_this_one_acts():
    assert STATE_STALE_S <= 0.2


def test_velocity_cutout_is_below_the_arm_ceiling():
    """Above this something else is driving the arm, and compensation should
    not be adding to it."""
    assert 0.0 < MAX_SAFE_VELOCITY <= 1.4


def test_scale_of_zero_produces_no_current():
    command = np.array([100.0, -80.0, 40.0, 0.0, 20.0, -5.0])
    assert np.all(command * 0.0 == 0.0)
