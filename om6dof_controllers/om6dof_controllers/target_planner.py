"""Bounded, deterministic absolute-pose search; no ROS or hardware writes."""

from dataclasses import dataclass
import math

import numpy as np

from .control_math import rotation_error


POSITION_TOLERANCE = 0.001  # metres
ORIENTATION_TOLERANCE = math.radians(0.5)
JOINT_TOLERANCE = math.radians(0.5)


def pose_error(ik, joints, position, rotation):
    xyz, actual_rotation = ik.fk_pose(np.asarray(joints, dtype=float))
    errors = (float(np.linalg.norm(position - xyz)),
              float(np.linalg.norm(rotation_error(rotation, actual_rotation))))
    if not all(math.isfinite(value) for value in errors):
        raise ValueError('non-finite FK error')
    return errors


def path_is_clear(ik, start, goal, radius):
    """Sample the straight joint path at <= 0.02 rad per joint.

    Uses the existing approximate link-capsule check, not an environment map
    or a continuous swept-volume collision guarantee.
    """
    steps = max(1, int(math.ceil(float(np.max(np.abs(goal - start))) / 0.02)))
    return not any(ik.self_collides(start + (goal - start) * fraction, radius)
                   for fraction in np.linspace(0.0, 1.0, steps + 1))


@dataclass
class TargetPlan:
    joints: np.ndarray
    approximate: bool
    position_error: float
    orientation_error: float


def plan_pose(ik, start, position, rotation, lower, upper, ready,
              collision_radius=None):
    """Try current, READY, zero and opposite-elbow seeds before approximation.

    Validate the final command after applying effective limits. Exact valid
    solutions take precedence over approximations; an approximate solution
    must improve the weighted pose error relative to the measured start.
    """
    start = np.asarray(start, dtype=float)
    lower, upper = np.asarray(lower), np.asarray(upper)
    position, rotation = np.asarray(position), np.asarray(rotation)
    start_errors = pose_error(ik, start, position, rotation)
    score = lambda errors: errors[0] + 0.10 * errors[1]
    heading = math.atan2(float(position[1]), float(position[0]))
    seeds = [start, np.asarray(ready), np.zeros(6),
             np.array([heading, -0.7, 1.4, 0, 0.8, 0]),
             np.array([heading, 0.7, -1.2, 0, 1.5, 0])]
    exact, approximate, seen = [], [], []
    for seed in seeds:
        seed = np.clip(seed, lower, upper)
        if any(np.allclose(seed, prior, atol=1e-8) for prior in seen):
            continue
        seen.append(seed)
        solution, _ = ik.solve_pose_ik(seed, position, rotation, max_iter=160)
        solution = np.asarray(solution, dtype=float)
        if solution.shape != (6,) or not np.all(np.isfinite(solution)):
            continue
        solution = np.clip(solution, lower, upper)
        errors = pose_error(ik, solution, position, rotation)
        is_exact = (errors[0] <= POSITION_TOLERANCE and
                    errors[1] <= ORIENTATION_TOLERANCE)
        if not is_exact and score(errors) >= score(start_errors) - 1e-6:
            continue
        if collision_radius is not None and not path_is_clear(
                ik, start, solution, collision_radius):
            continue
        candidate = TargetPlan(solution, not is_exact, *errors)
        if is_exact:
            exact.append(candidate)
            # A current-seeded exact solution normally needs least movement;
            # avoid exploring other elbow configurations unnecessarily.
            if len(seen) == 1:
                return candidate
        else:
            approximate.append(candidate)
    if exact:
        return min(exact, key=lambda plan: np.linalg.norm(plan.joints - start))
    if approximate:
        return min(approximate, key=lambda plan:
                   plan.position_error + 0.10 * plan.orientation_error)
    raise ValueError('No improving pose with a clear joint path found within limits')
