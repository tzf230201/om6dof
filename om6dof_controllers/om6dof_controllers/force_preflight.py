"""Read-only force-mode position/speed checks, before torque ownership changes."""

import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


FORCE_JOINT_LIMIT_MARGIN = 0.0


def load_force_limits():
    from .ik_solver import joint_limits_from_urdf
    names = [f'joint{i}' for i in range(1, 7)]
    path = Path(get_package_share_directory('om6dof_controllers')) / 'config/position_impedance.yaml'
    params = yaml.safe_load(path.read_text())['/**']['om6dof_position_impedance_controller']['ros__parameters']
    lower, upper = joint_limits_from_urdf(
        names, base_link=params['base_link'], tip_link=params['tip_link'])
    margin = FORCE_JOINT_LIMIT_MARGIN
    speeds = params['max_velocity']
    if len(speeds) == 1:
        speeds = speeds * 6
    if (not math.isfinite(margin) or margin < 0 or len(speeds) != 6 or
            any(not math.isfinite(v) or v <= 0 for v in speeds)):
        raise ValueError('Invalid force-mode limits')
    return ([v + margin for v in lower], [v - margin for v in upper],
            [min(float(v), .05) for v in speeds])


def force_pose_problem(positions, velocities, limits):
    if len(positions) != 6 or len(velocities) != 6 or not all(
            math.isfinite(v) for v in list(positions) + list(velocities)):
        return 'Complete finite measured joint position and velocity are required.'
    lower, upper, speeds = limits
    for i, (q, v, lo, hi, cap) in enumerate(zip(positions, velocities, lower, upper, speeds), 1):
        if q < lo or q > hi:
            return (f'joint{i} is {math.degrees(q):.2f} deg; force-mode range '
                    f'[{math.degrees(lo):.2f}, {math.degrees(hi):.2f}] deg. '
                    'Use position control to reach an interior pose before force start.')
        if abs(v) > cap:
            return f'joint{i} speed {v:.4f} rad/s exceeds activation limit {cap:.4f} rad/s.'
    return ''
