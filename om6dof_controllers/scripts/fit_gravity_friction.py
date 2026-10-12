#!/usr/bin/env python3
"""Fit gravity scale and Coulomb/viscous friction from a multisine CSV.

The input is produced by ``multisine_identification.py``.  ``effort`` is the
raw Dynamixel Present Current value, so the resulting friction values are in
that same unit; do not copy them into an Nm controller configuration until a
current-to-joint-torque scale has been calibrated.
"""

import argparse
import csv
import hashlib
import math
import subprocess
from collections import defaultdict
from pathlib import Path

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from urdf_parser_py.urdf import URDF


ARM_JOINTS = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6')


def rpy_matrix(rpy):
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def rigid_transform(xyz, rpy):
    transform = np.eye(4)
    transform[:3, :3] = rpy_matrix(rpy)
    transform[:3, 3] = [float(value) for value in xyz]
    return transform


def axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm == 0.0:
        return np.eye(4)
    x, y, z = axis / norm
    c, s = math.cos(angle), math.sin(angle)
    rotation = np.array([
        [c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
        [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
        [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)],
    ])
    transform = np.eye(4)
    transform[:3, :3] = rotation
    return transform


class UrdfGravity:
    """Potential-energy gravity model for the whole URDF tree, including branches."""

    def __init__(self, urdf_text):
        self.robot = URDF.from_xml_string(urdf_text)
        self.links = {link.name: link for link in self.robot.links}
        self.children = defaultdict(list)
        child_links = set()
        for joint in self.robot.joints:
            self.children[joint.parent].append(joint)
            child_links.add(joint.child)
        roots = set(self.links) - child_links
        if len(roots) != 1:
            raise RuntimeError('URDF must have exactly one root link, got: %s' % sorted(roots))
        self.root = roots.pop()

    @staticmethod
    def _joint_value(joint, positions):
        if joint.type == 'fixed':
            return 0.0
        if joint.name in positions:
            return positions[joint.name]
        if joint.mimic is not None and joint.mimic.joint in positions:
            return (positions[joint.mimic.joint] * joint.mimic.multiplier +
                    joint.mimic.offset)
        # Unrecorded joints (the gripper here) are held at their URDF zero pose.
        return 0.0

    def potential(self, positions):
        transforms = {self.root: np.eye(4)}
        pending = [self.root]
        while pending:
            parent = pending.pop()
            for joint in self.children[parent]:
                origin = joint.origin
                xyz = origin.xyz if origin is not None else (0.0, 0.0, 0.0)
                rpy = origin.rpy if origin is not None else (0.0, 0.0, 0.0)
                transform = transforms[parent] @ rigid_transform(xyz, rpy)
                value = self._joint_value(joint, positions)
                if joint.type in ('revolute', 'continuous'):
                    transform = transform @ axis_rotation(joint.axis, value)
                elif joint.type == 'prismatic':
                    translation = np.eye(4)
                    translation[:3, 3] = np.asarray(joint.axis, dtype=float) * value
                    transform = transform @ translation
                transforms[joint.child] = transform
                pending.append(joint.child)

        # U = m g z.  KDL's gravity solver yields dU/dq under the same convention.
        energy = 0.0
        for name, link in self.links.items():
            if link.inertial is None or link.inertial.mass <= 0.0:
                continue
            origin = link.inertial.origin
            com = origin.xyz if origin is not None else (0.0, 0.0, 0.0)
            world_com = transforms[name] @ np.array([*com, 1.0])
            energy += float(link.inertial.mass) * 9.80665 * world_com[2]
        return energy

    def gravity(self, positions, names=ARM_JOINTS, epsilon=1.0e-5):
        output = {}
        for name in names:
            plus, minus = dict(positions), dict(positions)
            plus[name] = plus.get(name, 0.0) + epsilon
            minus[name] = minus.get(name, 0.0) - epsilon
            output[name] = (self.potential(plus) - self.potential(minus)) / (2.0 * epsilon)
        return output


def expand_xacro(path):
    try:
        return subprocess.run(['xacro', path], check=True, text=True, capture_output=True).stdout
    except subprocess.CalledProcessError as error:
        raise RuntimeError('xacro failed: %s' % error.stderr.strip()) from error


def estimate_velocity(times, positions, window_seconds=1.0, max_gap_seconds=0.2):
    """Offline centered quadratic derivative on actual, possibly uneven timestamps.

    Never interpolate across an acquisition gap. Discard segment edges and
    sparse windows instead of extrapolating a derivative from one side.
    """
    times = np.asarray(times, dtype=float)
    positions = np.asarray(positions, dtype=float)
    if (not math.isfinite(window_seconds) or window_seconds <= 0 or
            not math.isfinite(max_gap_seconds) or max_gap_seconds <= 0):
        raise ValueError('velocity window and maximum gap must be finite and positive')
    if (len(times) != len(positions) or len(times) < 7 or
            not np.all(np.isfinite(times)) or not np.all(np.isfinite(positions)) or
            np.any(np.diff(times) <= 0)):
        raise ValueError('velocity estimation requires finite, strictly increasing timestamps and >=7 samples')
    velocity = np.full_like(positions, np.nan, dtype=float)
    half = window_seconds / 2.0
    boundaries = [0, *(np.flatnonzero(np.diff(times) > max_gap_seconds) + 1), len(times)]
    for start, stop in zip(boundaries, boundaries[1:]):
        segment = times[start:stop]
        for i in range(start, stop):
            t = times[i]
            if t - segment[0] < half or segment[-1] - t < half:
                continue
            lo = start + np.searchsorted(segment, t - half, side='left')
            hi = start + np.searchsorted(segment, t + half, side='right')
            if hi - lo < 7:
                continue
            x = (times[lo:hi] - t) / half
            design = np.column_stack((np.ones_like(x), x, x*x))
            coefficients, _, rank, _ = np.linalg.lstsq(design, positions[lo:hi], rcond=None)
            if rank == 3:
                velocity[i] = coefficients[1] / half
    return velocity


def load_samples(path, velocity_source='reported', window_seconds=1.0,
                 max_gap_seconds=0.2, return_metadata=False, row_slice=None):
    with open(path, newline='', encoding='utf-8') as file:
        rows = list(csv.DictReader(file))
    if not rows:
        raise RuntimeError('CSV has no data rows: %s' % path)
    required = [f'{name}_{field}' for name in ARM_JOINTS
                for field in ('position_rad', 'velocity_rad_s', 'effort_raw')]
    missing = [field for field in required if field not in rows[0]]
    if missing:
        raise RuntimeError('CSV does not look like multisine output; missing: %s' % ', '.join(missing))
    if row_slice is not None:
        # Split before smoothing so held-out observations cannot influence a
        # training derivative through the centered window.
        rows = rows[row_slice]
    samples, times = [], []
    for row in rows:
        positions = {name: float(row[f'{name}_position_rad']) for name in ARM_JOINTS}
        velocity = {name: float(row[f'{name}_velocity_rad_s']) for name in ARM_JOINTS}
        effort = {name: float(row[f'{name}_effort_raw']) for name in ARM_JOINTS}
        if all(math.isfinite(value) for value in (*positions.values(), *velocity.values(), *effort.values())):
            samples.append((positions, velocity, effort))
            if velocity_source == 'position':
                if 'ros_stamp_s' not in row:
                    raise RuntimeError('position-derived velocity requires ros_stamp_s in the CSV')
                times.append(float(row['ros_stamp_s']))
    if not samples:
        raise RuntimeError('CSV contains no finite samples')
    metadata = {
        'source': velocity_source,
        'source_rows': len(rows),
        'finite_rows': len(samples),
        'reported_zero_fraction': {name: float(np.mean([s[1][name] == 0 for s in samples]))
                                   for name in ARM_JOINTS},
    }
    if velocity_source == 'position':
        positions = np.array([[s[0][name] for name in ARM_JOINTS] for s in samples])
        estimated = estimate_velocity(times, positions, window_seconds, max_gap_seconds)
        samples = [(s[0], dict(zip(ARM_JOINTS, map(float, v))), s[2])
                   for s, v in zip(samples, estimated) if np.all(np.isfinite(v))]
        metadata.update({'method': 'centered_quadratic', 'time_column': 'ros_stamp_s',
                         'window_seconds': window_seconds, 'max_gap_seconds': max_gap_seconds})
        if not samples:
            raise RuntimeError('no complete velocity-estimation windows; check timestamps, gaps and duration')
    elif velocity_source != 'reported':
        raise ValueError('unknown velocity source: %s' % velocity_source)
    metadata['retained_rows'] = len(samples)
    return (samples, metadata) if return_metadata else samples


def fit_joint(samples, gravity_values, joint, deadzone, velocity_min):
    rows = [(sample, gravity) for sample, gravity in zip(samples, gravity_values)
            if abs(sample[1][joint]) >= velocity_min]
    if len(rows) < 20:
        raise RuntimeError('%s has only %d samples above velocity threshold' % (joint, len(rows)))
    gravity_column = np.array([gravity[joint] for _, gravity in rows])
    gravity_identifiable = np.ptp(gravity_column) >= 1.0e-6
    friction_columns = np.array([
        [math.tanh(sample[1][joint] / deadzone), sample[1][joint], 1.0]
        for sample, _ in rows
    ])
    design = (np.column_stack((gravity_column, friction_columns)) if gravity_identifiable
              else friction_columns)
    target = np.array([sample[2][joint] for sample, _ in rows])
    parameters, _, rank, singular = np.linalg.lstsq(design, target, rcond=None)
    predicted = design @ parameters
    residual = target - predicted
    centered = target - np.mean(target)
    variance = np.dot(centered, centered)
    r_squared = float(1.0 - np.dot(residual, residual) / variance) if variance > 0 else None
    if gravity_identifiable:
        gravity_scale, coulomb, viscous, bias = parameters
    else:
        gravity_scale = 0.0
        coulomb, viscous, bias = parameters
    # Raw condition numbers depend on column units; normalize columns before
    # diagnosing collinearity (e.g. Coulomb and viscous at one speed magnitude).
    norms = np.linalg.norm(design, axis=0)
    normalized_condition = float(np.linalg.cond(design / np.where(norms > 0, norms, 1)))
    positive = sum(sample[1][joint] > 0 for sample, _ in rows)
    negative = sum(sample[1][joint] < 0 for sample, _ in rows)
    flags = []
    if rank < design.shape[1]:
        flags.append('rank_deficient')
    if normalized_condition > 1000:
        flags.append('ill_conditioned')
    if min(positive, negative) < 10:
        flags.append('insufficient_bidirectional_samples')
    if coulomb < 0:
        flags.append('negative_coulomb')
    if viscous < 0:
        flags.append('negative_viscous')
    return {
        'gravity_identifiable': bool(gravity_identifiable),
        'gravity_scale_raw_per_nm': float(gravity_scale),
        'coulomb_raw': float(coulomb),
        'viscous_raw_per_rad_s': float(viscous),
        'bias_raw': float(bias),
        'rmse_raw': float(math.sqrt(np.mean(residual ** 2))),
        'r_squared': r_squared,
        'samples': len(rows),
        'rank': int(rank),
        'condition_number': float(singular[0] / singular[-1]) if singular[-1] > 0.0 else float('inf'),
        'normalized_condition_number': normalized_condition,
        'positive_velocity_samples': positive,
        'negative_velocity_samples': negative,
        'fit_quality': 'rejected' if flags else 'requires_independent_validation',
        'quality_flags': flags,
    }


def moving_joints(samples, velocity_min):
    """Only fit joints with sufficient bidirectional excitation in this dataset."""
    selected = []
    for name in ARM_JOINTS:
        velocity = np.array([sample[1][name] for sample in samples])
        position = np.array([sample[0][name] for sample in samples])
        if (np.count_nonzero(velocity > velocity_min) >= 10 and
                np.count_nonzero(velocity < -velocity_min) >= 10 and
                np.ptp(position) >= 0.005):
            selected.append(name)
    if not selected:
        raise RuntimeError('no joint has enough bidirectional motion; record a longer calibration run')
    return selected


def evaluate_joint(samples, gravity_values, joint, parameters, deadzone, velocity_min):
    rows = [(sample, gravity) for sample, gravity in zip(samples, gravity_values)
            if abs(sample[1][joint]) >= velocity_min]
    if len(rows) < 20:
        raise RuntimeError('%s has insufficient moving samples for evaluation' % joint)
    target = np.array([sample[2][joint] for sample, _ in rows])
    predicted = np.array([
        parameters['gravity_scale_raw_per_nm'] * gravity[joint] +
        parameters['coulomb_raw'] * math.tanh(sample[1][joint] / deadzone) +
        parameters['viscous_raw_per_rad_s'] * sample[1][joint] + parameters['bias_raw']
        for sample, gravity in rows])
    residual = target - predicted
    variance = np.sum((target - target.mean()) ** 2)
    return {'samples': len(rows), 'rmse_raw': float(np.sqrt(np.mean(residual ** 2))),
            'r_squared': float(1 - np.sum(residual ** 2) / variance) if variance > 0 else None}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', help='CSV from multisine_identification.py')
    parser.add_argument('--output', default='identified_gravity_friction.yaml')
    parser.add_argument('--deadzone', type=float, default=0.02,
                        help='tanh friction deadzone in rad/s (default: 0.02)')
    parser.add_argument('--velocity-min', type=float, default=0.005,
                        help='discard samples below this absolute velocity in rad/s (default: 0.005)')
    parser.add_argument('--stride', type=int, default=10,
                        help='use every Nth CSV row for gravity evaluation (default: 10)')
    parser.add_argument('--velocity-source', choices=['position', 'reported'], default='position',
                        help='derive velocity from timestamped positions (default), or use the reported motor value')
    parser.add_argument('--velocity-window', type=float, default=1.0,
                        help='centered quadratic fit window in seconds (default: 1.0)')
    parser.add_argument('--max-gap', type=float, default=0.2,
                        help='never derive velocity across a gap larger than this many seconds')
    parser.add_argument('--joints', nargs='+', default=['joint2', 'joint3'])
    parser.add_argument('--auto-joints', action='store_true',
                        help='fit only joints with enough bidirectional motion')
    parser.add_argument('--evaluate', metavar='MODEL_YAML',
                        help='evaluate saved coefficients on another dataset without refitting')
    parser.add_argument('--xacro', default=None, help='override the om6dof URDF Xacro path')
    parser.add_argument('--model', choices=['v1', 'v2'],
                        help='explicit model selection; required unless --xacro is given')
    args = parser.parse_args()
    if not args.model and not args.xacro:
        parser.error('select --model v1/v2 or --xacro; never infer a dataset model')
    if args.model and args.xacro:
        parser.error('use --model or --xacro, not both')
    if args.deadzone <= 0.0 or args.velocity_min < 0.0 or args.stride <= 0:
        parser.error('--deadzone/--stride must be positive and --velocity-min cannot be negative')
    invalid = set(args.joints) - set(ARM_JOINTS)
    if invalid:
        parser.error('unknown arm joints: %s' % ', '.join(sorted(invalid)))
    xacro = args.xacro
    if xacro is None:
        filename = 'om6dof_v2.urdf.xacro' if args.model == 'v2' else 'om6dof.urdf.xacro'
        xacro = get_package_share_directory('om6dof_description') + '/urdf/' + filename
    urdf_text = expand_xacro(xacro)
    urdf_hash = hashlib.sha256(urdf_text.encode()).hexdigest()
    csv_hash = hashlib.sha256(Path(args.csv).read_bytes()).hexdigest()
    candidate = None
    if args.evaluate:
        with open(args.evaluate, encoding='utf-8') as handle:
            candidate = yaml.safe_load(handle)
        if candidate.get('expanded_urdf_sha256') != urdf_hash:
            parser.error('evaluation URDF differs from the fitted model')
        if candidate.get('source_csv_sha256') == csv_hash:
            parser.error('choose an independent recording, not the training dataset')
        args.joints = list(candidate['joints'])
        args.deadzone = candidate['deadzone_rad_s']
        args.velocity_min = candidate['velocity_min_rad_s']
        processing = candidate.get('velocity_estimation', {'source': 'reported'})
        args.velocity_source = processing['source']
        args.velocity_window = processing.get('window_seconds', 1.0)
        args.max_gap = processing.get('max_gap_seconds', 0.2)
    all_samples, velocity_metadata = load_samples(
        args.csv, args.velocity_source, args.velocity_window, args.max_gap, return_metadata=True)
    samples = all_samples[::args.stride]
    if candidate is None and args.auto_joints:
        args.joints = moving_joints(samples, args.velocity_min)
    model = UrdfGravity(urdf_text)
    print('computing URDF gravity for %d samples...' % len(samples))
    gravity_values = [model.gravity(position, args.joints) for position, _, _ in samples]
    result = {
        'calibration_status': 'candidate_unvalidated',
        'model': args.model or 'custom',
        'model_xacro': str(Path(xacro).resolve()),
        'expanded_urdf_sha256': urdf_hash,
        'source_csv_sha256': csv_hash,
        'source_csv': args.csv,
        'source_samples': velocity_metadata['source_rows'],
        'velocity_estimation': velocity_metadata,
        'fit_samples_after_stride': len(samples),
        'stride': args.stride,
        'units': {
            'gravity_scale_raw_per_nm': 'raw Present Current per Nm',
            'coulomb_raw': 'raw Present Current',
            'viscous_raw_per_rad_s': 'raw Present Current per rad/s',
            'bias_raw': 'raw Present Current',
        },
        'deadzone_rad_s': args.deadzone,
        'velocity_min_rad_s': args.velocity_min,
        'joints': {},
    }
    for joint in args.joints:
        if candidate is None:
            result['joints'][joint] = fit_joint(
                samples, gravity_values, joint, args.deadzone, args.velocity_min)
        else:
            result['joints'][joint] = evaluate_joint(
                samples, gravity_values, joint, candidate['joints'][joint],
                args.deadzone, args.velocity_min)
    if candidate is not None:
        result['calibration_status'] = 'evaluation_only_not_deployed'
        result['evaluated_model'] = str(Path(args.evaluate).resolve())
    with open(args.output, 'w', encoding='utf-8') as file:
        yaml.safe_dump(result, file, sort_keys=False)
    print(yaml.safe_dump(result, sort_keys=False))
    print('wrote %s' % args.output)


if __name__ == '__main__':
    main()
