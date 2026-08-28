#!/usr/bin/env python3
"""Fit gravity scale and Coulomb/viscous friction from a multisine CSV.

The input is produced by ``multisine_identification.py``.  ``effort`` is the
raw Dynamixel Present Current value, so the resulting friction values are in
that same unit; do not copy them into an Nm controller configuration until a
current-to-joint-torque scale has been calibrated.
"""

import argparse
import csv
import math
import subprocess
from collections import defaultdict

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


def load_samples(path):
    with open(path, newline='', encoding='utf-8') as file:
        rows = list(csv.DictReader(file))
    if not rows:
        raise RuntimeError('CSV has no data rows: %s' % path)
    required = [f'{name}_{field}' for name in ARM_JOINTS
                for field in ('position_rad', 'velocity_rad_s', 'effort_raw')]
    missing = [field for field in required if field not in rows[0]]
    if missing:
        raise RuntimeError('CSV does not look like multisine output; missing: %s' % ', '.join(missing))
    samples = []
    for row in rows:
        positions = {name: float(row[f'{name}_position_rad']) for name in ARM_JOINTS}
        velocity = {name: float(row[f'{name}_velocity_rad_s']) for name in ARM_JOINTS}
        effort = {name: float(row[f'{name}_effort_raw']) for name in ARM_JOINTS}
        if all(math.isfinite(value) for value in (*positions.values(), *velocity.values(), *effort.values())):
            samples.append((positions, velocity, effort))
    if not samples:
        raise RuntimeError('CSV contains no finite samples')
    return samples


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
    r_squared = 1.0 - np.dot(residual, residual) / np.dot(centered, centered)
    if gravity_identifiable:
        gravity_scale, coulomb, viscous, bias = parameters
    else:
        gravity_scale = 0.0
        coulomb, viscous, bias = parameters
    return {
        'gravity_identifiable': bool(gravity_identifiable),
        'gravity_scale_raw_per_nm': float(gravity_scale),
        'coulomb_raw': float(coulomb),
        'viscous_raw_per_rad_s': float(viscous),
        'bias_raw': float(bias),
        'rmse_raw': float(math.sqrt(np.mean(residual ** 2))),
        'r_squared': float(r_squared),
        'samples': len(rows),
        'rank': int(rank),
        'condition_number': float(singular[0] / singular[-1]) if singular[-1] > 0.0 else float('inf'),
    }


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
    parser.add_argument('--joints', nargs='+', default=['joint2', 'joint3'])
    parser.add_argument('--xacro', default=None, help='override the om6dof URDF Xacro path')
    args = parser.parse_args()
    if args.deadzone <= 0.0 or args.velocity_min < 0.0 or args.stride <= 0:
        parser.error('--deadzone/--stride must be positive and --velocity-min cannot be negative')
    invalid = set(args.joints) - set(ARM_JOINTS)
    if invalid:
        parser.error('unknown arm joints: %s' % ', '.join(sorted(invalid)))
    xacro = args.xacro
    if xacro is None:
        xacro = get_package_share_directory('om6dof_description') + '/urdf/om6dof.urdf.xacro'
    all_samples = load_samples(args.csv)
    samples = all_samples[::args.stride]
    model = UrdfGravity(expand_xacro(xacro))
    print('computing URDF gravity for %d samples...' % len(samples))
    gravity_values = [model.gravity(position) for position, _, _ in samples]
    result = {
        'source_csv': args.csv,
        'source_samples': len(all_samples),
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
        result['joints'][joint] = fit_joint(
            samples, gravity_values, joint, args.deadzone, args.velocity_min)
    with open(args.output, 'w', encoding='utf-8') as file:
        yaml.safe_dump(result, file, sort_keys=False)
    print(yaml.safe_dump(result, sort_keys=False))
    print('wrote %s' % args.output)


if __name__ == '__main__':
    main()
