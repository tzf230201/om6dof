#!/usr/bin/env python3
"""Offline SI2017 model comparison; no ROS node, motion, or deployment.

Compare sign/tanh friction on identical moving samples, with nominal gravity
or a URDF mass/first-moment gravity regressor. Raw-current fits are independent
per joint: their gravity coefficients are NOT physical masses/CoMs or a shared
robot parameter estimate until signed current-to-joint-torque gains are known.
"""
import argparse
import hashlib
from pathlib import Path

import numpy as np
import yaml

from fit_gravity_friction import (
    ARM_JOINTS, UrdfGravity, axis_rotation, expand_xacro, load_samples,
    rigid_transform, get_package_share_directory,
)


class GravityRegressor(UrdfGravity):
    """dU/dq = Y_g(q) [m, m*cx, m*cy, m*cz] for every inertial link.

Analytic joint-axis Jacobians avoid finite-difference noise in rank diagnosis.
Fixed branches/payloads are included; unrecorded gripper joints stay at zero.
Inertial-origin rotation does not rotate its xyz CoM (xyz is in link frame).
"""
    def __init__(self, xml):
        super().__init__(xml)
        self.inertial_links = [link for link in self.robot.links if link.inertial is not None]
        self.columns = [f'{link.name}:{term}' for link in self.inertial_links
                        for term in ('mass', 'first_moment_x', 'first_moment_y', 'first_moment_z')]
        self.nominal = np.array([x for link in self.inertial_links
            for x in (float(link.inertial.mass), *(
                float(link.inertial.mass) * np.array(link.inertial.origin.xyz
                    if link.inertial.origin is not None else (0., 0., 0.))))])
        if any(j.mimic is not None and j.mimic.joint in ARM_JOINTS for j in self.robot.joints):
            raise ValueError('Arm-driven mimic joints need coupled Jacobians; not supported')

    def matrix(self, positions, names):
        transforms = {self.root: np.eye(4)}
        ancestors = {self.root: []}
        axes = {}
        pending = [self.root]
        while pending:
            parent = pending.pop()
            for joint in self.children[parent]:
                origin = joint.origin
                transform = transforms[parent] @ rigid_transform(
                    origin.xyz if origin is not None else (0., 0., 0.),
                    origin.rpy if origin is not None else (0., 0., 0.))
                chain = list(ancestors[parent])
                if joint.type in ('continuous', 'revolute', 'prismatic'):
                    axis = np.asarray(joint.axis, dtype=float)
                    if np.linalg.norm(axis) == 0:
                        raise ValueError('Zero joint axis')
                    axis = axis / np.linalg.norm(axis)
                    axes[joint.name] = (joint.type, transform[:3, :3] @ axis,
                                        transform[:3, 3].copy())
                    chain.append(joint.name)
                    value = self._joint_value(joint, positions)
                    if joint.type == 'prismatic':
                        translation = np.eye(4)
                        translation[:3, 3] = axis * value
                        transform = transform @ translation
                    else:
                        transform = transform @ axis_rotation(axis, value)
                elif joint.type != 'fixed':
                    raise ValueError('Unsupported joint type: ' + joint.type)
                transforms[joint.child] = transform
                ancestors[joint.child] = chain
                pending.append(joint.child)
        output = np.zeros((len(names), len(self.columns)))
        for k, link in enumerate(self.inertial_links):
            transform = transforms[link.name]
            for i, name in enumerate(names):
                if name not in ancestors[link.name]:
                    continue
                kind, axis, pivot = axes[name]
                if kind == 'prismatic':
                    output[i, 4*k] = 9.80665 * axis[2]
                else:
                    output[i, 4*k] = 9.80665 * np.cross(axis, transform[:3, 3]-pivot)[2]
                    output[i, 4*k+1:4*k+4] = 9.80665 * np.cross(axis, transform[:3, :3].T)[:, 2]
        return output


def solve(x, y, rcond=1e-6):
    """Unit-column SVD; report non-identifiability, never hide it with clipping."""
    scale = np.linalg.norm(x, axis=0)
    active = scale > 1e-10
    if not np.any(active):
        raise ValueError('Empty regressor')
    normalized = x[:, active] / scale[active]
    p, _, rank, singular = np.linalg.lstsq(normalized, y, rcond=rcond)
    coefficients = np.zeros(x.shape[1])
    coefficients[active] = p / scale[active]
    return coefficients, {
        'columns': x.shape[1], 'active_columns': int(active.sum()), 'rank': int(rank),
        'rcond': rcond, 'singular_values_unit_columns': singular.tolist(),
        'active_coefficients_unique': bool(rank == active.sum()),
        'zero_column_indices': np.flatnonzero(~active).tolist(),
    }


def metrics(x, y, p):
    residual = y - x @ p
    variance = np.sum((y-y.mean())**2)
    return {'samples': len(y), 'rmse_raw': float(np.sqrt(np.mean(residual**2))),
            'r_squared': float(1-np.sum(residual**2)/variance) if variance > 0 else None}


def dataset(path, model, joint, source, window, minimum):
    samples, metadata = load_samples(path, source, window, return_metadata=True)
    samples = [s for s in samples if abs(s[1][joint]) >= minimum]
    v = np.array([s[1][joint] for s in samples])
    if len(v) < 20 or min(np.sum(v > 0), np.sum(v < 0)) < 10:
        raise ValueError('Insufficient bidirectional moving samples for ' + joint)
    g = np.array([model.matrix(s[0], [joint])[0] for s in samples])
    return g, v, np.array([s[2][joint] for s in samples]), metadata


def compare(training, validation, model, deadzone):
    g, v, y, _ = training
    ge, ve, ye, _ = validation
    results = []
    for gravity_model in ('nominal_scale', 'link_first_moments'):
        basis = g if gravity_model == 'link_first_moments' else (g @ model.nominal)[:, None]
        test_basis = ge if gravity_model == 'link_first_moments' else (ge @ model.nominal)[:, None]
        labels = model.columns if gravity_model == 'link_first_moments' else ['nominal_gravity_scale']
        for friction in ('sign', 'tanh'):
            c = np.sign(v) if friction == 'sign' else np.tanh(v/deadzone)
            ce = np.sign(ve) if friction == 'sign' else np.tanh(ve/deadzone)
            x = np.column_stack((basis, c, v, np.ones(len(v))))
            xe = np.column_stack((test_basis, ce, ve, np.ones(len(ve))))
            p, info = solve(x, y)
            flags = []
            if p[-3] < 0:
                flags.append('negative_coulomb_under_assumed_current_polarity')
            if p[-2] < 0:
                flags.append('negative_viscous_under_assumed_current_polarity')
            if not info['active_coefficients_unique']:
                flags.append('nonunique_active_coefficients')
            results.append({
                'gravity_model': gravity_model, 'friction_model': friction,
                'parameter_names': [*labels, 'coulomb_raw', 'viscous_raw_per_rad_s', 'bias_raw'],
                'coefficients': p.tolist(), 'identifiability': info,
                'training': metrics(x, y, p), 'independent_recording': metrics(xe, ye, p),
                'quality_flags': flags, 'deployable': False,
                'interpretation': 'Current-domain diagnostic; no physical mass/CoM or calibrated Nm inferred.'})
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('training', type=Path)
    parser.add_argument('validation', type=Path)
    parser.add_argument('--joint', choices=ARM_JOINTS, required=True)
    parser.add_argument('--model', choices=['v2'], required=True)
    parser.add_argument('--velocity-source', choices=['position', 'reported'], default='position')
    parser.add_argument('--velocity-window', type=float, default=1.)
    parser.add_argument('--velocity-min', type=float, default=.005)
    parser.add_argument('--deadzone', type=float, default=.005)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not all(np.isfinite(v) and v > 0 for v in
               (args.velocity_window, args.velocity_min, args.deadzone)):
        parser.error('window, velocity-min and deadzone must be finite and positive')
    if args.output.exists():
        parser.error('Output exists; choose a new report path')
    hashes = [hashlib.sha256(path.read_bytes()).hexdigest() for path in (args.training, args.validation)]
    if hashes[0] == hashes[1]:
        parser.error('Validation must be a different recording')
    xacro = Path(get_package_share_directory('om6dof_description')) / 'urdf/om6dof_v2.urdf.xacro'
    xml = expand_xacro(str(xacro))
    model = GravityRegressor(xml)
    training = dataset(args.training, model, args.joint, args.velocity_source,
                       args.velocity_window, args.velocity_min)
    validation = dataset(args.validation, model, args.joint, args.velocity_source,
                         args.velocity_window, args.velocity_min)
    results = compare(training, validation, model, args.deadzone)
    report = {
        'status': 'offline_diagnostic_not_deployable', 'joint': args.joint,
        'source_csvs': [str(p.resolve()) for p in (args.training, args.validation)],
        'csv_sha256': hashes, 'expanded_urdf_sha256': hashlib.sha256(xml.encode()).hexdigest(),
        'model_xacro': str(xacro.resolve()), 'preprocessing': [training[3], validation[3]],
        'velocity_min_rad_s': args.velocity_min, 'tanh_deadzone_rad_s': args.deadzone,
        'assumptions': ['Same physical setup for both recordings; no runtime URDF snapshot in CSV.',
                        'Gravity along base -Z; unrecorded gripper at zero.',
                        'Current sign agrees with generalized torque: NOT physically verified.',
                        'Unknown current/torque gain prevents shared physical parameter identification.',
                        'Full link basis is redundant; SVD solution is diagnostic, not identified masses.',
                        'No inertia/Coriolis model; no proof of collision or leader-arm safety.'],
        'results': results,
    }
    with args.output.open('x') as handle:
        yaml.safe_dump(report, handle, sort_keys=False)
    for result in results:
        print(result['gravity_model'], result['friction_model'],
              'C/B/bias=', result['coefficients'][-3:],
              'heldout=', result['independent_recording'], 'flags=', result['quality_flags'])
    print('Wrote diagnostic only:', args.output)


if __name__ == '__main__':
    main()
