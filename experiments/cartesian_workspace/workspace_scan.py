#!/usr/bin/env python3
"""Offline, sampled Cartesian workspace experiment. Never publishes ROS commands."""

import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial import cKDTree

# Run from the source checkout without rebuilding or starting any ROS node.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'om6dof_controller'))
from om6dof_controller.ik_solver import IKSolver, render_chain_urdf
from om6dof_controller.control_math import cartesian_rpy_to_tip_rotation, rotation_error


COLORS = {
    'pose_found': '#15976b',
    'pose_near_singular': '#e5a21a',
    'orientation_unresolved': '#397ac6',
    'collision_candidates_only': '#9658ad',
    'position_unresolved': '#d15b59',
}
LABELS = {
    'pose_found': 'Pose found (tested orientation)',
    'pose_near_singular': 'Pose found; best tested solution near singular',
    'orientation_unresolved': 'Position found; orientation unresolved',
    'collision_candidates_only': 'Only colliding position candidates found',
    'position_unresolved': 'Position unresolved (NOT proof of impossibility)',
}


def grid_points(radius, spacing):
    """All grid centres inside a sphere, including negative Z and the origin."""
    n = int(math.floor(radius / spacing))
    axis = np.arange(-n, n + 1) * spacing
    points = np.array(np.meshgrid(axis, axis, axis, indexing='ij')).reshape(3, -1).T
    return points[np.linalg.norm(points, axis=1) <= radius + 1e-12]


def conditioning(ik, q, length_scale):
    """Dimensionless sigma_min/sigma_max; linear Jacobian rows scaled by length."""
    jacobian = ik.jacobian(q).copy()
    jacobian[:3] /= length_scale
    singular_values = np.linalg.svd(jacobian, compute_uv=False)
    return float(singular_values[-1] / max(singular_values[0], 1e-15))


def candidate(ik, q, position, rotation, args):
    q = np.asarray(q, dtype=float)
    if q.shape != (6,) or not np.all(np.isfinite(q)):
        return None
    if np.any(q < ik.q_min - 1e-10) or np.any(q > ik.q_max + 1e-10):
        return None
    xyz, actual_rotation = ik.fk_pose(q)
    ep = float(np.linalg.norm(xyz - position))
    er = float(np.linalg.norm(rotation_error(rotation, actual_rotation)))
    if not math.isfinite(ep + er):
        return None
    return {'q': q, 'position_error_mm': ep * 1000,
            'orientation_error_deg': math.degrees(er),
            'collision': bool(ik.self_collides(q, args.collision_radius_mm / 1000))}


def scan_point(ik, position, seeds, rotation, args):
    """Keep numerical failure, endpoint collision and orientation failure distinct."""
    position_solutions, pose_solutions = [], []
    collision_seen = False
    best_error = math.inf
    for seed in seeds:
        q, _ = ik.solve_position_ik(seed, position, max_iter=args.iterations)
        result = candidate(ik, q, position, rotation, args)
        if result is None:
            continue
        best_error = min(best_error, result['position_error_mm'])
        if result['position_error_mm'] <= args.position_tolerance_mm:
            if result['collision']:
                collision_seen = True
            else:
                position_solutions.append(result)

    # Free-position solutions are useful extra seeds, not orientation evidence.
    pose_seeds = [result['q'] for result in position_solutions[:2]] + list(seeds)
    for seed in pose_seeds:
        q, _ = ik.solve_pose_ik(seed, position, rotation, max_iter=args.iterations)
        result = candidate(ik, q, position, rotation, args)
        if result is None:
            continue
        best_error = min(best_error, result['position_error_mm'])
        if result['position_error_mm'] <= args.position_tolerance_mm:
            if result['collision']:
                collision_seen = True
            else:
                position_solutions.append(result)
                if result['orientation_error_deg'] <= args.orientation_tolerance_deg:
                    result['rcond'] = conditioning(ik, result['q'], args.length_scale_mm / 1000)
                    pose_solutions.append(result)

    row = dict(zip(('x_mm', 'y_mm', 'z_mm'), position * 1000))
    row.update(position_found=bool(position_solutions), pose_found=bool(pose_solutions),
               best_position_error_mm=best_error if math.isfinite(best_error) else None,
               rcond=None, joint_margin_deg=None, orientation_error_deg=None,
               position_error_mm=None)
    selected = None
    if pose_solutions:
        selected = max(pose_solutions, key=lambda result: result['rcond'])
        row['status'] = ('pose_near_singular' if selected['rcond'] < args.singular_threshold
                         else 'pose_found')
        row['rcond'] = selected['rcond']
    elif position_solutions:
        row['status'] = 'orientation_unresolved'
        selected = min(position_solutions, key=lambda result: result['position_error_mm'])
    else:
        row['status'] = 'collision_candidates_only' if collision_seen else 'position_unresolved'
    if selected is not None:
        q = selected['q']
        row['joint_margin_deg'] = math.degrees(float(np.min(np.minimum(q-ik.q_min, ik.q_max-q))))
        row['position_error_mm'] = selected['position_error_mm']
        row['orientation_error_deg'] = selected['orientation_error_deg']
    for i in range(6):
        row[f'q{i+1}_rad'] = float(selected['q'][i]) if selected is not None else None
    return row


def plot_results(rows, output, spacing_mm, orientation_label='', skeleton_mm=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    xyz = np.array([[r['x_mm'], r['y_mm'], r['z_mm']] for r in rows])
    status = np.array([r['status'] for r in rows])
    free = np.array([r['position_found'] for r in rows])
    fig = plt.figure(figsize=(15, 8))
    for i, title in enumerate(('Position reachability (orientation free)', 'Tested gripper orientation'), 1):
        ax = fig.add_subplot(1, 2, i, projection='3d')
        colors = np.where(free, '#15976b', '#d15b59') if i == 1 else [COLORS[s] for s in status]
        ax.scatter(*xyz.T, c=colors, s=9, alpha=.40, depthshade=False)
        ax.scatter([0], [0], [0], c='black', marker='x', s=45)
        if skeleton_mm is not None:
            ax.plot(*np.asarray(skeleton_mm).T, color='black', linewidth=2,
                    marker='o', markersize=3, label='URDF skeleton at READY')
        ax.set(xlabel='X (mm)', ylabel='Y (mm)', zlabel='Z (mm)', title=title)
        extent = max(float(np.abs(xyz).max()), spacing_mm)
        ax.set_xlim(-extent, extent)
        ax.set_ylim(-extent, extent)
        ax.set_zlim(-extent, extent)
        ax.set_box_aspect((1, 1, 1))
        if i == 1:
            ax.legend(handles=[
                Line2D([0], [0], marker='o', linestyle='', color='#15976b', label='Position found'),
                Line2D([0], [0], marker='o', linestyle='', color='#d15b59', label='Position unresolved'),
            ], loc='upper left', fontsize=8)
    handles = [Line2D([0], [0], marker='o', linestyle='', color=COLORS[s], label=LABELS[s]) for s in COLORS]
    fig.legend(handles=handles, loc='lower center', ncol=2, fontsize=9)
    skeleton_label = '; black line = URDF at READY' if skeleton_mm is not None else ''
    fig.suptitle(f'OM6DOF sampled workspace — {len(rows)} points, {spacing_mm:g} mm grid\n'
                 f'{orientation_label}{skeleton_label}\n'
                 'Red/purple are unresolved, not proven mechanical dead zones')
    fig.subplots_adjust(bottom=.18, top=.84)
    fig.savefig(output / 'workspace_3d.png', dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(1, 3, figsize=(16, 6))
    for ax, (a, b, normal) in zip(axes, ((0, 1, 2), (0, 2, 1), (1, 2, 0))):
        mask = np.abs(xyz[:, normal]) < spacing_mm / 4
        ax.scatter(xyz[mask, a], xyz[mask, b], c=[COLORS[s] for s in status[mask]], marker='s', s=50)
        ax.set(xlabel='XYZ'[a]+' (mm)', ylabel='XYZ'[b]+' (mm)',
               title='XYZ'[normal]+' = 0 mm', aspect='equal')
        ax.grid(alpha=.2)
    fig.legend(handles=handles, loc='lower center', ncol=2, fontsize=9)
    fig.suptitle('Central cross sections — discrete samples only\n' + orientation_label)
    fig.tight_layout(rect=(0, .2, 1, .94))
    fig.savefig(output / 'workspace_slices.png', dpi=150)
    plt.close(fig)


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, default=Path('/tmp') / datetime.now().strftime('om6dof-workspace-%Y%m%d-%H%M%S'))
    p.add_argument('--spacing-mm', type=float, default=60)
    p.add_argument('--radius-mm', type=float, help='Sphere centred on base frame; default conservative URDF chain-length bound')
    p.add_argument('--samples', type=int, default=2000, help='FK seed-bank size, not number of Cartesian test points')
    p.add_argument('--seeds', type=int, default=4, help='Nearest FK seeds per grid point plus zero/READY')
    p.add_argument('--iterations', type=int, default=80)
    p.add_argument('--seed', type=int, default=42)
    p.add_argument('--orientation', choices=('radial', 'fixed'), default='radial')
    p.add_argument('--rpy-deg', type=float, nargs=3, default=[0, 0, 0], help='Cartesian target gripper frame; radial adds atan2(y,x) to yaw')
    p.add_argument('--joint-margin-rad', type=float, default=.02, help='Use 0 for hard URDF mechanical range')
    p.add_argument('--collision-radius-mm', type=float, default=25)
    p.add_argument('--position-tolerance-mm', type=float, default=1)
    p.add_argument('--orientation-tolerance-deg', type=float, default=.5)
    p.add_argument('--singular-threshold', type=float, default=.01)
    p.add_argument('--length-scale-mm', type=float, default=300)
    p.add_argument('--base-link', default='world')
    p.add_argument('--tip-link', default='end_effector_link')
    p.add_argument('--urdf-pkg', default='om6dof_description')
    return p


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    positive = [args.spacing_mm, args.samples, args.seeds, args.iterations,
                args.collision_radius_mm, args.position_tolerance_mm,
                args.orientation_tolerance_deg, args.length_scale_mm, args.singular_threshold]
    if (not all(math.isfinite(v) and v > 0 for v in positive)
            or not math.isfinite(args.joint_margin_rad) or args.joint_margin_rad < 0
            or not all(math.isfinite(v) for v in args.rpy_deg)
            or (args.radius_mm is not None and (not math.isfinite(args.radius_mm) or args.radius_mm <= 0))):
        p.error('Lengths, counts and tolerances must be finite/positive; joint margin must be nonnegative.')
    if args.output.exists():
        p.error('Output already exists; choose a new directory to preserve prior experiments.')

    started = time.monotonic()
    ik = IKSolver(base_link=args.base_link, tip_link=args.tip_link, urdf_pkg=args.urdf_pkg)
    urdf = render_chain_urdf(args.urdf_pkg)
    hard_lower, hard_upper = ik.q_min.copy(), ik.q_max.copy()
    ik.q_min += args.joint_margin_rad
    ik.q_max -= args.joint_margin_rad
    if np.any(ik.q_min >= ik.q_max):
        p.error('Joint margin removes the entire joint range.')
    # Sum segment translation norms is a conservative bound for this revolute chain.
    skeleton = np.asarray(ik.link_points(np.zeros(6)))
    bound = float(np.linalg.norm(np.diff(skeleton, axis=0), axis=1).sum())
    radius = bound if args.radius_mm is None else args.radius_mm / 1000
    if radius / (args.spacing_mm / 1000) > 100:
        p.error('Grid too large (>100 steps per radius); start with a coarser scan.')
    points = grid_points(radius, args.spacing_mm / 1000)
    args.output.mkdir(parents=True)
    (args.output / 'model.urdf').write_text(urdf)
    rng = np.random.default_rng(args.seed)
    bank = rng.uniform(ik.q_min, ik.q_max, size=(args.samples, 6))
    xyz = np.array([ik.fk_pose(q)[0] for q in bank])
    tree = cKDTree(xyz)
    np.savez_compressed(args.output / 'seed_bank.npz', joints=bank, positions_m=xyz)
    print(f'Scanning {len(points)} grid points; radius={radius*1000:.1f} mm; offline only.', flush=True)
    rows = []
    ready = np.clip([0, -.6806, 1.3613, 0, .8901, 0], ik.q_min, ik.q_max)
    zero = np.clip(np.zeros(6), ik.q_min, ik.q_max)
    with (args.output / 'points.csv').open('w', newline='') as stream:
        writer = None
        for index, point in enumerate(points):
            _, indices = tree.query(point, k=min(args.seeds, len(bank)))
            seeds = [bank[i] for i in np.atleast_1d(indices)] + [ready, zero]
            rpy = np.radians(args.rpy_deg)
            if args.orientation == 'radial':
                rpy[2] += math.atan2(point[1], point[0])
            rotation = cartesian_rpy_to_tip_rotation(*rpy)
            row = scan_point(ik, point, seeds, rotation, args)
            row.update(zip(('target_roll_deg', 'target_pitch_deg', 'target_yaw_deg'), np.degrees(rpy)))
            rows.append(row)
            if writer is None:
                writer = csv.DictWriter(stream, fieldnames=list(row))
                writer.writeheader()
            writer.writerow(row)
            if (index + 1) % 25 == 0 or index == len(points)-1:
                stream.flush()
                print(f'{index+1}/{len(points)} points, {time.monotonic()-started:.1f}s', flush=True)
    summary = {
        'completed_utc': datetime.now(timezone.utc).isoformat(),
        'arguments': {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
        'radius_mm': radius*1000, 'conservative_urdf_bound_mm': bound*1000,
        'urdf_sha256': hashlib.sha256(urdf.encode()).hexdigest(),
        'hard_lower_rad': hard_lower.tolist(), 'hard_upper_rad': hard_upper.tolist(),
        'effective_lower_rad': ik.q_min.tolist(), 'effective_upper_rad': ik.q_max.tolist(),
        'points': len(rows), 'counts': dict(Counter(r['status'] for r in rows)),
        'position_found': sum(r['position_found'] for r in rows),
        'pose_found': sum(r['pose_found'] for r in rows),
        'elapsed_seconds': time.monotonic()-started,
        'limitations': [
            'Finite grid and finite multi-start IK: unresolved is not proven unreachable.',
            'One orientation per point; not an exhaustive SO(3) orientation study.',
            'Endpoint capsule self-collision only; no swept path, table or external obstacles.',
            'No dynamics, torque, payload, compliance, calibration or physical accuracy measurement.',
            'Singularity class describes the best tested pose candidate, not all possible configurations.',
            'Full sphere includes negative Z; installed base/table clearance is not modelled.',
        ],
    }
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False))
    plot_results(rows, args.output, args.spacing_mm,
                 f'{args.orientation} gripper orientation; RPY offset {args.rpy_deg} deg; frame {args.base_link}',
                 ik.link_points(ready) * 1000)
    print(json.dumps({'output': str(args.output), 'counts': summary['counts']}, indent=2), flush=True)


if __name__ == '__main__':
    main()
