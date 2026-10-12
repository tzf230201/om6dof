#!/usr/bin/env python3
"""Offline signed current-to-torque candidate from independent reference data.

Reference torque must come from a documented measurement or known static load
with a correct lever arm, not from the URDF torque being fitted. It must be the
total required actuator joint torque, including gravity, not merely an added
external load. Static friction/hysteresis remain confounders. This program
does not acquire measurements, send motor commands, or deploy calibration.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np

FIELDS = ('joint', 'reference_id', 'reference_torque_nm', 'effort_raw', 'velocity_rad_s')


def load_reference(path, joint, max_velocity):
    with Path(path).open(newline='') as handle:
        reader = csv.DictReader(handle)
        if not set(FIELDS).issubset(reader.fieldnames or []):
            raise ValueError('Required columns: ' + ','.join(FIELDS))
        rows = [r for r in reader if r['joint'] == joint]
    if len(rows) < 6:
        raise ValueError('Need at least six independent reference points for ' + joint)
    ids = [r['reference_id'].strip() for r in rows]
    if any(not value for value in ids) or len(set(ids)) != len(ids):
        raise ValueError('Use one averaged observation per unique reference_id, not repeated samples')
    values = np.array([[float(r[f]) for f in FIELDS[2:]] for r in rows])
    if not np.all(np.isfinite(values)):
        raise ValueError('Nonfinite reference data')
    torque, current, velocity = values.T
    if np.max(np.abs(velocity)) > max_velocity:
        raise ValueError('Reference not static within the selected velocity bound')
    if min(np.count_nonzero(torque > 0), np.count_nonzero(torque < 0)) < 3:
        raise ValueError('Need at least three positive and three negative reference torques')
    if len(np.unique(torque)) < 4 or np.ptp(current) < 1e-6:
        raise ValueError('Insufficient independent load/current variation')
    return torque, current, ids


def fit_reference(training, validation):
    torque, current, ids = training
    test_torque, test_current, test_ids = validation
    if set(ids) & set(test_ids):
        raise ValueError('Training and validation reference IDs overlap')
    center, scale = current.mean(), current.std()
    if scale < 1e-6:
        raise ValueError('Current has no useful variation')
    x = np.column_stack(((current-center)/scale, np.ones(len(current))))
    p, _, rank, _ = np.linalg.lstsq(x, torque, rcond=None)
    if rank != 2:
        raise ValueError('Rank-deficient current-to-torque fit')
    gain = float(p[0]/scale)
    bias = float(p[1]-gain*center)
    def metrics(t, i):
        residual = t-(gain*i+bias)
        variance = np.sum((t-t.mean())**2)
        return {'samples': len(t), 'rmse_nm': float(np.sqrt(np.mean(residual**2))),
                'max_abs_error_nm': float(np.max(np.abs(residual))),
                'r_squared': float(1-np.sum(residual**2)/variance) if variance > 0 else None}
    flags = ['requires_physical_reference_and_hysteresis_review']
    if abs(gain) < 1e-12:
        flags.append('near_zero_gain_no_polarity_identified')
    if test_current.min() < current.min() or test_current.max() > current.max():
        flags.append('validation_extrapolates_training_current_range')
    return {'status': 'current_torque_candidate_not_deployed', 'deployable': False,
            'equation': 'reference_torque_nm = nm_per_raw * effort_raw + bias_nm',
            'nm_per_raw': gain, 'bias_nm': bias,
            'candidate_polarity': 'positive' if gain > 1e-12 else 'negative' if gain < -1e-12 else 'unresolved',
            'training_current_range_raw': [float(current.min()), float(current.max())],
            'training': metrics(torque, current), 'independent_validation': metrics(test_torque, test_current),
            'quality_flags': flags,
            'limitations': 'Reference accuracy, torque sign convention, static friction, hysteresis and payload must be reviewed. No automatic acceptance threshold or motor configuration update.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('training', type=Path)
    parser.add_argument('validation', type=Path)
    parser.add_argument('--joint', choices=[f'joint{i}' for i in range(1, 7)], required=True)
    parser.add_argument('--reference-source', required=True,
                        help='Document measurement instrument/setup, torque sign and load provenance')
    parser.add_argument('--max-static-velocity', type=float, default=.005)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if not args.reference_source.strip():
        parser.error('Document the independent torque reference')
    if not math.isfinite(args.max_static_velocity) or args.max_static_velocity <= 0:
        parser.error('max-static-velocity must be finite and positive')
    if args.output.exists():
        parser.error('Output exists; choose a new report path')
    try:
        hashes = [hashlib.sha256(p.read_bytes()).hexdigest() for p in (args.training, args.validation)]
        if hashes[0] == hashes[1]:
            raise ValueError('Validation must be an independent recording')
        result = fit_reference(*(load_reference(p, args.joint, args.max_static_velocity)
                                 for p in (args.training, args.validation)))
    except (OSError, ValueError, KeyError) as error:
        parser.error(str(error))
    result.update(joint=args.joint, reference_source=args.reference_source,
                  max_static_velocity_rad_s=args.max_static_velocity,
                  sources=[str(p.resolve()) for p in (args.training, args.validation)], sha256=hashes)
    with args.output.open('x') as handle:
        json.dump(result, handle, indent=2, allow_nan=False)
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
