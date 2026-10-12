#!/usr/bin/env python3
"""Record action feedback and align it with measured current offline; no motion."""
import argparse
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path
import time

JOINTS = tuple(f'joint{i}' for i in range(1, 7))
FIELDS = ('desired_position_rad', 'actual_position_rad', 'error_position_rad',
          'desired_velocity_rad_s', 'actual_velocity_rad_s', 'error_velocity_rad_s')


def tracking_path(csv_path):
    return Path(str(csv_path) + '.tracking.csv')


def report_path(csv_path):
    return Path(str(csv_path) + '.tracking.json')


class TrackingRecorder:
    """Feedback is goal-specific, not a reconstructed trajectory or stale target."""
    def __init__(self, csv_path):
        self.path = tracking_path(csv_path)
        self.file = self.path.open('x', newline='', encoding='utf-8')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['feedback_stamp_s', 'received_monotonic_s', 'goal_id',
                              'desired_time_from_start_s'] +
                             [f'{j}_{f}' for j in JOINTS for f in FIELDS])
        self.count = 0
        self.file.flush()

    def write(self, message):
        f = message.feedback
        stamp = f.header.stamp.sec + f.header.stamp.nanosec * 1e-9
        duration = f.desired.time_from_start
        row = [stamp, time.monotonic(), bytes(message.goal_id.uuid).hex(),
               duration.sec + duration.nanosec * 1e-9]
        indices = {name: i for i, name in enumerate(f.joint_names)}
        # Missing values stay NaN; never invent a zero measurement.
        for joint in JOINTS:
            i = indices.get(joint)
            for attribute in ('positions', 'velocities'):
                for point in (f.desired, f.actual, f.error):
                    values = getattr(point, attribute)
                    row.append(values[i] if i is not None and i < len(values) else float('nan'))
        self.writer.writerow(row)
        self.count += 1
        if self.count % 20 == 0:
            self.file.flush()

    def close(self):
        self.file.close()


def read_rows(path, stamp_name):
    with Path(path).open(newline='') as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError('No samples in ' + str(path))
    times = [float(row[stamp_name]) for row in rows]
    if any(not math.isfinite(t) or t <= 0 for t in times):
        raise ValueError('Missing/invalid source timestamps in ' + str(path))
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError('Non-increasing timestamps in ' + str(path))
    return rows, times


def nearest_index(times, target):
    i = bisect.bisect_left(times, target)
    return min((k for k in (i-1, i) if 0 <= k < len(times)),
               key=lambda k: abs(times[k]-target))


def rms(values):
    return math.sqrt(sum(v*v for v in values)/len(values)) if values else None


def analyze(csv_path, max_skew=.025):
    if not math.isfinite(max_skew) or max_skew <= 0:
        raise ValueError('max-skew must be finite and positive')
    feedback_path = tracking_path(csv_path)
    measured, mt = read_rows(csv_path, 'ros_stamp_s')
    feedback, ft = read_rows(feedback_path, 'feedback_stamp_s')
    goals = {r['goal_id'] for r in feedback}
    if len(goals) != 1 or not next(iter(goals)) or set(next(iter(goals))) == {'0'}:
        raise ValueError('Tracking must contain exactly one nonzero action goal ID')
    rows, errors = [], {j: {'tracking': [], 'actual_disagreement': [], 'error_consistency': [],
                          'velocity_error': []}
                        for j in JOINTS}
    for row, stamp in zip(feedback, ft):
        i = nearest_index(mt, stamp)
        skew = mt[i] - stamp
        if abs(skew) > max_skew:
            continue
        out = {'feedback_stamp_s': stamp, 'joint_state_stamp_s': mt[i],
               'stamp_skew_s': skew, 'goal_id': row['goal_id']}
        for joint in JOINTS:
            values = {field: float(row[f'{joint}_{field}']) for field in FIELDS}
            measured_position = float(measured[i][f'{joint}_position_rad'])
            effort = float(measured[i][f'{joint}_effort_raw'])
            for key, value in values.items():
                out[f'{joint}_{key}'] = value
            out[f'{joint}_measured_position_rad'] = measured_position
            out[f'{joint}_effort_raw'] = effort
            desired, actual = values['desired_position_rad'], values['actual_position_rad']
            if all(math.isfinite(v) for v in (desired, actual, measured_position, effort)):
                errors[joint]['tracking'].append(desired-actual)
                errors[joint]['actual_disagreement'].append(actual-measured_position)
                error = values['error_position_rad']
                if math.isfinite(error):
                    errors[joint]['error_consistency'].append(error-(desired-actual))
                dv, av = values['desired_velocity_rad_s'], values['actual_velocity_rad_s']
                if math.isfinite(dv) and math.isfinite(av):
                    errors[joint]['velocity_error'].append(dv-av)
        rows.append(out)
    coverage = len(rows)/len(feedback)
    result = {
        'status': 'tracking_diagnostic_not_safety_or_torque_validation',
        'source_csv': str(Path(csv_path).resolve()), 'tracking_csv': str(feedback_path.resolve()),
        'sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                   for p in (Path(csv_path), feedback_path)},
        'feedback_samples': len(feedback), 'matched_samples': len(rows),
        'unmatched_samples': len(feedback)-len(rows), 'matched_fraction': coverage,
        'max_allowed_stamp_skew_s': max_skew,
        'observed_max_abs_skew_s': max((abs(r['stamp_skew_s']) for r in rows), default=None),
        'matching': 'Nearest source timestamp, no interpolation. A measured sample may serve multiple feedback rows.',
        'warning': 'Time-aligned streams, NOT atomic hardware sampling; current remains raw, not Nm.',
        'quality_flags': ([] if coverage >= .95 else ['low_timestamp_match_coverage']),
        'joints': {},
    }
    for joint, e in errors.items():
        result['joints'][joint] = {
            'finite_matched_samples': len(e['tracking']),
            'tracking_rmse_rad': rms(e['tracking']),
            'tracking_max_abs_rad': max(map(abs, e['tracking']), default=None),
            'controller_vs_joint_state_rmse_rad': rms(e['actual_disagreement']),
            'reported_vs_recomputed_error_rmse_rad': rms(e['error_consistency']),
            'finite_velocity_samples': len(e['velocity_error']),
            'velocity_tracking_rmse_rad_s': rms(e['velocity_error']),
        }
        if len(e['tracking']) != len(rows) or not e['tracking']:
            result['quality_flags'].append(joint + '_missing_or_nonfinite_data')
        if len(e['velocity_error']) != len(rows):
            result['quality_flags'].append(joint + '_missing_velocity')
    return result, rows


def write_report(csv_path):
    result, _ = analyze(csv_path)
    with report_path(csv_path).open('x') as handle:
        json.dump(result, handle, indent=2, allow_nan=False)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('--max-skew', type=float, default=.025)
    parser.add_argument('--output', type=Path, required=True, help='New JSON report path')
    parser.add_argument('--aligned-output', type=Path, help='Optional new aligned CSV path')
    args = parser.parse_args()
    if args.output.exists() or (args.aligned_output and args.aligned_output.exists()):
        parser.error('Output exists; choose new paths')
    if args.aligned_output and args.aligned_output.resolve() == args.output.resolve():
        parser.error('Output paths must differ')
    try:
        report, rows = analyze(args.csv, args.max_skew)
    except (OSError, ValueError, KeyError) as error:
        parser.error(str(error))
    with args.output.open('x') as handle:
        json.dump(report, handle, indent=2, allow_nan=False)
    if args.aligned_output and rows:
        with args.aligned_output.open('x', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
