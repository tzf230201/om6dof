#!/usr/bin/env python3
"""Offline comparison join/statistics regressions; no ROS nodes or hardware."""

from collections import Counter
import copy
import csv
from html.parser import HTMLParser
import json
import math
from pathlib import Path
import subprocess
import tempfile
import unittest


STATUSES = [
    'pose_found', 'pose_near_singular', 'orientation_unresolved',
    'collision_candidates_only', 'position_unresolved',
]
FIELDS = [
    'x_mm', 'y_mm', 'z_mm', 'position_found', 'pose_found', 'status',
    'rcond', 'joint_margin_deg', 'position_error_mm', 'orientation_error_deg',
    'target_roll_deg', 'target_pitch_deg', 'target_yaw_deg',
]


class EmbeddedComparisonData(HTMLParser):
    """Parse the HTML data boundary, so injected script closure cannot hide."""

    def __init__(self):
        super().__init__(convert_charrefs=False)
        self.collecting = False
        self.blocks = []
        self.tags = []

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        self.tags.append((tag, attrs))
        if tag == 'script' and attrs.get('id') == 'comparison-data':
            self.collecting = True
            self.blocks.append('')

    def handle_endtag(self, tag):
        if tag == 'script':
            self.collecting = False

    def handle_data(self, data):
        if self.collecting:
            self.blocks[-1] += data


class WorkspaceComparisonTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.build = tempfile.TemporaryDirectory(prefix='om6dof-comparison-build-')
        cls.binary = Path(cls.build.name) / 'workspace_report'
        subprocess.run([
            'g++', '-std=c++17', '-Wall', '-Wextra', '-Wpedantic', '-O2',
            str(Path(__file__).with_name('report.cpp')), '-o', str(cls.binary),
        ], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.build.cleanup()

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='om6dof-comparison-fixture-')
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.inputs = [self.directory / 'first', self.directory / 'second']
        self.output = self.directory / 'comparison'
        self.rows = [[], []]
        # Every possible category transition occurs exactly once. Independent
        # XY coordinates make the expected 5x5 matrix and plane counts explicit.
        for a in range(5):
            for b in range(5):
                x, y = 25 * a, 25 * b
                for model, status in enumerate((a, b)):
                    self.rows[model].append(dict(zip(FIELDS, [
                        x, y, 0, status <= 2, status <= 1, STATUSES[status],
                        .001 if status == 1 else .1 if status == 0 else '',
                        .25 if status <= 2 else '', .01 if status <= 2 else '',
                        .1 if status <= 1 else 15 if status == 2 else '',
                        0, 0, math.degrees(math.atan2(y, x)),
                    ])))
        self.metadata = [self.summary(rows) for rows in self.rows]
        # These describe different model geometry and may legitimately differ.
        self.metadata[1]['conservative_urdf_bound_mm'] = 497.539217
        for i, path in enumerate(self.inputs):
            path.mkdir()
            self.write_csv(i)
            self.write_summary(i)
            (path / 'index.html').write_text(f'Model {i + 1} report\n')
            (path / 'viewer3d.html').write_text(f'Model {i + 1} viewer\n')
            (path / 'slices').mkdir()
            (path / 'slices' / 'fixture.svg').write_text('<svg/>\n')

    @staticmethod
    def summary(rows):
        return {
            'engine': 'C++17 Eigen/KDL, bounded multi-start LM',
            'points': len(rows), 'radius_mm': 500,
            'conservative_urdf_bound_mm': 491.308652,
            'elapsed_seconds': .1,
            'position_found': sum(row['position_found'] for row in rows),
            'pose_found': sum(row['pose_found'] for row in rows),
            'arguments': {
                'spacing_mm': 25, 'samples': 2000, 'seeds': 4,
                'iterations': 100, 'seed': 42, 'orientation': 'radial',
                'base_link': 'world', 'tip_link': 'end_effector_link',
                'joint_margin_rad': .02, 'collision_radius_mm': 25,
                'position_tolerance_mm': 1, 'orientation_tolerance_deg': .5,
                'length_scale_mm': 300, 'singular_threshold': .01,
                'rpy_deg': [0, 0, 0],
            },
            'hard_lower_rad': [-1.5] * 6, 'hard_upper_rad': [1.5] * 6,
            'effective_lower_rad': [-1.48] * 6,
            'effective_upper_rad': [1.48] * 6,
            'counts': dict(Counter(row['status'] for row in rows)),
            'limitations': ['Finite grid and seeds; unresolved is not proven unreachable'],
        }

    def write_csv(self, model):
        with (self.inputs[model] / 'points.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDS)
            writer.writeheader()
            writer.writerows(self.rows[model])

    def write_summary(self, model):
        (self.inputs[model] / 'summary.json').write_text(
            json.dumps(self.metadata[model], ensure_ascii=False) + '\n')

    def refresh_totals(self, model):
        fresh = self.summary(self.rows[model])
        for key in ('points', 'position_found', 'pose_found', 'counts'):
            self.metadata[model][key] = fresh[key]
        self.write_csv(model)
        self.write_summary(model)

    @staticmethod
    def snapshot(path):
        return {str(p.relative_to(path)): p.read_bytes()
                for p in path.rglob('*') if p.is_file()}

    def run_comparison(self, output=None):
        return subprocess.run([
            str(self.binary), '--input', str(self.inputs[0]),
            '--compare', str(self.inputs[1]), '--output', str(output or self.output),
        ], text=True, capture_output=True, timeout=30)

    def assert_rejected_without_writes(self):
        snapshots = [self.snapshot(p) for p in self.inputs]
        result = self.run_comparison()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertFalse(self.output.exists(), result.stderr)
        self.assertEqual([self.snapshot(p) for p in self.inputs], snapshots)
        return result

    def embedded_data(self):
        parser = EmbeddedComparisonData()
        parser.feed((self.output / 'comparison.html').read_text())
        self.assertEqual(len(parser.blocks), 1)
        return json.loads(parser.blocks[0]), parser

    def test_counts_transitions_per_plane_and_preserved_inputs(self):
        before = [self.snapshot(p) for p in self.inputs]
        result = self.run_comparison()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual([self.snapshot(p) for p in self.inputs], before)
        stats = json.loads((self.output / 'comparison.json').read_text())
        self.assertEqual(stats['points'], 25)
        self.assertEqual(stats['transition_status_order'], STATUSES)
        self.assertEqual(stats['transitions'], [[1] * 5 for _ in range(5)])
        self.assertEqual(stats['positions'], {'both': 9, 'v1_only': 6, 'v2_only': 6, 'neither': 4})
        self.assertEqual(stats['poses'], {'both': 4, 'v1_only': 6, 'v2_only': 6, 'neither': 9})
        with (self.output / 'comparison_slices.csv').open(newline='') as stream:
            slices = list(csv.DictReader(stream))
        self.assertEqual(len(slices), 11)
        for axis in 'XYZ':
            self.assertEqual(sum(int(s['total']) for s in slices if s['axis'] == axis), 25)
        for row in slices:
            axis, level = row['axis'].lower() + '_mm', float(row['constant_mm'])
            left, right = [[p for p in rows if p[axis] == level] for rows in self.rows]
            self.assertEqual(int(row['total']), len(left))
            for metric in ('position', 'pose'):
                counts = [sum(p[metric + '_found'] for p in rows) for rows in (left, right)]
                self.assertEqual([int(row['v1_' + metric]), int(row['v2_' + metric])], counts)
                self.assertAlmostEqual(float(row[metric + '_delta_pp']),
                                       100 * (counts[1] - counts[0]) / len(left), places=5)
        data, _ = self.embedded_data()
        self.assertEqual(data['statistics'], stats)
        self.assertEqual(len(data['points']), 25)
        for point in data['points']:
            self.assertEqual(len(point), 11)
            self.assertEqual(point[3:5], [int(point[0] / 25), int(point[1] / 25)])
        for i, label in enumerate(('v1', 'v2')):
            self.assertEqual(self.snapshot(self.output / label), before[i])

    def test_reordered_csv_rows_join_by_xyz_not_row_number(self):
        self.rows[1].reverse()
        self.write_csv(1)
        result = self.run_comparison()
        self.assertEqual(result.returncode, 0, result.stderr)
        data, _ = self.embedded_data()
        self.assertEqual(data['statistics']['transitions'], [[1] * 5 for _ in range(5)])
        for point in data['points']:
            self.assertEqual(point[3:5], [int(point[0] / 25), int(point[1] / 25)])

    def test_mismatched_arguments_radius_or_engine_rejected(self):
        original = copy.deepcopy(self.metadata[1])
        for key, value in [('seed', 43), ('spacing_mm', 50), ('orientation', 'fixed'),
                           ('base_link', 'link1'), ('position_tolerance_mm', 2)]:
            with self.subTest(argument=key):
                self.metadata[1] = copy.deepcopy(original)
                self.metadata[1]['arguments'][key] = value
                self.write_summary(1)
                self.assert_rejected_without_writes()
        for key, value in [('radius_mm', 450), ('engine', 'other solver')]:
            with self.subTest(key=key):
                self.metadata[1] = copy.deepcopy(original)
                self.metadata[1][key] = value
                self.write_summary(1)
                self.assert_rejected_without_writes()

    def test_missing_required_argument_in_both_models_rejected(self):
        for model in range(2):
            del self.metadata[model]['arguments']['seeds']
            self.write_summary(model)
        self.assert_rejected_without_writes()

    def test_missing_xyz_with_equal_row_counts_rejected(self):
        self.rows[1][0]['x_mm'] = -25
        self.write_csv(1)
        self.assert_rejected_without_writes()

    def test_missing_row_rejected(self):
        self.rows[1].pop()
        self.refresh_totals(1)
        self.assert_rejected_without_writes()

    def test_duplicate_xyz_in_either_model_rejected(self):
        original = copy.deepcopy(self.rows)
        for model in range(2):
            with self.subTest(model=model):
                self.rows = copy.deepcopy(original)
                self.rows[model][1] = copy.deepcopy(self.rows[model][0])
                for i in range(2):
                    self.refresh_totals(i)
                result = self.assert_rejected_without_writes()
                self.assertIn('Duplicate XYZ', result.stderr)

    def test_missing_nonfinite_or_different_target_orientation_rejected(self):
        for value in ('', 'nan', 'inf', 1):
            with self.subTest(value=value):
                self.rows[1][0]['target_roll_deg'] = value
                self.write_csv(1)
                self.assert_rejected_without_writes()

    def test_summary_totals_and_categories_must_match_csv(self):
        original = copy.deepcopy(self.metadata[1])
        for key in ('points', 'position_found', 'pose_found'):
            with self.subTest(key=key):
                self.metadata[1] = copy.deepcopy(original)
                self.metadata[1][key] += 1
                self.write_summary(1)
                self.assert_rejected_without_writes()
        self.metadata[1] = copy.deepcopy(original)
        self.metadata[1]['counts']['pose_found'] -= 1
        self.metadata[1]['counts']['pose_near_singular'] += 1
        self.write_summary(1)
        self.assert_rejected_without_writes()

    def test_existing_output_or_input_directory_cannot_be_overwritten(self):
        self.output.mkdir()
        (self.output / 'keep.txt').write_text('User content\n')
        for destination in [self.output, *self.inputs]:
            with self.subTest(destination=destination.name):
                before = self.snapshot(self.directory)
                result = self.run_comparison(destination)
                self.assertNotEqual(result.returncode, 0, result.stdout)
                self.assertEqual(self.snapshot(self.directory), before)

    def test_embedded_metadata_escapes_script_closure_and_preserves_text(self):
        hostile = '</script><script id="metadata-injection">alert("&")</script>\u2028\u2029'
        self.metadata[0]['fixture_note'] = hostile
        self.write_summary(0)
        result = self.run_comparison()
        self.assertEqual(result.returncode, 0, result.stderr)
        data, parser = self.embedded_data()
        self.assertFalse(any(attrs.get('id') == 'metadata-injection' for _, attrs in parser.tags))
        restored = json.loads(data['statistics']['v1SummaryText'])
        self.assertEqual(restored['fixture_note'], hostile)
        self.assertNotIn('</script', parser.blocks[0].lower())


if __name__ == '__main__':
    unittest.main()
