#!/usr/bin/env python3
"""Standalone reporter regression tests: no ROS, robot, or GUI required."""
import csv
from html.parser import HTMLParser
import json
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET


class EmbeddedWorkspaceData(HTMLParser):
    """Extract data as a browser parses it, detecting premature script closure."""

    def __init__(self):
        super().__init__(convert_charrefs=False)
        self.collecting = False
        self.blocks = []
        self.tags = []

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        self.tags.append((tag, attrs))
        if tag == 'script' and attrs.get('id') == 'workspace-data':
            self.collecting = True
            self.blocks.append('')

    def handle_endtag(self, tag):
        if tag == 'script':
            self.collecting = False

    def handle_data(self, data):
        if self.collecting:
            self.blocks[-1] += data


class WorkspaceReportTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.build = tempfile.TemporaryDirectory(prefix='om6dof-report-build-')
        cls.binary = Path(cls.build.name) / 'workspace_report'
        subprocess.run([
            'g++', '-std=c++17', '-Wall', '-Wextra', '-Wpedantic', '-O2',
            str(Path(__file__).with_name('report.cpp')), '-o', str(cls.binary),
        ], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.build.cleanup()

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='om6dof-report-fixture-')
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.csv_path = self.directory / 'points.csv'
        self.fields = [
            'x_mm', 'y_mm', 'z_mm', 'position_found', 'pose_found', 'status',
            'rcond', 'joint_margin_deg', 'position_error_mm', 'orientation_error_deg',
        ]
        statuses = [
            'pose_found', 'pose_near_singular', 'orientation_unresolved',
            'collision_candidates_only', 'position_unresolved',
        ]
        self.rows = []
        for x in [-50, 0, 50]:
            for y in [-50, 0, 50]:
                for z in [-50, 0, 50]:
                    index = len(self.rows) % 5
                    self.rows.append(dict(zip(self.fields, [
                        x, y, z, index <= 2, index <= 1, statuses[index],
                        .001 if index == 1 else .1 if index == 0 else '',
                        .25 if index <= 2 else '', .01 if index <= 2 else '',
                        .1 if index <= 1 else 15 if index == 2 else '',
                    ])))
        self.write_csv()
        (self.directory / 'summary.json').write_text('{"fixture": true}\n')

    def write_csv(self):
        with self.csv_path.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=self.fields)
            writer.writeheader()
            writer.writerows(self.rows)

    def run_report(self, *args):
        return subprocess.run([
            str(self.binary), '--input', str(self.directory), *args,
        ], text=True, capture_output=True)

    def viewer_data(self):
        parser = EmbeddedWorkspaceData()
        parser.feed((self.directory / 'viewer3d.html').read_text())
        self.assertEqual(len(parser.blocks), 1)
        return json.loads(parser.blocks[0]), parser

    def test_every_constant_plane_and_csv_counts(self):
        original = self.csv_path.read_bytes()
        result = self.run_report()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.csv_path.read_bytes(), original)
        self.assertEqual((self.directory / 'summary.json').read_text(), '{"fixture": true}\n')
        with (self.directory / 'slices.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 9)
        for axis in 'XYZ':
            group = [row for row in rows if row['constant_axis'] == axis]
            self.assertEqual([float(row['constant_mm']) for row in group], [-50, 0, 50])
            self.assertEqual(sum(int(row['total']) for row in group), 27)
            self.assertEqual([int(row['total']) for row in group], [9, 9, 9])
            ET.parse(self.directory / f'slices_{axis}.svg')
        self.assertEqual(len(list((self.directory / 'slices').glob('*.svg'))), 9)
        for row in rows:
            root = ET.parse(self.directory / row['svg']).getroot()
            self.assertEqual(root.tag, '{http://www.w3.org/2000/svg}svg')
        analysis = (self.directory / 'analysis.md').read_text()
        self.assertIn('27 discrete points', analysis)
        self.assertIn('**Blue triangles** mean a pose was found', analysis)
        self.assertIn('**Yellow squares** separate orientation constraints', analysis)
        self.assertIn('does not prove', analysis)
        self.assertIn('SO(3)', analysis)
        self.assertIn('(viewer3d.html)', analysis)
        html = (self.directory / 'index.html').read_text()
        self.assertNotIn('<script', html)
        self.assertIn('Z_plus_50mm.svg', html)
        self.assertIn('lang="en"', html)
        self.assertIn('href="viewer3d.html"', html)
        self.assertIn('The 2D report is static', html)
        self.assertIn('No internet access, ROS nodes, or hardware connections', html)
        self.assertIn('Pose found', html)
        self.assertNotIn('Posisi ditemukan', html)
        data, _ = self.viewer_data()
        self.assertEqual(len(data['points']), 27)
        self.assertEqual(data['summaryText'], '{"fixture": true}\n')
        for index, point in enumerate(data['points']):
            self.assertEqual(len(point), 17)
            self.assertEqual(point[:3], [self.rows[index][name] for name in ['x_mm', 'y_mm', 'z_mm']])
            self.assertEqual(point[3], index % 5)
            self.assertEqual(point[8:], [None] * 9)
        self.assertEqual(data['points'][0][4:8], [.1, .25, .01, .1])
        self.assertEqual(data['points'][3][4:8], [None] * 4)

    def test_optional_witness_values_and_nonfinite_json(self):
        optional = [f'q{joint}_rad' for joint in range(1, 7)]
        optional += ['target_roll_deg', 'target_pitch_deg', 'target_yaw_deg']
        self.fields.extend(optional)
        values = [.1, -.2, .3, -.4, .5, -.6, 20, -30, 180]
        self.rows[0].update(zip(optional, values))
        self.rows[1]['q1_rad'] = 'nan'
        self.rows[1]['q2_rad'] = 'inf'
        self.rows[1]['q3_rad'] = '-inf'
        self.rows[1]['target_roll_deg'] = 'nan'
        self.rows[1]['rcond'] = 'nan'
        self.write_csv()
        result = self.run_report()
        self.assertEqual(result.returncode, 0, result.stderr)
        data, _ = self.viewer_data()
        self.assertEqual(data['points'][0][8:], values)
        self.assertEqual(data['points'][1][8:], [None] * 9)
        self.assertIsNone(data['points'][1][4])

    def test_distinct_shapes_match_points_and_all_legends(self):
        result = self.run_report()
        self.assertEqual(result.returncode, 0, result.stderr)
        namespace = '{http://www.w3.org/2000/svg}'
        expected = {
            'pose_found': ('circle', 'circle', '#15976b'),
            'pose_near_singular': ('triangle', 'polygon', '#397ac6'),
            'orientation_unresolved': ('square', 'rect', '#e5a21a'),
            'collision_candidates_only': ('diamond', 'polygon', '#9658ad'),
            'position_unresolved': ('cross', 'path', '#d15b59'),
        }

        def check_markers(root, context, count):
            markers = root.findall(f'.//*[@data-context="{context}"]')
            self.assertEqual(len(markers), count)
            self.assertEqual({marker.get('data-status') for marker in markers}, set(expected))
            for marker in markers:
                shape_name, tag, color = expected[marker.get('data-status')]
                self.assertEqual(marker.get('data-shape'), shape_name)
                graphics = [child for child in marker if child.tag != namespace + 'title']
                self.assertEqual(len(graphics), 1)
                graphic = graphics[0]
                self.assertEqual(graphic.tag, namespace + tag)
                if tag == 'path':
                    self.assertEqual(graphic.get('fill'), 'none')
                    self.assertEqual(graphic.get('stroke'), color)
                    self.assertEqual(graphic.get('stroke-linecap'), 'round')
                    self.assertGreater(float(graphic.get('stroke-width')), 0)
                    # An X is two independent diagonal strokes, not a filled plus sign.
                    tokens = graphic.get('d').split()
                    self.assertEqual([tokens[i] for i in (0, 3, 6, 9)], ['M', 'L', 'M', 'L'])
                    endpoints = [tuple(map(float, tokens[i:i + 2])) for i in (1, 4, 7, 10)]
                    a, b, c, d = endpoints
                    self.assertLess(a[0], b[0])
                    self.assertLess(a[1], b[1])
                    self.assertEqual(a[0], d[0])
                    self.assertEqual(a[1], c[1])
                    self.assertEqual(b[0], c[0])
                    self.assertEqual(b[1], d[1])
                else:
                    self.assertEqual(graphic.get('fill'), color)
                if tag == 'polygon':
                    self.assertEqual(len(graphic.get('points').split()), 3 if shape_name == 'triangle' else 4)
                elif tag == 'rect':
                    self.assertEqual(graphic.get('width'), graphic.get('height'))
                    self.assertIsNone(graphic.get('rx'))
                if context == 'point':
                    title = marker.find(namespace + 'title')
                    self.assertIsNotNone(title)
                    self.assertRegex(title.text, r'^X=-?\d+\.\d+; Y=-?\d+\.\d+; Z=-?\d+\.\d+ mm; ')
                    self.assertIn('; position error=', title.text)
                    self.assertIn('; orientation error=', title.text)
                    self.assertIn('; rcond=', title.text)
                    self.assertIn('; margin=', title.text)

        for path in (self.directory / 'slices').glob('*.svg'):
            root = ET.parse(path).getroot()
            check_markers(root, 'point', 9)
            check_markers(root, 'legend', 5)
        for axis in 'XYZ':
            root = ET.parse(self.directory / f'slices_{axis}.svg').getroot()
            check_markers(root, 'point', 27)
            check_markers(root, 'legend', 5)

        html = (self.directory / 'index.html').read_text()
        inline_svgs = re.findall(r'<svg\b.*?</svg>', html, flags=re.DOTALL)
        self.assertEqual(len(inline_svgs), 5)
        combined = ET.Element('root')
        for svg in inline_svgs:
            combined.append(ET.fromstring(svg))
        check_markers(combined, 'legend', 5)
        for shape in ['circle', 'square', 'triangle', 'diamond', 'X']:
            self.assertIn(f'({shape})', html)
        analysis = (self.directory / 'analysis.md').read_text()
        self.assertNotIn('Colored squares', analysis)
        self.assertIn('red X markers', analysis)

    def test_metadata_is_preserved_as_safe_script_data(self):
        metadata = '{"note":"</script><img src=x onerror=alert(1)> & \\\"quoted\\\" C:\\\\robot"}\n\t\x01'
        (self.directory / 'summary.json').write_text(metadata)
        result = self.run_report()
        self.assertEqual(result.returncode, 0, result.stderr)
        data, parser = self.viewer_data()
        self.assertEqual(data['summaryText'], metadata)
        self.assertFalse(any('onerror' in attrs for _, attrs in parser.tags))
        block = parser.blocks[0]
        self.assertNotIn('<', block)
        self.assertNotIn('>', block)
        self.assertNotIn('&', block)
        self.assertNotIn('\x01', block)
        self.assertEqual((self.directory / 'summary.json').read_text(), metadata)

    def test_zero_one_booleans(self):
        for row in self.rows:
            row['position_found'] = int(row['position_found'])
            row['pose_found'] = int(row['pose_found'])
        self.write_csv()
        self.assertEqual(self.run_report().returncode, 0)

    def test_empty_intermediate_planes_are_not_failures(self):
        result = self.run_report('--slice-step-mm', '25')
        self.assertEqual(result.returncode, 0, result.stderr)
        with (self.directory / 'slices.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 15)
        empty = [row for row in rows if abs(float(row['constant_mm'])) == 25]
        self.assertEqual(len(empty), 6)
        self.assertTrue(all(row['total'] == '0' and row['position_percent'] == 'n/a' for row in empty))
        self.assertIn('No samples on this plane', (self.directory / empty[0]['svg']).read_text())

    def test_bad_arguments_and_inconsistent_status_rejected(self):
        for step in ['0', '-10', 'nan', 'abc']:
            self.assertNotEqual(self.run_report('--slice-step-mm', step).returncode, 0)
        self.rows[0]['pose_found'] = False
        self.write_csv()
        result = self.run_report()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Inconsistent', result.stderr)


if __name__ == '__main__':
    unittest.main()
