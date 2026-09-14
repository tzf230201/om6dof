# SPDX-License-Identifier: Apache-2.0
"""Offline V2 D435 frame checks; no camera, robot, or ROS node is started."""

import math
from pathlib import Path
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import xacro
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def expand_model(filename):
    """Resolve this package from source, avoiding stale installed YAML/URDF."""
    from ament_index_python.packages import get_package_share_directory

    def find_package(name):
        if name == 'om6dof_description':
            return str(PACKAGE_ROOT)
        return get_package_share_directory(name)

    with patch('ament_index_python.packages.get_package_share_directory', find_package):
        document = xacro.process_file(str(PACKAGE_ROOT / 'urdf' / filename))
    return ET.fromstring(document.toxml())


def multiply(left, right):
    return [[sum(left[row][k] * right[k][col] for k in range(4))
             for col in range(4)] for row in range(4)]


def origin_matrix(element):
    xyz = [float(v) for v in element.get('xyz', '0 0 0').split()]
    roll, pitch, yaw = [float(v) for v in element.get('rpy', '0 0 0').split()]
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll),
                             math.cos(pitch), math.sin(pitch),
                             math.cos(yaw), math.sin(yaw))
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
        [-sp, cp * sr, cp * cr, xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def relative_transform(robot, parent, child):
    edges = {j.find('child').get('link'): j for j in robot.findall('joint')}
    transform = [[float(row == col) for col in range(4)] for row in range(4)]
    visited = set()
    while child != parent:
        if child in visited:
            raise AssertionError('Cycle in camera frame chain')
        visited.add(child)
        joint = edges[child]
        transform = multiply(origin_matrix(joint.find('origin')), transform)
        child = joint.find('parent').get('link')
    return transform


class D435FramesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.v2 = expand_model('om6dof_v2.urdf.xacro')
        cls.v1 = expand_model('om6dof.urdf.xacro')

    def assert_vector_close(self, actual, expected, places=9):
        self.assertEqual(len(actual), len(expected))
        for got, wanted in zip(actual, expected):
            self.assertAlmostEqual(got, wanted, places=places)

    def test_v2_has_distinct_depth_and_colour_frames_not_d405_aliases(self):
        links = {link.get('name') for link in self.v2.findall('link')}
        self.assertTrue({'d435_bottom_screw_frame', 'd435_link',
                         'd435_depth_frame', 'd435_depth_optical_frame',
                         'd435_color_frame', 'd435_color_optical_frame'} <= links)
        self.assertNotIn('d405_link', links)
        self.assertNotIn('d405_depth_optical_frame', links)
        self.assertIn('d405_payload_link', links)

    def test_v1_keeps_its_original_camera_frames(self):
        links = {link.get('name') for link in self.v1.findall('link')}
        self.assertIn('d405_link', links)
        self.assertIn('d405_depth_optical_frame', links)
        self.assertNotIn('d435_depth_optical_frame', links)

    def test_registered_bottom_screw_and_nominal_lens_positions(self):
        expected = {
            'd435_bottom_screw_frame': [0.01715000049, 0.00423634936, -2.57e-9],
            'd435_depth_optical_frame': [0.02775000049, 0.01673634936, -0.01750000257],
            'd435_color_optical_frame': [0.02775000049, 0.01673634936, -0.03250000257],
        }
        for frame, position in expected.items():
            with self.subTest(frame=frame):
                transform = relative_transform(self.v2, 'd405_payload_link', frame)
                self.assert_vector_close([transform[i][3] for i in range(3)], position)

    def test_optical_axes_point_out_of_camera_and_match_gripper_direction(self):
        for frame in ['d435_depth_optical_frame', 'd435_color_optical_frame']:
            with self.subTest(frame=frame):
                transform = relative_transform(self.v2, 'link7', frame)
                # Optical X -> link7 -Y; optical Y -> link7 +X; optical Z -> +Z.
                expected_rotation = [[0, 1, 0], [-1, 0, 0], [0, 0, 1]]
                for row in range(3):
                    self.assert_vector_close(transform[row][:3], expected_rotation[row])
        transform = relative_transform(self.v2, 'd435_link', 'd435_color_frame')
        self.assert_vector_close([transform[i][3] for i in range(3)], [0, 0.015, 0])

    def test_payload_mount_geometry_and_mass_remain_unchanged(self):
        link = self.v2.find("link[@name='d405_payload_link']")
        joint = self.v2.find("joint[@name='d405_payload_joint']")
        self.assert_vector_close([float(v) for v in joint.find('origin').get('xyz').split()],
                                 [-0.035917740, 0.0, 0.028])
        for kind in ['visual', 'collision']:
            geometry = link.find(kind)
            self.assert_vector_close([float(v) for v in geometry.find('origin').get('xyz').split()],
                                     [-0.051870518, -0.0119893, -0.039075])
            mesh = geometry.find('geometry/mesh')
            self.assertEqual(mesh.get('filename'),
                             'package://om6dof_description/meshes/d435_wrist_cam.stl')
            self.assertEqual(mesh.get('scale'), '0.001 0.001 0.001')
        self.assertAlmostEqual(float(link.find('inertial/mass').get('value')), 0.08147)

    def test_camera_model_is_explicitly_nominal_not_mass_derived(self):
        with (PACKAGE_ROOT / 'config' / 'camera_d435.yaml').open() as stream:
            camera = yaml.safe_load(stream)['camera']
        self.assertEqual(camera['model'], 'D435')
        self.assertEqual(camera['calibration_status'], 'cad_nominal')
        self.assertNotIn('mass', camera)
        self.assertNotIn('com', camera)


if __name__ == '__main__':
    unittest.main()
