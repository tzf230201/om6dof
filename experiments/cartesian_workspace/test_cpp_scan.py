"""Independent native scanner checks; never starts ROS nodes or moves hardware.

Build the binary and export a rendered URDF first (see README). Set
OM6DOF_CPP_SCAN_BINARY / OM6DOF_CPP_SCAN_URDF for non-default locations.
Tests skip cleanly when those optional generated prerequisites are absent.
"""

from collections import Counter
import csv
import json
import math
import os
from pathlib import Path
import subprocess

import numpy as np
import pytest

from workspace_scan import IKSolver, cartesian_rpy_to_tip_rotation, conditioning
from om6dof_controller import ik_solver


@pytest.fixture(scope='module')
def native():
    binary = Path(os.environ.get(
        'OM6DOF_CPP_SCAN_BINARY', '/tmp/om6dof-workspace-cpp-build/workspace_scan_cpp'))
    urdf = Path(os.environ.get(
        'OM6DOF_CPP_SCAN_URDF', '/tmp/om6dof-workspace-cpp-model.urdf'))
    if not binary.is_file() or not urdf.is_file():
        pytest.skip('Native scanner and rendered URDF must be generated first')
    return binary, urdf


@pytest.fixture
def reference(native, monkeypatch):
    # Compare identical XML, not potentially stale source/install URDFs.
    xml = native[1].read_text()
    monkeypatch.setattr(ik_solver, 'render_chain_urdf', lambda *a, **kw: xml)
    return IKSolver()


def run_native(native, *args, check=True):
    return subprocess.run(
        [str(native[0]), '--urdf', str(native[1]), *map(str, args)],
        check=check, text=True, capture_output=True, timeout=120,
    )


def as_bool(value):
    assert value.lower() in ('true', 'false', '1', '0')
    return value.lower() in ('true', '1')


@pytest.mark.parametrize('q', [
    [0., 0., 0., 0., 0., 0.],
    [0., -.6806, 1.3613, 0., .8901, 0.],
    [.4, -.6, .8, .3, -.2, .5],
    [-.8, .3, -.9, -.7, 1.8, -.4],
    [1.4, -1., 1.5, .6, -1., -.9],
])
def test_probe_fk_and_scaled_jacobian_match_python(native, reference, q):
    result = json.loads(run_native(native, '--probe', *q).stdout)
    position, rotation = reference.fk_pose(np.asarray(q))
    np.testing.assert_allclose(result['position'], position, atol=1e-12, rtol=1e-12)
    np.testing.assert_allclose(
        np.asarray(result['rotation']).reshape(3, 3), rotation, atol=1e-12, rtol=1e-12)
    assert result['rcond'] == pytest.approx(
        conditioning(reference, np.asarray(q), .3), abs=1e-12, rel=1e-10)


def test_native_self_tests(native):
    run_native(native, '--self-test')


def test_native_grid_includes_all_seven_sphere_centres(native, tmp_path):
    output = tmp_path / 'seven-points'
    run_native(native, '--output', output, '--radius-mm', 50, '--spacing-mm', 50,
               '--samples', 16, '--seeds', 2, '--iterations', 20)
    with (output / 'points.csv').open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    expected = {(0., 0., 0.)}
    for axis in range(3):
        for sign in (-1, 1):
            point = [0., 0., 0.]
            point[axis] = sign * 50.
            expected.add(tuple(point))
    actual = {tuple(float(row[f'{axis}_mm']) for axis in 'xyz') for row in rows}
    assert len(rows) == len(actual) == 7
    assert actual == expected
    summary = json.loads((output / 'summary.json').read_text())
    assert summary['points'] == 7
    assert summary['counts'] == dict(Counter(row['status'] for row in rows))
    assert summary['position_found'] == sum(as_bool(row['position_found']) for row in rows)
    assert summary['pose_found'] == sum(as_bool(row['pose_found']) for row in rows)
    assert (output / 'model.urdf').read_text() == native[1].read_text()


def test_finer_grid_adds_real_intermediate_samples(native, tmp_path):
    grids = {}
    for spacing in (50, 25):
        output = tmp_path / f'grid-{spacing}'
        run_native(native, '--output', output, '--radius-mm', 50, '--spacing-mm', spacing,
                   '--samples', 16, '--seeds', 2, '--iterations', 20)
        with (output / 'points.csv').open(newline='') as stream:
            rows = list(csv.DictReader(stream))
        grids[spacing] = {tuple(float(row[f'{axis}_mm']) for axis in 'xyz') for row in rows}
        assert len(grids[spacing]) == len(rows)
        summary = json.loads((output / 'summary.json').read_text())
        assert summary['arguments']['spacing_mm'] == spacing
    assert len(grids[50]) == 7
    assert len(grids[25]) == 33
    assert grids[50] < grids[25]
    assert (25., 0., 0.) in grids[25]


def test_saved_witnesses_match_python_fk_limits_and_collision(native, reference, tmp_path):
    output = tmp_path / 'witnesses'
    run_native(native, '--output', output, '--radius-mm', 350, '--spacing-mm', 150,
               '--samples', 128, '--seeds', 4, '--iterations', 100)
    with (output / 'points.csv').open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    witnessed = [row for row in rows if as_bool(row['position_found'])]
    assert witnessed, 'Fixture must independently verify at least one real IK witness'
    reference.q_min += .02
    reference.q_max -= .02
    for row in rows:
        position_found, pose_found = as_bool(row['position_found']), as_bool(row['pose_found'])
        assert not pose_found or position_found
        assert pose_found == (row['status'] in ('pose_found', 'pose_near_singular'))
        if not position_found:
            assert row['status'] in ('position_unresolved', 'collision_candidates_only')
            assert all(not row[f'q{i}_rad'] for i in range(1, 7))
            continue
        q = np.array([float(row[f'q{i}_rad']) for i in range(1, 7)])
        assert np.all(np.isfinite(q))
        assert np.all(q >= reference.q_min - 1e-10)
        assert np.all(q <= reference.q_max + 1e-10)
        assert not reference.self_collides(q, .025)
        actual_position, actual_rotation = reference.fk_pose(q)
        desired_position = np.array([float(row[f'{axis}_mm']) for axis in 'xyz']) / 1000
        desired_rpy = np.radians([float(row[f'target_{axis}_deg']) for axis in ('roll', 'pitch', 'yaw')])
        desired_rotation = cartesian_rpy_to_tip_rotation(*desired_rpy)
        position_error = 1000 * np.linalg.norm(actual_position - desired_position)
        # Use the controller's quaternion-based logarithm. The older helper
        # divides by sin(angle), losing precision for almost-180-degree errors.
        orientation_error = math.degrees(np.linalg.norm(
            reference.orientation_difference(desired_rotation, actual_rotation)))
        assert position_error <= 1. + 1e-8
        assert float(row['position_error_mm']) == pytest.approx(position_error, abs=1e-7)
        assert float(row['orientation_error_deg']) == pytest.approx(orientation_error, abs=1e-7)
        if pose_found:
            assert orientation_error <= .5 + 1e-8
            assert float(row['rcond']) == pytest.approx(conditioning(reference, q, .3), abs=1e-10)
            assert (row['status'] == 'pose_near_singular') == (float(row['rcond']) < .01)
    known = next(row for row in rows if [float(row[f'{axis}_mm']) for axis in 'xyz'] == [150., 0., 300.])
    assert as_bool(known['pose_found']), 'Previously problematic 150/0/300 mm case must have a pose witness'


@pytest.mark.parametrize('argument,value', [
    ('--spacing-mm', '0'), ('--radius-mm', 'nan'), ('--samples', '-1'),
    ('--joint-margin-rad', '-.1'), ('--iterations', '0'),
    ('--radius-mm', '1e308'), ('--spacing-mm', '1e-300'),
])
def test_invalid_settings_fail_without_output(native, tmp_path, argument, value):
    output = tmp_path / 'invalid'
    result = run_native(native, '--output', output, argument, value, check=False)
    assert result.returncode != 0
    assert not output.exists()


def test_existing_output_is_not_overwritten(native, tmp_path):
    result = run_native(native, '--output', tmp_path, check=False)
    assert result.returncode != 0
    assert not list(tmp_path.iterdir())
