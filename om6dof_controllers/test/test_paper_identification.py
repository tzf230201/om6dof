"""Offline checks; no robot connection."""
import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
spec = importlib.util.spec_from_file_location('paper_compare', SCRIPTS / 'compare_paper_identification.py')
paper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(paper)


def robot(kind='revolute'):
    return f'''<robot name="test"><link name="base"/><link name="arm">
    <inertial><origin xyz=".2 -.1 .3" rpy=".2 .4 .6"/><mass value="2"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
    <link name="payload"><inertial><origin xyz=".4 .2 -.1"/><mass value=".7"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
    <joint name="joint2" type="{kind}"><parent link="base"/><child link="arm"/>
    <origin xyz=".1 .2 .3" rpy=".1 .3 -.2"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="10" velocity="2"/></joint>
    <joint name="camera" type="fixed"><parent link="arm"/><child link="payload"/>
    <origin xyz=".3 -.2 .4" rpy=".3 .2 .1"/></joint></robot>'''


@pytest.mark.parametrize('kind', ['revolute', 'prismatic'])
def test_regressor_reconstructs_potential_derivative_with_fixed_payload(kind):
    model = paper.GravityRegressor(robot(kind))
    for q in np.linspace(-1, 1, 15):
        positions = {'joint2': q}
        actual = model.matrix(positions, ['joint2', 'joint6']) @ model.nominal
        expected = model.gravity(positions, ['joint2', 'joint6'])
        np.testing.assert_allclose(actual, [expected['joint2'], 0.], atol=1e-8)


def test_basis_columns_are_derivatives_with_respect_to_inertial_parameters():
    model = paper.GravityRegressor(robot())
    baseline = model.matrix({'joint2': .7}, ['joint2'])[0]
    for index, link in enumerate(model.inertial_links):
        original_mass = link.inertial.mass
        original_xyz = list(link.inertial.origin.xyz)
        # Each h perturbation changes m*c while holding m fixed.
        for axis in range(3):
            before = model.gravity({'joint2': .7}, ['joint2'])['joint2']
            link.inertial.origin.xyz[axis] += .01 / original_mass
            after = model.gravity({'joint2': .7}, ['joint2'])['joint2']
            assert (after-before)/.01 == pytest.approx(baseline[4*index+axis+1], abs=1e-7)
            link.inertial.origin.xyz = list(original_xyz)


def test_svd_recovers_sign_model_and_exposes_duplicate_columns():
    t = np.linspace(0, 40, 1000)
    v = .1*np.cos(1.3*t)
    x = np.column_stack((np.sin(.7*t), np.sign(v), v, np.ones(len(t))))
    expected = np.array([175, 20, -4, -3])
    p, info = paper.solve(x, x @ expected)
    np.testing.assert_allclose(p, expected, atol=1e-9)
    assert info['active_coefficients_unique']
    _, info = paper.solve(np.column_stack((x, x[:, 0])), x @ expected)
    assert not info['active_coefficients_unique']
    assert info['rank'] == 4


def test_comparisons_use_same_samples_and_frozen_coefficients():
    model = paper.GravityRegressor(robot())
    t = np.linspace(0, 40, 500)
    g = np.zeros((len(t), len(model.columns)))
    g[:, 0] = np.sin(.7*t)
    v = .1*np.cos(1.3*t)
    y = 175*(g @ model.nominal) + 20*np.sign(v) + 4*v - 3
    results = paper.compare((g,v,y,{}), (g,v,y+10,{}), model, .005)
    assert len(results) == 4
    for result in results:
        assert result['training']['samples'] == 500
        assert result['independent_recording']['samples'] == 500
        assert not result['deployable']
        if result['friction_model'] == 'sign':
            assert result['training']['rmse_raw'] < 1e-8
            assert result['independent_recording']['rmse_raw'] == pytest.approx(10)


def test_actual_v2_regressor_matches_nominal_gravity_on_all_axes():
    xacro = SCRIPTS.parents[1] / 'om6dof_description/urdf/om6dof_v2.urdf.xacro'
    model = paper.GravityRegressor(paper.expand_xacro(str(xacro)))
    # Synthetic configurations only: nothing is sent to hardware.
    for q in np.random.default_rng(7).uniform(-.7, .7, size=(12, 6)):
        positions = dict(zip(paper.ARM_JOINTS, q))
        actual = model.matrix(positions, paper.ARM_JOINTS) @ model.nominal
        expected = model.gravity(positions, paper.ARM_JOINTS)
        np.testing.assert_allclose(actual, list(expected.values()), atol=2e-8)


def test_cli_refuses_same_recording_and_existing_output(tmp_path, monkeypatch):
    recording = tmp_path / 'data.csv'
    recording.write_text('test')
    output = tmp_path / 'report.yaml'
    argv = ['compare', str(recording), str(recording), '--joint', 'joint3',
            '--model', 'v2', '--output', str(output)]
    monkeypatch.setattr(sys, 'argv', argv)
    with pytest.raises(SystemExit):
        paper.main()
    assert not output.exists()
    output.write_text('keep')
    with pytest.raises(SystemExit):
        paper.main()
    assert output.read_text() == 'keep'
