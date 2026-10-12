"""Offline calibration regression tests; no ROS node or motor is started."""

import importlib.util
from pathlib import Path
import subprocess
import sys
import csv
import yaml
import pytest

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'fit_gravity_friction.py'
spec = importlib.util.spec_from_file_location('fit_gravity_friction', SCRIPT)
fit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fit)


def test_model_must_be_selected_before_reading_data():
    result = subprocess.run([sys.executable, str(SCRIPT), 'missing.csv'],
                            capture_output=True, text=True)
    assert result.returncode == 2
    assert 'select --model v1/v2 or --xacro' in result.stderr


def test_synthetic_coefficients_are_recovered():
    samples, gravity = [], []
    for t in np.linspace(0, 30, 300):
        g, velocity = np.sin(.7 * t), .2 * np.cos(1.3 * t)
        current = 175 * g + 20 * np.tanh(velocity / .02) + 4 * velocity - 3
        samples.append(({}, {'joint2': velocity}, {'joint2': current}))
        gravity.append({'joint2': g})
    result = fit.fit_joint(samples, gravity, 'joint2', .02, .005)
    np.testing.assert_allclose([
        result['gravity_scale_raw_per_nm'], result['coulomb_raw'],
        result['viscous_raw_per_rad_s'], result['bias_raw'],
    ], [175, 20, 4, -3], atol=1e-8)
    assert result['rmse_raw'] < 1e-8


def test_fit_and_independent_evaluation_round_trip(monkeypatch, tmp_path):
    urdf = '''<robot name="test"><link name="base"/><link name="bob">
    <inertial><origin xyz="1 0 0"/><mass value="1"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
    <joint name="joint2" type="continuous"><parent link="base"/><child link="bob"/>
    <axis xyz="0 1 0"/></joint></robot>'''
    monkeypatch.setattr(fit, 'expand_xacro', lambda _: urdf)
    gravity = fit.UrdfGravity(urdf)
    def write_data(path, bias):
        with path.open('w', newline='') as handle:
            writer = csv.writer(handle)
            writer.writerow([f'{j}_{f}' for j in fit.ARM_JOINTS
                             for f in ('position_rad', 'velocity_rad_s', 'effort_raw')])
            for t in np.linspace(0, 60, 200):
                q = .7 + .4*np.sin(.3*t)
                v = .12*np.cos(.3*t)
                g = gravity.gravity({'joint2':q}, ['joint2'])['joint2']
                current = 175*g + 20*np.tanh(v/.02) + 4*v + bias
                writer.writerow([value for j in fit.ARM_JOINTS
                                 for value in ((q,v,current) if j=='joint2' else (0,0,0))])
    training, validation = tmp_path/'training.csv', tmp_path/'validation.csv'
    model, report = tmp_path/'fit.yaml', tmp_path/'evaluation.yaml'
    write_data(training, -3)
    write_data(validation, 7)
    common = ['--xacro', 'test.xacro', '--stride', '1', '--velocity-source', 'reported']
    monkeypatch.setattr(sys, 'argv', [str(SCRIPT), str(training), *common,
                                    '--auto-joints', '--output', str(model)])
    fit.main()
    result = yaml.safe_load(model.read_text())
    assert list(result['joints']) == ['joint2']
    assert result['joints']['joint2']['coulomb_raw'] == pytest.approx(20)
    assert result['calibration_status'] == 'candidate_unvalidated'
    original = model.read_bytes()
    monkeypatch.setattr(sys, 'argv', [str(SCRIPT), str(validation), *common,
                                    '--evaluate', str(model), '--output', str(report)])
    fit.main()
    assert yaml.safe_load(report.read_text())['joints']['joint2']['rmse_raw'] == pytest.approx(10)
    assert model.read_bytes() == original
    monkeypatch.setattr(sys, 'argv', [str(SCRIPT), str(training), *common,
                                    '--evaluate', str(model), '--output', str(report)])
    with pytest.raises(SystemExit):
        fit.main()


def test_quadratic_derivative_uses_irregular_timestamps():
    times = np.cumsum(np.random.default_rng(8).uniform(.01, .03, 250))
    positions = np.column_stack((2 + .03*times + .002*times**2, np.ones(len(times))))
    velocity = fit.estimate_velocity(times, positions, 1., .2)
    valid = np.all(np.isfinite(velocity), axis=1)
    assert valid.sum() > 100
    np.testing.assert_allclose(velocity[valid, 0], .03 + .004*times[valid], atol=1e-10)
    np.testing.assert_allclose(velocity[valid, 1], 0, atol=1e-10)
    assert not valid[0] and not valid[-1]


def test_position_derivative_resolves_slow_quantized_motion():
    times = np.arange(0, 12, .015)
    exact = .1*np.sin(.25*times)
    quantized = np.round(exact/.001534)*.001534
    velocity = fit.estimate_velocity(times, quantized, 1., .2)
    valid = np.isfinite(velocity)
    rmse = np.sqrt(np.mean((velocity[valid] - .025*np.cos(.25*times[valid]))**2))
    assert rmse < .0015


def test_velocity_estimation_does_not_bridge_gaps():
    times = np.r_[np.arange(0, 2, .02), np.arange(3, 5, .02)]
    velocity = fit.estimate_velocity(times, times*.02, 1., .2)
    assert np.all(np.isnan(velocity[(times > 1.5) & (times < 3.5)]))
    np.testing.assert_allclose(velocity[np.isfinite(velocity)], .02, atol=1e-10)


@pytest.mark.parametrize('times', [[0, 1, 2, 2, 3, 4, 5], [0, 1, 2, 1, 3, 4, 5]])
def test_invalid_time_order_is_rejected(times):
    with pytest.raises(ValueError, match='strictly increasing'):
        fit.estimate_velocity(times, np.zeros(len(times)))


def test_single_speed_friction_fit_is_marked_rank_deficient():
    samples, gravity = [], []
    for i in range(100):
        velocity = .024 if i % 2 else -.024
        g = np.sin(i*.1)
        samples.append(({}, {'joint2':velocity}, {'joint2':175*g + 10*np.sign(velocity)}))
        gravity.append({'joint2':g})
    result = fit.fit_joint(samples, gravity, 'joint2', .02, .005)
    assert 'rank_deficient' in result['quality_flags']
    assert result['fit_quality'] == 'rejected'


def test_position_preprocessing_records_method_and_does_not_leak_across_split(tmp_path):
    path = tmp_path / 'quantized.csv'
    times = np.arange(0, 4, .02)
    with path.open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['ros_stamp_s'] + [f'{j}_{field}' for j in fit.ARM_JOINTS
            for field in ('position_rad', 'velocity_rad_s', 'effort_raw')])
        for t in times:
            # A discontinuity in the held-out half must not enter a training window.
            q = .02*t + (10 if t >= 2 else 0)
            writer.writerow([t] + [x for _ in fit.ARM_JOINTS for x in (q, 0., 20.)])
    samples, metadata = fit.load_samples(path, 'position', return_metadata=True,
                                        row_slice=slice(0, 100))
    assert metadata['source'] == 'position'
    assert metadata['method'] == 'centered_quadratic'
    assert metadata['reported_zero_fraction']['joint2'] == 1.
    assert 0 < len(samples) < 100
    np.testing.assert_allclose([s[1]['joint2'] for s in samples], .02, atol=1e-10)


def test_negative_viscous_is_visible_not_silently_clipped():
    samples, gravity = [], []
    for t in np.linspace(0, 30, 300):
        g, v = np.sin(.7*t), .2*np.cos(1.3*t)
        samples.append(({}, {'joint2':v}, {'joint2':175*g+20*np.tanh(v/.02)-4*v-3}))
        gravity.append({'joint2':g})
    result = fit.fit_joint(samples, gravity, 'joint2', .02, .005)
    assert result['viscous_raw_per_rad_s'] == pytest.approx(-4)
    assert result['fit_quality'] == 'rejected'
    assert 'negative_viscous' in result['quality_flags']
