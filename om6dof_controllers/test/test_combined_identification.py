"""Combined profile checks without constructing a ROS node or moving hardware."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
spec = importlib.util.spec_from_file_location('combined_runner', SCRIPTS/'multisine_identification.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


def node():
    n = SimpleNamespace(_args=SimpleNamespace(profile='teaching_combined', joints=None,
        amplitude_scale=.5, frequency_scale=.5, duration=60., period=.05),
        _latest=SimpleNamespace(name=list(runner.ARM_JOINTS),
            position=[0.,-1.0472,.2618,0.,1.0472,0.]))
    for name in ('_omegas','_coefficients','_excited_joints'):
        method=getattr(runner.IdentificationRun,name)
        setattr(n,name,lambda *args, method=method: method(n,*args))
    return n


def test_joint_specific_patterns_start_at_rest_and_stay_bounded():
    n=node()
    patterns=[]
    frequencies=[]
    for name in n._excited_joints():
        a,w=n._coefficients(name),n._omegas(name)
        frequencies.append(w)
        assert runner.multisine(a,w,0.) == pytest.approx((0.,0.),abs=1e-12)
        assert sum(map(abs,a)) < .065  # <3.73 deg offset bound at fixed web scales
        assert sum(abs(ak*wk) for ak,wk in zip(a,w)) < .03
        pattern=np.array([runner.multisine(a,w,t)[0] for t in np.linspace(0,60,601)])
        assert np.max(np.abs(pattern)) <= sum(map(abs,a))
        patterns.append(pattern/np.linalg.norm(pattern))
    assert len(set(frequencies)) == 6
    for i,p in enumerate(patterns):
        for other in patterns[:i]:
            assert not np.allclose(p,other)


def test_trajectory_uses_each_joint_profile_and_keeps_all_six_names():
    n=node()
    goal=runner.IdentificationRun._trajectory(n)
    assert goal.trajectory.joint_names == list(runner.ARM_JOINTS)
    for point in goal.trajectory.points[:-1:37]:
        t=point.time_from_start.sec+point.time_from_start.nanosec*1e-9
        for i,name in enumerate(runner.ARM_JOINTS):
            offset,velocity=runner.multisine(n._coefficients(name),n._omegas(name),t)
            assert point.positions[i] == pytest.approx(n._latest.position[i]+offset)
            assert point.velocities[i] == pytest.approx(velocity)
            lower,upper=runner.JOINT_LIMITS[name]
            assert lower+runner.LIMIT_MARGIN < point.positions[i] < upper-runner.LIMIT_MARGIN
    assert list(goal.trajectory.points[-1].velocities) == [0.]*6


def test_existing_profile_frequency_behavior_unchanged():
    n=node()
    n._args.profile='teaching_all'
    for name in runner.ARM_JOINTS:
        assert n._omegas(name) == (.25,.4,.6)
