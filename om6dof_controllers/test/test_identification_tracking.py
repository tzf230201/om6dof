"""Synthetic offline tests; never construct a ROS node or contact hardware."""
import csv
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'


def module(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / (name+'.py'))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


tracking = module('identification_tracking')
torque = module('check_current_torque')


def feedback(stamp=1., missing=False):
    def point(q, v):
        return NS(positions=[q]*6, velocities=[] if missing else [v]*6,
                  time_from_start=NS(sec=0, nanosec=0))
    return NS(goal_id=NS(uuid=[1]*16), feedback=NS(
        header=NS(stamp=NS(sec=int(stamp), nanosec=round((stamp-int(stamp))*1e9))),
        joint_names=list(reversed(tracking.JOINTS)), desired=point(.2,.02),
        actual=point(.1,.01), error=point(.1,.01)))


def measured(path, stamps):
    with path.open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['ros_stamp_s'] + [f'{j}_{field}' for j in tracking.JOINTS
            for field in ('position_rad','velocity_rad_s','effort_raw')])
        for stamp in stamps:
            w.writerow([stamp]+[v for _ in tracking.JOINTS for v in (.1,.01,20.)])


def test_feedback_and_current_join_by_source_stamp_not_receipt_time(tmp_path):
    path = tmp_path/'data.csv'
    measured(path, [1.01, 1.11])
    recorder = tracking.TrackingRecorder(path)
    recorder.write(feedback(1.))
    recorder.write(feedback(1.1))
    recorder.write(feedback(2.))  # Must not carry forward stale current.
    recorder.close()
    report, rows = tracking.analyze(path, .025)
    assert report['matched_samples'] == 2
    assert report['unmatched_samples'] == 1
    assert report['joints']['joint3']['tracking_rmse_rad'] == pytest.approx(.1)
    assert rows[0]['joint3_effort_raw'] == 20
    assert rows[0]['stamp_skew_s'] == pytest.approx(.01)
    assert 'low_timestamp_match_coverage' in report['quality_flags']


def test_missing_velocity_remains_nan_and_name_mapping_is_correct(tmp_path):
    path = tmp_path/'data.csv'
    recorder = tracking.TrackingRecorder(path)
    message = feedback(missing=True)
    message.feedback.desired.positions = [1,2,3,4,5,6]
    recorder.write(message)
    recorder.close()
    with tracking.tracking_path(path).open() as f:
        row = next(csv.DictReader(f))
    assert float(row['joint1_desired_position_rad']) == 6
    assert np.isnan(float(row['joint3_actual_velocity_rad_s']))
    original = tracking.tracking_path(path).read_bytes()
    with pytest.raises(FileExistsError):
        tracking.TrackingRecorder(path)
    assert tracking.tracking_path(path).read_bytes() == original


@pytest.mark.parametrize('stamps', [[0.,1.], [1.,1.], [2.,1.]])
def test_invalid_time_never_silently_aligns(tmp_path, stamps):
    path = tmp_path/'data.csv'
    measured(path, stamps)
    with pytest.raises(ValueError):
        tracking.read_rows(path, 'ros_stamp_s')


def test_negative_current_gain_is_preserved_and_validation_not_refitted():
    current = np.array([-30.,-20.,-10.,10.,20.,30.])
    known_torque = -.01*current + .002
    train = (known_torque,current,[f't{i}' for i in range(6)])
    test = (known_torque+.01,current,[f'v{i}' for i in range(6)])
    report = torque.fit_reference(train,test)
    assert report['nm_per_raw'] == pytest.approx(-.01)
    assert report['bias_nm'] == pytest.approx(.002)
    assert report['candidate_polarity'] == 'negative'
    assert report['independent_validation']['rmse_nm'] == pytest.approx(.01)
    assert not report['deployable']
    with pytest.raises(ValueError, match='overlap'):
        torque.fit_reference(train,train)


@pytest.mark.parametrize('bad', ['moving','duplicate','nonfinite','one_direction'])
def test_reference_quality_rejects_invalid_input(tmp_path, bad):
    path = tmp_path/'reference.csv'
    with path.open('w', newline='') as f:
        w=csv.writer(f)
        w.writerow(torque.FIELDS)
        for i, value in enumerate([-3,-2,-1,1,2,3]):
            w.writerow(['joint3', 'same' if bad=='duplicate' else str(i),
                        abs(value) if bad=='one_direction' else value,
                        float('nan') if bad=='nonfinite' else 100*value,
                        .1 if bad=='moving' else 0.])
    with pytest.raises(ValueError):
        torque.load_reference(path,'joint3',.005)
