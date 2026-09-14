import hashlib
import importlib.util
from pathlib import Path

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('experiment_launch', PACKAGE / 'launch/ddgng_experiment_reachability.launch.py')
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)


def test_separate_launch_preserves_environment_and_freezes_dataset(tmp_path):
    base = PACKAGE / 'config/topo_gng_v2.yaml'
    dataset = tmp_path / 'points.csv'
    data = ('x_mm,y_mm,z_mm,position_found,q1_rad,q2_rad,q3_rad,q4_rad,q5_rad,q6_rad\n'
            '100,0,200,1,0,0,0,0,0,0\n0,0,0,0,,,,,,\n').encode()
    dataset.write_bytes(data)
    run = tmp_path / 'run'
    run.mkdir()
    config = launch.prepare_config(base, dataset, run)
    document = yaml.safe_load(config.read_text())
    original = yaml.safe_load(base.read_text())
    for key in original:
        if key != 'reachability_graph_node':
            assert document[key] == original[key]
    p = document['reachability_graph_node']['ros__parameters']
    assert p['graph_method'] == 'workspace_samples'
    assert p['sample_count'] == 1
    assert p['exact_collision_enabled'] and p['include_target_in_collision']
    assert not p['target_refinement_enabled']
    assert p['workspace_samples_sha256'] == hashlib.sha256(data).hexdigest()
    dataset.write_text('changed after launch')
    assert Path(p['workspace_samples_file']).read_bytes() == data


def test_invalid_dataset_fails_before_launching_nodes(tmp_path):
    dataset = tmp_path / 'points.csv'
    dataset.write_text('x_mm,y_mm,z_mm\n1,2,3\n')
    with pytest.raises(ValueError, match='q1_rad'):
        launch.prepare_config(PACKAGE / 'config/topo_gng_v2.yaml', dataset, tmp_path)


def test_execution_is_opt_in(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_log"))
    from launch.actions import DeclareLaunchArgument
    args = {a.name: a for a in launch.generate_launch_description().entities
            if isinstance(a, DeclareLaunchArgument)}
    from launch import LaunchContext
    from launch.utilities import perform_substitutions
    assert perform_substitutions(LaunchContext(), args['execution_enabled'].default_value) == 'false'
