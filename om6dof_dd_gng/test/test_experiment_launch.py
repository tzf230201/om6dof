import hashlib
import importlib.util
import csv
import json
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
    config = launch.prepare_config(base, dataset, run, sample_selection='all')
    document = yaml.safe_load(config.read_text())
    original = yaml.safe_load(base.read_text())
    for key in original:
        if key != 'reachability_graph_node':
            assert document[key] == original[key]
    p = document['reachability_graph_node']['ros__parameters']
    assert p['graph_method'] == 'workspace_samples'
    assert p['sample_count'] == 1
    assert p['exact_collision_enabled'] and p['include_target_in_collision']
    assert not p['exclude_selected_target_from_collision']
    assert p['capsule_collision_veto']
    assert not p['target_refinement_enabled']
    assert p['target_node_selection'] == 'component_center'
    assert p['exact_max_replans'] == 80
    assert p['allowed_self_collision_pairs'] == ['link2:link6']
    assert p['workspace_samples_sha256'] == hashlib.sha256(data).hexdigest()
    dataset.write_text('changed after launch')
    assert Path(p['workspace_samples_file']).read_bytes() == data


def test_invalid_dataset_fails_before_launching_nodes(tmp_path):
    dataset = tmp_path / 'points.csv'
    dataset.write_text('x_mm,y_mm,z_mm\n1,2,3\n')
    with pytest.raises(ValueError, match='q1_rad'):
        launch.prepare_config(PACKAGE / 'config/topo_gng_v2.yaml', dataset, tmp_path)


def test_legacy_center_candidate_argument_does_not_change_centre_mode(tmp_path):
    dataset = tmp_path / 'points.csv'
    dataset.write_text('x_mm,y_mm,z_mm,position_found,q1_rad,q2_rad,q3_rad,q4_rad,q5_rad,q6_rad\n'
                       '100,0,200,1,0,0,0,0,0,0\n')
    config = launch.prepare_config(PACKAGE / 'config/topo_gng_v2.yaml', dataset, tmp_path,
                                   sample_selection='all', center_candidates=0)
    assert yaml.safe_load(config.read_text())['reachability_graph_node']['ros__parameters'][
        'target_node_selection'] == 'component_center'
    with pytest.raises(ValueError, match='exact_replan_budget'):
        launch.prepare_config(PACKAGE / 'config/topo_gng_v2.yaml', dataset, tmp_path,
                              sample_selection='all', exact_replan_budget=0)


def test_execution_is_opt_in(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_log"))
    from launch.actions import DeclareLaunchArgument
    args = {a.name: a for a in launch.generate_launch_description().entities
            if isinstance(a, DeclareLaunchArgument)}
    from launch import LaunchContext
    from launch.utilities import perform_substitutions
    assert perform_substitutions(LaunchContext(), args['execution_enabled'].default_value) == 'false'
    assert perform_substitutions(LaunchContext(), args['sample_selection'].default_value) == 'green'


def test_green_selection_excludes_blue_yellow_and_inconsistent_flags(tmp_path):
    dataset = tmp_path / 'points.csv'
    header = 'x_mm,y_mm,z_mm,position_found,pose_found,status,q1_rad,q2_rad,q3_rad,q4_rad,q5_rad,q6_rad\n'
    rows = ['100,0,200,1,1,pose_found,0,0,0,0,0,0',
            '125,0,200,1,1,pose_near_singular,0,0,0,0,0,0',
            '150,0,200,1,0,orientation_unresolved,0,0,0,0,0,0',
            '175,0,200,0,1,pose_found,0,0,0,0,0,0',
            '200,0,200,1,0,pose_found,0,0,0,0,0,0']
    original = (header + '\n'.join(rows) + '\n').encode()
    dataset.write_bytes(original)
    config = launch.prepare_config(PACKAGE / 'config/topo_gng_v2.yaml', dataset, tmp_path)
    p = yaml.safe_load(config.read_text())['reachability_graph_node']['ros__parameters']
    snapshot = Path(p['workspace_samples_file']).read_bytes()
    selected = list(csv.DictReader(snapshot.decode().splitlines()))
    assert len(selected) == p['sample_count'] == 1
    assert selected[0]['x_mm'] == '100'
    assert p['workspace_samples_sha256'] == hashlib.sha256(snapshot).hexdigest()
    manifest = json.loads((tmp_path / 'workspace_samples_manifest.json').read_text())
    assert manifest['source_sha256'] == hashlib.sha256(original).hexdigest()
    assert manifest['sample_selection'] == 'green'
    assert manifest['selected_witnesses'] == 1
    assert dataset.read_bytes() == original


def test_grasp_profile_adds_insertion_without_changing_perception(tmp_path):
    dataset = tmp_path / 'points.csv'
    dataset.write_text('x_mm,y_mm,z_mm,position_found,pose_found,status,q1_rad,q2_rad,q3_rad,q4_rad,q5_rad,q6_rad\n'
                       '100,0,200,1,1,pose_found,0,0,0,0,0,0\n')
    base = PACKAGE / 'config/topo_gng_v2.yaml'
    document = yaml.safe_load(launch.prepare_config(
        base, dataset, tmp_path, final_grasp=True).read_text())
    original = yaml.safe_load(base.read_text())
    for key in original:
        if key != 'reachability_graph_node':
            assert document[key] == original[key]
    p = document['reachability_graph_node']['ros__parameters']
    assert p['final_grasp_enabled'] and p['include_target_in_collision']
    assert p['exclude_selected_target_from_collision']
    assert p['exact_collision_enabled']
    assert not p['capsule_collision_veto']
    assert p['pregrasp_tool_approach_axis'] == [0., 0., 1.]
    assert p['pregrasp_min_alignment'] == .95
    assert p['grasp_tcp_to_pinch'] == [0., 0., 0.]
    assert p['grasp_gripper_open'] > p['grasp_gripper_close']


@pytest.mark.parametrize('pickup', [False, True])
def test_launch_generated_planner_and_coordinator_target_policies_match(
        tmp_path, monkeypatch, pickup):
    from launch import LaunchContext
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_log'))
    run = tmp_path / 'generated'
    run.mkdir()
    monkeypatch.setattr(launch.tempfile, 'mkdtemp', lambda **_: str(run))
    monkeypatch.setattr(launch, 'get_package_share_directory',
                        lambda package: str(PACKAGE.parent / package))
    dataset = tmp_path / 'points.csv'
    dataset.write_text('x_mm,y_mm,z_mm,position_found,pose_found,status,q1_rad,q2_rad,q3_rad,q4_rad,q5_rad,q6_rad\n'
                       '100,0,200,1,1,pose_found,0,0,0,0,0,0\n')
    context = LaunchContext()
    context.launch_configurations.update({
        'pickup': str(pickup).lower(), 'workspace_samples_file': str(dataset),
        'sample_selection': 'green', 'component_center_candidates': '1',
        'exact_replan_budget': '80',
    })
    # Builds launch actions and frozen YAML only; no nodes or hardware execute.
    assert launch.setup(context)
    planner = yaml.safe_load((run / 'experiment_topology.yaml').read_text())[
        'reachability_graph_node']['ros__parameters']
    pick_path = (run / 'graph_grasp.yaml' if pickup else
                 PACKAGE.parent / 'om6dof_pick_and_place/config/graph_pick.yaml')
    coordinator = yaml.safe_load(pick_path.read_text())['graph_pick']['ros__parameters']
    assert planner['exclude_selected_target_from_collision'] is pickup
    assert coordinator['selected_target_excluded_from_collision'] is pickup
    assert planner['exact_collision_enabled'] and planner['include_target_in_collision']
    assert coordinator['require_calibration_verified']
    assert coordinator['task_mode'] == ('pickup' if pickup else 'move_to_target')


@pytest.mark.parametrize('extra_header,extra_value,error', [
    ('', '', 'requires status and pose_found'),
    (',pose_found,status', ',1,pose_near_singular', 'no green'),
])
def test_green_selection_fails_closed(tmp_path, extra_header, extra_value, error):
    dataset = tmp_path / 'points.csv'
    dataset.write_text('x_mm,y_mm,z_mm,position_found,q1_rad,q2_rad,q3_rad,q4_rad,q5_rad,q6_rad'
                       + extra_header + '\n100,0,200,1,0,0,0,0,0,0' + extra_value + '\n')
    with pytest.raises(ValueError, match=error):
        launch.prepare_config(PACKAGE / 'config/topo_gng_v2.yaml', dataset, tmp_path)
