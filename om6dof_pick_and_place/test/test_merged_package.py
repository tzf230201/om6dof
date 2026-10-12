"""Verify the unified installed package without starting processes or APIs."""

import os
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest


def test_installed_merged_assets_and_entrypoints():
    share = Path(get_package_share_directory('om6dof_pick_and_place'))
    assert ET.parse(share / 'package.xml').findtext('name') == 'om6dof_pick_and_place'
    for asset in ('config/gemini_pick.yaml', 'config/places.yaml',
                  'config/target_grasp_viewer.rviz', 'config/boxer_pick.yaml',
                  'config/waypoints.yaml', 'docs/gemini.md',
                  'scripts/setup_graspnet_jetson.sh'):
        assert (share / asset).is_file()
    scripts = Path(get_package_prefix('om6dof_pick_and_place')) / 'lib/om6dof_pick_and_place'
    for name in ('gemini_pick_node', 'gemini_probe', 'rgbd_viewer', 'target_grasp_viewer',
                 'pick_place_node', 'direct_pick_node', 'graph_pick_node', 'boxer_pick_node'):
        assert os.access(scripts / name, os.X_OK), name


@pytest.mark.parametrize('filename,start_moveit', [
    ('gemini_pick.launch.py', 'true'),
    ('gemini_pick.launch.py', 'false'),
    ('gemini_target_pick.launch.py', 'true'),
    ('gemini_target_pick.launch.py', 'false'),
    ('rgbd_viewer.launch.py', 'false'),
    ('target_grasp_viewer.launch.py', 'false'),
])
def test_migrated_launches_preserve_motion_defaults(filename, start_moveit, monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'roslog'))
    packages, gemini_params = [], []

    def record_node(action, context):
        package = perform_substitutions(
            context, normalize_to_list_of_substitutions(action.node_package))
        packages.append(package)
        if package == 'om6dof_pick_and_place' and filename.startswith('gemini_'):
            params = evaluate_parameters(context, action._Node__parameters)
            for item in params:
                if isinstance(item, dict):
                    gemini_params.append(item)
                else:
                    assert Path(item).is_file()
        return []

    def reject_process(*args, **kwargs):
        raise AssertionError('No processes may start during launch validation')

    monkeypatch.setattr(Node, 'execute', record_node)
    monkeypatch.setattr(ExecuteProcess, 'execute', reject_process)
    share = Path(get_package_share_directory('om6dof_pick_and_place'))
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / filename)),
            launch_arguments={'start_moveit': start_moveit}.items()),
    ]))
    assert service.run() == 0
    assert 'om6dof_pick_and_place' in packages
    assert not {'controller_manager', 'om6dof_bringup'} & set(packages)
    if filename.startswith('gemini_'):
        assert ('moveit_ros_move_group' in packages) == (start_moveit == 'true')
        assert len(gemini_params) == 1
        for key in ('auto_run', 'execute_motion', 'calibration_validated',
                    'gripper_calibration_validated', 'place_poses_validated'):
            assert gemini_params[0][key] is False
