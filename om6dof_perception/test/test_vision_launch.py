"""Package/launch migration checks: no camera, inference, or ROS process starts."""

import os
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
import pytest


def test_installed_assets_and_executables():
    share = Path(get_package_share_directory('om6dof_perception'))
    assert ET.parse(share / 'package.xml').findtext('name') == 'om6dof_perception'
    for asset in ('launch/boxer_only.launch.py', 'launch/yolox_boxer3d_onnx.launch.py',
                  'rviz/boxer.rviz', 'README.md', 'docs/boxer.md',
                  'docs/camera_utilities.md', 'requirements.txt'):
        assert (share / asset).is_file()
    scripts = Path(get_package_prefix('om6dof_perception')) / 'lib/om6dof_perception'
    for name in ('boxer_preview.py', 'barath_boxer3d_preview.py', 'yolo3d_pickup_bridge.py',
                 'event_camera_simulator.py', 'realsense_rgb_viewer.py',
                 'perception_node', 'perception_view', 'world_projection_node'):
        assert os.access(scripts / name, os.X_OK), name
    for name in ('boxer_adapter.py', 'barath_boxer3d_adapter.py', 'boxer_pick_geometry.py'):
        assert (scripts / name).is_file()


@pytest.mark.parametrize('rviz', ['true', 'false'])
@pytest.mark.parametrize('filename,script,detector', [
    ('boxer_only.launch.py', 'boxer_preview.py', False),
    ('yolox_boxer3d_onnx.launch.py', 'barath_boxer3d_preview.py', True),
])
def test_launch_resolves_without_starting_processes(filename, script, detector, rviz, monkeypatch):
    packages, commands = [], []

    def resolve(context, value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))

    def record_node(action, context):
        packages.append(resolve(context, action.node_package))
        return []

    def record_process(action, context):
        commands.append([resolve(context, part) for part in action.cmd])
        return []

    monkeypatch.setattr(Node, 'execute', record_node)
    monkeypatch.setattr(ExecuteProcess, 'execute', record_process)
    share = Path(get_package_share_directory('om6dof_perception'))
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / filename)),
            launch_arguments={'launch_rviz': rviz}.items()),
    ]))
    assert service.run() == 0
    expected = ['robot_state_publisher']
    if detector:
        expected.append('om6dof_dd_gng')
    if rviz == 'true':
        expected.append('rviz2')
    assert packages == expected
    assert len(commands) == 1
    installed = Path(get_package_prefix('om6dof_perception')) / 'lib/om6dof_perception' / script
    assert commands[0][2] == str(installed)
    assert installed.is_file()
    if detector:
        flag = commands[0].index('--publish-pick-detections')
        assert commands[0][flag + 1] == 'false'
