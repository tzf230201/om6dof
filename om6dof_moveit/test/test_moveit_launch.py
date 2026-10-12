"""Resolve the renamed package and V2 launch without starting any ROS process."""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
import pytest


def test_installed_package_and_moveit_assets():
    share = Path(get_package_share_directory('om6dof_moveit'))
    assert ET.parse(share / 'package.xml').findtext('name') == 'om6dof_moveit'
    for asset in ('om6dof.srdf', 'kinematics.yaml', 'joint_limits.yaml',
                  'moveit_controllers.yaml', 'ompl_planning.yaml',
                  'pilz_cartesian_limits.yaml', 'moveit.rviz'):
        assert (share / 'config' / asset).is_file()


@pytest.mark.parametrize('rviz', ['true', 'false'])
def test_launch_resolves_new_package_without_starting_processes(rviz, monkeypatch):
    packages = []

    def record_node(action, context):
        packages.append(perform_substitutions(
            context, normalize_to_list_of_substitutions(action.node_package)))
        return []

    monkeypatch.setattr(Node, 'execute', record_node)
    path = Path(get_package_share_directory('om6dof_moveit')) / 'launch' / 'om6dof_moveit.launch.py'
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(path)),
            launch_arguments={'start_rviz': rviz}.items()),
    ]))
    assert service.run() == 0
    assert packages == (['moveit_ros_move_group', 'rviz2'] if rviz == 'true'
                        else ['moveit_ros_move_group'])
