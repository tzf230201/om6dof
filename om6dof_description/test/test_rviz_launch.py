"""Check RViz conditions through the actual launch engine without a display."""

from pathlib import Path

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
import pytest


@pytest.mark.parametrize('rviz', ['true', 'false'])
@pytest.mark.parametrize('launch_name', ['view_dynamic.launch.py', 'view_jacobian.launch.py'])
def test_rviz_argument_and_read_only_hardware_launch(rviz, launch_name, monkeypatch):
    path = Path(__file__).parents[1] / 'launch' / launch_name
    executed_packages = []

    def record_node(action, context):
        executed_packages.append(perform_substitutions(
            context, normalize_to_list_of_substitutions(action.node_package)))
        preset = 'jacobian.rviz' if 'jacobian' in launch_name else 'dynamic.rviz'
        assert Path(context.launch_configurations['rviz_config']).name == preset
        assert context.launch_configurations['jacobian_scale'] == '0.6'
        return []

    monkeypatch.setattr(Node, 'execute', record_node)
    service = LaunchService()
    # ros2 launch supplies CLI launch arguments through a root include.
    service.include_launch_description(LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(path)),
            launch_arguments={'rviz': rviz, 'jacobian_scale': '0.6'}.items()),
    ]))
    assert service.run() == 0
    assert executed_packages.count('rviz2') == (1 if rviz == 'true' else 0)
    assert executed_packages.count('robot_state_publisher') == 0
    assert executed_packages.count('joint_state_publisher_gui') == 0
    assert executed_packages.count('om6dof_description') == 1
    assert executed_packages.count('controller_manager') == 0
    assert executed_packages.count('om6dof_dynamic') == 0
