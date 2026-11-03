"""Inspect loader only: never launch the spawner or hardware."""
import importlib.util
from pathlib import Path

import yaml
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def test_loader_is_inactive_only_and_has_no_hardware_or_automatic_ready_move():
    root = Path(__file__).parents[1]
    path = root / 'launch/position_impedance.launch.py'
    spec = importlib.util.spec_from_file_location('impedance_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    actions = module.generate_launch_description().entities
    assert all(isinstance(action, (Node, DeclareLaunchArgument)) for action in actions)
    nodes = [action for action in actions if isinstance(action, Node)]
    assert len(nodes) == 4
    assert "executable='impedance_target'" in path.read_text()
    assert "package='om6dof_teleop'" in path.read_text()
    text = path.read_text()
    assert "'--inactive'" in text
    assert "package='controller_manager'" in text
    assert "executable='spawner'" in text
    config = yaml.safe_load((root / 'config/position_impedance.yaml').read_text())['/**']
    p = config['om6dof_position_impedance_controller']['ros__parameters']
    assert p['enable_on_activation'] is False
    assert p['allow_mock_hardware'] is False
    assert p['require_bus_health'] is True
    assert p['update_rate'] == 100
    assert p['friction_compensation'] is False
    assert all(value > 0 for value in p['max_effort'])
    assert p['enable_cartesian_wrench'] is True
    assert p['max_cartesian_wrench'] == [4.0, 4.0, 4.0, 0.2, 0.2, 0.2]
    force = yaml.safe_load((root / 'config/cartesian_force.yaml').read_text())
    assert force['om6dof_cartesian_force_bridge']['ros__parameters']['wrench_limits'] == [4.0, 4.0, 4.0, 0.2, 0.2, 0.2]
