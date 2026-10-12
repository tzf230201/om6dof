import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import Shutdown
from launch.events.process import ProcessExited
from launch_ros.actions import Node
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
import yaml


def _load_controller_launch_module():
    path = Path(__file__).parents[1] / "launch" / "controller.launch.py"
    spec = importlib.util.spec_from_file_location("om6dof_controller_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _process_exit(action):
    return ProcessExited(
        returncode=1,
        action=action,
        name="controller",
        cmd=[],
        cwd=None,
        env=None,
        pid=1,
    )


def test_controller_exit_requests_full_launch_shutdown():
    launch_description = _load_controller_launch_module().generate_launch_description()
    controller = next(
        entity for entity in launch_description.entities
        if isinstance(entity, Node)
    )
    registration = next(
        entity for entity in launch_description.entities
        if isinstance(entity, RegisterEventHandler)
    )

    event = _process_exit(controller)
    assert registration.event_handler.matches(event)
    actions = registration.event_handler.handle(event, LaunchContext())

    assert len(actions) == 1
    assert isinstance(actions[0], EmitEvent)
    assert isinstance(actions[0].event, Shutdown)
    assert actions[0].event.reason == "om6dof_controller exited"


def test_controller_exit_handler_ignores_other_processes():
    launch_description = _load_controller_launch_module().generate_launch_description()
    registration = next(
        entity for entity in launch_description.entities
        if isinstance(entity, RegisterEventHandler)
    )
    other = Node(package="other_package", executable="other_node")

    assert not registration.event_handler.matches(_process_exit(other))


def test_merged_launch_uses_new_package_and_preserves_parameter_node_name():
    description = _load_controller_launch_module().generate_launch_description()
    controller = next(entity for entity in description.entities if isinstance(entity, Node))
    context = LaunchContext()
    package = perform_substitutions(
        context, normalize_to_list_of_substitutions(controller.node_package))
    executable = perform_substitutions(
        context, normalize_to_list_of_substitutions(controller.node_executable))
    assert package == 'om6dof_controllers'
    assert executable == 'controller_node'
    config = Path(__file__).parents[1] / 'config' / 'controller.yaml'
    parameters = yaml.safe_load(config.read_text())
    assert list(parameters) == ['om6dof_controller']
    assert 'ready_pose_degrees' in parameters['om6dof_controller']['ros__parameters']
