"""Construct/evaluate launch actions only; never execute ROS nodes."""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.utilities import evaluate_parameters


@pytest.fixture(autouse=True)
def _temporary_launch_log(monkeypatch, tmp_path):
    import launch.logging
    monkeypatch.setattr(launch.logging.launch_config, "_log_dir", str(tmp_path))


def _module(filename):
    path = Path(__file__).resolve().parents[1] / "launch" / filename
    spec = importlib.util.spec_from_file_location(filename.replace(".", "_"), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context(module, **overrides):
    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            context.launch_configurations[entity.name] = perform_substitutions(
                context, entity.default_value)
    context.launch_configurations.update(overrides)
    return context


def test_generic_launch_preserves_v1_and_serial_string():
    module = _module("perception.launch.py")
    context = _context(module, camera_serial="000123")
    nodes = module._nodes(context)
    assert len(nodes) == 1
    params = evaluate_parameters(context, nodes[0]._Node__parameters)[0]
    assert params["model_version"] == "v1"
    assert params["camera_serial"] == "000123"
    assert params["ee_pixels_calibrated"] is False
    assert params["reacquire_period_sec"] == 2.0
    assert params["frame_rate_hz"] == 20.0
    assert params["web_stream_max_width"] == 0


def test_v2_launch_wires_same_version_into_perception_and_projector():
    module = _module("perception.launch.py")
    context = _context(module, model_version="v2", publish_world_point="true")
    nodes = module._nodes(context)
    assert len(nodes) == 2
    for node in nodes:
        assert evaluate_parameters(context, node._Node__parameters)[0]["model_version"] == "v2"


def test_v1_world_preview_is_rejected_instead_of_using_wrong_kinematics():
    module = _module("perception.launch.py")
    with pytest.raises(ValueError, match="V2 nominal"):
        module._nodes(_context(module, publish_world_point="true"))


def test_v2_wrapper_sets_version_and_allows_disabling_projection(monkeypatch):
    wrapper = _module("perception_v2.launch.py")
    monkeypatch.setattr(wrapper, "get_package_share_directory", lambda name: "/offline/" + name)
    description = wrapper.generate_launch_description()
    version = next(action for action in description.entities
                   if isinstance(action, DeclareLaunchArgument) and action.name == "model_version")
    assert perform_substitutions(LaunchContext(), version.default_value) == "v2"
    include = next(action for action in description.entities
                   if isinstance(action, IncludeLaunchDescription))
    args = dict(include.launch_arguments)
    assert args["model_version"] == "v2"
    context = _context(wrapper, publish_world_point="false")
    assert args["publish_world_point"].perform(context) == "false"
    assert args["camera_model"].perform(context) == "D435i"
    assert args["reacquire_period_sec"].perform(context) == "0.5"
    assert args["frame_rate_hz"].perform(context) == "15.0"
    assert args["web_stream_max_width"].perform(context) == "480"
