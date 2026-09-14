"""Resolve launch arguments and parameter files without starting ROS processes."""

import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters, normalize_parameters


PACKAGE = Path(__file__).resolve().parents[1]
BASELINES = [("dd_gng", "ddgng"), ("dbl_gng", "dblgng")]


@pytest.fixture(autouse=True)
def temporary_launch_logs(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))


def resolve_launch(monkeypatch, stem, overrides=None):
    spec = importlib.util.spec_from_file_location(
        f"test_launch_{stem}", PACKAGE / "launch" / f"{stem}.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "get_package_share_directory", lambda name: str(PACKAGE))
    records = {}

    def record_node(**kwargs):
        records[kwargs["name"]] = kwargs
        return Node(**kwargs)

    monkeypatch.setattr(module, "Node", record_node)
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(overrides or {})
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context, records


def mapper_parameters(context, record):
    evaluated = evaluate_parameters(context, normalize_parameters(record["parameters"]))
    merged = {}
    for value in evaluated:
        if isinstance(value, Path):
            document = yaml.safe_load(value.read_text())
            selector = f'/{record["namespace"]}/{record["name"]}'
            assert selector in document, "YAML selector must match the launched ROS node"
            merged.update(document[selector]["ros__parameters"])
        else:
            merged.update(value)
    return merged


@pytest.mark.parametrize("stem,algorithm", BASELINES)
def test_default_profile_resolves_to_native_world_graph(monkeypatch, stem, algorithm):
    context, records = resolve_launch(monkeypatch, stem)
    params = mapper_parameters(context, records[stem])
    assert records[stem]["executable"] == "f_gng_node"
    assert params["algorithm"] == algorithm
    assert params["world_frame"] == "world"
    assert params["camera_frame"] == "d435_depth_optical_frame"
    assert params["memory_mode"] == "world"
    assert params["max_nodes"] == 768
    assert params["memory_capacity"] == 16000
    assert params["input_budget"] == 12000
    assert params["replay_budget"] == 4000
    assert params["edge_support_radius"] == 0.0
    assert "max_edge_length" not in params
    assert not any("focus" in name or "gaze" in name for name in params)
    camera = mapper_parameters(context, records["realsense_depth"])
    assert (camera["min_depth"], camera["max_depth"]) == (
        params["min_depth"], params["max_depth"])
    assert camera["stride"] == 2
    if algorithm == "ddgng":
        assert params["dd_updates"] == 500
    else:
        assert params["dbl_edge_cut_period_frames"] == 10


@pytest.mark.parametrize("stem,algorithm", BASELINES)
def test_external_depth_and_support_arguments_reach_correct_nodes(monkeypatch, stem, algorithm):
    overrides = {
        "start_camera": "false", "use_rviz": "false",
        "depth_topic": "/recording/depth", "camera_info_topic": "/recording/info",
        "world_frame": "map", "camera_frame": "recording_optical",
        "max_nodes": "320", "memory_mode": "current",
        "edge_support_radius": "0.05",
        "edge_cut_period_frames": "7",
        "dd_updates": "125",
    }
    context, records = resolve_launch(monkeypatch, stem, overrides)
    params = mapper_parameters(context, records[stem])
    assert params["algorithm"] == algorithm
    assert params["world_frame"] == "map"
    assert params["camera_frame"] == "recording_optical"
    assert params["memory_mode"] == "current"
    assert params["max_nodes"] == 320 and type(params["max_nodes"]) is int
    assert params["edge_support_radius"] == 0.05
    assert "max_edge_length" not in params
    assert not records["realsense_depth"]["condition"].evaluate(context)
    assert not records[f"{stem}_rviz"]["condition"].evaluate(context)
    for name in [stem, "realsense_depth"]:
        remaps = {
            source: perform_substitutions(context, normalize_to_list_of_substitutions(target))
            for source, target in records[name]["remappings"]
        }
        assert remaps == {"depth/image_raw": "/recording/depth",
                          "depth/camera_info": "/recording/info"}
    if algorithm == "ddgng":
        assert params["dd_updates"] == 125 and type(params["dd_updates"]) is int
    else:
        assert params["dbl_edge_cut_period_frames"] == 7
        assert type(params["dbl_edge_cut_period_frames"]) is int


@pytest.mark.parametrize("stem,algorithm", BASELINES)
def test_rviz_uses_world_graph_and_does_not_offer_foveation(monkeypatch, stem, algorithm):
    context, records = resolve_launch(monkeypatch, stem)
    arguments = records[f"{stem}_rviz"]["arguments"]
    config = yaml.safe_load(Path(arguments[1]).read_text())["Visualization Manager"]
    assert config["Global Options"]["Fixed Frame"] == "world"
    assert config["Views"]["Current"]["Invert Z Axis"] is False
    marker_displays = [display for display in config["Displays"]
                       if display["Class"] == "rviz_default_plugins/MarkerArray"]
    assert {display["Topic"]["Value"] for display in marker_displays} == {
        "/om6dof_f_gng/graph", "/om6dof_f_gng/focus"}
    assert all(display["Topic"]["Durability Policy"] == "Transient Local"
               for display in marker_displays)
    assert not any(tool["Class"] == "rviz_default_plugins/PublishPoint"
                   for tool in config["Tools"])
