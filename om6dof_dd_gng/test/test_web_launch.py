"""Construct launch actions without opening a camera or starting a ROS node."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


PACKAGE = Path(__file__).resolve().parents[1]


@pytest.fixture(autouse=True)
def _temporary_launch_logs(monkeypatch, tmp_path):
    import launch.logging
    monkeypatch.setattr(launch.logging.launch_config, "_log_dir", str(tmp_path))


def load(filename):
    spec = importlib.util.spec_from_file_location("web_launch", PACKAGE / "launch" / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def command(monkeypatch, **overrides):
    module = load("dd_gng_yolo.launch.py")
    context = LaunchContext()
    context.launch_configurations.update({
        "model_version": "v2", "camera_model": "auto", "camera_serial": "",
        "camera_fps": "30", "headless": "false",
        "image_topic": "/application_web_monitor/ddgng/image/compressed",
        "labels_topic": "/application_web_monitor/ddgng/labels", **overrides,
    })
    monkeypatch.setattr(module, "get_package_prefix", lambda _name: "/test/install")
    monkeypatch.setattr(module, "ExecuteProcess", lambda **kwargs: SimpleNamespace(**kwargs))
    actions = module._launch_setup(context)
    assert len(actions) == 1
    return actions[0].cmd


def test_auto_camera_needs_no_empty_launch_serial(monkeypatch):
    cmd = command(monkeypatch)
    assert cmd[1] == "/test/install/lib/om6dof_dd_gng/dd_gng_yolo.py"
    assert cmd[cmd.index("--model-version") + 1] == "v2"
    assert cmd[cmd.index("--camera-model") + 1] == "auto"
    assert "--camera-serial" not in cmd
    assert "--headless" not in cmd
    assert "--ros-args" not in cmd


def test_web_headless_serial_is_preserved_as_string(monkeypatch):
    cmd = command(monkeypatch, headless="true", camera_serial="0012345", camera_model="D435i")
    assert "--headless" in cmd
    assert cmd[cmd.index("--camera-serial") + 1] == "0012345"
    assert cmd[cmd.index("--camera-model") + 1] == "D435i"
    assert "/application_web_monitor/ddgng/labels" in cmd


@pytest.mark.parametrize("filename,version", [
    ("dd_gng_yolo.launch.py", "v1"), ("dd_gng_yolo_v2.launch.py", "v2"),
])
def test_model_defaults(filename, version):
    desc = load(filename).generate_launch_description()
    argument = next(action for action in desc.entities
                    if isinstance(action, DeclareLaunchArgument) and action.name == "model_version")
    context = LaunchContext()
    argument.execute(context)
    assert context.launch_configurations["model_version"] == version
