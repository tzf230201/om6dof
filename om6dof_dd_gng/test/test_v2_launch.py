"""Offline launch/provenance checks: no ROS executor, camera, or robot starts."""

import hashlib
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription


PACKAGE = Path(__file__).resolve().parents[1]


@pytest.fixture(autouse=True)
def _temporary_launch_logs(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_log"))


def _load(filename):
    spec = importlib.util.spec_from_file_location(
        filename.replace(".", "_"), PACKAGE / "launch" / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context(version="v1", **overrides):
    context = LaunchContext()
    context.launch_configurations.update({
        "model_version": version, "params_file": "", "camera_model": "auto",
        "camera_serial": "", "camera_calibration_file": "",
        "publish_robot_state": "false",
        "launch_rviz": "false", "launch_reachability": "true",
        "rviz_config": "",
        "graph_method": "gng", "sample_count": "800",
        "halton_start_index": "17", "sample_stream_seed": "0",
        "gng_guard_fraction": "0.25", "query_mode": "false",
        **overrides,
    })
    return context


def _fixture_inputs(tmp_path, monkeypatch, helper):
    moveit = tmp_path / "moveit"
    (moveit / "config").mkdir(parents=True)
    srdf_bytes = (
        b'<?xml version="1.0"?>\n<robot name="om6dof">\n'
        b'<group name="arm"><chain base_link="link1" tip_link="link2"/></group>'
        b'<disable_collisions link1="link1" link2="link2" reason="Adjacent"/>'
        b'<disable_collisions link1="link1" link2="d405_link" reason="Adjacent"/>'
        b'<disable_collisions link1="link1" link2="d405_payload_link" reason="Never"/>'
        b'</robot>\n'
    )
    (moveit / "config" / "om6dof.srdf").write_bytes(srdf_bytes)
    description = tmp_path / "description"
    calls = []

    def expand(argv, **_kwargs):
        calls.append(argv)
        v2 = Path(argv[1]).name == "om6dof_v2.urdf.xacro"
        name = "om6dof_v2" if v2 else "om6dof"
        optical = "d435_depth_optical_frame" if v2 else "d405_link"
        data = (
            f'<robot name="{name}"><link name="world"/><link name="link1"/>'
            f'<link name="link2"/><link name="d405_payload_link"/>'
            f'<link name="{optical}"/></robot>\n'
        ).encode()
        return SimpleNamespace(returncode=0, stdout=data, stderr=b"")

    monkeypatch.setattr(helper.shutil, "which", lambda _name: "/test/xacro")
    monkeypatch.setattr(helper.subprocess, "run", expand)
    return dict(moveit_share=moveit, description_share=description,
                dd_gng_share=PACKAGE), calls, srdf_bytes


def test_v1_model_and_provenance_stay_byte_identical(tmp_path, monkeypatch):
    helper = _load("_model_inputs.py")
    kwargs, calls, srdf_bytes = _fixture_inputs(tmp_path, monkeypatch, helper)
    inputs = helper.resolve_model_inputs(_context(), **kwargs)
    params = inputs["model_parameters"]
    assert Path(calls[0][1]).name == "om6dof.urdf.xacro"
    assert inputs["remappings"] == []
    assert params["robot_description_semantic"].encode() == srdf_bytes
    assert params["srdf_sha256"] == hashlib.sha256(srdf_bytes).hexdigest()
    expected = (PACKAGE / "config" / "topo_gng.yaml").read_bytes()
    assert params["reachability_parameters_sha256"] == hashlib.sha256(expected).hexdigest()


def test_v2_selects_exact_model_and_hashes_adapted_semantics(tmp_path, monkeypatch):
    helper = _load("_model_inputs.py")
    kwargs, calls, original_srdf = _fixture_inputs(tmp_path, monkeypatch, helper)
    inputs = helper.resolve_model_inputs(_context("v2"), **kwargs)
    params = inputs["model_parameters"]
    assert Path(calls[0][1]).name == "om6dof_v2.urdf.xacro"
    assert Path(inputs["params_path"]).name == "topo_gng_v2.yaml"
    urdf = params["robot_description"].encode()
    srdf = params["robot_description_semantic"].encode()
    assert params["expanded_urdf_sha256"] == hashlib.sha256(urdf).hexdigest()
    assert params["srdf_sha256"] == hashlib.sha256(srdf).hexdigest()
    assert srdf != original_srdf
    semantic = ET.fromstring(srdf)
    assert semantic.get("name") == "om6dof_v2"
    assert semantic.find("group").get("name") == "arm"
    pairs = semantic.findall("disable_collisions")
    assert len(pairs) == 1
    assert pairs[0].get("link2") == "link2"
    expected = (PACKAGE / "config" / "topo_gng_v2.yaml").read_bytes()
    assert params["reachability_parameters_sha256"] == hashlib.sha256(expected).hexdigest()


def test_custom_parameter_file_hash_uses_its_exact_bytes(tmp_path, monkeypatch):
    helper = _load("_model_inputs.py")
    kwargs, _calls, _srdf = _fixture_inputs(tmp_path, monkeypatch, helper)
    params_path = tmp_path / "custom.yaml"
    raw = b"reachability_graph_node:\n  ros__parameters:\n    sample_count: 42\n"
    params_path.write_bytes(raw)
    inputs = helper.resolve_model_inputs(
        _context("v2", params_file=str(params_path)), **kwargs)
    assert inputs["params_path"] == str(params_path)
    assert inputs["model_parameters"]["reachability_parameters_sha256"] == hashlib.sha256(raw).hexdigest()


def test_unknown_model_fails_before_xacro(tmp_path, monkeypatch):
    helper = _load("_model_inputs.py")
    kwargs, calls, _srdf = _fixture_inputs(tmp_path, monkeypatch, helper)
    with pytest.raises(RuntimeError, match="model_version"):
        helper.resolve_model_inputs(_context("v3"), **kwargs)
    assert calls == []


@pytest.mark.parametrize("filename", ["topo_gng_node.launch.py", "reachability_graph.launch.py"])
def test_all_v2_consumers_share_isolated_tf_and_model(tmp_path, monkeypatch, filename):
    module = _load(filename)
    kwargs, _calls, _srdf = _fixture_inputs(tmp_path, monkeypatch, module._model_inputs)
    monkeypatch.setattr(module, "Node", lambda **kwargs: SimpleNamespace(**kwargs))
    context = _context("v2", publish_robot_state="true", camera_serial="test-serial")
    nodes = module._launch_setup(context, **kwargs)
    by_executable = {node.executable: node for node in nodes}
    for node in nodes:
        remaps = dict(node.remappings)
        assert remaps["/tf"] == "/om6dof_dd_gng/v2/tf"
        assert remaps["/tf_static"] == "/om6dof_dd_gng/v2/tf_static"
        assert remaps["/robot_description"] == "/om6dof_dd_gng/v2/robot_description"
        assert remaps["/om6dof_topo_gng/environment_graph_data"] == "/om6dof_topo_gng_v2/environment_graph_data"
        assert "/joint_states" not in remaps  # consume measured joints, no fake state
    assert not {"ros2_control_node", "move_group", "joint_state_publisher", "controller_node"} & set(by_executable)
    state = by_executable["robot_state_publisher"]
    assert state.condition.evaluate(context)
    graph = by_executable["reachability_graph_node"]
    assert state.parameters[0]["robot_description"] == graph.parameters[1]["robot_description"]
    assert by_executable["rviz2"].arguments == ["-d", str(PACKAGE / "rviz" / "topo_gng_v2.rviz")]
    if "topo_gng_node" in by_executable:
        overrides = by_executable["topo_gng_node"].parameters[1]
        assert overrides.pop("camera_serial").evaluate(context) == "test-serial"
        assert overrides == {"model_version": "v2", "camera_model": "auto",
                             "camera_frame": "auto"}


def test_empty_serial_does_not_override_profile_or_require_device(tmp_path, monkeypatch):
    module = _load("topo_gng_node.launch.py")
    kwargs, _calls, _srdf = _fixture_inputs(tmp_path, monkeypatch, module._model_inputs)
    monkeypatch.setattr(module, "Node", lambda **kwargs: SimpleNamespace(**kwargs))
    nodes = module._launch_setup(_context("v2"), **kwargs)
    topo = next(node for node in nodes if node.executable == "topo_gng_node")
    assert "camera_serial" not in topo.parameters[1]


def test_measured_hand_eye_artifact_is_hashed_and_forwarded(tmp_path):
    helper = _load("_model_inputs.py")
    artifact = tmp_path / "hand_eye.yaml"
    artifact.write_text("""\
schema: om6dof.hand_eye.v1
camera: {model: D435i, serial: '123456'}
transform:
  parent_frame: end_effector_link
  child_frame: d435_color_optical_frame
  xyz: [0.01, -0.02, 0.03]
  quaternion_xyzw: [0.0, 0.0, 0.0, 1.0]
validation:
  method: opencv_calibrateHandEye_eye_in_hand
  samples: 12
  translation_rmse_m: 0.004
  rotation_rmse_rad: 0.03
""")
    parameters = helper.load_hand_eye_calibration(str(artifact))
    assert parameters["camera_calibration_serial"] == "123456"
    assert parameters["camera_calibration_model"] == "D435I"
    assert parameters["camera_calibration_samples"] == 12
    assert parameters["camera_calibration_sha256"] == hashlib.sha256(
        artifact.read_bytes()).hexdigest()


@pytest.mark.parametrize("field,value,message", [
    ("samples", "7", "at least 8"),
    ("translation_rmse_m", "0.016", "translation RMSE"),
    ("rotation_rmse_rad", "0.081", "rotation RMSE"),
])
def test_hand_eye_artifact_rejects_weak_validation(tmp_path, field, value, message):
    helper = _load("_model_inputs.py")
    artifact = tmp_path / "bad.yaml"
    validation = {
        "samples": "12", "translation_rmse_m": "0.004",
        "rotation_rmse_rad": "0.03", field: value,
    }
    artifact.write_text(f"""\
schema: om6dof.hand_eye.v1
camera: {{model: D435i, serial: '123456'}}
transform:
  parent_frame: end_effector_link
  child_frame: d435_color_optical_frame
  xyz: [0.01, -0.02, 0.03]
  quaternion_xyzw: [0.0, 0.0, 0.0, 1.0]
validation:
  method: opencv_calibrateHandEye_eye_in_hand
  samples: {validation['samples']}
  translation_rmse_m: {validation['translation_rmse_m']}
  rotation_rmse_rad: {validation['rotation_rmse_rad']}
""")
    with pytest.raises(RuntimeError, match=message):
        helper.load_hand_eye_calibration(str(artifact))


def test_v2_profile_has_conservative_mesh_bounds_and_isolated_endpoints():
    config = yaml.safe_load((PACKAGE / "config" / "topo_gng_v2.yaml").read_text())
    for section in config.values():
        params = section["ros__parameters"]
        assert params["body_radius.link3"] == 0.045
        assert params["body_radius.link4"] == 0.045
        assert params["body_radius.d405_payload_link"] == 0.060
        assert params["world_frame"] == "world"
        for value in params.values():
            assert not (isinstance(value, str) and value.startswith("/om6dof_topo_gng/"))
    camera = config["topo_gng_node"]["ros__parameters"]
    assert camera["model_version"] == "v2"
    assert camera["camera_model"] == "auto"
    assert camera["camera_frame"] == "auto"
    assert camera["camera_serial"] == ""
    assert camera["nominal_world_z_offset_m"] == pytest.approx(-0.0185)
    assert camera["target_classes_topic"] == "/om6dof_topo_gng_v2/set_target_classes"
    assert camera["rgb_image_topic"] == "/om6dof_topo_gng_v2/rgb/image/compressed"
    assert camera["yolo_backend"] == "tensorrt"
    assert camera["yolo_engine_path"].endswith("/yolox_s_fp16.engine")
    assert camera["yolo_period_sec"] <= 0.15


def test_v2_rviz_uses_latched_private_description_and_v2_graph_topics():
    config = yaml.safe_load((PACKAGE / "rviz" / "topo_gng_v2.rviz").read_text())
    displays = config["Visualization Manager"]["Displays"]
    robot = next(item for item in displays if item["Class"] == "rviz_default_plugins/RobotModel")
    assert robot["Description Topic"]["Durability Policy"] == "Transient Local"
    assert robot["Description Topic"]["Value"] == "/om6dof_dd_gng/v2/robot_description"
    for item in displays:
        topic = item.get("Topic", {}).get("Value", "")
        assert not topic.startswith("/om6dof_topo_gng/")


@pytest.mark.parametrize("filename", ["topo_gng_v2.launch.py", "reachability_v2.launch.py"])
def test_dedicated_v2_wrappers_enable_read_only_state_publisher(filename):
    module = _load(filename)
    entities = module.generate_launch_description().entities
    args = {item.name: item for item in entities if isinstance(item, DeclareLaunchArgument)}
    context = LaunchContext()
    for action in args.values():
        action.execute(context)
    assert context.launch_configurations["model_version"] == "v2"
    assert context.launch_configurations["publish_robot_state"] == "true"
    assert args["model_version"].choices == ["v2"]
    assert len([item for item in entities if isinstance(item, IncludeLaunchDescription)]) == 1
