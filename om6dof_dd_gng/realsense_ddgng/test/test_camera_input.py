"""Device-free camera contract and registration regression tests."""

import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import camera_input
from camera_input import (
    CameraFrame, RealSenseInput, camera_contract, project_camera_points, sample_camera_points,
)
from dd_gng_yolo import parse_args as yolo_args, segment_3d_box
from ddgng_realsense import parse_args as basic_args


def arguments(*options):
    return basic_args(list(options))


def test_symlink_install_prefers_active_overlay_library(tmp_path, monkeypatch):
    import ament_index_python.packages
    import ddgng_realsense
    source_dir = tmp_path / 'source'
    source_library = source_dir / 'build_om6dof' / 'libddgng.so'
    installed_library = tmp_path / 'install/lib/om6dof_dd_gng/libddgng.so'
    for path in (source_library, installed_library):
        path.parent.mkdir(parents=True)
        path.touch()
    monkeypatch.setattr(ddgng_realsense, '__file__', str(source_dir / 'ddgng_realsense.py'))
    monkeypatch.setattr(ament_index_python.packages, 'get_package_prefix',
                        lambda _name: str(tmp_path / 'install'))
    loader = Mock(return_value=Mock())
    monkeypatch.setattr(ddgng_realsense.ctypes, 'CDLL', loader)
    ddgng_realsense.load_lib()
    loader.assert_called_once_with(str(installed_library))


def fake_sdk(names=('Intel RealSense D435i',), rates=(6, 30), start_error=None):
    intr = SimpleNamespace(width=2, height=2, fx=100., fy=100., ppx=0., ppy=0.,
                           model='none', coeffs=[0.] * 5)
    extr = SimpleNamespace(rotation=np.eye(3).flatten(order='F'),
                           translation=[0.015, 0., 0.])
    profiles = {}
    for kind, fmt in [('color', 'bgr8'), ('depth', 'z16')]:
        for fps in rates:
            stream = Mock()
            stream.is_video_stream_profile.return_value = True
            stream.as_video_stream_profile.return_value = stream
            stream.width.return_value = 2
            stream.height.return_value = 2
            stream.stream_type.return_value = kind
            stream.format.return_value = fmt
            stream.fps.return_value = fps
            stream.get_intrinsics.return_value = intr
            stream.get_extrinsics_to.return_value = extr
            profiles[kind, fps] = stream
    devices = []
    for index, name in enumerate(names):
        device = Mock()
        identity = {'name': name, 'serial': f'unit-{index}'}
        device.get_info.side_effect = identity.__getitem__
        sensor = Mock()
        sensor.get_stream_profiles.return_value = list(profiles.values())
        sensor.get_depth_scale.return_value = 0.001
        device.query_sensors.return_value = [sensor]
        device.first_depth_sensor.return_value = sensor
        devices.append(device)
    sdk = SimpleNamespace(
        stream=SimpleNamespace(color='color', depth='depth'),
        format=SimpleNamespace(bgr8='bgr8', z16='z16'),
        camera_info=SimpleNamespace(name='name', serial_number='serial'),
        context=Mock(), config=Mock(), pipeline=Mock(), align=Mock(), pointcloud=Mock(),
    )
    sdk.context.return_value.query_devices.return_value = devices
    active = sdk.pipeline.return_value.start.return_value
    active.get_device.return_value = devices[0]
    active.get_stream.side_effect = lambda kind: profiles[kind, rates[-1]]
    if start_error is not None:
        sdk.pipeline.return_value.start.side_effect = start_error
    frames = sdk.pipeline.return_value.wait_for_frames.return_value
    color, depth = Mock(), Mock()
    color.profile = profiles['color', rates[-1]]
    color.get_data.return_value = np.zeros((2, 2, 3), dtype=np.uint8)
    depth.get_data.return_value = np.full((2, 2), 500, dtype=np.uint16)
    frames.get_color_frame.return_value = color
    frames.get_depth_frame.return_value = depth
    sdk.align.return_value.process.return_value = frames
    points = sdk.pointcloud.return_value.calculate.return_value
    points.get_vertices.return_value = np.array(
        [[0., 0., .5], [.01, 0., .5], [0., .01, .5], [.01, .01, .5]], dtype=np.float32)
    points.get_texture_coordinates.return_value = np.array(
        [[0., 0.], [.5, 0.], [0., .5], [.5, .5]], dtype=np.float32)
    return sdk


@pytest.mark.parametrize('parse', [basic_args, yolo_args])
def test_cli_preserves_v1_default_and_does_not_require_serial(parse):
    args = parse([])
    assert args.model_version == 'v1'
    assert args.camera_serial == ''
    assert camera_contract(args).camera_model == 'D405'


def test_v2_default_is_d435i_and_plain_d435_is_explicit():
    assert camera_contract(arguments('--model-version', 'v2')).camera_model == 'D435i'
    assert camera_contract(arguments('--model-version', 'v2', '--camera-model', 'D435')).camera_model == 'D435'
    with pytest.raises(ValueError):
        camera_contract(arguments('--camera-model', 'D435i'))


def test_one_matching_camera_selects_automatically_without_device_access_in_constructor():
    sdk = fake_sdk()
    camera = RealSenseInput(sdk, arguments('--model-version', 'v2'), 2, 2)
    sdk.context.assert_not_called()
    camera.start()
    sdk.config.return_value.enable_device.assert_called_once_with('unit-0')
    assert camera.metadata['camera_model'] == 'D435i'
    assert camera.metadata['frame'] == 'd435_color_optical_frame'
    assert camera.metadata['coordinate_space'] == 'camera_optical'
    assert camera.metadata['units'] == 'metres'
    assert camera.metadata['world_projection'] is False
    assert camera.metadata['robot_kinematics_applied'] is False


def test_multiple_same_model_requires_serial_and_no_other_variant_fallback():
    sdk = fake_sdk(('D435i', 'D435i'))
    with pytest.raises(ValueError, match='Multiple'):
        RealSenseInput(sdk, arguments('--model-version', 'v2'), 2, 2).start()
    sdk.pipeline.assert_not_called()
    RealSenseInput(sdk, arguments('--model-version', 'v2', '--camera-serial', 'unit-1'), 2, 2).start()
    sdk.config.return_value.enable_device.assert_called_with('unit-1')
    sdk = fake_sdk(('D435',))
    with pytest.raises(ValueError, match='No D435i'):
        RealSenseInput(sdk, arguments('--model-version', 'v2'), 2, 2).start()
    sdk.pipeline.assert_not_called()


def test_explicit_d435_streams_with_its_own_identity():
    camera = RealSenseInput(fake_sdk(('D435',)),
                           arguments('--model-version', 'v2', '--camera-model', 'D435'), 2, 2).start()
    assert camera.metadata['camera_model'] == 'D435'


def test_low_light_fps_comes_from_device_not_d405_constant(monkeypatch):
    monkeypatch.setattr(camera_input, 'configure_depth_sensor', lambda *args: 'depth configured')
    monkeypatch.setattr(camera_input, 'configure_color_sensor', lambda *args: 'RGB configured')
    sdk = fake_sdk(rates=(6, 15, 30))
    camera = RealSenseInput(sdk, arguments('--model-version', 'v2'), 2, 2,
                           {'enabled': True}).start()
    assert camera.metadata['fps'] == 6
    assert all(call.args[-1] == 6 for call in sdk.config.return_value.enable_stream.call_args_list)
    with pytest.raises(ValueError, match='unsupported'):
        RealSenseInput(fake_sdk(), arguments('--model-version', 'v2', '--camera-fps', '5'), 2, 2).start()


def test_normal_v2_does_not_disable_emitter(monkeypatch):
    configure_depth = Mock()
    monkeypatch.setattr(camera_input, 'configure_depth_sensor', configure_depth)
    monkeypatch.setattr(camera_input, 'configure_color_sensor', lambda *args: 'RGB unchanged')
    RealSenseInput(fake_sdk(), arguments('--model-version', 'v2'), 2, 2,
                  {'enabled': False}).start()
    configure_depth.assert_not_called()


def test_start_failure_stops_pipeline_and_close_is_idempotent():
    sdk = fake_sdk(start_error=RuntimeError('bad profile'))
    camera = RealSenseInput(sdk, arguments('--model-version', 'v2'), 2, 2)
    with pytest.raises(RuntimeError, match='bad profile'):
        camera.start()
    camera.close()
    sdk.pipeline.return_value.stop.assert_called_once()


def test_v2_registers_native_vertices_in_rgb_frame_metres():
    sdk = fake_sdk()
    camera = RealSenseInput(sdk, arguments('--model-version', 'v2'), 2, 2).start()
    frame = camera.read()
    sdk.align.assert_not_called()
    frames = sdk.pipeline.return_value.wait_for_frames.return_value
    sdk.pointcloud.return_value.map_to.assert_called_once_with(frames.get_color_frame.return_value)
    sdk.pointcloud.return_value.calculate.assert_called_once_with(frames.get_depth_frame.return_value)
    assert frame.depth_scale == 1.0
    np.testing.assert_allclose(frame.points_xyz[0, 0], [.015, 0., .5])
    np.testing.assert_allclose(frame.depth, .5)
    sampled = sample_camera_points(frame, 1, .2, 4.)
    assert sampled.dtype == np.float64
    assert sampled.flags.c_contiguous
    assert sampled.shape == (4, 3)
    np.testing.assert_allclose(sampled[0], [.015, 0., .5])


def test_v1_retains_aligned_depth_pinhole_path():
    sdk = fake_sdk(('D405',), rates=(5, 30))
    camera = RealSenseInput(sdk, arguments(), 2, 2).start()
    frame = camera.read()
    sdk.align.return_value.process.assert_called_once()
    sdk.pointcloud.assert_not_called()
    assert frame.points_xyz is None
    np.testing.assert_allclose(sample_camera_points(frame, 1, .2, 4.)[1], [.005, 0., .5])


def test_registered_segment_uses_actual_xyz_not_pinhole_reconstruction():
    yy, xx = np.mgrid[:20, :20]
    xyz = np.stack((.1 + xx * .001, -.03 + yy * .001,
                    np.full((20, 20), .5)), axis=2)
    box = segment_3d_box(xyz[:, :, 2], (0, 0, 20, 20), 1.,
                         fx=1., fy=1., ppx=0., ppy=0., points_xyz=xyz)
    assert box is not None
    np.testing.assert_allclose(box['center'], [.1095, -.0205, .5])


def test_sdk_projection_and_invalid_nodes():
    sdk = SimpleNamespace(rs2_project_point_to_pixel=Mock(return_value=[12.9, 23.1]))
    intr = SimpleNamespace(fx=10., fy=10., ppx=0., ppy=0.)
    uv = project_camera_points([[1., 2., .5], [0., 0., 0.], [np.nan, 0., 1.]], intr, sdk)
    np.testing.assert_array_equal(uv, [[12, 23], [-1, -1], [-1, -1]])
    sdk.rs2_project_point_to_pixel.assert_called_once()
