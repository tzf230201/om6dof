"""Expand the actual isolated launch without starting processes or cameras."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import pytest
from launch import LaunchContext


def load_launch():
    path = Path(__file__).parents[1] / 'launch/yolox_boxer3d_pickup.launch.py'
    spec = importlib.util.spec_from_file_location('simple_boxer_pick_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_isolated_launch_uses_calibration_and_never_starts_hardware(monkeypatch, tmp_path):
    import yaml
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'roslog'))
    module = load_launch()
    calibration = tmp_path / 'camera.yaml'
    calibration.write_text(yaml.safe_dump({
        'schema': 'om6dof.hand_eye.v1', 'camera': {'serial':'test','model':'D435i'},
        'transform': {'parent_frame':'end_effector_link','child_frame':'d435_color_optical_frame',
                      'xyz':[0.,0.,0.], 'quaternion_xyzw':[0.,0.,0.,1.]},
        'validation': {'method':'opencv_calibrateHandEye_eye_in_hand','samples':9,
                       'translation_rmse_m':.01,'rotation_rmse_rad':.04}}))
    context = LaunchContext()
    context.launch_configurations.update({
        'camera_calibration_file': str(calibration), 'camera_serial':'', 'target_class':'bottle',
        'execution_enabled':'false','max_boxes':'3','onnx_threads':'8','display_window':'false'})
    constructed = []
    def node(**kwargs):
        constructed.append(kwargs)
        return SimpleNamespace(**kwargs)
    monkeypatch.setattr(module, 'Node', node)
    result = module.launch_setup(context)
    assert len(result) == 5
    assert not any(x['package'] in ['controller_manager','om6dof_bringup'] for x in constructed)
    group = next(x for x in constructed if x['executable'] == 'move_group')
    assert group['namespace'] == 'boxer_pick'
    assert group['parameters'][1]['allow_trajectory_execution'] is False
    assert group['parameters'][0]['moveit_manage_controllers'] is False
    assert 'MoveGroupExecuteTrajectoryAction' in group['parameters'][0]['disable_capabilities']
    assert ('/tf','/boxer/tf') in group['remappings']
    picker = next(x for x in constructed if x['executable'] == 'boxer_pick_node')
    assert picker['parameters'][1]['execution_enabled'] is False
    assert picker['parameters'][1]['camera_serial'] == 'test'
    assert picker['parameters'][1]['camera_calibration_sha256']
    assert 'om6dof_v2' in picker['parameters'][1]['robot_description']
