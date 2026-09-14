import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from boxer_adapter import (local_rotation,inside_obb,scaled_boxes,
                           select_gravity_source,up_from_camera_quaternion,rgb_distortion_supported)


def test_gravity_frame_is_right_handed_and_roundtrips():
    for up in ([0,-9.81,0],[9.81,0,0],[1,-9,2]):
        R=local_rotation(up)
        assert np.allclose(R@R.T,np.eye(3),atol=1e-6)
        assert np.linalg.det(R)==pytest.approx(1.)
        assert np.allclose(R@np.asarray(up)/np.linalg.norm(up),[0,0,1],atol=1e-6)
        points=np.array([[.1,.2,.3],[-.2,.1,1]])
        assert np.allclose((points@R.T)@R,points,atol=1e-6)


def test_oriented_cut_uses_inverse_rotation_and_meter_bounds():
    R=np.array([[0,-1,0],[1,0,0],[0,0,1.]])
    translation=np.array([1,2,3.]);local=np.array([[0,0,0],[.2,0,0],[0,.2,0],[0,0,.4]])
    points=local@R.T+translation
    assert inside_obb(points,R,translation,[-.3,.3,-.1,.1,-.2,.2]).tolist()==[True,True,False,False]
    with pytest.raises(ValueError):inside_obb(points,R*2,translation,[-1,1,-1,1,-1,1])


def test_box_format_and_non_square_image_scaling():
    assert np.allclose(scaled_boxes([[100,50,200,100]],640,480,960),[[150,450,100,300]])
    with pytest.raises(ValueError):scaled_boxes([[600,0,100,1]],640,480,960)
    with pytest.raises(ValueError):scaled_boxes([[0,0,0,1]],640,480,960)


def test_missing_accelerometer_falls_back_to_tf_not_unsupported_stream():
    assert select_gravity_source('auto',[])=='tf'
    assert select_gravity_source('auto',[100,200])=='imu'
    assert select_gravity_source('tf',[100])=='tf'
    with pytest.raises(ValueError,match='no accelerometer'):select_gravity_source('imu',[])


def test_tf_world_up_to_optical_axes():
    # Camera local Y points down; world +Z is camera -Y.
    q=[-np.sqrt(.5),0,0,np.sqrt(.5)]
    assert np.allclose(up_from_camera_quaternion(q),[0,-1,0],atol=1e-6)
    assert np.allclose(up_from_camera_quaternion(np.array(q)*2),[0,-1,0],atol=1e-6)
    with pytest.raises(ValueError):up_from_camera_quaternion([0,0,0,0])


def test_d435i_zero_inverse_brown_is_pinhole():
    assert rgb_distortion_supported('distortion.inverse_brown_conrady',[0,0,0,0,0])
    assert not rgb_distortion_supported('distortion.inverse_brown_conrady',[.1,0,0,0,0])
    assert not rgb_distortion_supported('distortion.kannala_brandt4',[0,0,0,0,0])
