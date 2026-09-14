import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from boxer_adapter import local_rotation,inside_obb,scaled_boxes


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
