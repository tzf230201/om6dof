import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from barath_boxer3d_adapter import (box_corners, letterbox_image_and_intrinsics,
    normalized_boxes, parse_yolox_detections, ray_encoding, sparse_depth_patches,
    anchor_center_to_depth)


def test_yolo_message_uses_source_stamp_filters_and_keeps_top_three():
    message={'stamp_sec':7,'stamp_nanosec':12,'detections':[
        {'class':'keyboard','score':.95,'x':2,'y':3,'w':20,'h':10},
        {'class':'bottle','score':.8,'x':20,'y':30,'w':40,'h':50},
        {'class':'bottle','score':.7,'x':30,'y':40,'w':20,'h':30},
        {'class':'bottle','score':.6,'x':40,'y':50,'w':10,'h':20},
        {'class':'bottle','score':.1,'x':1,'y':1,'w':10,'h':10}]}
    stamp,boxes,labels,scores=parse_yolox_detections(json.dumps(message),['bottle'],.25,3)
    assert stamp==7_000_000_012
    assert labels==['bottle']*3 and scores==[.8,.7,.6]
    assert boxes[0]==[20.,30.,40.,50.]


def test_full_rgb_is_letterboxed_and_box_mapping_matches_intrinsics():
    image=np.zeros((480,640,3),np.uint8);K=np.array([[600,0,320],[0,600,240],[0,0,1]],np.float32)
    square,scaled,scale,left,top=letterbox_image_and_intrinsics(image,K)
    assert square.shape==(960,960,3)
    assert scale==pytest.approx(1.5) and left==0 and top==120
    assert np.allclose(scaled,[[900,0,480],[0,900,480],[0,0,1]])
    boxes=normalized_boxes([[0,0,640,480]],640,480,scale,left,top)
    assert np.allclose(boxes,[[[.5/960,960.5/960,120.5/960,840.5/960]]])


def test_sdp_and_rays_use_rgb_projected_depth_and_local_rotation():
    K=np.array([[16,0,8],[0,16,8],[0,0,1]],np.float32)
    points=np.array([[0,0,1],[0,0,3],[1,0,1]],np.float32)
    patches=sparse_depth_patches(points,K,size=32,patch=16)
    assert patches.shape==(1,1,2,2) and patches[0,0,0,0]==pytest.approx(1.)
    rays=ray_encoding(K,np.eye(3),size=32,patch=16)
    assert rays.shape==(1,4,6) and np.allclose(rays[0,:,3:],0)
    assert np.allclose(np.linalg.norm(rays[0,:,:3],axis=1),1)


def test_yaw_box_has_rigid_rotation_and_requested_size():
    corners,rotation=box_corners([1,2,3],[.2,.4,.6],np.pi/2)
    assert np.allclose(rotation.T@rotation,np.eye(3),atol=1e-6)
    assert np.ptp(corners[:,0])==pytest.approx(.4)
    assert np.ptp(corners[:,1])==pytest.approx(.2)
    assert np.ptp(corners[:,2])==pytest.approx(.6)


def test_stable_roi_depth_anchors_front_face_with_bounded_shift():
    K=np.array([[100,0,50],[0,100,50],[0,0,1]],np.float32)
    points=np.array([[x,y,1.] for x in np.linspace(-.1,.1,10) for y in np.linspace(-.1,.1,10)],np.float32)
    center=np.array([0,0,1.10],np.float32);rotation=np.eye(3,dtype=np.float32);size=np.array([.2,.2,.2])
    corrected,info=anchor_center_to_depth(points,K,[35,35,30,30],np.eye(3),center,rotation,size)
    assert info['applied'] and info['core_point_count']>=30
    assert corrected[2]==pytest.approx(1.09,abs=1e-5)
    far,info=anchor_center_to_depth(points,K,[35,35,30,30],np.eye(3),[0,0,1.5],rotation,size)
    assert not info['applied'] and far[2]==pytest.approx(1.5)
