"""Geometry and official BoxerNet adapter. All positions are metres in a local snapshot frame."""
from pathlib import Path
import sys
import time
import cv2
import numpy as np

EDGES = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]


def local_rotation(up_camera):
    """Camera -> local: +Z is measured up, +Y roughly camera forward. No robot TF."""
    up = np.asarray(up_camera, dtype=np.float64)
    if up.shape != (3,) or not np.isfinite(up).all() or np.linalg.norm(up) < 1e-6:
        raise ValueError('Invalid measured up vector')
    up /= np.linalg.norm(up)
    right = np.array([1., 0., 0.])
    right -= up * np.dot(up, right)
    if np.linalg.norm(right) < .05:
        right = np.array([0., 0., 1.]); right -= up * np.dot(up, right)
    right /= np.linalg.norm(right)
    forward = np.cross(up, right)
    return np.stack([right, forward, up]).astype(np.float32)


def inside_obb(points, rotation, translation, bounds):
    """OBB object->local rotation/translation and xmin,xmax,ymin,ymax,zmin,zmax."""
    points = np.asarray(points); rotation = np.asarray(rotation); bounds = np.asarray(bounds)
    if bounds.shape != (6,) or not np.isfinite(bounds).all() or np.any(bounds[1::2] <= bounds[::2]):
        raise ValueError('Invalid BOXER extents')
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4) or not np.isclose(np.linalg.det(rotation),1.,atol=1e-4):
        raise ValueError('Invalid BOXER rotation')
    q = (points - np.asarray(translation)) @ rotation
    return np.isfinite(q).all(axis=1) & np.all(q >= bounds[::2],axis=1) & np.all(q <= bounds[1::2],axis=1)


def scaled_boxes(boxes_xywh, width, height, model_hw):
    out=[]
    for x,y,w,h in boxes_xywh:
        if not all(np.isfinite([x,y,w,h])) or min(w,h)<=0 or x<0 or y<0 or x+w>width or y+h>height:
            raise ValueError('ROI must be nonempty and inside source image')
        out.append([x/width*model_hw,(x+w)/width*model_hw,y/height*model_hw,(y+h)/height*model_hw])
    return np.asarray(out,dtype=np.float32).reshape(-1,4)


class BoxerRunner:
    def __init__(self, repo, device='cpu'):
        repo=Path(repo).expanduser().resolve()
        sys.path.insert(0,str(repo))
        import torch
        from boxernet.boxernet import BoxerNet
        from loaders.base_loader import BaseLoader
        from utils.tw.pose import PoseTW
        if device=='cuda' and not torch.cuda.is_available():
            raise RuntimeError('CUDA PyTorch unavailable; use device:=cpu for this isolated test')
        torch.set_num_threads(4)
        self.torch=torch; self.PoseTW=PoseTW; self.loader=BaseLoader
        self.model=BoxerNet.load_from_checkpoint(str(repo/'ckpts/boxernet_hw960in2x6d768-c88128f8.ckpt'),device=device)

    def infer(self, snapshot, rois, labels, threshold=.5):
        t=time.monotonic(); torch=self.torch
        bgr=snapshot['image_bgr']; h,w=bgr.shape[:2]; size=int(self.model.hw)
        K=np.asarray(snapshot['K']); R=np.asarray(snapshot['R_local_camera'],dtype=np.float32)
        if R.shape!=(3,3) or not np.allclose(R.T@R,np.eye(3),atol=1e-4) or not np.isclose(np.linalg.det(R),1.,atol=1e-4):
            raise ValueError('Snapshot camera rotation is not a rigid rotation')
        if len(rois)!=len(labels) or not rois:
            raise ValueError('Draw one rectangle for each displayed label, in that order')
        bb=scaled_boxes(rois,w,h,size)
        rgb=cv2.cvtColor(cv2.resize(bgr,(size,size)),cv2.COLOR_BGR2RGB)
        cam=self.loader.pinhole_from_K(size,size,K[0,0]*size/w,K[1,1]*size/h,K[0,2]*size/w,K[1,2]*size/h)
        points=np.asarray(snapshot['points_camera'],dtype=np.float32)@R.T
        valid=np.isfinite(points).all(axis=1)
        points=points[valid]
        sample=points[::max(1,int(np.ceil(len(points)/10000)))]
        datum={'img0':torch.from_numpy(rgb.copy()).permute(2,0,1).float()[None]/255.,
               'cam0':cam,'T_world_rig0':self.PoseTW.from_Rt(torch.from_numpy(R),torch.zeros(3)),
               'sdp_w':torch.from_numpy(sample.copy()),'bb2d':torch.from_numpy(bb)}
        with torch.inference_mode():
            obbs=self.model.forward(datum)['obbs_pr_w'].cpu()[0]
        boxes=[]; masks=[]
        for i,label in enumerate(labels):
            obb=obbs[i]; bounds=obb.bb3_object.numpy().reshape(6)
            rotation=obb.T_world_object.R.numpy().reshape(3,3)
            translation=obb.T_world_object.t.numpy().reshape(3)
            score=float(obb.prob.reshape(-1)[0]); corners=obb.bb3corners_world.numpy().reshape(8,3)
            finite=bool(np.isfinite(corners).all() and np.isfinite(score))
            accepted=finite and score>=threshold
            mask=inside_obb(points,rotation,translation,bounds) if accepted else np.zeros(len(points),dtype=bool)
            masks.append(mask)
            boxes.append({'label':label,'label_source':'manual_prompt','score':score,
                          'accepted':accepted,'bounds_object_m':bounds.tolist(),
                          'rotation_local_object':rotation.tolist(),'translation_local_object_m':translation.tolist(),
                          'corners_local_m':corners.tolist(),'inside_point_count':int(mask.sum())})
        return {'boxes':boxes,'inference_seconds':time.monotonic()-t,
                'frame_id':'boxer_local','pose_source':'snapshot_local_gravity_not_robot_world',
                'overlap_point_count':int((np.sum(masks,axis=0)>1).sum())},points,masks
