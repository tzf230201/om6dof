#!/usr/bin/env python3
"""Barath19/Boxer3D ONNX adapter for timestamp-paired D435 RGB-D snapshots."""
import json
import time
import cv2
import numpy as np

from boxer_adapter import inside_obb


def parse_yolox_detections(payload, allowed_labels=(), min_score=0.25, max_boxes=3):
    """Return source ROS stamp, top YOLO xywh prompts and their class names."""
    message=json.loads(payload) if isinstance(payload,str) else payload
    sec=message.get('stamp_sec');nanosec=message.get('stamp_nanosec')
    if not isinstance(sec,int) or not isinstance(nanosec,int) or not 0<=nanosec<1_000_000_000:
        raise ValueError('YOLO detection message has no valid source timestamp')
    allowed={str(label).strip().lower() for label in allowed_labels if str(label).strip()}
    candidates=[]
    for item in message.get('detections',[]):
        try:
            label=str(item['class']);score=float(item['score'])
            box=[float(item[key]) for key in ('x','y','w','h')]
        except (KeyError,TypeError,ValueError):
            continue
        if (not np.isfinite([score,*box]).all() or score<min_score or min(box[2:])<=0 or
                (allowed and label.lower() not in allowed)):
            continue
        candidates.append((score,label,box))
    candidates.sort(key=lambda item:item[0],reverse=True)
    candidates=candidates[:max(1,int(max_boxes))]
    return sec*1_000_000_000+nanosec,[item[2] for item in candidates],\
        [item[1] for item in candidates],[item[0] for item in candidates]


def letterbox_image_and_intrinsics(bgr, K, size=960):
    """Preserve the complete RGB field of view while mapping it into a square."""
    height,width=bgr.shape[:2]
    scale=min(size/width,size/height)
    resized_w=max(1,int(round(width*scale)));resized_h=max(1,int(round(height*scale)))
    left=(size-resized_w)//2;top=(size-resized_h)//2
    canvas=np.zeros((size,size,3),dtype=np.uint8)
    canvas[top:top+resized_h,left:left+resized_w]=cv2.resize(bgr,(resized_w,resized_h))
    scaled=np.asarray(K,dtype=np.float32).copy()
    scaled[0,0]*=scale;scaled[1,1]*=scale
    scaled[0,2]=scaled[0,2]*scale+left;scaled[1,2]=scaled[1,2]*scale+top
    return canvas,scaled,scale,left,top


def normalized_boxes(boxes_xywh, width, height, scale, left, top, size=960):
    boxes=[]
    for x,y,w,h in boxes_xywh:
        x0=max(0.,min(float(width-1),x));y0=max(0.,min(float(height-1),y))
        x1=max(x0+1.,min(float(width),x+w));y1=max(y0+1.,min(float(height),y+h))
        boxes.append([(x0*scale+left+.5)/size,(x1*scale+left+.5)/size,
                      (y0*scale+top+.5)/size,(y1*scale+top+.5)/size])
    return np.asarray(boxes,dtype=np.float32).reshape(1,-1,4)


def sparse_depth_patches(points_camera, K_model, size=960, patch=16):
    """Median RGB-projected D435 depth per BoxerNet image patch."""
    points=np.asarray(points_camera,dtype=np.float32)
    z=points[:,2]
    valid=np.isfinite(points).all(axis=1)&(z>.1)&(z<3.)
    points=points[valid];z=points[:,2]
    uv=points@np.asarray(K_model,dtype=np.float32).T
    uv=uv[:,:2]/uv[:,2:]
    grid=size//patch
    col=np.floor(uv[:,0]/patch).astype(np.int32);row=np.floor(uv[:,1]/patch).astype(np.int32)
    inside=np.isfinite(uv).all(axis=1)&(col>=0)&(col<grid)&(row>=0)&(row<grid)
    ids=(row[inside]*grid+col[inside]);depth=z[inside]
    result=np.full(grid*grid,-1.,dtype=np.float32)
    for idx in np.unique(ids):result[idx]=np.median(depth[ids==idx])
    return result.reshape(1,1,grid,grid)


def ray_encoding(K_model, R_local_camera, size=960, patch=16):
    grid=size//patch
    u=(np.arange(grid,dtype=np.float32)+.5)*patch
    v=(np.arange(grid,dtype=np.float32)+.5)*patch
    uu,vv=np.meshgrid(u,v)
    K=np.asarray(K_model,dtype=np.float32)
    directions=np.stack([(uu-K[0,2])/K[0,0],(vv-K[1,2])/K[1,1],np.ones_like(uu)],axis=-1).reshape(-1,3)
    directions/=np.linalg.norm(directions,axis=1,keepdims=True)
    directions=directions@np.asarray(R_local_camera,dtype=np.float32).T
    moments=np.zeros_like(directions)  # local origin is the captured camera centre
    return np.concatenate([directions,moments],axis=1)[None].astype(np.float32)


def box_corners(center, size, yaw):
    half=np.asarray(size,dtype=np.float32)/2.
    local=np.asarray([[-half[0],-half[1],-half[2]],[half[0],-half[1],-half[2]],
                      [half[0],half[1],-half[2]],[-half[0],half[1],-half[2]],
                      [-half[0],-half[1],half[2]],[half[0],-half[1],half[2]],
                      [half[0],half[1],half[2]],[-half[0],half[1],half[2]]])
    c=np.cos(yaw);s=np.sin(yaw)
    rotation=np.asarray([[c,-s,0],[s,c,0],[0,0,1]],dtype=np.float32)
    return local@rotation.T+np.asarray(center),rotation


def anchor_center_to_depth(points_camera, K, roi_xywh, R_local_camera, center,
                           rotation, size, margin_m=.01, max_shift_m=.12):
    """Anchor the OBB front face to stable depth in the central 60% of its YOLO ROI."""
    points=np.asarray(points_camera,dtype=np.float32);K=np.asarray(K,dtype=np.float32)
    z=points[:,2];uv=points@K.T;uv=uv[:,:2]/uv[:,2:]
    x,y,w,h=[float(value) for value in roi_xywh]
    core=(np.isfinite(uv).all(axis=1)&np.isfinite(points).all(axis=1)&(z>.1)&
          (uv[:,0]>=x+.2*w)&(uv[:,0]<=x+.8*w)&
          (uv[:,1]>=y+.2*h)&(uv[:,1]<=y+.8*h))
    count=int(core.sum())
    metadata={'applied':False,'core_point_count':count}
    if count<30:return np.asarray(center,dtype=np.float32),metadata
    local=points[core]@np.asarray(R_local_camera,dtype=np.float32).T
    center=np.asarray(center,dtype=np.float32);distance=float(np.linalg.norm(center))
    if distance<.1:return center,metadata
    view=center/distance;projected=local@view
    p10,front,p90=np.percentile(projected,[10,50,90]);spread=float(p90-p10)
    metadata.update({'surface_range_m':float(front),'core_spread_m':spread})
    if not np.isfinite([p10,front,p90]).all() or spread>.08:return center,metadata
    half=np.asarray(size,dtype=np.float32)/2.
    support=float(np.sum(half*np.abs(np.asarray(rotation,dtype=np.float32).T@view)))
    desired=float(front+support-margin_m);shift=desired-distance
    metadata.update({'raw_center_m':center.tolist(),'shift_m':shift})
    if abs(shift)>max_shift_m:return center,metadata
    corrected=center+view*shift
    metadata.update({'applied':True,'corrected_center_m':corrected.tolist()})
    return corrected.astype(np.float32),metadata


class BarathBoxer3DRunner:
    def __init__(self, model_path, threads=8):
        import onnxruntime as ort
        options=ort.SessionOptions();options.intra_op_num_threads=int(threads);options.inter_op_num_threads=1
        options.graph_optimization_level=ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.session=ort.InferenceSession(str(model_path),sess_options=options,providers=['CPUExecutionProvider'])

    def infer(self, snapshot, rois, labels, yolo_scores, threshold=.3):
        if not rois or len(rois)!=len(labels) or len(labels)!=len(yolo_scores):
            raise ValueError('Boxer3D requires one YOLO label and score for every rectangle')
        started=time.monotonic()
        bgr=np.asarray(snapshot['image_bgr']);height,width=bgr.shape[:2]
        K=np.asarray(snapshot['K'],dtype=np.float32)
        R=np.asarray(snapshot['R_local_camera'],dtype=np.float32)
        if R.shape!=(3,3) or not np.allclose(R.T@R,np.eye(3),atol=1e-4) or not np.isclose(np.linalg.det(R),1.,atol=1e-4):
            raise ValueError('Snapshot camera rotation is not a rigid rotation')
        model_image,K_model,scale,left,top=letterbox_image_and_intrinsics(bgr,K)
        boxes=normalized_boxes(rois,width,height,scale,left,top)
        points_camera=np.asarray(snapshot['points_camera'],dtype=np.float32)
        points=points_camera@R.T
        feeds={
            'image':cv2.cvtColor(model_image,cv2.COLOR_BGR2RGB).transpose(2,0,1)[None].astype(np.float32)/255.,
            'sdp_patches':sparse_depth_patches(points_camera,K_model),
            'bb2d':boxes,
            'ray_encoding':ray_encoding(K_model,R),
        }
        centers,sizes,yaws,confidences=self.session.run(
            ['center','size','yaw','confidence'],feeds)
        output_boxes=[];masks=[]
        for index,(label,yolo_score) in enumerate(zip(labels,yolo_scores)):
            center=np.asarray(centers[0,index],dtype=np.float32)
            size=np.asarray(sizes[0,index],dtype=np.float32)
            yaw=float(yaws[0,index,0]);score=float(confidences[0,index,0])
            finite=bool(np.isfinite(center).all() and np.isfinite(size).all() and
                        np.isfinite(yaw) and np.isfinite(score) and np.all(size>0))
            accepted=finite and score>=threshold
            corners,rotation=box_corners(center,size,yaw) if finite else (np.zeros((8,3)),np.eye(3))
            raw_center=center.copy();anchor={'applied':False,'core_point_count':0}
            if finite:
                center,anchor=anchor_center_to_depth(points_camera,K,rois[index],R,center,rotation,size)
                corners,rotation=box_corners(center,size,yaw)
            bounds=np.column_stack((-size/2.,size/2.)).reshape(-1) if finite else np.zeros(6)
            mask=inside_obb(points,rotation,center,bounds) if accepted else np.zeros(len(points),dtype=bool)
            masks.append(mask)
            output_boxes.append({
                'label':label,'label_source':'yolo','yolo_score':float(yolo_score),
                'score':score,'accepted':accepted,'bounds_object_m':bounds.tolist(),
                'rotation_local_object':rotation.tolist(),'translation_local_object_m':center.tolist(),
                'raw_boxer_center_local_m':raw_center.tolist(),'depth_anchor':anchor,
                'size_m':size.tolist(),'yaw_rad':yaw,'corners_local_m':corners.tolist(),
                'inside_point_count':int(mask.sum()),
            })
        return {'boxes':output_boxes,'inference_seconds':time.monotonic()-started,
                'backend':'Barath19 Boxer3D ONNX Runtime CPU','frame_id':'boxer_local',
                'pose_source':'snapshot_local_gravity_not_robot_world',
                'overlap_point_count':int((np.sum(masks,axis=0)>1).sum())},points,masks
