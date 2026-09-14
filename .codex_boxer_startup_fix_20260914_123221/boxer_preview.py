#!/usr/bin/env python3
"""Standalone, snapshot-based BOXER experiment. No robot commands or global TF."""
import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import threading
import time
import cv2
import numpy as np
from boxer_adapter import BoxerRunner, EDGES, local_rotation


class Capture:
    def __init__(self, serial):
        import pyrealsense2 as rs
        self.rs=rs; self.lock=threading.Lock(); self.latest=None; self.accel=deque(maxlen=20)
        # Avoid opening a second camera pipeline alongside the established stack.
        for proc in Path('/proc').iterdir():
            if not proc.name.isdigit(): continue
            try: args=(proc/'cmdline').read_bytes().split(b'\0')
            except OSError: continue
            if args and Path(args[0].decode(errors='replace')).name in ('topo_gng_node','center_depth_urdf_check'):
                raise RuntimeError('Camera owner is active. Stop the DD-GNG/center-depth camera launch before BOXER; keep robot controllers untouched.')
        self.pipeline=rs.pipeline(); cfg=rs.config()
        if serial:cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
        cfg.enable_stream(rs.stream.depth,640,480,rs.format.z16,30)
        cfg.enable_stream(rs.stream.accel,rs.format.motion_xyz32f,63)
        def callback(frame):
            with self.lock:
                if frame.is_frameset():self.latest=frame.as_frameset()
                elif frame.is_motion_frame() and frame.get_profile().stream_type()==rs.stream.accel:
                    a=frame.as_motion_frame().get_motion_data()
                    self.accel.append((frame.get_timestamp(),np.array([a.x,a.y,a.z],dtype=np.float32)))
        try:self.profile=self.pipeline.start(cfg,callback)
        except RuntimeError as error:raise RuntimeError(f'RealSense could not start; ensure only BOXER owns the D435i: {error}') from error
        self.pc=rs.pointcloud()
        color=self.profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr=color.get_intrinsics(); self.K=np.array([[intr.fx,0,intr.ppx],[0,intr.fy,intr.ppy],[0,0,1]],dtype=np.float32)
        self.D=np.array(intr.coeffs,dtype=np.float32)
        if intr.model not in (rs.distortion.none,rs.distortion.brown_conrady,rs.distortion.modified_brown_conrady):
            self.close();raise RuntimeError(f'Unsupported RGB distortion: {intr.model}')
        depth=self.profile.get_stream(rs.stream.depth)
        extr=depth.get_extrinsics_to(color)
        self.R_dc=np.array(extr.rotation).reshape(3,3,order='F');self.t_dc=np.array(extr.translation)
        extr=self.profile.get_stream(rs.stream.accel).get_extrinsics_to(color)
        self.R_ac=np.array(extr.rotation).reshape(3,3,order='F')

    def frame(self, freeze=False):
        with self.lock:frames=self.latest;accel=list(self.accel)
        if frames is None:return None
        color=frames.get_color_frame();depth=frames.get_depth_frame()
        if not color or not depth:return None
        image=cv2.undistort(np.asanyarray(color.get_data()),self.K,self.D)
        if not freeze:return image
        if abs(color.get_timestamp()-depth.get_timestamp())>50:
            raise ValueError('RGB/depth timestamps differ by >50ms; capture again')
        recent=[a for stamp,a in accel if abs(stamp-color.get_timestamp())<200]
        if len(recent)<5:raise ValueError('Fresh gravity samples unavailable; hold camera still and capture again')
        a=np.mean(recent,axis=0)
        if not 8.0<np.linalg.norm(a)<11.0 or np.max(np.std(recent,axis=0))>.3:
            raise ValueError('IMU not stationary; hold camera still and capture again')
        # At rest the accelerometer measures specific force (up), opposite gravity.
        R=local_rotation(self.R_ac@a)
        vertices=np.asanyarray(self.pc.calculate(depth).get_vertices()).view(np.float32).reshape(-1,3).copy()
        keep=np.isfinite(vertices).all(axis=1)&(vertices[:,2]>.1)&(vertices[:,2]<3.0)
        pts=(vertices[keep]@self.R_dc.T+self.t_dc).astype(np.float32)
        pts=pts[::2]
        return {'image_bgr':image.copy(),'K':self.K.copy(),'points_camera':pts,
                'R_local_camera':R,'stamp_ms':np.array(color.get_timestamp()),'up_camera':self.R_ac@a}

    def close(self):self.pipeline.stop()


class Outputs:
    def __init__(self):
        import rclpy
        from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2
        from visualization_msgs.msg import MarkerArray
        rclpy.init();self.rclpy=rclpy;self.node=rclpy.create_node('boxer_preview')
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.box_pub=self.node.create_publisher(MarkerArray,'/boxer/boxes',qos)
        self.cloud_pub=self.node.create_publisher(PointCloud2,'/boxer/points',qos)
        self.cut_pub=self.node.create_publisher(PointCloud2,'/boxer/cut_points',qos)

    def publish(self,result,points,masks):
        from visualization_msgs.msg import Marker,MarkerArray
        from geometry_msgs.msg import Point
        from sensor_msgs.msg import PointField
        from sensor_msgs_py import point_cloud2
        from std_msgs.msg import Header
        header=Header(frame_id='boxer_local',stamp=self.node.get_clock().now().to_msg())
        markers=MarkerArray();clear=Marker();clear.action=Marker.DELETEALL;markers.markers.append(clear)
        palette=[(230,40,180),(40,230,80),(40,140,255),(255,200,40)]
        colored=[]
        for i,(box,mask) in enumerate(zip(result['boxes'],masks)):
            if not box['accepted']:continue
            rgb=palette[i%len(palette)]; corners=box['corners_local_m']
            marker=Marker();marker.header=header;marker.ns='boxer_obb';marker.id=i
            marker.type=Marker.LINE_LIST;marker.action=Marker.ADD;marker.pose.orientation.w=1.
            marker.scale.x=.004;marker.color.r=rgb[0]/255.;marker.color.g=rgb[1]/255.;marker.color.b=rgb[2]/255.;marker.color.a=1.
            for a,b in EDGES:
                for v in (corners[a],corners[b]):marker.points.append(Point(x=float(v[0]),y=float(v[1]),z=float(v[2])))
            markers.markers.append(marker)
            text=Marker();text.header=header;text.ns='boxer_label';text.id=i;text.type=Marker.TEXT_VIEW_FACING;text.action=Marker.ADD
            c=np.mean(corners,axis=0);text.pose.position=Point(x=float(c[0]),y=float(c[1]),z=float(max(v[2] for v in corners)+.03));text.pose.orientation.w=1.
            text.scale.z=.025;text.color=marker.color;text.text=f"{box['label']} (manual) {box['score']:.2f}"
            markers.markers.append(text)
            color=(rgb[0]<<16)|(rgb[1]<<8)|rgb[2]
            colored.extend((float(p[0]),float(p[1]),float(p[2]),color) for p in points[mask])
        fields=[PointField(name=n,offset=o,datatype=t,count=1) for n,o,t in
                [('x',0,PointField.FLOAT32),('y',4,PointField.FLOAT32),('z',8,PointField.FLOAT32),('rgb',12,PointField.UINT32)]]
        self.box_pub.publish(markers);self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header,points.tolist()))
        self.cut_pub.publish(point_cloud2.create_cloud(header,fields,colored))

    def close(self):self.node.destroy_node();self.rclpy.shutdown()


def overlay(snapshot,result):
    canvas=snapshot['image_bgr'].copy();K=snapshot['K'];R=snapshot['R_local_camera']
    for i,box in enumerate(result['boxes']):
        if not box['accepted']:continue
        camera=np.asarray(box['corners_local_m'])@R
        color=[(180,40,230),(80,230,40),(255,140,40)][i%3]
        for a,b in EDGES:
            if min(camera[a,2],camera[b,2])<=.01:continue
            uv=camera[[a,b]]@K.T;uv=uv[:,:2]/uv[:,2:]
            if not np.isfinite(uv).all() or np.max(np.abs(uv))>100000:continue
            cv2.line(canvas,tuple(np.rint(uv[0]).astype(int)),tuple(np.rint(uv[1]).astype(int)),color,2)
    cv2.putText(canvas,f"BOXER: {result['inference_seconds']:.2f}s | 3D boxes, not masks",(8,25),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,255,255),1)
    return canvas


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--repo',required=True);parser.add_argument('--serial',default='243222076197')
    parser.add_argument('--device',default='cpu',choices=['cpu','cuda']);parser.add_argument('--labels',default='bottle,keyboard')
    parser.add_argument('--snapshot',default='');parser.add_argument('--rois',default='')
    parser.add_argument('--headless',action='store_true');parser.add_argument('--output-dir',required=True)
    args=parser.parse_args();labels=[x.strip() for x in args.labels.split(',') if x.strip()]
    if not labels:parser.error('At least one manual box label is required')
    if args.headless and not (args.snapshot and args.rois):parser.error('Headless mode requires --snapshot and --rois')
    output=Path(args.output_dir).expanduser();output.mkdir(parents=True,exist_ok=True)
    print('Loading official BOXER + DINO. Manual prompt labels:',labels,flush=True)
    runner=BoxerRunner(args.repo,args.device)
    print('BOXER ready:',args.device,'| CPU mode is an initial compatibility test, not real-time.',flush=True)
    capture=None;outputs=None
    def process(snap,rois):
        result,points,masks=runner.infer(snap,rois,labels)
        destination=output/('capture_'+str(time.time_ns()));destination.mkdir()
        np.savez_compressed(destination/'input.npz',**snap)
        result['rois_xywh']=np.asarray(rois).tolist();result['device']=args.device
        (destination/'boxes.json').write_text(json.dumps(result,indent=2))
        for i,mask in enumerate(masks):np.save(destination/f'cut_{i}.npy',points[mask])
        image=overlay(snap,result);cv2.imwrite(str(destination/'boxer_overlay.png'),image)
        print(json.dumps({'saved':str(destination),'seconds':result['inference_seconds'],'boxes':[(b['label'],b['score'],b['inside_point_count']) for b in result['boxes']],'overlap_points':result['overlap_point_count']}),flush=True)
        return result,points,masks,image
    try:
        if args.snapshot:
            with np.load(args.snapshot,allow_pickle=False) as f:snap={k:f[k] for k in f.files}
        else:
            capture=Capture(args.serial);snap=None
        if args.headless:
            process(snap,json.loads(args.rois));return
        outputs=Outputs();title='BOXER only - SPACE freeze / N new frame / Q close'
        cv2.namedWindow(title,cv2.WINDOW_NORMAL);cv2.resizeWindow(title,960,720)
        image=None;future=None;status='SPACE: freeze then draw '+', '.join(labels)
        with ThreadPoolExecutor(max_workers=1) as pool:
            while outputs.rclpy.ok():
                outputs.rclpy.spin_once(outputs.node,timeout_sec=.01)
                if snap is None:
                    frame=capture.frame()
                    if frame is not None:image=frame
                elif image is None:image=snap['image_bgr'].copy()
                if future is not None and future.done():
                    try:
                        result,points,masks,image=future.result();outputs.publish(result,points,masks)
                        status='Result frozen. N: new capture; SPACE: redraw boxes'
                    except Exception as error:status=str(error);print('BOXER ERROR:',error,flush=True)
                    future=None
                if image is not None:
                    display=image.copy();cv2.putText(display,status[:100],(8,display.shape[0]-12),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,255),1);cv2.imshow(title,display)
                key=cv2.waitKey(1)&255
                if key in (ord('q'),27) or (image is not None and cv2.getWindowProperty(title,cv2.WND_PROP_VISIBLE)<1):break
                if future is not None:continue
                if key==ord('n') and capture is not None:snap=None;status='SPACE: freeze then draw '+', '.join(labels)
                if key==32 and image is not None:
                    try:
                        if snap is None:snap=capture.frame(freeze=True)
                        if snap is None:continue
                        roi_title='Draw boxes IN ORDER: '+', '.join(labels)+' | Enter each, Esc finish'
                        rois=cv2.selectROIs(roi_title,snap['image_bgr'],showCrosshair=True,fromCenter=False);cv2.destroyWindow(roi_title)
                        if len(rois)!=len(labels):raise ValueError(f'Expected {len(labels)} rectangles, received {len(rois)}')
                        future=pool.submit(process,snap,rois.tolist());status='BOXER inference running on frozen RGB/depth...'
                    except Exception as error:status=str(error);print('Capture:',error,flush=True)
    finally:
        if capture is not None:capture.close()
        if outputs is not None:outputs.close()
        if not args.headless:cv2.destroyAllWindows()


if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:pass
