#!/usr/bin/env python3
"""Automatic YOLOX -> Barath19 Boxer3D ONNX preview for one D435i owner."""
import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import time
import cv2

from boxer_preview import Capture, Outputs, overlay
from barath_boxer3d_adapter import BarathBoxer3DRunner, parse_yolox_detections
from boxer_adapter import local_rotation
from boxer_pick_geometry import (load_pick_calibration, rigid_transform,
    capture_world_camera, world_detection_message)


class AutomaticOutputs(Outputs):
    def __init__(self, rgb_topic, detections_topic, *, camera_serial='',
                 calibration=None):
        super().__init__()
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from sensor_msgs.msg import CompressedImage
        from std_msgs.msg import String
        self.CompressedImage=CompressedImage;self.detections=deque(maxlen=1)
        sensor_qos=QoSProfile(depth=2,reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)
        detection_qos=QoSProfile(depth=2,reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.VOLATILE)
        self.rgb_pub=self.node.create_publisher(CompressedImage,rgb_topic,sensor_qos)
        self.detection_sub=self.node.create_subscription(
            String,detections_topic,lambda message:self.detections.append(message.data),detection_qos)
        self.camera_serial=camera_serial;self.calibration=calibration
        self.pick_pub=None;self.world_box_pub=None
        if calibration is not None:
            from visualization_msgs.msg import MarkerArray
            pick_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pick_pub=self.node.create_publisher(String,'/boxer3d/detections',pick_qos)
            self.world_box_pub=self.node.create_publisher(MarkerArray,'/boxer3d/world_boxes',pick_qos)

    def freeze_pick_transform(self, snapshot):
        """Capture world pose before YOLO and slow Boxer inference run."""
        if self.calibration is None:return
        stamp=self.rclpy.time.Time(nanoseconds=int(snapshot['receipt_ns']))
        deadline=time.monotonic()+.4
        while True:
            try:
                transform=self.tf.lookup_transform('world','end_effector_link',stamp).transform
                p=transform.translation;q=transform.rotation
                snapshot['T_world_camera']=capture_world_camera(
                    rigid_transform([p.x,p.y,p.z],[q.x,q.y,q.z,q.w]),self.calibration)
                if str(snapshot.get('gravity_source',''))=='tf':
                    # Pickup uses the measured camera mount for TF-derived
                    # gravity as well as position, keeping the OBB axes coherent.
                    up=snapshot['T_world_camera'][:3,:3].T @ [0.,0.,1.]
                    snapshot['up_camera']=up
                    snapshot['R_local_camera']=local_rotation(up)
                return
            except Exception as error:
                if time.monotonic()>=deadline:
                    raise ValueError('Capture-time world to end_effector_link TF unavailable') from error
                self.rclpy.spin_once(self.node,timeout_sec=.02)

    def invalidate_pick(self, reason, source_stamp_ns=None):
        if self.pick_pub is None:return
        now=self.node.get_clock().now().nanoseconds
        self._publish_pick_message({
            'schema':'om6dof.boxer3d_detections.v1','frame_id':'world',
            'source_stamp_ns':int(source_stamp_ns or now),'result_stamp_ns':now,
            'camera_serial':str(self.camera_serial),'calibration_verified':True,
            'calibration_sha256':self.calibration['camera_calibration_sha256'],
            'boxes':[],'reason':str(reason)})

    def publish_pick(self, result, snapshot):
        if self.pick_pub is None:return
        message=world_detection_message(result,snapshot,self.calibration,self.camera_serial,
                                       self.node.get_clock().now().nanoseconds)
        self._publish_pick_message(message)

    def _publish_pick_message(self, payload):
        from std_msgs.msg import String
        from visualization_msgs.msg import Marker,MarkerArray
        message=String();message.data=json.dumps(payload,allow_nan=False);self.pick_pub.publish(message)
        markers=MarkerArray();clear=Marker();clear.action=Marker.DELETEALL;markers.markers.append(clear)
        for box in payload['boxes']:
            marker=Marker();marker.header.frame_id='world'
            marker.header.stamp=self.rclpy.time.Time(nanoseconds=payload['source_stamp_ns']).to_msg()
            marker.ns='boxer3d_world_boxes';marker.id=box['id'];marker.type=Marker.CUBE;marker.action=Marker.ADD
            marker.pose.position.x,marker.pose.position.y,marker.pose.position.z=box['center']
            (marker.pose.orientation.x,marker.pose.orientation.y,marker.pose.orientation.z,
             marker.pose.orientation.w)=box['quaternion_xyzw']
            marker.scale.x,marker.scale.y,marker.scale.z=box['size']
            marker.color.r=.2;marker.color.g=.8;marker.color.b=1.;marker.color.a=.3
            marker.lifetime.sec=30
            markers.markers.append(marker)
        self.world_box_pub.publish(markers)

    def publish_rgb(self, bgr, receipt_ns):
        ok,encoded=cv2.imencode('.jpg',bgr,[cv2.IMWRITE_JPEG_QUALITY,90])
        if not ok:return
        message=self.CompressedImage();message.header.frame_id='d435_color_optical_frame'
        message.header.stamp=self.rclpy.time.Time(nanoseconds=int(receipt_ns)).to_msg()
        message.format='jpeg';message.data=encoded.tobytes();self.rgb_pub.publish(message)

    def take_detection(self):
        if not self.detections:return None
        result=self.detections[-1];self.detections.clear();return result


def draw_yolo(bgr, rois, labels, scores, status):
    image=bgr.copy()
    for box,label,score in zip(rois,labels,scores):
        x,y,w,h=[int(round(value)) for value in box]
        cv2.rectangle(image,(x,y),(x+w,y+h),(220,80,230),2)
        cv2.putText(image,f'{label} {score:.0%}',(x,max(18,y-5)),cv2.FONT_HERSHEY_SIMPLEX,.55,(220,80,230),2)
    cv2.putText(image,status[:115],(8,image.shape[0]-12),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,255),1)
    return image


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--model',required=True);parser.add_argument('--serial',default='243222076197')
    parser.add_argument('--target-classes',default='');parser.add_argument('--yolo-min-score',type=float,default=.25)
    parser.add_argument('--boxer-min-score',type=float,default=.3);parser.add_argument('--max-boxes',type=int,default=3)
    parser.add_argument('--threads',type=int,default=8)
    parser.add_argument('--gravity-source',default='auto',choices=['auto','imu','tf'])
    parser.add_argument('--rgb-topic',default='/boxer3d/rgb/image/compressed')
    parser.add_argument('--detections-topic',default='/boxer3d/yolox/detections')
    parser.add_argument('--output-dir',required=True)
    parser.add_argument('--camera-calibration-file',default='')
    parser.add_argument('--publish-pick-detections',choices=['true','false'],default='false')
    parser.add_argument('--display-window',choices=['true','false'],default='true')
    args=parser.parse_args()
    allowed=[item.strip() for item in args.target_classes.split(',') if item.strip()]
    destination=Path(args.output_dir).expanduser();destination.mkdir(parents=True,exist_ok=True)
    calibration=(load_pick_calibration(args.camera_calibration_file,args.serial)
                 if args.publish_pick_detections=='true' else None)
    display=args.display_window=='true'
    print(f'Loading Barath19 Boxer3D ONNX with {args.threads} CPU threads',flush=True)
    runner=BarathBoxer3DRunner(args.model,args.threads)
    print('Barath19 Boxer3D ready; YOLO boxes are automatic; top',args.max_boxes,flush=True)
    outputs=AutomaticOutputs(args.rgb_topic,args.detections_topic,
        camera_serial=args.serial,calibration=calibration);capture=None
    latest_rois=[];latest_labels=[];latest_scores=[];future=None
    pending_snapshot=None;pending_stamp=0;last_seen_stamp=0;pending_since=0.
    status='Waiting for YOLOX detections...';title='YOLOX + Boxer3D ONNX - Q close'
    if display:
        # A launch that requests display-window but runs on a host with no
        # local X session (headless AGX over SSH, or no DISPLAY exported)
        # used to crash the whole process here, before the try/finally below
        # even starts -- taking publish-pick-detections down with it. Degrade
        # to headless instead, same as yolox_viewer_node's display_window.
        if not os.environ.get('DISPLAY'):
            print('display-window requested but no DISPLAY is set; running headless',flush=True)
            display=False
        else:
            try:
                cv2.namedWindow(title,cv2.WINDOW_NORMAL);cv2.resizeWindow(title,960,720)
            except cv2.error as error:
                print(f'Cannot create display window ({error}); running headless',flush=True)
                display=False

    def process(snapshot,rois,labels,scores):
        result,points,masks=runner.infer(snapshot,rois,labels,scores,args.boxer_min_score)
        output=destination/('capture_'+str(time.time_ns()));output.mkdir()
        result['rois_xywh']=rois;result['target_classes']=allowed
        (output/'boxes.json').write_text(json.dumps(result,indent=2))
        import numpy as np
        np.savez_compressed(output/'input.npz',**snapshot)
        for index,mask in enumerate(masks):np.save(output/f'cut_{index}.npy',points[mask])
        rendered=overlay(snapshot,result);cv2.imwrite(str(output/'boxer3d_overlay.png'),rendered)
        print(json.dumps({'saved':str(output),'seconds':result['inference_seconds'],
                          'boxes':[(box['label'],box['score'],box['inside_point_count']) for box in result['boxes']]}),flush=True)
        return result,points,masks,rendered,snapshot

    try:
        capture=Capture(args.serial,args.gravity_source,outputs)
        with ThreadPoolExecutor(max_workers=1) as pool:
            while outputs.rclpy.ok():
                outputs.rclpy.spin_once(outputs.node,timeout_sec=.005)
                if future is not None and future.done():
                    try:
                        result,points,masks,_,snapshot=future.result()
                        outputs.publish(result,points,masks);outputs.publish_pick(result,snapshot)
                        status=f"Boxer3D {result['inference_seconds']:.2f}s; waiting for fresh YOLO frame"
                    except Exception as error:
                        status='Boxer3D error: '+str(error);print(status,flush=True)
                        outputs.invalidate_pick(status)
                    future=None
                if pending_snapshot is not None and time.monotonic()-pending_since>3.:
                    outputs.invalidate_pick('YOLO source frame response timed out',pending_stamp)
                    pending_snapshot=None;pending_stamp=0
                current_stamp=capture.latest_receipt_ns()
                if not current_stamp or current_stamp==last_seen_stamp:
                    if display and cv2.waitKey(1)&255 in (ord('q'),27):break
                    time.sleep(.005)
                    continue
                live=capture.frame(receipt_ns=current_stamp,with_receipt=True)
                if live is None:continue
                image,receipt_ns=live;last_seen_stamp=receipt_ns
                detection=outputs.take_detection() if future is None and pending_snapshot is not None else None
                if detection is not None:
                    try:
                        stamp,rois,labels,scores=parse_yolox_detections(
                            detection,allowed,args.yolo_min_score,args.max_boxes)
                        if stamp!=pending_stamp:
                            print('Ignoring nonmatching YOLO timestamp',stamp,'expected',pending_stamp,flush=True)
                            continue
                        latest_rois,latest_labels,latest_scores=rois,labels,scores
                        if rois:
                            snapshot=pending_snapshot;pending_snapshot=None;pending_stamp=0
                            outputs.observed(snapshot,clear_boxes=False)
                            future=pool.submit(process,snapshot,rois,labels,scores)
                            status=f'Boxer3D running for {len(rois)} YOLO boxes...'
                        else:
                            outputs.invalidate_pick('YOLOX: no matching target objects',pending_stamp)
                            pending_snapshot=None;pending_stamp=0
                            status='YOLOX: no matching target objects'
                    except Exception as error:
                        status=str(error);print('Automatic prompt:',error,flush=True)
                        outputs.invalidate_pick(status,pending_stamp)
                        pending_snapshot=None;pending_stamp=0
                if future is None and pending_snapshot is None:
                    try:
                        snapshot=capture.frame(freeze=True,receipt_ns=receipt_ns)
                        if snapshot is not None:
                            outputs.freeze_pick_transform(snapshot)
                            outputs.detections.clear()
                            outputs.publish_rgb(snapshot['image_bgr'],receipt_ns)
                            pending_snapshot=snapshot;pending_stamp=receipt_ns
                            pending_since=time.monotonic()
                            status='Waiting for YOLOX on paired RGB-depth snapshot...'
                    except ValueError as error:
                        status=str(error)
                        outputs.invalidate_pick(status,receipt_ns)
                if display:
                    cv2.imshow(title,draw_yolo(image,latest_rois,latest_labels,latest_scores,status))
                    key=cv2.waitKey(1)&255
                    if key in (ord('q'),27) or cv2.getWindowProperty(title,cv2.WND_PROP_VISIBLE)==0:break
    finally:
        if capture is not None:capture.close()
        outputs.close();cv2.destroyAllWindows()


if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:pass
