#!/usr/bin/env python3
"""Automatic YOLOX -> Barath19 Boxer3D ONNX preview for one D435i owner."""
import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import time
import cv2

from boxer_preview import Capture, Outputs, overlay
from barath_boxer3d_adapter import BarathBoxer3DRunner, parse_yolox_detections


class AutomaticOutputs(Outputs):
    def __init__(self, rgb_topic, detections_topic):
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
    args=parser.parse_args()
    allowed=[item.strip() for item in args.target_classes.split(',') if item.strip()]
    destination=Path(args.output_dir).expanduser();destination.mkdir(parents=True,exist_ok=True)
    print(f'Loading Barath19 Boxer3D ONNX with {args.threads} CPU threads',flush=True)
    runner=BarathBoxer3DRunner(args.model,args.threads)
    print('Barath19 Boxer3D ready; YOLO boxes are automatic; top',args.max_boxes,flush=True)
    outputs=AutomaticOutputs(args.rgb_topic,args.detections_topic);capture=None
    latest_rois=[];latest_labels=[];latest_scores=[];future=None
    pending_snapshot=None;pending_stamp=0;last_seen_stamp=0
    status='Waiting for YOLOX detections...';title='YOLOX + Boxer3D ONNX - Q close'
    cv2.namedWindow(title,cv2.WINDOW_NORMAL);cv2.resizeWindow(title,960,720)

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
        return result,points,masks,rendered

    try:
        capture=Capture(args.serial,args.gravity_source,outputs)
        with ThreadPoolExecutor(max_workers=1) as pool:
            while outputs.rclpy.ok():
                outputs.rclpy.spin_once(outputs.node,timeout_sec=.005)
                current_stamp=capture.latest_receipt_ns()
                if not current_stamp or current_stamp==last_seen_stamp:
                    if cv2.waitKey(1)&255 in (ord('q'),27):break
                    time.sleep(.005)
                    continue
                live=capture.frame(receipt_ns=current_stamp,with_receipt=True)
                if live is None:continue
                image,receipt_ns=live;last_seen_stamp=receipt_ns
                if future is not None and future.done():
                    try:
                        result,points,masks,_=future.result();outputs.publish(result,points,masks)
                        status=f"Boxer3D {result['inference_seconds']:.2f}s; waiting for fresh YOLO frame"
                    except Exception as error:
                        status='Boxer3D error: '+str(error);print(status,flush=True)
                    future=None
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
                            pending_snapshot=None;pending_stamp=0
                            status='YOLOX: no matching target objects'
                    except Exception as error:
                        status=str(error);print('Automatic prompt:',error,flush=True)
                        pending_snapshot=None;pending_stamp=0
                if future is None and pending_snapshot is None:
                    try:
                        snapshot=capture.frame(freeze=True,receipt_ns=receipt_ns)
                        if snapshot is not None:
                            outputs.detections.clear()
                            outputs.publish_rgb(snapshot['image_bgr'],receipt_ns)
                            pending_snapshot=snapshot;pending_stamp=receipt_ns
                            status='Waiting for YOLOX on paired RGB-depth snapshot...'
                    except ValueError as error:
                        status=str(error)
                cv2.imshow(title,draw_yolo(image,latest_rois,latest_labels,latest_scores,status))
                key=cv2.waitKey(1)&255
                if key in (ord('q'),27) or cv2.getWindowProperty(title,cv2.WND_PROP_VISIBLE)==0:break
    finally:
        if capture is not None:capture.close()
        outputs.close();cv2.destroyAllWindows()


if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:pass
