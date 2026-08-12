#!/usr/bin/env python3
"""DD-GNG nodes labelled by YOLO names and RealSense 3D segmentation.

The RealSense point cloud feeds the unchanged DD-GNG core. GNG nodes are
used as semantic seeds. For every YOLO detection, aligned RealSense depth
segments the foreground surface and derives a robust camera-frame 3D bounding
box. A DD-GNG node receives the YOLO class label only when it lies in that
3D box.

Optional ROS outputs:
  * CompressedImage: annotated DD-GNG + 3D box preview
  * String: JSON node labels and 3D object boxes in camera coordinates
"""

import argparse
import ctypes
import json
import os
import threading
import time

import cv2
import numpy as np
import pyrealsense2 as rs

from ddgng_realsense import (
    FPS,
    GNG_ITERS,
    H,
    MAX_EDGES,
    MAX_NODES,
    PIXEL_STEP,
    W,
    Z_MAX,
    Z_MIN,
    load_lib,
)
from om6dof_perception.yolox_detector import YoloXDetector
from om6dof_perception.realsense_low_light import (
    configure_depth_sensor,
    load_low_light_config,
)


DEFAULT_MODEL = os.path.expanduser(
    "~/.cache/om6dof_perception/yolox_s.onnx"
)


def foreground_depth(depth, bbox, depth_scale, depth_band_m=0.06):
    """Robust front-surface depth for one YOLO ROI, or None."""
    if depth is None:
        return None
    image_h, image_w = depth.shape[:2]
    x, y, w, h = (int(value) for value in bbox)
    # Slight inset avoids most box-edge background while retaining the object.
    mx, my = int(w * 0.08), int(h * 0.05)
    x0, x1 = max(0, x + mx), min(image_w, x + w - mx)
    y0, y1 = max(0, y + my), min(image_h, y + h - my)
    if x1 <= x0 or y1 <= y0:
        return None
    values = depth[y0:y1, x0:x1].astype(np.float64)
    values = values[values > 0] * float(depth_scale)
    if values.size < 20:
        return None
    anchor = float(np.quantile(values, 0.20))
    foreground = values[values <= anchor + float(depth_band_m)]
    if foreground.size < 10:
        return None
    return float(np.median(foreground))


def segment_3d_box(depth, bbox, depth_scale, fx, fy, ppx, ppy,
                   depth_band_m=0.12, min_points=40):
    """Segment the near object surface in a YOLO ROI and return a 3D box.

    YOLO supplies the class/name and 2D region.  Depth supplies the actual
    foreground geometry.  Quantile bounds reject isolated depth outliers while
    retaining a stable, camera-frame axis-aligned box in metres.
    """
    if depth is None:
        return None
    image_h, image_w = depth.shape[:2]
    x, y, w, h = (int(value) for value in bbox)
    inset_x, inset_y = int(w * 0.06), int(h * 0.06)
    x0, x1 = max(0, x + inset_x), min(image_w, x + w - inset_x)
    y0, y1 = max(0, y + inset_y), min(image_h, y + h - inset_y)
    if x1 <= x0 or y1 <= y0:
        return None
    raw = depth[y0:y1, x0:x1].astype(np.float64) * float(depth_scale)
    valid = np.isfinite(raw) & (raw > Z_MIN) & (raw < Z_MAX)
    if int(valid.sum()) < int(min_points):
        return None
    anchor = float(np.quantile(raw[valid], 0.15))
    foreground = valid & (raw <= anchor + float(depth_band_m))
    if int(foreground.sum()) < int(min_points):
        return None
    vv, uu = np.nonzero(foreground)
    uu = uu.astype(np.float64) + x0
    vv = vv.astype(np.float64) + y0
    zz = raw[foreground]
    points = np.column_stack(((uu - float(ppx)) / float(fx) * zz,
                              (vv - float(ppy)) / float(fy) * zz,
                              zz))
    lower = np.quantile(points, 0.02, axis=0)
    upper = np.quantile(points, 0.98, axis=0)
    if not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)):
        return None
    # A flat front surface is still an object volume.  Keep a small physical
    # thickness so the projected box is visible and GNG nodes are not rejected
    # solely because RealSense quantised every sample to one depth value.
    center = (lower + upper) * 0.5
    size = np.maximum(upper - lower, np.array([0.005, 0.005, 0.010]))
    lower, upper = center - size * 0.5, center + size * 0.5
    return {
        "min": lower,
        "max": upper,
        "center": center,
        "size": size,
        "point_count": int(points.shape[0]),
    }


def box_corners(segment):
    """Return the eight camera-frame corners of one segmented 3D box."""
    lo, hi = segment["min"], segment["max"]
    return np.array([
        [lo[0], lo[1], lo[2]], [hi[0], lo[1], lo[2]],
        [hi[0], hi[1], lo[2]], [lo[0], hi[1], lo[2]],
        [lo[0], lo[1], hi[2]], [hi[0], lo[1], hi[2]],
        [hi[0], hi[1], hi[2]], [lo[0], hi[1], hi[2]],
    ], dtype=np.float64)


def assign_node_labels_3d(node_xyz, detections, segments, padding_m=0.015):
    """Label GNG nodes only when they are inside a named segmented 3D box."""
    labels = [None] * len(node_xyz)
    ranked = sorted(range(len(detections)),
                    key=lambda index: float(detections[index].score),
                    reverse=True)
    for index in ranked:
        segment = segments[index]
        if segment is None:
            continue
        lower = segment["min"] - float(padding_m)
        upper = segment["max"] + float(padding_m)
        for node_index, xyz in enumerate(node_xyz):
            if labels[node_index] is not None:
                continue
            if np.all(xyz >= lower) and np.all(xyz <= upper):
                labels[node_index] = detections[index]
    return labels


def draw_projected_3d_box(image, segment, fx, fy, ppx, ppy, color):
    """Project a camera-frame 3D box onto the aligned RGB image."""
    corners = box_corners(segment)
    uv = np.column_stack((corners[:, 0] / corners[:, 2] * float(fx) + float(ppx),
                          corners[:, 1] / corners[:, 2] * float(fy) + float(ppy)))
    uv = np.round(uv).astype(np.int32)
    for first, second in ((0, 1), (1, 2), (2, 3), (3, 0),
                          (4, 5), (5, 6), (6, 7), (7, 4),
                          (0, 4), (1, 5), (2, 6), (3, 7)):
        cv2.line(image, tuple(uv[first]), tuple(uv[second]), color, 2,
                 cv2.LINE_AA)
    return uv


def assign_node_labels(node_uv, node_xyz, detections, detection_depths,
                       depth_tolerance_m=0.08):
    """Return one detection (or None) per node using pixel + depth overlap.

    Highest-confidence boxes win when boxes overlap. Invalid/off-image GNG
    nodes remain unlabelled. A missing ROI depth falls back to 2D intersection
    so RGB-only detections remain usable.
    """
    labels = [None] * len(node_uv)
    ranked = sorted(
        range(len(detections)),
        key=lambda index: float(detections[index].score),
        reverse=True,
    )
    for index in ranked:
        detection = detections[index]
        x, y, w, h = detection.bbox
        surface_z = detection_depths[index]
        for node_index, ((u, v), xyz) in enumerate(zip(node_uv, node_xyz)):
            if labels[node_index] is not None:
                continue
            if not (x <= int(u) < x + w and y <= int(v) < y + h):
                continue
            node_z = float(xyz[2])
            if node_z <= 0.0:
                continue
            if surface_z is not None and abs(node_z - surface_z) > depth_tolerance_m:
                continue
            labels[node_index] = detection
    return labels


def class_color(class_id):
    """Stable bright BGR colour for a YOLO class id."""
    hue = (int(class_id) * 47) % 180
    pixel = np.uint8([[[hue, 220, 255]]])
    bgr = cv2.cvtColor(pixel, cv2.COLOR_HSV2BGR)[0, 0]
    return tuple(int(value) for value in bgr)


class AsyncYolo:
    """Run OpenCV-DNN outside the camera/GNG loop."""

    def __init__(self, detector, period_s, allowed_classes=()):
        self.detector = detector
        self.period_s = max(0.05, float(period_s))
        self.allowed_classes = frozenset(allowed_classes)
        self.lock = threading.Lock()
        self.busy = False
        self.last_start = 0.0
        self.detections = []

    def submit(self, bgr):
        now = time.monotonic()
        with self.lock:
            if self.busy or now - self.last_start < self.period_s:
                return
            self.busy = True
            self.last_start = now
        threading.Thread(
            target=self._worker, args=(bgr.copy(),), daemon=True
        ).start()

    def _worker(self, bgr):
        try:
            detections = self.detector.detect(bgr)
            if self.allowed_classes:
                detections = [
                    item for item in detections
                    if item.class_name in self.allowed_classes
                ]
            with self.lock:
                self.detections = detections
        finally:
            with self.lock:
                self.busy = False

    def snapshot(self):
        with self.lock:
            return list(self.detections)


def parse_args():
    parser = argparse.ArgumentParser(description="OM6DOF DD-GNG + YOLO 3D segmentation")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--confidence", type=float, default=0.35)
    parser.add_argument("--nms-threshold", type=float, default=0.5)
    parser.add_argument("--yolo-period", type=float, default=0.5)
    parser.add_argument("--depth-tolerance", type=float, default=0.08)
    parser.add_argument("--segment-depth-band", type=float, default=0.12,
                        help="foreground depth band used for each 3D box (m)")
    parser.add_argument("--segment-min-points", type=int, default=40,
                        help="minimum valid foreground depth samples per box")
    parser.add_argument(
        "--classes", default="",
        help="optional comma-separated COCO classes; empty means all classes",
    )
    parser.add_argument(
        "--ros-topic", default="",
        help="publish annotated sensor_msgs/CompressedImage",
    )
    parser.add_argument(
        "--labels-topic", default="",
        help="publish labelled nodes as std_msgs/String JSON",
    )
    parser.add_argument(
        "--hide-node-labels", action="store_true",
        help="colour matched nodes but omit per-node label text",
    )
    return parser.parse_args()


def _ros_outputs(args):
    if not args.ros_topic and not args.labels_topic:
        return None, None, None, None
    import rclpy
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String

    rclpy.init(args=None)
    node = rclpy.create_node("om6dof_dd_gng_yolo")
    image_pub = (
        node.create_publisher(CompressedImage, args.ros_topic, 2)
        if args.ros_topic else None
    )
    labels_pub = (
        node.create_publisher(String, args.labels_topic, 2)
        if args.labels_topic else None
    )
    return node, image_pub, labels_pub, (CompressedImage, String)


def main():
    args = parse_args()
    allowed_classes = tuple(
        value.strip() for value in args.classes.split(",") if value.strip()
    )
    yolo = AsyncYolo(
        YoloXDetector(args.model, args.confidence, args.nms_threshold),
        args.yolo_period,
        allowed_classes,
    )
    lib = load_lib()
    net = lib.ddgng_create()
    ros_node, image_pub, labels_pub, ros_types = _ros_outputs(args)

    pipe = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
    config.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
    profile = pipe.start(config)
    low_light = configure_depth_sensor(profile, rs, load_low_light_config())
    align = rs.align(rs.stream.color)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    intr = profile.get_stream(
        rs.stream.color
    ).as_video_stream_profile().get_intrinsics()
    fx, fy, ppx, ppy = intr.fx, intr.fy, intr.ppx, intr.ppy

    us = np.arange(0, W, PIXEL_STEP)
    vs = np.arange(0, H, PIXEL_STEP)
    uu, vv = np.meshgrid(us, vs)
    uu = uu.ravel().astype(np.float64)
    vv = vv.ravel().astype(np.float64)
    node_buf = np.zeros((MAX_NODES, 3), dtype=np.float64)
    edge_buf = np.zeros((MAX_EDGES, 2), dtype=np.int32)

    window = "OM6DOF DD-GNG 3D Segmentation"
    if not args.headless:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    print("[dd_gng_yolo] 3D segmentation running; " + low_light + (" headless" if args.headless else ""))
    previous_time = time.time()
    fps = 0.0

    try:
        while True:
            frames = align.process(pipe.wait_for_frames())
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            color = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(depth_frame.get_data())
            yolo.submit(color)
            detections = yolo.snapshot()

            z = depth[vs][:, us].ravel().astype(np.float64) * depth_scale
            valid = (z > Z_MIN) & (z < Z_MAX)
            zf = z[valid]
            xf = (uu[valid] - ppx) / fx * zf
            yf = (vv[valid] - ppy) / fy * zf
            points = np.ascontiguousarray(np.stack([xf, yf, zf], axis=1))
            point_count = points.shape[0]
            if point_count:
                lib.ddgng_feed(
                    net,
                    points.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                    point_count,
                    float(xf.mean()), float(yf.mean()), float(zf.mean()),
                    GNG_ITERS,
                )

            node_count = lib.ddgng_get_nodes(
                net,
                node_buf.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                MAX_NODES,
            )
            edge_count = lib.ddgng_get_edges(
                net,
                edge_buf.ctypes.data_as(ctypes.POINTER(ctypes.c_int)),
                MAX_EDGES,
            )
            nodes = node_buf[:node_count]
            node_z = np.maximum(nodes[:, 2], 1e-6)
            node_uv = np.stack([
                nodes[:, 0] / node_z * fx + ppx,
                nodes[:, 1] / node_z * fy + ppy,
            ], axis=1).astype(np.int32)
            segments = [
                segment_3d_box(
                    depth, item.bbox, depth_scale, fx, fy, ppx, ppy,
                    args.segment_depth_band, args.segment_min_points,
                )
                for item in detections
            ]
            labels = assign_node_labels_3d(nodes, detections, segments)

            # Semantic edges inherit a class colour only when both endpoints
            # have the same YOLO label; other topology stays muted green.
            for first, second in edge_buf[:edge_count]:
                if first >= node_count or second >= node_count:
                    continue
                first_label, second_label = labels[first], labels[second]
                if (first_label is not None and second_label is not None and
                        first_label.class_id == second_label.class_id):
                    edge_color = class_color(first_label.class_id)
                    thickness = 2
                else:
                    edge_color = (0, 130, 0)
                    thickness = 1
                cv2.line(
                    color, tuple(node_uv[first]), tuple(node_uv[second]),
                    edge_color, thickness, cv2.LINE_AA,
                )

            label_counts = {}
            labelled_nodes = []
            for index, ((u, v), xyz, label) in enumerate(
                    zip(node_uv, nodes, labels)):
                if not (0 <= u < W and 0 <= v < H):
                    continue
                if label is None:
                    cv2.circle(color, (u, v), 2, (0, 0, 255), -1,
                               cv2.LINE_AA)
                    continue
                node_color = class_color(label.class_id)
                cv2.circle(color, (u, v), 4, node_color, -1, cv2.LINE_AA)
                label_counts[label.class_name] = (
                    label_counts.get(label.class_name, 0) + 1
                )
                if not args.hide_node_labels:
                    cv2.putText(
                        color, label.class_name, (u + 4, v - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.30, node_color, 1,
                        cv2.LINE_AA,
                    )
                labelled_nodes.append({
                    "index": index,
                    "label": label.class_name,
                    "confidence": round(float(label.score), 4),
                    "xyz": [round(float(value), 5) for value in xyz],
                    "uv": [int(u), int(v)],
                })

            for detection, segment in zip(detections, segments):
                x, y, w, h = detection.bbox
                box_color = class_color(detection.class_id)
                if segment is not None:
                    draw_projected_3d_box(
                        color, segment, fx, fy, ppx, ppy, box_color,
                    )
                else:
                    cv2.rectangle(color, (x, y), (x + w, y + h), box_color, 1)
                count = label_counts.get(detection.class_name, 0)
                text = (
                    f"{detection.class_name} {detection.score:.2f} | "
                    f"GNG nodes={count}"
                )
                if segment is not None:
                    size_cm = segment["size"] * 100.0
                    center = segment["center"]
                    text += (f" | {size_cm[0]:.0f}x{size_cm[1]:.0f}x"
                             f"{size_cm[2]:.0f}cm z={center[2]:.2f}m")
                cv2.putText(
                    color, text, (x, max(18, y - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, box_color, 2,
                    cv2.LINE_AA,
                )

            now = time.time()
            fps = 0.9 * fps + 0.1 / max(now - previous_time, 1e-3)
            previous_time = now
            cv2.putText(
                color,
                f"DD-GNG 3D boxes={sum(item is not None for item in segments)} "
                f"nodes={node_count} labelled={len(labelled_nodes)} fps={fps:.1f}",
                (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (255, 255, 255), 2, cv2.LINE_AA,
            )

            if labels_pub is not None:
                payload = {
                    "frame": "camera_color_optical_frame",
                    "node_count": node_count,
                    "labelled_count": len(labelled_nodes),
                    "nodes": labelled_nodes,
                    "boxes": [
                        {
                            "label": detection.class_name,
                            "confidence": round(float(detection.score), 4),
                            "center_m": [round(float(value), 5)
                                         for value in segment["center"]],
                            "size_m": [round(float(value), 5)
                                       for value in segment["size"]],
                            "point_count": segment["point_count"],
                        }
                        for detection, segment in zip(detections, segments)
                        if segment is not None
                    ],
                }
                labels_pub.publish(ros_types[1](data=json.dumps(payload)))
            if image_pub is not None:
                ok, jpeg = cv2.imencode(
                    ".jpg", color, [cv2.IMWRITE_JPEG_QUALITY, 80]
                )
                if ok:
                    msg = ros_types[0]()
                    msg.header.stamp = ros_node.get_clock().now().to_msg()
                    msg.header.frame_id = "camera_color_optical_frame"
                    msg.format = "jpeg"
                    msg.data = jpeg.tobytes()
                    image_pub.publish(msg)

            if not args.headless:
                cv2.imshow(window, color)
                if (cv2.waitKey(1) & 0xFF) in (27, ord("q")):
                    break
    finally:
        pipe.stop()
        lib.ddgng_destroy(net)
        if not args.headless:
            cv2.destroyAllWindows()
        if ros_node is not None:
            ros_node.destroy_node()
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
