#!/usr/bin/env python3
"""Versioned DD-GNG camera-space graph, overlaid on the RGB view.

Pipeline per frame:
    V1: aligned depth with the original pinhole preprocessing
    V2: native depth vertices registered into RGB using device extrinsics
      -> sub-sampled valid colour-camera XYZ points (metres)
      -> fed to the original DD-GNG core (libddgng.so, from gng.cpp)
      -> node positions + edges read back
      -> nodes projected back to pixels and drawn on the colour image.

This graph is camera-relative, not world-fixed; no robot FK is used.

The GNG algorithm is the unmodified simulation code (Fritzke 1995 parameters);
only the input (real depth instead of a simulated ray scan) and the output
(OpenCV overlay instead of GL) are new.

Run inside the venv that has pyrealsense2 + cv2:
    DISPLAY=:1002 LD_LIBRARY_PATH=build_om6dof ~/ggcnn_env/bin/python ddgng_realsense.py
For V2 + the installed D435i add --model-version v2; a serial is optional
when exactly one D435i is connected.
"""
import ctypes
import argparse
import json
import os
import time

import cv2
import numpy as np
import pyrealsense2 as rs

from camera_input import (
    RealSenseInput, add_camera_arguments, project_camera_points, sample_camera_points,
)

# --- tunables (pre-processing only; never touch the GNG parameters) ----------
W, H, FPS = 640, 480, 30
PIXEL_STEP = 6          # sub-sample stride -> ~ (W/step)*(H/step) candidate points
Z_MIN, Z_MAX = 0.2, 4.0  # metres; clamp depth to a sane working range
GNG_ITERS = 4           # learning passes per frame (matches the sim's 4x loop)
MAX_NODES = 500         # must match GNGN in the core
MAX_EDGES = 8000


def load_lib():
    candidates = []
    # With --symlink-install CPython can import this helper from the source
    # directory even when the entry script lives in install/. Prefer the
    # active ROS overlay's core instead of a stale source-tree build.
    try:
        from ament_index_python.packages import get_package_prefix, PackageNotFoundError
    except ImportError:
        pass  # Standalone plain-CMake deployment, without a ROS environment.
    else:
        try:
            prefix = get_package_prefix("om6dof_dd_gng")
            candidates.append(os.path.join(prefix, "lib", "om6dof_dd_gng", "libddgng.so"))
        except PackageNotFoundError:
            pass
    here = os.path.dirname(os.path.abspath(__file__))
    candidates.extend((os.path.join(here, "libddgng.so"),
                       os.path.join(here, "build_om6dof", "libddgng.so"),
                       os.path.join(here, "build", "libddgng.so"),
                       "libddgng.so"))
    for cand in candidates:
        if os.path.exists(cand):
            lib = ctypes.CDLL(cand)
            break
    else:
        raise FileNotFoundError("libddgng.so not found - build/source om6dof_dd_gng with colcon, "
                                "or build realsense_ddgng with CMake")

    d, dp, ip, i, dbl = (ctypes.c_void_p, ctypes.POINTER(ctypes.c_double),
                         ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_double)
    lib.ddgng_create.restype = d
    lib.ddgng_feed.argtypes = [d, dp, i, dbl, dbl, dbl, i]
    lib.ddgng_feed.restype = i
    lib.ddgng_get_nodes.argtypes = [d, dp, i]
    lib.ddgng_get_nodes.restype = i
    lib.ddgng_get_edges.argtypes = [d, ip, i]
    lib.ddgng_get_edges.restype = i
    lib.ddgng_destroy.argtypes = [d]
    lib.ddgng_destroy.restype = None
    return lib


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="OM6DOF DD-GNG RealSense")
    add_camera_arguments(parser)
    parser.add_argument(
        "--headless", action="store_true",
        help="disable the local OpenCV window (for systemd/web monitor)",
    )
    parser.add_argument(
        "--ros-topic", default="",
        help="publish the annotated JPEG as sensor_msgs/CompressedImage",
    )
    parser.add_argument(
        "--jpeg-quality", type=int, default=50,
        help="JPEG quality for --ros-topic (1-100). The web monitor forwards "
             "this JPEG untouched, so it sets the preview's bandwidth: 80 "
             "costs about 52 kB per 640x480 frame, 50 about 25 kB",
    )
    return parser.parse_args(argv)


def main():
    args = parse_args()
    lib = load_lib()
    net = lib.ddgng_create()

    ros_node = None
    ros_publisher = None
    CompressedImage = None
    if args.ros_topic:
        import rclpy
        from sensor_msgs.msg import CompressedImage as RosCompressedImage

        CompressedImage = RosCompressedImage
        rclpy.init(args=None)
        ros_node = rclpy.create_node("om6dof_dd_gng")
        ros_publisher = ros_node.create_publisher(
            CompressedImage, args.ros_topic, 2
        )
        ros_node.get_logger().info(
            f"DD-GNG web stream: {args.ros_topic}"
        )

    camera = RealSenseInput(rs, args, W, H)

    node_buf = np.zeros((MAX_NODES, 3), dtype=np.float64)
    edge_buf = np.zeros((MAX_EDGES, 2), dtype=np.int32)

    win = "DD-GNG RealSense (RGB overlay)"
    if not args.headless:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        print("[ddgng] running - press ESC or q in the window to quit")
    else:
        print("[ddgng] running headless")

    t_prev = time.time()
    fps = 0.0
    try:
        camera.start()
        print('[ddgng] camera-space graph (no robot FK): ' + json.dumps(camera.metadata))
        while True:
            frame = camera.read()
            color = frame.color

            # --- deproject sub-sampled valid pixels to 3D points (metres) ----
            pts = sample_camera_points(frame, PIXEL_STEP, Z_MIN, Z_MAX)
            n = pts.shape[0]

            if n > 0:
                cx, cy, cz = (float(value) for value in pts.mean(axis=0))
                lib.ddgng_feed(
                    net, pts.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                    n, cx, cy, cz, GNG_ITERS)

            # --- read GNG back and overlay on the colour image ---------------
            mnode = lib.ddgng_get_nodes(
                net, node_buf.ctypes.data_as(ctypes.POINTER(ctypes.c_double)), MAX_NODES)
            kedge = lib.ddgng_get_edges(
                net, edge_buf.ctypes.data_as(ctypes.POINTER(ctypes.c_int)), MAX_EDGES)

            uv = project_camera_points(
                node_buf[:mnode], frame.intrinsics,
                sdk=rs if camera.contract.version == 'v2' else None)

            # edges (green) first, then nodes (red) on top
            for a, b in edge_buf[:kedge]:
                if (0 <= a < mnode and 0 <= b < mnode and
                        node_buf[a, 2] > 1e-6 and node_buf[b, 2] > 1e-6):
                    cv2.line(color, tuple(uv[a]), tuple(uv[b]), (0, 255, 0), 1, cv2.LINE_AA)
            for (x, y) in uv:
                if 0 <= x < W and 0 <= y < H:
                    cv2.circle(color, (x, y), 2, (0, 0, 255), -1, cv2.LINE_AA)

            # HUD
            now = time.time()
            fps = 0.9 * fps + 0.1 * (1.0 / max(now - t_prev, 1e-3))
            t_prev = now
            cv2.putText(color, f"{camera.contract.version.upper()} {camera.contract.camera_model} camera | "
                        f"nodes={mnode} edges={kedge} pts={n} fps={fps:4.1f}",
                        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

            if ros_publisher is not None:
                ok, jpeg = cv2.imencode(
                    ".jpg", color,
                    [cv2.IMWRITE_JPEG_QUALITY, max(1, min(100, args.jpeg_quality))]
                )
                if ok:
                    msg = CompressedImage()
                    msg.header.stamp = ros_node.get_clock().now().to_msg()
                    msg.header.frame_id = camera.contract.optical_frame
                    msg.format = "jpeg"
                    msg.data = jpeg.tobytes()
                    ros_publisher.publish(msg)

            if not args.headless:
                cv2.imshow(win, color)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord('q')):
                    break
    finally:
        camera.close()
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
