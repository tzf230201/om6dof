#!/usr/bin/env python3
"""Publish synthetic perception telemetry for recorder smoke tests only."""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


def payload(sequence: int) -> dict:
    return {
        "schema": "om6dof.perception_metrics.v1",
        "capture_sequence": sequence,
        "frame_received": True,
        "accepted": True,
        "drop_reason": "",
        "provenance": {
            "ros_host_receipt_ns": time.time_ns(),
            "realsense_depth_timestamp_ms": 1000.0 + sequence,
            "realsense_depth_timestamp_domain": "hardware_clock",
            "realsense_color_timestamp_ms": 1000.0 + sequence,
            "realsense_color_timestamp_domain": "hardware_clock",
            "depth_color_timestamp_delta_ms": 0.0,
            "depth_frame_number": sequence,
            "color_frame_number": sequence,
            "device_frame_gap": 0,
            "device_frame_number_reset": False,
            "world_frame": "world",
            "camera_frame": "camera_depth_optical_frame",
            "tf_mode": "exact_frame_stamp",
        },
        "stages_ms": {
            "capture_wait": 30.0,
            "alignment": 1.0,
            "camera_tf_lookup": 0.2,
            "body_tf_lookup": 0.4,
            "robot_graph_publication": 0.1,
            "deprojection_world_tf_self_mask": 2.0,
            "dd_gng_update": 4.0,
            "graph_snapshot": 0.1,
            "yolo_submit": 0.1,
            "yolo_snapshot": 0.1,
            "semantic_fusion": 1.0,
            "core_publication": 0.3,
            "debug_image": 2.0,
            "processing_total": 10.3,
            "host_receipt_to_pipeline_complete": 10.4,
            "capture_cycle_total": 40.4,
        },
        "counts": {
            "sampled_pixels": 8640,
            "depth_rejected": 100,
            "deprojection_rejected": 0,
            "self_masked": 30,
            "accepted_points": 8510,
            "body_segments": 10,
            "graph_nodes": 500,
            "graph_edges": 900,
            "detections": 4,
            "labeled_nodes": 24,
        },
        "yolo": {
            "submit_outcome": "rate_limited",
            "result_input_frame_sequence": max(0, sequence - 1),
            "result_sequence": sequence,
            "result_available": True,
            "result_matches_current_frame": False,
            "started_count": sequence,
            "busy_skip_count": 0,
            "rate_limited_count": sequence,
            "completed_count": sequence,
            "inference_ms": 42.0,
            "result_age_ms": 12.0,
            "busy": False,
        },
        "drops_cumulative": {
            "capture_attempts": sequence,
            "received_frames": sequence,
            "published_frames": sequence,
            "dropped_received_frames": 0,
            "capture_failures": 0,
            "camera_timeouts": 0,
            "camera_errors": 0,
            "invalid_frames": 0,
            "stale_before_tf": 0,
            "camera_tf_unavailable": 0,
            "body_tf_unavailable": 0,
            "stale_before_deprojection": 0,
            "stale_before_publish": 0,
            "processing_errors": 0,
            "device_frame_gap_total": 0,
            "device_frame_number_resets": 0,
        },
        "debug_image": {"enabled": True, "published_this_frame": sequence % 2 == 0},
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/om6dof_topo_gng_v2/perception_metrics")
    parser.add_argument("--count", type=int, default=5)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("four_object_metrics_fixture_publisher")
    publisher = node.create_publisher(String, args.topic, qos_profile_sensor_data)
    time.sleep(0.5)
    for sequence in range(1, args.count + 1):
        message = String()
        message.data = json.dumps(payload(sequence), separators=(",", ":"))
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
