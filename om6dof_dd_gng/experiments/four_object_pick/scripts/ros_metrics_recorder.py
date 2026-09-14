#!/usr/bin/env python3
"""Record typed semantic-topology and reachability metrics without commanding motion."""

from __future__ import annotations

import argparse
import csv
import json
import time
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import String

from om6dof_dd_gng.msg import EnvironmentGraph, ReachabilityGraph, ReachabilityPlan


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def header_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


class CsvSink:
    def __init__(self, path: Path, fields: list[str]):
        self.handle = path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.handle, fieldnames=fields)
        self.writer.writeheader()
        self.handle.flush()

    def write(self, row: dict) -> None:
        self.writer.writerow(row)
        self.handle.flush()

    def close(self) -> None:
        self.handle.close()


class MetricsRecorder(Node):
    def __init__(self, args):
        super().__init__("four_object_metrics_recorder")
        args.output_dir.mkdir(parents=True, exist_ok=True)
        self.started = time.monotonic()
        self.counts = Counter({name: 0 for name in (
            "reachability_plan", "reachability_graph", "environment_graph",
            "labels", "status", "perception_metrics", "debug_image", "joint_states",
        )})
        self.last_counts = Counter()
        self.last_health_ns = time.monotonic_ns()
        self.health_windows = 0

        common = ["received_utc", "received_unix_ns", "received_monotonic_ns", "header_ros_ns"]
        self.plan = CsvSink(args.output_dir / "reachability_plan.csv", common + [
            "graph_method", "graph_revision", "query_id", "scene_id", "requested_target_id",
            "requested_target_x", "requested_target_y", "requested_target_z",
            "selected_target_id", "valid", "reason", "blocked_nodes", "blocked_edges",
            "planning_time_ms", "exact_valid", "exact_checks", "exact_replans",
            "exact_time_ms", "start_node_id", "goal_node_id", "target_distance",
            "graph_cost", "start_connection_cost", "total_joint_path_cost",
            "path_node_count", "trajectory_point_count", "ee_path_pose_count",
        ])
        self.graph = CsvSink(args.output_dir / "reachability_graph.csv", common + [
            "graph_method", "graph_revision", "requested_nodes", "nodes", "edges",
            "components", "build_time_ms", "anchor_nodes", "prototype_budget",
            "prototype_nodes", "requested_guard_nodes", "guard_nodes", "fill_nodes",
            "candidate_attempts", "gng_training_samples", "guard_fraction", "sample_stream_seed",
            "halton_start_index", "sample_stream_type", "group_name", "end_effector_link",
            "joint_names_json", "joint_lower_bounds_json", "joint_upper_bounds_json",
            "urdf_sha256", "srdf_sha256", "parameters_sha256",
        ])
        self.environment = CsvSink(args.output_dir / "environment_graph.csv", common + [
            "nodes", "edges", "labelled_nodes", "class_histogram_json", "message_interval_ms",
        ])
        self.perception = CsvSink(args.output_dir / "perception_metrics.csv", [
            "received_utc", "received_unix_ns", "received_monotonic_ns", "schema",
            "capture_sequence", "frame_received", "accepted", "drop_reason",
            "ros_host_receipt_ns", "realsense_depth_timestamp_ms",
            "realsense_depth_timestamp_domain", "realsense_color_timestamp_ms",
            "realsense_color_timestamp_domain", "depth_color_timestamp_delta_ms",
            "depth_frame_number", "color_frame_number", "device_frame_gap",
            "device_frame_number_reset", "world_frame", "camera_frame", "tf_mode",
            "capture_wait_ms", "alignment_ms", "camera_tf_lookup_ms", "body_tf_lookup_ms",
            "robot_graph_publication_ms", "deprojection_world_tf_self_mask_ms",
            "dd_gng_update_ms", "graph_snapshot_ms", "yolo_submit_ms", "yolo_snapshot_ms",
            "semantic_fusion_ms", "core_publication_ms", "debug_image_ms",
            "processing_total_ms", "host_receipt_to_pipeline_complete_ms",
            "capture_cycle_total_ms", "sampled_pixels", "depth_rejected",
            "deprojection_rejected", "self_masked", "accepted_points", "body_segments",
            "graph_nodes", "graph_edges", "detections", "labeled_nodes",
            "yolo_submit_outcome", "yolo_result_input_frame_sequence", "yolo_result_sequence",
            "yolo_result_available", "yolo_result_matches_current_frame", "yolo_started_count",
            "yolo_busy_skip_count", "yolo_rate_limited_count", "yolo_completed_count",
            "yolo_inference_ms", "yolo_result_age_ms", "yolo_busy",
            "capture_attempts_total", "received_frames_total", "published_frames_total",
            "dropped_received_frames_total", "capture_failures_total", "camera_timeouts_total",
            "camera_errors_total", "invalid_frames_total", "stale_before_tf_total",
            "camera_tf_unavailable_total", "body_tf_unavailable_total",
            "stale_before_deprojection_total", "stale_before_publish_total",
            "processing_errors_total", "device_frame_gap_total", "device_frame_number_resets_total",
            "debug_image_enabled", "debug_image_published_this_frame",
        ])
        self.health = CsvSink(args.output_dir / "topic_health.csv", [
            "utc_iso", "monotonic_ns", "topic", "messages_total", "messages_in_interval",
            "interval_sec", "rate_hz", "window_kind",
        ])
        self.labels_handle = (args.output_dir / "semantic_labels.jsonl").open("w", encoding="utf-8")
        self.status_handle = (args.output_dir / "topo_status.jsonl").open("w", encoding="utf-8")
        self.perception_handle = (args.output_dir / "perception_metrics.jsonl").open(
            "w", encoding="utf-8"
        )
        self.last_environment_ns = 0

        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=64,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(ReachabilityPlan, args.plan_topic, self.on_plan, latched)
        self.create_subscription(ReachabilityGraph, args.graph_topic, self.on_graph, latched)
        self.create_subscription(EnvironmentGraph, args.environment_topic, self.on_environment, reliable)
        self.create_subscription(String, args.labels_topic, self.on_labels, reliable)
        self.create_subscription(String, args.status_topic, self.on_status, latched)
        self.create_subscription(String, args.metrics_topic, self.on_perception, sensor)
        self.create_subscription(CompressedImage, args.debug_image_topic, self.on_debug_image, sensor)
        self.create_subscription(JointState, args.joint_states_topic, self.on_joint_states, sensor)
        self.create_timer(1.0, self.on_health)
        self.get_logger().info(f"recording metrics to {args.output_dir}")

    @staticmethod
    def reception() -> dict:
        return {
            "received_utc": utc_now(),
            "received_unix_ns": time.time_ns(),
            "received_monotonic_ns": time.monotonic_ns(),
        }

    def on_plan(self, msg: ReachabilityPlan) -> None:
        self.counts["reachability_plan"] += 1
        self.plan.write({**self.reception(), "header_ros_ns": header_ns(msg),
            "graph_method": msg.graph_method, "graph_revision": msg.graph_revision,
            "query_id": msg.query_id, "scene_id": msg.scene_id,
            "requested_target_id": msg.requested_target_environment_node_id,
            "requested_target_x": msg.requested_target_position.x,
            "requested_target_y": msg.requested_target_position.y,
            "requested_target_z": msg.requested_target_position.z,
            "selected_target_id": msg.target_environment_node_id, "valid": int(msg.valid),
            "reason": msg.reason, "blocked_nodes": msg.blocked_node_count,
            "blocked_edges": msg.blocked_edge_count, "planning_time_ms": msg.planning_time_ms,
            "exact_valid": int(msg.exact_collision_valid), "exact_checks": msg.exact_state_checks,
            "exact_replans": msg.exact_replans, "exact_time_ms": msg.exact_validation_time_ms,
            "start_node_id": msg.start_node_id, "goal_node_id": msg.goal_node_id,
            "target_distance": msg.target_distance, "graph_cost": msg.graph_cost,
            "start_connection_cost": msg.start_connection_cost,
            "total_joint_path_cost": msg.total_joint_path_cost,
            "path_node_count": len(msg.reachability_node_ids),
            "trajectory_point_count": len(msg.joint_path_preview.points),
            "ee_path_pose_count": len(msg.end_effector_path.poses)})

    def on_graph(self, msg: ReachabilityGraph) -> None:
        self.counts["reachability_graph"] += 1
        self.graph.write({**self.reception(), "header_ros_ns": header_ns(msg),
            "graph_method": msg.graph_method, "graph_revision": msg.graph_revision,
            "requested_nodes": msg.requested_node_count, "nodes": len(msg.nodes),
            "edges": len(msg.edges), "components": msg.connected_components,
            "build_time_ms": msg.build_time_ms, "anchor_nodes": msg.anchor_node_count,
            "prototype_budget": msg.prototype_budget, "prototype_nodes": msg.prototype_node_count,
            "requested_guard_nodes": msg.requested_guard_node_count, "guard_nodes": msg.guard_node_count,
            "fill_nodes": msg.fill_sample_node_count, "candidate_attempts": msg.candidate_attempts,
            "gng_training_samples": msg.gng_training_sample_count,
            "guard_fraction": msg.effective_guard_fraction, "sample_stream_seed": msg.sample_stream_seed,
            "halton_start_index": msg.halton_start_index,
            "sample_stream_type": msg.sample_stream_type,
            "group_name": msg.group_name, "end_effector_link": msg.end_effector_link,
            "joint_names_json": json.dumps(list(msg.joint_names), separators=(",", ":")),
            "joint_lower_bounds_json": json.dumps(list(msg.joint_lower_bounds), separators=(",", ":")),
            "joint_upper_bounds_json": json.dumps(list(msg.joint_upper_bounds), separators=(",", ":")),
            "urdf_sha256": msg.expanded_urdf_sha256, "srdf_sha256": msg.srdf_sha256,
            "parameters_sha256": msg.reachability_parameters_sha256})

    def on_environment(self, msg: EnvironmentGraph) -> None:
        self.counts["environment_graph"] += 1
        now_ns = time.monotonic_ns()
        interval = (now_ns - self.last_environment_ns) / 1e6 if self.last_environment_ns else 0.0
        self.last_environment_ns = now_ns
        classes = Counter(str(node.class_id) for node in msg.nodes if node.class_id >= 0)
        self.environment.write({**self.reception(), "header_ros_ns": header_ns(msg),
            "nodes": len(msg.nodes), "edges": len(msg.edges),
            "labelled_nodes": sum(classes.values()),
            "class_histogram_json": json.dumps(classes, sort_keys=True),
            "message_interval_ms": f"{interval:.6f}"})

    def write_jsonl(self, handle, topic: str, data: str) -> None:
        self.counts[topic] += 1
        handle.write(json.dumps({
            "received_utc": utc_now(), "received_unix_ns": time.time_ns(),
            "received_monotonic_ns": time.monotonic_ns(), "data": data,
        }, separators=(",", ":")) + "\n")
        handle.flush()

    def on_labels(self, msg: String) -> None:
        self.write_jsonl(self.labels_handle, "labels", msg.data)

    def on_status(self, msg: String) -> None:
        self.write_jsonl(self.status_handle, "status", msg.data)

    def on_debug_image(self, _msg: CompressedImage) -> None:
        self.counts["debug_image"] += 1

    def on_joint_states(self, _msg: JointState) -> None:
        self.counts["joint_states"] += 1

    def on_perception(self, msg: String) -> None:
        reception = self.reception()
        self.counts["perception_metrics"] += 1
        self.perception_handle.write(json.dumps({
            **reception, "data": msg.data,
        }, separators=(",", ":")) + "\n")
        self.perception_handle.flush()
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.counts["perception_metrics_parse_error"] += 1
            return
        provenance = payload.get("provenance", {})
        stages = payload.get("stages_ms", {})
        counts = payload.get("counts", {})
        yolo = payload.get("yolo", {})
        drops = payload.get("drops_cumulative", {})
        debug = payload.get("debug_image", {})
        self.perception.write({
            **reception,
            "schema": payload.get("schema", ""),
            "capture_sequence": payload.get("capture_sequence", ""),
            "frame_received": int(bool(payload.get("frame_received", False))),
            "accepted": int(bool(payload.get("accepted", False))),
            "drop_reason": payload.get("drop_reason", ""),
            "ros_host_receipt_ns": provenance.get("ros_host_receipt_ns", ""),
            "realsense_depth_timestamp_ms": provenance.get("realsense_depth_timestamp_ms", ""),
            "realsense_depth_timestamp_domain": provenance.get("realsense_depth_timestamp_domain", ""),
            "realsense_color_timestamp_ms": provenance.get("realsense_color_timestamp_ms", ""),
            "realsense_color_timestamp_domain": provenance.get("realsense_color_timestamp_domain", ""),
            "depth_color_timestamp_delta_ms": provenance.get("depth_color_timestamp_delta_ms", ""),
            "depth_frame_number": provenance.get("depth_frame_number", ""),
            "color_frame_number": provenance.get("color_frame_number", ""),
            "device_frame_gap": provenance.get("device_frame_gap", ""),
            "device_frame_number_reset": int(bool(provenance.get("device_frame_number_reset", False))),
            "world_frame": provenance.get("world_frame", ""),
            "camera_frame": provenance.get("camera_frame", ""),
            "tf_mode": provenance.get("tf_mode", ""),
            "capture_wait_ms": stages.get("capture_wait", ""),
            "alignment_ms": stages.get("alignment", ""),
            "camera_tf_lookup_ms": stages.get("camera_tf_lookup", ""),
            "body_tf_lookup_ms": stages.get("body_tf_lookup", ""),
            "robot_graph_publication_ms": stages.get("robot_graph_publication", ""),
            "deprojection_world_tf_self_mask_ms": stages.get("deprojection_world_tf_self_mask", ""),
            "dd_gng_update_ms": stages.get("dd_gng_update", ""),
            "graph_snapshot_ms": stages.get("graph_snapshot", ""),
            "yolo_submit_ms": stages.get("yolo_submit", ""),
            "yolo_snapshot_ms": stages.get("yolo_snapshot", ""),
            "semantic_fusion_ms": stages.get("semantic_fusion", ""),
            "core_publication_ms": stages.get("core_publication", ""),
            "debug_image_ms": stages.get("debug_image", ""),
            "processing_total_ms": stages.get("processing_total", ""),
            "host_receipt_to_pipeline_complete_ms": stages.get("host_receipt_to_pipeline_complete", ""),
            "capture_cycle_total_ms": stages.get("capture_cycle_total", ""),
            **{name: counts.get(name, "") for name in (
                "sampled_pixels", "depth_rejected", "deprojection_rejected", "self_masked",
                "accepted_points", "body_segments", "graph_nodes", "graph_edges", "detections",
                "labeled_nodes",
            )},
            "yolo_submit_outcome": yolo.get("submit_outcome", ""),
            "yolo_result_input_frame_sequence": yolo.get("result_input_frame_sequence", ""),
            "yolo_result_sequence": yolo.get("result_sequence", ""),
            "yolo_result_available": int(bool(yolo.get("result_available", False))),
            "yolo_result_matches_current_frame": int(bool(yolo.get("result_matches_current_frame", False))),
            "yolo_started_count": yolo.get("started_count", ""),
            "yolo_busy_skip_count": yolo.get("busy_skip_count", ""),
            "yolo_rate_limited_count": yolo.get("rate_limited_count", ""),
            "yolo_completed_count": yolo.get("completed_count", ""),
            "yolo_inference_ms": yolo.get("inference_ms", ""),
            "yolo_result_age_ms": yolo.get("result_age_ms", ""),
            "yolo_busy": int(bool(yolo.get("busy", False))),
            **{f"{name}_total": drops.get(name, "") for name in (
                "capture_attempts", "received_frames", "published_frames",
                "dropped_received_frames", "capture_failures", "camera_timeouts",
                "camera_errors", "invalid_frames", "stale_before_tf", "camera_tf_unavailable",
                "body_tf_unavailable", "stale_before_deprojection", "stale_before_publish",
                "processing_errors",
            )},
            "device_frame_gap_total": drops.get("device_frame_gap_total", ""),
            "device_frame_number_resets_total": drops.get("device_frame_number_resets", ""),
            "debug_image_enabled": int(bool(debug.get("enabled", False))),
            "debug_image_published_this_frame": int(bool(debug.get("published_this_frame", False))),
        })

    def on_health(self, window_kind: str = "periodic") -> None:
        now_ns = time.monotonic_ns()
        interval_sec = max((now_ns - self.last_health_ns) / 1e9, 1e-9)
        if window_kind == "periodic" and self.health_windows == 0:
            window_kind = "startup"
        for topic in sorted(set(self.counts) | set(self.last_counts)):
            messages = self.counts[topic] - self.last_counts[topic]
            self.health.write({"utc_iso": utc_now(), "monotonic_ns": now_ns,
                "topic": topic, "messages_total": self.counts[topic],
                "messages_in_interval": messages, "interval_sec": f"{interval_sec:.9f}",
                "rate_hz": f"{messages / interval_sec:.6f}", "window_kind": window_kind})
        self.last_counts = self.counts.copy()
        self.last_health_ns = now_ns
        self.health_windows += 1

    def close(self) -> None:
        # The last messages often arrive after the final 1 Hz timer callback.
        # Preserve their count and use the true partial-window duration rather
        # than silently dropping them from topic-health totals.
        self.on_health("shutdown_partial")
        for sink in (self.plan, self.graph, self.environment, self.perception, self.health):
            sink.close()
        self.labels_handle.close()
        self.status_handle.close()
        self.perception_handle.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--plan-topic", default="/om6dof_topo_gng_v2/reachability_plan")
    parser.add_argument("--graph-topic", default="/om6dof_topo_gng_v2/reachability_graph_data")
    parser.add_argument("--environment-topic", default="/om6dof_topo_gng_v2/environment_graph_data")
    parser.add_argument("--labels-topic", default="/om6dof_topo_gng_v2/labels")
    parser.add_argument("--status-topic", default="/om6dof_topo_gng_v2/status")
    parser.add_argument("--metrics-topic", default="/om6dof_topo_gng_v2/perception_metrics")
    parser.add_argument("--debug-image-topic", default="/om6dof_topo_gng_v2/debug_image/compressed")
    parser.add_argument("--joint-states-topic", default="/joint_states")
    args = parser.parse_args()
    rclpy.init()
    node = MetricsRecorder(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
