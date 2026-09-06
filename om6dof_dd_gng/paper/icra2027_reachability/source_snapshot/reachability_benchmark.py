#!/usr/bin/env python3
"""Repeatable preview-only GNG vs Halton/PRM benchmark.

The controller launches one isolated reachability process per method/seed. A
probe child publishes synthetic JointState and typed EnvironmentGraph messages,
then records both a clear path and a replanning case with an obstacle on the
original route. Nothing in this script publishes to a controller topic.
"""

import argparse
import csv
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


def summarize_plan(message):
    return {
        "valid": bool(message.valid),
        "reason": message.reason,
        "blocked_nodes": int(message.blocked_node_count),
        "blocked_edges": int(message.blocked_edge_count),
        "planning_time_ms": float(message.planning_time_ms),
        "exact_valid": bool(message.exact_collision_valid),
        "exact_checks": int(message.exact_state_checks),
        "exact_replans": int(message.exact_replans),
        "exact_time_ms": float(message.exact_validation_time_ms),
        "path_nodes": len(message.reachability_node_ids),
        "path_ids": list(message.reachability_node_ids),
    }


def run_probe(timeout):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from om6dof_dd_gng.msg import (
        EnvironmentGraph,
        EnvironmentNode,
        ReachabilityGraph,
        ReachabilityPlan,
    )

    class Probe(Node):
        def __init__(self):
            super().__init__("reachability_benchmark_probe")
            latched = QoSProfile(depth=1)
            latched.reliability = ReliabilityPolicy.RELIABLE
            latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(
                ReachabilityGraph,
                "/om6dof_topo_gng/reachability_graph_data",
                self.graph_callback,
                latched,
            )
            self.create_subscription(
                ReachabilityPlan,
                "/om6dof_topo_gng/reachability_plan",
                self.plan_callback,
                latched,
            )
            self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
            self.environment_pub = self.create_publisher(
                EnvironmentGraph, "/om6dof_topo_gng/environment_graph_data", 10
            )
            self.graph = None
            self.start = None
            self.goal = None
            self.obstacle_position = None
            self.clear = None
            self.dynamic = None
            self.phase = "waiting_graph"
            self.last_publish = 0.0

        def graph_callback(self, message):
            if self.graph is not None or not message.nodes:
                return
            self.graph = message
            by_id = {node.id: node for node in message.nodes}
            adjacency = {node.id: [] for node in message.nodes}
            for edge in message.edges:
                if edge.source_id in adjacency and edge.target_id in adjacency:
                    adjacency[edge.source_id].append(edge.target_id)
                    adjacency[edge.target_id].append(edge.source_id)
            self.start = message.nodes[0]
            component = {self.start.id}
            frontier = [self.start.id]
            while frontier:
                current = frontier.pop()
                for neighbor in adjacency[current]:
                    if neighbor not in component:
                        component.add(neighbor)
                        frontier.append(neighbor)
            start_xyz = (
                self.start.pose.position.x,
                self.start.pose.position.y,
                self.start.pose.position.z,
            )
            self.goal = max(
                (by_id[node_id] for node_id in component),
                key=lambda node: math.dist(
                    start_xyz,
                    (node.pose.position.x, node.pose.position.y, node.pose.position.z),
                ),
            )
            self.phase = "clear"

        def publish_inputs(self):
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = list(self.graph.joint_names)
            joint_state.position = list(self.start.joint_positions)
            self.joint_pub.publish(joint_state)

            environment = EnvironmentGraph()
            environment.header.stamp = joint_state.header.stamp
            environment.header.frame_id = "world"
            target = EnvironmentNode()
            target.id = 900001
            target.position = self.goal.pose.position
            target.class_id = 1
            target.confidence = 1.0
            environment.nodes.append(target)
            if self.phase == "dynamic" and self.obstacle_position is not None:
                obstacle = EnvironmentNode()
                obstacle.id = 900002
                obstacle.position = self.obstacle_position
                obstacle.class_id = -1
                obstacle.confidence = 1.0
                environment.nodes.append(obstacle)
            self.environment_pub.publish(environment)

        def plan_callback(self, message):
            if self.graph is None:
                return
            if self.phase == "clear" and message.valid and message.exact_collision_valid:
                self.clear = summarize_plan(message)
                by_id = {node.id: node for node in self.graph.nodes}
                path_ids = self.clear["path_ids"]
                obstacle_id = path_ids[len(path_ids) // 2] if path_ids else self.start.id
                self.obstacle_position = by_id[obstacle_id].pose.position
                self.phase = "dynamic"
            elif self.phase == "dynamic" and message.blocked_edge_count > 0:
                self.dynamic = summarize_plan(message)
                self.phase = "done"

    rclpy.init()
    probe = Probe()
    deadline = time.monotonic() + timeout
    try:
        while rclpy.ok() and probe.phase != "done" and time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.05)
            now = time.monotonic()
            if probe.graph is not None and probe.phase in ("clear", "dynamic"):
                if now - probe.last_publish >= 0.2:
                    probe.publish_inputs()
                    probe.last_publish = now
        if probe.phase != "done":
            print(json.dumps({"error": "probe_timeout", "phase": probe.phase}), flush=True)
            return 2
        output = {
            "method": probe.graph.graph_method,
            "nodes": len(probe.graph.nodes),
            "edges": len(probe.graph.edges),
            "components": int(probe.graph.connected_components),
            "build_time_ms": float(probe.graph.build_time_ms),
            "clear": probe.clear,
            "dynamic": probe.dynamic,
        }
        print(json.dumps(output, sort_keys=True), flush=True)
        return 0
    finally:
        probe.destroy_node()
        rclpy.shutdown()


def flatten(result, method, seed):
    row = {
        "method": method,
        "seed": seed,
        "nodes": result.get("nodes", 0),
        "edges": result.get("edges", 0),
        "components": result.get("components", 0),
        "build_time_ms": result.get("build_time_ms", math.nan),
    }
    for scenario in ("clear", "dynamic"):
        values = result.get(scenario) or {}
        for key in (
            "valid",
            "reason",
            "blocked_nodes",
            "blocked_edges",
            "planning_time_ms",
            "exact_valid",
            "exact_checks",
            "exact_replans",
            "exact_time_ms",
            "path_nodes",
        ):
            row[f"{scenario}_{key}"] = values.get(key, "")
    row["error"] = result.get("error", "")
    return row


def stop_launch(process):
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=8)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)


def run_controller(args):
    script = str(Path(__file__).resolve())
    methods = [item.strip() for item in args.methods.split(",") if item.strip()]
    invalid = set(methods) - {"gng", "halton_prm"}
    if invalid:
        raise SystemExit(f"unsupported methods: {sorted(invalid)}")
    rows = []
    run_index = 0
    for method in methods:
        for seed in range(args.seeds):
            domain = args.domain_base + run_index
            run_index += 1
            sample_offset = 17 + seed * 7919
            env = os.environ.copy()
            env["ROS_DOMAIN_ID"] = str(domain)
            launch = subprocess.Popen(
                [
                    "ros2",
                    "launch",
                    "om6dof_dd_gng",
                    "reachability_graph.launch.py",
                    "launch_rviz:=false",
                    f"graph_method:={method}",
                    f"sample_count:={args.sample_count}",
                    f"halton_start_index:={sample_offset}",
                ],
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            try:
                completed = subprocess.run(
                    [sys.executable, script, "--probe", "--timeout", str(args.timeout)],
                    env=env,
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    timeout=args.timeout + 10,
                    check=False,
                )
                lines = [line for line in completed.stdout.splitlines() if line.startswith("{")]
                result = json.loads(lines[-1]) if lines else {
                    "error": f"probe_exit_{completed.returncode}: {completed.stdout[-300:]}"
                }
            except subprocess.TimeoutExpired:
                result = {"error": "probe_process_timeout"}
            finally:
                stop_launch(launch)
            row = flatten(result, method, seed)
            rows.append(row)
            print(json.dumps(row, sort_keys=True), flush=True)

    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {len(rows)} runs to {output}")
    return 0 if all(not row["error"] for row in rows) else 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--probe", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--timeout", type=float, default=35.0)
    parser.add_argument("--methods", default="gng,halton_prm")
    parser.add_argument("--seeds", type=int, default=3)
    parser.add_argument("--sample-count", type=int, default=800)
    parser.add_argument("--domain-base", type=int, default=210)
    parser.add_argument("--output", default="reachability_benchmark.csv")
    args = parser.parse_args()
    if args.probe:
        return run_probe(args.timeout)
    if args.seeds < 1 or args.sample_count < 2:
        parser.error("--seeds must be >=1 and --sample-count must be >=2")
    return run_controller(args)


if __name__ == "__main__":
    sys.exit(main())
