#!/usr/bin/env python3
"""Capture one native topology debug JPEG without opening the camera device."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


DEFAULT_TOPIC = "/om6dof_topo_gng_v2/debug_image/compressed"


class OneShotCompressedImage(Node):
    """Keep only the first received sample; QoS matches the best-effort publisher."""

    def __init__(self, topic: str):
        super().__init__("four_object_debug_image_capture")
        self.message: CompressedImage | None = None
        self.received_monotonic_ns = 0
        self.subscription = self.create_subscription(
            CompressedImage, topic, self.on_image, qos_profile_sensor_data
        )

    def on_image(self, message: CompressedImage) -> None:
        if self.message is None:
            self.received_monotonic_ns = time.monotonic_ns()
            self.message = message


def write_atomic(output: Path, payload: bytes) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb", prefix=f".{output.name}.", suffix=".part",
            dir=output.parent, delete=False,
        ) as handle:
            temporary_path = Path(handle.name)
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        temporary_path.replace(output)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Save the first native DD-GNG/YOLO CompressedImage sample as a JPEG."
    )
    parser.add_argument("output", type=Path)
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    parser.add_argument(
        "--force", action="store_true", help="replace an existing output file"
    )
    args = parser.parse_args()

    if not args.topic:
        raise SystemExit("--topic must not be empty")
    if not (args.timeout_sec > 0.0):
        raise SystemExit("--timeout-sec must be positive")
    output = args.output.expanduser().resolve()
    if output.exists() and not args.force:
        raise SystemExit(f"refusing to overwrite existing file: {output}; pass --force")

    rclpy.init()
    node = OneShotCompressedImage(args.topic)
    deadline = time.monotonic() + args.timeout_sec
    try:
        while node.message is None and rclpy.ok():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise SystemExit(
                    f"timeout after {args.timeout_sec:.3f} s waiting for {args.topic}"
                )
            rclpy.spin_once(node, timeout_sec=min(0.25, remaining))
        if node.message is None:
            raise SystemExit("ROS shutdown before an image was received")

        message = node.message
        payload = bytes(message.data)
        if len(payload) < 4 or payload[:2] != b"\xff\xd8" or payload[-2:] != b"\xff\xd9":
            raise SystemExit(
                f"received {len(payload)} bytes but payload is not a complete JPEG"
            )
        if "jpeg" not in message.format.lower() and "jpg" not in message.format.lower():
            raise SystemExit(f"unexpected CompressedImage format: {message.format!r}")
        write_atomic(output, payload)

        header_ros_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        print(json.dumps({
            "schema": "om6dof.debug_image_capture.v1",
            "captured_utc": datetime.now(timezone.utc).isoformat(),
            "received_monotonic_ns": node.received_monotonic_ns,
            "topic": args.topic,
            "output": str(output),
            "bytes": len(payload),
            "sha256": hashlib.sha256(payload).hexdigest(),
            "message_format": message.format,
            "header_frame_id": message.header.frame_id,
            "header_ros_ns": header_ros_ns,
        }, sort_keys=True, separators=(",", ":")))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
