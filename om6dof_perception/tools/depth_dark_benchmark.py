#!/usr/bin/env python3
"""Measure RealSense depth availability and temporal stability in low light.

This benchmark does not claim absolute accuracy: that requires a calibrated
target at a known distance.  It records repeatable evidence about coverage,
dropout, and short-term depth noise while also measuring RGB luminance.
"""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", type=int, default=60)
    parser.add_argument("--warmup", type=int, default=20)
    parser.add_argument("--fps", type=int, default=5)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--min-depth-m", type=float, default=0.10)
    parser.add_argument("--max-depth-m", type=float, default=3.00)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def percentile(values, q):
    return float(np.percentile(values, q)) if values.size else None


def describe(values, scale=1.0):
    values = np.asarray(values, dtype=np.float64) * scale
    return {
        "mean": float(np.mean(values)) if values.size else None,
        "p05": percentile(values, 5),
        "p50": percentile(values, 50),
        "p95": percentile(values, 95),
        "min": float(np.min(values)) if values.size else None,
        "max": float(np.max(values)) if values.size else None,
    }


def device_info(device):
    result = {}
    for key, label in (
        (rs.camera_info.name, "name"),
        (rs.camera_info.serial_number, "serial"),
        (rs.camera_info.firmware_version, "firmware"),
        (rs.camera_info.usb_type_descriptor, "usb"),
    ):
        if device.supports(key):
            result[label] = device.get_info(key)
    return result


def coverage_grid(valid, rows=4, columns=4):
    """Return mean valid-depth fraction for equal spatial cells."""
    _, height, width = valid.shape
    grid = []
    for row in range(rows):
        y0, y1 = row * height // rows, (row + 1) * height // rows
        values = []
        for column in range(columns):
            x0, x1 = column * width // columns, (column + 1) * width // columns
            values.append(float(valid[:, y0:y1, x0:x1].mean()))
        grid.append(values)
    return grid


def main():
    args = parse_args()
    if args.frames < 3 or args.warmup < 0:
        raise ValueError("--frames must be >= 3 and --warmup must be >= 0")

    pipe = rs.pipeline()
    config = rs.config()
    config.enable_stream(
        rs.stream.depth, args.width, args.height, rs.format.z16, args.fps
    )
    config.enable_stream(
        rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps
    )
    profile = pipe.start(config)
    device = profile.get_device()
    depth_sensor = device.first_depth_sensor()
    depth_scale = float(depth_sensor.get_depth_scale())
    align = rs.align(rs.stream.color)

    depths = []
    rgb_luminance = []
    timestamps_ms = []
    started = time.monotonic()
    try:
        for index in range(args.warmup + args.frames):
            frames = align.process(pipe.wait_for_frames(timeout_ms=5000))
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            if index < args.warmup:
                continue
            depth = np.asanyarray(depth_frame.get_data()).astype(np.float32)
            bgr = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            depths.append(depth * depth_scale)
            rgb_luminance.append([
                float(np.mean(gray)),
                float(np.median(gray)),
                float(np.percentile(gray, 90)),
            ])
            timestamps_ms.append(float(depth_frame.get_timestamp()))
    finally:
        pipe.stop()

    stack = np.stack(depths)
    nonzero = (stack > 0.0) & np.isfinite(stack)
    valid = (
        (stack >= args.min_depth_m)
        & (stack <= args.max_depth_m)
        & np.isfinite(stack)
    )
    coverage_by_frame = valid.mean(axis=(1, 2))
    validity_by_pixel = valid.mean(axis=0)
    stable_mask = validity_by_pixel >= 0.80

    stable_samples = np.where(
        valid[:, stable_mask], stack[:, stable_mask], np.nan
    )
    stable_median = np.nanmedian(stable_samples, axis=0)
    stable_mad = np.nanmedian(
        np.abs(stable_samples - stable_median[np.newaxis, :]), axis=0
    )

    deltas = []
    for previous, current, previous_valid, current_valid in zip(
        stack[:-1], stack[1:], valid[:-1], valid[1:]
    ):
        common = previous_valid & current_valid
        if np.any(common):
            deltas.append(np.abs(current[common] - previous[common]))
    delta_values = np.concatenate(deltas) if deltas else np.array([])

    lum = np.asarray(rgb_luminance)
    intervals = np.diff(np.asarray(timestamps_ms))
    report = {
        "schema_version": 1,
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "device": device_info(device),
        "capture": {
            "requested_frames": args.frames,
            "captured_frames": int(stack.shape[0]),
            "warmup_frames": args.warmup,
            "resolution": [args.width, args.height],
            "requested_fps": args.fps,
            "observed_fps": (
                1000.0 / float(np.median(intervals)) if intervals.size else None
            ),
            "elapsed_s": time.monotonic() - started,
            "depth_range_m": [args.min_depth_m, args.max_depth_m],
            "depth_scale_m": depth_scale,
        },
        "rgb_luminance_0_255": {
            "frame_mean": describe(lum[:, 0]),
            "frame_median": describe(lum[:, 1]),
            "frame_p90": describe(lum[:, 2]),
        },
        "depth": {
            "nonzero_pixel_fraction_by_frame": describe(
                nonzero.mean(axis=(1, 2))
            ),
            "in_range_pixel_fraction_by_frame": describe(coverage_by_frame),
            "in_range_coverage_grid_4x4": coverage_grid(valid),
            "pixels_valid_at_least_80pct_fraction": float(stable_mask.mean()),
            "temporal_mad_mm_for_stable_pixels": describe(
                stable_mad, scale=1000.0
            ),
            "consecutive_frame_abs_delta_mm": describe(
                delta_values, scale=1000.0
            ),
            "median_depth_m_for_stable_pixels": describe(
                stable_median[np.isfinite(stable_median)]
            ),
        },
        "interpretation_limits": [
            "Temporal stability is not the same as absolute accuracy.",
            "Absolute accuracy needs a planar target at a measured distance.",
            "Scene motion during capture inflates consecutive-frame deltas.",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
