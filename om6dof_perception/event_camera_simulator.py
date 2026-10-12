#!/usr/bin/env python3
"""Frame-based pseudo-events; timestamps do not imply real event-camera timing."""

import argparse
from collections import deque
import math
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np

# Configuration defaults (command-line arguments can override these).
CAMERA_SOURCE = "realsense"
CAMERA_INDEX = 0
WIDTH, HEIGHT = 640, 480
CAPTURE_FPS = 30
CONTRAST_THRESHOLD = 0.20
EPSILON = 1.0 / 255.0  # Grayscale intensity is normalized to [0, 1].
BUFFER_SIZE = 200_000  # Maximum number of retained events, not frames.
BUFFER_SECONDS = 5.0
IMPROVED_MODE = False
THRESHOLD_STEP = 0.02
MIN_THRESHOLD = 0.01
SAVE_DIRECTORY = Path(__file__).resolve().parent / "event_recordings"
WINDOWS = ("Original camera", "Pseudo-events: + blue / - red")
# OpenCV BGR; follow the E2VID / UZH DVS renderer polarity convention.
POSITIVE_COLOR = (255, 0, 0)  # Blue: increasing intensity / ON.
NEGATIVE_COLOR = (0, 0, 255)  # Red: decreasing intensity / OFF.
EVENT_DTYPE = np.dtype([
    ("x", np.int32), ("y", np.int32),
    ("timestamp", np.float64), ("polarity", np.int8),
])


class RealSenseCapture:
    """Read the RealSense RGB sensor in OpenCV-compatible BGR channel order."""

    def __init__(self, width, height, fps, serial=None):
        try:
            import pyrealsense2 as rs
        except ImportError as error:
            raise RuntimeError(
                "RealSense requires pyrealsense2. Use the Python environment "
                "that runs realsense_rgb_viewer.py."
            ) from error
        self.pipeline = rs.pipeline()
        self.started = False
        config = rs.config()
        if serial:
            config.enable_device(serial)
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        try:
            profile = self.pipeline.start(config)
            self.started = True
            device = profile.get_device()
            print(f"RealSense: {device.get_info(rs.camera_info.name)}")
            print(f"Serial: {device.get_info(rs.camera_info.serial_number)}")
            print(f"RGB stream: {width}x{height} @ {fps} FPS")
        except RuntimeError:
            self.release()
            raise

    def isOpened(self):
        return self.started

    def read(self):
        color = self.pipeline.wait_for_frames(5000).get_color_frame()
        if not color:
            return False, None
        return True, np.asanyarray(color.get_data()).copy()

    def release(self):
        if self.started:
            try:
                self.pipeline.stop()
            finally:
                self.started = False


class EventBuffer:
    """Retain recent NumPy batches while bounding event count and age."""

    def __init__(self, capacity, seconds):
        self.capacity, self.seconds = capacity, seconds
        self.batches = deque()
        self.count = 0

    def append(self, events, timestamp):
        if events.size:
            # Copy a large tail so the discarded allocation is not retained.
            kept = events[-self.capacity:].copy() if events.size > self.capacity else events
            self.batches.append((timestamp, kept))
            self.count += kept.size
        while self.batches and self.batches[0][0] < timestamp - self.seconds:
            self.count -= self.batches.popleft()[1].size
        while self.count > self.capacity:
            stamp, oldest = self.batches.popleft()
            excess = self.count - self.capacity
            removed = min(excess, oldest.size)
            self.count -= removed
            if removed < oldest.size:
                self.batches.appendleft((stamp, oldest[removed:].copy()))

    def reset(self):
        self.batches.clear()
        self.count = 0


class EventCameraSimulator:
    def __init__(self, threshold=CONTRAST_THRESHOLD, epsilon=EPSILON,
                 improved=IMPROVED_MODE, buffer_size=BUFFER_SIZE,
                 buffer_seconds=BUFFER_SECONDS):
        if not math.isfinite(threshold) or threshold < MIN_THRESHOLD:
            raise ValueError(f"Threshold must be finite and >= {MIN_THRESHOLD}.")
        if not math.isfinite(epsilon) or epsilon <= 0:
            raise ValueError("Epsilon must be finite and positive.")
        if buffer_size <= 0 or not math.isfinite(buffer_seconds) or buffer_seconds <= 0:
            raise ValueError("Buffer size and duration must be positive.")
        self.threshold, self.epsilon, self.improved = threshold, epsilon, improved
        self.reference = None
        self.buffer = EventBuffer(buffer_size, buffer_seconds)
        self.rate_samples = deque()
        self.rate_count = 0
        self.rate_start = None
        self.event_rate = 0.0
        self.positive_count = 0
        self.negative_count = 0

    def generate_events(self, frame, timestamp=None):
        """Return records (x,y,timestamp,polarity) and a current-frame event image."""
        if timestamp is None:
            timestamp = time.perf_counter()
        # Both capture sources supply color images in OpenCV's BGR order.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        log_image = np.log(gray.astype(np.float64) / 255.0 + self.epsilon)
        image = np.zeros((*gray.shape, 3), dtype=np.uint8)
        events = np.empty(0, dtype=EVENT_DTYPE)

        if self.reference is None or self.reference.shape != log_image.shape:
            # Initialize without generating artificial startup events.
            self.reference = log_image.copy()
        else:
            delta = log_image - self.reference
            positive = delta >= self.threshold
            negative = delta <= -self.threshold
            active = positive | negative
            ys, xs = np.nonzero(active)
            polarity = np.where(positive[ys, xs], 1, -1).astype(np.int8)

            if self.improved:
                # Vectorized threshold crossings, including multiple events/pixel.
                counts = np.floor(np.abs(delta[ys, xs]) / self.threshold).astype(np.int64)
                indices = np.repeat(np.arange(xs.size), counts)
                # Retain the subthreshold residual for subsequent frames.
                self.reference[ys, xs] += polarity * counts * self.threshold
            else:
                indices = np.arange(xs.size)
                self.reference = log_image.copy()

            events = np.empty(indices.size, dtype=EVENT_DTYPE)
            events["timestamp"] = timestamp  # Same observation time for this frame.
            events["x"], events["y"] = xs[indices], ys[indices]
            events["polarity"] = polarity[indices]
            image[positive] = POSITIVE_COLOR
            image[negative] = NEGATIVE_COLOR

        self.positive_count = int(np.count_nonzero(events["polarity"] == 1))
        self.negative_count = int(events.size - self.positive_count)
        self.buffer.append(events, timestamp)
        self._update_rate(timestamp, events.size)
        return events, image

    def _update_rate(self, timestamp, count):
        # Event rate counts ALL generated events, including evicted buffer entries.
        if self.rate_start is None:
            self.rate_start = timestamp
        self.rate_samples.append((timestamp, count))
        self.rate_count += count
        while self.rate_samples and self.rate_samples[0][0] <= timestamp - 1.0:
            self.rate_count -= self.rate_samples.popleft()[1]
        elapsed = min(1.0, timestamp - self.rate_start)
        self.event_rate = self.rate_count / elapsed if elapsed > 0 else 0.0

    def save_events(self, path):
        """Export the retained buffer; loop over batches, not individual pixels."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8", newline="") as output:
            output.write("timestamp,x,y,polarity\n")
            for _, batch in self.buffer.batches:
                ordered = batch[["timestamp", "x", "y", "polarity"]]
                np.savetxt(output, ordered, delimiter=",", fmt=["%.9f", "%d", "%d", "%d"])
        return self.buffer.count


def visualize_events(frame, event_image, simulator, fps, event_count):
    """Add status below images without covering the event pixels."""
    source = cv2.copyMakeBorder(frame, 0, 95, 0, 0, cv2.BORDER_CONSTANT)
    view = cv2.copyMakeBorder(event_image, 0, 95, 0, 0, cv2.BORDER_CONSTANT)
    mode = "reference" if simulator.improved else "basic"
    lines = [
        f"FPS {fps:.1f} | events/frame {event_count:,} | {simulator.event_rate:,.0f} events/s",
        f"C {simulator.threshold:.3f} | {mode} | buffer {simulator.buffer.count:,}",
    ]
    for canvas in (source, view):
        for index, text in enumerate(lines):
            cv2.putText(canvas, text, (8, frame.shape[0] + 18 + index * 21),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.43, (0, 220, 255), 1, cv2.LINE_AA)
        # Explicit legend and separate event counts make polarity unambiguous.
        cv2.putText(canvas, f"+ BLUE: {simulator.positive_count:,}",
                    (8, frame.shape[0] + 60), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, POSITIVE_COLOR, 1, cv2.LINE_AA)
        cv2.putText(canvas, f"- RED: {simulator.negative_count:,}",
                    (canvas.shape[1] // 2, frame.shape[0] + 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, NEGATIVE_COLOR, 1, cv2.LINE_AA)
        cv2.putText(canvas, "+/- threshold | R reset buffer | S save | Q/Esc quit",
                    (8, frame.shape[0] + 83), cv2.FONT_HERSHEY_SIMPLEX,
                    0.43, (0, 220, 255), 1, cv2.LINE_AA)
    cv2.imshow(WINDOWS[0], source)
    cv2.imshow(WINDOWS[1], view)


def window_is_closed(name):
    try:
        visible = cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE)
        # GTK can return -1 for an unsupported VISIBLE property.
        if visible >= 0:
            return visible == 0
        return cv2.getWindowProperty(name, cv2.WND_PROP_AUTOSIZE) < 0
    except cv2.error:
        return True


def positive_int(value):
    result = int(value)
    if result <= 0:
        raise argparse.ArgumentTypeError("Must be positive.")
    return result


def positive_float(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("Must be finite and positive.")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", choices=("realsense", "webcam"),
                        default=CAMERA_SOURCE, help="Live camera source (default: RealSense).")
    parser.add_argument("--camera", type=int, default=CAMERA_INDEX)
    parser.add_argument("--serial", help="Optional RealSense camera serial number.")
    parser.add_argument("--video", help="Prerecorded video file; overrides --source.")
    parser.add_argument("--width", type=positive_int, default=WIDTH)
    parser.add_argument("--height", type=positive_int, default=HEIGHT)
    parser.add_argument("--fps", type=positive_int, default=CAPTURE_FPS)
    parser.add_argument("--threshold", type=positive_float, default=CONTRAST_THRESHOLD)
    parser.add_argument("--epsilon", type=positive_float, default=EPSILON)
    parser.add_argument("--buffer-size", type=positive_int, default=BUFFER_SIZE)
    parser.add_argument("--buffer-seconds", type=positive_float, default=BUFFER_SECONDS)
    parser.add_argument("--mode", choices=("basic", "reference"),
                        default="reference" if IMPROVED_MODE else "basic")
    parser.add_argument("--save-dir", type=Path, default=SAVE_DIRECTORY)
    args = parser.parse_args()
    if args.threshold < MIN_THRESHOLD:
        parser.error(f"Threshold must be >= {MIN_THRESHOLD}.")
    if args.video and not Path(args.video).is_file():
        parser.error(
            f"Video file not found: {args.video}. "
            "For live RealSense RGB, run without --video."
        )
    if sys.platform.startswith("linux") and not (
        os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    ):
        print("Run from a desktop terminal or use SSH with X11 forwarding.", file=sys.stderr)
        return 1

    simulator = EventCameraSimulator(args.threshold, args.epsilon,
                                     args.mode == "reference", args.buffer_size,
                                     args.buffer_seconds)
    capture = None
    try:
        if args.video:
            capture = cv2.VideoCapture(str(Path(args.video).resolve()))
        elif args.source == "realsense":
            capture = RealSenseCapture(args.width, args.height, args.fps, args.serial)
        else:
            capture = cv2.VideoCapture(args.camera)
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
            capture.set(cv2.CAP_PROP_FPS, args.fps)
        if not capture.isOpened():
            print("Cannot open camera/video. Check index, connection, or file.", file=sys.stderr)
            return 1
        for name in WINDOWS:
            cv2.namedWindow(name, cv2.WINDOW_AUTOSIZE)
        positioned, previous_time, fps = False, None, 0.0
        video_fps = capture.get(cv2.CAP_PROP_FPS) if args.video else 0.0
        playback_period = 1.0 / video_fps if math.isfinite(video_fps) and video_fps > 0 else 0.0
        print("Pseudo-events only: frame timing, not microsecond event-camera timing.")
        print("+ or = increases C; - decreases C; R clears buffer; S saves; Q/Esc quits.")
        while True:
            frame_start = time.perf_counter()
            ok, frame = capture.read()
            timestamp = time.perf_counter()
            if not ok:
                if args.video:
                    print("Video ended.")
                    break
                raise RuntimeError("Camera stopped delivering frames.")
            if previous_time is not None:
                measured = 1.0 / max(timestamp - previous_time, 1e-9)
                fps = measured if fps == 0 else 0.9 * fps + 0.1 * measured
            previous_time = timestamp
            events, event_image = simulator.generate_events(frame, timestamp)
            visualize_events(frame, event_image, simulator, fps, events.size)
            if not positioned:
                cv2.moveWindow(WINDOWS[0], 100, 80)
                cv2.moveWindow(WINDOWS[1], frame.shape[1] + 140, 80)
                positioned = True
            # Pace prerecorded input; event timestamps remain system observations.
            remaining = playback_period - (time.perf_counter() - frame_start)
            delay = max(1, math.ceil(remaining * 1000)) if args.video else 1
            key = cv2.waitKey(delay) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if any(window_is_closed(name) for name in WINDOWS):
                break
            if key in (ord("+"), ord("=")):
                simulator.threshold += THRESHOLD_STEP
            elif key in (ord("-"), ord("_")):
                simulator.threshold = max(MIN_THRESHOLD, simulator.threshold - THRESHOLD_STEP)
            elif key in (ord("r"), ord("R")):
                simulator.buffer.reset()  # Preserve intensity reference and live rate.
                print("Event buffer reset.")
            elif key in (ord("s"), ord("S")):
                path = args.save_dir / f"events_{time.perf_counter_ns()}.csv"
                try:
                    count = simulator.save_events(path)
                    print(f"Saved {count:,} events to {path}")
                except OSError as error:
                    print(f"CSV save failed: {error}", file=sys.stderr)
    except KeyboardInterrupt:
        pass
    except (cv2.error, RuntimeError) as error:
        print(f"Capture/display failed: {error}", file=sys.stderr)
        if not args.video and args.source == "realsense":
            print("Check the USB connection and stop other applications using "
                  "the camera. Use a supported resolution/FPS.", file=sys.stderr)
        return 1
    finally:
        try:
            if capture is not None:
                capture.release()
        finally:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
