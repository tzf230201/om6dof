"""Regression checks for pseudo-event semantics and bounded recording."""

import csv
import sys

import numpy as np
import pytest

from event_camera_simulator import EventCameraSimulator
import event_camera_simulator as simulator_module


def frame(values):
    gray = np.array([values], dtype=np.uint8)
    return np.repeat(gray[:, :, None], 3, axis=2)


@pytest.mark.parametrize("improved", [False, True])
def test_first_black_frame_and_unchanged_input(improved):
    simulator = EventCameraSimulator(improved=improved)
    for timestamp in (1.0, 2.0):
        events, image = simulator.generate_events(frame([0, 255]), timestamp)
        assert events.size == 0
        assert not image.any()
        assert np.isfinite(simulator.reference).all()


def test_basic_polarity_coordinates_and_timestamp():
    simulator = EventCameraSimulator()
    simulator.generate_events(frame([20, 100, 80]), 1.0)
    events, image = simulator.generate_events(frame([80, 20, 80]), 1.1)
    assert events[0].tolist() == (0, 0, 1.1, 1)
    assert events["x"].tolist() == [0, 1]
    assert events["y"].tolist() == [0, 0]
    assert events["polarity"].tolist() == [1, -1]
    assert events["timestamp"].tolist() == [1.1, 1.1]
    assert image.tolist() == [[[255, 0, 0], [0, 0, 255], [0, 0, 0]]]
    assert simulator.positive_count == 1
    assert simulator.negative_count == 1


def test_reference_accumulates_slow_changes_and_retains_residual():
    basic = EventCameraSimulator(threshold=0.3)
    improved = EventCameraSimulator(threshold=0.3, improved=True)
    basic_counts, improved_counts = [], []
    # Every individual log change is <0.3, but their sum crosses thresholds.
    for index, intensity in enumerate([10, 13, 16, 20, 25, 31, 37]):
        basic_counts.append(basic.generate_events(frame([intensity]), float(index))[0].size)
        improved_counts.append(improved.generate_events(frame([intensity]), float(index))[0].size)
    assert sum(basic_counts) == 0
    assert sum(improved_counts) == 4
    residual = np.log(38.0 / 256.0) - improved.reference[0, 0]
    assert 0 <= residual < 0.3
    # A stable scene does not keep emitting events after crossings are consumed.
    assert improved.generate_events(frame([37]), 8.0)[0].size == 0


def test_reference_multiple_events_both_polarities():
    simulator = EventCameraSimulator(threshold=0.2, improved=True)
    simulator.generate_events(frame([10, 255]), 1.0)
    events, image = simulator.generate_events(frame([255, 10]), 1.2)
    # floor(log((255+1)/(10+1))/0.2) = 15 crossings per pixel.
    assert events.size == 30
    assert np.count_nonzero(events["polarity"] == 1) == 15
    assert np.count_nonzero(events["polarity"] == -1) == 15
    assert np.count_nonzero(events["x"] == 0) == 15
    assert image.tolist() == [[[255, 0, 0], [0, 0, 255]]]
    assert simulator.positive_count == 15
    assert simulator.negative_count == 15


def test_buffer_capacity_age_reset_and_csv(tmp_path):
    simulator = EventCameraSimulator(buffer_size=3, buffer_seconds=0.5)
    simulator.generate_events(frame([10, 10]), 1.0)
    simulator.generate_events(frame([100, 100]), 1.1)
    simulator.generate_events(frame([10, 10]), 1.2)
    assert simulator.buffer.count == 3
    path = tmp_path / "events.csv"
    assert simulator.save_events(path) == 3
    with path.open(newline="") as source:
        rows = list(csv.DictReader(source))
    assert list(rows[0]) == ["timestamp", "x", "y", "polarity"]
    assert [int(row["polarity"]) for row in rows] == [1, -1, -1]
    assert [float(row["timestamp"]) for row in rows] == [1.1, 1.2, 1.2]
    simulator.generate_events(frame([10, 10]), 2.0)
    assert simulator.buffer.count == 0
    simulator.generate_events(frame([100, 100]), 2.1)
    reference = simulator.reference.copy()
    simulator.buffer.reset()
    assert simulator.buffer.count == 0
    assert np.array_equal(reference, simulator.reference)
    assert simulator.save_events(path) == 0
    assert path.read_text() == "timestamp,x,y,polarity\n"


def test_large_batch_retains_only_latest_events_and_rate_counts_all():
    simulator = EventCameraSimulator(buffer_size=2)
    simulator.generate_events(frame([10, 10, 10, 10]), 10.0)
    events, _ = simulator.generate_events(frame([100, 100, 100, 100]), 10.5)
    assert events.size == 4
    assert simulator.buffer.count == 2
    assert simulator.buffer.batches[0][1]["x"].tolist() == [2, 3]
    assert simulator.event_rate == 8.0
    simulator.generate_events(frame([100, 100, 100, 100]), 11.6)
    assert simulator.event_rate == 0.0


@pytest.mark.parametrize("kwargs", [
    {"threshold": 0}, {"threshold": float("nan")}, {"epsilon": 0},
    {"epsilon": float("inf")}, {"buffer_size": 0}, {"buffer_seconds": 0},
])
def test_invalid_configuration(kwargs):
    with pytest.raises(ValueError):
        EventCameraSimulator(**kwargs)


def test_missing_video_file_reports_path_before_opening_capture(monkeypatch, tmp_path, capsys):
    missing = tmp_path / "missing.mp4"
    monkeypatch.setattr(sys, "argv", ["simulator", "--video", str(missing)])
    monkeypatch.setattr(simulator_module.cv2, "VideoCapture",
                        lambda *args: pytest.fail("Missing file must not reach OpenCV"))
    with pytest.raises(SystemExit) as error:
        simulator_module.main()
    assert error.value.code == 2
    message = capsys.readouterr().err
    assert "Video file not found" in message and "without --video" in message


@pytest.mark.parametrize("source", ["realsense", "webcam"])
def test_live_source_selection_and_release(monkeypatch, source):
    calls = []

    class Capture:
        def isOpened(self):
            return True

        def set(self, prop, value):
            return True

        def read(self):
            return True, frame([10, 20])

        def release(self):
            calls.append("released")

    def open_realsense(*args):
        calls.append("realsense")
        return Capture()

    def open_webcam(index):
        calls.append(("webcam", index))
        return Capture()

    argv = ["simulator"] if source == "realsense" else ["simulator", "--source", "webcam", "--camera", "2"]
    monkeypatch.setattr(sys, "argv", argv)
    monkeypatch.setenv("DISPLAY", ":test")
    monkeypatch.setattr(simulator_module, "RealSenseCapture", open_realsense)
    monkeypatch.setattr(simulator_module.cv2, "VideoCapture", open_webcam)
    for name in ("namedWindow", "moveWindow", "imshow", "destroyAllWindows"):
        monkeypatch.setattr(simulator_module.cv2, name, lambda *args: None)
    monkeypatch.setattr(simulator_module.cv2, "waitKey", lambda delay: ord("q"))
    assert simulator_module.main() == 0
    assert calls == ["realsense" if source == "realsense" else ("webcam", 2), "released"]
