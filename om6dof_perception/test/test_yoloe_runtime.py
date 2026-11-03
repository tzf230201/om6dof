"""Prompt changes must not relabel observations from an older inference."""
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from om6dof_perception.perception_node import Entity, PerceptionNode
from om6dof_perception.yoloe_detector import YoloESegDetector


def test_checkpoint_reencodes_changed_prompts_only():
    detector = object.__new__(YoloESegDetector)
    detector._lock = threading.RLock()
    detector.is_engine = False
    detector.classes = ("bottle",)
    encoded, applied = [], []
    detector.model = SimpleNamespace(
        model=SimpleNamespace(get_text_pe=lambda labels, **kw: encoded.append(labels) or "embedding"),
        set_classes=lambda labels, **kw: applied.append((labels, kw["embeddings"])))
    detector.set_classes(["  red mug  ", "red mug"])
    detector.set_classes(["red mug"])
    assert encoded == [["red mug"]]
    assert applied == [(["red mug"], "embedding")]
    assert detector.classes == ("red mug",)
    with pytest.raises(ValueError, match="nonempty"):
        detector.set_classes([" "])


def test_engine_rejects_changes_without_claiming_new_classes():
    detector = object.__new__(YoloESegDetector)
    detector._lock = threading.RLock()
    detector.is_engine = True
    detector.classes = ("bottle",)
    with pytest.raises(ValueError, match="frozen"):
        detector.set_classes(["screwdriver"])
    assert detector.classes == ("bottle",)


def _node():
    node = object.__new__(PerceptionNode)
    node.frame_lock = threading.RLock()
    node.detection_lock = threading.Lock()
    node.stop_event = threading.Event()
    node.detector_backend = "yoloe"
    node.open_vocabulary = True
    node.yoloe_classes = ("bottle",)
    node.target = Entity("target", "bottle", "bottle")
    node.ee = Entity("ee", "gripper")
    node.ee_mode = "disabled"
    node.camera_epoch = 1
    node.rgb = np.zeros((8, 12, 3), dtype=np.uint8)
    node.get_logger = lambda: SimpleNamespace(
        info=lambda *a, **kw: None, warn=lambda *a, **kw: None,
        error=lambda *a, **kw: None)
    return node


def test_free_text_retarget_clears_all_old_observations():
    node = _node()
    node.target.tracker = object()
    node.target.mask = np.ones((8, 12), dtype=bool)
    node.target.point = [.1, .2, .3]
    node.target.confidence = .9
    node._on_set_target(SimpleNamespace(data="Glass jar with a black lid"))
    assert node.target.class_name == "glass jar with a black lid"
    assert node.target.state == "acquiring"
    assert node.target.tracker is node.target.mask is node.target.point is None
    assert node.target.confidence is None
    assert node.prompt_update_pending is True


def test_prompt_change_during_inference_discards_old_mask():
    node = _node()
    detection = SimpleNamespace(class_name="bottle", score=.9, bbox=(0, 0, 4, 4))

    def detect(_image):
        node._on_set_target(SimpleNamespace(data="screwdriver"))
        return [detection]

    node.yolo = SimpleNamespace(classes=("bottle",), set_classes=lambda labels: None, detect=detect)
    node._acquire(node.target, node.rgb, 1, 0)
    assert node.target.class_name == "screwdriver"
    assert node.target.state == "acquiring"
    assert node.target.mask is node.target.tracker is None
    assert node.prompt_update_pending is True


def test_prompt_failure_is_reported_without_installing_old_observations():
    node = _node()
    node._on_set_target(SimpleNamespace(data="red mug"))

    def fail(_labels):
        raise RuntimeError("encoder unavailable")

    node.yolo = SimpleNamespace(set_classes=fail)
    node._acquire(node.target, node.rgb, 1)
    assert node.target.state == "error"
    assert node.detector_error == "encoder unavailable"
    assert node.target.mask is node.target.point is None
