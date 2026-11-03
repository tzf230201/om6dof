"""YOLOE instance-segmentation adapter with an optional Ultralytics runtime.

The import is deliberately deferred: the existing YOLOX/OpenCV deployment must
continue to start on minimal ROS images where PyTorch and Ultralytics are not
installed.  A TensorRT engine must be built on the target Jetson after its
text/visual prompts have been frozen into the export.
"""

from dataclasses import dataclass
from pathlib import Path
import ctypes
import site
import threading

import cv2
import numpy as np


def _preload_jetson_cudss():
    """Load cuDSS when it was installed as a user-level CUDA wheel.

    Recent Jetson PyTorch wheels link against ``libcudss.so.0``.  On the
    system CUDA installation it is normally present in the dynamic-loader
    path; when it is supplied by ``nvidia-cudss-cu12`` instead, preload it
    before Ultralytics imports PyTorch.  Missing cuDSS is left for PyTorch to
    report with its normal, actionable import error.
    """
    try:
        user_site = Path(site.getusersitepackages())
    except (AttributeError, TypeError):
        return
    cudss = user_site / 'nvidia' / 'cu12' / 'lib' / 'libcudss.so.0'
    if cudss.is_file():
        ctypes.CDLL(str(cudss), mode=ctypes.RTLD_GLOBAL)


@dataclass(frozen=True)
class SegmentationDetection:
    """One source-image detection; ``mask`` is a bool image in source pixels."""

    bbox: tuple
    score: float
    class_id: int
    class_name: str
    mask: np.ndarray


class YoloESegDetector:
    """Run a promptable YOLOE checkpoint or a frozen TensorRT engine.

    ``.pt`` checkpoints accept new text prompts during inference. ``.engine``
    models are intentionally prompt-static: the labels must have been baked
    into the engine at export time, which avoids a text-encoder cost per frame.
    """

    def __init__(self, model_path, classes, *, confidence=.35, imgsz=640,
                 device=''):
        path = Path(model_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(
                f'YOLOE model not found: {path}. Set yoloe_model_path to the '
                'actual .engine or .pt file; /path/... in the documentation is '
                'only a placeholder.')
        self.model_path = str(path)
        labels = tuple(dict.fromkeys(str(name).strip() for name in classes if str(name).strip()))
        if not labels:
            raise ValueError('yoloe_classes must contain at least one prompt')
        self.confidence = float(confidence)
        self.imgsz = int(imgsz)
        if self.imgsz < 32:
            raise ValueError('yoloe_imgsz must be at least 32')
        self.device = str(device).strip()
        self._lock = threading.RLock()
        self.classes = ()
        try:
            _preload_jetson_cudss()
            from ultralytics import YOLO, YOLOE
        except ImportError as error:
            raise RuntimeError(
                'detector_backend=yoloe requires the optional ultralytics '
                'runtime on this machine') from error
        self.is_engine = path.suffix.lower() == '.engine'
        self.supports_open_vocabulary = not self.is_engine
        if self.is_engine:
            # An exported YOLOE engine is an ordinary, prompt-static YOLO
            # graph.  Re-applying prompts to it is unsupported by Ultralytics.
            self.model = YOLO(self.model_path)
            self.classes = labels
        else:
            import torch

            self.model = YOLOE(self.model_path)
            self.model.to(self.device or ('cuda:0' if torch.cuda.is_available() else 'cpu'))
            # Reuse the encoder stored beside the checkpoint, independent of
            # the directory from which ros2 launch was invoked.
            variant = self.model.model.text_model.split(':')[0]
            encoder_name = {'mobileclip2': 'mobileclip2_b.ts',
                            'mobileclip': 'mobileclip_blt.ts'}.get(variant)
            encoder_path = path.parent / encoder_name if encoder_name else None
            if encoder_path is not None and encoder_path.is_file():
                from ultralytics.nn.text_model import MobileCLIPTS

                self.model.model.clip_model = MobileCLIPTS(
                    device=next(self.model.model.parameters()).device,
                    weight=str(encoder_path))
            self.set_classes(labels)

    def set_classes(self, classes):
        """Encode prompts only when they change; serialize with prediction."""
        labels = tuple(dict.fromkeys(str(name).strip() for name in classes if str(name).strip()))
        if not labels:
            raise ValueError('YOLOE requires at least one nonempty text prompt')
        with self._lock:
            if labels == self.classes:
                return
            if self.is_engine:
                raise ValueError('This TensorRT engine has frozen labels; use a .pt checkpoint for free text')
            embeddings = self.model.model.get_text_pe(list(labels), cache_clip_model=True)
            self.model.set_classes(list(labels), embeddings=embeddings)
            self.classes = labels

    def detect(self, bgr):
        with self._lock:
            results = self.model.predict(
                source=bgr, imgsz=self.imgsz, conf=self.confidence,
                device=self.device or None, verbose=False, retina_masks=True,
            )
        if not results:
            return []
        result = results[0]
        if result.boxes is None or result.masks is None:
            return []
        boxes = result.boxes.xyxy.cpu().numpy()
        scores = result.boxes.conf.cpu().numpy()
        class_ids = result.boxes.cls.cpu().numpy().astype(int)
        masks = result.masks.data.cpu().numpy()
        height, width = bgr.shape[:2]
        names = result.names
        detections = []
        for box, score, class_id, mask in zip(boxes, scores, class_ids, masks):
            x0, y0, x1, y1 = box
            x0 = max(0, min(width - 1, int(round(x0))))
            y0 = max(0, min(height - 1, int(round(y0))))
            x1 = max(x0 + 1, min(width, int(round(x1))))
            y1 = max(y0 + 1, min(height, int(round(y1))))
            if mask.shape != (height, width):
                mask = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)
            label = names.get(class_id, str(class_id)) if isinstance(names, dict) else names[class_id]
            # Engines produced with frozen prompts may expose numeric labels;
            # recover the configured label when its class index is known.
            if str(label) == str(class_id) and 0 <= class_id < len(self.classes):
                label = self.classes[class_id]
            detections.append(SegmentationDetection(
                bbox=(x0, y0, x1 - x0, y1 - y0), score=float(score),
                class_id=class_id, class_name=str(label), mask=mask.astype(bool),
            ))
        return detections
