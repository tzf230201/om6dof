"""Google Gemini REST client for the two reasoning steps in the pick loop.

The ROBOTIS OMY story uses Gemini for exactly two jobs, and so does this:

``classify``
    After the grasp, a crop of the colour image centred on the picked object is
    sent up and Gemini names it. The name selects the place bin.

``locate``
    Before the grasp, the full colour image plus a plain-text description
    ("the red screwdriver") comes back as a pixel, which narrows the grasp
    candidates down to the object the operator asked for.

Called over plain HTTPS with ``requests`` — the ``google-genai`` SDK is not
installed on this machine and is not needed for two endpoints. The API key is
never logged, never put in a URL, and never written to a parameter dump; it is
read from an environment variable or a mode-600 file at call time.

Standalone probe, to check a key without touching the robot::

    ros2 run om6dof_pick_and_place_gemini gemini_probe --image /tmp/frame.jpg
"""

from __future__ import annotations

import base64
import json
import os
import re
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

ENDPOINT = ("https://generativelanguage.googleapis.com/v1beta/"
            "models/{model}:generateContent")
DEFAULT_MODEL = "gemini-3.5-flash-lite"
DEFAULT_KEY_ENV = "GEMINI_API_KEY"
DEFAULT_KEY_FILE = "~/.config/om6dof/gemini_api_key"

# Gemini 2.x reports 2-D points and boxes on a 0-1000 normalised grid.
NORMALISED_SPAN = 1000.0


class GeminiError(RuntimeError):
    """The request went out and came back unusable."""


class GeminiUnavailable(GeminiError):
    """No key, or no ``requests`` — the caller should degrade, not crash."""


@dataclass
class Classification:
    label: str
    confidence: float
    reason: str = ""
    raw: str = ""


@dataclass
class Localization:
    pixel: Tuple[float, float]
    box: Optional[Tuple[float, float, float, float]] = None   # x0, y0, x1, y1
    confidence: float = 0.0
    found: bool = True
    reason: str = ""
    raw: str = ""


def resolve_api_key(explicit: str = "", env_var: str = DEFAULT_KEY_ENV,
                    key_file: str = DEFAULT_KEY_FILE) -> str:
    """First of: an explicit key, ``$env_var``, the first line of ``key_file``.

    Returns ``""`` when nothing is configured; callers turn that into a clear
    "Gemini disabled" message rather than a mid-sequence failure.
    """
    if explicit and explicit.strip() and not explicit.startswith("<"):
        return explicit.strip()
    from_env = os.environ.get(env_var, "").strip()
    if from_env:
        return from_env
    path = os.path.expanduser(key_file or "")
    if path and os.path.isfile(path):
        with open(path, "r", encoding="utf-8") as handle:
            return handle.readline().strip()
    return ""


def _strip_json_fence(text: str) -> str:
    """Gemini often wraps JSON in a ```json fence even when asked not to."""
    text = text.strip()
    fence = re.match(r"^```(?:json)?\s*(.*?)\s*```$", text,
                     re.DOTALL | re.IGNORECASE)
    if fence:
        return fence.group(1).strip()
    return text


def _balanced_json_objects(text: str) -> List[str]:
    """Every top-level ``{...}``/``[...]`` substring, tracking bracket depth.

    A plain greedy bracket regex is unsafe when the prose itself contains a
    itself contains a brace — a model narrating "JSON format: `{"label": ...}`"
    before its real answer, which Gemma does even with
    ``responseMimeType: application/json`` set — spans from the *first* brace
    in the prose to the *last* brace in the real answer, splicing the two into
    one invalid blob. Tracking depth keeps each candidate self-contained.
    """
    candidates = []
    stack: List[str] = []
    start = -1
    opening = {"{": "}", "[": "]"}
    closing = {"}": "{", "]": "["}
    for index, char in enumerate(text):
        if char in opening:
            if not stack:
                start = index
            stack.append(opening[char])
        elif char in closing:
            if stack and stack[-1] == char:
                stack.pop()
                if not stack and start >= 0:
                    candidates.append(text[start:index + 1])
                    start = -1
            else:
                stack.clear()
                start = -1
    return candidates


def parse_json_payload(text: str) -> dict:
    """Parse a model reply into a dict, tolerating prose around the JSON.

    When several balanced ``{...}``/``[...]`` spans are present — a model that
    narrates its reasoning before answering, as Gemma does — the *last* one
    that actually parses wins, since that is where the final answer lives.
    """
    stripped = _strip_json_fence(text)
    try:
        parsed = json.loads(stripped)
    except json.JSONDecodeError:
        parsed = None
        for candidate in reversed(_balanced_json_objects(stripped)):
            try:
                parsed = json.loads(candidate)
                break
            except json.JSONDecodeError:
                continue
        if parsed is None:
            raise GeminiError(f"no JSON in reply: {text[:200]!r}") from None
    if isinstance(parsed, list):
        if not parsed:
            raise GeminiError("model returned an empty list")
        parsed = parsed[0]
    if not isinstance(parsed, dict):
        raise GeminiError(f"expected a JSON object, got {type(parsed).__name__}")
    return parsed


def encode_jpeg(image_bgr: np.ndarray, quality: int = 85) -> bytes:
    import cv2

    ok, buffer = cv2.imencode(".jpg", image_bgr,
                              [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise GeminiError("cv2.imencode failed on the crop")
    return buffer.tobytes()


def crop_around(image_bgr: np.ndarray, pixel: Sequence[float],
                half_size: int, bbox=None, pad: int = 16) -> np.ndarray:
    """Square crop around a pixel, or a padded bounding box when one is known."""
    height, width = image_bgr.shape[:2]
    if bbox is not None:
        x0, y0, x1, y1 = [int(v) for v in bbox]
        x0, y0 = max(0, x0 - pad), max(0, y0 - pad)
        x1, y1 = min(width - 1, x1 + pad), min(height - 1, y1 + pad)
    else:
        cx, cy = int(pixel[0]), int(pixel[1])
        x0, y0 = max(0, cx - half_size), max(0, cy - half_size)
        x1, y1 = min(width - 1, cx + half_size), min(height - 1, cy + half_size)
    if x1 <= x0 or y1 <= y0:
        return image_bgr
    return image_bgr[y0:y1 + 1, x0:x1 + 1]


class GeminiClient:
    """Two prompts, one endpoint, no SDK."""

    def __init__(self, *, api_key: str = "", model: str = DEFAULT_MODEL,
                 key_env: str = DEFAULT_KEY_ENV,
                 key_file: str = DEFAULT_KEY_FILE,
                 timeout_sec: float = 20.0, max_retries: int = 2,
                 temperature: float = 0.0, jpeg_quality: int = 85,
                 logger=None) -> None:
        self.model = str(model)
        self.timeout_sec = float(timeout_sec)
        self.max_retries = int(max_retries)
        self.temperature = float(temperature)
        self.jpeg_quality = int(jpeg_quality)
        self._log = logger
        self._key = resolve_api_key(api_key, key_env, key_file)
        self._key_source = ("parameter" if api_key.strip()
                            and not api_key.startswith("<")
                            else f"${key_env}" if os.environ.get(key_env)
                            else key_file if self._key else "none")

    @property
    def enabled(self) -> bool:
        return bool(self._key)

    def describe(self) -> str:
        """Status line safe to log — says where the key came from, never what it is."""
        if not self._key:
            return "Gemini DISABLED (no API key configured)"
        return (f"Gemini enabled, model={self.model}, "
                f"key from {self._key_source} ({len(self._key)} chars)")

    # ---------------- transport ----------------
    def _post(self, parts: List[dict]) -> str:
        if not self._key:
            raise GeminiUnavailable(
                "no Gemini API key: set $GEMINI_API_KEY, write "
                f"{DEFAULT_KEY_FILE}, or set the gemini.api_key parameter")
        try:
            import requests
        except ImportError as exc:      # pragma: no cover - packaged everywhere
            raise GeminiUnavailable("python3-requests is not installed") from exc

        url = ENDPOINT.format(model=self.model)
        body = {
            "contents": [{"role": "user", "parts": parts}],
            "generationConfig": {
                "temperature": self.temperature,
                "responseMimeType": "application/json",
            },
        }
        headers = {"x-goog-api-key": self._key,
                   "Content-Type": "application/json"}

        last_error = ""
        for attempt in range(self.max_retries + 1):
            try:
                response = requests.post(url, headers=headers, json=body,
                                         timeout=self.timeout_sec)
            except Exception as exc:    # noqa: BLE001 - any transport failure retries
                last_error = f"transport: {exc}"
            else:
                if response.status_code == 200:
                    return self._extract_text(response.json())
                # 429/5xx are worth another go; 4xx otherwise is a real fault.
                last_error = f"HTTP {response.status_code}: {response.text[:200]}"
                if response.status_code < 500 and response.status_code != 429:
                    raise GeminiError(last_error)
            if attempt < self.max_retries:
                delay = 0.5 * (2 ** attempt)
                if self._log:
                    self._log.warn(f"Gemini retry {attempt + 1}: {last_error}")
                time.sleep(delay)
        raise GeminiError(f"Gemini request failed: {last_error}")

    @staticmethod
    def _extract_text(payload: dict) -> str:
        candidates = payload.get("candidates") or []
        if not candidates:
            blocked = payload.get("promptFeedback", {}).get("blockReason")
            raise GeminiError(f"no candidates in reply (blockReason={blocked})")
        parts = candidates[0].get("content", {}).get("parts") or []
        text = "".join(part.get("text", "") for part in parts)
        if not text.strip():
            raise GeminiError("empty text in reply")
        return text

    def _image_part(self, image_bgr: np.ndarray) -> dict:
        jpeg = encode_jpeg(image_bgr, self.jpeg_quality)
        return {"inline_data": {"mime_type": "image/jpeg",
                                "data": base64.b64encode(jpeg).decode("ascii")}}

    # ---------------- prompts ----------------
    def classify(self, image_bgr: np.ndarray,
                 categories: Sequence[str]) -> Classification:
        """Name the object filling the crop, restricted to ``categories``."""
        allowed = ", ".join(str(c) for c in categories)
        prompt = (
            "You are the vision step of a robot pick-and-place cell. The image "
            "is a close crop of a single object the gripper just picked up.\n"
            f"Classify it as exactly one of: {allowed}.\n"
            "If none of them fit, answer with the last category in the list.\n"
            'Reply with JSON only: {"label": "<one of the categories>", '
            '"confidence": <0.0-1.0>, "reason": "<max 10 words>"}')
        text = self._post([{"text": prompt}, self._image_part(image_bgr)])
        data = parse_json_payload(text)
        label = str(data.get("label", "")).strip().lower()
        known = {str(c).strip().lower(): str(c) for c in categories}
        return Classification(
            label=known.get(label, str(categories[-1]) if categories else label),
            confidence=float(data.get("confidence", 0.0) or 0.0),
            reason=str(data.get("reason", ""))[:120],
            raw=text,
        )

    def locate(self, image_bgr: np.ndarray, description: str) -> Localization:
        """Find the described object and return its pixel in image coordinates."""
        height, width = image_bgr.shape[:2]
        prompt = (
            "You are the vision step of a robot pick-and-place cell looking at "
            "a table.\n"
            f'Find this object: "{description}".\n'
            "Answer with JSON only, using a 0-1000 normalised grid over the "
            "image:\n"
            '{"found": true|false, "point": [y, x], '
            '"box_2d": [ymin, xmin, ymax, xmax], "confidence": <0.0-1.0>, '
            '"reason": "<max 10 words>"}\n'
            '"point" must be the centre of the object, on it, not beside it. '
            'If the object is not visible answer {"found": false}.')
        text = self._post([{"text": prompt}, self._image_part(image_bgr)])
        data = parse_json_payload(text)
        return localization_from_payload(data, width, height, raw=text)


def localization_from_payload(data: dict, width: int, height: int,
                              raw: str = "") -> Localization:
    """Turn Gemini's 0-1000 grid answer into pixels. Also used by the tests."""
    found = bool(data.get("found", True))
    reason = str(data.get("reason", ""))[:120]
    confidence = float(data.get("confidence", 0.0) or 0.0)

    box = None
    raw_box = data.get("box_2d") or data.get("box")
    if isinstance(raw_box, (list, tuple)) and len(raw_box) == 4:
        ymin, xmin, ymax, xmax = [float(v) for v in raw_box]
        box = (xmin / NORMALISED_SPAN * width, ymin / NORMALISED_SPAN * height,
               xmax / NORMALISED_SPAN * width, ymax / NORMALISED_SPAN * height)

    pixel = None
    raw_point = data.get("point")
    if isinstance(raw_point, (list, tuple)) and len(raw_point) == 2:
        y_norm, x_norm = float(raw_point[0]), float(raw_point[1])
        pixel = (x_norm / NORMALISED_SPAN * width,
                 y_norm / NORMALISED_SPAN * height)
    elif box is not None:
        pixel = (0.5 * (box[0] + box[2]), 0.5 * (box[1] + box[3]))

    if not found or pixel is None:
        return Localization(pixel=(0.0, 0.0), box=box, confidence=confidence,
                            found=False, reason=reason or "not found", raw=raw)
    pixel = (float(np.clip(pixel[0], 0, width - 1)),
             float(np.clip(pixel[1], 0, height - 1)))
    return Localization(pixel=pixel, box=box, confidence=confidence,
                        found=True, reason=reason, raw=raw)


def _capture_realsense(*, serial: str, width: int, height: int, fps: int,
                       warmup: int, save_to: str):
    """One BGR frame straight off the RealSense — no ROS, no node needed."""
    from .rgbd_source import RealSenseSource

    source = RealSenseSource(width=width, height=height, fps=fps, serial=serial)
    print(f"opening RealSense (serial={serial or 'first found'}, "
          f"{width}x{height}@{fps})...")
    try:
        source.start()
        frame = source.capture(warmup=warmup)
    finally:
        source.stop()
    if frame is None:
        raise RuntimeError("RealSense returned no frame (timed out)")
    print(f"captured {frame.color.shape[1]}x{frame.color.shape[0]} "
          f"at {frame.stamp:.3f}")
    if save_to:
        import cv2
        cv2.imwrite(save_to, frame.color)
        print(f"saved colour frame to {save_to}")
    return frame.color


def main(argv=None) -> int:
    """``gemini_probe`` — check the key and both prompts without the robot.

    Image source is one of, in this priority: ``--realsense`` (captures live
    from the wrist D405 over USB — no ROS needed), ``--image`` (a file), or a
    grey test card if neither is given.
    """
    import argparse

    import cv2

    parser = argparse.ArgumentParser(description="Probe the Gemini API key.")
    parser.add_argument("--realsense", action="store_true",
                        help="capture one live frame from a USB RealSense "
                             "instead of a file or the test card")
    parser.add_argument("--serial", default="",
                        help="RealSense serial number (default: first found)")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--warmup", type=int, default=10,
                        help="frames to discard while auto-exposure settles")
    parser.add_argument("--save", default="",
                        help="also write the captured/loaded frame to this path")
    parser.add_argument("--image", default="",
                        help="image file to send; a grey test card if omitted")
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--describe", default="",
                        help="run locate() with this description instead of classify()")
    parser.add_argument("--categories", default="bottle,cup,box,tool,unknown")
    args = parser.parse_args(argv)

    if args.realsense:
        try:
            image = _capture_realsense(
                serial=args.serial, width=args.width, height=args.height,
                fps=args.fps, warmup=args.warmup, save_to=args.save)
        except Exception as exc:   # noqa: BLE001 - report, don't traceback
            print(f"FAILED to capture from RealSense: {exc}")
            return 2
    elif args.image:
        image = cv2.imread(args.image)
        if image is None:
            print(f"cannot read {args.image}")
            return 2
    else:
        image = np.full((240, 320, 3), 128, np.uint8)
        cv2.circle(image, (160, 120), 50, (40, 40, 200), -1)
        if args.save:
            cv2.imwrite(args.save, image)
            print(f"saved test card to {args.save}")

    client = GeminiClient(model=args.model)
    print(client.describe())
    if not client.enabled:
        return 1
    try:
        if args.describe:
            print(client.locate(image, args.describe))
        else:
            print(client.classify(image, args.categories.split(",")))
    except GeminiError as exc:
        print(f"FAILED: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
