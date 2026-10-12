"""Hardware-independent nominal trajectory plus persistent world-frame trim.

This module computes references, not motor commands. A reference must pass
live whole-body/environment validation before any hardware consumer uses it.
"""
from dataclasses import dataclass
import math

import numpy as np

from .control_math import rotation_error, rotation_from_rotvec


def vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f'{name}: expected {size} finite values')
    return result.copy()


@dataclass(frozen=True)
class Reference:
    phase: float
    nominal_joints: np.ndarray
    nominal_position: np.ndarray
    nominal_rotation: np.ndarray
    position: np.ndarray
    rotation: np.ndarray
    offset: np.ndarray
    offset_rotation: np.ndarray
    finished: bool


class CombinedReference:
    """Both inputs remain active. Neutral/expired Twist preserves accumulated trim.

    Linear AND angular Twist are expressed in world. Translation is accumulated
    in world; rotation increments premultiply the stored rotation offset. Joint
    references are piecewise linear positions (not a trajectory execution law).
    Timing is local monotonic elapsed time; no ROS or wall-clock dependency.
    """
    def __init__(self, fk, linear_limit=0.05, angular_limit=0.5,
                 translation_limit=0.10, rotation_limit=0.5, timeout=0.3):
        limits = (linear_limit, angular_limit, translation_limit,
                  rotation_limit, timeout)
        if not all(math.isfinite(x) and x > 0 for x in limits):
            raise ValueError('limits and timeout must be finite and positive')
        self.fk = fk
        self.linear_limit, self.angular_limit = linear_limit, angular_limit
        self.translation_limit, self.rotation_limit = translation_limit, rotation_limit
        self.timeout = timeout
        self.offset = np.zeros(3)
        self.offset_rotation = np.eye(3)
        self.velocity = np.zeros(6)
        self.received = -math.inf
        self.times = None
        self.joints = None
        self.started = None
        self.last_tick = None
        self.last_twist_stamp = -math.inf

    def start(self, times, positions, now):
        times = np.asarray(times, dtype=float)
        positions = np.asarray(positions, dtype=float)
        if (times.ndim != 1 or len(times) < 2 or not np.isfinite(times).all()
                or times[0] != 0 or np.any(np.diff(times) <= 0)
                or positions.shape != (len(times), 6)
                or not np.isfinite(positions).all() or not math.isfinite(now)):
            raise ValueError('trajectory needs 2+ ordered finite six-joint points starting at t=0')
        # Preserve accumulated trim, but never carry a held command into a new
        # reference. A subsequent fresh Twist is required to accumulate more.
        self.times, self.joints = times.copy(), positions.copy()
        self.started = self.last_tick = now
        self.velocity[:] = 0
        self.received = -math.inf

    def twist(self, values, stamp, now, frame='world'):
        try:
            velocity = vector(values, 6, 'twist')
            if (frame != 'world' or not math.isfinite(now) or not math.isfinite(stamp)
                    or stamp <= self.last_twist_stamp or stamp > now
                    or now - stamp > self.timeout):
                raise ValueError('twist frame/stamp invalid, expired, or out of order')
            if np.linalg.norm(velocity[:3]) > self.linear_limit + 1e-12:
                raise ValueError('linear twist exceeds limit')
            if np.linalg.norm(velocity[3:]) > self.angular_limit + 1e-12:
                raise ValueError('angular twist exceeds limit')
        except (TypeError, ValueError):
            # Invalid packets stop integration, never refresh an older command.
            self.velocity[:] = 0
            self.received = -math.inf
            raise
        if self.started is not None:
            self._integrate(now)
        self.last_twist_stamp = stamp
        self.velocity = velocity
        self.received = stamp

    def reset_offset(self):
        self.offset[:] = 0
        self.offset_rotation = np.eye(3)
        self.velocity[:] = 0
        self.received = -math.inf

    def _integrate(self, now):
        if not math.isfinite(now) or now < self.last_tick:
            self.velocity[:] = 0
            self.received = -math.inf
            raise ValueError('clock moved backwards or is invalid')
        # Integrate only the interval covered by a fresh packet. Long timer
        # gaps never accumulate an old velocity for the duration of that gap.
        dt = max(0.0, min(now, self.received + self.timeout)
                 - max(self.last_tick, self.received))
        self.last_tick = now
        proposed = self.offset + self.velocity[:3] * dt
        length = np.linalg.norm(proposed)
        if length > self.translation_limit:
            proposed *= self.translation_limit / length
        rotation = rotation_from_rotvec(self.velocity[3:] * dt) @ self.offset_rotation
        angle = np.linalg.norm(rotation_error(rotation, np.eye(3)))
        if angle <= self.rotation_limit + 1e-12:
            self.offset_rotation = rotation
        self.offset = proposed
    def tick(self, now):
        if self.started is None:
            return None
        self._integrate(now)
        phase = min(now - self.started, self.times[-1])
        nominal = np.array([np.interp(phase, self.times, self.joints[:, i])
                            for i in range(6)])
        position, rotation = self.fk(nominal)
        position = vector(position, 3, 'FK position')
        rotation = np.asarray(rotation, dtype=float)
        if (rotation.shape != (3, 3) or not np.isfinite(rotation).all()
                or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
                or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6)):
            raise ValueError('FK rotation invalid')
        return Reference(float(phase), nominal, position, rotation,
                         position + self.offset, self.offset_rotation @ rotation,
                         self.offset.copy(), self.offset_rotation.copy(),
                         bool(phase >= self.times[-1]))
