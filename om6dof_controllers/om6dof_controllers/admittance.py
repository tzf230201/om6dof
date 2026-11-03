"""Bounded joint admittance mathematics, without ROS or motor writes.

The caller supplies a physical model torque and signed raw motor current in
the same joint order. The residual is an estimate, not a measured force:
``tau_external = tau_model - raw_current / raw_per_nm - torque_bias``.
Calibration, sign verification and the freshness decision belong to the
caller. No bias is learned from a potentially supported or contacted arm.

The absolute nominal target is never replaced by the displaced measurement.
A virtual spring therefore returns the displacement to zero after release.
Both the nominal reference and final command have rate governors. These limit
motion; they do not claim to solve an energy-optimal control problem.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real
from typing import Sequence


_N = 6


def _vector(values: Sequence[float], name: str) -> tuple[float, ...]:
    try:
        valid = len(values) == _N and all(
            isinstance(value, Real) and not isinstance(value, bool)
            and math.isfinite(float(value)) for value in values
        )
    except (TypeError, ValueError, OverflowError):
        valid = False
    if not valid:
        raise ValueError(f"{name} requires six finite numeric values")
    return tuple(float(value) for value in values)


def _clip(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass(frozen=True)
class AdmittanceConfig:
    lower: Sequence[float]
    upper: Sequence[float]
    raw_per_nm: Sequence[float]
    torque_bias: Sequence[float] = (0.0,) * _N
    inertia: Sequence[float] = (0.5,) * _N
    damping: Sequence[float] = (4.0,) * _N
    stiffness: Sequence[float] = (4.0,) * _N
    torque_on: Sequence[float] = (0.15,) * _N
    torque_off: Sequence[float] = (0.08,) * _N
    max_displacement: Sequence[float] = (0.15,) * _N
    max_velocity: Sequence[float] = (0.10,) * _N
    max_acceleration: Sequence[float] = (0.30,) * _N
    reference_velocity: Sequence[float] = (0.05,) * _N
    reference_acceleration: Sequence[float] = (0.15,) * _N
    max_tracking_error: Sequence[float] = (0.05,) * _N
    filter_time_constant: float = 0.08
    contact_dwell: float = 0.10
    release_dwell: float = 0.15
    max_dt: float = 0.10

    def __post_init__(self):
        arrays = (
            "lower", "upper", "raw_per_nm", "torque_bias", "inertia", "damping",
            "stiffness", "torque_on", "torque_off", "max_displacement",
            "max_velocity", "max_acceleration", "reference_velocity",
            "reference_acceleration",
            "max_tracking_error",
        )
        for name in arrays:
            object.__setattr__(self, name, _vector(getattr(self, name), name))
        for i in range(_N):
            if self.lower[i] >= self.upper[i]:
                raise ValueError("lower must be smaller than upper")
            for name in (
                "raw_per_nm", "inertia", "damping", "stiffness", "torque_on",
                "max_displacement", "max_velocity", "max_acceleration",
                "reference_velocity", "reference_acceleration",
                "max_tracking_error",
            ):
                if getattr(self, name)[i] <= 0.0:
                    raise ValueError(f"{name} must be positive")
            if not 0.0 <= self.torque_off[i] < self.torque_on[i]:
                raise ValueError("torque_off must satisfy 0 <= off < on")
        for name in ("filter_time_constant", "contact_dwell", "release_dwell", "max_dt"):
            value = getattr(self, name)
            if (not isinstance(value, Real) or isinstance(value, bool)
                    or not math.isfinite(value) or value <= 0.0):
                raise ValueError(f"{name} must be finite and positive")
        if self.max_dt > 0.1:
            raise ValueError("max_dt must not exceed 0.1 seconds")


@dataclass(frozen=True)
class AdmittanceResult:
    command: tuple[float, ...]
    nominal: tuple[float, ...]
    offset: tuple[float, ...]
    offset_velocity: tuple[float, ...]
    external_torque: tuple[float, ...]
    contact: tuple[bool, ...]
    valid: bool
    reason: str
    tracking_limited: bool = False


class JointAdmittance:
    """Six independent spring/damper displacements and bounded references.

    ``reset(q)`` starts a new explicitly requested target session from a fresh
    measured pose. ``step(q_nominal, ...)`` accepts the retained absolute target
    on every call. An unavailable model/contact estimate holds the last command
    and displacement; it does not infer that contact ended.
    """

    def __init__(self, config: AdmittanceConfig):
        if not isinstance(config, AdmittanceConfig):
            raise TypeError("config must be AdmittanceConfig")
        self.config = config
        self._initialized = False

    def reset(self, q: Sequence[float]) -> None:
        pose = _vector(q, "q")
        if any(not lo <= value <= hi for value, lo, hi in zip(
                pose, self.config.lower, self.config.upper)):
            raise ValueError("reset pose exceeds joint limits")
        self._nominal = list(pose)
        self._nominal_velocity = [0.0] * _N
        self._command = list(pose)
        self._command_velocity = [0.0] * _N
        self._offset = [0.0] * _N
        self._offset_velocity = [0.0] * _N
        self._external = [0.0] * _N
        self._contact = [False] * _N
        self._dwell = [0.0] * _N
        self._filter_initialized = False
        self._tracking_limited = False
        self._initialized = True

    @staticmethod
    def _govern(position, velocity, target, velocity_limit, acceleration_limit, dt, lower, upper):
        """Acceleration/speed bounded approach, anticipating the joint stops."""
        error = target - position
        snap_velocity = error / dt
        if (abs(snap_velocity) <= min(velocity_limit, acceleration_limit * dt)
                and abs(snap_velocity - velocity) <= acceleration_limit * dt):
            return target, snap_velocity
        braking_speed = max(0.0, math.hypot(acceleration_limit * dt,
                            math.sqrt(acceleration_limit) * math.sqrt(2.0 * abs(error)))
                            - acceleration_limit * dt)
        desired = math.copysign(min(velocity_limit, braking_speed), error) if error else 0.0
        # Include one integration interval in the braking distance. This avoids
        # hitting a hard joint boundary at full speed after the Euler step.
        margin = upper - position if desired >= 0.0 else position - lower
        bound = max(0.0, math.hypot(acceleration_limit * dt,
                                  math.sqrt(acceleration_limit) * math.sqrt(2.0 * max(0.0, margin)))
                    - acceleration_limit * dt)
        desired = _clip(desired, -bound, bound)
        next_velocity = _clip(desired, velocity - acceleration_limit * dt,
                              velocity + acceleration_limit * dt)
        next_velocity = _clip(next_velocity, -velocity_limit, velocity_limit)
        next_position = position + next_velocity * dt
        bounded = _clip(next_position, lower, upper)
        if bounded != next_position:
            # Hard position limits take precedence if a changed target makes
            # the acceleration and position constraints mutually infeasible.
            next_velocity = (bounded - position) / dt
        if abs(error) < 1.0e-9 and abs(next_velocity) < acceleration_limit * dt:
            return target, 0.0
        return bounded, next_velocity

    def _result(self, valid: bool, reason: str) -> AdmittanceResult:
        return AdmittanceResult(
            tuple(self._command), tuple(self._nominal), tuple(self._offset),
            tuple(self._offset_velocity), tuple(self._external), tuple(self._contact),
            valid, reason, self._tracking_limited,
        )

    def _freeze(self, reason: str) -> AdmittanceResult:
        self._nominal_velocity = [0.0] * _N
        self._command_velocity = [0.0] * _N
        self._offset_velocity = [0.0] * _N
        self._dwell = [0.0] * _N
        self._filter_initialized = False
        return self._result(False, reason)

    def step(self, q_nominal: Sequence[float], q_measured: Sequence[float],
             q_velocity: Sequence[float], raw_current: Sequence[float],
             model_torque: Sequence[float], dt: float,
             valid: bool = True) -> AdmittanceResult:
        if not self._initialized:
            raise RuntimeError("reset(q) is required before step")
        if valid is not True:
            return self._freeze("model or feedback is unavailable")
        if (not isinstance(dt, Real) or isinstance(dt, bool) or not math.isfinite(dt)
                or not 0.0 < dt <= self.config.max_dt):
            return self._freeze("invalid control interval")
        try:
            target = _vector(q_nominal, "q_nominal")
            measured = _vector(q_measured, "q_measured")
            _vector(q_velocity, "q_velocity")
            current = _vector(raw_current, "raw_current")
            model = _vector(model_torque, "model_torque")
            if any(not lo <= value <= hi for values in (target, measured)
                   for value, lo, hi in zip(values, self.config.lower, self.config.upper)):
                raise ValueError("target or measurement exceeds joint limits")
            residual = [model[i] - current[i] / self.config.raw_per_nm[i]
                        - self.config.torque_bias[i] for i in range(_N)]
            if not all(math.isfinite(value) for value in residual):
                raise ValueError("nonfinite torque residual")
        except (ValueError, TypeError, OverflowError) as exc:
            return self._freeze(str(exc))

        cfg = self.config
        self._tracking_limited = False
        alpha = dt / (cfg.filter_time_constant + dt)
        for i in range(_N):
            if self._filter_initialized:
                self._external[i] = ((1.0 - alpha) * self._external[i]
                                     + alpha * residual[i])
            else:
                self._external[i] = residual[i]
            level = abs(self._external[i])
            transition = (level <= cfg.torque_off[i] if self._contact[i]
                          else level >= cfg.torque_on[i])
            self._dwell[i] = self._dwell[i] + dt if transition else 0.0
            required = cfg.release_dwell if self._contact[i] else cfg.contact_dwell
            if self._dwell[i] + 1.0e-12 >= required:
                self._contact[i] = not self._contact[i]
                self._dwell[i] = 0.0

        contacted = any(self._contact)
        for i in range(_N):
            # Stop progressing the nominal path under a confirmed push. The
            # caller still retains the original absolute target, so release
            # resumes that path instead of adopting the displaced measurement.
            nominal_target = self._nominal[i] if contacted else target[i]
            lead_lower = max(cfg.lower[i], measured[i] - cfg.max_tracking_error[i])
            lead_upper = min(cfg.upper[i], measured[i] + cfg.max_tracking_error[i])
            # A contacted arm may be displaced farther than the tracking-lead
            # limit. Pulling the paused nominal reference into that moving
            # envelope would make it chase the push and ratchet indefinitely.
            bounded_target = (nominal_target if contacted else
                              _clip(nominal_target, lead_lower, lead_upper))
            self._tracking_limited |= bounded_target != nominal_target
            self._nominal[i], self._nominal_velocity[i] = self._govern(
                self._nominal[i], self._nominal_velocity[i], bounded_target,
                cfg.reference_velocity[i], cfg.reference_acceleration[i], dt,
                cfg.lower[i], cfg.upper[i],
            )
            force = self._external[i] if self._contact[i] else 0.0
            acceleration = (force - cfg.damping[i] * self._offset_velocity[i]
                            - cfg.stiffness[i] * self._offset[i]) / cfg.inertia[i]
            acceleration = _clip(acceleration, -cfg.max_acceleration[i], cfg.max_acceleration[i])
            velocity = _clip(self._offset_velocity[i] + acceleration * dt,
                             -cfg.max_velocity[i], cfg.max_velocity[i])
            offset = self._offset[i] + velocity * dt
            lo = max(-cfg.max_displacement[i], cfg.lower[i] - self._nominal[i])
            hi = min(cfg.max_displacement[i], cfg.upper[i] - self._nominal[i])
            bounded_offset = _clip(offset, lo, hi)
            if bounded_offset != offset and ((velocity < 0.0 and offset < lo)
                                              or (velocity > 0.0 and offset > hi)):
                velocity = 0.0  # No integrator windup against displacement/joint limits.
            self._offset[i] = bounded_offset
            self._offset_velocity[i] = velocity
            desired = self._nominal[i] + bounded_offset
            bounded_desired = _clip(desired, lead_lower, lead_upper)
            self._tracking_limited |= bounded_desired != desired
            self._command[i], self._command_velocity[i] = self._govern(
                self._command[i], self._command_velocity[i], bounded_desired,
                cfg.max_velocity[i], cfg.max_acceleration[i], dt,
                cfg.lower[i], cfg.upper[i],
            )
            # A measurement jump can put the existing command outside the
            # moving lead envelope. Preserve speed/acceleration continuity and
            # report the transient rather than snapping the commanded pose.
            self._tracking_limited |= not lead_lower <= self._command[i] <= lead_upper
        self._filter_initialized = True
        return self._result(True, "contact" if any(self._contact) else "returning or tracking")
