"""Pure complete-trajectory validation before the first hardware write."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np
from myarm_m750_core.domain.models import (
    JointLimits,
    JointState,
    JointTrajectory,
    SafetyViolation,
    SafetyViolationType,
    ValidationResult,
)
from myarm_m750_core.ports.kinematics import KinematicsPort


@dataclass(frozen=True)
class SafetyPolicy:
    """Core safety limits with explicit model/limit provenance."""

    enabled: bool
    joint_names: Tuple[str, ...]
    joint_limits: JointLimits
    max_trajectory_points: int
    max_workspace_resample_samples: int
    state_timeout_s: float
    command_timeout_s: float
    stop_timeout_s: float
    max_joint_step_rad: float
    max_joint_velocity_rad_s: Tuple[float, ...]
    max_joint_acceleration_rad_s2: Tuple[float, ...]
    joint_limit_margin_rad: float
    workspace_minimum_m: Tuple[float, float, float]
    workspace_maximum_m: Tuple[float, float, float]
    workspace_resample_step_rad: float
    singularity_enabled: bool
    minimum_singularity_score: float
    model_fingerprint: str
    limit_provenance: str

    def __post_init__(self) -> None:
        if not self.enabled:
            raise ValueError(
                "The mandatory trajectory safety policy cannot be disabled."
            )
        budgets = (
            self.max_trajectory_points,
            self.max_workspace_resample_samples,
        )
        if any(
            not isinstance(value, int) or isinstance(value, bool) or value <= 0
            for value in budgets
        ):
            raise ValueError(
                "Trajectory point and workspace-resample budgets must be "
                "positive integers."
            )
        if (
            not math.isfinite(self.workspace_resample_step_rad)
            or self.workspace_resample_step_rad <= 0.0
        ):
            raise ValueError(
                "workspace_resample_step_rad must be finite and positive."
            )


class TrajectoryValidator:
    """Validate names, timing, derivatives, limits, freshness, and workspace."""

    def __init__(
        self,
        kinematics: KinematicsPort,
        policy: SafetyPolicy,
        monotonic_clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._kinematics = kinematics
        self._policy = policy
        self._monotonic_clock = monotonic_clock

    @property
    def policy(self) -> SafetyPolicy:
        """Return the immutable policy used for admission."""
        return self._policy

    def validate(
        self, trajectory: JointTrajectory, current_state: JointState
    ) -> ValidationResult:
        """Return all safety violations without causing side effects."""
        violations = []  # type: List[SafetyViolation]
        current_position = np.asarray(current_state.position_rad, dtype=float)
        current_is_finite = bool(np.all(np.isfinite(current_position)))
        if not current_is_finite:
            violations.append(
                SafetyViolation(
                    violation_type=SafetyViolationType.NON_FINITE_VALUE,
                    message="Measured joint state contains NaN or infinity.",
                )
            )

        now_s = self._monotonic_clock()
        if not math.isfinite(current_state.received_monotonic_s):
            state_age_s = math.inf
        else:
            state_age_s = current_state.age_s(now_s)
        if state_age_s > self._policy.state_timeout_s:
            violations.append(
                SafetyViolation(
                    violation_type=SafetyViolationType.STALE_STATE,
                    message="Measured state is older than the admission timeout.",
                    measured_value=state_age_s,
                    limit_value=self._policy.state_timeout_s,
                )
            )

        if tuple(trajectory.joint_names) != self._policy.joint_names:
            violations.append(
                SafetyViolation(
                    violation_type=SafetyViolationType.TRAJECTORY_TIME,
                    message=(
                        "Trajectory joint names/order do not match the canonical model."
                    ),
                )
            )
            return ValidationResult(tuple(violations))

        if len(trajectory.points) > self._policy.max_trajectory_points:
            violations.append(
                SafetyViolation(
                    violation_type=SafetyViolationType.TRAJECTORY_BUDGET,
                    message=(
                        "Trajectory point count exceeds the bounded admission "
                        "budget."
                    ),
                    measured_value=float(len(trajectory.points)),
                    limit_value=float(self._policy.max_trajectory_points),
                )
            )
            return ValidationResult(tuple(violations))

        workspace_sample_count = self._workspace_sample_count(
            current_position,
            trajectory,
        )
        if (
            workspace_sample_count is not None
            and workspace_sample_count
            > self._policy.max_workspace_resample_samples
        ):
            violations.append(
                SafetyViolation(
                    violation_type=SafetyViolationType.TRAJECTORY_BUDGET,
                    message=(
                        "Total workspace resample count exceeds the bounded "
                        "admission budget."
                    ),
                    measured_value=float(workspace_sample_count),
                    limit_value=float(
                        self._policy.max_workspace_resample_samples
                    ),
                )
            )
            return ValidationResult(tuple(violations))

        previous_position = current_position
        previous_time_s = 0.0
        previous_velocity = np.zeros(len(self._policy.joint_names), dtype=float)
        for point_index, point in enumerate(trajectory.points):
            position = np.asarray(point.position_rad, dtype=float)
            position_is_finite = bool(np.all(np.isfinite(position)))
            if not position_is_finite:
                violations.append(
                    SafetyViolation(
                        violation_type=SafetyViolationType.NON_FINITE_VALUE,
                        message=f"Point {point_index} contains NaN or infinity.",
                    )
                )
            point_time_s = float(point.time_from_start_s)
            time_is_valid = math.isfinite(point_time_s) and point_time_s >= 0.0
            if point_index > 0 and point_time_s <= previous_time_s:
                time_is_valid = False
            if not time_is_valid:
                violations.append(
                    SafetyViolation(
                        violation_type=SafetyViolationType.TRAJECTORY_TIME,
                        message=(
                            "Trajectory times must be finite, non-negative, and "
                            "strictly increasing."
                        ),
                    )
                )
            delta = position - previous_position
            self._validate_position(position, delta, point_index, violations)
            delta_time_s = point_time_s - previous_time_s if time_is_valid else 0.0
            derived_velocity = self._validate_velocity(
                point.velocity_rad_s,
                delta,
                delta_time_s,
                point_index,
                violations,
            )
            self._validate_acceleration(
                point.acceleration_rad_s2,
                derived_velocity,
                previous_velocity,
                delta_time_s,
                point_index,
                violations,
            )
            if current_is_finite and position_is_finite:
                self._validate_workspace_segment(
                    previous_position, position, point_index, violations
                )
            if time_is_valid:
                previous_position = position
                previous_time_s = point_time_s
                previous_velocity = derived_velocity
                current_is_finite = position_is_finite
        return ValidationResult(tuple(violations))

    def _workspace_sample_count(
        self,
        current_position: np.ndarray,
        trajectory: JointTrajectory,
    ) -> Optional[int]:
        """Estimate total FK samples without performing any FK computation."""
        if not bool(np.all(np.isfinite(current_position))):
            return None
        total = 0
        previous_position = current_position
        for point in trajectory.points:
            position = np.asarray(point.position_rad, dtype=float)
            if not bool(np.all(np.isfinite(position))):
                return None
            maximum_delta = float(np.max(np.abs(position - previous_position)))
            sample_ratio = (
                maximum_delta / self._policy.workspace_resample_step_rad
            )
            remaining_budget = (
                self._policy.max_workspace_resample_samples - total
            )
            if (
                not math.isfinite(sample_ratio)
                or sample_ratio > remaining_budget
            ):
                return self._policy.max_workspace_resample_samples + 1
            sample_count = max(
                1,
                int(math.ceil(sample_ratio)),
            )
            total += sample_count
            if total > self._policy.max_workspace_resample_samples:
                return total
            previous_position = position
        return total

    def _validate_position(
        self,
        position: np.ndarray,
        delta: np.ndarray,
        point_index: int,
        violations: List[SafetyViolation],
    ) -> None:
        limits = self._policy.joint_limits
        for index, joint_name in enumerate(self._policy.joint_names):
            value = float(position[index])
            lower = limits.lower_rad[index] + self._policy.joint_limit_margin_rad
            upper = limits.upper_rad[index] - self._policy.joint_limit_margin_rad
            if value < lower or value > upper:
                violations.append(
                    SafetyViolation(
                        violation_type=SafetyViolationType.JOINT_LIMIT,
                        message=f"Point {point_index} violates the model limit.",
                        joint_name=joint_name,
                        measured_value=value,
                        limit_value=lower if value < lower else upper,
                    )
                )
            step = abs(float(delta[index]))
            if step > self._policy.max_joint_step_rad + 1.0e-12:
                violations.append(
                    SafetyViolation(
                        violation_type=SafetyViolationType.JOINT_STEP,
                        message=f"Point {point_index} exceeds the maximum joint step.",
                        joint_name=joint_name,
                        measured_value=step,
                        limit_value=self._policy.max_joint_step_rad,
                    )
                )

    def _validate_velocity(
        self,
        provided: Optional[Sequence[float]],
        delta: np.ndarray,
        delta_time_s: float,
        point_index: int,
        violations: List[SafetyViolation],
    ) -> np.ndarray:
        if delta_time_s > 0.0:
            derived_velocity = delta / delta_time_s
        elif np.all(np.isfinite(delta)) and np.max(np.abs(delta)) <= 1.0e-12:
            derived_velocity = np.zeros(len(self._policy.joint_names), dtype=float)
        else:
            derived_velocity = np.full(
                len(self._policy.joint_names), math.inf, dtype=float
            )
        if provided is not None:
            self._validate_derivative(
                np.asarray(provided, dtype=float),
                self._policy.max_joint_velocity_rad_s,
                SafetyViolationType.JOINT_VELOCITY,
                "provided velocity",
                "velocity",
                point_index,
                violations,
            )
        self._validate_derivative(
            derived_velocity,
            self._policy.max_joint_velocity_rad_s,
            SafetyViolationType.JOINT_VELOCITY,
            "position/time-derived velocity",
            "velocity",
            point_index,
            violations,
        )
        return derived_velocity

    def _validate_derivative(
        self,
        values: np.ndarray,
        limits: Sequence[float],
        violation_type: SafetyViolationType,
        source_name: str,
        unit_name: str,
        point_index: int,
        violations: List[SafetyViolation],
    ) -> None:
        for index, joint_name in enumerate(self._policy.joint_names):
            raw_value = float(values[index])
            value = abs(raw_value)
            limit = limits[index]
            if not math.isfinite(raw_value):
                violations.append(
                    SafetyViolation(
                        violation_type=SafetyViolationType.NON_FINITE_VALUE,
                        message=(
                            f"Point {point_index} contains a non-finite "
                            f"{source_name}."
                        ),
                        joint_name=joint_name,
                    )
                )
            if not math.isfinite(value) or value > limit + 1.0e-12:
                violations.append(
                    SafetyViolation(
                        violation_type=violation_type,
                        message=(
                            f"Point {point_index} {source_name} exceeds the "
                            f"{unit_name} limit."
                        ),
                        joint_name=joint_name,
                        measured_value=value,
                        limit_value=limit,
                    )
                )

    def _validate_acceleration(
        self,
        provided: Optional[Sequence[float]],
        velocity: np.ndarray,
        previous_velocity: np.ndarray,
        delta_time_s: float,
        point_index: int,
        violations: List[SafetyViolation],
    ) -> None:
        if delta_time_s > 0.0:
            derived_acceleration = (velocity - previous_velocity) / delta_time_s
        elif (
            np.all(np.isfinite(velocity))
            and np.all(np.isfinite(previous_velocity))
            and np.max(np.abs(velocity - previous_velocity)) <= 1.0e-12
        ):
            derived_acceleration = np.zeros(
                len(self._policy.joint_names), dtype=float
            )
        else:
            derived_acceleration = np.full(
                len(self._policy.joint_names), math.inf, dtype=float
            )
        if provided is not None:
            self._validate_derivative(
                np.asarray(provided, dtype=float),
                self._policy.max_joint_acceleration_rad_s2,
                SafetyViolationType.JOINT_ACCELERATION,
                "provided acceleration",
                "acceleration",
                point_index,
                violations,
            )
        self._validate_derivative(
            derived_acceleration,
            self._policy.max_joint_acceleration_rad_s2,
            SafetyViolationType.JOINT_ACCELERATION,
            "position/time-derived acceleration",
            "acceleration",
            point_index,
            violations,
        )

    def _validate_workspace_segment(
        self,
        start: np.ndarray,
        end: np.ndarray,
        point_index: int,
        violations: List[SafetyViolation],
    ) -> None:
        maximum_delta = float(np.max(np.abs(end - start)))
        sample_count = max(
            1,
            int(math.ceil(maximum_delta / self._policy.workspace_resample_step_rad)),
        )
        for sample_index in range(1, sample_count + 1):
            ratio = sample_index / float(sample_count)
            sample = start + ratio * (end - start)
            pose = self._kinematics.compute_fk(sample)
            outside = False
            for axis_index, axis_name in enumerate(("x", "y", "z")):
                value = pose.translation_m[axis_index]
                minimum = self._policy.workspace_minimum_m[axis_index]
                maximum = self._policy.workspace_maximum_m[axis_index]
                if value < minimum or value > maximum:
                    outside = True
                    violations.append(
                        SafetyViolation(
                            violation_type=SafetyViolationType.WORKSPACE,
                            message=(
                                f"Workspace resample for point {point_index} "
                                f"violates {axis_name}."
                            ),
                            measured_value=value,
                            limit_value=minimum if value < minimum else maximum,
                        )
                    )
            if (
                not outside
                and self._policy.singularity_enabled
                and self._kinematics.compute_singularity_score(sample)
                < self._policy.minimum_singularity_score
            ):
                violations.append(
                    SafetyViolation(
                        violation_type=SafetyViolationType.SINGULARITY,
                        message="Workspace resample violates the singularity threshold.",
                        limit_value=self._policy.minimum_singularity_score,
                    )
                )
