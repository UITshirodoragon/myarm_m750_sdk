"""Pure command and trajectory safety validation."""

from __future__ import annotations

import math
from typing import List, Sequence

import numpy as np

from myarm_m750_core.runtime.config.models import SafetyConfig
from myarm_m750_core.domain.models import (
    JointState,
    JointTarget,
    JointTrajectory,
    SafetyViolation,
    SafetyViolationType,
    ValidationResult,
)
from myarm_m750_core.ports.kinematics import KinematicsPort


class MotionGuard:
    """Centralized boundary checks for canonical joint commands."""

    def __init__(
        self,
        joint_names: Sequence[str],
        kinematics: KinematicsPort,
        config: SafetyConfig,
    ) -> None:
        self._joint_names = tuple(joint_names)
        self._kinematics = kinematics
        self._config = config

    def validate_joint_target(
        self, target: JointTarget, current: JointState
    ) -> ValidationResult:
        """Validate one target without modifying or sending it."""
        if not self._config.enabled:
            return ValidationResult()
        violations: List[SafetyViolation] = []
        target_array = np.asarray(target.position_rad, dtype=float)
        current_array = np.asarray(current.position_rad, dtype=float)

        if self._config.reject_nan_or_inf and not np.all(np.isfinite(target_array)):
            violations.append(
                SafetyViolation(
                    violation_type=SafetyViolationType.NON_FINITE_VALUE,
                    message="Joint target contains NaN or infinity.",
                )
            )
            return ValidationResult(tuple(violations))

        if not current.is_fresh(self._config.state_timeout_s):
            violations.append(
                SafetyViolation(
                    violation_type=SafetyViolationType.STALE_STATE,
                    message="Measured joint state is older than the configured timeout.",
                    measured_value=current.age_s(),
                    limit_value=self._config.state_timeout_s,
                )
            )

        limits = self._kinematics.joint_limits
        margin_rad = self._config.joint_limit_margin_rad
        for joint_index, joint_name in enumerate(self._joint_names):
            lower_rad = limits.lower_rad[joint_index] + margin_rad
            upper_rad = limits.upper_rad[joint_index] - margin_rad
            target_rad = target.position_rad[joint_index]
            if target_rad < lower_rad or target_rad > upper_rad:
                violations.append(
                    SafetyViolation(
                        violation_type=SafetyViolationType.JOINT_LIMIT,
                        joint_name=joint_name,
                        message=(
                            "Target for {0} is outside the configured model limit margin."
                        ).format(joint_name),
                        measured_value=target_rad,
                        limit_value=lower_rad if target_rad < lower_rad else upper_rad,
                    )
                )

            joint_step_rad = abs(target_rad - current.position_rad[joint_index])
            if joint_step_rad > self._config.max_joint_step_rad + 1.0e-12:
                violations.append(
                    SafetyViolation(
                        violation_type=SafetyViolationType.JOINT_STEP,
                        joint_name=joint_name,
                        message=(
                            "Single command step for {0} exceeds max_joint_step_rad."
                        ).format(joint_name),
                        measured_value=joint_step_rad,
                        limit_value=self._config.max_joint_step_rad,
                    )
                )

        if any(
            violation.violation_type
            in (SafetyViolationType.NON_FINITE_VALUE, SafetyViolationType.JOINT_LIMIT)
            for violation in violations
        ):
            return ValidationResult(tuple(violations))

        pose = self._kinematics.compute_fk(target.position_rad)
        for axis_index, axis_name in enumerate(("x", "y", "z")):
            position_m = pose.translation_m[axis_index]
            minimum_m = self._config.workspace.minimum_m[axis_index]
            maximum_m = self._config.workspace.maximum_m[axis_index]
            if position_m < minimum_m or position_m > maximum_m:
                violations.append(
                    SafetyViolation(
                        violation_type=SafetyViolationType.WORKSPACE,
                        message=(
                            "End-link {0}-position is outside the configured workspace."
                        ).format(axis_name),
                        measured_value=position_m,
                        limit_value=minimum_m if position_m < minimum_m else maximum_m,
                    )
                )

        if self._config.singularity.enabled:
            singularity_score = self._kinematics.compute_singularity_score(
                target.position_rad
            )
            if singularity_score < self._config.singularity.minimum_score:
                violations.append(
                    SafetyViolation(
                        violation_type=SafetyViolationType.SINGULARITY,
                        message="Target singularity score is below the configured minimum.",
                        measured_value=singularity_score,
                        limit_value=self._config.singularity.minimum_score,
                    )
                )

        return ValidationResult(tuple(violations))

    def validate_trajectory(
        self, trajectory: JointTrajectory, current: JointState
    ) -> ValidationResult:
        """Validate every point and the time sequence of a trajectory."""
        if not self._config.enabled:
            return ValidationResult()
        violations: List[SafetyViolation] = []
        previous_state = current
        previous_time_s = 0.0
        for point_index, point in enumerate(trajectory.points):
            if point.time_from_start_s < previous_time_s:
                violations.append(
                    SafetyViolation(
                        violation_type=SafetyViolationType.TRAJECTORY_TIME,
                        message="Trajectory times are not monotonic at point {0}.".format(
                            point_index
                        ),
                    )
                )
                break
            point_result = self.validate_joint_target(
                JointTarget(point.position_rad), previous_state
            )
            violations.extend(point_result.violations)
            previous_state = JointState(
                position_rad=point.position_rad,
                timestamp_s=current.timestamp_s,
                source="trajectory_validation",
                sequence=point_index,
            )
            previous_time_s = point.time_from_start_s
        return ValidationResult(tuple(violations))
