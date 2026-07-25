"""Shared deterministic damped-least-squares inverse kinematics."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Sequence

import numpy as np
from myarm_m750_core.domain.errors import KinematicsError
from myarm_m750_core.domain.kinematics.math3d import rotation_log_vector
from myarm_m750_core.domain.models import IkResult, JointLimits, RigidTransform


@dataclass(frozen=True)
class DampedLeastSquaresSettings:
    """Numerical IK settings with explicit SI units."""

    max_iterations: int = 250
    damping: float = 0.02
    max_step_rad: float = 0.20
    position_tolerance_m: float = 1.0e-4
    orientation_tolerance_rad: float = 1.0e-3

    def __post_init__(self) -> None:
        if self.max_iterations < 1:
            raise ValueError("max_iterations must be positive.")
        if self.damping <= 0.0:
            raise ValueError("damping must be positive.")
        if self.max_step_rad <= 0.0:
            raise ValueError("max_step_rad must be positive.")
        if self.position_tolerance_m <= 0.0:
            raise ValueError("position_tolerance_m must be positive.")
        if self.orientation_tolerance_rad <= 0.0:
            raise ValueError("orientation_tolerance_rad must be positive.")


def solve_damped_least_squares(
    target: RigidTransform,
    seed_joint_position_rad: Sequence[float],
    joint_limits: JointLimits,
    compute_fk_matrix: Callable[[Sequence[float]], np.ndarray],
    compute_jacobian: Callable[[Sequence[float]], np.ndarray],
    settings: DampedLeastSquaresSettings,
    *,
    expected_parent_frame: str,
    expected_child_frame: str,
) -> IkResult:
    """Solve IK using a base-frame ``[angular; linear]`` Jacobian."""
    if (
        target.parent_frame != expected_parent_frame
        or target.child_frame != expected_child_frame
    ):
        raise KinematicsError(
            "IK target frame contract mismatch: expected "
            f"{expected_parent_frame}->{expected_child_frame}, observed "
            f"{target.parent_frame}->{target.child_frame}."
        )
    joint_position = np.asarray(tuple(seed_joint_position_rad), dtype=float)
    if joint_position.shape != (len(joint_limits.lower_rad),):
        raise ValueError("seed_joint_position_rad has the wrong number of joints.")
    if not np.all(np.isfinite(joint_position)):
        raise ValueError("seed_joint_position_rad must contain finite values.")

    lower_rad = np.asarray(joint_limits.lower_rad, dtype=float)
    upper_rad = np.asarray(joint_limits.upper_rad, dtype=float)
    joint_position = np.clip(joint_position, lower_rad, upper_rad)
    target_matrix = target.as_matrix()
    position_error_norm = math.inf
    orientation_error_norm = math.inf

    for iteration in range(settings.max_iterations + 1):
        joint_values = tuple(float(value) for value in joint_position)
        current_matrix = compute_fk_matrix(joint_values)
        position_error = target_matrix[:3, 3] - current_matrix[:3, 3]
        orientation_error = rotation_log_vector(
            target_matrix[:3, :3].dot(current_matrix[:3, :3].T)
        )
        position_error_norm = float(np.linalg.norm(position_error))
        orientation_error_norm = float(np.linalg.norm(orientation_error))
        if (
            position_error_norm <= settings.position_tolerance_m
            and orientation_error_norm <= settings.orientation_tolerance_rad
        ):
            return IkResult(
                succeeded=True,
                joint_position_rad=tuple(joint_position),
                iterations=iteration,
                position_error_m=position_error_norm,
                orientation_error_rad=orientation_error_norm,
                message="IK converged.",
            )

        error_vector = np.concatenate((orientation_error, position_error))
        jacobian = compute_jacobian(joint_values)
        regularized = jacobian.dot(jacobian.T) + settings.damping**2 * np.eye(
            jacobian.shape[0], dtype=float
        )
        try:
            joint_delta = jacobian.T.dot(np.linalg.solve(regularized, error_vector))
        except np.linalg.LinAlgError:
            joint_delta = np.linalg.pinv(jacobian, rcond=1.0e-5).dot(error_vector)
        delta_norm = float(np.linalg.norm(joint_delta))
        if delta_norm > settings.max_step_rad:
            joint_delta *= settings.max_step_rad / delta_norm
        joint_position = np.clip(joint_position + joint_delta, lower_rad, upper_rad)

    return IkResult(
        succeeded=False,
        joint_position_rad=tuple(joint_position),
        iterations=settings.max_iterations,
        position_error_m=position_error_norm,
        orientation_error_rad=orientation_error_norm,
        message="IK did not converge within the configured iteration limit.",
    )
