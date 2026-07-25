"""Kinematics port independent of ROS 2 and hardware."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Sequence, Tuple

import numpy as np
from myarm_m750_core.domain.models import IkResult, JointLimits, RigidTransform


@dataclass(frozen=True)
class KinematicsInfo:
    """Traceable model and convention metadata for a kinematics provider."""

    provider_name: str
    provider_version: str
    model_fingerprint_sha256: str
    base_link: str
    end_link: str
    joint_names: Tuple[str, ...]
    jacobian_order: str = "angular_linear"
    jacobian_reference_frame: str = "base"
    jacobian_reference_point: str = "end_link_origin"
    dynamics_available: bool = False

    def __post_init__(self) -> None:
        if not self.provider_name:
            raise ValueError("provider_name must be non-empty.")
        if len(self.model_fingerprint_sha256) != 64:
            raise ValueError("model_fingerprint_sha256 must be a SHA-256 hex digest.")
        if not self.base_link or not self.end_link:
            raise ValueError("base_link and end_link must be non-empty.")
        if not self.joint_names or len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("joint_names must be non-empty and unique.")
        if self.jacobian_order != "angular_linear":
            raise ValueError("Only the [angular, linear] Jacobian order is supported.")
        if self.jacobian_reference_frame != "base":
            raise ValueError("Only a base-frame Jacobian is supported.")
        if self.jacobian_reference_point != "end_link_origin":
            raise ValueError("Only the end-link-origin Jacobian is supported.")


class KinematicsPort(ABC):
    """Pure software FK, IK, Jacobian, and singularity API."""

    @property
    @abstractmethod
    def info(self) -> KinematicsInfo:
        """Return provider, model fingerprint, frame, and convention metadata."""

    @property
    @abstractmethod
    def joint_limits(self) -> JointLimits:
        """Return limits loaded from the robot model."""

    @abstractmethod
    def compute_fk(self, joint_position_rad: Sequence[float]) -> RigidTransform:
        """Compute the configured end-link pose."""

    @abstractmethod
    def compute_jacobian(self, joint_position_rad: Sequence[float]) -> np.ndarray:
        """Return ``[angular; linear]`` at the end-link origin in the base frame."""

    @abstractmethod
    def compute_singularity_score(self, joint_position_rad: Sequence[float]) -> float:
        """Return the smallest singular value of the geometric Jacobian."""

    @abstractmethod
    def solve_ik(
        self,
        target: RigidTransform,
        seed_joint_position_rad: Sequence[float],
    ) -> IkResult:
        """Solve IK for this provider's exact base/end frame pair."""
