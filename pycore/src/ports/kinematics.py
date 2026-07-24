"""Kinematics port independent of ROS 2 and hardware."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Sequence

import numpy as np

from myarm_m750_core.domain.models import IkResult, JointLimits, RigidTransform


class KinematicsPort(ABC):
    """Pure software FK, IK, Jacobian, and singularity API."""

    @property
    @abstractmethod
    def joint_limits(self) -> JointLimits:
        """Return limits loaded from the robot model."""

    @abstractmethod
    def compute_fk(self, joint_position_rad: Sequence[float]) -> RigidTransform:
        """Compute the configured end-link pose."""

    @abstractmethod
    def compute_jacobian(self, joint_position_rad: Sequence[float]) -> np.ndarray:
        """Compute a 6x6 geometric Jacobian in the base frame."""

    @abstractmethod
    def compute_singularity_score(self, joint_position_rad: Sequence[float]) -> float:
        """Return the smallest singular value of the geometric Jacobian."""

    @abstractmethod
    def solve_ik(
        self,
        target: RigidTransform,
        seed_joint_position_rad: Sequence[float],
    ) -> IkResult:
        """Solve numerical IK without causing side effects."""
