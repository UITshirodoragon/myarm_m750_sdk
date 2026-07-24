"""Deterministic point-to-point joint trajectory generation."""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np

from myarm_m750_core.domain.models import JointTrajectory, JointTrajectoryPoint


class PointToPointTrajectoryGenerator:
    """Generate cubic time-scaled joint trajectories at a bounded rate."""

    def __init__(self, command_rate_hz: float) -> None:
        if command_rate_hz <= 0.0:
            raise ValueError("command_rate_hz must be positive.")
        self._command_rate_hz = float(command_rate_hz)

    def generate(
        self,
        joint_names: Sequence[str],
        start_position_rad: Sequence[float],
        target_position_rad: Sequence[float],
        duration_s: float,
    ) -> JointTrajectory:
        """Generate a cubic trajectory with zero endpoint velocities."""
        if duration_s <= 0.0:
            raise ValueError("duration_s must be positive.")
        start = np.asarray(tuple(start_position_rad), dtype=float)
        target = np.asarray(tuple(target_position_rad), dtype=float)
        if start.shape != (6,) or target.shape != (6,):
            raise ValueError("Start and target must each contain six values.")
        interval_count = max(1, int(math.ceil(duration_s * self._command_rate_hz)))
        points = []
        for interval_index in range(interval_count + 1):
            normalized_time = interval_index / float(interval_count)
            cubic_scale = 3.0 * normalized_time ** 2 - 2.0 * normalized_time ** 3
            position_rad = start + cubic_scale * (target - start)
            point_time_s = duration_s * normalized_time
            points.append(
                JointTrajectoryPoint(
                    position_rad=tuple(position_rad),
                    time_from_start_s=point_time_s,
                )
            )
        return JointTrajectory(
            joint_names=tuple(joint_names),
            points=tuple(points),
        )
