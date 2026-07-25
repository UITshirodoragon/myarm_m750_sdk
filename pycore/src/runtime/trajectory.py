"""Deterministic point-to-point joint trajectory generation."""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np
from myarm_m750_core.domain.models import (
    JointTrajectory,
    JointTrajectoryPoint,
    MotionProfile,
)


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
        motion_profile: MotionProfile,
    ) -> JointTrajectory:
        """Generate a cubic trajectory with zero endpoint velocities.

        Raises:
            ValueError: If an optional profile limit is below the analytic
                cubic velocity or acceleration extremum.
        """
        duration_s = motion_profile.duration_s
        start = np.asarray(tuple(start_position_rad), dtype=float)
        target = np.asarray(tuple(target_position_rad), dtype=float)
        if start.shape != (6,) or target.shape != (6,):
            raise ValueError("Start and target must each contain six values.")
        maximum_displacement_rad = float(np.max(np.abs(target - start)))
        cubic_maximum_velocity_rad_s = 1.5 * maximum_displacement_rad / duration_s
        cubic_maximum_acceleration_rad_s2 = 6.0 * maximum_displacement_rad / duration_s**2
        if (
            motion_profile.max_velocity_rad_s is not None
            and cubic_maximum_velocity_rad_s > motion_profile.max_velocity_rad_s + 1.0e-12
        ):
            raise ValueError(
                "Cubic trajectory velocity extremum "
                f"{cubic_maximum_velocity_rad_s:.6f} rad/s exceeds "
                f"MotionProfile limit {motion_profile.max_velocity_rad_s:.6f} rad/s."
            )
        if (
            motion_profile.max_acceleration_rad_s2 is not None
            and cubic_maximum_acceleration_rad_s2
            > motion_profile.max_acceleration_rad_s2 + 1.0e-12
        ):
            raise ValueError(
                "Cubic trajectory acceleration extremum "
                f"{cubic_maximum_acceleration_rad_s2:.6f} rad/s^2 exceeds "
                "MotionProfile limit "
                f"{motion_profile.max_acceleration_rad_s2:.6f} rad/s^2."
            )
        interval_count = max(1, int(math.ceil(duration_s * self._command_rate_hz)))
        points = []
        for interval_index in range(interval_count + 1):
            normalized_time = interval_index / float(interval_count)
            cubic_scale = 3.0 * normalized_time**2 - 2.0 * normalized_time**3
            velocity_scale = (6.0 * normalized_time - 6.0 * normalized_time**2) / duration_s
            acceleration_scale = (6.0 - 12.0 * normalized_time) / duration_s**2
            position_rad = start + cubic_scale * (target - start)
            velocity_rad_s = velocity_scale * (target - start)
            acceleration_rad_s2 = acceleration_scale * (target - start)
            point_time_s = duration_s * normalized_time
            points.append(
                JointTrajectoryPoint(
                    position_rad=tuple(position_rad),
                    time_from_start_s=point_time_s,
                    velocity_rad_s=tuple(velocity_rad_s),
                    acceleration_rad_s2=tuple(acceleration_rad_s2),
                )
            )
        return JointTrajectory(
            joint_names=tuple(joint_names),
            points=tuple(points),
        )
