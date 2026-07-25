import numpy as np
import pytest
from myarm_m750_core.domain.models import MotionProfile
from myarm_m750_core.runtime import PointToPointTrajectoryGenerator


def test_cubic_trajectory_contains_exact_endpoints(sdk_config) -> None:
    generator = PointToPointTrajectoryGenerator(command_rate_hz=5.0)
    start = [0.0] * 6
    target = [0.25, -0.2, 0.15, 0.1, -0.1, 0.2]
    trajectory = generator.generate(
        sdk_config.robot.joint_names,
        start,
        target,
        motion_profile=MotionProfile(duration_s=2.0),
    )
    assert len(trajectory.points) == 11
    np.testing.assert_allclose(trajectory.points[0].position_rad, start)
    np.testing.assert_allclose(trajectory.points[-1].position_rad, target)
    assert trajectory.duration_s == 2.0
    np.testing.assert_allclose(trajectory.points[0].velocity_rad_s, 0.0)
    np.testing.assert_allclose(trajectory.points[-1].velocity_rad_s, 0.0)


def test_cubic_trajectory_enforces_analytic_motion_profile_extrema(
    sdk_config,
) -> None:
    generator = PointToPointTrajectoryGenerator(command_rate_hz=5.0)
    start = [0.0] * 6
    target = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0]

    with pytest.raises(ValueError, match="velocity extremum"):
        generator.generate(
            sdk_config.robot.joint_names,
            start,
            target,
            MotionProfile(duration_s=2.0, max_velocity_rad_s=0.149),
        )
    with pytest.raises(ValueError, match="acceleration extremum"):
        generator.generate(
            sdk_config.robot.joint_names,
            start,
            target,
            MotionProfile(duration_s=2.0, max_acceleration_rad_s2=0.299),
        )

    trajectory = generator.generate(
        sdk_config.robot.joint_names,
        start,
        target,
        MotionProfile(
            duration_s=2.0,
            max_velocity_rad_s=0.15,
            max_acceleration_rad_s2=0.30,
        ),
    )
    peak_velocity = max(abs(value) for value in trajectory.points[5].velocity_rad_s)
    peak_acceleration = max(
        abs(value) for value in trajectory.points[0].acceleration_rad_s2
    )
    assert peak_velocity == pytest.approx(0.15)
    assert peak_acceleration == pytest.approx(0.30)
