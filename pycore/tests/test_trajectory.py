import numpy as np

from myarm_m750_core.runtime import PointToPointTrajectoryGenerator


def test_cubic_trajectory_contains_exact_endpoints(sdk_config) -> None:
    generator = PointToPointTrajectoryGenerator(command_rate_hz=5.0)
    start = [0.0] * 6
    target = [0.25, -0.2, 0.15, 0.1, -0.1, 0.2]
    trajectory = generator.generate(
        sdk_config.robot.joint_names, start, target, duration_s=2.0
    )
    assert len(trajectory.points) == 11
    np.testing.assert_allclose(trajectory.points[0].position_rad, start)
    np.testing.assert_allclose(trajectory.points[-1].position_rad, target)
    assert trajectory.duration_s == 2.0
