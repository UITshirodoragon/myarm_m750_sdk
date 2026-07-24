import numpy as np

from myarm_m750_core import RobotSession
from myarm_m750_core.runtime import DriverState


def test_public_session_runs_without_ros_or_hardware(repository_root) -> None:
    config_path = repository_root / "pycore" / "config" / "default.yaml"
    with RobotSession.from_config(str(config_path)) as robot:
        assert robot.state is DriverState.IDLE
        result = robot.move_joints(
            target=[0.2, -0.2, 0.15, 0.1, -0.1, 0.15], duration_s=3.0
        )
        assert result.succeeded, result.message
        np.testing.assert_allclose(
            robot.get_state().position_rad,
            [0.2, -0.2, 0.15, 0.1, -0.1, 0.15],
            atol=1.0e-12,
        )
    assert robot.state is DriverState.DISCONNECTED


def test_public_session_exposes_status_capabilities_and_cancel(repository_root) -> None:
    config_path = repository_root / "pycore" / "config" / "default.yaml"
    with RobotSession.from_config(str(config_path)) as robot:
        status = robot.get_hardware_status()
        capabilities = robot.get_capabilities()
        assert status.connected
        assert capabilities.supports_stop
        result = robot.move_joints(
            target=[0.05, 0.0, 0.0, 0.0, 0.0, 0.0],
            duration_s=1.0,
            cancel_requested=lambda: True,
        )
        assert result.status.value == "canceled"
        assert robot.state is DriverState.IDLE
