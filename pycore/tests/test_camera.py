import time

import pytest
from myarm_m750_core import CameraSessionBuilder, RobotSessionBuilder
from myarm_m750_core.adapters.camera import MockCameraAdapter
from myarm_m750_core.domain.camera import CameraState
from myarm_m750_core.domain.errors import ConfigurationError
from myarm_m750_core.runtime import VirtualScheduler
from myarm_m750_core.runtime.config import load_camera_configs


def _mock_config(repository_root):
    return repository_root / "pycore/config/camera/cameras_mock.yaml"


def test_camera_config_resolves_calibration_and_identity(repository_root) -> None:
    configs = load_camera_configs(str(_mock_config(repository_root)))
    first = configs[0]
    assert first.hardware_name == "mock_wrist_01"
    assert first.role == "wrist"
    assert first.hardware_serial == "MOCK-WRIST-001"
    assert first.calibration.camera_matrix[0] == 500.0
    assert first.calibration.source_path.is_file()


def test_two_mock_camera_workers_and_bounded_shutdown(repository_root) -> None:
    session = CameraSessionBuilder.from_file(str(_mock_config(repository_root))).build()
    with session:
        frames = [
            session.latest_frame(camera_name, timeout_s=1.0)
            for camera_name in session.camera_names
        ]
        assert all(frame.image.shape == (480, 640, 3) for frame in frames)
        assert all(
            session.state(camera_name) is CameraState.STREAMING
            for camera_name in session.camera_names
        )
    assert all(
        session.state(camera_name) is CameraState.CLOSED
        for camera_name in session.camera_names
    )


def test_timeout_reconnect_and_latest_queue_overflow(repository_root) -> None:
    builder = CameraSessionBuilder.from_file(str(_mock_config(repository_root)))
    builder.with_capture_factory(
        lambda _config: MockCameraAdapter(timeouts_before_success=1)
    )
    with builder.build() as session:
        frame = session.latest_frame("mock_wrist_01", timeout_s=1.0)
        assert frame.sequence >= 1
        time.sleep(0.16)
        metrics = session.metrics_snapshot("mock_wrist_01")
        assert metrics.read_timeouts == 1
        assert metrics.reconnect_count >= 1
        assert metrics.queue_overflow_count >= 1


def test_one_camera_fault_does_not_change_robot_state(
    repository_root,
) -> None:
    camera_builder = CameraSessionBuilder.from_file(str(_mock_config(repository_root)))
    camera_builder.with_capture_factory(
        lambda config: (
            MockCameraAdapter(capture_errors_before_success=100)
            if config.hardware_name == "mock_wrist_01"
            else MockCameraAdapter()
        )
    )
    robot = (
        RobotSessionBuilder.from_file(str(repository_root / "pycore/config/default.yaml"))
        .with_scheduler(VirtualScheduler())
        .build()
    )
    cameras = camera_builder.build()
    with robot, cameras:
        good_frame = cameras.latest_frame("mock_shoulder_02", timeout_s=1.0)
        assert good_frame.sequence >= 1
        deadline = time.monotonic() + 2.0
        while (
            cameras.state("mock_wrist_01") is not CameraState.FAULT
            and time.monotonic() < deadline
        ):
            time.sleep(0.02)
        assert cameras.state("mock_wrist_01") is CameraState.FAULT
        assert robot.read_hardware_status().connected


def test_empty_real_example_cannot_build(repository_root) -> None:
    builder = CameraSessionBuilder.from_file(
        str(repository_root / "pycore/config/camera/cameras_real.example.yaml")
    )
    with pytest.raises(ConfigurationError, match="no enabled"):
        builder.build()
