from myarm_m750_core import CameraSession
from myarm_m750_core.adapters.camera import MockCameraAdapter
from myarm_m750_core.runtime.config import load_camera_configs


def test_camera_config_keeps_hardware_name_separate_from_role(repository_root) -> None:
    configs = load_camera_configs(
        str(repository_root / "pycore/config/camera/cameras.yaml")
    )
    first = configs[0]
    assert first.hardware_name == "logitech_c922_01"
    assert first.role == "wrist"
    assert first.hardware_serial == "SERIAL_CAMERA_01"


def test_camera_session_runs_without_ros2(repository_root) -> None:
    received = []
    with CameraSession.from_config(
        config_path=str(repository_root / "pycore/config/camera/cameras.yaml"),
        hardware_name="logitech_c922_01",
        capture=MockCameraAdapter(),
    ) as camera:
        count = camera.run(received.append, max_frames=2)

    assert count == 2
    assert [frame.sequence for frame in received] == [1, 2]
    assert received[0].image.shape == (480, 640, 3)
