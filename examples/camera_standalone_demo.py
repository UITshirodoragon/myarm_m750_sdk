#!/usr/bin/env python3
"""Read deterministic frames without ROS 2 or physical camera hardware."""

from myarm_m750_core import CameraSession
from myarm_m750_core.adapters.camera import MockCameraAdapter


def main() -> None:
    with CameraSession.from_config(
        config_path="pycore/config/camera/cameras.yaml",
        hardware_name="logitech_c922_01",
        capture=MockCameraAdapter(),
    ) as camera:
        camera.run(
            frame_handler=lambda frame: print(
                frame.camera_name, frame.sequence, frame.image.shape
            ),
            max_frames=3,
        )


if __name__ == "__main__":
    main()
