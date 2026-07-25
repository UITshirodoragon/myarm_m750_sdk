#!/usr/bin/env python3
"""Read deterministic latest frames without ROS 2 or camera hardware."""

from myarm_m750_core import CameraSessionBuilder


def main() -> None:
    session = CameraSessionBuilder.from_file(
        "pycore/config/camera/cameras_mock.yaml"
    ).build()
    with session:
        for camera_name in session.camera_names:
            previous_sequence = 0
            for _ in range(3):
                frame = session.latest_frame(
                    camera_name,
                    timeout_s=1.0,
                    after_sequence=previous_sequence,
                )
                previous_sequence = frame.sequence
                print(frame.camera_name, frame.sequence, frame.image.shape)


if __name__ == "__main__":
    main()
