"""Camera core-to-ROS conversion contract tests."""

import array
import unittest
from pathlib import Path
from types import SimpleNamespace
from typing import cast

import numpy as np
from builtin_interfaces.msg import Time
from myarm_m750_camera.conversion import (
    camera_info_message,
    image_message,
    static_transforms,
    wall_time_message,
)
from myarm_m750_core.domain.camera import (
    CameraCalibration,
    CameraConfig,
    CameraExtrinsics,
    CameraFrame,
    CameraReconnectPolicy,
    CameraStreamConfig,
)


def _camera_config() -> CameraConfig:
    calibration = CameraCalibration(
        image_width_px=2,
        image_height_px=2,
        camera_matrix=(1.0, 0.0, 0.5, 0.0, 1.0, 0.5, 0.0, 0.0, 1.0),
        distortion_model="plumb_bob",
        distortion_coefficients=(0.0, 0.0, 0.0, 0.0, 0.0),
        rectification_matrix=(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        projection_matrix=(
            1.0,
            0.0,
            0.5,
            0.0,
            0.0,
            1.0,
            0.5,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ),
        source_path=Path("/tmp/mock_calibration.yaml"),
    )
    return CameraConfig(
        hardware_name="mock_wrist_01",
        enabled=True,
        backend="mock",
        hardware_model="deterministic_numpy_camera",
        hardware_serial="MOCK-WRIST-001",
        role="wrist",
        device_by_id="",
        stream=CameraStreamConfig(
            width_px=2,
            height_px=2,
            fps_hz=15.0,
            pixel_format="bgr8",
        ),
        camera_frame="mock_wrist_01_link",
        optical_frame="mock_wrist_01_optical_frame",
        calibration=calibration,
        extrinsics=CameraExtrinsics(
            parent_frame="tool0",
            child_frame="mock_wrist_01_link",
            translation_m=(0.0, 0.0, 0.05),
            quaternion_xyzw=(0.0, 0.0, 0.0, 2.0),
        ),
        reconnect=CameraReconnectPolicy(
            read_timeout_s=0.1,
            initial_backoff_s=0.01,
            maximum_backoff_s=0.1,
            multiplier=2.0,
            maximum_attempts=5,
        ),
    )


class CameraConversionTest(unittest.TestCase):
    """Verify ndarray bytes, timestamps, calibration, and frame transforms."""

    def test_direct_non_contiguous_ndarray_to_image(self) -> None:
        source = np.arange(24, dtype=np.uint8).reshape(2, 4, 3)
        non_contiguous = source[:, ::2, :]
        frame = CameraFrame(
            camera_name="mock_wrist_01",
            sequence=7,
            acquisition_monotonic_s=3.0,
            observation_wall_time_s=12.25,
            image=non_contiguous,
            encoding="bgr8",
            metadata={},
        )

        message = image_message(frame, "mock_wrist_01_optical_frame")

        self.assertEqual((message.height, message.width), (2, 2))
        self.assertEqual(message.encoding, "bgr8")
        self.assertEqual(message.step, 6)
        self.assertIsInstance(message.data, array.array)
        self.assertEqual(message.data.typecode, "B")
        self.assertEqual(message.header.stamp, Time(sec=12, nanosec=250_000_000))
        self.assertEqual(
            bytes(message.data),
            non_contiguous.copy(order="C").tobytes(),
        )

    def test_wall_timestamp_is_normalized(self) -> None:
        stamp = wall_time_message(1.9999999996)

        self.assertEqual(stamp.sec, 2)
        self.assertEqual(stamp.nanosec, 0)

    def test_rejects_dtype_and_channel_mismatch(self) -> None:
        cases = (
            (np.zeros((2, 2, 3), dtype=np.uint16), "bgr8", "uint8"),
            (np.zeros((2, 2), dtype=np.uint8), "bgr8", "HxWx3"),
            (np.zeros((2, 2, 3), dtype=np.uint8), "mono8", "two-dimensional"),
            (np.zeros((2, 2, 4), dtype=np.uint8), "rgba8", "Unsupported"),
        )
        for image, encoding, error_pattern in cases:
            with self.subTest(encoding=encoding, shape=image.shape):
                # Exercise the ROS boundary independently of the core DTO's
                # matching fail-fast invariant.
                frame = cast(
                    CameraFrame,
                    SimpleNamespace(
                        observation_wall_time_s=12.25,
                        image=image,
                        encoding=encoding,
                    ),
                )
                with self.assertRaisesRegex(ValueError, error_pattern):
                    image_message(frame, "mock_wrist_01_optical_frame")

    def test_camera_info_and_static_tf_follow_frame_contract(self) -> None:
        config = _camera_config()
        stamp = Time(sec=23, nanosec=45)

        info = camera_info_message(config, stamp)
        transforms = static_transforms(config)

        self.assertEqual(info.header.stamp, stamp)
        self.assertEqual(info.header.frame_id, config.optical_frame)
        self.assertEqual((info.width, info.height), (2, 2))
        self.assertEqual(tuple(info.k), config.calibration.camera_matrix)
        self.assertEqual(len(transforms), 2)
        self.assertEqual(transforms[0].header.frame_id, "tool0")
        self.assertEqual(transforms[0].child_frame_id, config.camera_frame)
        self.assertEqual(transforms[0].transform.rotation.w, 1.0)
        self.assertEqual(transforms[1].header.frame_id, config.camera_frame)
        self.assertEqual(transforms[1].child_frame_id, config.optical_frame)


if __name__ == "__main__":
    unittest.main()
