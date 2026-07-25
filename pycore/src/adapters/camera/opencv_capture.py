"""Optional OpenCV V4L capture adapter."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Optional

import numpy as np
from myarm_m750_core.domain.camera import CameraConfig, CameraFrame
from myarm_m750_core.domain.errors import CameraCaptureError, CameraTimeoutError
from myarm_m750_core.ports.camera import CameraCapturePort


class OpenCvCameraAdapter(CameraCapturePort):
    """Capture one stable by-id camera without importing ROS 2."""

    def __init__(self) -> None:
        self._cv2: Optional[Any] = None
        self._capture: Optional[Any] = None
        self._config: Optional[CameraConfig] = None
        self._sequence = 0

    @property
    def is_open(self) -> bool:
        """Return whether OpenCV reports an open device."""
        return bool(self._capture is not None and self._capture.isOpened())

    @staticmethod
    def _load_cv2() -> Any:
        try:
            import cv2  # type: ignore[import-not-found]
        except ImportError as error:
            raise CameraCaptureError(
                "OpenCV is optional; install myarm-m750-core[camera-host] "
                "or use the JetPack system package."
            ) from error
        return cv2

    def open(self, config: CameraConfig) -> None:
        """Open the configured by-id resource idempotently."""
        if self.is_open:
            if self._config == config:
                return
            raise CameraCaptureError("OpenCV adapter already owns another camera.")
        if not Path(config.device_by_id).exists():
            raise CameraCaptureError(
                f"Camera by-id resource does not exist: {config.device_by_id}"
            )
        cv2 = self._load_cv2()
        capture = cv2.VideoCapture(config.device_by_id)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, config.stream.width_px)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config.stream.height_px)
        capture.set(cv2.CAP_PROP_FPS, config.stream.fps_hz)
        if not capture.isOpened():
            capture.release()
            raise CameraCaptureError(f"Could not open camera device: {config.device_by_id}")
        self._cv2 = cv2
        self._capture = capture
        self._config = config

    def _convert_frame(self, image: Any, config: CameraConfig) -> np.ndarray:
        if not isinstance(image, np.ndarray):
            raise CameraCaptureError("OpenCV returned a non-NumPy camera frame.")
        if image.dtype != np.uint8:
            raise CameraCaptureError(
                f"OpenCV returned {image.dtype}; raw camera frames must use uint8."
            )
        if image.ndim < 2 or image.shape[:2] != (
            config.stream.height_px,
            config.stream.width_px,
        ):
            raise CameraCaptureError(
                "OpenCV frame dimensions do not match the configured stream: "
                f"expected {config.stream.height_px}x{config.stream.width_px}, "
                f"observed {image.shape}."
            )

        pixel_format = config.stream.pixel_format
        if pixel_format == "mono8":
            if image.ndim == 2:
                return np.ascontiguousarray(image)
            if image.ndim == 3 and image.shape[2] == 3:
                cv2 = self._cv2
                if cv2 is None:
                    raise CameraCaptureError("OpenCV conversion backend is unavailable.")
                converted = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                return np.ascontiguousarray(converted)
            raise CameraCaptureError(
                f"mono8 requires an HxW or HxWx3 source; observed {image.shape}."
            )

        if image.ndim != 3 or image.shape[2] != 3:
            raise CameraCaptureError(
                f"{pixel_format} requires an HxWx3 source; observed {image.shape}."
            )
        if pixel_format == "rgb8":
            cv2 = self._cv2
            if cv2 is None:
                raise CameraCaptureError("OpenCV conversion backend is unavailable.")
            converted = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            return np.ascontiguousarray(converted)
        if pixel_format == "bgr8":
            return np.ascontiguousarray(image)
        raise CameraCaptureError(f"Unsupported configured pixel format: {pixel_format}")

    def read_frame(self, timeout_s: float) -> CameraFrame:
        """Read until a bounded deadline or raise CameraTimeoutError."""
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive.")
        capture = self._capture
        config = self._config
        if not self.is_open or capture is None or config is None:
            raise CameraCaptureError("OpenCV camera is not open.")
        deadline_s = time.monotonic() + timeout_s
        while time.monotonic() < deadline_s:
            succeeded, image = capture.read()
            if succeeded and image is not None:
                converted = self._convert_frame(image, config)
                next_sequence = self._sequence + 1
                frame = CameraFrame(
                    camera_name=config.hardware_name,
                    sequence=next_sequence,
                    acquisition_monotonic_s=time.monotonic(),
                    observation_wall_time_s=time.time(),
                    image=converted,
                    encoding=config.stream.pixel_format,
                    metadata={
                        "backend": "opencv",
                        "serial": config.hardware_serial,
                    },
                )
                self._sequence = next_sequence
                return frame
            time.sleep(min(0.005, timeout_s))
        raise CameraTimeoutError(f"Camera read exceeded {timeout_s:.3f}s.")

    def close(self) -> None:
        """Release OpenCV resources idempotently."""
        if self._capture is not None:
            self._capture.release()
        self._cv2 = None
        self._capture = None
        self._config = None
