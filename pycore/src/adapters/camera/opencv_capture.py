"""Optional OpenCV camera capture adapter."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Optional

from myarm_m750_core.domain.camera import (
    CameraConfig,
    CameraFrame,
    CameraReadResult,
)


class OpenCvCameraAdapter:
    """Capture one V4L camera without importing ROS 2.

    OpenCV is imported lazily so control-only deployments do not need camera
    dependencies. The adapter prefers ``/dev/v4l/by-id`` and uses the fallback
    device only when the stable path is unavailable.
    """

    def __init__(self) -> None:
        self._cv2: Optional[Any] = None
        self._capture: Optional[Any] = None
        self._config: Optional[CameraConfig] = None
        self._sequence = 0

    @property
    def is_open(self) -> bool:
        return bool(self._capture is not None and self._capture.isOpened())

    @staticmethod
    def _load_cv2() -> Any:
        try:
            import cv2  # type: ignore[import-not-found]
        except ImportError as error:
            raise RuntimeError(
                "OpenCV camera backend is optional. Install requirements/camera.txt."
            ) from error
        return cv2

    @staticmethod
    def _select_device(config: CameraConfig) -> str:
        by_id = Path(config.device_by_id)
        if by_id.exists():
            return str(by_id)
        if config.fallback_path:
            return config.fallback_path
        raise FileNotFoundError(
            "Camera device does not exist and no fallback is configured: {0}".format(
                config.device_by_id
            )
        )

    def open(self, config: CameraConfig) -> None:
        if self.is_open:
            raise RuntimeError("Camera is already open.")
        cv2 = self._load_cv2()
        device = self._select_device(config)
        capture = cv2.VideoCapture(device)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, config.stream.width_px)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config.stream.height_px)
        capture.set(cv2.CAP_PROP_FPS, config.stream.fps_hz)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError("Could not open camera device: {0}".format(device))
        self._cv2 = cv2
        self._capture = capture
        self._config = config
        self._sequence = 0

    def read(self, timeout_s: float) -> CameraReadResult:
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive.")
        if not self.is_open or self._capture is None or self._config is None:
            return CameraReadResult(frame=None, timed_out=False, error="camera_not_open")

        deadline_s = time.monotonic() + timeout_s
        while time.monotonic() < deadline_s:
            succeeded, image = self._capture.read()
            if succeeded:
                self._sequence += 1
                frame = CameraFrame(
                    camera_name=self._config.hardware_name,
                    sequence=self._sequence,
                    timestamp_monotonic_s=time.monotonic(),
                    image=image,
                    metadata={"backend": "opencv"},
                )
                return CameraReadResult(frame=frame, timed_out=False)
            time.sleep(min(0.005, timeout_s))
        return CameraReadResult(frame=None, timed_out=True, error="camera_read_timeout")

    def close(self) -> None:
        if self._capture is not None:
            self._capture.release()
        self._capture = None
        self._config = None
