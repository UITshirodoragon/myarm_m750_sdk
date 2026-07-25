"""Deterministic, fault-injectable mock camera."""

from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np
from myarm_m750_core.domain.camera import CameraConfig, CameraFrame
from myarm_m750_core.domain.errors import CameraCaptureError, CameraTimeoutError
from myarm_m750_core.ports.camera import CameraCapturePort


class MockCameraAdapter(CameraCapturePort):
    """Generate configured raw frames and deterministic read faults."""

    def __init__(
        self,
        timeouts_before_success: int = 0,
        capture_errors_before_success: int = 0,
    ) -> None:
        self._config: Optional[CameraConfig] = None
        self._sequence = 0
        self._timeouts_remaining = int(timeouts_before_success)
        self._errors_remaining = int(capture_errors_before_success)
        self._lock = threading.RLock()

    @property
    def is_open(self) -> bool:
        """Return whether the synthetic device is open."""
        with self._lock:
            return self._config is not None

    def open(self, config: CameraConfig) -> None:
        """Open idempotently for worker reconnect paths."""
        with self._lock:
            if self._config is not None and self._config != config:
                raise CameraCaptureError("Mock adapter already owns another camera.")
            self._config = config

    def read_frame(self, timeout_s: float) -> CameraFrame:
        """Return one frame or a configured typed fault."""
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive.")
        with self._lock:
            config = self._config
            if config is None:
                raise CameraCaptureError("Mock camera is not open.")
            if self._timeouts_remaining > 0:
                self._timeouts_remaining -= 1
                raise CameraTimeoutError("Injected mock camera timeout.")
            if self._errors_remaining > 0:
                self._errors_remaining -= 1
                raise CameraCaptureError("Injected mock camera capture failure.")
            self._sequence += 1
            if config.stream.pixel_format == "mono8":
                image = np.full(
                    (config.stream.height_px, config.stream.width_px),
                    self._sequence % 255,
                    dtype=np.uint8,
                )
            else:
                image = np.zeros(
                    (config.stream.height_px, config.stream.width_px, 3),
                    dtype=np.uint8,
                )
                image[:, :, 0] = self._sequence % 255
            return CameraFrame(
                camera_name=config.hardware_name,
                sequence=self._sequence,
                acquisition_monotonic_s=time.monotonic(),
                observation_wall_time_s=time.time(),
                image=image,
                encoding=config.stream.pixel_format,
                metadata={"backend": "mock", "serial": config.hardware_serial},
            )

    def close(self) -> None:
        """Close idempotently."""
        with self._lock:
            self._config = None
