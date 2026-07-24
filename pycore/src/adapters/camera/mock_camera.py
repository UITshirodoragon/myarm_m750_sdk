"""Deterministic camera adapter for unit tests and non-hardware demos."""

from __future__ import annotations

import time
from typing import Optional

import numpy as np

from myarm_m750_core.domain.camera import (
    CameraConfig,
    CameraFrame,
    CameraReadResult,
)


class MockCameraAdapter:
    """Generate deterministic zero-valued frames from a camera config."""

    def __init__(self) -> None:
        self._config: Optional[CameraConfig] = None
        self._sequence = 0

    @property
    def is_open(self) -> bool:
        return self._config is not None

    def open(self, config: CameraConfig) -> None:
        if self.is_open:
            raise RuntimeError("Mock camera is already open.")
        self._config = config
        self._sequence = 0

    def read(self, timeout_s: float) -> CameraReadResult:
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive.")
        if self._config is None:
            return CameraReadResult(frame=None, timed_out=False, error="camera_not_open")
        self._sequence += 1
        image = np.zeros(
            (self._config.stream.height_px, self._config.stream.width_px, 3),
            dtype=np.uint8,
        )
        frame = CameraFrame(
            camera_name=self._config.hardware_name,
            sequence=self._sequence,
            timestamp_monotonic_s=time.monotonic(),
            image=image,
            metadata={"backend": "mock"},
        )
        return CameraReadResult(frame=frame, timed_out=False)

    def close(self) -> None:
        self._config = None
