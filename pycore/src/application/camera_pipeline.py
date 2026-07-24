"""Small standalone camera pipeline composed around a capture port."""

from __future__ import annotations

from typing import Callable, Optional

from myarm_m750_core.domain.camera import CameraConfig, CameraFrame
from myarm_m750_core.ports.camera import CameraCapturePort

FrameHandler = Callable[[CameraFrame], None]


class CameraPipeline:
    """Own camera lifecycle and forward frames to one explicit handler."""

    def __init__(self, config: CameraConfig, capture: CameraCapturePort) -> None:
        self._config = config
        self._capture = capture

    @property
    def is_open(self) -> bool:
        return self._capture.is_open

    def open(self) -> None:
        if not self._config.enabled:
            raise RuntimeError(
                "Camera is disabled in YAML: {0}".format(self._config.hardware_name)
            )
        self._capture.open(self._config)

    def read_one(self, timeout_s: float = 1.0) -> CameraFrame:
        result = self._capture.read(timeout_s)
        if not result.succeeded or result.frame is None:
            raise RuntimeError(result.error or "camera_read_failed")
        return result.frame

    def run(
        self,
        frame_handler: FrameHandler,
        max_frames: Optional[int] = None,
        timeout_s: float = 1.0,
    ) -> int:
        """Process frames synchronously until ``max_frames`` is reached.

        Side effects:
            Reads the configured camera and invokes ``frame_handler``.
        """
        if max_frames is not None and max_frames <= 0:
            raise ValueError("max_frames must be positive when provided.")
        processed = 0
        while max_frames is None or processed < max_frames:
            frame_handler(self.read_one(timeout_s=timeout_s))
            processed += 1
        return processed

    def close(self) -> None:
        self._capture.close()

    def __enter__(self) -> "CameraPipeline":
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:  # type: ignore[no-untyped-def]
        self.close()
