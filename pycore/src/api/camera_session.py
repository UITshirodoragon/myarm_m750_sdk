"""Public ROS-independent camera API."""

from __future__ import annotations

from typing import Optional

from myarm_m750_core.adapters.camera import OpenCvCameraAdapter
from myarm_m750_core.domain.camera import CameraFrame
from myarm_m750_core.application.camera_pipeline import CameraPipeline, FrameHandler
from myarm_m750_core.ports.camera import CameraCapturePort
from myarm_m750_core.runtime.config.camera_loader import camera_config_by_name


class CameraSession:
    """Create a standalone camera pipeline from YAML configuration."""

    def __init__(self, pipeline: CameraPipeline) -> None:
        self._pipeline = pipeline

    @classmethod
    def from_config(
        cls,
        config_path: str,
        hardware_name: str,
        capture: Optional[CameraCapturePort] = None,
    ) -> "CameraSession":
        config = camera_config_by_name(config_path, hardware_name)
        selected_capture = capture if capture is not None else OpenCvCameraAdapter()
        return cls(CameraPipeline(config=config, capture=selected_capture))

    @property
    def is_open(self) -> bool:
        return self._pipeline.is_open

    def open(self) -> None:
        self._pipeline.open()

    def read_one(self, timeout_s: float = 1.0) -> CameraFrame:
        return self._pipeline.read_one(timeout_s=timeout_s)

    def run(
        self,
        frame_handler: FrameHandler,
        max_frames: Optional[int] = None,
        timeout_s: float = 1.0,
    ) -> int:
        return self._pipeline.run(
            frame_handler=frame_handler,
            max_frames=max_frames,
            timeout_s=timeout_s,
        )

    def close(self) -> None:
        self._pipeline.close()

    def __enter__(self) -> "CameraSession":
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:  # type: ignore[no-untyped-def]
        self.close()
