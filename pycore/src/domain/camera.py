"""ROS-independent camera value objects."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Optional


@dataclass(frozen=True)
class CameraStreamConfig:
    """Requested camera stream format."""

    width_px: int
    height_px: int
    fps_hz: float
    pixel_format: str

    def __post_init__(self) -> None:
        if self.width_px <= 0 or self.height_px <= 0:
            raise ValueError("Camera width and height must be positive.")
        if self.fps_hz <= 0.0:
            raise ValueError("Camera FPS must be positive.")


@dataclass(frozen=True)
class CameraConfig:
    """One physical camera configuration independent from its deployment role."""

    hardware_name: str
    enabled: bool
    hardware_model: str
    hardware_serial: str
    role: str
    device_by_id: str
    fallback_path: str
    stream: CameraStreamConfig
    camera_frame: str
    optical_frame: str
    calibration_file: str


@dataclass(frozen=True)
class CameraFrame:
    """One captured frame and acquisition metadata.

    The image payload is intentionally typed as ``Any`` so the core does not
    force NumPy/OpenCV on every camera backend. OpenCV adapters return an ndarray.
    """

    camera_name: str
    sequence: int
    timestamp_monotonic_s: float
    image: Any
    metadata: Mapping[str, object]


@dataclass(frozen=True)
class CameraReadResult:
    """Bounded result from one camera read operation."""

    frame: Optional[CameraFrame]
    timed_out: bool
    error: str = ""

    @property
    def succeeded(self) -> bool:
        return self.frame is not None and not self.timed_out and not self.error
