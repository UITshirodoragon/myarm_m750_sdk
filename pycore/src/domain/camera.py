"""Typed ROS-independent camera contracts."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from types import MappingProxyType
from typing import Mapping, Tuple

import numpy as np

_ROS_SAFE_HARDWARE_NAME = re.compile(r"^[a-z][a-z0-9_]*$")
_SUPPORTED_RAW_ENCODINGS = ("mono8", "bgr8", "rgb8")


class CameraState(Enum):
    """Observable lifecycle state for one independent camera worker."""

    CLOSED = "closed"
    OPENING = "opening"
    STREAMING = "streaming"
    BACKOFF = "backoff"
    FAULT = "fault"
    STOPPING = "stopping"


@dataclass(frozen=True)
class CameraStreamConfig:
    """Requested raw camera stream format."""

    width_px: int
    height_px: int
    fps_hz: float
    pixel_format: str

    def __post_init__(self) -> None:
        if self.width_px <= 0 or self.height_px <= 0:
            raise ValueError("Camera width and height must be positive.")
        if not math.isfinite(self.fps_hz) or self.fps_hz <= 0.0:
            raise ValueError("Camera FPS must be finite and positive.")
        if self.pixel_format not in _SUPPORTED_RAW_ENCODINGS:
            supported = ", ".join(_SUPPORTED_RAW_ENCODINGS)
            raise ValueError(
                f"Unsupported pixel_format {self.pixel_format!r}; expected {supported}."
            )


@dataclass(frozen=True)
class CameraCalibration:
    """Validated pinhole calibration compatible with ROS CameraInfo."""

    image_width_px: int
    image_height_px: int
    camera_matrix: Tuple[float, ...]
    distortion_model: str
    distortion_coefficients: Tuple[float, ...]
    rectification_matrix: Tuple[float, ...]
    projection_matrix: Tuple[float, ...]
    source_path: Path

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "camera_matrix",
            tuple(float(value) for value in self.camera_matrix),
        )
        object.__setattr__(
            self,
            "distortion_coefficients",
            tuple(float(value) for value in self.distortion_coefficients),
        )
        object.__setattr__(
            self,
            "rectification_matrix",
            tuple(float(value) for value in self.rectification_matrix),
        )
        object.__setattr__(
            self,
            "projection_matrix",
            tuple(float(value) for value in self.projection_matrix),
        )
        expected = (
            ("camera_matrix", self.camera_matrix, 9),
            ("rectification_matrix", self.rectification_matrix, 9),
            ("projection_matrix", self.projection_matrix, 12),
        )
        for field_name, values, size in expected:
            if len(values) != size or not all(math.isfinite(value) for value in values):
                raise ValueError(f"{field_name} must contain {size} finite values.")
        if not self.distortion_coefficients or not all(
            math.isfinite(value) for value in self.distortion_coefficients
        ):
            raise ValueError("distortion_coefficients must be finite and non-empty.")
        if self.image_width_px <= 0 or self.image_height_px <= 0:
            raise ValueError("Calibration dimensions must be positive.")
        if not self.distortion_model:
            raise ValueError("distortion_model must be non-empty.")


@dataclass(frozen=True)
class CameraExtrinsics:
    """Static parent-to-camera transform with XYZW quaternion order."""

    parent_frame: str
    child_frame: str
    translation_m: Tuple[float, float, float]
    quaternion_xyzw: Tuple[float, float, float, float]

    def __post_init__(self) -> None:
        if len(self.translation_m) != 3:
            raise ValueError("Camera extrinsic translation must contain 3 values.")
        if len(self.quaternion_xyzw) != 4:
            raise ValueError("Camera extrinsic quaternion must contain 4 values.")
        translation_m = tuple(float(value) for value in self.translation_m)
        quaternion_xyzw = tuple(float(value) for value in self.quaternion_xyzw)
        values = translation_m + quaternion_xyzw
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Camera extrinsics must contain finite values.")
        norm = math.sqrt(sum(value * value for value in quaternion_xyzw))
        if norm < 1.0e-12:
            raise ValueError("Camera extrinsic quaternion must be non-zero.")
        object.__setattr__(self, "translation_m", translation_m)
        object.__setattr__(
            self,
            "quaternion_xyzw",
            tuple(value / norm for value in quaternion_xyzw),
        )
        if not self.parent_frame.strip() or not self.child_frame.strip():
            raise ValueError("Extrinsic frames must be non-empty.")
        if self.parent_frame == self.child_frame:
            raise ValueError("Extrinsic parent and child frames must be distinct.")


@dataclass(frozen=True)
class CameraReconnectPolicy:
    """Bounded timeout, exponential backoff, and reopen policy."""

    read_timeout_s: float
    initial_backoff_s: float
    maximum_backoff_s: float
    multiplier: float
    maximum_attempts: int

    def __post_init__(self) -> None:
        values = (
            self.read_timeout_s,
            self.initial_backoff_s,
            self.maximum_backoff_s,
            self.multiplier,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Camera reconnect timing values must be finite.")
        if self.read_timeout_s <= 0.0:
            raise ValueError("read_timeout_s must be positive.")
        if self.initial_backoff_s < 0.0:
            raise ValueError("initial_backoff_s must be non-negative.")
        if self.maximum_backoff_s < self.initial_backoff_s:
            raise ValueError("maximum_backoff_s must be at least initial_backoff_s.")
        if self.multiplier < 1.0:
            raise ValueError("Backoff multiplier must be at least one.")
        if self.maximum_attempts < 0:
            raise ValueError("maximum_attempts must be non-negative.")


@dataclass(frozen=True)
class CameraConfig:
    """One camera identified by immutable hardware identity."""

    hardware_name: str
    enabled: bool
    backend: str
    hardware_model: str
    hardware_serial: str
    role: str
    device_by_id: str
    stream: CameraStreamConfig
    camera_frame: str
    optical_frame: str
    calibration: CameraCalibration
    extrinsics: CameraExtrinsics
    reconnect: CameraReconnectPolicy

    def __post_init__(self) -> None:
        if not self.hardware_name.strip() or not self.hardware_serial.strip():
            raise ValueError("Camera hardware name and serial must be non-empty.")
        if _ROS_SAFE_HARDWARE_NAME.fullmatch(self.hardware_name) is None:
            raise ValueError(
                "Camera hardware_name must be lower_snake_case and ROS-name safe."
            )
        if not self.hardware_model.strip() or not self.role.strip():
            raise ValueError("Camera hardware model and role must be non-empty.")
        if self.backend not in ("mock", "opencv"):
            raise ValueError("Camera backend must be mock or opencv.")
        if self.backend == "opencv" and not self.device_by_id.startswith("/dev/v4l/by-id/"):
            raise ValueError("OpenCV camera must use a stable /dev/v4l/by-id path.")
        if not self.camera_frame.strip() or not self.optical_frame.strip():
            raise ValueError("Camera and optical frames must be non-empty.")
        if self.camera_frame == self.optical_frame:
            raise ValueError("Camera frame and optical frame must be distinct.")
        if self.extrinsics.child_frame != self.camera_frame:
            raise ValueError(
                "Camera extrinsics.child_frame must equal camera_frame."
            )
        if self.stream.width_px != self.calibration.image_width_px:
            raise ValueError("Stream/calibration image width mismatch.")
        if self.stream.height_px != self.calibration.image_height_px:
            raise ValueError("Stream/calibration image height mismatch.")


@dataclass(frozen=True)
class CameraFrame:
    """Latest raw frame with acquisition and observation clocks."""

    camera_name: str
    sequence: int
    acquisition_monotonic_s: float
    observation_wall_time_s: float
    image: np.ndarray
    encoding: str
    metadata: Mapping[str, str]

    def __post_init__(self) -> None:
        if not self.camera_name.strip():
            raise ValueError("Camera frame name must be non-empty.")
        if self.sequence < 0:
            raise ValueError("Camera frame sequence must be non-negative.")
        if not math.isfinite(self.acquisition_monotonic_s):
            raise ValueError("Camera acquisition monotonic time must be finite.")
        if not math.isfinite(self.observation_wall_time_s):
            raise ValueError("Camera observation wall time must be finite.")
        object.__setattr__(self, "metadata", MappingProxyType(dict(self.metadata)))
        if self.image.dtype != np.uint8:
            raise ValueError(f"{self.encoding} camera image must use uint8 data.")
        if self.encoding == "mono8":
            if self.image.ndim != 2 or self.image.size == 0:
                raise ValueError("mono8 camera image must be a non-empty HxW array.")
        elif self.encoding in ("bgr8", "rgb8"):
            if (
                self.image.ndim != 3
                or self.image.shape[2] != 3
                or self.image.size == 0
            ):
                raise ValueError(
                    f"{self.encoding} camera image must be a non-empty HxWx3 array."
                )
        else:
            supported = ", ".join(_SUPPORTED_RAW_ENCODINGS)
            raise ValueError(
                f"Unsupported raw camera encoding {self.encoding!r}; expected {supported}."
            )


@dataclass(frozen=True)
class CameraMetricsSnapshot:
    """Immutable per-camera capture and queue metrics."""

    frames_captured: int
    read_timeouts: int
    capture_errors: int
    reconnect_count: int
    queue_overflow_count: int
    last_frame_age_s: float
    last_error: str

    def __post_init__(self) -> None:
        counters = (
            self.frames_captured,
            self.read_timeouts,
            self.capture_errors,
            self.reconnect_count,
            self.queue_overflow_count,
        )
        if any(value < 0 for value in counters):
            raise ValueError("Camera metric counters must be non-negative.")
        if not math.isfinite(self.last_frame_age_s) or self.last_frame_age_s < 0.0:
            raise ValueError("last_frame_age_s must be finite and non-negative.")
