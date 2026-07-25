"""Camera capture boundary implemented by mock and OpenCV adapters."""

from __future__ import annotations

from abc import ABC, abstractmethod

from myarm_m750_core.domain.camera import CameraConfig, CameraFrame


class CameraCapturePort(ABC):
    """One blocking camera device; workers keep it off robot callbacks."""

    @property
    @abstractmethod
    def is_open(self) -> bool:
        """Return whether the capture device is open."""

    @abstractmethod
    def open(self, config: CameraConfig) -> None:
        """Open the configured physical/synthetic camera idempotently."""

    @abstractmethod
    def read_frame(self, timeout_s: float) -> CameraFrame:
        """Return one frame or raise a typed timeout/capture error."""

    @abstractmethod
    def close(self) -> None:
        """Release resources idempotently."""
