"""Camera capture port used by standalone and ROS 2 deployments."""

from __future__ import annotations

from typing import Protocol

from myarm_m750_core.domain.camera import CameraConfig, CameraReadResult


class CameraCapturePort(Protocol):
    """Minimal capture boundary implemented by OpenCV, mock, or future drivers."""

    @property
    def is_open(self) -> bool:
        """Return whether the capture device is currently open."""

    def open(self, config: CameraConfig) -> None:
        """Open one physical camera using validated configuration."""

    def read(self, timeout_s: float) -> CameraReadResult:
        """Read at most one frame using ``timeout_s`` as a best-effort budget.

        Backends must report timeout explicitly. A blocking native driver may not
        provide a hard realtime deadline, so camera reads must stay off the robot
        command callback path.
        """

    def close(self) -> None:
        """Release the camera resource. Repeated calls must be safe."""
