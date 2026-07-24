"""Hardware port implemented by real, mock, and replay adapters."""

from __future__ import annotations

from abc import ABC, abstractmethod

from myarm_m750_core.domain.models import (
    CommandResult,
    HardwareStatus,
    JointState,
    JointTarget,
    RobotCapabilities,
)


class RobotHardwarePort(ABC):
    """Canonical joint-position boundary to a robot backend."""

    @abstractmethod
    def connect(self) -> None:
        """Connect and place the adapter in an idle state."""

    @abstractmethod
    def disconnect(self) -> None:
        """Release backend resources."""

    @abstractmethod
    def read_state(self) -> JointState:
        """Read measured canonical joint positions."""

    @abstractmethod
    def write_joint_target(self, target: JointTarget) -> CommandResult:
        """Send an already validated canonical joint target."""

    @abstractmethod
    def stop(self) -> CommandResult:
        """Stop motion with the strongest capability provided by the backend."""

    @abstractmethod
    def pause(self) -> CommandResult:
        """Pause motion when supported."""

    @abstractmethod
    def resume(self) -> CommandResult:
        """Resume motion when supported."""

    @abstractmethod
    def capabilities(self) -> RobotCapabilities:
        """Return explicit backend capabilities."""

    @abstractmethod
    def status(self) -> HardwareStatus:
        """Return current adapter diagnostics."""
