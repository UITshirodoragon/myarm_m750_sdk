"""Hardware port implemented by real, mock, and replay adapters."""

from __future__ import annotations

from abc import ABC, abstractmethod

from myarm_m750_core.domain.models import (
    AdapterCapabilities,
    CommandContext,
    CommandResult,
    HardwareIdentity,
    HardwareStatus,
    JointState,
    JointTarget,
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
    def read_joint_state(self, context: CommandContext) -> JointState:
        """Read measured canonical joint positions."""

    @abstractmethod
    def write_joint_target(
        self, target: JointTarget, context: CommandContext
    ) -> CommandResult:
        """Send an already validated canonical joint target."""

    @abstractmethod
    def stop(self, context: CommandContext) -> CommandResult:
        """Stop motion with the strongest capability provided by the backend."""

    @abstractmethod
    def capabilities(self) -> AdapterCapabilities:
        """Return explicit backend capabilities."""

    @abstractmethod
    def read_hardware_status(self) -> HardwareStatus:
        """Return current adapter diagnostics."""

    @abstractmethod
    def probe_identity(self, context: CommandContext) -> HardwareIdentity:
        """Read backend identity without sending a motion command."""
