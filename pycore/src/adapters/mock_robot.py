"""Deterministic in-memory robot adapter for tests and RViz bring-up."""

from __future__ import annotations

import threading
import time
from typing import Sequence

from myarm_m750_core.domain.errors import HardwareTimeoutError, InvalidDriverStateError
from myarm_m750_core.domain.models import (
    AdapterCapabilities,
    CapabilityState,
    CommandContext,
    CommandResult,
    HardwareIdentity,
    HardwareStatus,
    JointState,
    JointTarget,
)
from myarm_m750_core.ports.robot_hardware import RobotHardwarePort


class MockRobotAdapter(RobotHardwarePort):
    """Immediate joint-position backend with no ROS 2 dependency."""

    def __init__(self, initial_position_rad: Sequence[float]) -> None:
        self._position_rad = tuple(float(value) for value in initial_position_rad)
        if len(self._position_rad) != 6:
            raise ValueError("MockRobotAdapter requires six initial joint values.")
        self._connected = False
        self._sequence = 0
        self._lock = threading.RLock()

    def connect(self) -> None:
        with self._lock:
            self._connected = True

    def disconnect(self) -> None:
        with self._lock:
            self._connected = False

    def _require_connected(self) -> None:
        if not self._connected:
            raise InvalidDriverStateError("Mock adapter is disconnected.")

    @staticmethod
    def _check_deadline(context: CommandContext) -> None:
        if time.monotonic() > context.deadline_monotonic_s:
            raise HardwareTimeoutError(
                f"Mock operation exceeded deadline for {context.command_id}."
            )

    def read_joint_state(self, context: CommandContext) -> JointState:
        with self._lock:
            self._require_connected()
            self._check_deadline(context)
            return JointState(
                position_rad=self._position_rad,
                sample_wall_time_s=time.time(),
                received_monotonic_s=time.monotonic(),
                source="mock",
                sequence=self._sequence,
            )

    def write_joint_target(
        self, target: JointTarget, context: CommandContext
    ) -> CommandResult:
        with self._lock:
            self._require_connected()
            self._check_deadline(context)
            self._position_rad = target.position_rad
            self._sequence += 1
            return CommandResult.success(
                "Mock target applied.", command_id=context.command_id
            )

    def stop(self, context: CommandContext) -> CommandResult:
        with self._lock:
            self._require_connected()
            self._check_deadline(context)
            return CommandResult.success(
                "Mock motion stopped.", command_id=context.command_id
            )

    def capabilities(self) -> AdapterCapabilities:
        return AdapterCapabilities(
            stop=CapabilityState.SUPPORTED,
            pause=CapabilityState.UNSUPPORTED,
            resume=CapabilityState.UNSUPPORTED,
            power_control=CapabilityState.UNSUPPORTED,
        )

    def read_hardware_status(self) -> HardwareStatus:
        state = "idle" if self._connected else "disconnected"
        return HardwareStatus(
            connected=self._connected,
            state=state,
            message="Deterministic in-memory adapter.",
        )

    def probe_identity(self, context: CommandContext) -> HardwareIdentity:
        self._require_connected()
        self._check_deadline(context)
        return HardwareIdentity(
            adapter="mock",
            model="myarm_m750_mock",
            firmware_version="mock-1",
            serial_resource="memory://myarm_m750",
            mapping_fingerprint="mock-canonical",
            capability_verification_reference="builtin://mock-adapter",
        )
