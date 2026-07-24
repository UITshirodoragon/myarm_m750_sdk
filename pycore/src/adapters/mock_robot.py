"""Deterministic in-memory robot adapter for tests and RViz bring-up."""

from __future__ import annotations

import threading
import time
from typing import Sequence

from myarm_m750_core.domain.errors import InvalidDriverStateError
from myarm_m750_core.domain.models import (
    CommandResult,
    HardwareStatus,
    JointState,
    JointTarget,
    RobotCapabilities,
)
from myarm_m750_core.ports.robot_hardware import RobotHardwarePort


class MockRobotAdapter(RobotHardwarePort):
    """Immediate joint-position backend with no ROS 2 dependency."""

    def __init__(self, initial_position_rad: Sequence[float]) -> None:
        self._position_rad = tuple(float(value) for value in initial_position_rad)
        if len(self._position_rad) != 6:
            raise ValueError("MockRobotAdapter requires six initial joint values.")
        self._connected = False
        self._paused = False
        self._sequence = 0
        self._lock = threading.RLock()

    def connect(self) -> None:
        with self._lock:
            self._connected = True
            self._paused = False

    def disconnect(self) -> None:
        with self._lock:
            self._connected = False

    def _require_connected(self) -> None:
        if not self._connected:
            raise InvalidDriverStateError("Mock adapter is disconnected.")

    def read_state(self) -> JointState:
        with self._lock:
            self._require_connected()
            return JointState(
                position_rad=self._position_rad,
                timestamp_s=time.time(),
                source="mock",
                sequence=self._sequence,
            )

    def write_joint_target(self, target: JointTarget) -> CommandResult:
        with self._lock:
            self._require_connected()
            if self._paused:
                return CommandResult.rejected(
                    "Mock adapter is paused.", "ADAPTER_PAUSED"
                )
            self._position_rad = target.position_rad
            self._sequence += 1
            return CommandResult.success("Mock target applied.")

    def stop(self) -> CommandResult:
        with self._lock:
            self._require_connected()
            return CommandResult.success("Mock motion stopped.")

    def pause(self) -> CommandResult:
        with self._lock:
            self._require_connected()
            self._paused = True
            return CommandResult.success("Mock motion paused.")

    def resume(self) -> CommandResult:
        with self._lock:
            self._require_connected()
            self._paused = False
            return CommandResult.success("Mock motion resumed.")

    def capabilities(self) -> RobotCapabilities:
        return RobotCapabilities(
            supports_pause=True,
            supports_resume=True,
            supports_stop=True,
            supports_power_control=False,
        )

    def status(self) -> HardwareStatus:
        state = "paused" if self._paused else "idle"
        if not self._connected:
            state = "disconnected"
        return HardwareStatus(
            connected=self._connected,
            state=state,
            message="Deterministic in-memory adapter.",
        )
