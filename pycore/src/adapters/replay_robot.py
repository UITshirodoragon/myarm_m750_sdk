"""Read-only JSONL replay adapter for deterministic regression tests."""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import List

from myarm_m750_core.domain.errors import (
    ConfigurationError,
    HardwareTimeoutError,
    InvalidDriverStateError,
)
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


class ReplayRobotAdapter(RobotHardwarePort):
    """Replay canonical joint samples from a JSON-lines file."""

    def __init__(self, replay_file: str, loop: bool = False) -> None:
        self._replay_path = Path(replay_file).expanduser().resolve()
        self._loop = bool(loop)
        self._samples: List[JointState] = []
        self._index = 0
        self._connected = False
        self._lock = threading.RLock()

    def connect(self) -> None:
        with self._lock:
            if not self._replay_path.is_file():
                raise ConfigurationError(f"Replay file does not exist: {self._replay_path}")
            samples: List[JointState] = []
            with self._replay_path.open("r", encoding="utf-8") as stream:
                for line_number, line in enumerate(stream, start=1):
                    stripped = line.strip()
                    if not stripped:
                        continue
                    try:
                        payload = json.loads(stripped)
                        positions = payload["position_rad"]
                    except (json.JSONDecodeError, KeyError, TypeError) as error:
                        raise ConfigurationError(
                            f"Invalid replay sample at line {line_number}: {error}"
                        ) from error
                    samples.append(
                        JointState(
                            position_rad=tuple(positions),
                            sample_wall_time_s=float(
                                payload.get("timestamp_s", time.time())
                            ),
                            received_monotonic_s=time.monotonic(),
                            source="replay",
                            sequence=int(payload.get("sequence", line_number - 1)),
                        )
                    )
            if not samples:
                raise ConfigurationError("Replay file contains no joint samples.")
            self._samples = samples
            self._index = 0
            self._connected = True

    def disconnect(self) -> None:
        with self._lock:
            self._connected = False

    def _require_connected(self) -> None:
        if not self._connected:
            raise InvalidDriverStateError("Replay adapter is disconnected.")

    @staticmethod
    def _check_deadline(context: CommandContext) -> None:
        if time.monotonic() > context.deadline_monotonic_s:
            raise HardwareTimeoutError(
                f"Replay operation exceeded deadline for {context.command_id}."
            )

    def read_joint_state(self, context: CommandContext) -> JointState:
        with self._lock:
            self._require_connected()
            self._check_deadline(context)
            sample = self._samples[self._index]
            if self._index < len(self._samples) - 1:
                self._index += 1
            elif self._loop:
                self._index = 0
            return JointState(
                position_rad=sample.position_rad,
                sample_wall_time_s=time.time(),
                received_monotonic_s=time.monotonic(),
                source="replay",
                sequence=sample.sequence,
            )

    def write_joint_target(
        self, target: JointTarget, context: CommandContext
    ) -> CommandResult:
        del target
        self._require_connected()
        self._check_deadline(context)
        return CommandResult.rejected(
            "Replay adapter is read-only.",
            "REPLAY_READ_ONLY",
            command_id=context.command_id,
        )

    def stop(self, context: CommandContext) -> CommandResult:
        self._require_connected()
        self._check_deadline(context)
        return CommandResult.success(
            "Replay cursor stopped at current sample.", command_id=context.command_id
        )

    def capabilities(self) -> AdapterCapabilities:
        return AdapterCapabilities(
            stop=CapabilityState.SUPPORTED,
            pause=CapabilityState.UNSUPPORTED,
            resume=CapabilityState.UNSUPPORTED,
            power_control=CapabilityState.UNSUPPORTED,
        )

    def read_hardware_status(self) -> HardwareStatus:
        return HardwareStatus(
            connected=self._connected,
            state="replay" if self._connected else "disconnected",
            message="Read-only JSONL replay adapter.",
        )

    def probe_identity(self, context: CommandContext) -> HardwareIdentity:
        self._require_connected()
        self._check_deadline(context)
        return HardwareIdentity(
            adapter="replay",
            model="myarm_m750_replay",
            firmware_version="replay-1",
            serial_resource=str(self._replay_path),
            mapping_fingerprint="replay-canonical",
            capability_verification_reference="builtin://replay-adapter",
        )
