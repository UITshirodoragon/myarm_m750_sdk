"""Read-only JSONL replay adapter for deterministic regression tests."""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import List

from myarm_m750_core.domain.errors import ConfigurationError, InvalidDriverStateError
from myarm_m750_core.domain.models import (
    CommandResult,
    HardwareStatus,
    JointState,
    JointTarget,
    RobotCapabilities,
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
                raise ConfigurationError(
                    "Replay file does not exist: {0}".format(self._replay_path)
                )
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
                            "Invalid replay sample at line {0}: {1}".format(
                                line_number, error
                            )
                        ) from error
                    samples.append(
                        JointState(
                            position_rad=tuple(positions),
                            timestamp_s=float(payload.get("timestamp_s", time.time())),
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

    def read_state(self) -> JointState:
        with self._lock:
            self._require_connected()
            sample = self._samples[self._index]
            if self._index < len(self._samples) - 1:
                self._index += 1
            elif self._loop:
                self._index = 0
            return JointState(
                position_rad=sample.position_rad,
                timestamp_s=time.time(),
                source="replay",
                sequence=sample.sequence,
            )

    def write_joint_target(self, target: JointTarget) -> CommandResult:
        del target
        self._require_connected()
        return CommandResult.rejected(
            "Replay adapter is read-only.", "REPLAY_READ_ONLY"
        )

    def stop(self) -> CommandResult:
        self._require_connected()
        return CommandResult.success("Replay cursor stopped at current sample.")

    def pause(self) -> CommandResult:
        self._require_connected()
        return CommandResult.rejected(
            "Replay pause is not implemented.", "CAPABILITY_NOT_SUPPORTED"
        )

    def resume(self) -> CommandResult:
        self._require_connected()
        return CommandResult.rejected(
            "Replay resume is not implemented.", "CAPABILITY_NOT_SUPPORTED"
        )

    def capabilities(self) -> RobotCapabilities:
        return RobotCapabilities(
            supports_pause=False,
            supports_resume=False,
            supports_stop=True,
            supports_power_control=False,
        )

    def status(self) -> HardwareStatus:
        return HardwareStatus(
            connected=self._connected,
            state="replay" if self._connected else "disconnected",
            message="Read-only JSONL replay adapter.",
        )
