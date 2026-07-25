"""Event-driven, thread-safe runtime state machine."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Dict, Optional, Tuple

from myarm_m750_core.domain.errors import InvalidDriverStateError


class DriverState(Enum):
    """Runtime states visible to diagnostics and tests."""

    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    IDLE = "idle"
    EXECUTING = "executing"
    RECOVERING = "recovering"
    FAULT = "fault"


class DriverEvent(Enum):
    """Named causes accepted by the runtime state machine."""

    CONNECT_REQUESTED = "connect_requested"
    CONNECT_SUCCEEDED = "connect_succeeded"
    CONNECT_FAILED = "connect_failed"
    DISCONNECT_REQUESTED = "disconnect_requested"
    EXECUTION_ACCEPTED = "execution_accepted"
    EXECUTION_SUCCEEDED = "execution_succeeded"
    EXECUTION_CANCELED = "execution_canceled"
    FAULT_DETECTED = "fault_detected"
    RECOVERY_REQUESTED = "recovery_requested"
    RECOVERY_SUCCEEDED = "recovery_succeeded"
    RECOVERY_FAILED = "recovery_failed"


@dataclass(frozen=True)
class TransitionRecord:
    """Traceable state transition used by diagnostics."""

    previous: DriverState
    current: DriverState
    event: DriverEvent
    reason: str
    command_id: Optional[str]
    timestamp_monotonic_s: float


_TRANSITIONS: Dict[Tuple[DriverState, DriverEvent], DriverState] = {
    (DriverState.DISCONNECTED, DriverEvent.CONNECT_REQUESTED): DriverState.CONNECTING,
    (DriverState.CONNECTING, DriverEvent.CONNECT_SUCCEEDED): DriverState.IDLE,
    (DriverState.CONNECTING, DriverEvent.CONNECT_FAILED): DriverState.FAULT,
    (DriverState.CONNECTING, DriverEvent.DISCONNECT_REQUESTED): DriverState.DISCONNECTED,
    (DriverState.IDLE, DriverEvent.DISCONNECT_REQUESTED): DriverState.DISCONNECTED,
    (DriverState.IDLE, DriverEvent.EXECUTION_ACCEPTED): DriverState.EXECUTING,
    (DriverState.IDLE, DriverEvent.FAULT_DETECTED): DriverState.FAULT,
    (DriverState.EXECUTING, DriverEvent.EXECUTION_SUCCEEDED): DriverState.IDLE,
    (DriverState.EXECUTING, DriverEvent.EXECUTION_CANCELED): DriverState.IDLE,
    (DriverState.EXECUTING, DriverEvent.FAULT_DETECTED): DriverState.FAULT,
    (DriverState.EXECUTING, DriverEvent.DISCONNECT_REQUESTED): DriverState.DISCONNECTED,
    (DriverState.FAULT, DriverEvent.RECOVERY_REQUESTED): DriverState.RECOVERING,
    (DriverState.FAULT, DriverEvent.DISCONNECT_REQUESTED): DriverState.DISCONNECTED,
    (DriverState.RECOVERING, DriverEvent.RECOVERY_SUCCEEDED): DriverState.IDLE,
    (DriverState.RECOVERING, DriverEvent.RECOVERY_FAILED): DriverState.FAULT,
    (DriverState.RECOVERING, DriverEvent.DISCONNECT_REQUESTED): DriverState.DISCONNECTED,
}


class DriverStateMachine:
    """Apply named events atomically and retain their diagnostic reason."""

    def __init__(self) -> None:
        self._state = DriverState.DISCONNECTED
        self._history = []  # type: list[TransitionRecord]
        self._lock = threading.RLock()

    @property
    def state(self) -> DriverState:
        """Return the current state."""
        with self._lock:
            return self._state

    @property
    def history(self) -> Tuple[TransitionRecord, ...]:
        """Return an immutable snapshot of accepted transitions."""
        with self._lock:
            return tuple(self._history)

    def apply(
        self,
        event: DriverEvent,
        reason: str,
        command_id: Optional[str] = None,
        timestamp_monotonic_s: Optional[float] = None,
    ) -> TransitionRecord:
        """Apply one permitted event or raise an explicit state error."""
        if not reason:
            raise ValueError("Every state transition requires a reason.")
        with self._lock:
            key = (self._state, event)
            next_state = _TRANSITIONS.get(key)
            if next_state is None:
                raise InvalidDriverStateError(
                    f"Invalid driver event {event.value} while {self._state.value}."
                )
            record = TransitionRecord(
                previous=self._state,
                current=next_state,
                event=event,
                reason=reason,
                command_id=command_id,
                timestamp_monotonic_s=(
                    time.monotonic()
                    if timestamp_monotonic_s is None
                    else float(timestamp_monotonic_s)
                ),
            )
            self._state = next_state
            self._history.append(record)
            return record
