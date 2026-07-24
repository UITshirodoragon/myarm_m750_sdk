"""Explicit, thread-safe runtime state machine for command execution."""

from __future__ import annotations

import threading
from enum import Enum
from typing import Dict, Set

from myarm_m750_core.domain.errors import InvalidDriverStateError


class DriverState(Enum):
    """Runtime states visible to diagnostics and tests."""

    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    IDLE = "idle"
    EXECUTING = "executing"
    PAUSED = "paused"
    RECOVERING = "recovering"
    FAULT = "fault"


_ALLOWED_TRANSITIONS: Dict[DriverState, Set[DriverState]] = {
    DriverState.DISCONNECTED: {DriverState.CONNECTING},
    DriverState.CONNECTING: {
        DriverState.IDLE,
        DriverState.FAULT,
        DriverState.DISCONNECTED,
    },
    DriverState.IDLE: {
        DriverState.EXECUTING,
        DriverState.PAUSED,
        DriverState.RECOVERING,
        DriverState.FAULT,
        DriverState.DISCONNECTED,
    },
    DriverState.EXECUTING: {
        DriverState.IDLE,
        DriverState.PAUSED,
        DriverState.FAULT,
    },
    DriverState.PAUSED: {
        DriverState.EXECUTING,
        DriverState.IDLE,
        DriverState.RECOVERING,
        DriverState.FAULT,
        DriverState.DISCONNECTED,
    },
    DriverState.RECOVERING: {
        DriverState.IDLE,
        DriverState.FAULT,
        DriverState.DISCONNECTED,
    },
    DriverState.FAULT: {DriverState.RECOVERING, DriverState.DISCONNECTED},
}


class DriverStateMachine:
    """Small explicit state machine with atomic validated transitions."""

    def __init__(self) -> None:
        self._state = DriverState.DISCONNECTED
        self._lock = threading.RLock()

    @property
    def state(self) -> DriverState:
        """Return the current state."""
        with self._lock:
            return self._state

    def transition_to(self, next_state: DriverState) -> None:
        """Move to a permitted state or raise an explicit error."""
        with self._lock:
            if next_state is self._state:
                return
            allowed = _ALLOWED_TRANSITIONS[self._state]
            if next_state not in allowed:
                raise InvalidDriverStateError(
                    "Invalid driver state transition: {0} -> {1}.".format(
                        self._state.value, next_state.value
                    )
                )
            self._state = next_state
