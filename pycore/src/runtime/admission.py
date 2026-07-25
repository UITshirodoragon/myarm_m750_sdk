"""Command admission separated from pure trajectory validation and execution."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

from myarm_m750_core.domain.models import (
    AdmittedTrajectory,
    CommandResult,
    JointState,
    JointTrajectory,
)
from myarm_m750_core.domain.safety import TrajectoryValidator
from myarm_m750_core.runtime.scheduler import Scheduler
from myarm_m750_core.runtime.state_machine import DriverState


@dataclass(frozen=True)
class AdmissionResult:
    """Exactly one admitted trajectory or public rejection."""

    admitted: Optional[AdmittedTrajectory] = None
    rejection: Optional[CommandResult] = None

    def __post_init__(self) -> None:
        if (self.admitted is None) == (self.rejection is None):
            raise ValueError("AdmissionResult must contain exactly one outcome.")


class CommandAdmission:
    """Admit a command only from IDLE after complete pure validation."""

    def __init__(
        self,
        validator: TrajectoryValidator,
        state_reader: Callable[[], DriverState],
        scheduler: Scheduler,
    ) -> None:
        self._validator = validator
        self._state_reader = state_reader
        self._scheduler = scheduler

    @property
    def state_timeout_s(self) -> float:
        """Return the mandatory measured-state freshness timeout."""
        return self._validator.policy.state_timeout_s

    def admit(
        self,
        command_id: str,
        trajectory: JointTrajectory,
        current_state: JointState,
    ) -> AdmissionResult:
        """Validate state and full trajectory without issuing hardware writes."""
        if self._state_reader() is not DriverState.IDLE:
            return AdmissionResult(
                rejection=CommandResult.rejected(
                    "Trajectory admission requires the IDLE state.",
                    "DRIVER_NOT_IDLE",
                    command_id=command_id,
                )
            )
        validation = self._validator.validate(trajectory, current_state)
        if not validation.is_valid:
            first = validation.first_violation
            message = first.message if first is not None else "Trajectory rejected."
            return AdmissionResult(
                rejection=CommandResult.rejected(
                    message,
                    "SAFETY_VALIDATION_FAILED",
                    command_id=command_id,
                )
            )
        policy = self._validator.policy
        return AdmissionResult(
            admitted=AdmittedTrajectory(
                command_id=command_id,
                trajectory=trajectory,
                initial_state=current_state,
                admitted_monotonic_s=self._scheduler.now(),
                model_fingerprint=policy.model_fingerprint,
                limit_provenance=policy.limit_provenance,
            )
        )
