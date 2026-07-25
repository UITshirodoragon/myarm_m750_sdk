"""Absolute-deadline trajectory execution with generation-based cancellation."""

from __future__ import annotations

import logging
import math
import threading
import time
import uuid
from typing import Callable, List, Optional, Tuple

from myarm_m750_core.domain.models import (
    CommandContext,
    CommandResult,
    ExecutionMetrics,
    JointState,
    JointTarget,
    JointTrajectory,
    JointTrajectoryPoint,
)
from myarm_m750_core.ports.robot_hardware import RobotHardwarePort
from myarm_m750_core.runtime.admission import CommandAdmission
from myarm_m750_core.runtime.scheduler import Scheduler
from myarm_m750_core.runtime.state_machine import (
    DriverEvent,
    DriverState,
    DriverStateMachine,
)

_LOGGER = logging.getLogger(__name__)
ProgressCallback = Callable[[int, JointTrajectoryPoint, JointState], None]


class TrajectoryExecutor:
    """Execute only admitted trajectories and own the adapter command lane."""

    def __init__(
        self,
        hardware: RobotHardwarePort,
        admission: CommandAdmission,
        state_machine: DriverStateMachine,
        scheduler: Scheduler,
        command_timeout_s: float,
        stop_timeout_s: float,
        operation_clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._hardware = hardware
        self._admission = admission
        self._state_machine = state_machine
        self._scheduler = scheduler
        self._command_timeout_s = float(command_timeout_s)
        self._stop_timeout_s = float(stop_timeout_s)
        self._operation_clock = operation_clock
        self._execution_lock = threading.Lock()
        self._command_lane_lock = threading.RLock()
        self._cancellation_generation = 0
        self._active_command_id: Optional[str] = None
        self._stop_outcome: Optional[Tuple[str, CommandResult]] = None
        self._operation_latency_s: List[float] = []
        self._scheduler_jitter_s: List[float] = []
        self._stop_latency_s: List[float] = []
        self._waypoint_count = 0
        self._overrun_count = 0
        self._stale_state_count = 0

    def _generation(self) -> int:
        with self._command_lane_lock:
            return self._cancellation_generation

    def _is_canceled(self, accepted_generation: int) -> bool:
        return self._generation() != accepted_generation

    def _issue_stop_locked(self, command_id: str) -> CommandResult:
        if self._stop_outcome is not None and self._stop_outcome[0] == command_id:
            return self._stop_outcome[1]
        context = CommandContext.with_timeout(self._stop_timeout_s, command_id=command_id)
        started_s = self._operation_clock()
        try:
            result = self._hardware.stop(context)
        except Exception as error:
            _LOGGER.exception(
                "bounded_stop_failed",
                extra={"command_id": command_id},
            )
            result = CommandResult.failed(
                f"Bounded hardware stop raised {type(error).__name__}: {error}",
                "STOP_EXCEPTION",
                command_id=command_id,
            )
        finally:
            self._stop_latency_s.append(self._operation_clock() - started_s)
        self._stop_outcome = (command_id, result)
        return result

    def _fault_after_stop_failure_locked(
        self, command_id: str, result: CommandResult
    ) -> None:
        if result.succeeded:
            return
        if self._state_machine.state in (DriverState.IDLE, DriverState.EXECUTING):
            self._state_machine.apply(
                DriverEvent.FAULT_DETECTED,
                reason=f"Bounded stop did not succeed: {result.message}",
                command_id=command_id,
            )

    def cancel_current_command(self) -> CommandResult:
        """Accept cancellation atomically, then issue one bounded stop."""
        with self._command_lane_lock:
            command_id = self._active_command_id
            if command_id is None:
                return CommandResult.rejected(
                    "There is no active trajectory to cancel.",
                    "NO_ACTIVE_COMMAND",
                )
            self._cancellation_generation += 1
            result = self._issue_stop_locked(command_id)
            self._fault_after_stop_failure_locked(command_id, result)
            return result

    def stop_now(self) -> CommandResult:
        """Cancel any generation and issue a bounded backend stop."""
        with self._command_lane_lock:
            self._cancellation_generation += 1
            command_id = self._active_command_id or str(uuid.uuid4())
            result = self._issue_stop_locked(command_id)
            self._fault_after_stop_failure_locked(command_id, result)
            return result

    def metrics_snapshot(self) -> ExecutionMetrics:
        """Return immutable executor and adapter metric samples."""
        status = self._hardware.read_hardware_status()
        return ExecutionMetrics(
            waypoint_count=self._waypoint_count,
            overrun_count=self._overrun_count,
            stale_state_count=self._stale_state_count,
            retry_count=status.retry_count,
            operation_latency_s=tuple(self._operation_latency_s),
            scheduler_jitter_s=tuple(self._scheduler_jitter_s),
            stop_latency_s=tuple(self._stop_latency_s),
        )

    def execute(
        self,
        trajectory: JointTrajectory,
        current_state: JointState,
        progress_callback: Optional[ProgressCallback] = None,
    ) -> CommandResult:
        """Admit the complete trajectory, then execute on absolute deadlines."""
        command_id = str(uuid.uuid4())
        if not self._execution_lock.acquire(blocking=False):
            return CommandResult.rejected(
                "Another trajectory already owns the robot adapter.",
                "COMMAND_BUSY",
                command_id=command_id,
            )
        try:
            admission = self._admission.admit(command_id, trajectory, current_state)
            if admission.rejection is not None:
                if admission.rejection.error_code == "SAFETY_VALIDATION_FAILED":
                    self._stale_state_count += int(
                        "older than" in admission.rejection.message
                    )
                return admission.rejection
            admitted = admission.admitted
            if admitted is None:
                raise RuntimeError("Admission returned no trajectory.")
            with self._command_lane_lock:
                accepted_generation = self._cancellation_generation
                self._active_command_id = command_id
                self._stop_outcome = None
            self._state_machine.apply(
                DriverEvent.EXECUTION_ACCEPTED,
                reason="complete trajectory admitted",
                command_id=command_id,
            )
            start_monotonic_s = self._scheduler.now()
            points = admitted.trajectory.points
            for point_index, point in enumerate(points):
                scheduled_s = start_monotonic_s + point.time_from_start_s
                jitter_s = self._scheduler.wait_until(
                    scheduled_s,
                    lambda: self._is_canceled(accepted_generation),
                )
                self._scheduler_jitter_s.append(jitter_s)
                if self._is_canceled(accepted_generation):
                    return self._finish_canceled(command_id, point_index)

                with self._command_lane_lock:
                    if self._is_canceled(accepted_generation):
                        return self._finish_canceled(command_id, point_index)
                    context = CommandContext.with_timeout(
                        self._command_timeout_s,
                        command_id=command_id,
                        point_index=point_index,
                    )
                    operation_start_s = self._operation_clock()
                    write_result = self._hardware.write_joint_target(
                        JointTarget(point.position_rad), context
                    )
                    latency_s = self._operation_clock() - operation_start_s
                    self._operation_latency_s.append(latency_s)
                    self._waypoint_count += 1
                if not write_result.succeeded:
                    return self._finish_failed(
                        command_id,
                        f"Hardware rejected point {point_index}: {write_result.message}",
                        write_result.error_code or "HARDWARE_WRITE_FAILED",
                    )
                actual = self._hardware.read_joint_state(
                    CommandContext.with_timeout(
                        self._command_timeout_s,
                        command_id=command_id,
                        point_index=point_index,
                    )
                )
                if not math.isfinite(actual.received_monotonic_s):
                    state_age_s = math.inf
                else:
                    state_age_s = actual.age_s(self._operation_clock())
                if state_age_s > self._admission.state_timeout_s:
                    self._stale_state_count += 1
                    return self._finish_failed(
                        command_id,
                        (
                            f"Measured state watchdog expired after point {point_index}: "
                            f"age {state_age_s:.6f}s exceeds "
                            f"{self._admission.state_timeout_s:.6f}s."
                        ),
                        "STALE_STATE_WATCHDOG",
                    )
                if progress_callback is not None:
                    progress_callback(point_index, point, actual)
                if point_index + 1 < len(points):
                    next_deadline_s = (
                        start_monotonic_s + points[point_index + 1].time_from_start_s
                    )
                    if self._scheduler.now() > next_deadline_s:
                        self._overrun_count += 1
            with self._command_lane_lock:
                if self._is_canceled(accepted_generation):
                    return self._finish_canceled(command_id, len(points))
                self._state_machine.apply(
                    DriverEvent.EXECUTION_SUCCEEDED,
                    reason="all admitted waypoints acknowledged",
                    command_id=command_id,
                )
                self._active_command_id = None
                return CommandResult.success(
                    "Trajectory completed successfully.", command_id=command_id
                )
        except Exception as error:
            _LOGGER.exception(
                "trajectory_execution_failed", extra={"command_id": command_id}
            )
            return self._finish_failed(
                command_id,
                f"Trajectory execution failed: {error}",
                type(error).__name__,
            )
        finally:
            with self._command_lane_lock:
                if self._active_command_id == command_id:
                    self._active_command_id = None
            self._execution_lock.release()

    def _finish_canceled(self, command_id: str, point_index: int) -> CommandResult:
        with self._command_lane_lock:
            stop_result = self._issue_stop_locked(command_id)
        if not stop_result.succeeded:
            return self._finish_failed(
                command_id,
                f"Bounded stop failed during cancellation: {stop_result.message}",
                stop_result.error_code or "STOP_FAILED",
            )
        if self._state_machine.state is DriverState.EXECUTING:
            self._state_machine.apply(
                DriverEvent.EXECUTION_CANCELED,
                reason="cancellation generation accepted",
                command_id=command_id,
            )
        return CommandResult.canceled(
            f"Trajectory canceled before point {point_index}.",
            command_id=command_id,
        )

    def _finish_failed(
        self,
        command_id: str,
        message: str,
        error_code: str,
    ) -> CommandResult:
        with self._command_lane_lock:
            stop_result = self._issue_stop_locked(command_id)
        result_message = message
        result_error_code = error_code
        if not stop_result.succeeded:
            stop_error_code = stop_result.error_code or "STOP_FAILED"
            stop_is_already_primary = (
                stop_error_code == error_code
                and stop_result.message in message
            )
            if not stop_is_already_primary:
                result_message = (
                    f"{message} Bounded stop also failed "
                    f"({stop_error_code}): {stop_result.message}"
                )
                result_error_code = f"{error_code}+{stop_error_code}"
        if self._state_machine.state in (DriverState.IDLE, DriverState.EXECUTING):
            self._state_machine.apply(
                DriverEvent.FAULT_DETECTED,
                reason=result_message,
                command_id=command_id,
            )
        return CommandResult.failed(
            result_message,
            result_error_code,
            command_id=command_id,
        )
