"""Validated 5 Hz trajectory execution independent of ROS 2."""

from __future__ import annotations

import logging
import threading
import time
import uuid
from typing import Callable, Optional

from myarm_m750_core.domain.models import (
    CommandResult,
    JointState,
    JointTarget,
    JointTrajectory,
)
from myarm_m750_core.ports.robot_hardware import RobotHardwarePort
from myarm_m750_core.runtime.state_machine import DriverState, DriverStateMachine
from myarm_m750_core.domain.safety.motion_guard import MotionGuard

_LOGGER = logging.getLogger(__name__)


class TrajectoryExecutor:
    """Validate complete trajectories before issuing any hardware target.

    Only one trajectory can own the adapter at a time. ``request_stop()`` is
    thread-safe and is checked both before and after each scheduled wait so a
    ROS action cancel or public stop command does not have to wait for the next
    serial write.
    """

    def __init__(
        self,
        hardware: RobotHardwarePort,
        motion_guard: MotionGuard,
        state_machine: DriverStateMachine,
        realtime_execution: bool,
        monotonic_clock: Callable[[], float] = time.monotonic,
        sleeper: Callable[[float], None] = time.sleep,
    ) -> None:
        self._hardware = hardware
        self._motion_guard = motion_guard
        self._state_machine = state_machine
        self._realtime_execution = bool(realtime_execution)
        self._monotonic_clock = monotonic_clock
        self._sleeper = sleeper
        self._execution_lock = threading.Lock()
        self._stop_requested = threading.Event()

    def request_stop(self) -> None:
        """Request cancellation of the active trajectory without blocking."""
        self._stop_requested.set()

    def _is_cancellation_requested(
        self, cancel_requested: Optional[Callable[[], bool]]
    ) -> bool:
        return self._stop_requested.is_set() or (
            cancel_requested is not None and cancel_requested()
        )

    def _transition_idle_if_active(self) -> None:
        if self._state_machine.state in (DriverState.EXECUTING, DriverState.PAUSED):
            self._state_machine.transition_to(DriverState.IDLE)

    def _cancel_result(self, command_id: str, point_index: int) -> CommandResult:
        self._hardware.stop()
        self._transition_idle_if_active()
        return CommandResult.canceled(
            "Trajectory stopped before point {0}.".format(point_index),
            command_id=command_id,
        )

    def execute(
        self,
        trajectory: JointTrajectory,
        current_state: JointState,
        cancel_requested: Optional[Callable[[], bool]] = None,
    ) -> CommandResult:
        """Execute a pre-generated trajectory after fail-fast validation."""
        command_id = str(uuid.uuid4())
        if not self._execution_lock.acquire(blocking=False):
            return CommandResult.rejected(
                "Another trajectory already owns the robot adapter.",
                error_code="COMMAND_BUSY",
                command_id=command_id,
            )

        try:
            validation = self._motion_guard.validate_trajectory(
                trajectory, current_state
            )
            if not validation.is_valid:
                first = validation.first_violation
                message = first.message if first is not None else "Trajectory rejected."
                _LOGGER.warning(
                    "trajectory_rejected",
                    extra={
                        "command_id": command_id,
                        "violation_count": len(validation.violations),
                        "first_violation": message,
                    },
                )
                return CommandResult.rejected(
                    message=message,
                    error_code="SAFETY_VALIDATION_FAILED",
                    command_id=command_id,
                )

            if self._state_machine.state is not DriverState.IDLE:
                return CommandResult.rejected(
                    "Trajectory execution requires the driver to be IDLE; current state is "
                    "{0}.".format(self._state_machine.state.value),
                    error_code="DRIVER_NOT_IDLE",
                    command_id=command_id,
                )

            self._stop_requested.clear()
            self._state_machine.transition_to(DriverState.EXECUTING)
            start_monotonic_s = self._monotonic_clock()
            _LOGGER.info(
                "trajectory_started",
                extra={
                    "command_id": command_id,
                    "point_count": len(trajectory.points),
                    "duration_s": trajectory.duration_s,
                },
            )

            for point_index, point in enumerate(trajectory.points):
                if self._is_cancellation_requested(cancel_requested):
                    return self._cancel_result(command_id, point_index)

                if self._realtime_execution:
                    target_monotonic_s = start_monotonic_s + point.time_from_start_s
                    sleep_duration_s = target_monotonic_s - self._monotonic_clock()
                    if sleep_duration_s > 0.0:
                        self._sleeper(sleep_duration_s)

                # A stop can arrive while sleeping. Check again before touching
                # hardware so no stale point is sent after cancellation.
                if self._is_cancellation_requested(cancel_requested):
                    return self._cancel_result(command_id, point_index)

                write_result = self._hardware.write_joint_target(
                    JointTarget(point.position_rad)
                )
                if not write_result.succeeded:
                    self._hardware.stop()
                    self._state_machine.transition_to(DriverState.FAULT)
                    return CommandResult.failed(
                        "Hardware rejected trajectory point {0}: {1}".format(
                            point_index, write_result.message
                        ),
                        error_code=write_result.error_code or "HARDWARE_WRITE_FAILED",
                        command_id=command_id,
                    )

            self._transition_idle_if_active()
            _LOGGER.info("trajectory_succeeded", extra={"command_id": command_id})
            return CommandResult.success(
                "Trajectory completed successfully.", command_id=command_id
            )
        except Exception as error:
            try:
                self._hardware.stop()
            finally:
                if self._state_machine.state is not DriverState.FAULT:
                    self._state_machine.transition_to(DriverState.FAULT)
            _LOGGER.exception(
                "trajectory_execution_failed",
                extra={"command_id": command_id},
            )
            return CommandResult.failed(
                message="Trajectory execution failed: {0}".format(error),
                error_code=type(error).__name__,
                command_id=command_id,
            )
        finally:
            self._execution_lock.release()
