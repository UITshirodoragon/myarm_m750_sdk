"""Application service behind the public session and ROS facade."""

from __future__ import annotations

import logging
from typing import Optional, Sequence

import numpy as np
from myarm_m750_core.domain.errors import HardwareStopError
from myarm_m750_core.domain.models import (
    AdapterCapabilities,
    CommandContext,
    CommandResult,
    ExecutionMetrics,
    HardwareIdentity,
    HardwareStatus,
    JointState,
    JointTarget,
    JointTrajectory,
    MotionProfile,
    RigidTransform,
)
from myarm_m750_core.ports.kinematics import KinematicsPort
from myarm_m750_core.ports.robot_hardware import RobotHardwarePort
from myarm_m750_core.runtime.executor import ProgressCallback, TrajectoryExecutor
from myarm_m750_core.runtime.state_machine import (
    DriverEvent,
    DriverState,
    DriverStateMachine,
)
from myarm_m750_core.runtime.trajectory import PointToPointTrajectoryGenerator

_LOGGER = logging.getLogger(__name__)


class RobotController:
    """Coordinate hardware, kinematics, admission, execution, and recovery."""

    def __init__(
        self,
        joint_names: Sequence[str],
        hardware: RobotHardwarePort,
        kinematics: KinematicsPort,
        trajectory_generator: PointToPointTrajectoryGenerator,
        trajectory_executor: TrajectoryExecutor,
        state_machine: DriverStateMachine,
        read_timeout_s: float,
    ) -> None:
        self._joint_names = tuple(joint_names)
        self._hardware = hardware
        self._kinematics = kinematics
        self._trajectory_generator = trajectory_generator
        self._trajectory_executor = trajectory_executor
        self._state_machine = state_machine
        self._read_timeout_s = float(read_timeout_s)

    @property
    def state(self) -> DriverState:
        """Return the explicit runtime state."""
        return self._state_machine.state

    @property
    def joint_names(self) -> Sequence[str]:
        """Return canonical joint names in model order."""
        return self._joint_names

    def connect(self) -> None:
        """Connect hardware and enter IDLE or a traceable FAULT."""
        self._state_machine.apply(
            DriverEvent.CONNECT_REQUESTED, reason="session connect requested"
        )
        try:
            self._hardware.connect()
            self._state_machine.apply(
                DriverEvent.CONNECT_SUCCEEDED,
                reason="adapter connection completed",
            )
        except Exception:
            cleanup_succeeded = False
            try:
                self._hardware.disconnect()
            except Exception:
                _LOGGER.exception("adapter_cleanup_after_connect_failure")
            else:
                cleanup_succeeded = True
            self._state_machine.apply(
                DriverEvent.CONNECT_FAILED,
                reason="adapter connection failed",
            )
            if cleanup_succeeded:
                self._state_machine.apply(
                    DriverEvent.DISCONNECT_REQUESTED,
                    reason="adapter cleanup completed after connection failure",
                )
            raise

    def disconnect(self) -> None:
        """Bound active work, release hardware, and become DISCONNECTED."""
        stop_error = None  # type: Optional[HardwareStopError]
        if self._state_machine.state is DriverState.EXECUTING:
            try:
                stop_result = self._trajectory_executor.stop_now()
            except Exception as error:
                stop_error = HardwareStopError(
                    (
                        "Active-command stop raised during shutdown: "
                        f"{type(error).__name__}: {error}"
                    ),
                    command_id="unknown",
                    error_code="STOP_EXCEPTION",
                )
            else:
                if not stop_result.succeeded:
                    stop_error = HardwareStopError(
                        (
                            "Active-command stop did not succeed during shutdown: "
                            f"{stop_result.message}"
                        ),
                        command_id=stop_result.command_id,
                        error_code=stop_result.error_code or "STOP_FAILED",
                    )

        disconnect_error = None  # type: Optional[Exception]
        try:
            self._hardware.disconnect()
        except Exception as error:
            disconnect_error = error
        else:
            if self._state_machine.state is not DriverState.DISCONNECTED:
                self._state_machine.apply(
                    DriverEvent.DISCONNECT_REQUESTED,
                    reason="session released adapter ownership",
                )

        if stop_error is not None:
            if disconnect_error is not None:
                combined = HardwareStopError(
                    (
                        f"{stop_error}; adapter disconnect also failed: "
                        f"{type(disconnect_error).__name__}: {disconnect_error}"
                    ),
                    command_id=stop_error.command_id,
                    error_code=stop_error.error_code,
                )
                raise combined from disconnect_error
            raise stop_error
        if disconnect_error is not None:
            raise disconnect_error

    def read_joint_state(self) -> JointState:
        """Read measured canonical state through a bounded hardware operation."""
        return self._hardware.read_joint_state(
            CommandContext.with_timeout(self._read_timeout_s)
        )

    def read_hardware_status(self) -> HardwareStatus:
        """Return adapter diagnostics without exposing a vendor object."""
        return self._hardware.read_hardware_status()

    def adapter_capabilities(self) -> AdapterCapabilities:
        """Return only explicitly verified adapter capability states."""
        return self._hardware.capabilities()

    def probe_hardware(self) -> HardwareIdentity:
        """Read identity and one state without issuing a motion command."""
        identity = self._hardware.probe_identity(
            CommandContext.with_timeout(self._read_timeout_s)
        )
        self.read_joint_state()
        return identity

    def compute_fk(self, joint_position_rad: Sequence[float]) -> RigidTransform:
        """Compute FK without reading or commanding hardware."""
        return self._kinematics.compute_fk(joint_position_rad)

    def compute_jacobian(self, joint_position_rad: Sequence[float]) -> np.ndarray:
        """Return ``[angular, linear]`` at tool origin in the base frame."""
        return self._kinematics.compute_jacobian(joint_position_rad)

    def move_joints(
        self,
        target_position_rad: Sequence[float],
        motion_profile: MotionProfile,
        progress_callback: Optional[ProgressCallback] = None,
    ) -> CommandResult:
        """Generate, fully validate, and execute a canonical trajectory."""
        current_state = self.read_joint_state()
        target = JointTarget(tuple(target_position_rad))
        trajectory = self._trajectory_generator.generate(
            joint_names=self._joint_names,
            start_position_rad=current_state.position_rad,
            target_position_rad=target.position_rad,
            motion_profile=motion_profile,
        )
        return self._trajectory_executor.execute(
            trajectory=trajectory,
            current_state=current_state,
            progress_callback=progress_callback,
        )

    def execute_trajectory(
        self,
        trajectory: JointTrajectory,
        progress_callback: Optional[ProgressCallback] = None,
    ) -> CommandResult:
        """Validate and execute a caller-supplied canonical trajectory."""
        if tuple(trajectory.joint_names) != self._joint_names:
            return CommandResult.rejected(
                "Trajectory joint order does not match the robot model.",
                "JOINT_NAME_MISMATCH",
            )
        return self._trajectory_executor.execute(
            trajectory=trajectory,
            current_state=self.read_joint_state(),
            progress_callback=progress_callback,
        )

    def move_pose(
        self,
        target_pose: RigidTransform,
        motion_profile: MotionProfile,
        seed_joint_position_rad: Optional[Sequence[float]] = None,
        progress_callback: Optional[ProgressCallback] = None,
    ) -> CommandResult:
        """Solve software IK, then execute through the same guarded path."""
        current_state = self.read_joint_state()
        seed = (
            current_state.position_rad
            if seed_joint_position_rad is None
            else tuple(seed_joint_position_rad)
        )
        ik_result = self._kinematics.solve_ik(target_pose, seed)
        if not ik_result.succeeded:
            _LOGGER.warning(
                "move_pose_ik_rejected",
                extra={
                    "position_error_m": ik_result.position_error_m,
                    "orientation_error_rad": ik_result.orientation_error_rad,
                    "iterations": ik_result.iterations,
                },
            )
            return CommandResult.rejected(ik_result.message, "IK_DID_NOT_CONVERGE")
        trajectory = self._trajectory_generator.generate(
            joint_names=self._joint_names,
            start_position_rad=current_state.position_rad,
            target_position_rad=ik_result.joint_position_rad,
            motion_profile=motion_profile,
        )
        return self._trajectory_executor.execute(
            trajectory=trajectory,
            current_state=current_state,
            progress_callback=progress_callback,
        )

    def cancel_current_command(self) -> CommandResult:
        """Cancel the active generation and issue one bounded stop."""
        return self._trajectory_executor.cancel_current_command()

    def stop(self) -> CommandResult:
        """Issue a bounded stop and invalidate the active generation."""
        return self._trajectory_executor.stop_now()

    def recover(self) -> CommandResult:
        """Recover a FAULT only after a valid bounded state read."""
        if self._state_machine.state is not DriverState.FAULT:
            return CommandResult.rejected(
                "Recover is only valid from FAULT.", "INVALID_DRIVER_STATE"
            )
        self._state_machine.apply(
            DriverEvent.RECOVERY_REQUESTED, reason="public recover requested"
        )
        try:
            self.read_joint_state()
        except Exception as error:
            self._state_machine.apply(
                DriverEvent.RECOVERY_FAILED,
                reason="recovery state read failed",
            )
            return CommandResult.failed(
                f"Recovery state read failed: {error}",
                type(error).__name__,
            )
        self._state_machine.apply(
            DriverEvent.RECOVERY_SUCCEEDED,
            reason="bounded state read succeeded",
        )
        return CommandResult.success("Driver recovered to IDLE.")

    def metrics_snapshot(self) -> ExecutionMetrics:
        """Return immutable execution metrics."""
        return self._trajectory_executor.metrics_snapshot()
