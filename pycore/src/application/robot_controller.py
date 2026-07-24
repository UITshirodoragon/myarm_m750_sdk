"""Application services used by the public SDK and ROS 2 bridge."""

from __future__ import annotations

import logging
from typing import Callable, Optional, Sequence

import numpy as np

from myarm_m750_core.domain.models import (
    CommandResult,
    HardwareStatus,
    JointState,
    JointTarget,
    JointTrajectory,
    RigidTransform,
    RobotCapabilities,
)
from myarm_m750_core.ports.kinematics import KinematicsPort
from myarm_m750_core.ports.robot_hardware import RobotHardwarePort
from myarm_m750_core.runtime.executor import TrajectoryExecutor
from myarm_m750_core.runtime.state_machine import DriverState, DriverStateMachine
from myarm_m750_core.runtime.trajectory import PointToPointTrajectoryGenerator

_LOGGER = logging.getLogger(__name__)


class RobotController:
    """Coordinate hardware, kinematics, safety, and trajectory services."""

    def __init__(
        self,
        joint_names: Sequence[str],
        hardware: RobotHardwarePort,
        kinematics: KinematicsPort,
        trajectory_generator: PointToPointTrajectoryGenerator,
        trajectory_executor: TrajectoryExecutor,
        state_machine: DriverStateMachine,
    ) -> None:
        self._joint_names = tuple(joint_names)
        self._hardware = hardware
        self._kinematics = kinematics
        self._trajectory_generator = trajectory_generator
        self._trajectory_executor = trajectory_executor
        self._state_machine = state_machine

    @property
    def state(self) -> DriverState:
        """Return the explicit runtime state."""
        return self._state_machine.state

    @property
    def joint_names(self) -> Sequence[str]:
        """Return canonical joint names in model order."""
        return self._joint_names

    def connect(self) -> None:
        """Connect hardware and enter IDLE or FAULT explicitly."""
        self._state_machine.transition_to(DriverState.CONNECTING)
        try:
            self._hardware.connect()
            self._state_machine.transition_to(DriverState.IDLE)
        except Exception:
            self._state_machine.transition_to(DriverState.FAULT)
            raise

    def disconnect(self) -> None:
        """Disconnect hardware from any non-executing stable state."""
        if self._state_machine.state in (DriverState.EXECUTING, DriverState.PAUSED):
            self._trajectory_executor.request_stop()
            self._hardware.stop()
            self._state_machine.transition_to(DriverState.IDLE)
        self._hardware.disconnect()
        if self._state_machine.state is DriverState.FAULT:
            self._state_machine.transition_to(DriverState.DISCONNECTED)
        elif self._state_machine.state is not DriverState.DISCONNECTED:
            self._state_machine.transition_to(DriverState.DISCONNECTED)

    def get_state(self) -> JointState:
        """Read measured canonical joint state from the active adapter."""
        return self._hardware.read_state()

    def get_hardware_status(self) -> HardwareStatus:
        """Return adapter diagnostics without exposing vendor objects."""
        return self._hardware.status()

    def get_capabilities(self) -> RobotCapabilities:
        """Return explicitly supported hardware operations."""
        return self._hardware.capabilities()

    def compute_fk(self, joint_position_rad: Sequence[float]) -> RigidTransform:
        """Compute software FK without reading or commanding hardware."""
        return self._kinematics.compute_fk(joint_position_rad)

    def compute_jacobian(self, joint_position_rad: Sequence[float]) -> np.ndarray:
        """Compute software geometric Jacobian without side effects."""
        return self._kinematics.compute_jacobian(joint_position_rad)

    def move_joints(
        self,
        target_position_rad: Sequence[float],
        duration_s: float,
        cancel_requested: Optional[Callable[[], bool]] = None,
    ) -> CommandResult:
        """Generate, validate, and execute a canonical joint trajectory."""
        current_state = self.get_state()
        target = JointTarget(tuple(target_position_rad))
        trajectory = self._trajectory_generator.generate(
            joint_names=self._joint_names,
            start_position_rad=current_state.position_rad,
            target_position_rad=target.position_rad,
            duration_s=duration_s,
        )
        return self._trajectory_executor.execute(
            trajectory=trajectory,
            current_state=current_state,
            cancel_requested=cancel_requested,
        )

    def execute_trajectory(
        self,
        trajectory: JointTrajectory,
        cancel_requested: Optional[Callable[[], bool]] = None,
    ) -> CommandResult:
        """Validate and execute a caller-supplied canonical trajectory."""
        if tuple(trajectory.joint_names) != self._joint_names:
            return CommandResult.rejected(
                "Trajectory joint order does not match the robot model.",
                "JOINT_NAME_MISMATCH",
            )
        return self._trajectory_executor.execute(
            trajectory=trajectory,
            current_state=self.get_state(),
            cancel_requested=cancel_requested,
        )

    def move_pose(
        self,
        target_pose: RigidTransform,
        duration_s: float,
        seed_joint_position_rad: Optional[Sequence[float]] = None,
        cancel_requested: Optional[Callable[[], bool]] = None,
    ) -> CommandResult:
        """Solve software IK, then execute the resulting joint trajectory."""
        current_state = self.get_state()
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
            return CommandResult.rejected(
                ik_result.message,
                "IK_DID_NOT_CONVERGE",
            )
        trajectory = self._trajectory_generator.generate(
            joint_names=self._joint_names,
            start_position_rad=current_state.position_rad,
            target_position_rad=ik_result.joint_position_rad,
            duration_s=duration_s,
        )
        return self._trajectory_executor.execute(
            trajectory=trajectory,
            current_state=current_state,
            cancel_requested=cancel_requested,
        )

    def stop(self) -> CommandResult:
        """Request an immediate stop and cancel any active trajectory."""
        self._trajectory_executor.request_stop()
        result = self._hardware.stop()
        if self._state_machine.state in (DriverState.EXECUTING, DriverState.PAUSED):
            self._state_machine.transition_to(DriverState.IDLE)
        return result

    def pause(self) -> CommandResult:
        """Pause an idle adapter when the capability is explicitly supported.

        Version 0.1.1 does not pause and later resume a software trajectory.
        Active actions must use cancellation or ``stop()`` so the executor and
        firmware cannot disagree about the next point.
        """
        if self._state_machine.state is DriverState.EXECUTING:
            return CommandResult.rejected(
                "Pause during trajectory execution is not supported in 0.1.1; "
                "cancel the action or call stop().",
                "ACTIVE_TRAJECTORY_PAUSE_UNSUPPORTED",
            )
        if not self._hardware.capabilities().supports_pause:
            return CommandResult.rejected(
                "Active adapter does not support pause.",
                "CAPABILITY_NOT_SUPPORTED",
            )
        result = self._hardware.pause()
        if result.succeeded and self._state_machine.state is DriverState.IDLE:
            self._state_machine.transition_to(DriverState.PAUSED)
        return result

    def resume(self) -> CommandResult:
        """Resume when the active adapter advertises the capability."""
        if not self._hardware.capabilities().supports_resume:
            return CommandResult.rejected(
                "Active adapter does not support resume.",
                "CAPABILITY_NOT_SUPPORTED",
            )
        result = self._hardware.resume()
        if result.succeeded and self._state_machine.state is DriverState.PAUSED:
            self._state_machine.transition_to(DriverState.IDLE)
        return result

    def recover(self) -> CommandResult:
        """Clear a runtime fault after the adapter can read a valid state."""
        if self._state_machine.state is not DriverState.FAULT:
            return CommandResult.rejected(
                "Recover is only valid from FAULT.", "INVALID_DRIVER_STATE"
            )
        self._state_machine.transition_to(DriverState.RECOVERING)
        try:
            self._hardware.read_state()
        except Exception as error:
            self._state_machine.transition_to(DriverState.FAULT)
            return CommandResult.failed(
                "Recovery state read failed: {0}".format(error),
                type(error).__name__,
            )
        self._state_machine.transition_to(DriverState.IDLE)
        return CommandResult.success("Driver recovered to IDLE.")
