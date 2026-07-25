"""Narrow public robot session API for v0.2.0."""

from __future__ import annotations

import logging
import threading
from typing import Optional, Sequence

import numpy as np
from myarm_m750_core.application.robot_controller import RobotController
from myarm_m750_core.domain.errors import InvalidDriverStateError
from myarm_m750_core.domain.models import (
    AdapterCapabilities,
    CapabilityState,
    CommandResult,
    ExecutionMetrics,
    HardwareIdentity,
    HardwareStatus,
    JointState,
    JointTrajectory,
    MotionProfile,
    RigidTransform,
)
from myarm_m750_core.runtime.config import SdkConfig
from myarm_m750_core.runtime.executor import ProgressCallback
from myarm_m750_core.runtime.state_machine import DriverState

_LOGGER = logging.getLogger(__name__)


class RobotSession:
    """Own one composed controller and its explicit adapter lifecycle."""

    def __init__(self, config: SdkConfig, controller: RobotController) -> None:
        self._config = config
        self._controller = controller
        self._connected = False
        self._hardware_identity: Optional[HardwareIdentity] = None
        self._lifecycle_lock = threading.RLock()

    @property
    def state(self) -> DriverState:
        """Return the explicit runtime state."""
        return self._controller.state

    @property
    def joint_names(self) -> Sequence[str]:
        """Return canonical model joint order."""
        return self._controller.joint_names

    @property
    def config(self) -> SdkConfig:
        """Return the immutable resolved configuration."""
        return self._config

    @property
    def adapter_kind(self) -> str:
        """Return the stable configured adapter discriminator."""
        return self._config.adapter.adapter_type

    def connect(self) -> None:
        """Open the adapter and complete mandatory real-hardware probing."""
        with self._lifecycle_lock:
            if self._connected:
                return
            self._controller.connect()
            try:
                if self.adapter_kind == "vendor_serial":
                    self._hardware_identity = self._controller.probe_hardware()
            except Exception:
                self._controller.disconnect()
                raise
            self._connected = True
            _LOGGER.info("robot_session_connected")

    def close(self) -> None:
        """Bound active ownership and release the adapter exactly once."""
        with self._lifecycle_lock:
            if (
                not self._connected
                and self._controller.state is DriverState.DISCONNECTED
            ):
                return
            try:
                self._controller.disconnect()
            finally:
                if self._controller.state is DriverState.DISCONNECTED:
                    self._connected = False
                    self._hardware_identity = None
                    _LOGGER.info("robot_session_closed")

    def __enter__(self) -> RobotSession:
        self.connect()
        return self

    def __exit__(
        self, exception_type: object, exception: object, traceback: object
    ) -> None:
        del exception_type, exception, traceback
        self.close()

    def read_joint_state(self) -> JointState:
        """Perform one bounded measured-state hardware read."""
        self._require_hardware_ready()
        return self._controller.read_joint_state()

    def read_hardware_status(self) -> HardwareStatus:
        """Return the adapter's local diagnostics snapshot.

        This snapshot remains available while disconnected and does not perform
        a serial query. Use :meth:`read_joint_state` for a measured hardware
        read.
        """
        return self._controller.read_hardware_status()

    def adapter_capabilities(self) -> AdapterCapabilities:
        """Return three-state verified capability metadata."""
        return self._controller.adapter_capabilities()

    def probe_hardware(self) -> HardwareIdentity:
        """Read hardware identity and one state without issuing motion.

        The session must already be connected. Identity, firmware, mapping,
        deadline, and reply validation remain owned by the configured adapter.
        """
        with self._lifecycle_lock:
            if not self._connected:
                raise InvalidDriverStateError(
                    "probe_hardware requires a connected session."
                )
            if self._hardware_identity is None:
                self._hardware_identity = self._controller.probe_hardware()
            return self._hardware_identity

    def compute_fk(self, joint_position_rad: Sequence[float]) -> RigidTransform:
        """Compute side-effect-free FK in the configured frame contract."""
        return self._controller.compute_fk(joint_position_rad)

    def compute_jacobian(self, joint_position_rad: Sequence[float]) -> np.ndarray:
        """Return base-frame ``[angular, linear]`` at the end-link origin."""
        return self._controller.compute_jacobian(joint_position_rad)

    def move_joints(
        self,
        target_position_rad: Sequence[float],
        motion_profile: MotionProfile,
        progress_callback: Optional[ProgressCallback] = None,
    ) -> CommandResult:
        """Generate and execute through admission and safety validation."""
        rejection = self._motion_readiness_rejection()
        if rejection is not None:
            return rejection
        return self._controller.move_joints(
            target_position_rad,
            motion_profile,
            progress_callback=progress_callback,
        )

    def execute_trajectory(
        self,
        trajectory: JointTrajectory,
        progress_callback: Optional[ProgressCallback] = None,
    ) -> CommandResult:
        """Execute a complete canonical trajectory through admission and safety."""
        rejection = self._motion_readiness_rejection()
        if rejection is not None:
            return rejection
        return self._controller.execute_trajectory(
            trajectory, progress_callback=progress_callback
        )

    def move_pose(
        self,
        target_pose: RigidTransform,
        motion_profile: MotionProfile,
        seed_joint_position_rad: Optional[Sequence[float]] = None,
        progress_callback: Optional[ProgressCallback] = None,
    ) -> CommandResult:
        """Solve IK and execute through the same admitted command path."""
        rejection = self._motion_readiness_rejection()
        if rejection is not None:
            return rejection
        return self._controller.move_pose(
            target_pose,
            motion_profile,
            seed_joint_position_rad,
            progress_callback,
        )

    def cancel_current_command(self) -> CommandResult:
        """Cancel the active generation and issue one bounded stop."""
        return self._controller.cancel_current_command()

    def stop(self) -> CommandResult:
        """Invalidate the active generation and issue a bounded stop."""
        self._require_hardware_ready()
        return self._controller.stop()

    def recover(self) -> CommandResult:
        """Recover from FAULT after a valid bounded state read."""
        self._require_hardware_ready()
        return self._controller.recover()

    def metrics_snapshot(self) -> ExecutionMetrics:
        """Return immutable watchdog and execution metrics."""
        return self._controller.metrics_snapshot()

    def _require_hardware_ready(self) -> None:
        """Linearize public hardware I/O after connect and real probe."""
        with self._lifecycle_lock:
            if not self._connected:
                raise InvalidDriverStateError(
                    "Hardware I/O requires a connected, ready session."
                )
            if (
                self.adapter_kind == "vendor_serial"
                and self._hardware_identity is None
            ):
                raise InvalidDriverStateError(
                    "Real hardware I/O requires a completed identity/state probe."
                )

    def _motion_readiness_rejection(self) -> Optional[CommandResult]:
        """Reject real motion unless a probed adapter verifies bounded stop."""
        with self._lifecycle_lock:
            self._require_hardware_ready()
            if self.adapter_kind != "vendor_serial":
                return None
            stop_state = self._controller.adapter_capabilities().stop
            if stop_state is CapabilityState.SUPPORTED:
                return None
            return CommandResult.rejected(
                (
                    "Real-hardware motion requires stop capability verified as "
                    f"supported after probe; observed {stop_state.value}."
                ),
                "STOP_CAPABILITY_NOT_VERIFIED",
            )
