"""Single adaptation boundary between the ROS driver and Python Core v0.2."""

from __future__ import annotations

from typing import Any, Callable, Optional, Sequence

from myarm_m750_driver.contracts import (
    CanonicalTrajectory,
    CanonicalTrajectoryPoint,
    CoreAdapterCapabilities,
    CoreCommandOutcome,
    CoreHardwareIdentity,
    CoreHardwareSnapshot,
    CoreJointSample,
)

ProgressCallback = Callable[
    [int, CanonicalTrajectoryPoint, CoreJointSample], None
]


class CoreRobotFacade:
    """Expose only the core operations required by the ROS driver."""

    def __init__(self, config_file: str) -> None:
        if not config_file:
            raise ValueError("config_file must be a non-empty absolute path.")
        self._config_file = config_file
        self._session: Optional[Any] = None

    def configure(self) -> None:
        """Build a disconnected core session from a validated v0.2 profile."""
        from myarm_m750_core import RobotSessionBuilder

        self._session = RobotSessionBuilder.from_file(self._config_file).build()

    @property
    def is_configured(self) -> bool:
        """Return whether a core session has been built."""
        return self._session is not None

    @property
    def joint_names(self) -> Sequence[str]:
        """Return canonical model joints."""
        return tuple(self._require_session().joint_names)

    @property
    def adapter_kind(self) -> str:
        """Return the public core adapter-kind identifier."""
        return str(self._require_session().adapter_kind)

    @property
    def runtime_state(self) -> str:
        """Return the core runtime state as a stable lowercase string."""
        state = self._require_session().state
        return str(getattr(state, "value", state))

    @property
    def model_contract_sha256(self) -> str:
        """Return the validated canonical kinematic-contract fingerprint."""
        return str(
            self._require_session().config.robot.kinematic_contract_fingerprint
        )

    @property
    def adapter_capabilities(self) -> CoreAdapterCapabilities:
        """Return normalized three-state adapter capability metadata."""
        capabilities = self._require_session().adapter_capabilities()
        return CoreAdapterCapabilities(
            stop=self._enum_value(capabilities.stop),
            pause=self._enum_value(capabilities.pause),
            resume=self._enum_value(capabilities.resume),
            power_control=self._enum_value(capabilities.power_control),
        )

    def connect(self) -> None:
        """Open the configured adapter."""
        self._require_session().connect()

    def close(self) -> None:
        """Release the configured adapter and session resources."""
        if self._session is not None:
            self._session.close()

    def read_joint_state(self) -> CoreJointSample:
        """Read measured joints without exposing core implementation types."""
        state = self._require_session().read_joint_state()
        return self._normalize_joint_sample(state)

    def read_hardware_status(self) -> CoreHardwareSnapshot:
        """Read hardware diagnostics without exposing vendor objects."""
        status = self._require_session().read_hardware_status()
        return CoreHardwareSnapshot(
            connected=bool(status.connected),
            state=str(status.state),
            message=str(status.message),
            protocol_error_count=int(status.protocol_error_count),
            timeout_count=int(status.timeout_count),
            retry_count=int(status.retry_count),
        )

    def probe_hardware(self) -> CoreHardwareIdentity:
        """Verify identity and one measured state before ROS activation."""
        identity = self._require_session().probe_hardware()
        return CoreHardwareIdentity(
            adapter=str(identity.adapter),
            model=str(identity.model),
            firmware_version=str(identity.firmware_version),
            serial_resource=str(identity.serial_resource),
            mapping_fingerprint=str(identity.mapping_fingerprint),
            capability_verification_reference=str(
                identity.capability_verification_reference
            ),
        )

    def execute_trajectory(
        self,
        trajectory: CanonicalTrajectory,
        progress_callback: Optional[ProgressCallback] = None,
    ) -> CoreCommandOutcome:
        """Execute a canonical trajectory through the core safety path."""
        from myarm_m750_core import JointTrajectory, JointTrajectoryPoint

        core_points = tuple(
            JointTrajectoryPoint(
                position_rad=point.position_rad,
                time_from_start_s=point.time_from_start_s,
                velocity_rad_s=point.velocity_rad_s,
                acceleration_rad_s2=point.acceleration_rad_s2,
            )
            for point in trajectory.points
        )
        core_trajectory = JointTrajectory(
            joint_names=trajectory.joint_names,
            points=core_points,
        )

        def normalize_progress(
            point_index: int, desired: Any, actual: Any
        ) -> None:
            if progress_callback is None:
                return
            progress_callback(
                int(point_index),
                CanonicalTrajectoryPoint(
                    position_rad=tuple(desired.position_rad),
                    time_from_start_s=float(desired.time_from_start_s),
                    velocity_rad_s=(
                        None
                        if desired.velocity_rad_s is None
                        else tuple(desired.velocity_rad_s)
                    ),
                    acceleration_rad_s2=(
                        None
                        if desired.acceleration_rad_s2 is None
                        else tuple(desired.acceleration_rad_s2)
                    ),
                ),
                self._normalize_joint_sample(actual),
            )

        result = self._require_session().execute_trajectory(
            core_trajectory,
            progress_callback=normalize_progress,
        )
        return self._normalize_command_result(result)

    def cancel_current_command(self) -> CoreCommandOutcome:
        """Invalidate queued execution and request bounded adapter stop."""
        result = self._require_session().cancel_current_command()
        return self._normalize_command_result(result)

    def stop(self) -> CoreCommandOutcome:
        """Request an immediate adapter stop."""
        result = self._require_session().stop()
        return self._normalize_command_result(result)

    def recover(self) -> CoreCommandOutcome:
        """Attempt core runtime recovery."""
        result = self._require_session().recover()
        return self._normalize_command_result(result)

    def _require_session(self) -> Any:
        if self._session is None:
            raise RuntimeError("Core session is not configured.")
        return self._session

    @staticmethod
    def _normalize_joint_sample(state: Any) -> CoreJointSample:
        return CoreJointSample(
            position_rad=tuple(state.position_rad),
            sample_wall_time_s=float(state.sample_wall_time_s),
            received_monotonic_s=float(state.received_monotonic_s),
            source=str(state.source),
            sequence=int(state.sequence),
        )

    @staticmethod
    def _normalize_command_result(result: Any) -> CoreCommandOutcome:
        status = getattr(result.status, "value", result.status)
        return CoreCommandOutcome(
            status=str(status),
            message=str(result.message),
            command_id=str(result.command_id),
            error_code=(
                None if result.error_code is None else str(result.error_code)
            ),
        )

    @staticmethod
    def _enum_value(value: Any) -> str:
        return str(getattr(value, "value", value))
