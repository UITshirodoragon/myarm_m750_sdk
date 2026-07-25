"""Stable, ROS-independent value objects for the MyArm M750 SDK."""

from __future__ import annotations

import math
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from types import MappingProxyType
from typing import Mapping, Optional, Sequence, Tuple

import numpy as np
from myarm_m750_core.domain.errors import ConfigurationError

JOINT_COUNT = 6


def _joint_tuple(values: Sequence[float], field_name: str) -> Tuple[float, ...]:
    converted = tuple(float(value) for value in values)
    if len(converted) != JOINT_COUNT:
        raise ValueError(
            f"{field_name} must contain exactly {JOINT_COUNT} joint values; "
            f"got {len(converted)}."
        )
    return converted


def _vector_tuple(
    values: Sequence[float], expected_size: int, field_name: str
) -> Tuple[float, ...]:
    converted = tuple(float(value) for value in values)
    if len(converted) != expected_size:
        raise ValueError(
            f"{field_name} must contain exactly {expected_size} values; "
            f"got {len(converted)}."
        )
    return converted


def _require_finite(values: Sequence[float], field_name: str) -> None:
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{field_name} must contain only finite values.")


@dataclass(frozen=True)
class JointState:
    """Measured canonical joint state.

    Args:
        position_rad: Six ROS/canonical joint positions in radians.
        sample_wall_time_s: Source wall-clock timestamp for observation output.
        received_monotonic_s: Local monotonic time used for freshness checks.
        source: Adapter or replay source name.
        sequence: Monotonic sample sequence when available.

    Side effects:
        None.
    """

    position_rad: Tuple[float, ...]
    sample_wall_time_s: float = field(default_factory=time.time)
    received_monotonic_s: float = field(default_factory=time.monotonic)
    source: str = "unknown"
    sequence: int = 0

    def __post_init__(self) -> None:
        position_rad = _joint_tuple(self.position_rad, "position_rad")
        _require_finite(position_rad, "position_rad")
        if not math.isfinite(self.sample_wall_time_s):
            raise ValueError("sample_wall_time_s must be finite.")
        if not math.isfinite(self.received_monotonic_s):
            raise ValueError("received_monotonic_s must be finite.")
        if not isinstance(self.source, str) or not self.source.strip():
            raise ValueError("source must be non-empty.")
        if self.sequence < 0:
            raise ValueError("sequence must be non-negative.")
        object.__setattr__(
            self,
            "position_rad",
            position_rad,
        )

    def age_s(self, now_s: Optional[float] = None) -> float:
        """Return the age of the sample in seconds."""
        effective_now_s = time.monotonic() if now_s is None else float(now_s)
        if not math.isfinite(effective_now_s):
            raise ValueError("now_s must be finite.")
        return max(0.0, effective_now_s - self.received_monotonic_s)

    def is_fresh(self, timeout_s: float, now_s: Optional[float] = None) -> bool:
        """Return whether the sample age is within ``timeout_s``."""
        effective_timeout_s = float(timeout_s)
        if not math.isfinite(effective_timeout_s) or effective_timeout_s < 0.0:
            raise ValueError("timeout_s must be finite and non-negative.")
        return self.age_s(now_s) <= effective_timeout_s


@dataclass(frozen=True)
class JointTarget:
    """Canonical joint-position command in radians."""

    position_rad: Tuple[float, ...]

    def __post_init__(self) -> None:
        position_rad = _joint_tuple(self.position_rad, "position_rad")
        _require_finite(position_rad, "position_rad")
        object.__setattr__(
            self,
            "position_rad",
            position_rad,
        )


@dataclass(frozen=True)
class JointLimits:
    """Lower and upper canonical joint limits in radians."""

    lower_rad: Tuple[float, ...]
    upper_rad: Tuple[float, ...]

    def __post_init__(self) -> None:
        lower_rad = _joint_tuple(self.lower_rad, "lower_rad")
        upper_rad = _joint_tuple(self.upper_rad, "upper_rad")
        if not all(
            math.isfinite(value) for value in lower_rad + upper_rad
        ):
            raise ConfigurationError("Joint limits must contain only finite values.")
        if any(lower >= upper for lower, upper in zip(lower_rad, upper_rad)):
            raise ConfigurationError(
                "Every lower joint limit must be below its upper limit."
            )
        object.__setattr__(self, "lower_rad", lower_rad)
        object.__setattr__(self, "upper_rad", upper_rad)


@dataclass(frozen=True)
class RigidTransform:
    """Rigid transform represented by translation and an XYZW quaternion."""

    translation_m: Tuple[float, ...]
    quaternion_xyzw: Tuple[float, ...]
    parent_frame: str = "base_link"
    child_frame: str = "tool0"
    timestamp_s: float = 0.0

    def __post_init__(self) -> None:
        translation_m = _vector_tuple(self.translation_m, 3, "translation_m")
        quaternion_xyzw = _vector_tuple(self.quaternion_xyzw, 4, "quaternion_xyzw")
        _require_finite(translation_m, "translation_m")
        _require_finite(quaternion_xyzw, "quaternion_xyzw")
        if not math.isfinite(self.timestamp_s):
            raise ValueError("timestamp_s must be finite.")
        quaternion_norm = math.sqrt(sum(value * value for value in quaternion_xyzw))
        if quaternion_norm < 1.0e-12:
            raise ValueError("quaternion_xyzw must have a non-zero norm.")
        normalized = tuple(value / quaternion_norm for value in quaternion_xyzw)
        object.__setattr__(self, "translation_m", translation_m)
        object.__setattr__(self, "quaternion_xyzw", normalized)
        if (
            not isinstance(self.parent_frame, str)
            or not self.parent_frame.strip()
            or not isinstance(self.child_frame, str)
            or not self.child_frame.strip()
        ):
            raise ValueError("parent_frame and child_frame must be non-empty.")
        if self.parent_frame == self.child_frame:
            raise ValueError("parent_frame and child_frame must be distinct.")

    @classmethod
    def from_matrix(
        cls,
        transform_matrix: np.ndarray,
        parent_frame: str = "base_link",
        child_frame: str = "tool0",
        timestamp_s: float = 0.0,
    ) -> RigidTransform:
        """Create a value object from a 4x4 homogeneous transform."""
        from myarm_m750_core.domain.kinematics.math3d import matrix_to_quaternion_xyzw

        matrix = np.asarray(transform_matrix, dtype=float)
        if matrix.shape != (4, 4):
            raise ValueError("transform_matrix must have shape (4, 4).")
        if not bool(np.all(np.isfinite(matrix))):
            raise ValueError("transform_matrix must contain only finite values.")
        if not bool(
            np.allclose(
                matrix[3, :],
                np.asarray((0.0, 0.0, 0.0, 1.0)),
                atol=1.0e-12,
                rtol=0.0,
            )
        ):
            raise ValueError("transform_matrix must have a homogeneous bottom row.")
        rotation_matrix = matrix[:3, :3]
        if not bool(
            np.allclose(
                rotation_matrix.T.dot(rotation_matrix),
                np.eye(3, dtype=float),
                atol=1.0e-9,
                rtol=0.0,
            )
        ) or not math.isclose(
            float(np.linalg.det(rotation_matrix)),
            1.0,
            abs_tol=1.0e-9,
        ):
            raise ValueError("transform_matrix rotation must be in SO(3).")
        return cls(
            translation_m=tuple(matrix[:3, 3]),
            quaternion_xyzw=tuple(matrix_to_quaternion_xyzw(rotation_matrix)),
            parent_frame=parent_frame,
            child_frame=child_frame,
            timestamp_s=timestamp_s,
        )

    def as_matrix(self) -> np.ndarray:
        """Return the transform as a 4x4 homogeneous matrix."""
        from myarm_m750_core.domain.kinematics.math3d import quaternion_xyzw_to_matrix

        transform_matrix = np.eye(4, dtype=float)
        transform_matrix[:3, :3] = quaternion_xyzw_to_matrix(
            np.asarray(self.quaternion_xyzw, dtype=float)
        )
        transform_matrix[:3, 3] = np.asarray(self.translation_m, dtype=float)
        return transform_matrix


@dataclass(frozen=True)
class JointTrajectoryPoint:
    """One unvalidated time-parameterized joint trajectory sample.

    Vector dimensions are structural here. Finite values and derivative limits
    are deliberately checked by ``TrajectoryValidator`` so it can report typed
    safety violations for malformed external trajectories.
    """

    position_rad: Tuple[float, ...]
    time_from_start_s: float
    velocity_rad_s: Optional[Tuple[float, ...]] = None
    acceleration_rad_s2: Optional[Tuple[float, ...]] = None

    def __post_init__(self) -> None:
        if isinstance(self.time_from_start_s, bool):
            raise ValueError("time_from_start_s must be numeric, not boolean.")
        time_from_start_s = float(self.time_from_start_s)
        object.__setattr__(
            self, "position_rad", _joint_tuple(self.position_rad, "position_rad")
        )
        if not math.isfinite(time_from_start_s) or time_from_start_s < 0.0:
            raise ValueError("time_from_start_s must be finite and non-negative.")
        object.__setattr__(self, "time_from_start_s", time_from_start_s)
        if self.velocity_rad_s is not None:
            object.__setattr__(
                self,
                "velocity_rad_s",
                _joint_tuple(self.velocity_rad_s, "velocity_rad_s"),
            )
        if self.acceleration_rad_s2 is not None:
            object.__setattr__(
                self,
                "acceleration_rad_s2",
                _joint_tuple(self.acceleration_rad_s2, "acceleration_rad_s2"),
            )


@dataclass(frozen=True)
class JointTrajectory:
    """Validated sequence of canonical joint positions."""

    joint_names: Tuple[str, ...]
    points: Tuple[JointTrajectoryPoint, ...]
    frame_id: str = "base_link"

    def __post_init__(self) -> None:
        if len(self.joint_names) != JOINT_COUNT:
            raise ValueError("joint_names must contain exactly six names.")
        if len(set(self.joint_names)) != JOINT_COUNT:
            raise ValueError("joint_names must be unique.")
        if not self.points:
            raise ValueError("A joint trajectory must contain at least one point.")
        previous_time_s: Optional[float] = None
        for point in self.points:
            if previous_time_s is not None and point.time_from_start_s <= previous_time_s:
                raise ValueError("Trajectory point times must be strictly increasing.")
            previous_time_s = point.time_from_start_s

    @property
    def duration_s(self) -> float:
        """Return the final point time."""
        return self.points[-1].time_from_start_s


class CommandStatus(Enum):
    """Result status for a public command."""

    SUCCEEDED = "succeeded"
    REJECTED = "rejected"
    CANCELED = "canceled"
    FAILED = "failed"


@dataclass(frozen=True)
class MotionProfile:
    """Time and derivative limits for a generated point-to-point motion."""

    duration_s: float
    max_velocity_rad_s: Optional[float] = None
    max_acceleration_rad_s2: Optional[float] = None

    def __post_init__(self) -> None:
        if not math.isfinite(self.duration_s) or self.duration_s <= 0.0:
            raise ValueError("duration_s must be finite and positive.")
        if self.max_velocity_rad_s is not None and (
            not math.isfinite(self.max_velocity_rad_s)
            or self.max_velocity_rad_s <= 0.0
        ):
            raise ValueError(
                "max_velocity_rad_s must be finite and positive when provided."
            )
        if self.max_acceleration_rad_s2 is not None and (
            not math.isfinite(self.max_acceleration_rad_s2)
            or self.max_acceleration_rad_s2 <= 0.0
        ):
            raise ValueError(
                "max_acceleration_rad_s2 must be finite and positive when provided."
            )


@dataclass(frozen=True)
class CommandContext:
    """Trace context and absolute deadline passed through the hardware boundary."""

    command_id: str
    deadline_monotonic_s: float
    attempt: int = 1
    trajectory_point_index: int = -1

    def __post_init__(self) -> None:
        if not isinstance(self.command_id, str) or not self.command_id.strip():
            raise ValueError("command_id must be non-empty.")
        if isinstance(self.deadline_monotonic_s, bool):
            raise ValueError("deadline_monotonic_s must be numeric, not boolean.")
        if not math.isfinite(self.deadline_monotonic_s):
            raise ValueError("deadline_monotonic_s must be finite.")
        if not isinstance(self.attempt, int) or isinstance(self.attempt, bool):
            raise ValueError("attempt must be an integer.")
        if self.attempt < 1:
            raise ValueError("attempt must be at least one.")
        if not isinstance(self.trajectory_point_index, int) or isinstance(
            self.trajectory_point_index, bool
        ):
            raise ValueError("trajectory_point_index must be an integer.")
        if self.trajectory_point_index < -1:
            raise ValueError("trajectory_point_index must be -1 or greater.")

    @classmethod
    def with_timeout(
        cls,
        timeout_s: float,
        command_id: Optional[str] = None,
        point_index: int = -1,
    ) -> CommandContext:
        """Create a context from a bounded duration using the monotonic clock."""
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("timeout_s must be finite and positive.")
        return cls(
            command_id=command_id or str(uuid.uuid4()),
            deadline_monotonic_s=time.monotonic() + timeout_s,
            trajectory_point_index=point_index,
        )

    def for_attempt(self, attempt: int) -> CommandContext:
        """Return the same command context for a retry attempt."""
        return CommandContext(
            command_id=self.command_id,
            deadline_monotonic_s=self.deadline_monotonic_s,
            attempt=attempt,
            trajectory_point_index=self.trajectory_point_index,
        )


@dataclass(frozen=True)
class CommandResult:
    """Traceable result returned by every public command."""

    status: CommandStatus
    message: str
    command_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    error_code: Optional[str] = None

    def __post_init__(self) -> None:
        if not isinstance(self.status, CommandStatus):
            raise ValueError("status must be a CommandStatus.")
        if not isinstance(self.message, str) or not self.message.strip():
            raise ValueError("message must be non-empty.")
        if not isinstance(self.command_id, str) or not self.command_id.strip():
            raise ValueError("command_id must be non-empty.")
        if self.status is CommandStatus.SUCCEEDED:
            if self.error_code is not None:
                raise ValueError("A successful command cannot contain an error_code.")
        elif not isinstance(self.error_code, str) or not self.error_code.strip():
            raise ValueError("A non-success command must contain an error_code.")

    @property
    def succeeded(self) -> bool:
        """Return whether the command completed successfully."""
        return self.status is CommandStatus.SUCCEEDED

    @classmethod
    def success(cls, message: str, command_id: Optional[str] = None) -> CommandResult:
        """Create a successful result."""
        return cls(
            status=CommandStatus.SUCCEEDED,
            message=message,
            command_id=command_id or str(uuid.uuid4()),
        )

    @classmethod
    def rejected(
        cls, message: str, error_code: str, command_id: Optional[str] = None
    ) -> CommandResult:
        """Create a rejected result."""
        return cls(
            status=CommandStatus.REJECTED,
            message=message,
            error_code=error_code,
            command_id=command_id or str(uuid.uuid4()),
        )

    @classmethod
    def failed(
        cls, message: str, error_code: str, command_id: Optional[str] = None
    ) -> CommandResult:
        """Create a failed result."""
        return cls(
            status=CommandStatus.FAILED,
            message=message,
            error_code=error_code,
            command_id=command_id or str(uuid.uuid4()),
        )

    @classmethod
    def canceled(cls, message: str, command_id: Optional[str] = None) -> CommandResult:
        """Create a canceled result."""
        return cls(
            status=CommandStatus.CANCELED,
            message=message,
            error_code="COMMAND_CANCELED",
            command_id=command_id or str(uuid.uuid4()),
        )


class SafetyViolationType(Enum):
    """Stable categories for safety failures."""

    TRAJECTORY_BUDGET = "trajectory_budget"
    NON_FINITE_VALUE = "non_finite_value"
    STALE_STATE = "stale_state"
    JOINT_LIMIT = "joint_limit"
    JOINT_STEP = "joint_step"
    JOINT_VELOCITY = "joint_velocity"
    JOINT_ACCELERATION = "joint_acceleration"
    WORKSPACE = "workspace"
    SINGULARITY = "singularity"
    TRAJECTORY_TIME = "trajectory_time"


@dataclass(frozen=True)
class SafetyViolation:
    """One concrete safety violation with machine-readable context."""

    violation_type: SafetyViolationType
    message: str
    joint_name: Optional[str] = None
    measured_value: Optional[float] = None
    limit_value: Optional[float] = None


@dataclass(frozen=True)
class ValidationResult:
    """Result of a pure safety validation operation."""

    violations: Tuple[SafetyViolation, ...] = ()

    @property
    def is_valid(self) -> bool:
        """Return whether no violations were found."""
        return not self.violations

    @property
    def first_violation(self) -> Optional[SafetyViolation]:
        """Return the first violation, if any."""
        return self.violations[0] if self.violations else None


@dataclass(frozen=True)
class AdmittedTrajectory:
    """Trajectory proven safe against one state/model/policy snapshot."""

    command_id: str
    trajectory: JointTrajectory
    initial_state: JointState
    admitted_monotonic_s: float
    model_fingerprint: str
    limit_provenance: str

    def __post_init__(self) -> None:
        if not isinstance(self.command_id, str) or not self.command_id.strip():
            raise ValueError("command_id must be non-empty.")
        if not math.isfinite(self.admitted_monotonic_s):
            raise ValueError("admitted_monotonic_s must be finite.")
        if not self.model_fingerprint.strip():
            raise ValueError("model_fingerprint must be non-empty.")
        if not self.limit_provenance.strip():
            raise ValueError("limit_provenance must be non-empty.")


@dataclass(frozen=True)
class IkResult:
    """Numerical inverse-kinematics result."""

    succeeded: bool
    joint_position_rad: Tuple[float, ...]
    iterations: int
    position_error_m: float
    orientation_error_rad: float
    message: str

    def __post_init__(self) -> None:
        joint_position_rad = _joint_tuple(
            self.joint_position_rad,
            "joint_position_rad",
        )
        _require_finite(joint_position_rad, "joint_position_rad")
        if not isinstance(self.succeeded, bool):
            raise ValueError("succeeded must be boolean.")
        if not isinstance(self.iterations, int) or isinstance(self.iterations, bool):
            raise ValueError("iterations must be a non-negative integer.")
        if self.iterations < 0:
            raise ValueError("iterations must be a non-negative integer.")
        errors = (self.position_error_m, self.orientation_error_rad)
        if not all(math.isfinite(value) and value >= 0.0 for value in errors):
            raise ValueError("IK errors must be finite and non-negative.")
        if not isinstance(self.message, str) or not self.message.strip():
            raise ValueError("IK result message must be non-empty.")
        object.__setattr__(
            self,
            "joint_position_rad",
            joint_position_rad,
        )


@dataclass(frozen=True)
class CapabilityState(Enum):
    """Verification state of one backend capability."""

    SUPPORTED = "supported"
    UNSUPPORTED = "unsupported"
    UNVERIFIED = "unverified"


@dataclass(frozen=True)
class AdapterCapabilities:
    """Backend capabilities; unverified features must never be advertised."""

    stop: CapabilityState
    pause: CapabilityState = CapabilityState.UNVERIFIED
    resume: CapabilityState = CapabilityState.UNVERIFIED
    power_control: CapabilityState = CapabilityState.UNVERIFIED

    def __post_init__(self) -> None:
        states = (self.stop, self.pause, self.resume, self.power_control)
        if not all(isinstance(state, CapabilityState) for state in states):
            raise ValueError("Adapter capability values must be CapabilityState.")

    def advertised(self) -> Tuple[str, ...]:
        """Return only capabilities verified as supported."""
        values = {
            "stop": self.stop,
            "pause": self.pause,
            "resume": self.resume,
            "power_control": self.power_control,
        }
        return tuple(
            name for name, state in values.items() if state is CapabilityState.SUPPORTED
        )


@dataclass(frozen=True)
class HardwareIdentity:
    """Observed device identity plus locally verified contract provenance."""

    adapter: str
    model: str
    firmware_version: str
    serial_resource: str
    mapping_fingerprint: str
    capability_verification_reference: str

    def __post_init__(self) -> None:
        fields = {
            "adapter": self.adapter,
            "model": self.model,
            "firmware_version": self.firmware_version,
            "serial_resource": self.serial_resource,
            "mapping_fingerprint": self.mapping_fingerprint,
            "capability_verification_reference": (
                self.capability_verification_reference
            ),
        }
        empty = [
            name
            for name, value in fields.items()
            if not isinstance(value, str) or not value.strip()
        ]
        if empty:
            raise ValueError(
                "Hardware identity fields must be non-empty: " + ", ".join(empty)
            )


@dataclass(frozen=True)
class EnvironmentInspection:
    """Read-only preflight result that never opens the hardware resource."""

    config_source: str
    adapter_type: str
    resources: Mapping[str, str]
    issues: Tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if (
            not isinstance(self.config_source, str)
            or not self.config_source.strip()
            or not isinstance(self.adapter_type, str)
            or not self.adapter_type.strip()
        ):
            raise ValueError(
                "Environment config source and adapter type must be non-empty."
            )
        resources = dict(self.resources)
        if any(
            not isinstance(key, str)
            or not key.strip()
            or not isinstance(value, str)
            for key, value in resources.items()
        ):
            raise ValueError("Environment resources must map names to strings.")
        object.__setattr__(self, "resources", MappingProxyType(resources))
        object.__setattr__(self, "issues", tuple(self.issues))

    @property
    def ready(self) -> bool:
        """Return whether static inspection found no issues."""
        return not self.issues


@dataclass(frozen=True)
class HardwareStatus:
    """Current adapter status for diagnostics."""

    connected: bool
    state: str
    message: str
    protocol_error_count: int = 0
    timeout_count: int = 0
    retry_count: int = 0
    identity: Optional[HardwareIdentity] = None

    def __post_init__(self) -> None:
        if not isinstance(self.state, str) or not self.state.strip():
            raise ValueError("Hardware status state must be non-empty.")
        if not isinstance(self.message, str) or not self.message.strip():
            raise ValueError("Hardware status message must be non-empty.")
        counters = (
            self.protocol_error_count,
            self.timeout_count,
            self.retry_count,
        )
        if any(value < 0 for value in counters):
            raise ValueError("Hardware status counters must be non-negative.")


@dataclass(frozen=True)
class ExecutionMetrics:
    """Immutable command execution metrics snapshot."""

    waypoint_count: int = 0
    overrun_count: int = 0
    stale_state_count: int = 0
    retry_count: int = 0
    operation_latency_s: Tuple[float, ...] = ()
    scheduler_jitter_s: Tuple[float, ...] = ()
    stop_latency_s: Tuple[float, ...] = ()

    def __post_init__(self) -> None:
        counters = (
            self.waypoint_count,
            self.overrun_count,
            self.stale_state_count,
            self.retry_count,
        )
        if any(value < 0 for value in counters):
            raise ValueError("Execution metric counters must be non-negative.")
        operation_latency_s = tuple(float(value) for value in self.operation_latency_s)
        scheduler_jitter_s = tuple(float(value) for value in self.scheduler_jitter_s)
        stop_latency_s = tuple(float(value) for value in self.stop_latency_s)
        _require_finite(operation_latency_s, "operation_latency_s")
        _require_finite(scheduler_jitter_s, "scheduler_jitter_s")
        _require_finite(stop_latency_s, "stop_latency_s")
        if any(value < 0.0 for value in operation_latency_s):
            raise ValueError("operation_latency_s must be non-negative.")
        if any(value < 0.0 for value in stop_latency_s):
            raise ValueError("stop_latency_s must be non-negative.")
        object.__setattr__(self, "operation_latency_s", operation_latency_s)
        object.__setattr__(self, "scheduler_jitter_s", scheduler_jitter_s)
        object.__setattr__(self, "stop_latency_s", stop_latency_s)
