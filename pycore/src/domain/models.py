"""Stable value objects shared by applications, adapters, and ROS 2 nodes."""

from __future__ import annotations

import math
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Sequence, Tuple

import numpy as np

from myarm_m750_core.domain.errors import ConfigurationError

JOINT_COUNT = 6


def _joint_tuple(values: Sequence[float], field_name: str) -> Tuple[float, ...]:
    converted = tuple(float(value) for value in values)
    if len(converted) != JOINT_COUNT:
        raise ValueError(
            "{0} must contain exactly {1} joint values; got {2}.".format(
                field_name, JOINT_COUNT, len(converted)
            )
        )
    return converted


def _vector_tuple(
    values: Sequence[float], expected_size: int, field_name: str
) -> Tuple[float, ...]:
    converted = tuple(float(value) for value in values)
    if len(converted) != expected_size:
        raise ValueError(
            "{0} must contain exactly {1} values; got {2}.".format(
                field_name, expected_size, len(converted)
            )
        )
    return converted


@dataclass(frozen=True)
class JointState:
    """Measured canonical joint state.

    Args:
        position_rad: Six ROS/canonical joint positions in radians.
        timestamp_s: Wall-clock timestamp in seconds.
        source: Adapter or replay source name.
        sequence: Monotonic sample sequence when available.

    Side effects:
        None.
    """

    position_rad: Tuple[float, ...]
    timestamp_s: float = field(default_factory=time.time)
    source: str = "unknown"
    sequence: int = 0

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "position_rad", _joint_tuple(self.position_rad, "position_rad")
        )

    def age_s(self, now_s: Optional[float] = None) -> float:
        """Return the age of the sample in seconds."""
        effective_now_s = time.time() if now_s is None else float(now_s)
        return max(0.0, effective_now_s - self.timestamp_s)

    def is_fresh(self, timeout_s: float, now_s: Optional[float] = None) -> bool:
        """Return whether the sample age is within ``timeout_s``."""
        return self.age_s(now_s) <= float(timeout_s)


@dataclass(frozen=True)
class JointTarget:
    """Canonical joint-position command in radians."""

    position_rad: Tuple[float, ...]

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "position_rad", _joint_tuple(self.position_rad, "position_rad")
        )


@dataclass(frozen=True)
class JointLimits:
    """Lower and upper canonical joint limits in radians."""

    lower_rad: Tuple[float, ...]
    upper_rad: Tuple[float, ...]

    def __post_init__(self) -> None:
        lower_rad = _joint_tuple(self.lower_rad, "lower_rad")
        upper_rad = _joint_tuple(self.upper_rad, "upper_rad")
        if any(lower >= upper for lower, upper in zip(lower_rad, upper_rad)):
            raise ConfigurationError("Every lower joint limit must be below its upper limit.")
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
        quaternion_xyzw = _vector_tuple(
            self.quaternion_xyzw, 4, "quaternion_xyzw"
        )
        quaternion_norm = math.sqrt(sum(value * value for value in quaternion_xyzw))
        if quaternion_norm < 1.0e-12:
            raise ValueError("quaternion_xyzw must have a non-zero norm.")
        normalized = tuple(value / quaternion_norm for value in quaternion_xyzw)
        object.__setattr__(self, "translation_m", translation_m)
        object.__setattr__(self, "quaternion_xyzw", normalized)
        if not self.parent_frame or not self.child_frame:
            raise ValueError("parent_frame and child_frame must be non-empty.")

    @classmethod
    def from_matrix(
        cls,
        transform_matrix: np.ndarray,
        parent_frame: str = "base_link",
        child_frame: str = "tool0",
        timestamp_s: float = 0.0,
    ) -> "RigidTransform":
        """Create a value object from a 4x4 homogeneous transform."""
        from myarm_m750_core.domain.kinematics.math3d import matrix_to_quaternion_xyzw

        matrix = np.asarray(transform_matrix, dtype=float)
        if matrix.shape != (4, 4):
            raise ValueError("transform_matrix must have shape (4, 4).")
        return cls(
            translation_m=tuple(matrix[:3, 3]),
            quaternion_xyzw=tuple(matrix_to_quaternion_xyzw(matrix[:3, :3])),
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
    """One time-parameterized joint trajectory sample."""

    position_rad: Tuple[float, ...]
    time_from_start_s: float

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "position_rad", _joint_tuple(self.position_rad, "position_rad")
        )
        if self.time_from_start_s < 0.0:
            raise ValueError("time_from_start_s must be non-negative.")


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
        previous_time_s = -1.0
        for point in self.points:
            if point.time_from_start_s < previous_time_s:
                raise ValueError("Trajectory point times must be monotonic.")
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
class CommandResult:
    """Traceable result returned by every public command."""

    status: CommandStatus
    message: str
    command_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    error_code: Optional[str] = None

    @property
    def succeeded(self) -> bool:
        """Return whether the command completed successfully."""
        return self.status is CommandStatus.SUCCEEDED

    @classmethod
    def success(cls, message: str, command_id: Optional[str] = None) -> "CommandResult":
        """Create a successful result."""
        return cls(
            status=CommandStatus.SUCCEEDED,
            message=message,
            command_id=command_id or str(uuid.uuid4()),
        )

    @classmethod
    def rejected(
        cls, message: str, error_code: str, command_id: Optional[str] = None
    ) -> "CommandResult":
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
    ) -> "CommandResult":
        """Create a failed result."""
        return cls(
            status=CommandStatus.FAILED,
            message=message,
            error_code=error_code,
            command_id=command_id or str(uuid.uuid4()),
        )

    @classmethod
    def canceled(
        cls, message: str, command_id: Optional[str] = None
    ) -> "CommandResult":
        """Create a canceled result."""
        return cls(
            status=CommandStatus.CANCELED,
            message=message,
            error_code="COMMAND_CANCELED",
            command_id=command_id or str(uuid.uuid4()),
        )


class SafetyViolationType(Enum):
    """Stable categories for safety failures."""

    NON_FINITE_VALUE = "non_finite_value"
    STALE_STATE = "stale_state"
    JOINT_LIMIT = "joint_limit"
    JOINT_STEP = "joint_step"
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
class IkResult:
    """Numerical inverse-kinematics result."""

    succeeded: bool
    joint_position_rad: Tuple[float, ...]
    iterations: int
    position_error_m: float
    orientation_error_rad: float
    message: str

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "joint_position_rad",
            _joint_tuple(self.joint_position_rad, "joint_position_rad"),
        )


@dataclass(frozen=True)
class RobotCapabilities:
    """Hardware features exposed without assuming unsupported behavior."""

    supports_pause: bool
    supports_resume: bool
    supports_stop: bool
    supports_power_control: bool = False


@dataclass(frozen=True)
class HardwareStatus:
    """Current adapter status for diagnostics."""

    connected: bool
    state: str
    message: str
    protocol_error_count: int = 0
    timeout_count: int = 0
