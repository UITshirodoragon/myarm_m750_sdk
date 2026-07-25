"""ROS-independent value objects used inside the driver package."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple


class DriverLifecycleState(Enum):
    """Lifecycle-equivalent states supported on ROS 2 Foxy."""

    UNCONFIGURED = "unconfigured"
    INACTIVE = "inactive"
    ACTIVE = "active"
    FAULT = "fault"


class TrajectoryErrorCode(Enum):
    """``FollowJointTrajectory`` result codes without importing ROS messages."""

    SUCCESSFUL = 0
    INVALID_GOAL = -1
    INVALID_JOINTS = -2
    OLD_HEADER_TIMESTAMP = -3
    PATH_TOLERANCE_VIOLATED = -4
    GOAL_TOLERANCE_VIOLATED = -5


@dataclass(frozen=True)
class CanonicalTrajectoryPoint:
    """One point reordered into canonical model joint order."""

    position_rad: Tuple[float, ...]
    time_from_start_s: float
    velocity_rad_s: Optional[Tuple[float, ...]] = None
    acceleration_rad_s2: Optional[Tuple[float, ...]] = None


@dataclass(frozen=True)
class CanonicalTrajectory:
    """Validated ROS trajectory plus its absolute requested start time."""

    joint_names: Tuple[str, ...]
    points: Tuple[CanonicalTrajectoryPoint, ...]
    start_time_ros_s: Optional[float]

    @property
    def duration_s(self) -> float:
        """Return the final point time in seconds."""
        return self.points[-1].time_from_start_s


@dataclass(frozen=True)
class JointToleranceSet:
    """Position tolerances in canonical joint order."""

    path_position_rad: Tuple[float, ...]
    goal_position_rad: Tuple[float, ...]
    goal_time_tolerance_s: float


@dataclass(frozen=True)
class AcceptedTrajectory:
    """All validated data required by the action coordinator."""

    trajectory: CanonicalTrajectory
    tolerance: JointToleranceSet


class GoalConversionError(ValueError):
    """A goal that cannot be represented by the core trajectory contract."""

    def __init__(self, code: TrajectoryErrorCode, message: str) -> None:
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class CoreJointSample:
    """Joint state returned by the core facade."""

    position_rad: Tuple[float, ...]
    sample_wall_time_s: float
    received_monotonic_s: float
    source: str
    sequence: int


@dataclass(frozen=True)
class CoreHardwareSnapshot:
    """Hardware diagnostics returned by the core facade."""

    connected: bool
    state: str
    message: str
    protocol_error_count: int
    timeout_count: int
    retry_count: int


@dataclass(frozen=True)
class CoreHardwareIdentity:
    """Identity verified during activation without exposing core types."""

    adapter: str
    model: str
    firmware_version: str
    serial_resource: str
    mapping_fingerprint: str
    capability_verification_reference: str


@dataclass(frozen=True)
class CoreAdapterCapabilities:
    """Capability verification states normalized across the core boundary."""

    stop: str
    pause: str
    resume: str
    power_control: str


@dataclass(frozen=True)
class CoreCommandOutcome:
    """Normalized command outcome returned by the core facade."""

    status: str
    message: str
    command_id: str
    error_code: Optional[str]

    @property
    def succeeded(self) -> bool:
        """Return whether the command completed successfully."""
        return self.status == "succeeded"
