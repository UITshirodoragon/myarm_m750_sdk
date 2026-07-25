"""Pure conversion and validation for ``FollowJointTrajectory`` goals."""

from __future__ import annotations

import math
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from myarm_m750_driver.contracts import (
    AcceptedTrajectory,
    CanonicalTrajectory,
    CanonicalTrajectoryPoint,
    GoalConversionError,
    JointToleranceSet,
    TrajectoryErrorCode,
)

_NANOSECONDS_PER_SECOND = 1_000_000_000.0
_UNSPECIFIED_TOLERANCE = 0.0
_ERASE_TOLERANCE = -1.0


def duration_to_seconds(duration: Any) -> float:
    """Convert a ROS-like duration object to seconds without side effects."""
    return float(duration.sec) + float(duration.nanosec) / _NANOSECONDS_PER_SECOND


def stamp_to_seconds(stamp: Any) -> float:
    """Convert a ROS-like time object to seconds without side effects."""
    return float(stamp.sec) + float(stamp.nanosec) / _NANOSECONDS_PER_SECOND


def convert_follow_joint_trajectory_goal(
    goal: Any,
    canonical_joint_names: Sequence[str],
    maximum_trajectory_points: int,
    now_ros_s: float,
    default_path_tolerance_rad: float,
    default_goal_tolerance_rad: float,
    default_goal_time_tolerance_s: float,
    old_header_tolerance_s: float,
) -> AcceptedTrajectory:
    """Validate and reorder one ROS-like action goal.

    Args:
        goal: Object with the fields of ``FollowJointTrajectory.Goal``.
        canonical_joint_names: Model joint names in core order.
        maximum_trajectory_points: Maximum bounded ROS point count.
        now_ros_s: Current ROS clock time in seconds.
        default_path_tolerance_rad: Position path tolerance for unspecified joints.
        default_goal_tolerance_rad: Position goal tolerance for unspecified joints.
        default_goal_time_tolerance_s: Extra settling time when unspecified.
        old_header_tolerance_s: Allowed transport delay for a non-zero header stamp.

    Returns:
        A canonical trajectory and resolved tolerance values.

    Raises:
        GoalConversionError: The goal violates the ROS/core boundary contract.

    Side effects:
        None.
    """
    _validate_non_negative_finite(
        default_path_tolerance_rad, "default_path_tolerance_rad"
    )
    if (
        isinstance(maximum_trajectory_points, bool)
        or not isinstance(maximum_trajectory_points, int)
        or maximum_trajectory_points <= 0
    ):
        raise ValueError("maximum_trajectory_points must be a positive integer.")
    point_count = len(goal.trajectory.points)
    if point_count > maximum_trajectory_points:
        raise GoalConversionError(
            TrajectoryErrorCode.INVALID_GOAL,
            "Trajectory contains "
            f"{point_count} points; maximum is {maximum_trajectory_points}.",
        )
    _validate_non_negative_finite(
        default_goal_tolerance_rad, "default_goal_tolerance_rad"
    )
    _validate_non_negative_finite(
        default_goal_time_tolerance_s, "default_goal_time_tolerance_s"
    )
    _validate_non_negative_finite(old_header_tolerance_s, "old_header_tolerance_s")

    canonical_names = tuple(canonical_joint_names)
    source_names = tuple(str(name) for name in goal.trajectory.joint_names)
    reorder_indices = _compute_reorder_indices(source_names, canonical_names)
    points = _convert_points(
        goal.trajectory.points,
        reorder_indices,
        len(canonical_names),
    )
    start_time_ros_s = _resolve_start_time(
        goal.trajectory.header.stamp,
        now_ros_s=now_ros_s,
        old_header_tolerance_s=old_header_tolerance_s,
    )
    tolerance = JointToleranceSet(
        path_position_rad=_resolve_position_tolerances(
            goal.path_tolerance,
            canonical_names,
            default_path_tolerance_rad,
            "path_tolerance",
        ),
        goal_position_rad=_resolve_position_tolerances(
            goal.goal_tolerance,
            canonical_names,
            default_goal_tolerance_rad,
            "goal_tolerance",
        ),
        goal_time_tolerance_s=_resolve_goal_time_tolerance(
            goal.goal_time_tolerance,
            default_goal_time_tolerance_s,
        ),
    )
    return AcceptedTrajectory(
        trajectory=CanonicalTrajectory(
            joint_names=canonical_names,
            points=points,
            start_time_ros_s=start_time_ros_s,
        ),
        tolerance=tolerance,
    )


def interpolate_desired_position(
    trajectory: CanonicalTrajectory, elapsed_s: float
) -> Tuple[float, ...]:
    """Linearly interpolate desired positions at an elapsed trajectory time."""
    if elapsed_s <= trajectory.points[0].time_from_start_s:
        return trajectory.points[0].position_rad
    for previous, following in zip(trajectory.points, trajectory.points[1:]):
        if elapsed_s <= following.time_from_start_s:
            interval_s = following.time_from_start_s - previous.time_from_start_s
            if interval_s <= 0.0:
                return following.position_rad
            ratio = (elapsed_s - previous.time_from_start_s) / interval_s
            return tuple(
                start + ratio * (end - start)
                for start, end in zip(
                    previous.position_rad, following.position_rad
                )
            )
    return trajectory.points[-1].position_rad


def violated_joint_names(
    actual_position_rad: Sequence[float],
    desired_position_rad: Sequence[float],
    tolerance_rad: Sequence[float],
    joint_names: Sequence[str],
) -> Tuple[str, ...]:
    """Return joints whose absolute position error exceeds tolerance."""
    return tuple(
        name
        for name, actual, desired, tolerance in zip(
            joint_names,
            actual_position_rad,
            desired_position_rad,
            tolerance_rad,
        )
        if math.isfinite(tolerance) and abs(actual - desired) > tolerance
    )


def _compute_reorder_indices(
    source_names: Tuple[str, ...], canonical_names: Tuple[str, ...]
) -> Tuple[int, ...]:
    if not source_names:
        raise GoalConversionError(
            TrajectoryErrorCode.INVALID_JOINTS,
            "trajectory.joint_names must not be empty.",
        )
    if len(set(source_names)) != len(source_names):
        raise GoalConversionError(
            TrajectoryErrorCode.INVALID_JOINTS,
            "trajectory.joint_names contains duplicate names.",
        )
    missing = sorted(set(canonical_names) - set(source_names))
    unknown = sorted(set(source_names) - set(canonical_names))
    if missing or unknown or len(source_names) != len(canonical_names):
        raise GoalConversionError(
            TrajectoryErrorCode.INVALID_JOINTS,
            f"Joint set mismatch; missing={missing}, unknown={unknown}.",
        )
    source_index = {name: index for index, name in enumerate(source_names)}
    return tuple(source_index[name] for name in canonical_names)


def _convert_points(
    ros_points: Iterable[Any],
    reorder_indices: Tuple[int, ...],
    joint_count: int,
) -> Tuple[CanonicalTrajectoryPoint, ...]:
    converted: List[CanonicalTrajectoryPoint] = []
    previous_time_s = -1.0
    for point_index, point in enumerate(ros_points):
        positions = _validate_vector(
            point.positions, joint_count, f"points[{point_index}].positions"
        )
        velocities = _validate_optional_vector(
            point.velocities,
            joint_count,
            f"points[{point_index}].velocities",
        )
        accelerations = _validate_optional_vector(
            point.accelerations,
            joint_count,
            f"points[{point_index}].accelerations",
        )
        if point.effort:
            raise GoalConversionError(
                TrajectoryErrorCode.INVALID_GOAL,
                f"points[{point_index}].effort is unsupported.",
            )
        time_from_start_s = duration_to_seconds(point.time_from_start)
        if not math.isfinite(time_from_start_s) or time_from_start_s < 0.0:
            raise GoalConversionError(
                TrajectoryErrorCode.INVALID_GOAL,
                f"points[{point_index}].time_from_start must be finite and non-negative.",
            )
        if point_index > 0 and time_from_start_s <= previous_time_s:
            raise GoalConversionError(
                TrajectoryErrorCode.INVALID_GOAL,
                "Trajectory point times must be strictly increasing.",
            )
        previous_time_s = time_from_start_s
        converted.append(
            CanonicalTrajectoryPoint(
                position_rad=_reorder(positions, reorder_indices),
                time_from_start_s=time_from_start_s,
                velocity_rad_s=(
                    None if velocities is None else _reorder(velocities, reorder_indices)
                ),
                acceleration_rad_s2=(
                    None
                    if accelerations is None
                    else _reorder(accelerations, reorder_indices)
                ),
            )
        )
    if not converted:
        raise GoalConversionError(
            TrajectoryErrorCode.INVALID_GOAL,
            "trajectory.points must contain at least one point.",
        )
    return tuple(converted)


def _resolve_start_time(
    stamp: Any, now_ros_s: float, old_header_tolerance_s: float
) -> Optional[float]:
    start_time_ros_s = stamp_to_seconds(stamp)
    if start_time_ros_s == 0.0:
        return None
    if not math.isfinite(start_time_ros_s) or start_time_ros_s < 0.0:
        raise GoalConversionError(
            TrajectoryErrorCode.INVALID_GOAL,
            "trajectory.header.stamp must be zero or a valid ROS time.",
        )
    if start_time_ros_s < now_ros_s - old_header_tolerance_s:
        raise GoalConversionError(
            TrajectoryErrorCode.OLD_HEADER_TIMESTAMP,
            "Trajectory header timestamp is older than the configured tolerance.",
        )
    return start_time_ros_s


def _resolve_position_tolerances(
    ros_tolerances: Iterable[Any],
    canonical_names: Tuple[str, ...],
    default_tolerance_rad: float,
    field_name: str,
) -> Tuple[float, ...]:
    resolved: Dict[str, float] = {
        name: default_tolerance_rad for name in canonical_names
    }
    seen = set()
    for tolerance in ros_tolerances:
        name = str(tolerance.name)
        if name not in resolved:
            raise GoalConversionError(
                TrajectoryErrorCode.INVALID_JOINTS,
                f"{field_name} contains unknown joint '{name}'.",
            )
        if name in seen:
            raise GoalConversionError(
                TrajectoryErrorCode.INVALID_GOAL,
                f"{field_name} contains duplicate joint '{name}'.",
            )
        seen.add(name)
        if float(tolerance.velocity) != _UNSPECIFIED_TOLERANCE:
            raise GoalConversionError(
                TrajectoryErrorCode.INVALID_GOAL,
                f"{field_name} velocity tolerance is unsupported.",
            )
        if float(tolerance.acceleration) != _UNSPECIFIED_TOLERANCE:
            raise GoalConversionError(
                TrajectoryErrorCode.INVALID_GOAL,
                f"{field_name} acceleration tolerance is unsupported.",
            )
        resolved[name] = _resolve_one_tolerance(
            float(tolerance.position), default_tolerance_rad, field_name
        )
    return tuple(resolved[name] for name in canonical_names)


def _resolve_one_tolerance(
    value: float, default_value: float, field_name: str
) -> float:
    if value == _UNSPECIFIED_TOLERANCE:
        return default_value
    if value == _ERASE_TOLERANCE:
        return math.inf
    if not math.isfinite(value) or value < 0.0:
        raise GoalConversionError(
            TrajectoryErrorCode.INVALID_GOAL,
            f"{field_name} must be positive, zero (default), or -1 (erase).",
        )
    return value


def _resolve_goal_time_tolerance(
    duration: Any, default_goal_time_tolerance_s: float
) -> float:
    value_s = duration_to_seconds(duration)
    if value_s == 0.0:
        return default_goal_time_tolerance_s
    if not math.isfinite(value_s) or value_s < 0.0:
        raise GoalConversionError(
            TrajectoryErrorCode.INVALID_GOAL,
            "goal_time_tolerance must be finite and non-negative.",
        )
    return value_s


def _validate_vector(
    values: Iterable[float], expected_size: int, field_name: str
) -> Tuple[float, ...]:
    converted = tuple(float(value) for value in values)
    if len(converted) != expected_size:
        raise GoalConversionError(
            TrajectoryErrorCode.INVALID_GOAL,
            f"{field_name} must contain {expected_size} values; got {len(converted)}.",
        )
    if not all(math.isfinite(value) for value in converted):
        raise GoalConversionError(
            TrajectoryErrorCode.INVALID_GOAL,
            f"{field_name} contains a non-finite value.",
        )
    return converted


def _validate_optional_vector(
    values: Iterable[float], expected_size: int, field_name: str
) -> Optional[Tuple[float, ...]]:
    converted = tuple(float(value) for value in values)
    if not converted:
        return None
    return _validate_vector(converted, expected_size, field_name)


def _reorder(
    values: Tuple[float, ...], reorder_indices: Tuple[int, ...]
) -> Tuple[float, ...]:
    return tuple(values[index] for index in reorder_indices)


def _validate_non_negative_finite(value: float, field_name: str) -> None:
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"{field_name} must be finite and non-negative.")
