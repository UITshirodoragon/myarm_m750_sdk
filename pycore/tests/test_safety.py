import time
from dataclasses import replace

import pytest
from myarm_m750_core.domain.models import (
    JointState,
    JointTrajectory,
    JointTrajectoryPoint,
    SafetyViolationType,
)
from myarm_m750_core.domain.safety import TrajectoryValidator


def _trajectory(joint_names, target, duration_s=1.0, velocity=None):
    return JointTrajectory(
        joint_names=tuple(joint_names),
        points=(
            JointTrajectoryPoint(
                position_rad=(0.0,) * 6,
                time_from_start_s=0.0,
                velocity_rad_s=(0.0,) * 6,
                acceleration_rad_s2=(0.0,) * 6,
            ),
            JointTrajectoryPoint(
                position_rad=tuple(target),
                time_from_start_s=duration_s,
                velocity_rad_s=velocity,
                acceleration_rad_s2=(0.0,) * 6,
            ),
        ),
    )


def test_validator_rejects_large_single_step(sdk_config, trajectory_validator) -> None:
    current = JointState(position_rad=(0.0,) * 6)
    result = trajectory_validator.validate(
        _trajectory(
            sdk_config.robot.joint_names,
            (0.2, 0.0, 0.0, 0.0, 0.0, 0.0),
        ),
        current,
    )
    assert any(
        violation.violation_type is SafetyViolationType.JOINT_STEP
        for violation in result.violations
    )


def test_validator_rejects_joint_limit(sdk_config, trajectory_validator) -> None:
    result = trajectory_validator.validate(
        _trajectory(
            sdk_config.robot.joint_names,
            (3.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            duration_s=10.0,
        ),
        JointState(position_rad=(0.0,) * 6),
    )
    assert any(
        violation.violation_type is SafetyViolationType.JOINT_LIMIT
        for violation in result.violations
    )


def test_validator_uses_monotonic_freshness(sdk_config, trajectory_validator) -> None:
    stale = JointState(
        position_rad=(0.0,) * 6,
        sample_wall_time_s=time.time() + 1000.0,
        received_monotonic_s=(time.monotonic() - sdk_config.safety.state_timeout_s - 1.0),
    )
    result = trajectory_validator.validate(
        _trajectory(sdk_config.robot.joint_names, (0.0,) * 6), stale
    )
    assert any(
        violation.violation_type is SafetyViolationType.STALE_STATE
        for violation in result.violations
    )


def test_validator_rejects_non_finite_and_velocity(
    sdk_config, trajectory_validator
) -> None:
    result = trajectory_validator.validate(
        _trajectory(
            sdk_config.robot.joint_names,
            (float("nan"), 0.0, 0.0, 0.0, 0.0, 0.0),
            velocity=(10.0,) * 6,
        ),
        JointState(position_rad=(0.0,) * 6),
    )
    kinds = {violation.violation_type for violation in result.violations}
    assert SafetyViolationType.NON_FINITE_VALUE in kinds
    assert SafetyViolationType.JOINT_VELOCITY in kinds


def test_validator_policy_cannot_disable_mandatory_safety(
    trajectory_validator,
) -> None:
    with pytest.raises(ValueError, match="cannot be disabled"):
        replace(trajectory_validator.policy, enabled=False)


def test_validator_rejects_noncanonical_joint_order(
    sdk_config, trajectory_validator
) -> None:
    invalid = _trajectory(
        tuple(reversed(sdk_config.robot.joint_names)),
        (3.0,) * 6,
    )
    result = trajectory_validator.validate(invalid, JointState(position_rad=(0.0,) * 6))
    assert result.first_violation.violation_type is SafetyViolationType.TRAJECTORY_TIME


def test_validator_rejects_spoofed_derivatives_and_time_zero_jump(
    sdk_config, trajectory_validator
) -> None:
    spoofed = JointTrajectory(
        joint_names=tuple(sdk_config.robot.joint_names),
        points=(
            JointTrajectoryPoint(
                position_rad=(0.0,) * 6,
                time_from_start_s=0.0,
                velocity_rad_s=(0.0,) * 6,
                acceleration_rad_s2=(0.0,) * 6,
            ),
            JointTrajectoryPoint(
                position_rad=(0.05, 0.0, 0.0, 0.0, 0.0, 0.0),
                time_from_start_s=0.01,
                velocity_rad_s=(0.0,) * 6,
                acceleration_rad_s2=(0.0,) * 6,
            ),
        ),
    )
    result = trajectory_validator.validate(
        spoofed,
        JointState(position_rad=(0.0,) * 6),
    )
    kinds = {violation.violation_type for violation in result.violations}
    assert SafetyViolationType.JOINT_VELOCITY in kinds
    assert SafetyViolationType.JOINT_ACCELERATION in kinds

    zero_time_jump = JointTrajectory(
        joint_names=tuple(sdk_config.robot.joint_names),
        points=(
            JointTrajectoryPoint(
                position_rad=(0.01, 0.0, 0.0, 0.0, 0.0, 0.0),
                time_from_start_s=0.0,
                velocity_rad_s=(0.0,) * 6,
                acceleration_rad_s2=(0.0,) * 6,
            ),
        ),
    )
    zero_time_result = trajectory_validator.validate(
        zero_time_jump,
        JointState(position_rad=(0.0,) * 6),
    )
    zero_time_kinds = {
        violation.violation_type for violation in zero_time_result.violations
    }
    assert SafetyViolationType.JOINT_VELOCITY in zero_time_kinds
    assert SafetyViolationType.JOINT_ACCELERATION in zero_time_kinds


def test_validator_rejects_nonfinite_supplied_derivatives(
    sdk_config, trajectory_validator
) -> None:
    trajectory = JointTrajectory(
        joint_names=tuple(sdk_config.robot.joint_names),
        points=(
            JointTrajectoryPoint(
                position_rad=(0.0,) * 6,
                time_from_start_s=0.0,
                velocity_rad_s=(float("nan"), 0.0, 0.0, 0.0, 0.0, 0.0),
                acceleration_rad_s2=(float("nan"), 0.0, 0.0, 0.0, 0.0, 0.0),
            ),
        ),
    )
    result = trajectory_validator.validate(
        trajectory,
        JointState(position_rad=(0.0,) * 6),
    )
    kinds = {violation.violation_type for violation in result.violations}
    assert SafetyViolationType.NON_FINITE_VALUE in kinds
    assert SafetyViolationType.JOINT_VELOCITY in kinds
    assert SafetyViolationType.JOINT_ACCELERATION in kinds


@pytest.mark.parametrize(
    ("field_name", "invalid_value"),
    [
        ("max_trajectory_points", 0),
        ("max_workspace_resample_samples", True),
    ],
)
def test_safety_policy_requires_positive_integer_admission_budgets(
    trajectory_validator,
    field_name: str,
    invalid_value,
) -> None:
    with pytest.raises(ValueError, match="positive integers"):
        replace(
            trajectory_validator.policy,
            **{field_name: invalid_value},
        )


@pytest.mark.parametrize(
    ("policy_changes", "expected_message"),
    [
        (
            {"max_trajectory_points": 1},
            "point count",
        ),
        (
            {"max_workspace_resample_samples": 1},
            "workspace resample count",
        ),
    ],
)
def test_admission_budgets_reject_before_any_fk_computation(
    sdk_config,
    trajectory_validator,
    policy_changes,
    expected_message: str,
) -> None:
    class FkMustNotRun:
        def compute_fk(self, _joint_position_rad):
            raise AssertionError("FK must not run after a budget rejection")

    validator = TrajectoryValidator(
        FkMustNotRun(),
        replace(trajectory_validator.policy, **policy_changes),
    )
    result = validator.validate(
        _trajectory(
            sdk_config.robot.joint_names,
            (0.05, 0.0, 0.0, 0.0, 0.0, 0.0),
        ),
        JointState(position_rad=(0.0,) * 6),
    )

    assert result.first_violation is not None
    assert (
        result.first_violation.violation_type
        is SafetyViolationType.TRAJECTORY_BUDGET
    )
    assert expected_message in result.first_violation.message


def test_validator_rejects_acceleration_workspace_and_singularity(
    sdk_config, kinematics, trajectory_validator
) -> None:
    acceleration_trajectory = JointTrajectory(
        joint_names=tuple(sdk_config.robot.joint_names),
        points=(
            JointTrajectoryPoint(
                position_rad=(0.0,) * 6,
                time_from_start_s=0.0,
                velocity_rad_s=(0.0,) * 6,
                acceleration_rad_s2=(100.0,) * 6,
            ),
        ),
    )
    acceleration = trajectory_validator.validate(
        acceleration_trajectory,
        JointState(position_rad=(0.0,) * 6),
    )
    assert any(
        violation.violation_type is SafetyViolationType.JOINT_ACCELERATION
        for violation in acceleration.violations
    )

    outside_validator = TrajectoryValidator(
        kinematics,
        replace(
            trajectory_validator.policy,
            workspace_minimum_m=(10.0, 10.0, 10.0),
            workspace_maximum_m=(11.0, 11.0, 11.0),
            singularity_enabled=False,
        ),
    )
    outside = outside_validator.validate(
        _trajectory(sdk_config.robot.joint_names, (0.0,) * 6),
        JointState(position_rad=(0.0,) * 6),
    )
    assert any(
        violation.violation_type is SafetyViolationType.WORKSPACE
        for violation in outside.violations
    )

    singular_validator = TrajectoryValidator(
        kinematics,
        replace(
            trajectory_validator.policy,
            workspace_minimum_m=(-10.0, -10.0, -10.0),
            workspace_maximum_m=(10.0, 10.0, 10.0),
            singularity_enabled=True,
            minimum_singularity_score=1.0e9,
        ),
    )
    singular = singular_validator.validate(
        _trajectory(sdk_config.robot.joint_names, (0.0,) * 6),
        JointState(position_rad=(0.0,) * 6),
    )
    assert any(
        violation.violation_type is SafetyViolationType.SINGULARITY
        for violation in singular.violations
    )
