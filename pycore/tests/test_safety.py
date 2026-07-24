import time

from myarm_m750_core.domain.models import JointState, JointTarget, SafetyViolationType
from myarm_m750_core.domain.safety import MotionGuard


def test_safety_rejects_large_single_step(sdk_config, kinematics) -> None:
    guard = MotionGuard(sdk_config.robot.joint_names, kinematics, sdk_config.safety)
    current = JointState(position_rad=(0.0,) * 6, timestamp_s=time.time(), source="test")
    result = guard.validate_joint_target(
        JointTarget((0.2, 0.0, 0.0, 0.0, 0.0, 0.0)), current
    )
    assert not result.is_valid
    assert any(
        violation.violation_type is SafetyViolationType.JOINT_STEP
        for violation in result.violations
    )


def test_safety_rejects_joint_limit(sdk_config, kinematics) -> None:
    guard = MotionGuard(sdk_config.robot.joint_names, kinematics, sdk_config.safety)
    current = JointState(position_rad=(0.0,) * 6, timestamp_s=time.time(), source="test")
    result = guard.validate_joint_target(
        JointTarget((3.0, 0.0, 0.0, 0.0, 0.0, 0.0)), current
    )
    assert any(
        violation.violation_type is SafetyViolationType.JOINT_LIMIT
        for violation in result.violations
    )


def test_safety_rejects_stale_state(sdk_config, kinematics) -> None:
    guard = MotionGuard(sdk_config.robot.joint_names, kinematics, sdk_config.safety)
    stale = JointState(
        position_rad=(0.0,) * 6,
        timestamp_s=time.time() - sdk_config.safety.state_timeout_s - 1.0,
        source="test",
    )
    result = guard.validate_joint_target(JointTarget((0.0,) * 6), stale)
    assert any(
        violation.violation_type is SafetyViolationType.STALE_STATE
        for violation in result.violations
    )


def test_safety_rejects_non_finite_target(sdk_config, kinematics) -> None:
    guard = MotionGuard(sdk_config.robot.joint_names, kinematics, sdk_config.safety)
    current = JointState(position_rad=(0.0,) * 6, timestamp_s=time.time(), source="test")
    result = guard.validate_joint_target(
        JointTarget((float("nan"), 0.0, 0.0, 0.0, 0.0, 0.0)), current
    )
    assert result.first_violation.violation_type is SafetyViolationType.NON_FINITE_VALUE
