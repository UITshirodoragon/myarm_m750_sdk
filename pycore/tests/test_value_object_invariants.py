import math

import numpy as np
import pytest
from myarm_m750_core.domain.camera import (
    CameraExtrinsics,
    CameraMetricsSnapshot,
    CameraReconnectPolicy,
)
from myarm_m750_core.domain.errors import ConfigurationError
from myarm_m750_core.domain.models import (
    AdapterCapabilities,
    AdmittedTrajectory,
    CapabilityState,
    CommandContext,
    CommandResult,
    CommandStatus,
    EnvironmentInspection,
    ExecutionMetrics,
    HardwareIdentity,
    HardwareStatus,
    IkResult,
    JointLimits,
    JointState,
    JointTarget,
    JointTrajectory,
    JointTrajectoryPoint,
    MotionProfile,
    RigidTransform,
)


@pytest.mark.parametrize("invalid_value", [math.nan, math.inf, -math.inf])
def test_joint_state_rejects_nonfinite_measurements_and_clocks(
    invalid_value: float,
) -> None:
    with pytest.raises(ValueError, match="position_rad"):
        JointState(position_rad=(invalid_value, 0.0, 0.0, 0.0, 0.0, 0.0))
    with pytest.raises(ValueError, match="sample_wall_time_s"):
        JointState(position_rad=(0.0,) * 6, sample_wall_time_s=invalid_value)
    with pytest.raises(ValueError, match="received_monotonic_s"):
        JointState(position_rad=(0.0,) * 6, received_monotonic_s=invalid_value)


def test_joint_state_freshness_rejects_invalid_query_values() -> None:
    state = JointState(position_rad=(0.0,) * 6, received_monotonic_s=10.0)
    with pytest.raises(ValueError, match="now_s"):
        state.age_s(math.nan)
    with pytest.raises(ValueError, match="timeout_s"):
        state.is_fresh(math.inf)
    with pytest.raises(ValueError, match="timeout_s"):
        state.is_fresh(-0.1)


@pytest.mark.parametrize("invalid_value", [math.nan, math.inf, -math.inf])
def test_rigid_transform_rejects_nonfinite_components(invalid_value: float) -> None:
    with pytest.raises(ValueError, match="translation_m"):
        RigidTransform(
            translation_m=(invalid_value, 0.0, 0.0),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        )
    with pytest.raises(ValueError, match="quaternion_xyzw"):
        RigidTransform(
            translation_m=(0.0, 0.0, 0.0),
            quaternion_xyzw=(0.0, 0.0, invalid_value, 1.0),
        )
    with pytest.raises(ValueError, match="timestamp_s"):
        RigidTransform(
            translation_m=(0.0, 0.0, 0.0),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            timestamp_s=invalid_value,
        )


def test_rigid_transform_rejects_frame_equivalence_and_non_rigid_matrix() -> None:
    with pytest.raises(ValueError, match="distinct"):
        RigidTransform(
            translation_m=(0.0, 0.0, 0.0),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            parent_frame="tool0",
            child_frame="tool0",
        )

    invalid = np.eye(4, dtype=float)
    invalid[0, 0] = 2.0
    with pytest.raises(ValueError, match=r"SO\(3\)"):
        RigidTransform.from_matrix(invalid)


@pytest.mark.parametrize("invalid_value", [math.nan, math.inf, -math.inf])
def test_motion_profile_rejects_nonfinite_optional_limits(
    invalid_value: float,
) -> None:
    with pytest.raises(ValueError, match="max_velocity_rad_s"):
        MotionProfile(duration_s=1.0, max_velocity_rad_s=invalid_value)
    with pytest.raises(ValueError, match="max_acceleration_rad_s2"):
        MotionProfile(duration_s=1.0, max_acceleration_rad_s2=invalid_value)


def test_trajectory_point_preserves_malformed_vector_for_safety_validator() -> None:
    point = JointTrajectoryPoint(
        position_rad=(math.nan, 0.0, 0.0, 0.0, 0.0, 0.0),
        velocity_rad_s=(math.inf, 0.0, 0.0, 0.0, 0.0, 0.0),
        time_from_start_s=0.0,
    )
    assert math.isnan(point.position_rad[0])
    assert math.isinf(point.velocity_rad_s[0])


def test_command_and_limit_dtos_reject_nonfinite_or_boolean_values() -> None:
    with pytest.raises(ConfigurationError, match="finite"):
        JointLimits(lower_rad=(math.nan,) * 6, upper_rad=(1.0,) * 6)
    with pytest.raises(ValueError, match="position_rad"):
        JointTarget(position_rad=(math.inf,) * 6)
    with pytest.raises(ValueError, match="boolean"):
        JointTrajectoryPoint(position_rad=(0.0,) * 6, time_from_start_s=True)
    with pytest.raises(ValueError, match="deadline_monotonic_s"):
        CommandContext(command_id="command", deadline_monotonic_s=True)
    with pytest.raises(ValueError, match="attempt"):
        CommandContext(
            command_id="command",
            deadline_monotonic_s=1.0,
            attempt=True,
        )
    with pytest.raises(ValueError, match="trajectory_point_index"):
        CommandContext(
            command_id="command",
            deadline_monotonic_s=1.0,
            trajectory_point_index=False,
        )


def test_command_result_enforces_status_and_error_contract() -> None:
    with pytest.raises(ValueError, match="successful"):
        CommandResult(
            status=CommandStatus.SUCCEEDED,
            message="done",
            error_code="SHOULD_NOT_EXIST",
        )
    with pytest.raises(ValueError, match="non-success"):
        CommandResult(status=CommandStatus.FAILED, message="failed")
    with pytest.raises(ValueError, match="CommandStatus"):
        CommandResult(status="failed", message="failed", error_code="FAILED")


def test_admitted_trajectory_and_ik_result_require_traceable_finite_data() -> None:
    state = JointState(position_rad=(0.0,) * 6)
    trajectory = JointTrajectory(
        joint_names=tuple(f"joint_{index}" for index in range(6)),
        points=(
            JointTrajectoryPoint(
                position_rad=(0.0,) * 6,
                time_from_start_s=0.0,
            ),
        ),
    )
    with pytest.raises(ValueError, match="admitted_monotonic_s"):
        AdmittedTrajectory(
            command_id="command",
            trajectory=trajectory,
            initial_state=state,
            admitted_monotonic_s=math.nan,
            model_fingerprint="model",
            limit_provenance="limits",
        )
    with pytest.raises(ValueError, match="joint_position_rad"):
        IkResult(
            succeeded=False,
            joint_position_rad=(math.nan,) * 6,
            iterations=1,
            position_error_m=0.1,
            orientation_error_rad=0.1,
            message="failed",
        )
    with pytest.raises(ValueError, match="IK errors"):
        IkResult(
            succeeded=False,
            joint_position_rad=(0.0,) * 6,
            iterations=1,
            position_error_m=math.inf,
            orientation_error_rad=0.1,
            message="failed",
        )
    with pytest.raises(ValueError, match="CapabilityState"):
        AdapterCapabilities(stop=True)
    assert (
        AdapterCapabilities(stop=CapabilityState.SUPPORTED).advertised()
        == ("stop",)
    )


def test_hardware_dtos_reject_empty_identity_and_negative_counters() -> None:
    with pytest.raises(ValueError, match="firmware_version"):
        HardwareIdentity(
            adapter="mock",
            model="myarm_m750",
            firmware_version="",
            serial_resource="memory://robot",
            mapping_fingerprint="mock-canonical",
            capability_verification_reference="builtin://mock-adapter",
        )
    with pytest.raises(ValueError, match="non-negative"):
        HardwareStatus(
            connected=True,
            state="idle",
            message="test",
            retry_count=-1,
        )
    resources = {"serial": "/dev/serial/by-id/test"}
    inspection = EnvironmentInspection(
        config_source="/tmp/config.yaml",
        adapter_type="vendor_serial",
        resources=resources,
    )
    resources["serial"] = "changed"
    assert inspection.resources["serial"] == "/dev/serial/by-id/test"
    with pytest.raises(TypeError):
        inspection.resources["serial"] = "changed"


def test_execution_metrics_reject_invalid_counts_and_samples() -> None:
    with pytest.raises(ValueError, match="counters"):
        ExecutionMetrics(waypoint_count=-1)
    with pytest.raises(ValueError, match="finite"):
        ExecutionMetrics(scheduler_jitter_s=(math.nan,))
    with pytest.raises(ValueError, match="operation_latency_s"):
        ExecutionMetrics(operation_latency_s=(-0.01,))
    metrics = ExecutionMetrics(scheduler_jitter_s=(-0.01, 0.02))
    assert metrics.scheduler_jitter_s == (-0.01, 0.02)


def test_camera_spatial_timing_and_metric_invariants() -> None:
    with pytest.raises(ValueError, match="distinct"):
        CameraExtrinsics(
            parent_frame="camera",
            child_frame="camera",
            translation_m=(0.0, 0.0, 0.0),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        )
    with pytest.raises(ValueError, match="finite"):
        CameraReconnectPolicy(
            read_timeout_s=math.nan,
            initial_backoff_s=0.1,
            maximum_backoff_s=1.0,
            multiplier=2.0,
            maximum_attempts=2,
        )
    with pytest.raises(ValueError, match="counters"):
        CameraMetricsSnapshot(
            frames_captured=-1,
            read_timeouts=0,
            capture_errors=0,
            reconnect_count=0,
            queue_overflow_count=0,
            last_frame_age_s=0.0,
            last_error="",
        )
    with pytest.raises(ValueError, match="last_frame_age_s"):
        CameraMetricsSnapshot(
            frames_captured=0,
            read_timeouts=0,
            capture_errors=0,
            reconnect_count=0,
            queue_overflow_count=0,
            last_frame_age_s=math.inf,
            last_error="",
        )
