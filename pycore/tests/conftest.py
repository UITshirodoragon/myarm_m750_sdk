from pathlib import Path

import pytest
from myarm_m750_core.domain.kinematics import PoeKinematics
from myarm_m750_core.domain.safety import SafetyPolicy, TrajectoryValidator
from myarm_m750_core.runtime.config import load_sdk_config


@pytest.fixture(scope="session")
def repository_root() -> Path:
    return Path(__file__).resolve().parents[2]


@pytest.fixture(scope="session")
def sdk_config(repository_root: Path):
    return load_sdk_config(str(repository_root / "pycore" / "config" / "default.yaml"))


@pytest.fixture(scope="session")
def kinematics(sdk_config):
    return PoeKinematics.from_urdf(
        urdf_path=sdk_config.robot.urdf_path,
        base_link=sdk_config.robot.base_link,
        end_link=sdk_config.robot.end_link,
        joint_names=sdk_config.robot.joint_names,
    )


@pytest.fixture
def trajectory_validator(sdk_config, kinematics):
    safety = sdk_config.safety
    policy = SafetyPolicy(
        enabled=safety.enabled,
        joint_names=sdk_config.robot.joint_names,
        joint_limits=kinematics.joint_limits,
        max_trajectory_points=safety.max_trajectory_points,
        max_workspace_resample_samples=safety.max_workspace_resample_samples,
        state_timeout_s=safety.state_timeout_s,
        command_timeout_s=safety.command_timeout_s,
        stop_timeout_s=safety.stop_timeout_s,
        max_joint_step_rad=safety.max_joint_step_rad,
        max_joint_velocity_rad_s=safety.max_joint_velocity_rad_s,
        max_joint_acceleration_rad_s2=safety.max_joint_acceleration_rad_s2,
        joint_limit_margin_rad=safety.joint_limit_margin_rad,
        workspace_minimum_m=safety.workspace.minimum_m,
        workspace_maximum_m=safety.workspace.maximum_m,
        workspace_resample_step_rad=safety.workspace.resample_step_rad,
        singularity_enabled=safety.singularity.enabled,
        minimum_singularity_score=safety.singularity.minimum_score,
        model_fingerprint=sdk_config.robot.kinematic_contract_fingerprint,
        limit_provenance=safety.provenance,
    )
    return TrajectoryValidator(kinematics, policy)
