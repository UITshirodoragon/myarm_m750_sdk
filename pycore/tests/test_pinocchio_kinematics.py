from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace

import numpy as np
import pytest
from myarm_m750_core.adapters.kinematics import (
    PinocchioKinematics,
    create_kinematics_provider,
)
from myarm_m750_core.domain.errors import KinematicsError
from myarm_m750_core.domain.kinematics import PoeKinematics

pytest.importorskip("pinocchio")

ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_flex_joint",
    "forearm_roll_joint",
    "wrist_flex_joint",
    "wrist_roll_joint",
)


@pytest.fixture(scope="module")
def providers(repository_root):
    urdf_path = (
        repository_root
        / "ros2/src/myarm_m750_description/urdf/generated/myarm_m750_kinematic.urdf"
    )
    poe = PoeKinematics.from_urdf(urdf_path, "base_link", "tool0", ARM_JOINTS)
    pinocchio = PinocchioKinematics(urdf_path, "base_link", "tool0", ARM_JOINTS)
    return poe, pinocchio


def test_pinocchio_and_poe_share_metadata_and_limits(providers) -> None:
    poe, pinocchio = providers
    assert pinocchio.info.provider_version == "2.6.17"
    assert pinocchio.info.model_fingerprint_sha256 == (poe.info.model_fingerprint_sha256)
    assert pinocchio.info.jacobian_order == "angular_linear"
    assert pinocchio.info.jacobian_reference_frame == "base"
    assert pinocchio.info.jacobian_reference_point == "end_link_origin"
    assert not pinocchio.info.dynamics_available
    assert pinocchio.joint_limits == poe.joint_limits


def test_128_seeded_golden_configurations_match_to_1e_9(providers) -> None:
    poe, pinocchio = providers
    lower_rad = np.asarray(poe.joint_limits.lower_rad)
    upper_rad = np.asarray(poe.joint_limits.upper_rad)
    random = np.random.RandomState(750)
    for _ in range(128):
        joint_position_rad = random.uniform(lower_rad * 0.9, upper_rad * 0.9)
        np.testing.assert_allclose(
            pinocchio.compute_fk(joint_position_rad).as_matrix(),
            poe.compute_fk(joint_position_rad).as_matrix(),
            atol=1.0e-9,
            rtol=0.0,
        )
        np.testing.assert_allclose(
            pinocchio.compute_jacobian(joint_position_rad),
            poe.compute_jacobian(joint_position_rad),
            atol=1.0e-9,
            rtol=0.0,
        )


def test_pinocchio_data_is_safe_for_parallel_read_only_evaluation(providers) -> None:
    _poe, pinocchio = providers
    configurations = [
        np.asarray([index * 0.01, -0.2, 0.3, 0.1, -0.1, 0.2]) for index in range(16)
    ]
    expected = [
        pinocchio.compute_fk(configuration).as_matrix() for configuration in configurations
    ]
    with ThreadPoolExecutor(max_workers=4) as executor:
        actual = list(executor.map(pinocchio.compute_fk, configurations))
    for expected_matrix, actual_pose in zip(expected, actual):
        np.testing.assert_allclose(
            actual_pose.as_matrix(), expected_matrix, atol=1.0e-12, rtol=0.0
        )


def test_shared_dls_converges_for_both_backends(providers) -> None:
    poe, pinocchio = providers
    target_joint_position_rad = [0.15, -0.20, 0.25, 0.10, -0.10, 0.05]
    target = poe.compute_fk(target_joint_position_rad)
    for provider in providers:
        result = provider.solve_ik(target, [0.0] * 6)
        assert result.succeeded, result.message
        np.testing.assert_allclose(
            provider.compute_fk(result.joint_position_rad).translation_m,
            target.translation_m,
            atol=1.0e-4,
            rtol=0.0,
        )


def test_shared_dls_rejects_target_frame_mismatch_for_both_backends(
    providers,
) -> None:
    target = providers[0].compute_fk([0.0] * 6)
    for provider in providers:
        with pytest.raises(KinematicsError, match="frame contract mismatch"):
            provider.solve_ik(
                replace(target, parent_frame="map"),
                [0.0] * 6,
            )


def test_factory_falls_back_only_when_pinocchio_import_is_unavailable(
    monkeypatch, repository_root
) -> None:
    import myarm_m750_core.adapters.kinematics.pinocchio_provider as provider_module

    def reject_pinocchio():
        raise provider_module.PinocchioUnavailableError("test import failure")

    monkeypatch.setattr(provider_module, "_load_pinocchio", reject_pinocchio)
    urdf_path = (
        repository_root
        / "ros2/src/myarm_m750_description/urdf/generated/myarm_m750_kinematic.urdf"
    )
    provider = create_kinematics_provider(
        urdf_path=urdf_path,
        base_link="base_link",
        end_link="tool0",
        joint_names=ARM_JOINTS,
    )
    assert isinstance(provider, PoeKinematics)


def test_factory_does_not_hide_pinocchio_model_errors(monkeypatch, repository_root) -> None:
    import myarm_m750_core.adapters.kinematics.pinocchio_provider as provider_module

    def reject_model(**_kwargs):
        raise KinematicsError("model contract mismatch")

    monkeypatch.setattr(provider_module, "PinocchioKinematics", reject_model)
    urdf_path = (
        repository_root
        / "ros2/src/myarm_m750_description/urdf/generated/myarm_m750_kinematic.urdf"
    )
    with pytest.raises(KinematicsError, match="model contract mismatch"):
        create_kinematics_provider(
            urdf_path=urdf_path,
            base_link="base_link",
            end_link="tool0",
            joint_names=ARM_JOINTS,
        )
