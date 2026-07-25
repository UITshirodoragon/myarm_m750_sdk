from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from typing import Sequence

import myarm_m750_core.adapters.kinematics.pinocchio_provider as provider_module
import numpy as np
import pytest
from myarm_m750_core.adapters.kinematics import (
    PinocchioKinematics,
    create_kinematics_provider,
)
from myarm_m750_core.domain.errors import KinematicsError
from myarm_m750_core.domain.kinematics import PoeKinematics

ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_flex_joint",
    "forearm_roll_joint",
    "wrist_flex_joint",
    "wrist_roll_joint",
)


class _Placement:
    def __init__(self, homogeneous: np.ndarray) -> None:
        self.homogeneous = np.asarray(homogeneous, dtype=float)

    @property
    def rotation(self) -> np.ndarray:
        return self.homogeneous[:3, :3]

    def inverse(self) -> "_Placement":
        return _Placement(np.linalg.inv(self.homogeneous))

    def __mul__(self, other: "_Placement") -> "_Placement":
        return _Placement(self.homogeneous.dot(other.homogeneous))


class _FakeData:
    def __init__(self) -> None:
        self.configuration = np.zeros(6, dtype=float)
        self.oMf = [_Placement(np.eye(4)), _Placement(np.eye(4))]


class _FakeModel:
    def __init__(
        self,
        *,
        frame_names: Sequence[str] = ("base_link", "tool0"),
        joint_names: Sequence[str] = ARM_JOINTS,
        q_indices: Sequence[int] = (0, 1, 2, 3, 4, 5),
        invalid_joint_size: bool = False,
    ) -> None:
        self._frame_ids = {
            frame_name: index for index, frame_name in enumerate(frame_names)
        }
        self.nframes = len(frame_names)
        self._joint_ids = {
            joint_name: index for index, joint_name in enumerate(joint_names, start=1)
        }
        self.joints = [SimpleNamespace(nq=0, nv=0, idx_q=0, idx_v=0)]
        for index, q_index in enumerate(q_indices):
            self.joints.append(
                SimpleNamespace(
                    nq=2 if invalid_joint_size and index == 0 else 1,
                    nv=1,
                    idx_q=q_index,
                    idx_v=q_index,
                )
            )
        self.lowerPositionLimit = np.full(6, -1.0)
        self.upperPositionLimit = np.full(6, 1.0)
        self.create_data_count = 0

    def getFrameId(self, frame_name: str) -> int:
        return self._frame_ids.get(frame_name, self.nframes)

    def getJointId(self, joint_name: str) -> int:
        # Pinocchio 2.6.17 uses the one-past-the-end joint id as its
        # unknown-name sentinel.
        return self._joint_ids.get(joint_name, len(self.joints))

    def createData(self) -> _FakeData:
        self.create_data_count += 1
        return _FakeData()


class _FakePinocchio:
    __version__ = "2.6.17"
    ReferenceFrame = SimpleNamespace(LOCAL_WORLD_ALIGNED="local_world_aligned")

    def __init__(
        self,
        model: _FakeModel,
        *,
        build_error: Exception = None,
    ) -> None:
        self.model = model
        self.build_error = build_error
        self.world_jacobian = np.arange(36, dtype=float).reshape(6, 6)

    def buildModelFromUrdf(self, _urdf_path: str) -> _FakeModel:
        if self.build_error is not None:
            raise self.build_error
        return self.model

    @staticmethod
    def neutral(_model: _FakeModel) -> np.ndarray:
        return np.zeros(6, dtype=float)

    @staticmethod
    def forwardKinematics(
        _model: _FakeModel,
        data: _FakeData,
        configuration: np.ndarray,
    ) -> None:
        data.configuration = np.asarray(configuration, dtype=float)

    @staticmethod
    def updateFramePlacements(_model: _FakeModel, data: _FakeData) -> None:
        end_transform = np.eye(4)
        end_transform[0, 3] = float(np.sum(data.configuration))
        data.oMf = [_Placement(np.eye(4)), _Placement(end_transform)]

    def computeFrameJacobian(
        self,
        _model: _FakeModel,
        _data: _FakeData,
        _configuration: np.ndarray,
        _end_frame_id: int,
        _reference_frame: object,
    ) -> np.ndarray:
        return self.world_jacobian.copy()


def _urdf_file(tmp_path: Path) -> Path:
    path = tmp_path / "model.urdf"
    path.write_text('<robot name="fake"/>', encoding="utf-8")
    return path


def _install_fake_pinocchio(monkeypatch, fake: _FakePinocchio) -> None:
    monkeypatch.setattr(
        provider_module.importlib,
        "import_module",
        lambda module_name: (
            fake
            if module_name == "pinocchio"
            else pytest.fail(f"Unexpected import: {module_name}")
        ),
    )


def test_fake_pinocchio_exercises_full_provider_contract(
    monkeypatch, tmp_path: Path
) -> None:
    model = _FakeModel()
    fake = _FakePinocchio(model)
    _install_fake_pinocchio(monkeypatch, fake)
    provider = PinocchioKinematics(
        _urdf_file(tmp_path),
        "base_link",
        "tool0",
        ARM_JOINTS,
    )

    assert provider.info.provider_name == "pinocchio"
    assert provider.info.provider_version == "2.6.17"
    assert not provider.info.dynamics_available
    assert provider.joint_limits.lower_rad == (-1.0,) * 6
    assert provider._data() is provider._data()
    assert model.create_data_count == 1

    joint_position = (0.1, 0.2, 0.0, 0.0, 0.0, 0.0)
    pose = provider.compute_fk(joint_position)
    assert pose.translation_m[0] == pytest.approx(0.3)

    jacobian = provider.compute_jacobian(joint_position)
    expected = np.vstack((fake.world_jacobian[3:, :], fake.world_jacobian[:3, :]))
    np.testing.assert_allclose(jacobian, expected)
    assert provider.compute_singularity_score(joint_position) >= 0.0

    result = provider.solve_ik(pose, joint_position)
    assert result.succeeded
    assert result.iterations == 0
    with pytest.raises(KinematicsError, match="frame contract mismatch"):
        provider.solve_ik(replace(pose, child_frame="camera_link"), joint_position)

    with pytest.raises(ValueError, match="six values"):
        provider.compute_fk((0.0,) * 5)
    with pytest.raises(ValueError, match="finite"):
        provider.compute_jacobian((0.0, 0.0, 0.0, np.nan, 0.0, 0.0))


def test_fake_pinocchio_fails_fast_on_model_contract_mismatch(
    monkeypatch, tmp_path: Path
) -> None:
    urdf_path = _urdf_file(tmp_path)

    _install_fake_pinocchio(
        monkeypatch,
        _FakePinocchio(_FakeModel(), build_error=RuntimeError("parser failure")),
    )
    with pytest.raises(KinematicsError, match="could not build URDF"):
        PinocchioKinematics(urdf_path, "base_link", "tool0", ARM_JOINTS)

    invalid_models = (
        (_FakeModel(frame_names=("base_link",)), ARM_JOINTS, "does not contain frame"),
        (_FakeModel(), ARM_JOINTS[:-1] + ("missing_joint",), "does not contain joint"),
        (
            _FakeModel(invalid_joint_size=True),
            ARM_JOINTS,
            "must have nq=nv=1",
        ),
        (
            _FakeModel(q_indices=(1, 0, 2, 3, 4, 5)),
            ARM_JOINTS,
            "model order",
        ),
    )
    for model, joint_names, message in invalid_models:
        _install_fake_pinocchio(monkeypatch, _FakePinocchio(model))
        with pytest.raises(KinematicsError, match=message):
            PinocchioKinematics(
                urdf_path,
                "base_link",
                "tool0",
                joint_names,
            )

    _install_fake_pinocchio(monkeypatch, _FakePinocchio(_FakeModel()))
    with pytest.raises(KinematicsError, match="six unique joints"):
        PinocchioKinematics(
            urdf_path,
            "base_link",
            "tool0",
            ARM_JOINTS[:-1] + (ARM_JOINTS[0],),
        )


def test_factory_falls_back_only_for_unavailable_pinocchio(
    monkeypatch, repository_root: Path
) -> None:
    def unavailable(**_kwargs):
        raise provider_module.PinocchioUnavailableError("test ABI failure")

    monkeypatch.setattr(provider_module, "PinocchioKinematics", unavailable)
    provider = create_kinematics_provider(
        repository_root / "pycore/src/resources/myarm_m750_kinematic.urdf",
        "base_link",
        "tool0",
        ARM_JOINTS,
    )
    assert isinstance(provider, PoeKinematics)
