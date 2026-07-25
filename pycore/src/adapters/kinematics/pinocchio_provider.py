"""Lazy Pinocchio 2.6.17 kinematics provider for ROS 2 Foxy."""

from __future__ import annotations

import importlib
import threading
from pathlib import Path
from typing import Any, Optional, Sequence, Tuple

import numpy as np
from myarm_m750_core.domain.errors import KinematicsError
from myarm_m750_core.domain.kinematics.model import fingerprint_urdf_path
from myarm_m750_core.domain.kinematics.poe import PoeKinematics
from myarm_m750_core.domain.kinematics.solver import (
    DampedLeastSquaresSettings,
    solve_damped_least_squares,
)
from myarm_m750_core.domain.models import IkResult, JointLimits, RigidTransform
from myarm_m750_core.ports.kinematics import KinematicsInfo, KinematicsPort

_SUPPORTED_PINOCCHIO_VERSION = "2.6.17"


class PinocchioUnavailableError(KinematicsError):
    """Raised only when Pinocchio cannot be imported or loaded."""


def _load_pinocchio() -> Any:
    try:
        pinocchio = importlib.import_module("pinocchio")
    except (ImportError, OSError) as error:
        raise PinocchioUnavailableError(
            "Pinocchio is unavailable. On ROS 2 Foxy ARM64, install the "
            "ros-foxy-pinocchio package (expected version 2.6.17)."
        ) from error
    version = str(getattr(pinocchio, "__version__", "unknown"))
    if version != _SUPPORTED_PINOCCHIO_VERSION:
        raise KinematicsError(
            f"Unsupported Pinocchio version {version}; "
            f"expected {_SUPPORTED_PINOCCHIO_VERSION} for ROS 2 Foxy."
        )
    return pinocchio


class PinocchioKinematics(KinematicsPort):
    """Pinocchio FK/Jacobian with per-thread mutable ``Data`` objects.

    Pinocchio internally orders spatial vectors as ``[linear, angular]``.
    This boundary reorders every Jacobian to the core contract
    ``[angular, linear]`` at the end-link origin, expressed in the base frame.
    """

    def __init__(
        self,
        urdf_path: Path,
        base_link: str,
        end_link: str,
        joint_names: Sequence[str],
        ik_settings: Optional[DampedLeastSquaresSettings] = None,
    ) -> None:
        self._pinocchio = _load_pinocchio()
        self._urdf_path = Path(urdf_path).expanduser().resolve()
        try:
            self._model = self._pinocchio.buildModelFromUrdf(str(self._urdf_path))
        except Exception as error:
            raise KinematicsError(
                f"Pinocchio could not build URDF {self._urdf_path}: {error}"
            ) from error

        self._base_link = str(base_link)
        self._end_link = str(end_link)
        self._joint_names = tuple(str(name) for name in joint_names)
        if len(self._joint_names) != 6 or len(set(self._joint_names)) != 6:
            raise KinematicsError("The MyArm M750 provider requires six unique joints.")
        self._base_frame_id = self._resolve_frame_id(self._base_link)
        self._end_frame_id = self._resolve_frame_id(self._end_link)
        self._joint_q_indices, self._joint_v_indices = self._resolve_joint_indices()
        self._neutral_configuration = np.asarray(
            self._pinocchio.neutral(self._model), dtype=float
        )
        self._joint_limits = JointLimits(
            lower_rad=tuple(
                float(self._model.lowerPositionLimit[index])
                for index in self._joint_q_indices
            ),
            upper_rad=tuple(
                float(self._model.upperPositionLimit[index])
                for index in self._joint_q_indices
            ),
        )
        self._thread_data = threading.local()
        self._ik_settings = ik_settings or DampedLeastSquaresSettings()
        self._info = KinematicsInfo(
            provider_name="pinocchio",
            provider_version=str(self._pinocchio.__version__),
            model_fingerprint_sha256=fingerprint_urdf_path(self._urdf_path),
            base_link=self._base_link,
            end_link=self._end_link,
            joint_names=self._joint_names,
            dynamics_available=False,
        )
        # Fail during composition, not in a later control callback.
        self._configuration([0.0] * len(self._joint_names))
        self._relative_placement([0.0] * len(self._joint_names))

    def _resolve_frame_id(self, frame_name: str) -> int:
        frame_id = int(self._model.getFrameId(frame_name))
        if frame_id >= int(self._model.nframes):
            raise KinematicsError(f"Pinocchio model does not contain frame '{frame_name}'.")
        return frame_id

    def _resolve_joint_indices(self) -> Tuple[Tuple[int, ...], Tuple[int, ...]]:
        q_indices = []
        v_indices = []
        for joint_name in self._joint_names:
            joint_id = int(self._model.getJointId(joint_name))
            # Pinocchio 2.6.17 returns ``model.njoints`` for an unknown name,
            # while some vendor/fake bindings return the universe id (zero).
            # Reject both sentinels before indexing the native vector.
            if joint_id <= 0 or joint_id >= len(self._model.joints):
                raise KinematicsError(
                    f"Pinocchio model does not contain joint '{joint_name}'."
                )
            joint_model = self._model.joints[joint_id]
            if int(joint_model.nq) != 1 or int(joint_model.nv) != 1:
                raise KinematicsError(
                    f"Joint '{joint_name}' must have nq=nv=1; "
                    f"got nq={joint_model.nq}, nv={joint_model.nv}."
                )
            q_indices.append(int(joint_model.idx_q))
            v_indices.append(int(joint_model.idx_v))
        if q_indices != sorted(q_indices):
            raise KinematicsError(
                "Configured joint order does not match the Pinocchio model order."
            )
        return tuple(q_indices), tuple(v_indices)

    def _data(self) -> Any:
        data = getattr(self._thread_data, "value", None)
        if data is None:
            data = self._model.createData()
            self._thread_data.value = data
        return data

    def _joint_array(self, joint_position_rad: Sequence[float]) -> np.ndarray:
        joint_position = np.asarray(tuple(joint_position_rad), dtype=float)
        if joint_position.shape != (len(self._joint_names),):
            raise ValueError("joint_position_rad must contain six values.")
        if not np.all(np.isfinite(joint_position)):
            raise ValueError("joint_position_rad must contain finite values.")
        return joint_position

    def _configuration(self, joint_position_rad: Sequence[float]) -> np.ndarray:
        joint_position = self._joint_array(joint_position_rad)
        configuration = self._neutral_configuration.copy()
        configuration[list(self._joint_q_indices)] = joint_position
        return configuration

    def _relative_placement(self, joint_position_rad: Sequence[float]) -> Any:
        configuration = self._configuration(joint_position_rad)
        data = self._data()
        self._pinocchio.forwardKinematics(self._model, data, configuration)
        self._pinocchio.updateFramePlacements(self._model, data)
        base_placement = data.oMf[self._base_frame_id]
        end_placement = data.oMf[self._end_frame_id]
        return base_placement.inverse() * end_placement

    def _fk_matrix(self, joint_position_rad: Sequence[float]) -> np.ndarray:
        return np.asarray(
            self._relative_placement(joint_position_rad).homogeneous, dtype=float
        )

    @property
    def info(self) -> KinematicsInfo:
        """Return Pinocchio version and normalized model metadata."""
        return self._info

    @property
    def joint_limits(self) -> JointLimits:
        """Return the six canonical arm limits from the loaded URDF."""
        return self._joint_limits

    def compute_fk(self, joint_position_rad: Sequence[float]) -> RigidTransform:
        """Compute the end-link pose relative to the configured base frame."""
        return RigidTransform.from_matrix(
            self._fk_matrix(joint_position_rad),
            parent_frame=self._base_link,
            child_frame=self._end_link,
        )

    def compute_jacobian(self, joint_position_rad: Sequence[float]) -> np.ndarray:
        """Return base-frame ``[angular; linear]`` end-origin Jacobian."""
        configuration = self._configuration(joint_position_rad)
        data = self._data()
        world_jacobian = np.asarray(
            self._pinocchio.computeFrameJacobian(
                self._model,
                data,
                configuration,
                self._end_frame_id,
                self._pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            ),
            dtype=float,
        )
        self._pinocchio.updateFramePlacements(self._model, data)
        base_rotation = np.asarray(data.oMf[self._base_frame_id].rotation, dtype=float)
        selected = world_jacobian[:, list(self._joint_v_indices)]
        linear_base = base_rotation.T.dot(selected[:3, :])
        angular_base = base_rotation.T.dot(selected[3:, :])
        return np.vstack((angular_base, linear_base))

    def compute_singularity_score(self, joint_position_rad: Sequence[float]) -> float:
        """Return the minimum singular value of the canonical Jacobian."""
        singular_values = np.linalg.svd(
            self.compute_jacobian(joint_position_rad), compute_uv=False
        )
        return float(np.min(singular_values))

    def solve_ik(
        self,
        target: RigidTransform,
        seed_joint_position_rad: Sequence[float],
    ) -> IkResult:
        """Solve IK through the same DLS contract used by the PoE provider."""
        return solve_damped_least_squares(
            target=target,
            seed_joint_position_rad=seed_joint_position_rad,
            joint_limits=self._joint_limits,
            compute_fk_matrix=self._fk_matrix,
            compute_jacobian=self.compute_jacobian,
            settings=self._ik_settings,
            expected_parent_frame=self._base_link,
            expected_child_frame=self._end_link,
        )


def create_kinematics_provider(
    urdf_path: Path,
    base_link: str,
    end_link: str,
    joint_names: Sequence[str],
    prefer_pinocchio: bool = True,
) -> KinematicsPort:
    """Create Pinocchio when import succeeds, otherwise the PoE reference.

    Model, frame, joint, and version mismatches deliberately do not trigger a
    fallback because that would hide a deployment error.
    """
    if prefer_pinocchio:
        try:
            return PinocchioKinematics(
                urdf_path=urdf_path,
                base_link=base_link,
                end_link=end_link,
                joint_names=joint_names,
            )
        except PinocchioUnavailableError:
            pass
    return PoeKinematics.from_urdf(
        urdf_path=urdf_path,
        base_link=base_link,
        end_link=end_link,
        joint_names=joint_names,
    )
