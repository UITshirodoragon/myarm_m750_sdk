"""URDF-driven Product-of-Exponentials kinematics for MyArm M750."""

from __future__ import annotations

import hashlib
import math
import xml.etree.ElementTree as element_tree
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np
from myarm_m750_core.domain.errors import KinematicsError
from myarm_m750_core.domain.kinematics.math3d import (
    adjoint,
    transform_from_xyz_rpy,
    twist_exp,
)
from myarm_m750_core.domain.kinematics.model import fingerprint_urdf_path
from myarm_m750_core.domain.kinematics.solver import (
    DampedLeastSquaresSettings,
    solve_damped_least_squares,
)
from myarm_m750_core.domain.models import IkResult, JointLimits, RigidTransform
from myarm_m750_core.ports.kinematics import KinematicsInfo, KinematicsPort


@dataclass(frozen=True)
class _UrdfJoint:
    name: str
    joint_type: str
    parent_link: str
    child_link: str
    origin_transform: np.ndarray
    axis: np.ndarray
    lower_rad: Optional[float]
    upper_rad: Optional[float]


def _parse_vector(raw_value: Optional[str], default: Sequence[float]) -> np.ndarray:
    if not raw_value:
        return np.asarray(tuple(default), dtype=float)
    values = tuple(float(token) for token in raw_value.split())
    if len(values) != 3:
        raise KinematicsError("URDF vector must contain exactly three values.")
    return np.asarray(values, dtype=float)


def _parse_joint(joint_element: element_tree.Element) -> _UrdfJoint:
    name = str(joint_element.attrib.get("name", ""))
    joint_type = str(joint_element.attrib.get("type", ""))
    parent_element = joint_element.find("parent")
    child_element = joint_element.find("child")
    if not name or parent_element is None or child_element is None:
        raise KinematicsError("Every URDF joint must have name, parent, and child.")
    parent_link = str(parent_element.attrib["link"])
    child_link = str(child_element.attrib["link"])
    origin_element = joint_element.find("origin")
    xyz_m = _parse_vector(
        origin_element.attrib.get("xyz") if origin_element is not None else None,
        (0.0, 0.0, 0.0),
    )
    rpy_rad = _parse_vector(
        origin_element.attrib.get("rpy") if origin_element is not None else None,
        (0.0, 0.0, 0.0),
    )
    axis_element = joint_element.find("axis")
    axis = _parse_vector(
        axis_element.attrib.get("xyz") if axis_element is not None else None,
        (1.0, 0.0, 0.0),
    )
    lower_rad: Optional[float] = None
    upper_rad: Optional[float] = None
    limit_element = joint_element.find("limit")
    if joint_type in ("revolute", "prismatic"):
        if limit_element is None:
            raise KinematicsError(f"Joint '{name}' is missing a limit element.")
        lower_rad = float(limit_element.attrib["lower"])
        upper_rad = float(limit_element.attrib["upper"])
    elif joint_type == "continuous":
        lower_rad = -math.pi
        upper_rad = math.pi
    return _UrdfJoint(
        name=name,
        joint_type=joint_type,
        parent_link=parent_link,
        child_link=child_link,
        origin_transform=transform_from_xyz_rpy(xyz_m, rpy_rad),
        axis=axis,
        lower_rad=lower_rad,
        upper_rad=upper_rad,
    )


class PoeKinematics(KinematicsPort):
    """Pure NumPy kinematics whose geometric source of truth is the URDF.

    The backend intentionally has no ROS 2 or hardware imports. It parses the
    configured serial chain, derives space screw axes, and evaluates PoE FK.
    Numerical IK uses damped least squares and explicit joint-limit clamping.
    """

    def __init__(
        self,
        joint_names: Sequence[str],
        screw_axes_space: np.ndarray,
        home_transform: np.ndarray,
        joint_limits: JointLimits,
        base_link: str,
        end_link: str,
        ik_max_iterations: int = 250,
        ik_damping: float = 0.02,
        ik_max_step_rad: float = 0.20,
        ik_position_tolerance_m: float = 1.0e-4,
        ik_orientation_tolerance_rad: float = 1.0e-3,
        model_fingerprint_sha256: Optional[str] = None,
    ) -> None:
        self._joint_names = tuple(joint_names)
        self._screw_axes_space = np.asarray(screw_axes_space, dtype=float)
        self._home_transform = np.asarray(home_transform, dtype=float)
        self._joint_limits = joint_limits
        self._base_link = base_link
        self._end_link = end_link
        self._ik_settings = DampedLeastSquaresSettings(
            max_iterations=int(ik_max_iterations),
            damping=float(ik_damping),
            max_step_rad=float(ik_max_step_rad),
            position_tolerance_m=float(ik_position_tolerance_m),
            orientation_tolerance_rad=float(ik_orientation_tolerance_rad),
        )
        if self._screw_axes_space.shape != (6, 6):
            raise KinematicsError("Expected six 6D screw axes for the 6R arm.")
        if self._home_transform.shape != (4, 4):
            raise KinematicsError("home_transform must have shape (4, 4).")
        if model_fingerprint_sha256 is None:
            digest = hashlib.sha256()
            digest.update(self._screw_axes_space.tobytes())
            digest.update(self._home_transform.tobytes())
            digest.update(repr(self._joint_limits).encode("utf-8"))
            model_fingerprint_sha256 = digest.hexdigest()
        self._info = KinematicsInfo(
            provider_name="poe",
            provider_version="numpy-reference-v1",
            model_fingerprint_sha256=model_fingerprint_sha256,
            base_link=base_link,
            end_link=end_link,
            joint_names=self._joint_names,
        )

    @classmethod
    def from_urdf(
        cls,
        urdf_path: Path,
        base_link: str,
        end_link: str,
        joint_names: Sequence[str],
    ) -> PoeKinematics:
        """Build screw axes, home pose, and limits from one URDF chain."""
        resolved_path = Path(urdf_path).expanduser().resolve()
        try:
            root = element_tree.parse(str(resolved_path)).getroot()
        except (OSError, element_tree.ParseError) as error:
            raise KinematicsError(
                f"Could not parse URDF {resolved_path}: {error}"
            ) from error

        joints = [_parse_joint(element) for element in root.findall("joint")]
        joint_by_child: Dict[str, _UrdfJoint] = {
            joint.child_link: joint for joint in joints
        }
        path_reverse: List[_UrdfJoint] = []
        current_link = end_link
        visited = set()
        while current_link != base_link:
            if current_link in visited:
                raise KinematicsError("Cycle detected while resolving URDF chain.")
            visited.add(current_link)
            joint = joint_by_child.get(current_link)
            if joint is None:
                raise KinematicsError(
                    f"No URDF chain from '{base_link}' to '{end_link}'. "
                    f"Missing parent of '{current_link}'."
                )
            path_reverse.append(joint)
            current_link = joint.parent_link
        chain = list(reversed(path_reverse))

        current_transform = np.eye(4, dtype=float)
        screw_axes: List[np.ndarray] = []
        lower_limits: List[float] = []
        upper_limits: List[float] = []
        parsed_actuated_names: List[str] = []
        for joint in chain:
            joint_transform = current_transform.dot(joint.origin_transform)
            if joint.joint_type in ("revolute", "continuous"):
                if joint.lower_rad is None or joint.upper_rad is None:
                    raise KinematicsError(
                        f"Joint '{joint.name}' is missing canonical position limits."
                    )
                axis_base = joint_transform[:3, :3].dot(joint.axis)
                axis_norm = float(np.linalg.norm(axis_base))
                if axis_norm < 1.0e-12:
                    raise KinematicsError(f"Joint '{joint.name}' has a zero rotation axis.")
                angular = axis_base / axis_norm
                point_m = joint_transform[:3, 3]
                linear = -np.cross(angular, point_m)
                screw_axes.append(np.concatenate((angular, linear)))
                parsed_actuated_names.append(joint.name)
                lower_limits.append(float(joint.lower_rad))
                upper_limits.append(float(joint.upper_rad))
            elif joint.joint_type == "prismatic":
                if joint.lower_rad is None or joint.upper_rad is None:
                    raise KinematicsError(
                        f"Joint '{joint.name}' is missing canonical position limits."
                    )
                axis_base = joint_transform[:3, :3].dot(joint.axis)
                axis_norm = float(np.linalg.norm(axis_base))
                if axis_norm < 1.0e-12:
                    raise KinematicsError(
                        f"Joint '{joint.name}' has a zero translation axis."
                    )
                screw_axes.append(
                    np.concatenate((np.zeros(3, dtype=float), axis_base / axis_norm))
                )
                parsed_actuated_names.append(joint.name)
                lower_limits.append(float(joint.lower_rad))
                upper_limits.append(float(joint.upper_rad))
            elif joint.joint_type != "fixed":
                raise KinematicsError(
                    f"Unsupported URDF joint type '{joint.joint_type}' "
                    "on the selected chain."
                )
            current_transform = joint_transform

        expected_names = tuple(joint_names)
        if tuple(parsed_actuated_names) != expected_names:
            raise KinematicsError(
                f"URDF actuated chain order {parsed_actuated_names} "
                f"does not match config {list(expected_names)}."
            )
        if len(screw_axes) != 6:
            raise KinematicsError(
                "Selected chain must contain exactly six actuated joints; "
                f"got {len(screw_axes)}."
            )
        screw_matrix = np.column_stack(screw_axes)
        return cls(
            joint_names=expected_names,
            screw_axes_space=screw_matrix,
            home_transform=current_transform,
            joint_limits=JointLimits(
                lower_rad=tuple(lower_limits), upper_rad=tuple(upper_limits)
            ),
            base_link=base_link,
            end_link=end_link,
            model_fingerprint_sha256=fingerprint_urdf_path(resolved_path),
        )

    @property
    def info(self) -> KinematicsInfo:
        """Return reference-backend and normalized model metadata."""
        return self._info

    @property
    def joint_limits(self) -> JointLimits:
        return self._joint_limits

    @property
    def home_transform(self) -> np.ndarray:
        """Return a defensive copy of the zero-configuration transform."""
        return self._home_transform.copy()

    @property
    def screw_axes_space(self) -> np.ndarray:
        """Return a defensive copy of the 6x6 space screw matrix."""
        return self._screw_axes_space.copy()

    @staticmethod
    def _joint_array(joint_position_rad: Sequence[float]) -> np.ndarray:
        joint_array = np.asarray(tuple(joint_position_rad), dtype=float)
        if joint_array.shape != (6,):
            raise ValueError("joint_position_rad must contain six values.")
        if not np.all(np.isfinite(joint_array)):
            raise ValueError("joint_position_rad must contain finite values.")
        return joint_array

    def _fk_matrix(self, joint_position_rad: Sequence[float]) -> np.ndarray:
        joint_array = self._joint_array(joint_position_rad)
        transform_matrix = np.eye(4, dtype=float)
        for joint_index in range(6):
            transform_matrix = transform_matrix.dot(
                twist_exp(self._screw_axes_space[:, joint_index], joint_array[joint_index])
            )
        return transform_matrix.dot(self._home_transform)

    def compute_fk(self, joint_position_rad: Sequence[float]) -> RigidTransform:
        """Compute end-link pose with the space-form PoE equation."""
        return RigidTransform.from_matrix(
            self._fk_matrix(joint_position_rad),
            parent_frame=self._base_link,
            child_frame=self._end_link,
        )

    def _space_jacobian(self, joint_position_rad: Sequence[float]) -> np.ndarray:
        joint_array = self._joint_array(joint_position_rad)
        jacobian = np.zeros((6, 6), dtype=float)
        jacobian[:, 0] = self._screw_axes_space[:, 0]
        transform_matrix = np.eye(4, dtype=float)
        for joint_index in range(1, 6):
            transform_matrix = transform_matrix.dot(
                twist_exp(
                    self._screw_axes_space[:, joint_index - 1],
                    joint_array[joint_index - 1],
                )
            )
            jacobian[:, joint_index] = adjoint(transform_matrix).dot(
                self._screw_axes_space[:, joint_index]
            )
        return jacobian

    def compute_jacobian(self, joint_position_rad: Sequence[float]) -> np.ndarray:
        """Compute geometric Jacobian ``[angular_velocity; point_velocity]``."""
        space_jacobian = self._space_jacobian(joint_position_rad)
        end_position_m = self._fk_matrix(joint_position_rad)[:3, 3]
        geometric_jacobian = space_jacobian.copy()
        for joint_index in range(6):
            angular = space_jacobian[:3, joint_index]
            spatial_linear = space_jacobian[3:, joint_index]
            geometric_jacobian[3:, joint_index] = spatial_linear + np.cross(
                angular, end_position_m
            )
        return geometric_jacobian

    def compute_singularity_score(self, joint_position_rad: Sequence[float]) -> float:
        """Return the minimum singular value of the geometric Jacobian."""
        singular_values = np.linalg.svd(
            self.compute_jacobian(joint_position_rad), compute_uv=False
        )
        return float(np.min(singular_values))

    def solve_ik(
        self,
        target: RigidTransform,
        seed_joint_position_rad: Sequence[float],
    ) -> IkResult:
        """Solve deterministic IK through the shared DLS implementation."""
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
