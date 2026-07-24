"""URDF-driven Product-of-Exponentials kinematics for MyArm M750."""

from __future__ import annotations

import logging
import math
import xml.etree.ElementTree as element_tree
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from myarm_m750_core.domain.errors import KinematicsError
from myarm_m750_core.domain.models import IkResult, JointLimits, RigidTransform
from myarm_m750_core.domain.kinematics.math3d import (
    adjoint,
    rotation_log_vector,
    transform_from_xyz_rpy,
    twist_exp,
)
from myarm_m750_core.ports.kinematics import KinematicsPort

_LOGGER = logging.getLogger(__name__)


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
            raise KinematicsError("Joint '{0}' is missing a limit element.".format(name))
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
    ) -> None:
        self._joint_names = tuple(joint_names)
        self._screw_axes_space = np.asarray(screw_axes_space, dtype=float)
        self._home_transform = np.asarray(home_transform, dtype=float)
        self._joint_limits = joint_limits
        self._base_link = base_link
        self._end_link = end_link
        self._ik_max_iterations = int(ik_max_iterations)
        self._ik_damping = float(ik_damping)
        self._ik_max_step_rad = float(ik_max_step_rad)
        self._ik_position_tolerance_m = float(ik_position_tolerance_m)
        self._ik_orientation_tolerance_rad = float(ik_orientation_tolerance_rad)
        if self._screw_axes_space.shape != (6, 6):
            raise KinematicsError("Expected six 6D screw axes for the 6R arm.")
        if self._home_transform.shape != (4, 4):
            raise KinematicsError("home_transform must have shape (4, 4).")

    @classmethod
    def from_urdf(
        cls,
        urdf_path: Path,
        base_link: str,
        end_link: str,
        joint_names: Sequence[str],
    ) -> "PoeKinematics":
        """Build screw axes, home pose, and limits from one URDF chain."""
        resolved_path = Path(urdf_path).expanduser().resolve()
        try:
            root = element_tree.parse(str(resolved_path)).getroot()
        except (OSError, element_tree.ParseError) as error:
            raise KinematicsError(
                "Could not parse URDF {0}: {1}".format(resolved_path, error)
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
                    "No URDF chain from '{0}' to '{1}'. Missing parent of '{2}'.".format(
                        base_link, end_link, current_link
                    )
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
                axis_base = joint_transform[:3, :3].dot(joint.axis)
                axis_norm = float(np.linalg.norm(axis_base))
                if axis_norm < 1.0e-12:
                    raise KinematicsError(
                        "Joint '{0}' has a zero rotation axis.".format(joint.name)
                    )
                angular = axis_base / axis_norm
                point_m = joint_transform[:3, 3]
                linear = -np.cross(angular, point_m)
                screw_axes.append(np.concatenate((angular, linear)))
                parsed_actuated_names.append(joint.name)
                lower_limits.append(float(joint.lower_rad))
                upper_limits.append(float(joint.upper_rad))
            elif joint.joint_type == "prismatic":
                axis_base = joint_transform[:3, :3].dot(joint.axis)
                axis_norm = float(np.linalg.norm(axis_base))
                if axis_norm < 1.0e-12:
                    raise KinematicsError(
                        "Joint '{0}' has a zero translation axis.".format(joint.name)
                    )
                screw_axes.append(
                    np.concatenate((np.zeros(3, dtype=float), axis_base / axis_norm))
                )
                parsed_actuated_names.append(joint.name)
                lower_limits.append(float(joint.lower_rad))
                upper_limits.append(float(joint.upper_rad))
            elif joint.joint_type != "fixed":
                raise KinematicsError(
                    "Unsupported URDF joint type '{0}' on the selected chain.".format(
                        joint.joint_type
                    )
                )
            current_transform = joint_transform

        expected_names = tuple(joint_names)
        if tuple(parsed_actuated_names) != expected_names:
            raise KinematicsError(
                "URDF actuated chain order {0} does not match config {1}.".format(
                    parsed_actuated_names, list(expected_names)
                )
            )
        if len(screw_axes) != 6:
            raise KinematicsError(
                "Selected chain must contain exactly six actuated joints; got {0}.".format(
                    len(screw_axes)
                )
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
        )

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
                twist_exp(
                    self._screw_axes_space[:, joint_index], joint_array[joint_index]
                )
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
        """Solve numerical IK using damped least squares.

        The error vector is expressed in the base frame and combines a rotation
        logarithm with Cartesian position error. The solver is deterministic,
        clamps each iteration, and never sends hardware commands.
        """
        joint_array = self._joint_array(seed_joint_position_rad).copy()
        target_matrix = target.as_matrix()
        lower_rad = np.asarray(self._joint_limits.lower_rad, dtype=float)
        upper_rad = np.asarray(self._joint_limits.upper_rad, dtype=float)
        joint_array = np.clip(joint_array, lower_rad, upper_rad)
        position_error_norm = math.inf
        orientation_error_norm = math.inf

        for iteration in range(self._ik_max_iterations + 1):
            current_matrix = self._fk_matrix(joint_array)
            position_error = target_matrix[:3, 3] - current_matrix[:3, 3]
            rotation_error_matrix = target_matrix[:3, :3].dot(
                current_matrix[:3, :3].T
            )
            orientation_error = rotation_log_vector(rotation_error_matrix)
            position_error_norm = float(np.linalg.norm(position_error))
            orientation_error_norm = float(np.linalg.norm(orientation_error))
            if (
                position_error_norm <= self._ik_position_tolerance_m
                and orientation_error_norm <= self._ik_orientation_tolerance_rad
            ):
                return IkResult(
                    succeeded=True,
                    joint_position_rad=tuple(joint_array),
                    iterations=iteration,
                    position_error_m=position_error_norm,
                    orientation_error_rad=orientation_error_norm,
                    message="IK converged.",
                )

            error_vector = np.concatenate((orientation_error, position_error))
            jacobian = self.compute_jacobian(joint_array)
            regularized = jacobian.dot(jacobian.T) + (
                self._ik_damping * self._ik_damping
            ) * np.eye(6, dtype=float)
            try:
                joint_delta = jacobian.T.dot(
                    np.linalg.solve(regularized, error_vector)
                )
            except np.linalg.LinAlgError:
                joint_delta = np.linalg.pinv(jacobian, rcond=1.0e-5).dot(
                    error_vector
                )
            delta_norm = float(np.linalg.norm(joint_delta))
            if delta_norm > self._ik_max_step_rad:
                joint_delta *= self._ik_max_step_rad / delta_norm
            joint_array = np.clip(joint_array + joint_delta, lower_rad, upper_rad)

        _LOGGER.warning(
            "ik_did_not_converge",
            extra={
                "position_error_m": position_error_norm,
                "orientation_error_rad": orientation_error_norm,
                "iterations": self._ik_max_iterations,
            },
        )
        return IkResult(
            succeeded=False,
            joint_position_rad=tuple(joint_array),
            iterations=self._ik_max_iterations,
            position_error_m=position_error_norm,
            orientation_error_rad=orientation_error_norm,
            message="IK did not converge within the configured iteration limit.",
        )
