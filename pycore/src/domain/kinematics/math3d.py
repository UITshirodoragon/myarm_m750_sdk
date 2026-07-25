"""Small SE(3) helpers used by the PoE kinematics backend."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np


def skew(vector: Iterable[float]) -> np.ndarray:
    """Return the 3x3 cross-product matrix of a 3-vector."""
    x_value, y_value, z_value = (float(value) for value in vector)
    return np.array(
        [
            [0.0, -z_value, y_value],
            [z_value, 0.0, -x_value],
            [-y_value, x_value, 0.0],
        ],
        dtype=float,
    )


def rotation_x(angle_rad: float) -> np.ndarray:
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return np.array(
        [[1.0, 0.0, 0.0], [0.0, cosine, -sine], [0.0, sine, cosine]],
        dtype=float,
    )


def rotation_y(angle_rad: float) -> np.ndarray:
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return np.array(
        [[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]],
        dtype=float,
    )


def rotation_z(angle_rad: float) -> np.ndarray:
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return np.array(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
        dtype=float,
    )


def rpy_to_matrix(rpy_rad: Iterable[float]) -> np.ndarray:
    """Return URDF fixed-axis RPY rotation ``Rz(yaw) Ry(pitch) Rx(roll)``."""
    roll_rad, pitch_rad, yaw_rad = (float(value) for value in rpy_rad)
    return rotation_z(yaw_rad).dot(rotation_y(pitch_rad)).dot(rotation_x(roll_rad))


def transform_from_xyz_rpy(xyz_m: Iterable[float], rpy_rad: Iterable[float]) -> np.ndarray:
    """Build a homogeneous transform from URDF origin values."""
    transform_matrix = np.eye(4, dtype=float)
    transform_matrix[:3, :3] = rpy_to_matrix(rpy_rad)
    transform_matrix[:3, 3] = np.asarray(tuple(xyz_m), dtype=float)
    return transform_matrix


def rotation_exp(axis: Iterable[float], angle_rad: float) -> np.ndarray:
    """Rodrigues exponential for a unit rotation axis."""
    axis_vector = np.asarray(tuple(axis), dtype=float)
    axis_norm = float(np.linalg.norm(axis_vector))
    if axis_norm < 1.0e-12:
        return np.eye(3, dtype=float)
    unit_axis = axis_vector / axis_norm
    axis_skew = skew(unit_axis)
    return (
        np.eye(3, dtype=float)
        + math.sin(angle_rad) * axis_skew
        + (1.0 - math.cos(angle_rad)) * axis_skew.dot(axis_skew)
    )


def twist_exp(screw_axis: Iterable[float], joint_position: float) -> np.ndarray:
    """Return ``exp([S] theta)`` for revolute or prismatic screw axes."""
    screw = np.asarray(tuple(screw_axis), dtype=float)
    if screw.shape != (6,):
        raise ValueError("screw_axis must contain six values.")
    angular = screw[:3]
    linear = screw[3:]
    transform_matrix = np.eye(4, dtype=float)
    angular_norm = float(np.linalg.norm(angular))
    if angular_norm < 1.0e-12:
        transform_matrix[:3, 3] = linear * float(joint_position)
        return transform_matrix

    unit_angular = angular / angular_norm
    theta = float(joint_position) * angular_norm
    angular_skew = skew(unit_angular)
    rotation_matrix = rotation_exp(unit_angular, theta)
    translation_matrix = (
        np.eye(3, dtype=float) * theta
        + (1.0 - math.cos(theta)) * angular_skew
        + (theta - math.sin(theta)) * angular_skew.dot(angular_skew)
    )
    transform_matrix[:3, :3] = rotation_matrix
    transform_matrix[:3, 3] = translation_matrix.dot(linear / angular_norm)
    return transform_matrix


def adjoint(transform_matrix: np.ndarray) -> np.ndarray:
    """Return the 6x6 adjoint for the [angular; linear] twist convention."""
    transform = np.asarray(transform_matrix, dtype=float)
    rotation_matrix = transform[:3, :3]
    translation_m = transform[:3, 3]
    result = np.zeros((6, 6), dtype=float)
    result[:3, :3] = rotation_matrix
    result[3:, 3:] = rotation_matrix
    result[3:, :3] = skew(translation_m).dot(rotation_matrix)
    return result


def quaternion_xyzw_to_matrix(quaternion_xyzw: Iterable[float]) -> np.ndarray:
    """Convert a normalized or non-normalized XYZW quaternion to a matrix."""
    x_value, y_value, z_value, w_value = (float(value) for value in quaternion_xyzw)
    quaternion_norm = math.sqrt(
        x_value * x_value + y_value * y_value + z_value * z_value + w_value * w_value
    )
    if quaternion_norm < 1.0e-12:
        raise ValueError("Quaternion norm must be non-zero.")
    x_value /= quaternion_norm
    y_value /= quaternion_norm
    z_value /= quaternion_norm
    w_value /= quaternion_norm
    return np.array(
        [
            [
                1.0 - 2.0 * (y_value * y_value + z_value * z_value),
                2.0 * (x_value * y_value - z_value * w_value),
                2.0 * (x_value * z_value + y_value * w_value),
            ],
            [
                2.0 * (x_value * y_value + z_value * w_value),
                1.0 - 2.0 * (x_value * x_value + z_value * z_value),
                2.0 * (y_value * z_value - x_value * w_value),
            ],
            [
                2.0 * (x_value * z_value - y_value * w_value),
                2.0 * (y_value * z_value + x_value * w_value),
                1.0 - 2.0 * (x_value * x_value + y_value * y_value),
            ],
        ],
        dtype=float,
    )


def quaternion_xyzw_to_wxyz(quaternion_xyzw: Iterable[float]) -> np.ndarray:
    """Convert an XYZW quaternion DTO to the WXYZ convention used by Pinocchio."""
    quaternion = np.asarray(tuple(quaternion_xyzw), dtype=float)
    if quaternion.shape != (4,):
        raise ValueError("quaternion_xyzw must contain four values.")
    if not np.all(np.isfinite(quaternion)):
        raise ValueError("quaternion_xyzw must contain finite values.")
    return quaternion[[3, 0, 1, 2]]


def quaternion_wxyz_to_xyzw(quaternion_wxyz: Iterable[float]) -> np.ndarray:
    """Convert a Pinocchio-style WXYZ quaternion to the core XYZW convention."""
    quaternion = np.asarray(tuple(quaternion_wxyz), dtype=float)
    if quaternion.shape != (4,):
        raise ValueError("quaternion_wxyz must contain four values.")
    if not np.all(np.isfinite(quaternion)):
        raise ValueError("quaternion_wxyz must contain finite values.")
    return quaternion[[1, 2, 3, 0]]


def matrix_to_quaternion_xyzw(rotation_matrix: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to an XYZW quaternion."""
    matrix = np.asarray(rotation_matrix, dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w_value = 0.25 * scale
        x_value = (matrix[2, 1] - matrix[1, 2]) / scale
        y_value = (matrix[0, 2] - matrix[2, 0]) / scale
        z_value = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w_value = (matrix[2, 1] - matrix[1, 2]) / scale
        x_value = 0.25 * scale
        y_value = (matrix[0, 1] + matrix[1, 0]) / scale
        z_value = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w_value = (matrix[0, 2] - matrix[2, 0]) / scale
        x_value = (matrix[0, 1] + matrix[1, 0]) / scale
        y_value = 0.25 * scale
        z_value = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w_value = (matrix[1, 0] - matrix[0, 1]) / scale
        x_value = (matrix[0, 2] + matrix[2, 0]) / scale
        y_value = (matrix[1, 2] + matrix[2, 1]) / scale
        z_value = 0.25 * scale
    quaternion = np.array([x_value, y_value, z_value, w_value], dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    return quaternion


def rotation_log_vector(rotation_matrix: np.ndarray) -> np.ndarray:
    """Return the axis-angle vector for a rotation matrix."""
    matrix = np.asarray(rotation_matrix, dtype=float)
    cosine_angle = max(-1.0, min(1.0, (float(np.trace(matrix)) - 1.0) * 0.5))
    angle_rad = math.acos(cosine_angle)
    if angle_rad < 1.0e-9:
        return np.zeros(3, dtype=float)
    if math.pi - angle_rad < 1.0e-6:
        diagonal = np.diagonal(matrix)
        axis = np.sqrt(np.maximum((diagonal + 1.0) * 0.5, 0.0))
        axis[0] = math.copysign(axis[0], matrix[2, 1] - matrix[1, 2])
        axis[1] = math.copysign(axis[1], matrix[0, 2] - matrix[2, 0])
        axis[2] = math.copysign(axis[2], matrix[1, 0] - matrix[0, 1])
        axis_norm = float(np.linalg.norm(axis))
        if axis_norm < 1.0e-9:
            axis = np.array([1.0, 0.0, 0.0], dtype=float)
        else:
            axis /= axis_norm
        return axis * angle_rad
    axis = np.array(
        [
            matrix[2, 1] - matrix[1, 2],
            matrix[0, 2] - matrix[2, 0],
            matrix[1, 0] - matrix[0, 1],
        ],
        dtype=float,
    ) / (2.0 * math.sin(angle_rad))
    return axis * angle_rad
