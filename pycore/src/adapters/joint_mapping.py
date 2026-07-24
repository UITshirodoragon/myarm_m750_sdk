"""Canonical ROS joint coordinates to firmware degree mapping."""

from __future__ import annotations

import math
from typing import Mapping, Sequence, Tuple

from myarm_m750_core.runtime.config.models import JointMappingConfig


class JointMapper:
    """Convert units and offsets exactly once at the hardware boundary."""

    def __init__(
        self,
        joint_names: Sequence[str],
        mapping: Mapping[str, JointMappingConfig],
    ) -> None:
        self._joint_names = tuple(joint_names)
        self._mapping = mapping

    def core_rad_to_firmware_deg(
        self, joint_position_rad: Sequence[float]
    ) -> Tuple[float, ...]:
        """Map canonical radians to firmware degrees."""
        if len(joint_position_rad) != len(self._joint_names):
            raise ValueError("Joint vector size does not match configured joint names.")
        firmware_degree = []
        for joint_name, core_rad in zip(self._joint_names, joint_position_rad):
            joint_mapping = self._mapping[joint_name]
            core_degree = math.degrees(float(core_rad))
            firmware_degree.append(
                joint_mapping.direction * core_degree + joint_mapping.offset_degree
            )
        return tuple(firmware_degree)

    def firmware_deg_to_core_rad(
        self, firmware_position_deg: Sequence[float]
    ) -> Tuple[float, ...]:
        """Map firmware degrees to canonical radians."""
        if len(firmware_position_deg) != len(self._joint_names):
            raise ValueError("Joint vector size does not match configured joint names.")
        core_rad = []
        for joint_name, firmware_degree in zip(
            self._joint_names, firmware_position_deg
        ):
            joint_mapping = self._mapping[joint_name]
            core_degree = (
                float(firmware_degree) - joint_mapping.offset_degree
            ) / joint_mapping.direction
            core_rad.append(math.radians(core_degree))
        return tuple(core_rad)
