"""Canonical ROS joint coordinates to firmware degree mapping."""

from __future__ import annotations

import math
from types import MappingProxyType
from typing import Mapping, Sequence, Tuple

from myarm_m750_core.runtime.config.models import (
    JointMappingConfig,
    joint_mapping_contract_fingerprint,
)


class JointMapper:
    """Convert units and offsets exactly once at the hardware boundary."""

    def __init__(
        self,
        joint_names: Sequence[str],
        mapping: Mapping[str, JointMappingConfig],
    ) -> None:
        self._joint_names = tuple(joint_names)
        if not self._joint_names or len(set(self._joint_names)) != len(
            self._joint_names
        ):
            raise ValueError("joint_names must be non-empty and unique.")
        if set(mapping) != set(self._joint_names):
            raise ValueError("Joint mapping keys must exactly match joint_names.")
        normalized_mapping = {}
        for joint_name in self._joint_names:
            profile = mapping[joint_name]
            if isinstance(profile.direction, bool) or profile.direction not in (-1, 1):
                raise ValueError("Joint mapping direction must be -1 or 1.")
            offset_degree = float(profile.offset_degree)
            if not math.isfinite(offset_degree):
                raise ValueError("Joint mapping offset_degree must be finite.")
            normalized_mapping[joint_name] = JointMappingConfig(
                offset_degree=offset_degree,
                direction=int(profile.direction),
            )
        self._mapping = MappingProxyType(normalized_mapping)
        self._contract_fingerprint = joint_mapping_contract_fingerprint(
            self._joint_names,
            self._mapping,
        )

    @property
    def joint_count(self) -> int:
        """Return the exact canonical vector length accepted by this mapper."""
        return len(self._joint_names)

    @property
    def contract_fingerprint(self) -> str:
        """Identify the ordered software mapping, not physical direction evidence."""
        return self._contract_fingerprint

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
        for joint_name, firmware_degree in zip(self._joint_names, firmware_position_deg):
            joint_mapping = self._mapping[joint_name]
            core_degree = (
                float(firmware_degree) - joint_mapping.offset_degree
            ) / joint_mapping.direction
            core_rad.append(math.radians(core_degree))
        return tuple(core_rad)
