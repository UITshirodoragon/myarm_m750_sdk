import math

import numpy as np
import pytest
from myarm_m750_core.adapters import JointMapper
from myarm_m750_core.runtime.config.loader import mapping_fingerprint
from myarm_m750_core.runtime.config.models import JointMappingConfig


def test_joint_mapping_applies_only_hardware_offsets(sdk_config) -> None:
    mapper = JointMapper(sdk_config.robot.joint_names, sdk_config.robot.joint_mapping)
    firmware_deg = mapper.core_rad_to_firmware_deg([0.0] * 6)
    assert firmware_deg == (0.0, 10.0, -10.0, 0.0, 0.0, 0.0)


def test_joint_mapping_round_trip(sdk_config) -> None:
    mapper = JointMapper(sdk_config.robot.joint_names, sdk_config.robot.joint_mapping)
    canonical_rad = (0.25, -0.30, 0.45, -0.20, 0.10, math.pi / 3.0)
    firmware_deg = mapper.core_rad_to_firmware_deg(canonical_rad)
    reconstructed_rad = mapper.firmware_deg_to_core_rad(firmware_deg)
    np.testing.assert_allclose(reconstructed_rad, canonical_rad, atol=1.0e-12)


def test_joint_mapping_fingerprint_attests_only_the_ordered_software_contract(
    sdk_config,
) -> None:
    mapper = JointMapper(sdk_config.robot.joint_names, sdk_config.robot.joint_mapping)
    assert mapper.joint_count == 6
    assert mapper.contract_fingerprint == mapping_fingerprint(
        sdk_config.robot.joint_mapping
    )
    assert len(mapper.contract_fingerprint) == 64

    changed = dict(sdk_config.robot.joint_mapping)
    first_joint = sdk_config.robot.joint_names[0]
    changed[first_joint] = JointMappingConfig(offset_degree=1.0, direction=-1)
    changed_mapper = JointMapper(sdk_config.robot.joint_names, changed)
    assert changed_mapper.contract_fingerprint != mapper.contract_fingerprint

    with pytest.raises(ValueError, match="exactly match"):
        JointMapper(sdk_config.robot.joint_names, {})
