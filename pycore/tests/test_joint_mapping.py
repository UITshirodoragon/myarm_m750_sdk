import math

import numpy as np

from myarm_m750_core.adapters import JointMapper


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
