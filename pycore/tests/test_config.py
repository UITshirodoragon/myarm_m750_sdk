from pathlib import Path

import pytest

from myarm_m750_core.runtime.config import load_sdk_config
from myarm_m750_core.domain.errors import ConfigurationError


def test_default_config_resolves_owned_files(repository_root: Path) -> None:
    config = load_sdk_config(str(repository_root / "pycore" / "config" / "default.yaml"))
    assert config.adapter.adapter_type == "mock"
    assert config.robot.urdf_path.is_file()
    assert config.robot.runtime.command_rate_hz == 5.0
    assert config.robot.joint_mapping["shoulder_lift_joint"].offset_degree == 10.0
    assert config.robot.joint_mapping["elbow_flex_joint"].offset_degree == -10.0


def test_invalid_config_fails_fast(tmp_path: Path) -> None:
    config_path = tmp_path / "invalid.yaml"
    config_path.write_text("sdk: []\n", encoding="utf-8")
    with pytest.raises(ConfigurationError):
        load_sdk_config(str(config_path))
