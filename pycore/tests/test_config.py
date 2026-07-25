from dataclasses import replace
from operator import setitem
from pathlib import Path

import pytest
from myarm_m750_core.domain.errors import (
    ConfigurationError,
    ConfigurationMigrationError,
)
from myarm_m750_core.runtime.config import load_sdk_config


def test_default_config_resolves_owned_files(repository_root: Path) -> None:
    config = load_sdk_config(str(repository_root / "pycore" / "config" / "default.yaml"))
    assert config.adapter.adapter_type == "mock"
    assert config.config_version == 1
    assert config.robot.urdf_path.is_file()
    assert config.robot.runtime.command_rate_hz == 5.0
    assert config.safety.max_trajectory_points == 1000
    assert config.safety.max_workspace_resample_samples == 10000
    assert config.robot.joint_mapping["shoulder_lift_joint"].offset_degree == 10.0
    assert config.robot.joint_mapping["elbow_flex_joint"].offset_degree == -10.0


def test_robot_joint_mapping_is_defensively_frozen(repository_root: Path) -> None:
    config = load_sdk_config(str(repository_root / "pycore/config/default.yaml"))
    mutable_mapping = dict(config.robot.joint_mapping)
    robot = replace(config.robot, joint_mapping=mutable_mapping)
    shoulder_mapping = robot.joint_mapping["shoulder_pan_joint"]

    mutable_mapping["shoulder_pan_joint"] = replace(
        shoulder_mapping,
        offset_degree=999.0,
    )

    assert robot.joint_mapping["shoulder_pan_joint"] == shoulder_mapping
    with pytest.raises(TypeError):
        setitem(
            robot.joint_mapping,
            "shoulder_pan_joint",
            replace(shoulder_mapping, offset_degree=999.0),
        )


def test_invalid_config_fails_fast(tmp_path: Path) -> None:
    config_path = tmp_path / "invalid.yaml"
    config_path.write_text("sdk: []\n", encoding="utf-8")
    with pytest.raises(ConfigurationMigrationError, match="Legacy"):
        load_sdk_config(str(config_path))


def test_unknown_fields_are_rejected(tmp_path: Path) -> None:
    config_path = tmp_path / "unknown.yaml"
    config_path.write_text(
        "config_version: 1\nsdk: {}\nunknown: true\n",
        encoding="utf-8",
    )
    with pytest.raises(ConfigurationError, match="unknown"):
        load_sdk_config(str(config_path))


def test_real_example_is_intentionally_non_runnable(repository_root: Path) -> None:
    with pytest.raises(ConfigurationError, match="serial_by_id"):
        load_sdk_config(str(repository_root / "pycore/config/default_real.example.yaml"))
