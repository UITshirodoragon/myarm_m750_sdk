"""Fail-fast YAML loading for the Python Core SDK."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Mapping, Sequence, Tuple

import yaml

from myarm_m750_core.runtime.config.models import (
    AdapterConfig,
    JointMappingConfig,
    LoggingConfig,
    RobotConfig,
    RuntimeConfig,
    SafetyConfig,
    SdkConfig,
    SingularityConfig,
    WorkspaceConfig,
)
from myarm_m750_core.domain.errors import ConfigurationError


def _read_yaml(path: Path) -> Dict[str, Any]:
    if not path.is_file():
        raise ConfigurationError("YAML config does not exist: {0}".format(path))
    try:
        with path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
    except yaml.YAMLError as error:
        raise ConfigurationError(
            "Invalid YAML in {0}: {1}".format(path, error)
        ) from error
    if not isinstance(data, dict):
        raise ConfigurationError("Top-level YAML value must be a mapping: {0}".format(path))
    return data


def _require_mapping(data: Mapping[str, Any], key: str, source: Path) -> Mapping[str, Any]:
    value = data.get(key)
    if not isinstance(value, dict):
        raise ConfigurationError(
            "Missing or invalid mapping '{0}' in {1}.".format(key, source)
        )
    return value


def _require_sequence(
    data: Mapping[str, Any], key: str, source: Path, expected_size: int
) -> Sequence[Any]:
    value = data.get(key)
    if not isinstance(value, list) or len(value) != expected_size:
        raise ConfigurationError(
            "'{0}' in {1} must be a list with {2} items.".format(
                key, source, expected_size
            )
        )
    return value


def _resolve_path(base_file: Path, configured_path: str) -> Path:
    candidate = Path(configured_path).expanduser()
    if candidate.is_absolute():
        return candidate.resolve()
    return (base_file.parent / candidate).resolve()


def _parse_robot(path: Path) -> RobotConfig:
    data = _read_yaml(path)
    robot = _require_mapping(data, "robot", path)
    joint_names_raw = _require_sequence(robot, "joint_names", path, 6)
    joint_names = tuple(str(name) for name in joint_names_raw)
    if len(set(joint_names)) != 6:
        raise ConfigurationError("robot.joint_names must contain six unique names.")

    model = _require_mapping(robot, "model", path)
    runtime_raw = _require_mapping(robot, "runtime", path)
    mapping_raw = robot.get("joint_mapping", {})
    if not isinstance(mapping_raw, dict):
        raise ConfigurationError("robot.joint_mapping must be a mapping.")

    joint_mapping: Dict[str, JointMappingConfig] = {}
    for joint_name in joint_names:
        joint_raw = mapping_raw.get(joint_name, {})
        if not isinstance(joint_raw, dict):
            raise ConfigurationError(
                "Mapping for joint '{0}' must be a mapping.".format(joint_name)
            )
        direction = int(joint_raw.get("direction", 1))
        if direction not in (-1, 1):
            raise ConfigurationError(
                "Mapping direction for '{0}' must be -1 or 1.".format(joint_name)
            )
        joint_mapping[joint_name] = JointMappingConfig(
            offset_degree=float(joint_raw.get("offset_degree", 0.0)),
            direction=direction,
        )

    command_rate_hz = float(runtime_raw.get("command_rate_hz", 5.0))
    state_rate_hz = float(runtime_raw.get("state_rate_hz", 5.0))
    if command_rate_hz <= 0.0 or state_rate_hz <= 0.0:
        raise ConfigurationError("Runtime rates must be positive.")

    urdf_path = _resolve_path(path, str(model.get("urdf_path", "")))
    if not urdf_path.is_file():
        raise ConfigurationError(
            "Configured URDF does not exist: {0}".format(urdf_path)
        )

    return RobotConfig(
        name=str(robot.get("name", "myarm_m750")),
        joint_names=joint_names,
        urdf_path=urdf_path,
        base_link=str(model.get("base_link", "base_link")),
        end_link=str(model.get("end_link", "tool0")),
        joint_mapping=joint_mapping,
        runtime=RuntimeConfig(
            command_rate_hz=command_rate_hz,
            state_rate_hz=state_rate_hz,
            realtime_execution=bool(runtime_raw.get("realtime_execution", True)),
        ),
    )


def _parse_safety(path: Path) -> SafetyConfig:
    data = _read_yaml(path)
    safety = _require_mapping(data, "safety", path)
    workspace_raw = _require_mapping(safety, "workspace", path)
    minimum_raw = _require_sequence(workspace_raw, "minimum_m", path, 3)
    maximum_raw = _require_sequence(workspace_raw, "maximum_m", path, 3)
    minimum_m = tuple(float(value) for value in minimum_raw)
    maximum_m = tuple(float(value) for value in maximum_raw)
    if any(lower >= upper for lower, upper in zip(minimum_m, maximum_m)):
        raise ConfigurationError("Workspace minimum values must be below maximum values.")

    singularity_raw = _require_mapping(safety, "singularity", path)
    max_joint_step_rad = float(safety.get("max_joint_step_rad", 0.08))
    if max_joint_step_rad <= 0.0:
        raise ConfigurationError("safety.max_joint_step_rad must be positive.")

    return SafetyConfig(
        enabled=bool(safety.get("enabled", True)),
        state_timeout_s=float(safety.get("state_timeout_s", 1.0)),
        command_timeout_s=float(safety.get("command_timeout_s", 2.0)),
        max_joint_step_rad=max_joint_step_rad,
        joint_limit_margin_rad=float(safety.get("joint_limit_margin_rad", 0.0)),
        reject_nan_or_inf=bool(safety.get("reject_nan_or_inf", True)),
        workspace=WorkspaceConfig(
            minimum_m=minimum_m,  # type: ignore[arg-type]
            maximum_m=maximum_m,  # type: ignore[arg-type]
        ),
        singularity=SingularityConfig(
            enabled=bool(singularity_raw.get("enabled", False)),
            minimum_score=float(singularity_raw.get("minimum_score", 0.0)),
        ),
    )


def _parse_logging(path: Path) -> LoggingConfig:
    data = _read_yaml(path)
    logging_raw = _require_mapping(data, "logging", path)
    configured_file = str(logging_raw.get("file", ""))
    file_path = ""
    if configured_file:
        file_path = str(_resolve_path(path, configured_file))
    return LoggingConfig(
        level=str(logging_raw.get("level", "INFO")),
        console=bool(logging_raw.get("console", True)),
        file=file_path,
        max_bytes=int(logging_raw.get("max_bytes", 5_000_000)),
        backup_count=int(logging_raw.get("backup_count", 3)),
        json_file=bool(logging_raw.get("json_file", True)),
    )


def _load_from_manifest(path: Path, data: Mapping[str, Any]) -> SdkConfig:
    sdk = _require_mapping(data, "sdk", path)
    config_files = _require_mapping(sdk, "config_files", path)
    robot_path = _resolve_path(path, str(config_files.get("robot", "robot_m750.yaml")))
    safety_path = _resolve_path(path, str(config_files.get("safety", "safety.yaml")))
    logging_path = _resolve_path(path, str(config_files.get("logging", "logging.yaml")))

    adapter_raw = _require_mapping(sdk, "adapter", path)
    adapter_type = str(adapter_raw.get("type", "mock"))
    options = dict(adapter_raw.get("options", {}))
    if not isinstance(adapter_raw.get("options", {}), dict):
        raise ConfigurationError("sdk.adapter.options must be a mapping.")

    return SdkConfig(
        source_path=path,
        robot=_parse_robot(robot_path),
        safety=_parse_safety(safety_path),
        logging=_parse_logging(logging_path),
        adapter=AdapterConfig(adapter_type=adapter_type, options=options),
    )


def _load_from_robot_file(path: Path, data: Mapping[str, Any]) -> SdkConfig:
    safety_path = path.parent / "safety.yaml"
    logging_path = path.parent / "logging.yaml"
    if not safety_path.is_file() or not logging_path.is_file():
        raise ConfigurationError(
            "Loading a robot YAML directly requires sibling safety.yaml and logging.yaml."
        )
    return SdkConfig(
        source_path=path,
        robot=_parse_robot(path),
        safety=_parse_safety(safety_path),
        logging=_parse_logging(logging_path),
        adapter=AdapterConfig(adapter_type="mock", options={}),
    )


def load_sdk_config(config_path: str) -> SdkConfig:
    """Load and validate a manifest or robot YAML file.

    Args:
        config_path: Path to ``default.yaml``/``default_real.yaml`` or directly
            to ``robot_m750.yaml``.

    Returns:
        Fully resolved immutable configuration.

    Raises:
        ConfigurationError: If a file or required value is invalid.
    """
    path = Path(config_path).expanduser().resolve()
    data = _read_yaml(path)
    if "sdk" in data:
        return _load_from_manifest(path, data)
    if "robot" in data:
        return _load_from_robot_file(path, data)
    raise ConfigurationError(
        "Config must contain either a top-level 'sdk' or 'robot' mapping: {0}".format(
            path
        )
    )
