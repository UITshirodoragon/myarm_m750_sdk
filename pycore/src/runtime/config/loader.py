"""Strict, fail-before-I/O YAML loader for the v0.2 configuration contract."""

from __future__ import annotations

import hashlib
import math
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence, Set, Tuple

import yaml
from myarm_m750_core.domain.errors import (
    ConfigurationError,
    ConfigurationMigrationError,
    KinematicsError,
)
from myarm_m750_core.domain.kinematics.model import fingerprint_urdf_path
from myarm_m750_core.domain.models import CapabilityState
from myarm_m750_core.runtime.config.models import (
    CONFIG_VERSION,
    AdapterConfig,
    CapabilityVerificationProfile,
    FirmwareProtocolProfile,
    HardwareProfile,
    JointMappingConfig,
    LoggingConfig,
    MockAdapterProfile,
    ReplayAdapterProfile,
    RobotConfig,
    RuntimeConfig,
    SafetyConfig,
    SdkConfig,
    SingularityConfig,
    WorkspaceConfig,
    joint_mapping_contract_fingerprint,
)


def _read_yaml(path: Path) -> Dict[str, Any]:
    if not path.is_file():
        raise ConfigurationError(f"YAML config does not exist: {path}")
    try:
        with path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
    except yaml.YAMLError as error:
        raise ConfigurationError(f"Invalid YAML in {path}: {error}") from error
    if not isinstance(data, dict):
        raise ConfigurationError(f"Top-level YAML must be a mapping: {path}")
    return data


def _check_keys(
    data: Mapping[str, Any],
    required: Set[str],
    source: Path,
    location: str,
    optional: Optional[Set[str]] = None,
) -> None:
    actual = set(data)
    missing = sorted(required - actual)
    unknown = sorted(actual - required - (optional or set()))
    if missing or unknown:
        details = []
        if missing:
            details.append(f"missing={','.join(missing)}")
        if unknown:
            details.append(f"unknown={','.join(unknown)}")
        raise ConfigurationError(
            f"{location} has invalid fields in {source}: {'; '.join(details)}."
        )


def _require_version(data: Mapping[str, Any], source: Path, root_key: str) -> None:
    if "config_version" not in data:
        raise ConfigurationMigrationError(
            f"Legacy configuration is not accepted by v0.2.0: {source}. "
            "Add config_version: 1 and migrate to the strict schema."
        )
    _check_keys(data, {"config_version", root_key}, source, "top level")
    version = data["config_version"]
    if isinstance(version, bool) or not isinstance(version, int):
        raise ConfigurationMigrationError(
            f"config_version in {source} must be the integer {CONFIG_VERSION}."
        )
    if version != CONFIG_VERSION:
        raise ConfigurationMigrationError(
            f"Unsupported config_version {version!r} in "
            f"{source}; expected {CONFIG_VERSION}."
        )


def _mapping(
    data: Mapping[str, Any], key: str, source: Path, location: str
) -> Mapping[str, Any]:
    value = data.get(key)
    if not isinstance(value, dict):
        raise ConfigurationError(f"{location}.{key} must be a mapping in {source}.")
    return value


def _sequence(
    data: Mapping[str, Any],
    key: str,
    source: Path,
    location: str,
    size: int,
) -> Sequence[Any]:
    value = data.get(key)
    if not isinstance(value, list) or len(value) != size:
        raise ConfigurationError(
            f"{location}.{key} must contain exactly {size} items in {source}."
        )
    return value


def _number(value: Any, source: Path, location: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigurationError(f"{location} must be numeric in {source}.")
    converted = float(value)
    if not math.isfinite(converted):
        raise ConfigurationError(f"{location} must be finite in {source}.")
    return converted


def _integer(value: Any, source: Path, location: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigurationError(f"{location} must be an integer in {source}.")
    return value


def _text(value: Any, source: Path, location: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ConfigurationError(
            f"{location} must be a non-empty string in {source}."
        )
    return value.strip()


def _boolean(value: Any, source: Path, location: str) -> bool:
    if not isinstance(value, bool):
        raise ConfigurationError(f"{location} must be boolean in {source}.")
    return value


def _resolve_path(base_file: Path, configured_path: Any) -> Path:
    configured_text = _text(configured_path, base_file, "Configured path")
    candidate = Path(configured_text).expanduser()
    if candidate.is_absolute():
        return candidate.resolve()
    return (base_file.parent / candidate).resolve()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _fingerprint(value: Any, source: Path, location: str) -> str:
    if not isinstance(value, str):
        raise ConfigurationError(
            f"{location} must be a lowercase SHA-256 value in {source}."
        )
    text = value
    if len(text) != 64 or any(character not in "0123456789abcdef" for character in text):
        raise ConfigurationError(
            f"{location} must be a lowercase SHA-256 value in {source}."
        )
    return text


def mapping_fingerprint(mapping: Mapping[str, JointMappingConfig]) -> str:
    """Return the deterministic canonical-to-firmware mapping fingerprint."""
    return joint_mapping_contract_fingerprint(tuple(mapping), mapping)


def _parse_robot(path: Path) -> RobotConfig:
    data = _read_yaml(path)
    _require_version(data, path, "robot")
    robot = _mapping(data, "robot", path, "top level")
    _check_keys(
        robot,
        {"name", "joint_names", "model", "joint_mapping", "runtime"},
        path,
        "robot",
    )
    joint_names_raw = _sequence(robot, "joint_names", path, "robot", 6)
    joint_names = tuple(
        _text(name, path, f"robot.joint_names[{index}]")
        for index, name in enumerate(joint_names_raw)
    )
    if len(set(joint_names)) != 6:
        raise ConfigurationError("robot.joint_names must contain six unique names.")
    robot_name = _text(robot["name"], path, "robot.name")

    model = _mapping(robot, "model", path, "robot")
    _check_keys(
        model,
        {
            "urdf_path",
            "base_link",
            "end_link",
            "resource_sha256",
            "kinematic_contract_sha256",
        },
        path,
        "robot.model",
    )
    urdf_path = _resolve_path(path, model["urdf_path"])
    if not urdf_path.is_file():
        raise ConfigurationError(f"Configured URDF does not exist: {urdf_path}")
    expected_sha256 = _fingerprint(
        model["resource_sha256"], path, "robot.model.resource_sha256"
    )
    actual_sha256 = _sha256(urdf_path)
    if actual_sha256 != expected_sha256:
        raise ConfigurationError(
            f"Robot model fingerprint mismatch for {urdf_path}: "
            f"expected {expected_sha256}, got {actual_sha256}."
        )
    contract_sha256 = _fingerprint(
        model["kinematic_contract_sha256"],
        path,
        "robot.model.kinematic_contract_sha256",
    )
    try:
        actual_contract_sha256 = fingerprint_urdf_path(urdf_path)
    except KinematicsError as error:
        raise ConfigurationError(
            f"Could not fingerprint robot kinematic contract: {error}"
        ) from error
    if actual_contract_sha256 != contract_sha256:
        raise ConfigurationError(
            "Robot kinematic contract fingerprint mismatch for "
            f"{urdf_path}: expected {contract_sha256}, "
            f"got {actual_contract_sha256}."
        )
    base_link = _text(model["base_link"], path, "robot.model.base_link")
    end_link = _text(model["end_link"], path, "robot.model.end_link")
    if base_link == end_link:
        raise ConfigurationError("Robot base_link and end_link must be distinct.")

    mapping_raw = _mapping(robot, "joint_mapping", path, "robot")
    _check_keys(mapping_raw, set(joint_names), path, "robot.joint_mapping")
    joint_mapping = {}
    for joint_name in joint_names:
        profile = _mapping(mapping_raw, joint_name, path, "robot.joint_mapping")
        _check_keys(
            profile,
            {"offset_degree", "direction"},
            path,
            f"robot.joint_mapping.{joint_name}",
        )
        direction = _integer(
            profile["direction"],
            path,
            f"robot.joint_mapping.{joint_name}.direction",
        )
        if direction not in (-1, 1):
            raise ConfigurationError(f"Mapping direction for {joint_name} must be -1 or 1.")
        joint_mapping[joint_name] = JointMappingConfig(
            offset_degree=_number(
                profile["offset_degree"],
                path,
                f"robot.joint_mapping.{joint_name}.offset_degree",
            ),
            direction=direction,
        )

    runtime = _mapping(robot, "runtime", path, "robot")
    _check_keys(runtime, {"command_rate_hz", "state_rate_hz"}, path, "robot.runtime")
    command_rate_hz = _number(
        runtime["command_rate_hz"], path, "robot.runtime.command_rate_hz"
    )
    state_rate_hz = _number(runtime["state_rate_hz"], path, "robot.runtime.state_rate_hz")
    if command_rate_hz <= 0.0 or state_rate_hz <= 0.0:
        raise ConfigurationError("Runtime rates must be positive.")
    return RobotConfig(
        name=robot_name,
        joint_names=joint_names,
        urdf_path=urdf_path,
        base_link=base_link,
        end_link=end_link,
        resource_fingerprint=actual_sha256,
        kinematic_contract_fingerprint=contract_sha256,
        joint_mapping=joint_mapping,
        runtime=RuntimeConfig(
            command_rate_hz=command_rate_hz,
            state_rate_hz=state_rate_hz,
        ),
    )


def _float_tuple(
    data: Mapping[str, Any],
    key: str,
    source: Path,
    location: str,
    size: int,
) -> Tuple[float, ...]:
    values = _sequence(data, key, source, location, size)
    return tuple(_number(value, source, f"{location}.{key}") for value in values)


def _parse_safety(path: Path) -> SafetyConfig:
    data = _read_yaml(path)
    _require_version(data, path, "safety")
    safety = _mapping(data, "safety", path, "top level")
    _check_keys(
        safety,
        {
            "enabled",
            "provenance",
            "max_trajectory_points",
            "max_workspace_resample_samples",
            "state_timeout_s",
            "command_timeout_s",
            "stop_timeout_s",
            "max_joint_step_rad",
            "max_joint_velocity_rad_s",
            "max_joint_acceleration_rad_s2",
            "joint_limit_margin_rad",
            "workspace",
            "singularity",
        },
        path,
        "safety",
    )
    velocity = _float_tuple(safety, "max_joint_velocity_rad_s", path, "safety", 6)
    acceleration = _float_tuple(safety, "max_joint_acceleration_rad_s2", path, "safety", 6)
    if any(value <= 0.0 for value in velocity + acceleration):
        raise ConfigurationError("Safety velocity/acceleration limits must be positive.")

    workspace = _mapping(safety, "workspace", path, "safety")
    _check_keys(
        workspace,
        {"minimum_m", "maximum_m", "resample_step_rad"},
        path,
        "safety.workspace",
    )
    minimum = _float_tuple(workspace, "minimum_m", path, "safety.workspace", 3)
    maximum = _float_tuple(workspace, "maximum_m", path, "safety.workspace", 3)
    if any(lower >= upper for lower, upper in zip(minimum, maximum)):
        raise ConfigurationError("Workspace minimum values must be below maximum values.")
    resample_step_rad = _number(
        workspace["resample_step_rad"],
        path,
        "safety.workspace.resample_step_rad",
    )
    singularity = _mapping(safety, "singularity", path, "safety")
    _check_keys(singularity, {"enabled", "minimum_score"}, path, "safety.singularity")

    positive_fields = {
        "state_timeout_s": _number(
            safety["state_timeout_s"], path, "safety.state_timeout_s"
        ),
        "command_timeout_s": _number(
            safety["command_timeout_s"], path, "safety.command_timeout_s"
        ),
        "stop_timeout_s": _number(safety["stop_timeout_s"], path, "safety.stop_timeout_s"),
        "max_joint_step_rad": _number(
            safety["max_joint_step_rad"], path, "safety.max_joint_step_rad"
        ),
    }
    if any(value <= 0.0 for value in positive_fields.values()):
        raise ConfigurationError("Safety timeouts, rates, and steps must be positive.")
    max_trajectory_points = _integer(
        safety["max_trajectory_points"],
        path,
        "safety.max_trajectory_points",
    )
    max_workspace_resample_samples = _integer(
        safety["max_workspace_resample_samples"],
        path,
        "safety.max_workspace_resample_samples",
    )
    if max_trajectory_points <= 0 or max_workspace_resample_samples <= 0:
        raise ConfigurationError(
            "Safety trajectory and workspace-resample budgets must be positive."
        )
    if resample_step_rad <= 0.0:
        raise ConfigurationError("safety.workspace.resample_step_rad must be positive.")
    provenance = _text(safety["provenance"], path, "safety.provenance")
    enabled = _boolean(safety["enabled"], path, "safety.enabled")
    if not enabled:
        raise ConfigurationError(
            "safety.enabled must be true; mandatory trajectory safety cannot "
            "be disabled in v0.2.0."
        )
    joint_limit_margin_rad = _number(
        safety["joint_limit_margin_rad"],
        path,
        "safety.joint_limit_margin_rad",
    )
    if joint_limit_margin_rad < 0.0:
        raise ConfigurationError("safety.joint_limit_margin_rad must be non-negative.")
    minimum_singularity_score = _number(
        singularity["minimum_score"],
        path,
        "safety.singularity.minimum_score",
    )
    if minimum_singularity_score < 0.0:
        raise ConfigurationError(
            "safety.singularity.minimum_score must be non-negative."
        )
    return SafetyConfig(
        enabled=enabled,
        provenance=provenance,
        max_trajectory_points=max_trajectory_points,
        max_workspace_resample_samples=max_workspace_resample_samples,
        state_timeout_s=positive_fields["state_timeout_s"],
        command_timeout_s=positive_fields["command_timeout_s"],
        stop_timeout_s=positive_fields["stop_timeout_s"],
        max_joint_step_rad=positive_fields["max_joint_step_rad"],
        max_joint_velocity_rad_s=velocity,
        max_joint_acceleration_rad_s2=acceleration,
        joint_limit_margin_rad=joint_limit_margin_rad,
        workspace=WorkspaceConfig(
            minimum_m=minimum,  # type: ignore[arg-type]
            maximum_m=maximum,  # type: ignore[arg-type]
            resample_step_rad=resample_step_rad,
        ),
        singularity=SingularityConfig(
            enabled=_boolean(singularity["enabled"], path, "safety.singularity.enabled"),
            minimum_score=minimum_singularity_score,
        ),
    )


def _parse_logging(path: Path) -> LoggingConfig:
    data = _read_yaml(path)
    _require_version(data, path, "logging")
    logging_data = _mapping(data, "logging", path, "top level")
    _check_keys(
        logging_data,
        {"level", "console", "file", "max_bytes", "backup_count", "json_file"},
        path,
        "logging",
    )
    configured_file_value = logging_data["file"]
    if not isinstance(configured_file_value, str):
        raise ConfigurationError(f"logging.file must be a string in {path}.")
    configured_file = configured_file_value.strip()
    file_path = str(_resolve_path(path, configured_file)) if configured_file else ""
    level = _text(logging_data["level"], path, "logging.level").upper()
    if level not in {"DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"}:
        raise ConfigurationError("logging.level is not supported.")
    max_bytes = _integer(logging_data["max_bytes"], path, "logging.max_bytes")
    backup_count = _integer(
        logging_data["backup_count"], path, "logging.backup_count"
    )
    if max_bytes <= 0 or backup_count < 0:
        raise ConfigurationError(
            "logging.max_bytes must be positive and backup_count non-negative."
        )
    return LoggingConfig(
        level=level,
        console=_boolean(logging_data["console"], path, "logging.console"),
        file=file_path,
        max_bytes=max_bytes,
        backup_count=backup_count,
        json_file=_boolean(logging_data["json_file"], path, "logging.json_file"),
    )


def _capability_state(value: Any, path: Path, location: str) -> CapabilityState:
    if not isinstance(value, str):
        raise ConfigurationError(
            f"{location} must be supported, unsupported, or unverified in {path}."
        )
    try:
        return CapabilityState(value)
    except ValueError as error:
        raise ConfigurationError(
            f"{location} must be supported, unsupported, or unverified in {path}."
        ) from error


def _parse_adapter(
    path: Path,
    sdk: Mapping[str, Any],
    robot: RobotConfig,
) -> AdapterConfig:
    adapter = _mapping(sdk, "adapter", path, "sdk")
    adapter_type = _text(adapter.get("type"), path, "sdk.adapter.type")
    if adapter_type == "mock":
        _check_keys(adapter, {"type", "mock"}, path, "sdk.adapter")
        profile = _mapping(adapter, "mock", path, "sdk.adapter")
        _check_keys(profile, {"initial_position_rad"}, path, "sdk.adapter.mock")
        initial = _float_tuple(profile, "initial_position_rad", path, "sdk.adapter.mock", 6)
        return AdapterConfig(
            adapter_type="mock",
            mock=MockAdapterProfile(initial_position_rad=initial),
        )
    if adapter_type == "replay":
        _check_keys(adapter, {"type", "replay"}, path, "sdk.adapter")
        profile = _mapping(adapter, "replay", path, "sdk.adapter")
        _check_keys(profile, {"replay_file", "loop"}, path, "sdk.adapter.replay")
        return AdapterConfig(
            adapter_type="replay",
            replay=ReplayAdapterProfile(
                replay_file=_resolve_path(path, profile["replay_file"]),
                loop=_boolean(profile["loop"], path, "sdk.adapter.replay.loop"),
            ),
        )
    if adapter_type != "vendor_serial":
        raise ConfigurationError(
            f"sdk.adapter.type must be mock, replay, or vendor_serial in {path}."
        )
    _check_keys(adapter, {"type", "hardware"}, path, "sdk.adapter")
    hardware = _mapping(adapter, "hardware", path, "sdk.adapter")
    _check_keys(
        hardware,
        {
            "serial_by_id",
            "baudrate",
            "operation_deadline_s",
            "max_retries",
            "retry_delay_s",
            "expected_model",
            "mapping_fingerprint",
            "firmware",
            "capabilities",
            "debug",
        },
        path,
        "sdk.adapter.hardware",
    )
    serial_by_id = _text(
        hardware["serial_by_id"],
        path,
        "sdk.adapter.hardware.serial_by_id",
    )
    serial_prefix = "/dev/serial/by-id/"
    serial_identifier = serial_by_id[len(serial_prefix) :]
    if (
        not serial_by_id.startswith(serial_prefix)
        or not serial_identifier
        or "/" in serial_identifier
        or "placeholder" in serial_identifier.lower()
    ):
        raise ConfigurationError(
            f"Real profile serial_by_id must use /dev/serial/by-id/, not {serial_by_id!r}."
        )
    firmware = _mapping(hardware, "firmware", path, "sdk.adapter.hardware")
    _check_keys(
        firmware,
        {"expected_version", "speed"},
        path,
        "sdk.adapter.hardware.firmware",
    )
    expected_version = _text(
        firmware["expected_version"],
        path,
        "sdk.adapter.hardware.firmware.expected_version",
    )
    if not expected_version or "placeholder" in expected_version.lower():
        raise ConfigurationError("A real firmware expected_version is mandatory.")
    speed = _integer(
        firmware["speed"],
        path,
        "sdk.adapter.hardware.firmware.speed",
    )
    if speed < 1 or speed > 100:
        raise ConfigurationError("Firmware speed must be in the range 1..100.")
    expected_model = _text(
        hardware["expected_model"],
        path,
        "sdk.adapter.hardware.expected_model",
    )
    if not expected_model or "placeholder" in expected_model.lower():
        raise ConfigurationError("A real hardware expected_model is mandatory.")
    configured_mapping_fingerprint = _fingerprint(
        hardware["mapping_fingerprint"],
        path,
        "sdk.adapter.hardware.mapping_fingerprint",
    )
    actual_mapping_fingerprint = mapping_fingerprint(robot.joint_mapping)
    if configured_mapping_fingerprint != actual_mapping_fingerprint:
        raise ConfigurationError(
            "Hardware mapping fingerprint does not match robot.joint_mapping."
        )
    capabilities = _mapping(hardware, "capabilities", path, "sdk.adapter.hardware")
    capability_keys = {
        "verification_reference",
        "stop",
        "pause",
        "resume",
        "power_control",
    }
    _check_keys(
        capabilities,
        capability_keys,
        path,
        "sdk.adapter.hardware.capabilities",
    )
    capability_states = {
        "stop": _capability_state(capabilities["stop"], path, "capabilities.stop"),
        "pause": _capability_state(capabilities["pause"], path, "capabilities.pause"),
        "resume": _capability_state(
            capabilities["resume"], path, "capabilities.resume"
        ),
        "power_control": _capability_state(
            capabilities["power_control"], path, "capabilities.power_control"
        ),
    }
    reference_raw = capabilities["verification_reference"]
    if not isinstance(reference_raw, str):
        raise ConfigurationError(
            "capabilities.verification_reference must be a string "
            f"in {path}."
        )
    verification_reference = reference_raw.strip()
    has_supported_capability = any(
        state is CapabilityState.SUPPORTED for state in capability_states.values()
    )
    reference_is_placeholder = (
        not verification_reference
        or any(
            token in verification_reference.lower()
            for token in ("placeholder", "changeme", "todo", "tbd")
        )
    )
    if has_supported_capability and reference_is_placeholder:
        raise ConfigurationError(
            "A non-placeholder capabilities.verification_reference is required "
            "when any real-hardware capability is supported."
        )

    operation_deadline_s = _number(
        hardware["operation_deadline_s"],
        path,
        "sdk.adapter.hardware.operation_deadline_s",
    )
    max_retries = _integer(
        hardware["max_retries"],
        path,
        "sdk.adapter.hardware.max_retries",
    )
    retry_delay_s = _number(
        hardware["retry_delay_s"], path, "sdk.adapter.hardware.retry_delay_s"
    )
    if operation_deadline_s <= 0.0 or max_retries < 0 or retry_delay_s < 0.0:
        raise ConfigurationError("Hardware deadlines/retry settings are invalid.")
    baudrate = _integer(
        hardware["baudrate"],
        path,
        "sdk.adapter.hardware.baudrate",
    )
    if baudrate <= 0:
        raise ConfigurationError("Hardware baudrate must be positive.")
    return AdapterConfig(
        adapter_type="vendor_serial",
        hardware=HardwareProfile(
            serial_by_id=serial_by_id,
            baudrate=baudrate,
            operation_deadline_s=operation_deadline_s,
            max_retries=max_retries,
            retry_delay_s=retry_delay_s,
            expected_model=expected_model,
            mapping_fingerprint=configured_mapping_fingerprint,
            firmware=FirmwareProtocolProfile(
                expected_version=expected_version,
                speed=speed,
            ),
            capabilities=CapabilityVerificationProfile(
                verification_reference=verification_reference,
                stop=capability_states["stop"],
                pause=capability_states["pause"],
                resume=capability_states["resume"],
                power_control=capability_states["power_control"],
            ),
            debug=_boolean(hardware["debug"], path, "sdk.adapter.hardware.debug"),
        ),
    )


def load_sdk_config(config_path: str) -> SdkConfig:
    """Load one strict v0.2 manifest without opening any hardware resource."""
    path = Path(config_path).expanduser().resolve()
    data = _read_yaml(path)
    _require_version(data, path, "sdk")
    sdk = _mapping(data, "sdk", path, "top level")
    _check_keys(sdk, {"config_files", "adapter"}, path, "sdk")
    files = _mapping(sdk, "config_files", path, "sdk")
    _check_keys(files, {"robot", "safety", "logging"}, path, "sdk.config_files")
    robot = _parse_robot(_resolve_path(path, files["robot"]))
    return SdkConfig(
        config_version=CONFIG_VERSION,
        source_path=path,
        robot=robot,
        safety=_parse_safety(_resolve_path(path, files["safety"])),
        logging=_parse_logging(_resolve_path(path, files["logging"])),
        adapter=_parse_adapter(path, sdk, robot),
    )
