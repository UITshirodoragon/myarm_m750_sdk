from pathlib import Path
from typing import Any, Dict, Sequence

import pytest
import yaml
from myarm_m750_core import CameraSessionBuilder
from myarm_m750_core.domain.errors import (
    ConfigurationError,
    ConfigurationMigrationError,
)
from myarm_m750_core.runtime.config import (
    camera_config_by_name,
    camera_loader,
    load_camera_configs,
    load_sdk_config,
    loader,
)


def _read_mapping(path: Path) -> Dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert isinstance(data, dict)
    return data


def _write_mapping(path: Path, data: Dict[str, Any]) -> None:
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")


def _set_nested(data: Dict[str, Any], keys: Sequence[str], value: Any) -> None:
    current = data
    for key in keys[:-1]:
        child = current[key]
        assert isinstance(child, dict)
        current = child
    current[keys[-1]] = value


def _sdk_manifest(
    tmp_path: Path,
    repository_root: Path,
    *,
    robot: Path = None,
    safety: Path = None,
    logging_config: Path = None,
    manifest_data: Dict[str, Any] = None,
) -> Path:
    config_root = repository_root / "pycore/config"
    data = (
        _read_mapping(config_root / "default.yaml")
        if manifest_data is None
        else manifest_data
    )
    files = data["sdk"]["config_files"]
    assert isinstance(files, dict)
    files.update(
        {
            "robot": str(robot or config_root / "robot_m750.yaml"),
            "safety": str(safety or config_root / "safety.yaml"),
            "logging": str(logging_config or config_root / "logging.yaml"),
        }
    )
    path = tmp_path / "sdk.yaml"
    _write_mapping(path, data)
    return path


def _mutated_component(
    tmp_path: Path,
    repository_root: Path,
    filename: str,
    keys: Sequence[str],
    value: Any,
) -> Path:
    original = repository_root / "pycore/config" / filename
    data = _read_mapping(original)
    if filename == "robot_m750.yaml":
        model = data["robot"]["model"]
        assert isinstance(model, dict)
        model["urdf_path"] = str(
            repository_root / "pycore/src/resources/myarm_m750_kinematic.urdf"
        )
    _set_nested(data, keys, value)
    target = tmp_path / filename
    _write_mapping(target, data)
    return target


def _camera_config(
    tmp_path: Path,
    repository_root: Path,
    *,
    data: Dict[str, Any] = None,
    calibration_path: Path = None,
) -> Path:
    source = repository_root / "pycore/config/camera/cameras_mock.yaml"
    configured = _read_mapping(source) if data is None else data
    cameras = configured.get("cameras")
    if isinstance(cameras, dict):
        for camera in cameras.values():
            if isinstance(camera, dict):
                camera["calibration_file"] = str(
                    calibration_path
                    or repository_root
                    / "pycore/config/camera/calibration/mock_640x480.yaml"
                )
    target = tmp_path / "cameras.yaml"
    target.parent.mkdir(parents=True, exist_ok=True)
    _write_mapping(target, configured)
    return target


def test_every_shipped_config_yaml_has_one_explicit_owner(
    repository_root: Path,
) -> None:
    config_root = repository_root / "pycore/config"
    shipped = {
        path.relative_to(config_root).as_posix()
        for path in config_root.rglob("*.yaml")
    }
    assert shipped == {
        "camera/calibration/mock_640x480.yaml",
        "camera/cameras_mock.yaml",
        "camera/cameras_real.example.yaml",
        "default.yaml",
        "default_real.example.yaml",
        "default_replay.yaml",
        "logging.yaml",
        "robot_m750.yaml",
        "safety.yaml",
    }

    assert load_sdk_config(str(config_root / "default.yaml")).adapter.adapter_type == "mock"
    assert (
        load_sdk_config(str(config_root / "default_replay.yaml")).adapter.adapter_type
        == "replay"
    )
    assert loader._parse_robot(config_root / "robot_m750.yaml").name == "myarm_m750"
    assert loader._parse_safety(config_root / "safety.yaml").enabled
    assert loader._parse_logging(config_root / "logging.yaml").level == "INFO"
    assert len(
        load_camera_configs(str(config_root / "camera/cameras_mock.yaml"))
    ) == 2
    assert (
        load_camera_configs(str(config_root / "camera/cameras_real.example.yaml"))
        == ()
    )
    assert (
        camera_loader._load_calibration(
            config_root / "camera/calibration/mock_640x480.yaml"
        ).image_width_px
        == 640
    )

    with pytest.raises(ConfigurationError, match=r"serial_by_id.*non-empty"):
        load_sdk_config(str(config_root / "default_real.example.yaml"))
    with pytest.raises(ConfigurationError, match="no enabled, deployable cameras"):
        CameraSessionBuilder.from_file(
            str(config_root / "camera/cameras_real.example.yaml")
        ).build()


@pytest.mark.parametrize(
    ("filename", "parser"),
    [
        ("robot_m750.yaml", loader._parse_robot),
        ("safety.yaml", loader._parse_safety),
        ("logging.yaml", loader._parse_logging),
    ],
)
def test_legacy_component_yaml_has_a_clear_migration_error(
    tmp_path: Path,
    repository_root: Path,
    filename: str,
    parser: Any,
) -> None:
    document = _read_mapping(repository_root / "pycore/config" / filename)
    document.pop("config_version")
    legacy_path = tmp_path / filename
    _write_mapping(legacy_path, document)

    with pytest.raises(
        ConfigurationMigrationError,
        match=r"Legacy configuration.*Add config_version: 1",
    ):
        parser(legacy_path)


def test_loader_primitives_report_boundary_context(tmp_path: Path) -> None:
    missing = tmp_path / "missing.yaml"
    with pytest.raises(ConfigurationError, match="does not exist"):
        loader._read_yaml(missing)

    malformed = tmp_path / "malformed.yaml"
    malformed.write_text("sdk: [\n", encoding="utf-8")
    with pytest.raises(ConfigurationError, match="Invalid YAML"):
        loader._read_yaml(malformed)

    sequence_root = tmp_path / "sequence.yaml"
    sequence_root.write_text("- one\n- two\n", encoding="utf-8")
    with pytest.raises(ConfigurationError, match="Top-level YAML"):
        loader._read_yaml(sequence_root)

    with pytest.raises(ConfigurationError, match=r"missing=required; unknown=extra"):
        loader._check_keys(
            {"known": 1, "extra": 2},
            {"known", "required"},
            sequence_root,
            "unit",
        )
    with pytest.raises(ConfigurationMigrationError, match="Unsupported"):
        loader._require_version({"config_version": 99, "sdk": {}}, sequence_root, "sdk")
    with pytest.raises(ConfigurationMigrationError, match="must be the integer"):
        loader._require_version(
            {"config_version": True, "sdk": {}},
            sequence_root,
            "sdk",
        )
    with pytest.raises(ConfigurationError, match="must be a mapping"):
        loader._mapping({"item": []}, "item", sequence_root, "unit")
    with pytest.raises(ConfigurationError, match="exactly 2"):
        loader._sequence({"items": [1]}, "items", sequence_root, "unit", 2)

    for value in (True, "1.0", object(), float("inf")):
        with pytest.raises(ConfigurationError, match="must be"):
            loader._number(value, sequence_root, "unit.number")
    for value in (True, 1.0, "1"):
        with pytest.raises(ConfigurationError, match="integer"):
            loader._integer(value, sequence_root, "unit.integer")
    with pytest.raises(ConfigurationError, match="non-empty string"):
        loader._text(1, sequence_root, "unit.text")
    with pytest.raises(ConfigurationError, match="boolean"):
        loader._boolean("true", sequence_root, "unit.flag")
    with pytest.raises(ConfigurationError, match="non-empty"):
        loader._resolve_path(sequence_root, "")
    absolute = tmp_path.resolve()
    assert loader._resolve_path(sequence_root, str(absolute)) == absolute
    with pytest.raises(ConfigurationError, match="SHA-256"):
        loader._fingerprint("A" * 64, sequence_root, "unit.fingerprint")
    with pytest.raises(ConfigurationError, match="SHA-256"):
        loader._fingerprint(int("1" * 64), sequence_root, "unit.fingerprint")
    with pytest.raises(ConfigurationError, match="supported"):
        loader._capability_state("maybe", sequence_root, "unit.capability")


@pytest.mark.parametrize(
    ("keys", "value", "message"),
    [
        (("robot", "joint_names"), ["duplicate"] * 6, "six unique"),
        (("robot", "model", "urdf_path"), "/does/not/exist.urdf", "does not exist"),
        (("robot", "model", "resource_sha256"), "0" * 64, "fingerprint mismatch"),
        (
            ("robot", "joint_mapping", "shoulder_pan_joint", "direction"),
            0,
            "must be -1 or 1",
        ),
        (
            ("robot", "joint_mapping", "shoulder_pan_joint", "direction"),
            1.0,
            "must be an integer",
        ),
        (
            ("robot", "model", "kinematic_contract_sha256"),
            "0" * 64,
            "kinematic contract fingerprint mismatch",
        ),
        (("robot", "name"), 750, "non-empty string"),
        (("robot", "model", "base_link"), "", "non-empty string"),
        (("robot", "model", "end_link"), "base_link", "must be distinct"),
        (("robot", "runtime", "command_rate_hz"), 0, "rates must be positive"),
        (("robot", "runtime", "command_rate_hz"), "5.0", "must be numeric"),
    ],
)
def test_robot_schema_rejects_invalid_boundaries_before_io(
    tmp_path: Path,
    repository_root: Path,
    keys: Sequence[str],
    value: Any,
    message: str,
) -> None:
    robot = _mutated_component(tmp_path, repository_root, "robot_m750.yaml", keys, value)
    manifest = _sdk_manifest(tmp_path, repository_root, robot=robot)
    with pytest.raises(ConfigurationError, match=message):
        load_sdk_config(str(manifest))


@pytest.mark.parametrize(
    ("keys", "value", "message"),
    [
        (
            ("safety", "max_joint_velocity_rad_s"),
            [0.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            "limits must be positive",
        ),
        (
            ("safety", "workspace", "minimum_m"),
            [0.70, -0.70, -0.10],
            "minimum values",
        ),
        (("safety", "state_timeout_s"), 0.0, "timeouts"),
        (("safety", "max_trajectory_points"), 0, "budgets must be positive"),
        (
            ("safety", "max_workspace_resample_samples"),
            True,
            "must be an integer",
        ),
        (("safety", "workspace", "resample_step_rad"), 0.0, "must be positive"),
        (("safety", "provenance"), "", "non-empty string"),
        (("safety", "enabled"), False, "mandatory trajectory safety"),
        (("safety", "joint_limit_margin_rad"), -0.01, "must be non-negative"),
        (
            ("safety", "singularity", "minimum_score"),
            -0.01,
            "must be non-negative",
        ),
        (("safety", "state_timeout_s"), True, "must be numeric"),
    ],
)
def test_safety_schema_rejects_invalid_boundaries(
    tmp_path: Path,
    repository_root: Path,
    keys: Sequence[str],
    value: Any,
    message: str,
) -> None:
    safety = _mutated_component(tmp_path, repository_root, "safety.yaml", keys, value)
    manifest = _sdk_manifest(tmp_path, repository_root, safety=safety)
    with pytest.raises(ConfigurationError, match=message):
        load_sdk_config(str(manifest))


def _valid_vendor_manifest(repository_root: Path) -> Dict[str, Any]:
    data = _read_mapping(repository_root / "pycore/config/default_real.example.yaml")
    hardware = data["sdk"]["adapter"]["hardware"]
    assert isinstance(hardware, dict)
    hardware["serial_by_id"] = "/dev/serial/by-id/usb-test-arm"
    hardware["expected_model"] = "myarm_m750"
    firmware = hardware["firmware"]
    assert isinstance(firmware, dict)
    firmware["expected_version"] = "1.2.3"
    return data


@pytest.mark.parametrize(
    ("keys", "value", "message"),
    [
        (("logging", "max_bytes"), 1.5, "must be an integer"),
        (("logging", "max_bytes"), True, "must be an integer"),
        (("logging", "max_bytes"), 0, "must be positive"),
        (("logging", "backup_count"), -1, "non-negative"),
        (("logging", "level"), "VERBOSE", "not supported"),
        (("logging", "file"), 7, "must be a string"),
    ],
)
def test_logging_schema_rejects_coercion_and_invalid_ranges(
    tmp_path: Path,
    repository_root: Path,
    keys: Sequence[str],
    value: Any,
    message: str,
) -> None:
    logging_config = _mutated_component(
        tmp_path,
        repository_root,
        "logging.yaml",
        keys,
        value,
    )
    manifest = _sdk_manifest(
        tmp_path,
        repository_root,
        logging_config=logging_config,
    )
    with pytest.raises(ConfigurationError, match=message):
        load_sdk_config(str(manifest))


def test_replay_and_complete_vendor_profiles_are_schema_valid(
    tmp_path: Path, repository_root: Path
) -> None:
    replay = load_sdk_config(str(repository_root / "pycore/config/default_replay.yaml"))
    assert replay.adapter.adapter_type == "replay"
    assert replay.adapter.replay is not None
    assert replay.adapter.replay.replay_file.is_absolute()
    assert replay.adapter.replay.loop

    vendor_path = _sdk_manifest(
        tmp_path,
        repository_root,
        manifest_data=_valid_vendor_manifest(repository_root),
    )
    vendor = load_sdk_config(str(vendor_path))
    assert vendor.adapter.adapter_type == "vendor_serial"
    assert vendor.adapter.hardware is not None
    assert vendor.adapter.hardware.firmware.expected_version == "1.2.3"
    assert vendor.adapter.hardware.capabilities.verification_reference == ""


@pytest.mark.parametrize(
    ("keys", "value", "message"),
    [
        (("expected_model",), "", "expected_model"),
        (("firmware", "expected_version"), "placeholder", "mandatory"),
        (("firmware", "speed"), 101, "range 1..100"),
        (("mapping_fingerprint",), "0" * 64, "does not match"),
        (("capabilities", "stop"), "maybe", "supported"),
        (
            ("capabilities", "verification_reference"),
            123,
            "must be a string",
        ),
        (("operation_deadline_s",), 0.0, "retry settings"),
        (("operation_deadline_s",), "0.1", "must be numeric"),
        (("baudrate",), 1_000_000.0, "must be an integer"),
        (("baudrate",), True, "must be an integer"),
        (("baudrate",), 0, "must be positive"),
        (("max_retries",), 1.0, "must be an integer"),
        (("max_retries",), True, "must be an integer"),
        (("max_retries",), -1, "retry settings"),
        (("firmware", "speed"), 30.0, "must be an integer"),
        (("firmware", "speed"), True, "must be an integer"),
        (("serial_by_id",), "/dev/serial/by-id/", "must use"),
        (
            ("serial_by_id",),
            "/dev/serial/by-id/placeholder",
            "must use",
        ),
        (("debug",), 0, "must be boolean"),
    ],
)
def test_vendor_profile_rejects_unverified_boundaries(
    tmp_path: Path,
    repository_root: Path,
    keys: Sequence[str],
    value: Any,
    message: str,
) -> None:
    data = _valid_vendor_manifest(repository_root)
    hardware = data["sdk"]["adapter"]["hardware"]
    assert isinstance(hardware, dict)
    _set_nested(hardware, keys, value)
    manifest = _sdk_manifest(tmp_path, repository_root, manifest_data=data)
    with pytest.raises(ConfigurationError, match=message):
        load_sdk_config(str(manifest))


def test_supported_real_capability_requires_traceable_reference(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    data = _valid_vendor_manifest(repository_root)
    capabilities = data["sdk"]["adapter"]["hardware"]["capabilities"]
    assert isinstance(capabilities, dict)
    capabilities["stop"] = "supported"

    missing_reference = _sdk_manifest(
        tmp_path,
        repository_root,
        manifest_data=data,
    )
    with pytest.raises(ConfigurationError, match="non-placeholder"):
        load_sdk_config(str(missing_reference))

    capabilities["verification_reference"] = "HIL-STOP-2026-001"
    referenced = _sdk_manifest(
        tmp_path,
        repository_root,
        manifest_data=data,
    )
    config = load_sdk_config(str(referenced))
    assert config.adapter.hardware is not None
    assert (
        config.adapter.hardware.capabilities.verification_reference
        == "HIL-STOP-2026-001"
    )


def test_unknown_adapter_type_is_rejected(tmp_path: Path, repository_root: Path) -> None:
    data = _read_mapping(repository_root / "pycore/config/default.yaml")
    data["sdk"]["adapter"] = {"type": "unknown"}
    manifest = _sdk_manifest(tmp_path, repository_root, manifest_data=data)
    with pytest.raises(ConfigurationError, match="must be mock, replay"):
        load_sdk_config(str(manifest))


def test_camera_loader_primitives_report_boundary_context(tmp_path: Path) -> None:
    missing = tmp_path / "missing.yaml"
    with pytest.raises(ConfigurationError, match="does not exist"):
        camera_loader._read(missing)

    malformed = tmp_path / "malformed.yaml"
    malformed.write_text("cameras: [\n", encoding="utf-8")
    with pytest.raises(ConfigurationError, match="Invalid camera YAML"):
        camera_loader._read(malformed)

    sequence_root = tmp_path / "sequence.yaml"
    sequence_root.write_text("- camera\n", encoding="utf-8")
    with pytest.raises(ConfigurationError, match="root must be a mapping"):
        camera_loader._read(sequence_root)

    with pytest.raises(ConfigurationError, match="fields invalid"):
        camera_loader._keys(
            {"known": 1, "extra": 2},
            {"known", "required"},
            sequence_root,
            "unit",
        )
    with pytest.raises(ConfigurationError, match="must be a mapping"):
        camera_loader._mapping({"item": []}, "item", sequence_root, "unit")
    with pytest.raises(ConfigurationError, match="contain 2 values"):
        camera_loader._sequence({"items": [1]}, "items", sequence_root, "unit", 2)
    with pytest.raises(ConfigurationError, match="finite"):
        camera_loader._numbers({"items": [float("nan")]}, "items", sequence_root, "unit", 1)
    for value in (True, "1.0"):
        with pytest.raises(ConfigurationError, match="numeric"):
            camera_loader._number(value, sequence_root, "unit.number")
    for value in (True, 1.0, "1"):
        with pytest.raises(ConfigurationError, match="integer"):
            camera_loader._integer(value, sequence_root, "unit.integer")
    with pytest.raises(ConfigurationError, match="boolean"):
        camera_loader._boolean("false", sequence_root, "unit.enabled")
    with pytest.raises(ConfigurationError, match="must be 3x3"):
        camera_loader._matrix_data(
            {"camera_matrix": {"rows": 2, "cols": 3, "data": [0.0] * 6}},
            "camera_matrix",
            3,
            3,
            sequence_root,
        )


def test_camera_root_and_entry_boundaries_are_rejected(
    tmp_path: Path, repository_root: Path
) -> None:
    original = _read_mapping(repository_root / "pycore/config/camera/cameras_mock.yaml")

    cases = []
    missing_version = dict(original)
    missing_version.pop("config_version")
    cases.append((missing_version, ConfigurationMigrationError, "Legacy"))

    unsupported_version = dict(original)
    unsupported_version["config_version"] = 2
    cases.append((unsupported_version, ConfigurationMigrationError, "Unsupported"))

    boolean_version = dict(original)
    boolean_version["config_version"] = True
    cases.append((boolean_version, ConfigurationMigrationError, "must be the integer"))

    invalid_profile = dict(original)
    invalid_profile["profile"] = "simulation"
    cases.append((invalid_profile, ConfigurationError, "profile"))

    invalid_cameras = dict(original)
    invalid_cameras["cameras"] = []
    cases.append((invalid_cameras, ConfigurationError, "must be a mapping"))

    for index, (data, exception_type, message) in enumerate(cases):
        path = _camera_config(tmp_path / str(index), repository_root, data=data)
        with pytest.raises(exception_type, match=message):
            load_camera_configs(str(path))

    camera_not_mapping = _read_mapping(
        repository_root / "pycore/config/camera/cameras_mock.yaml"
    )
    camera_not_mapping["cameras"] = {"bad_camera": []}
    path = _camera_config(tmp_path / "entry", repository_root, data=camera_not_mapping)
    with pytest.raises(ConfigurationError, match="entry must be a mapping"):
        load_camera_configs(str(path))

    backend_mismatch = _read_mapping(
        repository_root / "pycore/config/camera/cameras_mock.yaml"
    )
    first = next(iter(backend_mismatch["cameras"].values()))
    first["backend"] = "opencv"
    path = _camera_config(tmp_path / "backend", repository_root, data=backend_mismatch)
    with pytest.raises(ConfigurationError, match="backend"):
        load_camera_configs(str(path))

    identity_not_mapping = _read_mapping(
        repository_root / "pycore/config/camera/cameras_mock.yaml"
    )
    first = next(iter(identity_not_mapping["cameras"].values()))
    first["identity"] = []
    path = _camera_config(tmp_path / "identity", repository_root, data=identity_not_mapping)
    with pytest.raises(ConfigurationError, match="identity must be a mapping"):
        load_camera_configs(str(path))

    enabled_not_boolean = _read_mapping(
        repository_root / "pycore/config/camera/cameras_mock.yaml"
    )
    first = next(iter(enabled_not_boolean["cameras"].values()))
    first["enabled"] = "false"
    path = _camera_config(
        tmp_path / "enabled",
        repository_root,
        data=enabled_not_boolean,
    )
    with pytest.raises(ConfigurationError, match="enabled must be boolean"):
        load_camera_configs(str(path))


def test_real_camera_schema_and_name_lookup(tmp_path: Path, repository_root: Path) -> None:
    data = _read_mapping(repository_root / "pycore/config/camera/cameras_mock.yaml")
    data["profile"] = "real"
    for camera in data["cameras"].values():
        camera["backend"] = "opencv"
        camera["device_by_id"] = "/dev/v4l/by-id/test-camera"
    path = _camera_config(tmp_path, repository_root, data=data)

    configs = load_camera_configs(str(path))
    assert configs[0].backend == "opencv"
    assert camera_config_by_name(str(path), "mock_wrist_01") == configs[0]
    with pytest.raises(ConfigurationError, match="not defined"):
        camera_config_by_name(str(path), "missing")


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("hardware_name", "ROS-name safe"),
        ("model", "identity.model"),
        ("role", "role must be a non-empty string"),
        ("same_frames", "must be distinct"),
        ("extrinsic_child", "must equal"),
    ],
)
def test_camera_cross_field_contracts_are_rejected(
    tmp_path: Path,
    repository_root: Path,
    mutation: str,
    message: str,
) -> None:
    data = _read_mapping(
        repository_root / "pycore/config/camera/cameras_mock.yaml"
    )
    cameras = data["cameras"]
    assert isinstance(cameras, dict)
    hardware_name, camera = next(iter(cameras.items()))
    assert isinstance(camera, dict)
    if mutation == "hardware_name":
        cameras["bad/name"] = cameras.pop(hardware_name)
    elif mutation == "model":
        camera["identity"]["model"] = ""
    elif mutation == "role":
        camera["role"] = ""
    elif mutation == "same_frames":
        camera["frames"]["optical_frame"] = camera["frames"]["camera_frame"]
    else:
        camera["extrinsics"]["child_frame"] = "another_camera_link"

    path = _camera_config(tmp_path, repository_root, data=data)
    with pytest.raises(ConfigurationError, match=message):
        load_camera_configs(str(path))


@pytest.mark.parametrize(
    ("keys", "value", "message"),
    [
        (("stream", "width"), 640.0, "must be an integer"),
        (("stream", "height"), True, "must be an integer"),
        (("stream", "width"), 0, "must be positive"),
        (("stream", "fps"), "15", "must be numeric"),
        (("stream", "fps"), True, "must be numeric"),
        (("stream", "pixel_format"), "yuyv", "Unsupported pixel_format"),
        (("reconnect", "maximum_attempts"), 5.0, "must be an integer"),
        (("reconnect", "maximum_attempts"), True, "must be an integer"),
        (("reconnect", "maximum_attempts"), -1, "settings are invalid"),
        (("reconnect", "multiplier"), 0.5, "settings are invalid"),
        (("frames", "camera_frame"), 7, "non-empty string"),
        (("identity", "serial"), 7, "non-empty string"),
    ],
)
def test_camera_schema_rejects_coercion_and_invalid_ranges(
    tmp_path: Path,
    repository_root: Path,
    keys: Sequence[str],
    value: Any,
    message: str,
) -> None:
    data = _read_mapping(
        repository_root / "pycore/config/camera/cameras_mock.yaml"
    )
    first = next(iter(data["cameras"].values()))
    assert isinstance(first, dict)
    _set_nested(first, keys, value)
    path = _camera_config(tmp_path, repository_root, data=data)
    with pytest.raises(ConfigurationError, match=message):
        load_camera_configs(str(path))


@pytest.mark.parametrize(
    ("device_by_id", "message"),
    [
        ("/dev/v4l/by-id/", "stable"),
        ("/dev/v4l/by-id/placeholder", "stable"),
        ("/dev/video0", "stable"),
        (7, "must be a string"),
    ],
)
def test_real_camera_requires_concrete_stable_by_id(
    tmp_path: Path,
    repository_root: Path,
    device_by_id: Any,
    message: str,
) -> None:
    data = _read_mapping(
        repository_root / "pycore/config/camera/cameras_mock.yaml"
    )
    data["profile"] = "real"
    for camera in data["cameras"].values():
        camera["backend"] = "opencv"
        camera["device_by_id"] = "/dev/v4l/by-id/valid-camera"
    first = next(iter(data["cameras"].values()))
    first["device_by_id"] = device_by_id
    path = _camera_config(tmp_path, repository_root, data=data)
    with pytest.raises(ConfigurationError, match=message):
        load_camera_configs(str(path))


@pytest.mark.parametrize(
    ("keys", "value", "message"),
    [
        (("image_width",), 640.0, "must be an integer"),
        (("image_height",), True, "must be an integer"),
        (("camera_matrix", "rows"), 3.0, "must be an integer"),
        (("distortion_coefficients", "cols"), 0, "must be positive"),
        (("camera_matrix", "data", 0), True, "must be numeric"),
        (("camera_name",), "", "non-empty string"),
    ],
)
def test_calibration_schema_rejects_coercion_and_invalid_ranges(
    tmp_path: Path,
    repository_root: Path,
    keys: Sequence[Any],
    value: Any,
    message: str,
) -> None:
    source = (
        repository_root
        / "pycore/config/camera/calibration/mock_640x480.yaml"
    )
    calibration = _read_mapping(source)
    current: Any = calibration
    for key in keys[:-1]:
        current = current[key]
    current[keys[-1]] = value
    calibration_path = tmp_path / "calibration.yaml"
    _write_mapping(calibration_path, calibration)
    camera_path = _camera_config(
        tmp_path,
        repository_root,
        calibration_path=calibration_path,
    )
    with pytest.raises(ConfigurationError, match=message):
        load_camera_configs(str(camera_path))
