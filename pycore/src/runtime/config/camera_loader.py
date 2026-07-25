"""Strict versioned camera and calibration YAML loading."""

from __future__ import annotations

import math
import re
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence, Set, Tuple

import yaml
from myarm_m750_core.domain.camera import (
    CameraCalibration,
    CameraConfig,
    CameraExtrinsics,
    CameraReconnectPolicy,
    CameraStreamConfig,
)
from myarm_m750_core.domain.errors import (
    ConfigurationError,
    ConfigurationMigrationError,
)
from myarm_m750_core.runtime.config.models import CONFIG_VERSION

_ROS_SAFE_HARDWARE_NAME = re.compile(r"^[a-z][a-z0-9_]*$")


def _read(path: Path) -> Mapping[str, Any]:
    if not path.is_file():
        raise ConfigurationError(f"Camera YAML does not exist: {path}")
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except yaml.YAMLError as error:
        raise ConfigurationError(f"Invalid camera YAML: {error}") from error
    if not isinstance(data, dict):
        raise ConfigurationError("Camera YAML root must be a mapping.")
    return data


def _keys(
    data: Mapping[str, Any],
    required: Set[str],
    source: Path,
    location: str,
) -> None:
    missing = sorted(required - set(data))
    unknown = sorted(set(data) - required)
    if missing or unknown:
        raise ConfigurationError(
            f"{location} fields invalid in {source}: missing={missing}, unknown={unknown}."
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
        raise ConfigurationError(f"{location}.{key} must contain {size} values.")
    return value


def _numbers(
    data: Mapping[str, Any],
    key: str,
    source: Path,
    location: str,
    size: int,
) -> Tuple[float, ...]:
    return tuple(
        _number(value, source, f"{location}.{key}")
        for value in _sequence(data, key, source, location, size)
    )


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


def _matrix_data(
    calibration: Mapping[str, Any],
    key: str,
    rows: int,
    columns: int,
    source: Path,
) -> Tuple[float, ...]:
    matrix = _mapping(calibration, key, source, "calibration")
    _keys(matrix, {"rows", "cols", "data"}, source, f"calibration.{key}")
    actual_rows = _integer(matrix["rows"], source, f"calibration.{key}.rows")
    actual_columns = _integer(matrix["cols"], source, f"calibration.{key}.cols")
    if actual_rows != rows or actual_columns != columns:
        raise ConfigurationError(f"Calibration {key} must be {rows}x{columns}.")
    return _numbers(
        matrix,
        "data",
        source,
        f"calibration.{key}",
        rows * columns,
    )


def _load_calibration(path: Path) -> CameraCalibration:
    data = _read(path)
    required = {
        "image_width",
        "image_height",
        "camera_name",
        "camera_matrix",
        "distortion_model",
        "distortion_coefficients",
        "rectification_matrix",
        "projection_matrix",
    }
    _keys(data, required, path, "calibration")
    distortion = _mapping(data, "distortion_coefficients", path, "calibration")
    _keys(
        distortion,
        {"rows", "cols", "data"},
        path,
        "calibration.distortion_coefficients",
    )
    rows = _integer(
        distortion["rows"],
        path,
        "calibration.distortion_coefficients.rows",
    )
    columns = _integer(
        distortion["cols"],
        path,
        "calibration.distortion_coefficients.cols",
    )
    if rows <= 0 or columns <= 0:
        raise ConfigurationError(
            "Calibration distortion dimensions must be positive."
        )
    image_width_px = _integer(data["image_width"], path, "calibration.image_width")
    image_height_px = _integer(data["image_height"], path, "calibration.image_height")
    if image_width_px <= 0 or image_height_px <= 0:
        raise ConfigurationError("Calibration image dimensions must be positive.")
    _text(data["camera_name"], path, "calibration.camera_name")
    distortion_model = _text(
        data["distortion_model"],
        path,
        "calibration.distortion_model",
    )
    try:
        return CameraCalibration(
            image_width_px=image_width_px,
            image_height_px=image_height_px,
            camera_matrix=_matrix_data(data, "camera_matrix", 3, 3, path),
            distortion_model=distortion_model,
            distortion_coefficients=_numbers(
                distortion,
                "data",
                path,
                "calibration.distortion_coefficients",
                rows * columns,
            ),
            rectification_matrix=_matrix_data(
                data, "rectification_matrix", 3, 3, path
            ),
            projection_matrix=_matrix_data(data, "projection_matrix", 3, 4, path),
            source_path=path,
        )
    except ValueError as error:
        raise ConfigurationError(f"Invalid camera calibration: {error}") from error


def _resolve(base: Path, configured: Any) -> Path:
    value = Path(_text(configured, base, "Configured camera path")).expanduser()
    return value.resolve() if value.is_absolute() else (base.parent / value).resolve()


def load_camera_configs(config_path: str) -> Tuple[CameraConfig, ...]:
    """Load strict camera definitions without importing OpenCV or ROS."""
    source = Path(config_path).expanduser().resolve()
    root = _read(source)
    if "config_version" not in root:
        raise ConfigurationMigrationError(
            "Legacy camera YAML is rejected; migrate to config_version: 1."
        )
    _keys(root, {"config_version", "profile", "cameras"}, source, "root")
    version = root["config_version"]
    if isinstance(version, bool) or not isinstance(version, int):
        raise ConfigurationMigrationError(
            f"Camera config_version must be the integer {CONFIG_VERSION}."
        )
    if version != CONFIG_VERSION:
        raise ConfigurationMigrationError(
            f"Unsupported camera config_version {version!r}."
        )
    profile = _text(root["profile"], source, "profile")
    if profile not in ("mock", "real"):
        raise ConfigurationError("Camera profile must be mock or real.")
    cameras = _mapping(root, "cameras", source, "root")
    result = []
    for hardware_name, camera_value in cameras.items():
        if (
            not isinstance(hardware_name, str)
            or _ROS_SAFE_HARDWARE_NAME.fullmatch(hardware_name) is None
        ):
            raise ConfigurationError(
                "Camera hardware names must be lower_snake_case and ROS-name safe."
            )
        if not isinstance(camera_value, dict):
            raise ConfigurationError("Each camera entry must be a mapping.")
        camera = camera_value
        _keys(
            camera,
            {
                "enabled",
                "backend",
                "identity",
                "role",
                "device_by_id",
                "stream",
                "frames",
                "calibration_file",
                "extrinsics",
                "reconnect",
            },
            source,
            f"cameras.{hardware_name}",
        )
        backend = _text(
            camera["backend"],
            source,
            f"cameras.{hardware_name}.backend",
        )
        if (profile == "mock" and backend != "mock") or (
            profile == "real" and backend != "opencv"
        ):
            raise ConfigurationError("Camera backend does not match the selected profile.")
        identity = _mapping(camera, "identity", source, "camera")
        _keys(identity, {"model", "serial"}, source, "camera.identity")
        hardware_model = _text(
            identity["model"],
            source,
            f"cameras.{hardware_name}.identity.model",
        )
        hardware_serial = _text(
            identity["serial"],
            source,
            f"cameras.{hardware_name}.identity.serial",
        )
        role = _text(
            camera["role"],
            source,
            f"cameras.{hardware_name}.role",
        )
        if profile == "real" and (
            "placeholder" in hardware_model.lower()
            or "placeholder" in hardware_serial.lower()
        ):
            raise ConfigurationError(
                "Real camera identity cannot contain placeholder values."
            )
        stream = _mapping(camera, "stream", source, "camera")
        _keys(stream, {"width", "height", "fps", "pixel_format"}, source, "stream")
        width_px = _integer(
            stream["width"],
            source,
            f"cameras.{hardware_name}.stream.width",
        )
        height_px = _integer(
            stream["height"],
            source,
            f"cameras.{hardware_name}.stream.height",
        )
        fps_hz = _number(
            stream["fps"],
            source,
            f"cameras.{hardware_name}.stream.fps",
        )
        if width_px <= 0 or height_px <= 0 or fps_hz <= 0.0:
            raise ConfigurationError(
                "Camera width, height, and FPS must be positive."
            )
        pixel_format = _text(
            stream["pixel_format"],
            source,
            f"cameras.{hardware_name}.stream.pixel_format",
        )
        frames = _mapping(camera, "frames", source, "camera")
        _keys(frames, {"camera_frame", "optical_frame"}, source, "frames")
        camera_frame = _text(
            frames["camera_frame"],
            source,
            f"cameras.{hardware_name}.frames.camera_frame",
        )
        optical_frame = _text(
            frames["optical_frame"],
            source,
            f"cameras.{hardware_name}.frames.optical_frame",
        )
        if camera_frame == optical_frame:
            raise ConfigurationError(
                "Camera frame and optical frame must be distinct."
            )
        extrinsics = _mapping(camera, "extrinsics", source, "camera")
        _keys(
            extrinsics,
            {
                "parent_frame",
                "child_frame",
                "translation_m",
                "quaternion_xyzw",
            },
            source,
            "extrinsics",
        )
        extrinsic_parent_frame = _text(
            extrinsics["parent_frame"],
            source,
            f"cameras.{hardware_name}.extrinsics.parent_frame",
        )
        extrinsic_child_frame = _text(
            extrinsics["child_frame"],
            source,
            f"cameras.{hardware_name}.extrinsics.child_frame",
        )
        if extrinsic_child_frame != camera_frame:
            raise ConfigurationError(
                "Camera extrinsics.child_frame must equal frames.camera_frame."
            )
        reconnect = _mapping(camera, "reconnect", source, "camera")
        _keys(
            reconnect,
            {
                "read_timeout_s",
                "initial_backoff_s",
                "maximum_backoff_s",
                "multiplier",
                "maximum_attempts",
            },
            source,
            "reconnect",
        )
        read_timeout_s = _number(
            reconnect["read_timeout_s"],
            source,
            f"cameras.{hardware_name}.reconnect.read_timeout_s",
        )
        initial_backoff_s = _number(
            reconnect["initial_backoff_s"],
            source,
            f"cameras.{hardware_name}.reconnect.initial_backoff_s",
        )
        maximum_backoff_s = _number(
            reconnect["maximum_backoff_s"],
            source,
            f"cameras.{hardware_name}.reconnect.maximum_backoff_s",
        )
        multiplier = _number(
            reconnect["multiplier"],
            source,
            f"cameras.{hardware_name}.reconnect.multiplier",
        )
        maximum_attempts = _integer(
            reconnect["maximum_attempts"],
            source,
            f"cameras.{hardware_name}.reconnect.maximum_attempts",
        )
        if (
            read_timeout_s <= 0.0
            or initial_backoff_s < 0.0
            or maximum_backoff_s < initial_backoff_s
            or multiplier < 1.0
            or maximum_attempts < 0
        ):
            raise ConfigurationError("Camera reconnect settings are invalid.")
        device_value = camera["device_by_id"]
        if not isinstance(device_value, str):
            raise ConfigurationError(
                f"cameras.{hardware_name}.device_by_id must be a string."
            )
        device_by_id = device_value.strip()
        if profile == "real":
            device_prefix = "/dev/v4l/by-id/"
            device_identifier = device_by_id[len(device_prefix) :]
            if (
                not device_by_id.startswith(device_prefix)
                or not device_identifier
                or "/" in device_identifier
                or "placeholder" in device_identifier.lower()
            ):
                raise ConfigurationError(
                    "Real camera device_by_id must name one stable "
                    "/dev/v4l/by-id resource."
                )
        calibration_path = _resolve(source, camera["calibration_file"])
        calibration = _load_calibration(calibration_path)
        try:
            config = CameraConfig(
                hardware_name=hardware_name,
                enabled=_boolean(
                    camera["enabled"],
                    source,
                    f"cameras.{hardware_name}.enabled",
                ),
                backend=backend,
                hardware_model=hardware_model,
                hardware_serial=hardware_serial,
                role=role,
                device_by_id=device_by_id,
                stream=CameraStreamConfig(
                    width_px=width_px,
                    height_px=height_px,
                    fps_hz=fps_hz,
                    pixel_format=pixel_format,
                ),
                camera_frame=camera_frame,
                optical_frame=optical_frame,
                calibration=calibration,
                extrinsics=CameraExtrinsics(
                    parent_frame=extrinsic_parent_frame,
                    child_frame=extrinsic_child_frame,
                    translation_m=_numbers(
                        extrinsics, "translation_m", source, "extrinsics", 3
                    ),  # type: ignore[arg-type]
                    quaternion_xyzw=_numbers(
                        extrinsics,
                        "quaternion_xyzw",
                        source,
                        "extrinsics",
                        4,
                    ),  # type: ignore[arg-type]
                ),
                reconnect=CameraReconnectPolicy(
                    read_timeout_s=read_timeout_s,
                    initial_backoff_s=initial_backoff_s,
                    maximum_backoff_s=maximum_backoff_s,
                    multiplier=multiplier,
                    maximum_attempts=maximum_attempts,
                ),
            )
        except ValueError as error:
            raise ConfigurationError(
                f"Invalid camera {hardware_name}: {error}"
            ) from error
        result.append(config)
    return tuple(result)


def camera_config_by_name(config_path: str, hardware_name: str) -> CameraConfig:
    """Return one configured camera by stable hardware identity name."""
    indexed: Dict[str, CameraConfig] = {
        config.hardware_name: config for config in load_camera_configs(config_path)
    }
    try:
        return indexed[hardware_name]
    except KeyError as error:
        raise ConfigurationError(
            f"Camera is not defined in {config_path}: {hardware_name}"
        ) from error
