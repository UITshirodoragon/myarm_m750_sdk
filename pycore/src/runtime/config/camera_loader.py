"""Validate hardware-based camera YAML for ROS-independent use."""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Mapping, Tuple

import yaml

from myarm_m750_core.domain.camera import CameraConfig, CameraStreamConfig
from myarm_m750_core.domain.errors import ConfigurationError


def _mapping(value: object, label: str, source: Path) -> Mapping[str, object]:
    if not isinstance(value, dict):
        raise ConfigurationError("{0} must be a mapping in {1}.".format(label, source))
    return value


def load_camera_configs(config_path: str) -> Tuple[CameraConfig, ...]:
    """Load camera definitions without importing ROS 2 or OpenCV."""
    source = Path(config_path).expanduser().resolve()
    if not source.is_file():
        raise ConfigurationError("Camera YAML does not exist: {0}".format(source))
    try:
        raw = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
    except yaml.YAMLError as error:
        raise ConfigurationError("Invalid camera YAML: {0}".format(error)) from error

    root = _mapping(raw, "root", source)
    cameras = _mapping(root.get("cameras"), "cameras", source)
    result = []
    for hardware_name, camera_value in cameras.items():
        camera = _mapping(camera_value, "camera {0}".format(hardware_name), source)
        device = _mapping(camera.get("device"), "device", source)
        stream = _mapping(camera.get("stream"), "stream", source)
        frames = _mapping(camera.get("frames"), "frames", source)
        hardware_serial = str(camera.get("hardware_serial", "")).strip()
        if not hardware_serial:
            raise ConfigurationError(
                "Camera {0} requires hardware_serial.".format(hardware_name)
            )
        result.append(
            CameraConfig(
                hardware_name=str(hardware_name),
                enabled=bool(camera.get("enabled", False)),
                hardware_model=str(camera.get("hardware_model", hardware_name)),
                hardware_serial=hardware_serial,
                role=str(camera.get("role", "unassigned")),
                device_by_id=str(device.get("by_id", "")),
                fallback_path=str(device.get("fallback_path", "")),
                stream=CameraStreamConfig(
                    width_px=int(stream.get("width", 640)),
                    height_px=int(stream.get("height", 480)),
                    fps_hz=float(stream.get("fps", 30.0)),
                    pixel_format=str(stream.get("pixel_format", "mjpeg")),
                ),
                camera_frame=str(frames.get("camera_frame", hardware_name + "_link")),
                optical_frame=str(
                    frames.get("optical_frame", hardware_name + "_optical_frame")
                ),
                calibration_file=str(camera.get("calibration_file", "")),
            )
        )
    return tuple(result)


def camera_config_by_name(config_path: str, hardware_name: str) -> CameraConfig:
    """Return one configured physical camera by its stable hardware name."""
    indexed: Dict[str, CameraConfig] = {
        config.hardware_name: config for config in load_camera_configs(config_path)
    }
    try:
        return indexed[hardware_name]
    except KeyError as error:
        raise ConfigurationError(
            "Camera is not defined in {0}: {1}".format(config_path, hardware_name)
        ) from error
