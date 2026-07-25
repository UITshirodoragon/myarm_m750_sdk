"""Optional pytransform3d helpers for offline model inspection."""

from __future__ import annotations

import importlib
import os
from pathlib import Path
from typing import Any, Mapping, Optional

from myarm_m750_core.domain.errors import KinematicsError

_SUPPORTED_PYTRANSFORM3D_VERSION = "3.16.0"


def load_urdf_transform_manager(
    urdf_path: Path,
    joint_position_by_name_rad: Optional[Mapping[str, float]] = None,
    package_prefix_directory: Optional[Path] = None,
) -> Any:
    """Load a URDF into pytransform3d without making it a core dependency."""
    try:
        package = importlib.import_module("pytransform3d")
        urdf_module = importlib.import_module("pytransform3d.urdf")
    except ImportError as error:
        raise KinematicsError(
            "pytransform3d is optional; install geometry-tools "
            "(expected pytransform3d==3.16.0) for offline inspection."
        ) from error
    version = str(getattr(package, "__version__", "unknown"))
    if version != _SUPPORTED_PYTRANSFORM3D_VERSION:
        raise KinematicsError(
            f"Unsupported pytransform3d version {version}; "
            f"expected {_SUPPORTED_PYTRANSFORM3D_VERSION}."
        )

    resolved_path = Path(urdf_path).expanduser().resolve()
    try:
        urdf_xml = resolved_path.read_text(encoding="utf-8")
    except OSError as error:
        raise KinematicsError(f"Could not read URDF {resolved_path}: {error}") from error
    manager = urdf_module.UrdfTransformManager()
    uses_package_uri = "package://" in urdf_xml
    effective_package_prefix = package_prefix_directory
    if effective_package_prefix is None and uses_package_uri:
        try:
            effective_package_prefix = resolved_path.parents[3]
        except IndexError as error:
            raise KinematicsError(
                "package_prefix_directory is required for package:// mesh URIs."
            ) from error
    package_prefix = (
        str(Path(effective_package_prefix).expanduser().resolve()) + os.sep
        if effective_package_prefix is not None
        else None
    )
    manager.load_urdf(
        urdf_xml,
        mesh_path=None if uses_package_uri else str(resolved_path.parent),
        package_dir=package_prefix,
    )
    for joint_name, joint_position_rad in (joint_position_by_name_rad or {}).items():
        manager.set_joint(joint_name, float(joint_position_rad))
    return manager
