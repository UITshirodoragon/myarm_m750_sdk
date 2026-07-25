"""Installed model resources for ROS-independent core deployments."""

from __future__ import annotations

import hashlib
import json
from importlib import resources
from types import MappingProxyType
from typing import Any, Mapping

_ROOT_KEYS = {
    "core_snapshot",
    "kinematic_contract_sha256",
    "model_name",
    "model_revision",
    "provenance",
    "schema_version",
    "source",
    "variants",
}
_VARIANT_KEYS = {
    "artifact_sha256",
    "collision_geometry",
    "kinematic_contract_sha256",
    "path",
    "visual_geometry",
}


def _require_keys(
    value: object,
    expected: set,
    location: str,
) -> Mapping[str, object]:
    if not isinstance(value, dict):
        raise ValueError(f"{location} must be a mapping.")
    actual = set(value)
    if actual != expected:
        raise ValueError(
            f"{location} fields differ: "
            f"missing={sorted(expected - actual)}, "
            f"unknown={sorted(actual - expected)}."
        )
    return value


def _require_sha256(value: object, location: str) -> str:
    text = str(value)
    if len(text) != 64 or any(
        character not in "0123456789abcdef" for character in text
    ):
        raise ValueError(f"{location} must be a lowercase SHA-256 value.")
    return text


def _freeze(value: Any) -> Any:
    if isinstance(value, dict):
        return MappingProxyType(
            {str(key): _freeze(item) for key, item in value.items()}
        )
    if isinstance(value, list):
        return tuple(_freeze(item) for item in value)
    return value


def _validate_model_manifest(
    payload: object,
    kinematic_urdf: str,
) -> Mapping[str, object]:
    root = _require_keys(payload, _ROOT_KEYS, "model manifest")
    if root["schema_version"] != 1:
        raise ValueError("Model manifest schema_version must equal 1.")
    if not str(root["model_name"]) or not str(root["model_revision"]):
        raise ValueError("Model name and revision must be non-empty.")
    contract_hash = _require_sha256(
        root["kinematic_contract_sha256"],
        "kinematic_contract_sha256",
    )
    source = _require_keys(root["source"], {"path", "sha256"}, "source")
    if not str(source["path"]):
        raise ValueError("Model source path must be non-empty.")
    _require_sha256(source["sha256"], "source.sha256")

    variants = _require_keys(
        root["variants"],
        {"full", "lightweight", "kinematic"},
        "variants",
    )
    for variant_name, variant_value in variants.items():
        variant = _require_keys(
            variant_value,
            _VARIANT_KEYS,
            f"variants.{variant_name}",
        )
        _require_sha256(
            variant["artifact_sha256"],
            f"variants.{variant_name}.artifact_sha256",
        )
        if (
            _require_sha256(
                variant["kinematic_contract_sha256"],
                f"variants.{variant_name}.kinematic_contract_sha256",
            )
            != contract_hash
        ):
            raise ValueError(
                f"Variant {variant_name} has a different kinematic contract."
            )

    core_snapshot = _require_keys(
        root["core_snapshot"],
        {"artifact_sha256", "package", "resource"},
        "core_snapshot",
    )
    expected_artifact_hash = _require_sha256(
        core_snapshot["artifact_sha256"],
        "core_snapshot.artifact_sha256",
    )
    observed_artifact_hash = hashlib.sha256(
        kinematic_urdf.encode("utf-8")
    ).hexdigest()
    if expected_artifact_hash != observed_artifact_hash:
        raise ValueError("Installed kinematic URDF differs from model manifest.")
    kinematic_variant = _require_keys(
        variants["kinematic"],
        _VARIANT_KEYS,
        "variants.kinematic",
    )
    if kinematic_variant["artifact_sha256"] != observed_artifact_hash:
        raise ValueError("Kinematic variant and core snapshot hashes differ.")
    _require_keys(
        root["provenance"],
        {"collision", "inertial", "visual_meshes"},
        "provenance",
    )
    return _freeze(dict(root))


def read_kinematic_urdf() -> str:
    """Return the generated geometry-free canonical URDF snapshot."""
    return resources.read_text(
        "myarm_m750_core.resources",
        "myarm_m750_kinematic.urdf",
        encoding="utf-8",
    )


def read_model_manifest() -> Mapping[str, object]:
    """Return a strict, deeply immutable model provenance manifest."""
    manifest_text = resources.read_text(
        "myarm_m750_core.resources",
        "model_manifest.json",
        encoding="utf-8",
    )
    return _validate_model_manifest(
        json.loads(manifest_text),
        read_kinematic_urdf(),
    )


__all__ = ["read_kinematic_urdf", "read_model_manifest"]
