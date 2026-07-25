"""Strict consumer for the canonical description model catalog."""

from __future__ import annotations

import hashlib
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Mapping

import yaml

_VARIANT_NAMES = ("full", "lightweight", "kinematic")
_SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")


@dataclass(frozen=True)
class ModelVariant:
    """One verified installed model artifact."""

    name: str
    path: Path
    artifact_sha256: str
    kinematic_contract_sha256: str
    visual_geometry: str
    collision_geometry: str


@dataclass(frozen=True)
class ModelCatalog:
    """Validated model locator shared by MoveIt launch consumers."""

    default_variant: str
    base_link: str
    flange_link: str
    end_link: str
    kinematic_contract_sha256: str
    collision_provenance: str
    variants: Mapping[str, ModelVariant]

    def planning_variant(self, name: str) -> ModelVariant:
        """Return a collision-enabled planning variant."""
        if name not in ("full", "lightweight"):
            raise ValueError("MoveIt model_variant must be full or lightweight.")
        variant = self.variants[name]
        if variant.collision_geometry == "none":
            raise ValueError(f"Model variant '{name}' has no collision geometry.")
        return variant


def load_model_catalog(description_share: Path) -> ModelCatalog:
    """Load strict YAML locator and cross-check its canonical JSON manifest."""
    package_root = description_share.resolve()
    catalog_path = package_root / "config" / "model.yaml"
    document = _mapping(
        yaml.safe_load(catalog_path.read_text(encoding="utf-8")),
        "document",
    )
    _exact_keys(document, {"config_version", "model"}, "document")
    if document["config_version"] != 1:
        raise ValueError("model.yaml config_version must equal 1.")
    model = _mapping(document["model"], "model")
    _exact_keys(
        model,
        {
            "source_xacro",
            "manifest",
            "default_variant",
            "variants",
            "base_link",
            "flange_link",
            "end_link",
            "dynamics_enabled",
            "collision_provenance",
        },
        "model",
    )
    if model["dynamics_enabled"] is not False:
        raise ValueError("Dynamics must remain disabled without inertial provenance.")
    variants_yaml = _mapping(model["variants"], "model.variants")
    _exact_keys(variants_yaml, set(_VARIANT_NAMES), "model.variants")

    manifest_path = _resolved_resource(
        package_root,
        _string(model["manifest"], "model.manifest"),
        "model.manifest",
    )
    manifest = _mapping(
        json.loads(manifest_path.read_text(encoding="utf-8")),
        "manifest",
    )
    _exact_keys(
        manifest,
        {
            "schema_version",
            "model_name",
            "model_revision",
            "source",
            "kinematic_contract_sha256",
            "variants",
            "core_snapshot",
            "provenance",
        },
        "manifest",
    )
    if manifest["schema_version"] != 1:
        raise ValueError("model manifest schema_version must equal 1.")
    contract_sha256 = _sha256(
        manifest["kinematic_contract_sha256"],
        "manifest.kinematic_contract_sha256",
    )
    manifest_variants = _mapping(manifest["variants"], "manifest.variants")
    _exact_keys(manifest_variants, set(_VARIANT_NAMES), "manifest.variants")

    source = _mapping(manifest["source"], "manifest.source")
    _exact_keys(source, {"path", "sha256"}, "manifest.source")
    source_path = _resolved_resource(
        package_root,
        _string(model["source_xacro"], "model.source_xacro"),
        "model.source_xacro",
    )
    if _string(source["path"], "manifest.source.path") != str(
        model["source_xacro"]
    ):
        raise ValueError("model.yaml and manifest source paths differ.")
    _verify_file_hash(
        source_path,
        _sha256(source["sha256"], "manifest.source.sha256"),
    )

    variants: Dict[str, ModelVariant] = {}
    for variant_name in _VARIANT_NAMES:
        variant_data = _mapping(
            manifest_variants[variant_name],
            f"manifest.variants.{variant_name}",
        )
        _exact_keys(
            variant_data,
            {
                "path",
                "artifact_sha256",
                "kinematic_contract_sha256",
                "visual_geometry",
                "collision_geometry",
            },
            f"manifest.variants.{variant_name}",
        )
        yaml_path = _string(
            variants_yaml[variant_name],
            f"model.variants.{variant_name}",
        )
        manifest_variant_path = _string(
            variant_data["path"],
            f"manifest.variants.{variant_name}.path",
        )
        if yaml_path != manifest_variant_path:
            raise ValueError(
                f"Variant '{variant_name}' path differs between YAML and JSON."
            )
        variant_contract = _sha256(
            variant_data["kinematic_contract_sha256"],
            f"manifest.variants.{variant_name}.kinematic_contract_sha256",
        )
        if variant_contract != contract_sha256:
            raise ValueError(
                f"Variant '{variant_name}' has a different kinematic contract."
            )
        artifact_sha256 = _sha256(
            variant_data["artifact_sha256"],
            f"manifest.variants.{variant_name}.artifact_sha256",
        )
        variant_path = _resolved_resource(
            package_root,
            yaml_path,
            f"model.variants.{variant_name}",
        )
        _verify_file_hash(variant_path, artifact_sha256)
        variants[variant_name] = ModelVariant(
            name=variant_name,
            path=variant_path,
            artifact_sha256=artifact_sha256,
            kinematic_contract_sha256=variant_contract,
            visual_geometry=_string(
                variant_data["visual_geometry"],
                f"manifest.variants.{variant_name}.visual_geometry",
            ),
            collision_geometry=_string(
                variant_data["collision_geometry"],
                f"manifest.variants.{variant_name}.collision_geometry",
            ),
        )

    default_variant = _string(
        model["default_variant"],
        "model.default_variant",
    )
    if default_variant not in variants:
        raise ValueError("model.default_variant does not name an installed variant.")
    return ModelCatalog(
        default_variant=default_variant,
        base_link=_string(model["base_link"], "model.base_link"),
        flange_link=_string(model["flange_link"], "model.flange_link"),
        end_link=_string(model["end_link"], "model.end_link"),
        kinematic_contract_sha256=contract_sha256,
        collision_provenance=_string(
            model["collision_provenance"],
            "model.collision_provenance",
        ),
        variants=variants,
    )


def _mapping(value: Any, field_name: str) -> Dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{field_name} must be a mapping.")
    return value


def _exact_keys(
    value: Mapping[str, Any],
    expected: set,
    field_name: str,
) -> None:
    observed = set(value)
    if observed != expected:
        missing = sorted(expected - observed)
        unknown = sorted(observed - expected)
        raise ValueError(
            f"{field_name} keys differ; missing={missing}, unknown={unknown}."
        )


def _string(value: Any, field_name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field_name} must be a non-empty string.")
    return value.strip()


def _sha256(value: Any, field_name: str) -> str:
    fingerprint = _string(value, field_name)
    if _SHA256_PATTERN.fullmatch(fingerprint) is None:
        raise ValueError(f"{field_name} must be a lowercase SHA-256 value.")
    return fingerprint


def _resolved_resource(
    package_root: Path,
    relative_path: str,
    field_name: str,
) -> Path:
    candidate = Path(relative_path)
    if candidate.is_absolute():
        raise ValueError(f"{field_name} must be package-relative.")
    if ".." in candidate.parts:
        raise ValueError(f"{field_name} escapes the description package.")
    installed_path = package_root / candidate
    if not installed_path.is_file():
        raise ValueError(
            f"{field_name} does not resolve to a file: {installed_path}"
        )
    # Keep the install-space path instead of resolving its final symlink.
    # ``colcon --symlink-install`` intentionally points resources at source.
    return installed_path


def _verify_file_hash(path: Path, expected_sha256: str) -> None:
    observed = hashlib.sha256(path.read_bytes()).hexdigest()
    if observed != expected_sha256:
        raise ValueError(
            f"Model artifact hash mismatch for {path}: "
            f"expected {expected_sha256}, observed {observed}."
        )
