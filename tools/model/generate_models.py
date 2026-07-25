#!/usr/bin/env python3
"""Render and verify deterministic MyArm M750 model artifacts."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import xml.etree.ElementTree as element_tree
from collections.abc import Mapping
from pathlib import Path
from typing import Dict, Optional, Tuple

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
SOURCE_PATH = REPOSITORY_ROOT / "ros2/src/myarm_m750_description/urdf/myarm_m750.urdf.xacro"
GENERATED_DIRECTORY = REPOSITORY_ROOT / "ros2/src/myarm_m750_description/urdf/generated"
DESCRIPTION_MANIFEST_PATH = (
    REPOSITORY_ROOT / "ros2/src/myarm_m750_description/config/model_manifest.json"
)
CORE_RESOURCE_DIRECTORY = REPOSITORY_ROOT / "pycore/src/resources"
CORE_SNAPSHOT_PATH = CORE_RESOURCE_DIRECTORY / "myarm_m750_kinematic.urdf"
CORE_MANIFEST_PATH = CORE_RESOURCE_DIRECTORY / "model_manifest.json"
VARIANTS = ("full", "lightweight", "kinematic")


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _normalized_vector(
    raw_value: Optional[str], default: str
) -> Tuple[float, ...]:
    values = tuple(float(value) for value in (raw_value or default).split())
    return tuple(0.0 if value == 0.0 else value for value in values)


def _child_attributes(element: element_tree.Element, child_name: str) -> Mapping[str, str]:
    child = element.find(child_name)
    if child is None:
        return {}
    return {key: child.attrib[key] for key in sorted(child.attrib)}


def _kinematic_contract(urdf_xml: str) -> Mapping[str, object]:
    root = element_tree.fromstring(urdf_xml)
    links = sorted(
        str(link.attrib["name"]) for link in root.findall("link") if link.attrib.get("name")
    )
    joints = []
    for joint in sorted(
        root.findall("joint"), key=lambda item: str(item.attrib.get("name", ""))
    ):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            raise ValueError("Every joint must have parent and child.")
        origin = joint.find("origin")
        axis = joint.find("axis")
        joints.append(
            {
                "name": str(joint.attrib.get("name", "")),
                "type": str(joint.attrib.get("type", "")),
                "parent": str(parent.attrib.get("link", "")),
                "child": str(child.attrib.get("link", "")),
                "origin_xyz_m": _normalized_vector(
                    origin.attrib.get("xyz") if origin is not None else None,
                    "0 0 0",
                ),
                "origin_rpy_rad": _normalized_vector(
                    origin.attrib.get("rpy") if origin is not None else None,
                    "0 0 0",
                ),
                "axis": _normalized_vector(
                    axis.attrib.get("xyz") if axis is not None else None,
                    "1 0 0",
                ),
                "limit": _child_attributes(joint, "limit"),
                "dynamics": _child_attributes(joint, "dynamics"),
                "mimic": _child_attributes(joint, "mimic"),
            }
        )
    return {
        "robot_name": str(root.attrib.get("name", "")),
        "links": links,
        "joints": joints,
    }


def _contract_fingerprint(urdf_xml: str) -> str:
    encoded = json.dumps(
        _kinematic_contract(urdf_xml),
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return _sha256(encoded)


def _render_variant(variant: str) -> str:
    source_relative_path = SOURCE_PATH.relative_to(REPOSITORY_ROOT)
    result = subprocess.run(
        ["xacro", str(source_relative_path), f"model_variant:={variant}"],
        cwd=str(REPOSITORY_ROOT),
        check=False,
        capture_output=True,
        encoding="utf-8",
    )
    if result.returncode != 0:
        raise RuntimeError(f"xacro failed for {variant}: {result.stderr.strip()}")
    element_tree.fromstring(result.stdout)
    return result.stdout


def _manifest(rendered: Mapping[str, str]) -> Mapping[str, object]:
    fingerprints = {
        variant: _contract_fingerprint(rendered[variant]) for variant in VARIANTS
    }
    if len(set(fingerprints.values())) != 1:
        raise RuntimeError(
            f"Generated variants have different kinematic contracts: {fingerprints}"
        )
    variants: Dict[str, Mapping[str, object]] = {}
    for variant in VARIANTS:
        artifact_name = f"myarm_m750_{variant}.urdf"
        variants[variant] = {
            "path": f"urdf/generated/{artifact_name}",
            "artifact_sha256": _sha256(rendered[variant].encode("utf-8")),
            "kinematic_contract_sha256": fingerprints[variant],
            "visual_geometry": (
                "detailed_mesh"
                if variant == "full"
                else "primitive"
                if variant == "lightweight"
                else "none"
            ),
            "collision_geometry": "none" if variant == "kinematic" else "primitive",
        }
    return {
        "schema_version": 1,
        "model_name": "myarm_m750",
        "model_revision": "3.2",
        "source": {
            "path": "urdf/myarm_m750.urdf.xacro",
            "sha256": _sha256(SOURCE_PATH.read_bytes()),
        },
        "kinematic_contract_sha256": next(iter(fingerprints.values())),
        "variants": variants,
        "core_snapshot": {
            "package": "myarm_m750_core.resources",
            "resource": "myarm_m750_kinematic.urdf",
            "artifact_sha256": _sha256(rendered["kinematic"].encode("utf-8")),
        },
        "provenance": {
            "visual_meshes": "supplied MyArm M750 v3.2 assets; license review pending",
            "collision": "provisional primitives; physical collision review pending",
            "inertial": "unavailable; dynamics, gravity, and torque disabled",
        },
    }


def _expected_files(
    rendered: Mapping[str, str], manifest: Mapping[str, object]
) -> Mapping[Path, str]:
    manifest_text = json.dumps(manifest, indent=2, sort_keys=True) + "\n"
    expected: Dict[Path, str] = {
        GENERATED_DIRECTORY / f"myarm_m750_{variant}.urdf": rendered[variant]
        for variant in VARIANTS
    }
    expected[DESCRIPTION_MANIFEST_PATH] = manifest_text
    expected[CORE_SNAPSHOT_PATH] = rendered["kinematic"]
    expected[CORE_MANIFEST_PATH] = manifest_text
    return expected


def _check(expected_files: Mapping[Path, str]) -> int:
    stale_paths = []
    for path, expected_text in expected_files.items():
        try:
            actual_text = path.read_text(encoding="utf-8")
        except OSError:
            stale_paths.append(path)
            continue
        if actual_text != expected_text:
            stale_paths.append(path)
    if stale_paths:
        for path in stale_paths:
            print(f"STALE {path.relative_to(REPOSITORY_ROOT)}", file=sys.stderr)
        return 1
    print("Model artifacts are deterministic and current.")
    return 0


def _write(expected_files: Mapping[Path, str]) -> None:
    for path, text in expected_files.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
        print(f"WROTE {path.relative_to(REPOSITORY_ROOT)}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--check",
        action="store_true",
        help="Fail when committed artifacts differ from a fresh xacro render.",
    )
    arguments = parser.parse_args()
    rendered = {variant: _render_variant(variant) for variant in VARIANTS}
    manifest = _manifest(rendered)
    expected_files = _expected_files(rendered, manifest)
    if arguments.check:
        return _check(expected_files)
    _write(expected_files)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
