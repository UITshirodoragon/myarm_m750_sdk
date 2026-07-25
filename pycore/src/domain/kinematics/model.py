"""Deterministic URDF kinematic-contract normalization and fingerprinting."""

from __future__ import annotations

import hashlib
import json
import xml.etree.ElementTree as element_tree
from pathlib import Path
from typing import List, Mapping, Optional, Tuple

from myarm_m750_core.domain.errors import KinematicsError


def _normalized_vector(raw_value: Optional[str], default: str) -> Tuple[float, ...]:
    values = tuple(float(value) for value in (raw_value or default).split())
    return tuple(0.0 if value == 0.0 else value for value in values)


def _child_attributes(
    element: element_tree.Element, child_name: str
) -> Mapping[str, object]:
    child = element.find(child_name)
    if child is None:
        return {}
    return {key: child.attrib[key] for key in sorted(child.attrib)}


def normalized_kinematic_contract(urdf_xml: str) -> Mapping[str, object]:
    """Return the geometry-independent, JSON-serializable URDF contract.

    Visual, collision, material, transmission, and inertial elements are
    intentionally excluded. Frame topology, joint origins, axes, limits,
    dynamics, and mimic rules remain in the contract.
    """
    try:
        root = element_tree.fromstring(urdf_xml)
    except element_tree.ParseError as error:
        raise KinematicsError(f"Could not parse URDF XML: {error}") from error
    if root.tag != "robot":
        raise KinematicsError("URDF root element must be <robot>.")

    links = sorted(
        str(link.attrib["name"]) for link in root.findall("link") if link.attrib.get("name")
    )
    joints: List[Mapping[str, object]] = []
    for joint in sorted(
        root.findall("joint"), key=lambda item: str(item.attrib.get("name", ""))
    ):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            raise KinematicsError("Every URDF joint must have parent and child.")
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


def kinematic_contract_fingerprint(urdf_xml: str) -> str:
    """Return SHA-256 of a normalized geometry-independent URDF contract."""
    contract_json = json.dumps(
        normalized_kinematic_contract(urdf_xml),
        sort_keys=True,
        separators=(",", ":"),
    )
    return hashlib.sha256(contract_json.encode("utf-8")).hexdigest()


def fingerprint_urdf_path(urdf_path: Path) -> str:
    """Read a URDF and return its normalized kinematic-contract fingerprint."""
    resolved_path = Path(urdf_path).expanduser().resolve()
    try:
        urdf_xml = resolved_path.read_text(encoding="utf-8")
    except OSError as error:
        raise KinematicsError(f"Could not read URDF {resolved_path}: {error}") from error
    return kinematic_contract_fingerprint(urdf_xml)
