#!/usr/bin/env python3
"""Inspect the canonical URDF frame graph and mesh resources headlessly."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import xml.etree.ElementTree as element_tree
from collections import deque
from pathlib import Path
from typing import List, Mapping, Optional, Sequence, Tuple

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DESCRIPTION_DIRECTORY = (
    REPOSITORY_ROOT / "ros2/src/myarm_m750_description"
)
DEFAULT_MODEL = (
    DESCRIPTION_DIRECTORY / "urdf/generated/myarm_m750_full.urdf"
)
_COLLADA_NAMESPACE = "{http://www.collada.org/2005/11/COLLADASchema}"
_Matrix4 = Tuple[Tuple[float, float, float, float], ...]


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _values(text: Optional[str], expected_size: int) -> Tuple[float, ...]:
    raw_values = tuple(float(value) for value in (text or "").split())
    if len(raw_values) != expected_size:
        raise ValueError(
            f"Expected {expected_size} vector values, got {len(raw_values)}."
        )
    if not all(math.isfinite(value) for value in raw_values):
        raise ValueError("Vector contains a non-finite value.")
    return raw_values


def _identity_matrix() -> _Matrix4:
    return (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _multiply(left: _Matrix4, right: _Matrix4) -> _Matrix4:
    return tuple(
        tuple(
            sum(left[row][index] * right[index][column] for index in range(4))
            for column in range(4)
        )
        for row in range(4)
    )


def _origin_matrix(origin: Optional[element_tree.Element]) -> _Matrix4:
    if origin is None:
        return _identity_matrix()
    x, y, z = _values(origin.attrib.get("xyz", "0 0 0"), 3)
    roll, pitch, yaw = _values(origin.attrib.get("rpy", "0 0 0"), 3)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (
            cy * cp,
            cy * sp * sr - sy * cr,
            cy * sp * cr + sy * sr,
            x,
        ),
        (
            sy * cp,
            sy * sp * sr + cy * cr,
            sy * sp * cr - cy * sr,
            y,
        ),
        (-sp, cp * sr, cp * cr, z),
        (0.0, 0.0, 0.0, 1.0),
    )


def _graph_report(
    links: Sequence[str],
    joints: Sequence[Mapping[str, str]],
) -> Mapping[str, object]:
    children_by_parent = {}
    parents = set()
    children = set()
    undirected = {link: [] for link in links}
    for joint in joints:
        parent = joint["parent"]
        child = joint["child"]
        parents.add(parent)
        children.add(child)
        children_by_parent.setdefault(parent, []).append(child)
        undirected.setdefault(parent, []).append(child)
        undirected.setdefault(child, []).append(parent)

    cycle_paths = []
    visited = set()
    active = set()

    def visit(link: str, path: List[str]) -> None:
        if link in active:
            cycle_start = path.index(link)
            cycle_paths.append(path[cycle_start:] + [link])
            return
        if link in visited:
            return
        active.add(link)
        for child_link in children_by_parent.get(link, ()):
            visit(child_link, path + [child_link])
        active.remove(link)
        visited.add(link)

    for link in sorted(links):
        visit(link, [link])

    components = []
    remaining = set(links)
    while remaining:
        component = []
        queue = deque([min(remaining)])
        remaining.remove(queue[0])
        while queue:
            link = queue.popleft()
            component.append(link)
            for neighbour in undirected.get(link, ()):
                if neighbour in remaining:
                    remaining.remove(neighbour)
                    queue.append(neighbour)
        components.append(sorted(component))

    roots = sorted(set(links) - children)
    missing_link_references = sorted((parents | children) - set(links))
    return {
        "roots": roots,
        "connected_components": components,
        "cycle_paths": cycle_paths,
        "missing_link_references": missing_link_references,
        "is_connected": len(components) == 1,
        "is_acyclic": not cycle_paths,
    }


def _zero_pose(
    root: element_tree.Element,
    base_frame: str,
    end_frame: str,
) -> Mapping[str, object]:
    edges = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        edges.setdefault(parent.attrib["link"], []).append(
            (child.attrib["link"], _origin_matrix(joint.find("origin")))
        )
    transforms = {base_frame: _identity_matrix()}
    queue = deque([base_frame])
    while queue:
        parent_frame = queue.popleft()
        for child_frame, parent_to_child in edges.get(parent_frame, ()):
            if child_frame in transforms:
                continue
            transforms[child_frame] = _multiply(
                transforms[parent_frame], parent_to_child
            )
            queue.append(child_frame)
    if end_frame not in transforms:
        return {
            "parent_frame": base_frame,
            "child_frame": end_frame,
            "resolved": False,
        }
    matrix = transforms[end_frame]
    return {
        "parent_frame": base_frame,
        "child_frame": end_frame,
        "joint_position_rad": "all movable joints at zero",
        "resolved": True,
        "translation_m": [matrix[index][3] for index in range(3)],
        "rotation_matrix": [list(row[:3]) for row in matrix[:3]],
    }


def _resolve_mesh_uri(uri: str) -> Optional[Path]:
    prefix = "package://myarm_m750_description/"
    if uri.startswith(prefix):
        return (DESCRIPTION_DIRECTORY / uri[len(prefix) :]).resolve()
    if uri.startswith("file://"):
        return Path(uri[7:]).expanduser().resolve()
    return None


def _collada_bounds(
    mesh_path: Path,
    urdf_scale: Sequence[float],
) -> Mapping[str, object]:
    unit_meter = 1.0
    up_axis = "UNKNOWN"
    minimum = [math.inf, math.inf, math.inf]
    maximum = [-math.inf, -math.inf, -math.inf]
    position_count = 0
    triangle_count = 0
    in_position_source = False
    for event, element in element_tree.iterparse(
        str(mesh_path), events=("start", "end")
    ):
        if (
            event == "start"
            and element.tag == f"{_COLLADA_NAMESPACE}source"
            and element.attrib.get("name", "").lower() == "position"
        ):
            in_position_source = True
            continue
        if event == "start":
            continue
        if element.tag == f"{_COLLADA_NAMESPACE}unit":
            unit_meter = float(element.attrib.get("meter", "1"))
        elif element.tag == f"{_COLLADA_NAMESPACE}up_axis":
            up_axis = (element.text or "UNKNOWN").strip()
        elif element.tag == f"{_COLLADA_NAMESPACE}triangles":
            triangle_count += int(element.attrib.get("count", "0"))
        elif (
            in_position_source
            and element.tag == f"{_COLLADA_NAMESPACE}float_array"
        ):
            coordinates = tuple(
                float(value) for value in (element.text or "").split()
            )
            if len(coordinates) % 3 != 0:
                raise ValueError(
                    f"Position array is not XYZ triples: {mesh_path}"
                )
            for offset in range(0, len(coordinates), 3):
                for axis in range(3):
                    value = (
                        coordinates[offset + axis]
                        * unit_meter
                        * urdf_scale[axis]
                    )
                    if not math.isfinite(value):
                        raise ValueError(
                            f"Mesh contains non-finite position: {mesh_path}"
                        )
                    minimum[axis] = min(minimum[axis], value)
                    maximum[axis] = max(maximum[axis], value)
                position_count += 1
        elif (
            in_position_source
            and element.tag == f"{_COLLADA_NAMESPACE}source"
        ):
            in_position_source = False
        element.clear()
    bounds_available = position_count > 0
    return {
        "collada_unit_meter": unit_meter,
        "up_axis": up_axis,
        "position_count": position_count,
        "triangle_count": triangle_count,
        "bounds_minimum_m": minimum if bounds_available else None,
        "bounds_maximum_m": maximum if bounds_available else None,
    }


def _mesh_reports(root: element_tree.Element) -> List[Mapping[str, object]]:
    reports = []
    for mesh in root.findall(".//mesh"):
        uri = mesh.attrib.get("filename", "")
        scale = _values(mesh.attrib.get("scale", "1 1 1"), 3)
        resolved_path = _resolve_mesh_uri(uri)
        exists = resolved_path is not None and resolved_path.is_file()
        report = {
            "uri": uri,
            "scale": list(scale),
            "resolved_path": str(resolved_path) if resolved_path else None,
            "exists": exists,
        }
        if exists and resolved_path is not None:
            report["size_bytes"] = resolved_path.stat().st_size
            if resolved_path.suffix.lower() == ".dae":
                report.update(_collada_bounds(resolved_path, scale))
        reports.append(report)
    return sorted(reports, key=lambda item: str(item["uri"]))


def _joint_records(root: element_tree.Element) -> List[Mapping[str, str]]:
    joints = []
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        joints.append(
            {
                "name": joint.attrib.get("name", ""),
                "type": joint.attrib.get("type", ""),
                "parent": parent.attrib.get("link", "") if parent is not None else "",
                "child": child.attrib.get("link", "") if child is not None else "",
            }
        )
    return joints


def _inspect(
    model_path: Path,
    base_frame: str = "base_link",
    end_frame: str = "tool0",
) -> Mapping[str, object]:
    urdf_bytes = model_path.read_bytes()
    root = element_tree.fromstring(urdf_bytes)
    links = sorted(
        link.attrib["name"] for link in root.findall("link")
    )
    joints = _joint_records(root)
    graph = _graph_report(links, joints)
    mesh_reports = _mesh_reports(root)
    zero_pose = _zero_pose(root, base_frame, end_frame)
    unresolved_meshes = [
        report["uri"] for report in mesh_reports if not report["exists"]
    ]
    valid = bool(
        graph["is_connected"]
        and graph["is_acyclic"]
        and not graph["missing_link_references"]
        and not unresolved_meshes
        and zero_pose["resolved"]
    )
    return {
        "path": str(model_path),
        "artifact_sha256": _sha256(urdf_bytes),
        "robot_name": root.attrib.get("name", ""),
        "link_count": len(links),
        "joint_count": len(joints),
        "joints": joints,
        "frame_graph": graph,
        "meshes": mesh_reports,
        "unresolved_mesh_uris": unresolved_meshes,
        "zero_configuration_pose": zero_pose,
        "has_inertial_data": bool(root.findall(".//inertial")),
        "valid": valid,
    }


def _load_pytransform3d(model_path: Path) -> Mapping[str, object]:
    try:
        import pytransform3d
        from pytransform3d.urdf import UrdfTransformManager
    except ImportError as error:
        raise RuntimeError(
            "pytransform3d is unavailable; install geometry-tools."
        ) from error
    manager = UrdfTransformManager()
    manager.load_urdf(
        model_path.read_text(encoding="utf-8"),
        mesh_path=None,
        package_dir=str(model_path.parents[3]) + os.sep,
    )
    return {
        "version": str(pytransform3d.__version__),
        "node_count": len(manager.nodes),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("model", nargs="?", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--end-frame", default="tool0")
    parser.add_argument("--with-pytransform3d", action="store_true")
    parser.add_argument("--output", type=Path)
    arguments = parser.parse_args()
    model_path = arguments.model.expanduser().resolve()
    report = dict(
        _inspect(
            model_path,
            base_frame=arguments.base_frame,
            end_frame=arguments.end_frame,
        )
    )
    if arguments.with_pytransform3d:
        report["pytransform3d"] = _load_pytransform3d(model_path)
    report_text = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if arguments.output:
        arguments.output.write_text(report_text, encoding="utf-8")
    else:
        print(report_text, end="")
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
