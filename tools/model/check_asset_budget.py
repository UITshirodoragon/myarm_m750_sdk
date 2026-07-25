#!/usr/bin/env python3
"""Report visual COLLADA size/triangle budgets without modifying assets."""

from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as element_tree
from collections.abc import Mapping
from pathlib import Path
from typing import Dict

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
VISUAL_DIRECTORY = REPOSITORY_ROOT / "ros2/src/myarm_m750_description/meshes/visual"
TOTAL_SIZE_BUDGET_BYTES = 40 * 1024 * 1024
TOTAL_TRIANGLE_BUDGET = 300_000
PER_MESH_TRIANGLE_BUDGET = 75_000


def _triangle_count(path: Path) -> int:
    root = element_tree.parse(str(path)).getroot()
    triangle_count = 0
    for element in root.iter():
        local_name = element.tag.rsplit("}", 1)[-1]
        if local_name == "triangles":
            triangle_count += int(element.attrib.get("count", "0"))
        elif local_name == "polylist":
            vcount = next(
                (child for child in element if child.tag.rsplit("}", 1)[-1] == "vcount"),
                None,
            )
            if vcount is not None and vcount.text:
                triangle_count += sum(
                    max(0, int(vertex_count) - 2) for vertex_count in vcount.text.split()
                )
    return triangle_count


def _report() -> Mapping[str, object]:
    meshes: Dict[str, Mapping[str, object]] = {}
    total_size_bytes = 0
    total_triangles = 0
    for path in sorted(VISUAL_DIRECTORY.glob("*.dae")):
        size_bytes = path.stat().st_size
        triangle_count = _triangle_count(path)
        total_size_bytes += size_bytes
        total_triangles += triangle_count
        meshes[path.name] = {
            "size_bytes": size_bytes,
            "triangle_count": triangle_count,
            "within_triangle_budget": triangle_count <= PER_MESH_TRIANGLE_BUDGET,
        }
    passed = (
        total_size_bytes <= TOTAL_SIZE_BUDGET_BYTES
        and total_triangles <= TOTAL_TRIANGLE_BUDGET
        and all(item["within_triangle_budget"] for item in meshes.values())
    )
    return {
        "passed": passed,
        "budgets": {
            "total_size_bytes": TOTAL_SIZE_BUDGET_BYTES,
            "total_triangles": TOTAL_TRIANGLE_BUDGET,
            "per_mesh_triangles": PER_MESH_TRIANGLE_BUDGET,
        },
        "observed": {
            "total_size_bytes": total_size_bytes,
            "total_triangles": total_triangles,
        },
        "meshes": meshes,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--enforce",
        action="store_true",
        help="Return failure when supplied visual assets exceed release budgets.",
    )
    arguments = parser.parse_args()
    report = _report()
    print(json.dumps(report, indent=2, sort_keys=True))
    return 1 if arguments.enforce and not report["passed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
