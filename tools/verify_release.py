#!/usr/bin/env python3
"""Run source-only release checks that do not require ROS 2 or robot hardware."""

from __future__ import annotations

import ast
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import yaml

ROOT = Path(__file__).resolve().parents[1]
VERSION = (ROOT / "VERSION").read_text(encoding="utf-8").strip()
ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_flex_joint",
    "forearm_roll_joint",
    "wrist_flex_joint",
    "wrist_roll_joint",
)
EXPECTED_AXES = (
    "0 0 1",
    "0 1 0",
    "0 1 0",
    "1 0 0",
    "0 1 0",
    "1 0 0",
)
PYCORE_DIRS = {
    "api",
    "application",
    "domain",
    "ports",
    "adapters",
    "runtime",
    "diagnostics",
}
ROS2_DIRS = {
    "description",
    "driver",
    "bringup",
    "visualization",
    "camera",
    "moveit_config",
    "gazebo",
    "msgs",
}


def fail(message: str) -> None:
    raise AssertionError(message)


def normalize_vector(text: str) -> Tuple[float, ...]:
    return tuple(float(value) for value in text.split())


def check_required_paths() -> None:
    required_files = (
        "README.md",
        "CHANGELOG.md",
        "VERSION",
        "AGENTS.md",
        "agent.md",
        "plans.md",
        "requirements/base.txt",
        "requirements/dev.txt",
        "requirements/camera.txt",
        "requirements/camera-host.txt",
        "requirements/camera-jetson.txt",
        "requirements/serial.txt",
        "requirements/simulation.txt",
        "requirements/ros2.txt",
        "pycore/pyproject.toml",
        "pycore/config/default.yaml",
        "pycore/config/camera/cameras.yaml",
        "pycore/src/__init__.py",
        "pycore/src/api/camera_session.py",
        "pycore/src/adapters/camera/opencv_capture.py",
        "ros2/src/myarm_m750_description/urdf/myarm_m750_poe_v3_2.urdf",
        "ros2/src/myarm_m750_driver/myarm_m750_driver/driver_node.py",
        "ros2/src/myarm_m750_visualization/rviz/robot_host.rviz",
        "ros2/src/myarm_m750_msgs/msg/DiagnosticEvent.msg",
        "docs/design/01-kien-truc-va-pham-vi.qmd",
        "docs/development/clean-code-clean-comment-robotics.qmd",
    )
    missing = [path for path in required_files if not (ROOT / path).is_file()]
    if missing:
        fail("Missing required release files: {0}".format(", ".join(missing)))


def check_requested_layout() -> None:
    pycore_root = ROOT / "pycore/src"
    pycore_actual = {
        path.name
        for path in pycore_root.iterdir()
        if path.is_dir() and not path.name.startswith(".") and path.name != "__pycache__"
    }
    if pycore_actual != PYCORE_DIRS:
        fail(
            "Unexpected pycore/src layout. expected={0}, actual={1}".format(
                sorted(PYCORE_DIRS), sorted(pycore_actual)
            )
        )
    if (pycore_root / "myarm_m750_core").exists():
        fail("Physical src/myarm_m750_core wrapper must not be reintroduced.")

    ros2_root = ROOT / "ros2/src"
    ros2_actual = {
        path.name
        for path in ros2_root.iterdir()
        if path.is_dir() and not path.name.startswith(".") and path.name != "__pycache__"
    }
    if ros2_actual != ROS2_DIRS:
        fail(
            "Unexpected ros2/src layout. expected={0}, actual={1}".format(
                sorted(ROS2_DIRS), sorted(ros2_actual)
            )
        )


def check_python_package_mapping() -> None:
    pyproject = (ROOT / "pycore/pyproject.toml").read_text(encoding="utf-8")
    if 'package-dir = {"myarm_m750_core" = "src"}' not in pyproject:
        fail("pyproject.toml must map physical pycore/src to myarm_m750_core.")
    for required_package in (
        '"myarm_m750_core.api"',
        '"myarm_m750_core.domain.kinematics"',
        '"myarm_m750_core.adapters.camera"',
        '"myarm_m750_core.runtime.config"',
    ):
        if required_package not in pyproject:
            fail("Missing explicit package mapping: {0}".format(required_package))


def check_yaml_files() -> None:
    for path in ROOT.rglob("*.yaml"):
        with path.open("r", encoding="utf-8") as stream:
            yaml.safe_load(stream)


def check_xml_files() -> None:
    for suffix in ("*.xml", "*.urdf"):
        for path in ROOT.rglob(suffix):
            ET.parse(str(path))


def package_versions() -> Iterable[Tuple[Path, str]]:
    for package_xml in (ROOT / "ros2/src").glob("*/package.xml"):
        root = ET.parse(str(package_xml)).getroot()
        version_element = root.find("version")
        if version_element is None or version_element.text is None:
            fail("Missing <version> in {0}".format(package_xml))
        yield package_xml, version_element.text.strip()

    pyproject = (ROOT / "pycore/pyproject.toml").read_text(encoding="utf-8")
    match = re.search(r'^version\s*=\s*"([^"]+)"', pyproject, flags=re.MULTILINE)
    if match is None:
        fail("Could not read pycore version from pyproject.toml")
    yield ROOT / "pycore/pyproject.toml", match.group(1)

    for setup_py in (ROOT / "ros2/src").glob("*/setup.py"):
        text = setup_py.read_text(encoding="utf-8")
        match = re.search(r'version\s*=\s*"([^"]+)"', text)
        if match is None:
            fail("Could not read version from {0}".format(setup_py))
        yield setup_py, match.group(1)


def check_versions() -> None:
    mismatches = [
        "{0}: {1}".format(path.relative_to(ROOT), version)
        for path, version in package_versions()
        if version != VERSION
    ]
    if mismatches:
        fail("Version mismatch against {0}: {1}".format(VERSION, "; ".join(mismatches)))

    init_text = (ROOT / "pycore/src/__init__.py").read_text(encoding="utf-8")
    if '__version__ = "{0}"'.format(VERSION) not in init_text:
        fail("pycore/src/__init__.py version does not match VERSION.")


def joint_map(urdf_path: Path) -> Dict[str, ET.Element]:
    root = ET.parse(str(urdf_path)).getroot()
    return {joint.attrib["name"]: joint for joint in root.findall("joint")}


def joint_contract(
    joint: ET.Element,
) -> Tuple[Tuple[float, ...], Tuple[float, ...], Tuple[float, ...], Tuple[float, ...]]:
    origin = joint.find("origin")
    axis = joint.find("axis")
    limit = joint.find("limit")
    if origin is None or axis is None or limit is None:
        fail("Incomplete arm joint definition: {0}".format(joint.attrib.get("name")))
    return (
        normalize_vector(origin.attrib["xyz"]),
        normalize_vector(origin.attrib.get("rpy", "0 0 0")),
        normalize_vector(axis.attrib["xyz"]),
        (float(limit.attrib["lower"]), float(limit.attrib["upper"])),
    )


def check_urdf_contract() -> None:
    supplied = ROOT / "ros2/src/myarm_m750_description/urdf/myarm_m750_poe_v3_2.urdf"
    standalone = ROOT / "ros2/src/myarm_m750_description/urdf/myarm_m750_standalone.urdf"
    supplied_joints = joint_map(supplied)
    standalone_joints = joint_map(standalone)

    for name, axis_text in zip(ARM_JOINTS, EXPECTED_AXES):
        if name not in supplied_joints or name not in standalone_joints:
            fail("Missing arm joint {0}".format(name))
        supplied_contract = joint_contract(supplied_joints[name])
        standalone_contract = joint_contract(standalone_joints[name])
        if supplied_contract != standalone_contract:
            fail("Standalone URDF diverges from supplied URDF at {0}".format(name))
        if supplied_contract[1] != (0.0, 0.0, 0.0):
            fail("Arm joint origin rpy must remain canonical at {0}".format(name))
        if supplied_contract[2] != normalize_vector(axis_text):
            fail("Unexpected PoE axis at {0}".format(name))

    robot_config = yaml.safe_load(
        (ROOT / "pycore/config/robot_m750.yaml").read_text(encoding="utf-8")
    )["robot"]
    if tuple(robot_config["joint_names"]) != ARM_JOINTS:
        fail("robot_m750.yaml joint order differs from the URDF contract")
    mapping = robot_config["joint_mapping"]
    offsets = tuple(float(mapping[name]["offset_degree"]) for name in ARM_JOINTS)
    if offsets != (0.0, 10.0, -10.0, 0.0, 0.0, 0.0):
        fail("q_ros/q_real offset contract was changed")


def imported_roots(path: Path) -> List[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    imports: List[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.extend(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imports.append(node.module.split(".")[0])
    return imports


def check_dependency_boundaries() -> None:
    core_root = ROOT / "pycore/src"
    ros_independent = (
        core_root / "domain",
        core_root / "runtime",
        core_root / "application",
        core_root / "ports",
    )
    forbidden = {"rclpy", "pymycobot", "cv2"}
    violations: List[str] = []
    for directory in ros_independent:
        for path in directory.rglob("*.py"):
            found = forbidden.intersection(imported_roots(path))
            if found:
                violations.append(
                    "{0}: {1}".format(path.relative_to(ROOT), ", ".join(sorted(found)))
                )
    if violations:
        fail("Core dependency boundary violated: {0}".format("; ".join(violations)))


def check_default_is_mock() -> None:
    default_config = yaml.safe_load(
        (ROOT / "pycore/config/default.yaml").read_text(encoding="utf-8")
    )
    adapter_type = default_config["sdk"]["adapter"]["type"]
    if adapter_type != "mock":
        fail("Safe default must remain the mock adapter, got {0}".format(adapter_type))


def check_camera_contract() -> None:
    data = yaml.safe_load(
        (ROOT / "pycore/config/camera/cameras.yaml").read_text(encoding="utf-8")
    )
    cameras = data.get("cameras", {})
    if not cameras:
        fail("At least one camera config is required.")
    for hardware_name, config in cameras.items():
        if config.get("role") in hardware_name:
            fail("Camera role must not be embedded in hardware name: {0}".format(hardware_name))
        if not config.get("hardware_serial"):
            fail("Camera hardware_serial is required: {0}".format(hardware_name))
        by_id = config.get("device", {}).get("by_id", "")
        if "/dev/v4l/by-id/" not in by_id:
            fail("Camera must prefer a stable by-id path: {0}".format(hardware_name))


def check_requirements_profiles() -> None:
    all_text = (ROOT / "requirements/all.txt").read_text(encoding="utf-8")
    for profile in ("dev.txt", "camera.txt", "serial.txt", "simulation.txt"):
        if "-r {0}".format(profile) not in all_text:
            fail("requirements/all.txt does not include {0}".format(profile))
    ros2_text = (ROOT / "requirements/ros2.txt").read_text(encoding="utf-8")
    if "rclpy" in ros2_text.lower() and "not" not in ros2_text.lower():
        fail("requirements/ros2.txt must not pip-install rclpy.")


def main() -> int:
    checks = (
        check_required_paths,
        check_requested_layout,
        check_python_package_mapping,
        check_yaml_files,
        check_xml_files,
        check_versions,
        check_urdf_contract,
        check_dependency_boundaries,
        check_default_is_mock,
        check_camera_contract,
        check_requirements_profiles,
    )
    for check in checks:
        check()
        print("PASS {0}".format(check.__name__))
    print("Release verification passed for v{0}.".format(VERSION))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print("FAIL {0}: {1}".format(type(error).__name__, error), file=sys.stderr)
        sys.exit(1)
