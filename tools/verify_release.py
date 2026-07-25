#!/usr/bin/env python3
"""Run source-only release checks without ROS 2 or physical hardware."""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable, List, Mapping, NoReturn, Optional, Sequence, Tuple

import yaml

ROOT = Path(__file__).resolve().parents[1]
VERSION = (ROOT / "VERSION").read_text(encoding="utf-8").strip()
INVENTORY_PATH = ROOT / "docs/release/inventory.yaml"
ACTIVE_ROS_PACKAGES = {
    "myarm_m750_bringup",
    "myarm_m750_camera",
    "myarm_m750_description",
    "myarm_m750_driver",
    "myarm_m750_moveit_config",
    "myarm_m750_visualization",
}
PYCORE_DIRS = {
    "adapters",
    "api",
    "application",
    "diagnostics",
    "domain",
    "ports",
    "resources",
    "runtime",
}
FORBIDDEN_RELEASE_PATHS = (
    "agent.md",
    "plans.md",
    "docker",
    "simulation",
    "pycore/config/camera/cameras.yaml",
    "pycore/config/default_real.yaml",
    "ros2/src/myarm_m750_bringup/config/network_jetson_local.yaml",
    "ros2/src/myarm_m750_description/urdf/meshes",
    "ros2/src/myarm_m750_description/urdf/myarm_m750_poe_v3_2.urdf",
    "ros2/src/myarm_m750_description/urdf/myarm_m750_standalone.urdf",
    "ros2/src/myarm_m750_gazebo",
    "ros2/src/myarm_m750_driver/config/robot_real.yaml",
    "ros2/src/myarm_m750_msgs",
    "ros2/src/myarm_m750_visualization/config/visualization.yaml",
    "ros2/src/myarm_m750_visualization/myarm_m750_visualization/marker_node.py",
    "ros2/src/myarm_m750_visualization/rviz/debug_host.rviz",
    "ros2/src/myarm_m750_visualization/rviz/myarm_m750_mdh_v3_2.rviz",
)
GENERATED_MODEL_PATHS = (
    "ros2/src/myarm_m750_description/urdf/generated/myarm_m750_full.urdf",
    "ros2/src/myarm_m750_description/urdf/generated/myarm_m750_lightweight.urdf",
    "ros2/src/myarm_m750_description/urdf/generated/myarm_m750_kinematic.urdf",
    "ros2/src/myarm_m750_description/config/model_manifest.json",
    "pycore/src/resources/myarm_m750_kinematic.urdf",
    "pycore/src/resources/model_manifest.json",
)
FORBIDDEN_GENERATED_COMPONENTS = {
    "__pycache__",
    ".mypy_cache",
    ".pytest_cache",
    ".ruff_cache",
}
FORBIDDEN_GENERATED_SUFFIXES = {".pyc", ".pyo"}


def fail(message: str) -> NoReturn:
    """Raise a release-check failure with actionable context."""
    raise AssertionError(message)


def required_paths() -> Sequence[str]:
    """Return source paths required in every release."""
    paths = [
        ".github/workflows/core-ci.yml",
        ".github/workflows/ros2-foxy-ci.yml",
        "AGENTS.md",
        "CHANGELOG.md",
        "LICENSE",
        "PLANS.md",
        "README.md",
        "VERSION",
        "docs/development/clean-code-clean-comment-robotics.qmd",
        "docs/deployment/remote-rviz2-wlan.md",
        "docs/release/IMPLEMENTATION_NOTES.md",
        "docs/release/MIGRATION_0.2.md",
        "docs/release/VERIFICATION.md",
        "docs/release/inventory.yaml",
        "pycore/config/camera/calibration/mock_640x480.yaml",
        "pycore/config/camera/cameras_mock.yaml",
        "pycore/config/camera/cameras_real.example.yaml",
        "pycore/config/default.yaml",
        "pycore/config/default_real.example.yaml",
        "pycore/pyproject.toml",
        "pycore/src/__init__.py",
        "requirements/README.md",
        "requirements/constraints-py38.txt",
        "ros2/README.md",
        "ros2/src/myarm_m750_camera/launch/camera_bridge.launch.py",
        "ros2/src/myarm_m750_camera/myarm_m750_camera/camera_bridge.py",
        (
            "ros2/src/myarm_m750_moveit_config/test/"
            "moveit_runtime_probe.py"
        ),
        "tools/bootstrap_core.sh",
        "tools/bootstrap_ros.sh",
        "tools/check_coverage.py",
        "tools/moveit_runtime_gate.py",
        "tools/model/check_asset_budget.py",
        "tools/model/generate_models.py",
        "tools/model/inspect_model.py",
        "tools/ros_runtime_gate.py",
        "tools/test_all.sh",
        "tools/test_core.sh",
        "tools/test_ros.sh",
        "tools/verify_release.py",
        "tools/wheel_smoke.py",
    ]
    paths.extend(GENERATED_MODEL_PATHS)
    paths.extend(f"ros2/src/{package}" for package in sorted(ACTIVE_ROS_PACKAGES))
    return paths


def check_required_paths() -> None:
    """Require canonical docs, active packages, model artifacts and gate tools."""
    missing = [path for path in required_paths() if not (ROOT / path).exists()]
    if missing:
        fail(f"Missing required release paths: {', '.join(missing)}")

    non_executable = [
        path
        for path in (
            "tools/bootstrap_core.sh",
            "tools/bootstrap_ros.sh",
            "tools/test_all.sh",
            "tools/test_core.sh",
            "tools/test_ros.sh",
        )
        if not os.access(str(ROOT / path), os.X_OK)
    ]
    if non_executable:
        fail(f"Quality scripts must be executable: {', '.join(non_executable)}")


def check_forbidden_paths() -> None:
    """Reject removed aliases, custom messages and unimplemented scaffolds."""
    present = [path for path in FORBIDDEN_RELEASE_PATHS if (ROOT / path).exists()]
    if present:
        fail(f"Legacy/scaffold paths must be removed: {', '.join(present)}")


def check_requested_layout() -> None:
    """Validate the physical Python and ROS package boundaries."""
    pycore_root = ROOT / "pycore/src"
    pycore_actual = {
        path.name
        for path in pycore_root.iterdir()
        if path.is_dir() and path.name != "__pycache__" and not path.name.startswith(".")
    }
    if pycore_actual != PYCORE_DIRS:
        fail(
            "Unexpected pycore/src layout. "
            f"expected={sorted(PYCORE_DIRS)}, actual={sorted(pycore_actual)}"
        )
    if (pycore_root / "myarm_m750_core").exists():
        fail("Physical pycore/src/myarm_m750_core wrapper must not be reintroduced.")

    ros2_root = ROOT / "ros2/src"
    ros2_actual = {
        path.name
        for path in ros2_root.iterdir()
        if path.is_dir() and path.name != "__pycache__" and not path.name.startswith(".")
    }
    if ros2_actual != ACTIVE_ROS_PACKAGES:
        fail(
            "Unexpected ros2/src layout. "
            f"expected={sorted(ACTIVE_ROS_PACKAGES)}, actual={sorted(ros2_actual)}"
        )


def inventory_entries() -> List[Mapping[str, object]]:
    """Load and validate the machine-readable release inventory."""
    payload = yaml.safe_load(INVENTORY_PATH.read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or payload.get("inventory_version") != 1:
        fail("docs/release/inventory.yaml must use inventory_version: 1")
    entries = payload.get("entries")
    if not isinstance(entries, list) or not entries:
        fail("Release inventory must contain a non-empty entries list.")
    if any(not isinstance(entry, dict) for entry in entries):
        fail("Every release inventory entry must be a mapping.")
    return entries


def releasable_worktree_files() -> List[str]:
    """Return cached and untracked, non-ignored files in the worktree.

    A breaking refactor commonly contains new files before it is staged.  A
    verifier that only inspects the Git index can therefore report a false
    success while skipping the implementation being reviewed.
    """
    if not (ROOT / ".git").exists():
        fail("Inventory verification requires a Git worktree.")
    process = subprocess.run(
        [
            "git",
            "ls-files",
            "-z",
            "--cached",
            "--others",
            "--exclude-standard",
        ],
        cwd=str(ROOT),
        check=True,
        stdout=subprocess.PIPE,
    )
    paths = process.stdout.decode("utf-8").split("\0")
    return [path for path in paths if path and (ROOT / path).is_file()]


def matching_inventory_entry(
    relative_path: str,
    entries: Iterable[Mapping[str, object]],
) -> Mapping[str, object]:
    """Return the most specific inventory prefix for one tracked path."""
    candidates = []
    for entry in entries:
        prefix = str(entry["path"]).rstrip("/")
        if relative_path == prefix or relative_path.startswith(f"{prefix}/"):
            candidates.append(entry)
    if not candidates:
        fail(f"Tracked file has no inventory owner/consumer: {relative_path}")
    return max(candidates, key=lambda item: len(str(item["path"])))


def check_release_inventory() -> None:
    """Require owner/consumer metadata and reject tracked generated/legacy files."""
    entries = inventory_entries()
    allowed_classes = {"active", "optional", "generated", "legacy"}
    seen_paths = set()
    for entry in entries:
        path_value = entry.get("path")
        classification = entry.get("classification")
        owner = entry.get("owner")
        consumer = entry.get("consumer")
        release = entry.get("release")
        if not all(
            isinstance(value, str) and value for value in (path_value, owner, consumer)
        ):
            fail(f"Invalid inventory metadata: {entry}")
        path = str(path_value)
        if path in seen_paths:
            fail(f"Duplicate inventory path: {path}")
        seen_paths.add(path)
        if classification not in allowed_classes:
            fail(f"Invalid inventory classification for {path}: {classification}")
        if not isinstance(release, bool):
            fail(f"Inventory release flag must be boolean: {path}")
        if classification in {"active", "optional"}:
            if not release or not (ROOT / path).exists():
                fail(f"Active/optional inventory path is not releasable: {path}")
        elif release:
            fail(f"Generated/legacy inventory path cannot be releasable: {path}")

    for worktree_path in releasable_worktree_files():
        path = Path(worktree_path)
        if (
            FORBIDDEN_GENERATED_COMPONENTS.intersection(path.parts)
            or path.suffix in FORBIDDEN_GENERATED_SUFFIXES
        ):
            fail(f"Generated cache file cannot be released: {worktree_path}")
        entry = matching_inventory_entry(worktree_path, entries)
        classification = entry["classification"]
        if classification not in {"active", "optional"} or not entry["release"]:
            fail(
                f"Worktree file is classified {classification}, not releasable: "
                f"{worktree_path}"
            )


def check_python_package_mapping() -> None:
    """Keep the short physical layout and installed namespace contract."""
    pyproject = (ROOT / "pycore/pyproject.toml").read_text(encoding="utf-8")
    expected_mapping = 'package-dir = {"myarm_m750_core" = "src"}'
    if expected_mapping not in pyproject:
        fail("pyproject.toml must map pycore/src to myarm_m750_core.")
    for required_package in (
        '"myarm_m750_core.adapters.kinematics"',
        '"myarm_m750_core.api"',
        '"myarm_m750_core.domain.kinematics"',
        '"myarm_m750_core.resources"',
        '"myarm_m750_core.runtime.config"',
    ):
        if required_package not in pyproject:
            fail(f"Missing explicit package mapping: {required_package}")
    package_data_pattern = (
        r'^"myarm_m750_core\.resources"\s*=\s*\["\*\.urdf",\s*"\*\.json"\]'
    )
    if re.search(package_data_pattern, pyproject, re.MULTILINE) is None:
        fail("Core kinematic model resources must be included as package data.")


def source_files_with_suffixes(suffixes: Iterable[str]) -> Iterable[Path]:
    """Yield tracked source files matching one of the requested suffixes."""
    suffix_set = set(suffixes)
    for relative_path in releasable_worktree_files():
        path = ROOT / relative_path
        if path.suffix in suffix_set:
            yield path


def check_yaml_and_xml_files() -> None:
    """Parse every tracked YAML, XML, URDF and Xacro source artifact."""
    for path in source_files_with_suffixes((".yaml", ".yml")):
        with path.open("r", encoding="utf-8") as stream:
            yaml.safe_load(stream)
    for path in source_files_with_suffixes((".xml", ".urdf", ".xacro")):
        ET.parse(str(path))


def package_versions() -> Iterable[Tuple[Path, str]]:
    """Yield declared versions from Python and ROS package metadata."""
    for package_xml in (ROOT / "ros2/src").glob("*/package.xml"):
        root = ET.parse(str(package_xml)).getroot()
        version_element = root.find("version")
        if version_element is None or version_element.text is None:
            fail(f"Missing <version> in {package_xml}")
        yield package_xml, version_element.text.strip()

    pyproject = (ROOT / "pycore/pyproject.toml").read_text(encoding="utf-8")
    match = re.search(r'^version\s*=\s*"([^"]+)"', pyproject, flags=re.MULTILINE)
    if match is None:
        fail("Could not read core version from pycore/pyproject.toml")
    yield ROOT / "pycore/pyproject.toml", match.group(1)

    for setup_py in (ROOT / "ros2/src").glob("*/setup.py"):
        text = setup_py.read_text(encoding="utf-8")
        match = re.search(r'version\s*=\s*"([^"]+)"', text)
        if match is None:
            fail(f"Could not read version from {setup_py}")
        yield setup_py, match.group(1)


def check_versions() -> None:
    """Require one version across Python and ROS release artifacts."""
    mismatches = [
        f"{path.relative_to(ROOT)}: {version}"
        for path, version in package_versions()
        if version != VERSION
    ]
    if mismatches:
        fail(f"Version mismatch against {VERSION}: {'; '.join(mismatches)}")

    init_text = (ROOT / "pycore/src/__init__.py").read_text(encoding="utf-8")
    if f'__version__ = "{VERSION}"' not in init_text:
        fail("pycore/src/__init__.py version does not match VERSION.")


def check_generated_models() -> None:
    """Validate model manifests and artifacts without importing ROS/xacro."""
    description_root = ROOT / "ros2/src/myarm_m750_description"
    description_manifest_path = description_root / "config/model_manifest.json"
    core_manifest_path = ROOT / "pycore/src/resources/model_manifest.json"
    description_manifest_bytes = description_manifest_path.read_bytes()
    if description_manifest_bytes != core_manifest_path.read_bytes():
        fail("Description and core model manifests differ.")

    manifest = json.loads(description_manifest_bytes)
    if manifest.get("schema_version") != 1:
        fail("Model manifest must use schema_version: 1")
    source = manifest.get("source", {})
    source_path = description_root / str(source.get("path", ""))
    if not source_path.is_file():
        fail(f"Model manifest source does not exist: {source_path}")
    source_sha256 = hashlib.sha256(source_path.read_bytes()).hexdigest()
    if source_sha256 != source.get("sha256"):
        fail("Canonical Xacro hash differs from model manifest.")

    contract_sha256 = manifest.get("kinematic_contract_sha256")
    variants = manifest.get("variants", {})
    if set(variants) != {"full", "lightweight", "kinematic"}:
        fail("Model manifest must contain full, lightweight and kinematic variants.")
    for variant_name, variant in variants.items():
        artifact_path = description_root / str(variant.get("path", ""))
        artifact_bytes = artifact_path.read_bytes()
        artifact_sha256 = hashlib.sha256(artifact_bytes).hexdigest()
        if artifact_sha256 != variant.get("artifact_sha256"):
            fail(f"Artifact hash differs from manifest: {variant_name}")
        if variant.get("kinematic_contract_sha256") != contract_sha256:
            fail(f"Kinematic contract hash differs for variant: {variant_name}")

        robot = ET.fromstring(artifact_bytes)
        collision_meshes = robot.findall(".//collision//mesh")
        if collision_meshes:
            fail(f"Detailed mesh is forbidden in collision geometry: {variant_name}")
        if variant_name == "kinematic" and (
            robot.findall(".//visual") or robot.findall(".//collision")
        ):
            fail("Kinematic model variant must not contain geometry.")

    core_snapshot_path = ROOT / "pycore/src/resources/myarm_m750_kinematic.urdf"
    core_snapshot_sha256 = hashlib.sha256(core_snapshot_path.read_bytes()).hexdigest()
    expected_core_sha256 = manifest.get("core_snapshot", {}).get("artifact_sha256")
    if core_snapshot_sha256 != expected_core_sha256:
        fail("Core kinematic snapshot hash differs from model manifest.")
    kinematic_path = description_root / str(variants["kinematic"]["path"])
    if core_snapshot_path.read_bytes() != kinematic_path.read_bytes():
        fail("Core kinematic snapshot differs from generated kinematic variant.")


def imported_roots(path: Path) -> List[str]:
    """Return top-level imports from one Python source file."""
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    imports: List[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.extend(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imports.append(node.module.split(".")[0])
    return imports


def check_dependency_boundaries() -> None:
    """Keep ROS, vendor, camera and optional geometry imports behind adapters."""
    core_root = ROOT / "pycore/src"
    ros_independent = (
        core_root / "application",
        core_root / "domain",
        core_root / "ports",
        core_root / "runtime",
    )
    forbidden = {
        "cv2",
        "pinocchio",
        "pymycobot",
        "pytransform3d",
        "rclpy",
        "spatialmath",
    }
    violations = []
    for directory in ros_independent:
        for path in directory.rglob("*.py"):
            found = forbidden.intersection(imported_roots(path))
            if found:
                relative_path = path.relative_to(ROOT)
                violations.append(f"{relative_path}: {', '.join(sorted(found))}")
    if violations:
        fail(f"Core dependency boundary violated: {'; '.join(violations)}")


def check_safe_defaults() -> None:
    """Require mock hardware/cameras and disabled ROS command interfaces."""
    default_config = yaml.safe_load(
        (ROOT / "pycore/config/default.yaml").read_text(encoding="utf-8")
    )
    if default_config.get("config_version") != 1:
        fail("pycore/config/default.yaml must declare config_version: 1")
    adapter_type = default_config["sdk"]["adapter"]["type"]
    if adapter_type != "mock":
        fail(f"Safe core default must be mock, got {adapter_type}")

    mock_cameras = yaml.safe_load(
        (ROOT / "pycore/config/camera/cameras_mock.yaml").read_text(encoding="utf-8")
    )
    if mock_cameras.get("config_version") != 1 or mock_cameras.get("profile") != "mock":
        fail("Camera mock profile must declare config_version 1 and profile: mock.")
    cameras = mock_cameras.get("cameras", {})
    if not cameras or any(camera.get("backend") != "mock" for camera in cameras.values()):
        fail("Default camera profile must contain only mock backends.")

    real_camera_example = yaml.safe_load(
        (ROOT / "pycore/config/camera/cameras_real.example.yaml").read_text(
            encoding="utf-8"
        )
    )
    if real_camera_example != {
        "config_version": 1,
        "profile": "real",
        "cameras": {},
    }:
        fail("Camera real.example must remain intentionally empty and non-runnable.")

    real_robot_example = yaml.safe_load(
        (ROOT / "pycore/config/default_real.example.yaml").read_text(encoding="utf-8")
    )
    hardware = real_robot_example["sdk"]["adapter"]["hardware"]
    empty_identity_fields = (
        hardware.get("serial_by_id"),
        hardware.get("expected_model"),
        hardware.get("firmware", {}).get("expected_version"),
    )
    if any(empty_identity_fields):
        fail("Robot real.example identity fields must remain empty and non-runnable.")

    driver_source = (
        ROOT / "ros2/src/myarm_m750_driver/myarm_m750_driver/driver_node.py"
    ).read_text(encoding="utf-8")
    for parameter_name in ("use_real_hardware", "enable_command_interfaces"):
        declaration = f'self.declare_parameter("{parameter_name}", False)'
        if declaration not in driver_source:
            fail(f"ROS driver must declare {parameter_name}=false by default.")

    driver_launch = (ROOT / "ros2/src/myarm_m750_driver/launch/driver.launch.py").read_text(
        encoding="utf-8"
    )
    for argument_name in ("use_real_hardware", "enable_command_interfaces"):
        launch_default = f'"{argument_name}", default_value="false"'
        if launch_default not in driver_launch:
            fail(f"ROS launch must default {argument_name} to false.")

    jetson_local_path = (
        ROOT / "ros2/src/myarm_m750_bringup/config/network_jetson_local.env"
    )
    jetson_local = dict(
        line.split("=", 1)
        for line in jetson_local_path.read_text(encoding="utf-8").splitlines()
        if line and not line.startswith("#")
    )
    host_local = yaml.safe_load(
        (
            ROOT
            / "ros2/src/myarm_m750_visualization/config/network_host_local.yaml"
        ).read_text(encoding="utf-8")
    )["network"]
    local_contract = {
        "RMW_IMPLEMENTATION": host_local["rmw_implementation"],
        "ROS_DOMAIN_ID": str(host_local["ros_domain_id"]),
        "MYARM_M750_WLAN_INTERFACE": host_local["wlan_interface"],
    }
    mismatches = {
        key: (jetson_local.get(key), expected)
        for key, expected in local_contract.items()
        if jetson_local.get(key) != expected
    }
    if mismatches:
        fail(
            "Jetson/Host local DDS profiles must share RMW, domain and interface: "
            f"{mismatches}"
        )
    if (
        jetson_local.get("MYARM_M750_ROLE") != "jetson"
        or host_local.get("role") != "host"
        or host_local.get("interface_address") != "127.0.0.1"
        or host_local.get("discovery", {}).get("mode") != "multicast"
    ):
        fail("Local DDS profiles must remain loopback-only Jetson/Host contracts.")


def check_dependency_policy() -> None:
    """Require pyproject extras, one Python 3.8 constraint file and no pip ROS."""
    requirement_files = sorted(path.name for path in (ROOT / "requirements").glob("*.txt"))
    if requirement_files != ["constraints-py38.txt"]:
        fail(
            "Duplicate requirement profiles are forbidden; expected only "
            f"constraints-py38.txt, got {requirement_files}"
        )

    pyproject = (ROOT / "pycore/pyproject.toml").read_text(encoding="utf-8")
    for extra in ("dev", "serial", "geometry-tools", "camera-host"):
        if re.search(rf"^{re.escape(extra)}\s*=", pyproject, re.MULTILINE) is None:
            fail(f"Missing pyproject optional dependency group: {extra}")
    for removed_extra in ("simulation", "camera"):
        if re.search(
            rf"^{re.escape(removed_extra)}\s*=",
            pyproject,
            re.MULTILINE,
        ):
            fail(f"Removed optional dependency group is still present: {removed_extra}")

    constraints = (ROOT / "requirements/constraints-py38.txt").read_text(encoding="utf-8")
    expected_constraints = (
        "numpy==1.23.5",
        "pymycobot==4.0.5",
        "pytransform3d==3.16.0",
        "pytest==7.4.4",
        "types-PyYAML==6.0.12.20240808",
    )
    for constraint in expected_constraints:
        if constraint not in constraints:
            fail(f"Missing Python 3.8 constraint: {constraint}")

    dependency_block = pyproject.split("[tool.setuptools]", maxsplit=1)[0].lower()
    for system_dependency in ("rclpy", "pinocchio"):
        if system_dependency in dependency_block:
            fail(f"{system_dependency} must be managed by ROS/apt, not pip.")
    declared_dependencies = dependency_block + "\n" + constraints.lower()
    for forbidden_dependency in ("spatialmath", "open3d", "meshcat"):
        if forbidden_dependency in declared_dependencies:
            fail(
                f"{forbidden_dependency} is excluded from v0.2.0 dependencies."
            )


def check_visual_asset_budget() -> None:
    """Block an artifact release while detailed meshes exceed its budget."""
    process = subprocess.run(
        [
            sys.executable,
            str(ROOT / "tools/model/check_asset_budget.py"),
            "--enforce",
        ],
        cwd=str(ROOT),
        check=False,
        capture_output=True,
        encoding="utf-8",
    )
    if process.returncode == 0:
        return
    try:
        report = json.loads(process.stdout)
        observed = report["observed"]
        fail(
            "Visual asset release budget failed: "
            f"{observed['total_size_bytes']} bytes, "
            f"{observed['total_triangles']} triangles."
        )
    except (KeyError, TypeError, json.JSONDecodeError):
        fail(
            "Visual asset budget checker failed without a valid report: "
            f"{process.stderr.strip()}"
        )


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Run source invariants and optionally the final artifact-readiness gate."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--release-ready",
        action="store_true",
        help="Also enforce gates that may intentionally block artifact publication.",
    )
    arguments = parser.parse_args(argv)
    checks = (
        check_required_paths,
        check_forbidden_paths,
        check_requested_layout,
        check_release_inventory,
        check_python_package_mapping,
        check_yaml_and_xml_files,
        check_versions,
        check_generated_models,
        check_dependency_boundaries,
        check_safe_defaults,
        check_dependency_policy,
    )
    for check in checks:
        check()
        print(f"PASS {check.__name__}")
    if arguments.release_ready:
        check_visual_asset_budget()
        print("PASS check_visual_asset_budget")
        print(f"Artifact release readiness passed for v{VERSION}.")
    else:
        print(f"Source release verification passed for v{VERSION}.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"FAIL {type(error).__name__}: {error}", file=sys.stderr)
        sys.exit(1)
