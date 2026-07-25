import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from myarm_m750_core.domain.kinematics.model import (
    kinematic_contract_fingerprint,
)
from myarm_m750_core.resources import (
    _validate_model_manifest,
    read_kinematic_urdf,
    read_model_manifest,
)

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


def _vector(text):
    return tuple(float(value) for value in text.split())


def _joint_contract(path):
    root = ET.parse(str(path)).getroot()
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    result = {}
    for name in ARM_JOINTS:
        joint = joints[name]
        origin = joint.find("origin")
        axis = joint.find("axis")
        limit = joint.find("limit")
        result[name] = (
            _vector(origin.attrib["xyz"]),
            _vector(origin.attrib["rpy"]),
            _vector(axis.attrib["xyz"]),
            (float(limit.attrib["lower"]), float(limit.attrib["upper"])),
        )
    return result


def _generated_models(repository_root: Path):
    generated = repository_root / "ros2/src/myarm_m750_description/urdf/generated"
    return {
        variant: generated / f"myarm_m750_{variant}.urdf"
        for variant in ("full", "lightweight", "kinematic")
    }


def test_generated_variants_share_one_kinematic_contract(repository_root) -> None:
    models = _generated_models(repository_root)
    contracts = [_joint_contract(path) for path in models.values()]
    assert all(contract == contracts[0] for contract in contracts[1:])
    fingerprints = {
        kinematic_contract_fingerprint(path.read_text(encoding="utf-8"))
        for path in models.values()
    }
    assert len(fingerprints) == 1


def test_arm_origins_and_axes_are_canonical_poe(repository_root) -> None:
    path = _generated_models(repository_root)["kinematic"]
    contract = _joint_contract(path)
    for name, expected_axis in zip(ARM_JOINTS, EXPECTED_AXES):
        assert contract[name][1] == (0.0, 0.0, 0.0)
        assert contract[name][2] == _vector(expected_axis)


def test_variant_geometry_contract(repository_root) -> None:
    models = _generated_models(repository_root)
    full = ET.parse(str(models["full"])).getroot()
    lightweight = ET.parse(str(models["lightweight"])).getroot()
    kinematic = ET.parse(str(models["kinematic"])).getroot()
    assert len(full.findall(".//mesh")) == 9
    assert not full.findall(".//collision//mesh")
    assert not lightweight.findall(".//mesh")
    assert len(lightweight.findall(".//collision")) == 9
    assert not kinematic.findall(".//visual")
    assert not kinematic.findall(".//collision")


def test_manifest_hashes_artifacts_and_core_snapshot(repository_root) -> None:
    manifest_path = (
        repository_root / "ros2/src/myarm_m750_description/config/model_manifest.json"
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    models = _generated_models(repository_root)
    for variant, path in models.items():
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        assert manifest["variants"][variant]["artifact_sha256"] == digest
        assert (
            manifest["variants"][variant]["kinematic_contract_sha256"]
            == manifest["kinematic_contract_sha256"]
        )
    core_snapshot = repository_root / "pycore/src/resources/myarm_m750_kinematic.urdf"
    assert core_snapshot.read_bytes() == models["kinematic"].read_bytes()


def test_installed_core_resource_matches_manifest() -> None:
    urdf_xml = read_kinematic_urdf()
    manifest = read_model_manifest()
    assert (
        hashlib.sha256(urdf_xml.encode("utf-8")).hexdigest()
        == manifest["core_snapshot"]["artifact_sha256"]
    )
    assert kinematic_contract_fingerprint(urdf_xml) == manifest["kinematic_contract_sha256"]


def test_installed_manifest_is_strict_and_deeply_immutable(repository_root) -> None:
    manifest = read_model_manifest()
    with pytest.raises(TypeError):
        manifest["model_name"] = "mutated"
    with pytest.raises(TypeError):
        manifest["variants"]["full"]["path"] = "mutated.urdf"

    manifest_path = (
        repository_root
        / "ros2/src/myarm_m750_description/config/model_manifest.json"
    )
    payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    payload["unknown"] = True
    with pytest.raises(ValueError, match="unknown"):
        _validate_model_manifest(payload, read_kinematic_urdf())


def test_manifest_rejects_resource_hash_drift(repository_root) -> None:
    manifest_path = (
        repository_root
        / "ros2/src/myarm_m750_description/config/model_manifest.json"
    )
    payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    payload["core_snapshot"]["artifact_sha256"] = "0" * 64
    with pytest.raises(ValueError, match="differs"):
        _validate_model_manifest(payload, read_kinematic_urdf())
