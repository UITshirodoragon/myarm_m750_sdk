#!/usr/bin/env python3
"""Build and read a mock session using only installed wheel resources."""

from __future__ import annotations

import hashlib
import tempfile
from pathlib import Path
from typing import Mapping

import yaml
from myarm_m750_core import RobotSessionBuilder
from myarm_m750_core.resources import (
    read_kinematic_urdf,
    read_model_manifest,
)

_JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_flex_joint",
    "forearm_roll_joint",
    "wrist_flex_joint",
    "wrist_roll_joint",
)


def _write_yaml(path: Path, document: Mapping[str, object]) -> None:
    path.write_text(
        yaml.safe_dump(document, sort_keys=False),
        encoding="utf-8",
    )


def _robot_document(
    model_path: Path,
    manifest: Mapping[str, object],
) -> Mapping[str, object]:
    contract_hash = str(manifest["kinematic_contract_sha256"])
    artifact_hash = hashlib.sha256(model_path.read_bytes()).hexdigest()
    mapping = {
        joint_name: {
            "offset_degree": (
                10.0
                if joint_name == "shoulder_lift_joint"
                else -10.0
                if joint_name == "elbow_flex_joint"
                else 0.0
            ),
            "direction": 1,
        }
        for joint_name in _JOINT_NAMES
    }
    return {
        "config_version": 1,
        "robot": {
            "name": "myarm_m750",
            "joint_names": list(_JOINT_NAMES),
            "model": {
                "urdf_path": str(model_path),
                "base_link": "base_link",
                "end_link": "tool0",
                "resource_sha256": artifact_hash,
                "kinematic_contract_sha256": contract_hash,
            },
            "joint_mapping": mapping,
            "runtime": {"command_rate_hz": 5.0, "state_rate_hz": 5.0},
        },
    }


def _safety_document() -> Mapping[str, object]:
    return {
        "config_version": 1,
        "safety": {
            "enabled": True,
            "provenance": "wheel_install_smoke",
            "max_trajectory_points": 1000,
            "max_workspace_resample_samples": 10000,
            "state_timeout_s": 1.0,
            "command_timeout_s": 1.0,
            "stop_timeout_s": 1.0,
            "max_joint_step_rad": 0.08,
            "max_joint_velocity_rad_s": [1.0] * 6,
            "max_joint_acceleration_rad_s2": [2.0] * 6,
            "joint_limit_margin_rad": 0.01,
            "workspace": {
                "minimum_m": [-1.0, -1.0, -1.0],
                "maximum_m": [1.0, 1.0, 1.0],
                "resample_step_rad": 0.04,
            },
            "singularity": {"enabled": False, "minimum_score": 0.002},
        },
    }


def main() -> int:
    """Create deployment-owned YAML, compose the wheel, and read mock state."""
    manifest = read_model_manifest()
    with tempfile.TemporaryDirectory(prefix="myarm-m750-wheel-smoke-") as temp_name:
        temp_directory = Path(temp_name)
        model_path = temp_directory / "myarm_m750_kinematic.urdf"
        model_path.write_text(read_kinematic_urdf(), encoding="utf-8")
        _write_yaml(
            temp_directory / "robot.yaml",
            _robot_document(model_path, manifest),
        )
        _write_yaml(temp_directory / "safety.yaml", _safety_document())
        _write_yaml(
            temp_directory / "logging.yaml",
            {
                "config_version": 1,
                "logging": {
                    "level": "INFO",
                    "console": False,
                    "file": str(temp_directory / "sdk.jsonl"),
                    "max_bytes": 100_000,
                    "backup_count": 1,
                    "json_file": True,
                },
            },
        )
        sdk_path = temp_directory / "sdk.yaml"
        _write_yaml(
            sdk_path,
            {
                "config_version": 1,
                "sdk": {
                    "config_files": {
                        "robot": "robot.yaml",
                        "safety": "safety.yaml",
                        "logging": "logging.yaml",
                    },
                    "adapter": {
                        "type": "mock",
                        "mock": {"initial_position_rad": [0.0] * 6},
                    },
                },
            },
        )
        with RobotSessionBuilder.from_file(str(sdk_path)).build() as session:
            state = session.read_joint_state()
            if state.position_rad != (0.0,) * 6:
                raise RuntimeError(
                    f"Unexpected installed-wheel mock state: {state.position_rad}"
                )
    print("PASS installed wheel mock composition/read smoke.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
