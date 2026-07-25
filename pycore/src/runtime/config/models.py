"""Immutable configuration objects accepted by the v0.2 runtime."""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Mapping, Optional, Sequence, Tuple

from myarm_m750_core.domain.models import CapabilityState

CONFIG_VERSION = 1


@dataclass(frozen=True)
class JointMappingConfig:
    """One canonical-radian to firmware-degree mapping."""

    offset_degree: float
    direction: int


def joint_mapping_contract_fingerprint(
    joint_names: Sequence[str],
    mapping: Mapping[str, JointMappingConfig],
) -> str:
    """Fingerprint the ordered canonical-to-firmware software contract."""
    payload = [
        {
            "joint": joint_name,
            "offset_degree": float(mapping[joint_name].offset_degree),
            "direction": int(mapping[joint_name].direction),
        }
        for joint_name in joint_names
    ]
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


@dataclass(frozen=True)
class RuntimeConfig:
    """Runtime publication and command rates."""

    command_rate_hz: float
    state_rate_hz: float


@dataclass(frozen=True)
class RobotConfig:
    """Canonical robot model, frames, joints, mapping, and provenance."""

    name: str
    joint_names: Tuple[str, ...]
    urdf_path: Path
    base_link: str
    end_link: str
    resource_fingerprint: str
    kinematic_contract_fingerprint: str
    joint_mapping: Mapping[str, JointMappingConfig]
    runtime: RuntimeConfig

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "joint_mapping",
            MappingProxyType(dict(self.joint_mapping)),
        )


@dataclass(frozen=True)
class SingularityConfig:
    """Optional singularity guard settings."""

    enabled: bool
    minimum_score: float


@dataclass(frozen=True)
class WorkspaceConfig:
    """Axis-aligned workspace boundary in the base frame."""

    minimum_m: Tuple[float, float, float]
    maximum_m: Tuple[float, float, float]
    resample_step_rad: float


@dataclass(frozen=True)
class SafetyConfig:
    """Trajectory safety policy and its traceable provenance."""

    enabled: bool
    provenance: str
    max_trajectory_points: int
    max_workspace_resample_samples: int
    state_timeout_s: float
    command_timeout_s: float
    stop_timeout_s: float
    max_joint_step_rad: float
    max_joint_velocity_rad_s: Tuple[float, ...]
    max_joint_acceleration_rad_s2: Tuple[float, ...]
    joint_limit_margin_rad: float
    workspace: WorkspaceConfig
    singularity: SingularityConfig


@dataclass(frozen=True)
class LoggingConfig:
    """Process logging settings."""

    level: str
    console: bool
    file: str
    max_bytes: int
    backup_count: int
    json_file: bool


@dataclass(frozen=True)
class MockAdapterProfile:
    """Deterministic in-memory adapter settings."""

    initial_position_rad: Tuple[float, ...]


@dataclass(frozen=True)
class ReplayAdapterProfile:
    """Read-only replay adapter settings."""

    replay_file: Path
    loop: bool


@dataclass(frozen=True)
class FirmwareProtocolProfile:
    """Firmware protocol identity and bounded command settings."""

    expected_version: str
    speed: int


@dataclass(frozen=True)
class CapabilityVerificationProfile:
    """Expected capability states verified during a hardware probe."""

    verification_reference: str
    stop: CapabilityState
    pause: CapabilityState
    resume: CapabilityState
    power_control: CapabilityState


@dataclass(frozen=True)
class HardwareProfile:
    """Explicit real-hardware profile; every risky field is mandatory."""

    serial_by_id: str
    baudrate: int
    operation_deadline_s: float
    max_retries: int
    retry_delay_s: float
    expected_model: str
    mapping_fingerprint: str
    firmware: FirmwareProtocolProfile
    capabilities: CapabilityVerificationProfile
    debug: bool


@dataclass(frozen=True)
class AdapterConfig:
    """Exactly one selected, fully typed adapter profile."""

    adapter_type: str
    mock: Optional[MockAdapterProfile] = None
    replay: Optional[ReplayAdapterProfile] = None
    hardware: Optional[HardwareProfile] = None


@dataclass(frozen=True)
class SdkConfig:
    """Resolved SDK startup configuration."""

    config_version: int
    source_path: Path
    robot: RobotConfig
    safety: SafetyConfig
    logging: LoggingConfig
    adapter: AdapterConfig
