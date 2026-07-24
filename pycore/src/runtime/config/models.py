"""Validated configuration value objects."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Mapping, Tuple


@dataclass(frozen=True)
class JointMappingConfig:
    """One canonical-to-firmware joint mapping."""

    offset_degree: float = 0.0
    direction: int = 1


@dataclass(frozen=True)
class RuntimeConfig:
    """Bounded initial runtime rates."""

    command_rate_hz: float
    state_rate_hz: float
    realtime_execution: bool


@dataclass(frozen=True)
class RobotConfig:
    """Robot model and adapter-boundary configuration."""

    name: str
    joint_names: Tuple[str, ...]
    urdf_path: Path
    base_link: str
    end_link: str
    joint_mapping: Mapping[str, JointMappingConfig]
    runtime: RuntimeConfig


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


@dataclass(frozen=True)
class SafetyConfig:
    """Safety validation settings."""

    enabled: bool
    state_timeout_s: float
    command_timeout_s: float
    max_joint_step_rad: float
    joint_limit_margin_rad: float
    reject_nan_or_inf: bool
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
class AdapterConfig:
    """Selected hardware adapter and its owned options."""

    adapter_type: str
    options: Dict[str, object]


@dataclass(frozen=True)
class SdkConfig:
    """Resolved SDK startup configuration."""

    source_path: Path
    robot: RobotConfig
    safety: SafetyConfig
    logging: LoggingConfig
    adapter: AdapterConfig
