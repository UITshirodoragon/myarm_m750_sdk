"""Composition roots for robot and camera sessions."""

from __future__ import annotations

import importlib.util
import platform
import sys
import time
from pathlib import Path
from typing import TYPE_CHECKING, Callable, Optional, Tuple

from myarm_m750_core.adapters import (
    JointMapper,
    MockRobotAdapter,
    ReplayRobotAdapter,
    VendorSerialRobotAdapter,
)
from myarm_m750_core.adapters.camera import MockCameraAdapter, OpenCvCameraAdapter
from myarm_m750_core.adapters.kinematics import create_kinematics_provider
from myarm_m750_core.application.camera_pipeline import CameraWorker
from myarm_m750_core.application.robot_controller import RobotController
from myarm_m750_core.diagnostics import configure_logging
from myarm_m750_core.domain.camera import CameraConfig
from myarm_m750_core.domain.errors import ConfigurationError
from myarm_m750_core.domain.kinematics import PoeKinematics
from myarm_m750_core.domain.models import (
    AdapterCapabilities,
    EnvironmentInspection,
    HardwareIdentity,
)
from myarm_m750_core.domain.safety import SafetyPolicy, TrajectoryValidator
from myarm_m750_core.ports.camera import CameraCapturePort
from myarm_m750_core.ports.kinematics import KinematicsPort
from myarm_m750_core.ports.robot_hardware import RobotHardwarePort
from myarm_m750_core.runtime.admission import CommandAdmission
from myarm_m750_core.runtime.config import (
    SdkConfig,
    load_camera_configs,
    load_sdk_config,
)
from myarm_m750_core.runtime.config.models import LoggingConfig
from myarm_m750_core.runtime.executor import TrajectoryExecutor
from myarm_m750_core.runtime.scheduler import DeadlineScheduler, Scheduler
from myarm_m750_core.runtime.state_machine import DriverStateMachine
from myarm_m750_core.runtime.trajectory import PointToPointTrajectoryGenerator

AdapterFactory = Callable[[SdkConfig, JointMapper], RobotHardwarePort]
KinematicsFactory = Callable[[SdkConfig], KinematicsPort]
if TYPE_CHECKING:
    from myarm_m750_core.api.camera_session import CameraSession
    from myarm_m750_core.api.session import RobotSession

LoggingConfigurator = Callable[[LoggingConfig], None]
CameraCaptureFactory = Callable[[CameraConfig], CameraCapturePort]


class RobotSessionBuilder:
    """Build an SDK object graph without opening the configured hardware."""

    def __init__(self, config: SdkConfig) -> None:
        self._config = config
        self._adapter_factory: Optional[AdapterFactory] = None
        self._kinematics_factory: Optional[KinematicsFactory] = None
        self._scheduler: Optional[Scheduler] = None
        self._logging_configurator: LoggingConfigurator = configure_logging
        self._operation_clock = time.monotonic

    @classmethod
    def from_file(cls, config_path: str) -> RobotSessionBuilder:
        """Load and fully validate a strict v0.2 manifest without hardware I/O."""
        return cls(load_sdk_config(config_path))

    def with_adapter_factory(self, factory: AdapterFactory) -> RobotSessionBuilder:
        """Inject a vendor-fake or test adapter factory."""
        self._adapter_factory = factory
        return self

    def with_kinematics_factory(self, factory: KinematicsFactory) -> RobotSessionBuilder:
        """Inject a tested kinematics provider factory."""
        self._kinematics_factory = factory
        return self

    def with_pinocchio(self) -> RobotSessionBuilder:
        """Select Pinocchio 2.6.17, falling back only when it is unavailable."""
        self._kinematics_factory = lambda config: create_kinematics_provider(
            urdf_path=config.robot.urdf_path,
            base_link=config.robot.base_link,
            end_link=config.robot.end_link,
            joint_names=config.robot.joint_names,
            prefer_pinocchio=True,
        )
        return self

    def with_scheduler(self, scheduler: Scheduler) -> RobotSessionBuilder:
        """Inject a virtual scheduler for tests or an alternate production clock."""
        self._scheduler = scheduler
        return self

    def with_logging_configurator(
        self, configurator: LoggingConfigurator
    ) -> RobotSessionBuilder:
        """Inject process logging configuration at the composition boundary."""
        self._logging_configurator = configurator
        return self

    def inspect_environment(self) -> EnvironmentInspection:
        """Inspect static resources without importing or opening hardware."""
        issues = []
        if sys.version_info[:2] != (3, 8):
            issues.append(
                f"Release target is Python 3.8; observed {platform.python_version()}."
            )
        resources = {
            "python": platform.python_version(),
            "architecture": platform.machine(),
            "model": str(self._config.robot.urdf_path),
            "model_sha256": self._config.robot.resource_fingerprint,
        }
        if self._config.adapter.adapter_type == "vendor_serial":
            profile = self._config.adapter.hardware
            if profile is None:
                issues.append("Vendor serial profile is missing.")
            else:
                resources["serial_by_id"] = profile.serial_by_id
                if not Path(profile.serial_by_id).exists():
                    issues.append(
                        f"Serial by-id resource is not present: {profile.serial_by_id}"
                    )
                if importlib.util.find_spec("pymycobot") is None:
                    issues.append("Pinned pymycobot==4.0.5 is not importable.")
        if (
            self._config.adapter.adapter_type == "replay"
            and self._config.adapter.replay is not None
            and not self._config.adapter.replay.replay_file.is_file()
        ):
            issues.append(
                f"Replay resource is not present: {self._config.adapter.replay.replay_file}"
            )
        return EnvironmentInspection(
            config_source=str(self._config.source_path),
            adapter_type=self._config.adapter.adapter_type,
            resources=resources,
            issues=tuple(issues),
        )

    def probe_hardware(self) -> HardwareIdentity:
        """Open, read identity/state, and close without sending motion."""
        session = self.build()
        try:
            session.connect()
            return session.probe_hardware()
        finally:
            session.close()

    def build(self) -> RobotSession:
        """Compose a disconnected session after validating every dependency."""
        from myarm_m750_core.api.session import RobotSession

        self._logging_configurator(self._config.logging)
        kinematics = (
            self._kinematics_factory(self._config)
            if self._kinematics_factory is not None
            else PoeKinematics.from_urdf(
                urdf_path=self._config.robot.urdf_path,
                base_link=self._config.robot.base_link,
                end_link=self._config.robot.end_link,
                joint_names=self._config.robot.joint_names,
            )
        )
        if (
            kinematics.info.model_fingerprint_sha256
            != self._config.robot.kinematic_contract_fingerprint
        ):
            raise ConfigurationError(
                "Kinematics provider loaded a different model fingerprint."
            )
        mapper = JointMapper(
            joint_names=self._config.robot.joint_names,
            mapping=self._config.robot.joint_mapping,
        )
        hardware = (
            self._adapter_factory(self._config, mapper)
            if self._adapter_factory is not None
            else self._create_hardware(mapper)
        )
        scheduler = self._scheduler or DeadlineScheduler()
        state_machine = DriverStateMachine()
        safety = self._config.safety
        policy = SafetyPolicy(
            enabled=safety.enabled,
            joint_names=self._config.robot.joint_names,
            joint_limits=kinematics.joint_limits,
            max_trajectory_points=safety.max_trajectory_points,
            max_workspace_resample_samples=safety.max_workspace_resample_samples,
            state_timeout_s=safety.state_timeout_s,
            command_timeout_s=safety.command_timeout_s,
            stop_timeout_s=safety.stop_timeout_s,
            max_joint_step_rad=safety.max_joint_step_rad,
            max_joint_velocity_rad_s=safety.max_joint_velocity_rad_s,
            max_joint_acceleration_rad_s2=safety.max_joint_acceleration_rad_s2,
            joint_limit_margin_rad=safety.joint_limit_margin_rad,
            workspace_minimum_m=safety.workspace.minimum_m,
            workspace_maximum_m=safety.workspace.maximum_m,
            workspace_resample_step_rad=safety.workspace.resample_step_rad,
            singularity_enabled=safety.singularity.enabled,
            minimum_singularity_score=safety.singularity.minimum_score,
            model_fingerprint=self._config.robot.kinematic_contract_fingerprint,
            limit_provenance=safety.provenance,
        )
        validator = TrajectoryValidator(
            kinematics=kinematics,
            policy=policy,
            monotonic_clock=scheduler.now,
        )
        admission = CommandAdmission(
            validator=validator,
            state_reader=lambda: state_machine.state,
            scheduler=scheduler,
        )
        executor = TrajectoryExecutor(
            hardware=hardware,
            admission=admission,
            state_machine=state_machine,
            scheduler=scheduler,
            command_timeout_s=safety.command_timeout_s,
            stop_timeout_s=safety.stop_timeout_s,
            operation_clock=self._operation_clock,
        )
        controller = RobotController(
            joint_names=self._config.robot.joint_names,
            hardware=hardware,
            kinematics=kinematics,
            trajectory_generator=PointToPointTrajectoryGenerator(
                self._config.robot.runtime.command_rate_hz
            ),
            trajectory_executor=executor,
            state_machine=state_machine,
            read_timeout_s=safety.command_timeout_s,
        )
        return RobotSession(config=self._config, controller=controller)

    def _create_hardware(self, mapper: JointMapper) -> RobotHardwarePort:
        adapter = self._config.adapter
        if adapter.adapter_type == "mock" and adapter.mock is not None:
            return MockRobotAdapter(adapter.mock.initial_position_rad)
        if adapter.adapter_type == "replay" and adapter.replay is not None:
            if not adapter.replay.replay_file.is_file():
                raise ConfigurationError(
                    f"Replay file does not exist: {adapter.replay.replay_file}"
                )
            return ReplayRobotAdapter(
                replay_file=str(adapter.replay.replay_file),
                loop=adapter.replay.loop,
            )
        if adapter.adapter_type == "vendor_serial" and adapter.hardware is not None:
            profile = adapter.hardware
            return VendorSerialRobotAdapter(
                port=profile.serial_by_id,
                baudrate=profile.baudrate,
                timeout_s=profile.operation_deadline_s,
                firmware_speed=profile.firmware.speed,
                mapper=mapper,
                max_retries=profile.max_retries,
                retry_delay_s=profile.retry_delay_s,
                expected_model=profile.expected_model,
                expected_firmware_version=profile.firmware.expected_version,
                mapping_fingerprint=profile.mapping_fingerprint,
                capability_verification_reference=(
                    profile.capabilities.verification_reference
                ),
                verified_capabilities=AdapterCapabilities(
                    stop=profile.capabilities.stop,
                    pause=profile.capabilities.pause,
                    resume=profile.capabilities.resume,
                    power_control=profile.capabilities.power_control,
                ),
                debug=profile.debug,
            )
        raise ConfigurationError("Adapter profile is internally inconsistent.")


class CameraSessionBuilder:
    """Compose independent camera workers from one strict profile."""

    def __init__(self, configs: Tuple[CameraConfig, ...]) -> None:
        self._configs = configs
        self._capture_factory: Optional[CameraCaptureFactory] = None

    @classmethod
    def from_file(cls, config_path: str) -> CameraSessionBuilder:
        """Load and validate camera/calibration/extrinsic contracts."""
        return cls(load_camera_configs(config_path))

    def with_capture_factory(self, factory: CameraCaptureFactory) -> CameraSessionBuilder:
        """Inject one fresh capture adapter per camera."""
        self._capture_factory = factory
        return self

    def build(self) -> CameraSession:
        """Build a stopped multi-camera session without opening devices."""
        from myarm_m750_core.api.camera_session import CameraSession

        enabled = tuple(config for config in self._configs if config.enabled)
        if not enabled:
            raise ConfigurationError(
                "Camera profile contains no enabled, deployable cameras."
            )
        workers = {}
        configs = {}
        for config in enabled:
            capture = (
                self._capture_factory(config)
                if self._capture_factory is not None
                else self._default_capture(config)
            )
            workers[config.hardware_name] = CameraWorker(config, capture)
            configs[config.hardware_name] = config
        return CameraSession(configs=configs, workers=workers)

    @staticmethod
    def _default_capture(config: CameraConfig) -> CameraCapturePort:
        if config.backend == "mock":
            return MockCameraAdapter()
        if config.backend == "opencv":
            return OpenCvCameraAdapter()
        raise ConfigurationError(f"Unsupported camera backend: {config.backend}")
