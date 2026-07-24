"""Public Python API for MyArm M750 applications."""

from __future__ import annotations

import logging
import threading
from pathlib import Path
from typing import Callable, Optional, Sequence

import numpy as np

from myarm_m750_core.adapters import (
    JointMapper,
    MockRobotAdapter,
    ReplayRobotAdapter,
    VendorSerialRobotAdapter,
)
from myarm_m750_core.application.robot_controller import RobotController
from myarm_m750_core.runtime.config import SdkConfig, load_sdk_config
from myarm_m750_core.diagnostics import configure_logging
from myarm_m750_core.domain.errors import ConfigurationError
from myarm_m750_core.domain.models import (
    CommandResult,
    HardwareStatus,
    JointState,
    JointTrajectory,
    RigidTransform,
    RobotCapabilities,
)
from myarm_m750_core.domain.kinematics import PoeKinematics
from myarm_m750_core.ports.robot_hardware import RobotHardwarePort
from myarm_m750_core.runtime import (
    DriverState,
    DriverStateMachine,
    PointToPointTrajectoryGenerator,
    TrajectoryExecutor,
)
from myarm_m750_core.domain.safety import MotionGuard

_LOGGER = logging.getLogger(__name__)


class RobotSession:
    """High-level SDK session with explicit connect/disconnect ownership."""

    def __init__(self, config: SdkConfig, controller: RobotController) -> None:
        self._config = config
        self._controller = controller
        self._connected = False
        self._lifecycle_lock = threading.RLock()

    @classmethod
    def from_config(cls, config_path: str) -> "RobotSession":
        """Build one complete SDK graph from YAML.

        Args:
            config_path: Manifest or robot YAML path.

        Returns:
            Disconnected session. Enter it as a context manager or call
            ``connect()`` before reading state or moving.

        Side effects:
            Configures process logging. Hardware is not opened until connect.
        """
        config = load_sdk_config(config_path)
        configure_logging(config.logging)
        kinematics = PoeKinematics.from_urdf(
            urdf_path=config.robot.urdf_path,
            base_link=config.robot.base_link,
            end_link=config.robot.end_link,
            joint_names=config.robot.joint_names,
        )
        mapper = JointMapper(
            joint_names=config.robot.joint_names,
            mapping=config.robot.joint_mapping,
        )
        hardware = cls._create_hardware(config, mapper)
        state_machine = DriverStateMachine()
        motion_guard = MotionGuard(
            joint_names=config.robot.joint_names,
            kinematics=kinematics,
            config=config.safety,
        )
        trajectory_generator = PointToPointTrajectoryGenerator(
            command_rate_hz=config.robot.runtime.command_rate_hz
        )
        executor = TrajectoryExecutor(
            hardware=hardware,
            motion_guard=motion_guard,
            state_machine=state_machine,
            realtime_execution=config.robot.runtime.realtime_execution,
        )
        controller = RobotController(
            joint_names=config.robot.joint_names,
            hardware=hardware,
            kinematics=kinematics,
            trajectory_generator=trajectory_generator,
            trajectory_executor=executor,
            state_machine=state_machine,
        )
        return cls(config=config, controller=controller)

    @staticmethod
    def _create_hardware(
        config: SdkConfig, mapper: JointMapper
    ) -> RobotHardwarePort:
        adapter_type = config.adapter.adapter_type
        options = config.adapter.options
        if adapter_type == "mock":
            initial = options.get("initial_position_rad", [0.0] * 6)
            if not isinstance(initial, list):
                raise ConfigurationError(
                    "mock.initial_position_rad must be a six-value list."
                )
            return MockRobotAdapter(initial_position_rad=initial)
        if adapter_type == "replay":
            replay_file = str(options.get("replay_file", ""))
            if replay_file and not Path(replay_file).is_absolute():
                replay_file = str(
                    (config.source_path.parent / replay_file).resolve()
                )
            return ReplayRobotAdapter(
                replay_file=replay_file,
                loop=bool(options.get("loop", False)),
            )
        if adapter_type == "vendor_serial":
            return VendorSerialRobotAdapter(
                port=str(options.get("port", "/dev/ttyUSB0")),
                baudrate=int(options.get("baudrate", 1_000_000)),
                timeout_s=float(options.get("timeout_s", 0.1)),
                firmware_speed=int(options.get("firmware_speed", 30)),
                mapper=mapper,
                max_retries=int(options.get("max_retries", 1)),
                retry_delay_s=float(options.get("retry_delay_s", 0.05)),
                debug=bool(options.get("debug", False)),
            )
        raise ConfigurationError(
            "Unsupported adapter type '{0}'. Expected mock, replay, or vendor_serial.".format(
                adapter_type
            )
        )

    @property
    def state(self) -> DriverState:
        """Return the explicit runtime state."""
        return self._controller.state

    @property
    def joint_names(self) -> Sequence[str]:
        """Return canonical joint names."""
        return self._controller.joint_names

    @property
    def config(self) -> SdkConfig:
        """Return immutable resolved configuration."""
        return self._config

    def connect(self) -> None:
        """Open the configured robot adapter exactly once."""
        with self._lifecycle_lock:
            if not self._connected:
                self._controller.connect()
                self._connected = True
                _LOGGER.info("robot_session_connected")

    def close(self) -> None:
        """Stop ownership and release adapter resources exactly once."""
        with self._lifecycle_lock:
            if self._connected:
                self._controller.disconnect()
                self._connected = False
                _LOGGER.info("robot_session_closed")

    def __enter__(self) -> "RobotSession":
        self.connect()
        return self

    def __exit__(self, exception_type: object, exception: object, traceback: object) -> None:
        del exception_type, exception, traceback
        self.close()

    def get_state(self) -> JointState:
        """Read measured canonical joint state."""
        return self._controller.get_state()

    def get_hardware_status(self) -> HardwareStatus:
        """Return adapter diagnostics without exposing vendor internals."""
        return self._controller.get_hardware_status()

    def get_capabilities(self) -> RobotCapabilities:
        """Return operations explicitly supported by the active adapter."""
        return self._controller.get_capabilities()

    def compute_fk(self, joint_position_rad: Sequence[float]) -> RigidTransform:
        """Compute software FK from canonical joints."""
        return self._controller.compute_fk(joint_position_rad)

    def compute_jacobian(self, joint_position_rad: Sequence[float]) -> np.ndarray:
        """Compute the 6x6 geometric Jacobian."""
        return self._controller.compute_jacobian(joint_position_rad)

    def move_joints(
        self,
        target: Sequence[float],
        duration_s: float,
        cancel_requested: Optional[Callable[[], bool]] = None,
    ) -> CommandResult:
        """Move to a canonical joint target through validated trajectory points."""
        return self._controller.move_joints(
            target_position_rad=target,
            duration_s=duration_s,
            cancel_requested=cancel_requested,
        )

    def execute_trajectory(
        self,
        trajectory: JointTrajectory,
        cancel_requested: Optional[Callable[[], bool]] = None,
    ) -> CommandResult:
        """Execute a standard canonical joint trajectory."""
        return self._controller.execute_trajectory(trajectory, cancel_requested)

    def move_pose(
        self,
        target: RigidTransform,
        duration_s: float,
        seed_joint_position_rad: Optional[Sequence[float]] = None,
        cancel_requested: Optional[Callable[[], bool]] = None,
    ) -> CommandResult:
        """Solve software IK and execute the joint result."""
        return self._controller.move_pose(
            target_pose=target,
            duration_s=duration_s,
            seed_joint_position_rad=seed_joint_position_rad,
            cancel_requested=cancel_requested,
        )

    def stop(self) -> CommandResult:
        """Stop motion through the active hardware adapter."""
        return self._controller.stop()

    def pause(self) -> CommandResult:
        """Pause motion when supported."""
        return self._controller.pause()

    def resume(self) -> CommandResult:
        """Resume motion when supported."""
        return self._controller.resume()

    def recover(self) -> CommandResult:
        """Attempt a bounded transition from FAULT to IDLE."""
        return self._controller.recover()
