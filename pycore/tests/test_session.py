import threading
import time
from dataclasses import replace

import numpy as np
import pytest
from myarm_m750_core import MotionProfile, RobotSessionBuilder
from myarm_m750_core.adapters import MockRobotAdapter
from myarm_m750_core.domain.errors import (
    HardwareConnectionError,
    InvalidDriverStateError,
    ProtocolError,
)
from myarm_m750_core.domain.models import (
    AdapterCapabilities,
    CapabilityState,
    CommandContext,
    CommandResult,
    CommandStatus,
    HardwareIdentity,
    JointTarget,
)
from myarm_m750_core.runtime import DriverState, VirtualScheduler
from myarm_m750_core.runtime.config.models import AdapterConfig


def _config(repository_root):
    return repository_root / "pycore/config/default.yaml"


def test_public_builder_and_session_run_without_ros(repository_root) -> None:
    builder = RobotSessionBuilder.from_file(str(_config(repository_root)))
    inspection = builder.inspect_environment()
    assert inspection.ready
    robot = builder.with_scheduler(VirtualScheduler()).build()
    with robot:
        assert robot.state is DriverState.IDLE
        result = robot.move_joints(
            [0.05, -0.05, 0.04, 0.03, -0.03, 0.04],
            MotionProfile(duration_s=1.0),
        )
        assert result.succeeded, result.message
        np.testing.assert_allclose(
            robot.read_joint_state().position_rad,
            [0.05, -0.05, 0.04, 0.03, -0.03, 0.04],
            atol=1.0e-12,
        )
    assert robot.state is DriverState.DISCONNECTED


def test_status_capabilities_and_adapter_kind(repository_root) -> None:
    with RobotSessionBuilder.from_file(str(_config(repository_root))).build() as robot:
        assert robot.adapter_kind == "mock"
        assert robot.read_hardware_status().connected
        assert robot.adapter_capabilities().stop is CapabilityState.SUPPORTED
        identity = robot.probe_hardware()
        assert identity.adapter == "mock"
        assert identity.model == "myarm_m750_mock"


def test_cancel_generation_prevents_later_waypoints(repository_root) -> None:
    robot = RobotSessionBuilder.from_file(str(_config(repository_root))).build()
    result_holder = []
    with robot:
        worker = threading.Thread(
            target=lambda: result_holder.append(
                robot.move_joints(
                    [0.05, 0.0, 0.0, 0.0, 0.0, 0.0],
                    MotionProfile(duration_s=1.0),
                )
            )
        )
        worker.start()
        deadline_s = time.monotonic() + 2.0
        while (
            robot.state is not DriverState.EXECUTING
            and time.monotonic() < deadline_s
        ):
            time.sleep(0.005)
        assert robot.state is DriverState.EXECUTING
        stop_result = robot.cancel_current_command()
        worker.join(timeout=1.0)
        assert stop_result.succeeded
        assert not worker.is_alive()
        assert result_holder[0].status.value == "canceled"
        assert robot.state is DriverState.IDLE


def test_default_5_hz_fake_backend_meets_deadline_budget(repository_root) -> None:
    with RobotSessionBuilder.from_file(str(_config(repository_root))).build() as robot:
        result = robot.move_joints(
            [0.05, -0.04, 0.03, 0.02, -0.01, 0.01],
            MotionProfile(duration_s=1.0),
        )
        assert result.succeeded
        metrics = robot.metrics_snapshot()
        assert metrics.waypoint_count == 6
        assert metrics.overrun_count == 0
        assert np.percentile(metrics.operation_latency_s, 99) < 0.16
        assert np.percentile(np.abs(metrics.scheduler_jitter_s), 99) < 0.02


def test_real_session_connect_probes_once_before_exposing_commands(
    sdk_config,
) -> None:
    class ProbeAdapter(MockRobotAdapter):
        def __init__(self):
            super().__init__([0.0] * 6)
            self.probe_count = 0

        def probe_identity(self, context: CommandContext) -> HardwareIdentity:
            self.probe_count += 1
            return super().probe_identity(context)

    adapter = ProbeAdapter()
    real_intent = replace(
        sdk_config,
        adapter=AdapterConfig(adapter_type="vendor_serial"),
    )
    robot = (
        RobotSessionBuilder(real_intent)
        .with_adapter_factory(lambda _config, _mapper: adapter)
        .build()
    )
    with robot:
        assert adapter.probe_count == 1
        assert robot.probe_hardware().adapter == "mock"
        assert adapter.probe_count == 1
    assert not adapter.read_hardware_status().connected


def test_real_session_probe_failure_closes_adapter(sdk_config) -> None:
    class RejectingProbeAdapter(MockRobotAdapter):
        def probe_identity(self, context: CommandContext) -> HardwareIdentity:
            del context
            raise ProtocolError("identity mismatch")

    adapter = RejectingProbeAdapter([0.0] * 6)
    real_intent = replace(
        sdk_config,
        adapter=AdapterConfig(adapter_type="vendor_serial"),
    )
    robot = (
        RobotSessionBuilder(real_intent)
        .with_adapter_factory(lambda _config, _mapper: adapter)
        .build()
    )
    with pytest.raises(ProtocolError, match="identity mismatch"):
        robot.connect()
    assert robot.state is DriverState.DISCONNECTED
    assert not adapter.read_hardware_status().connected


def test_real_connect_probe_linearizes_before_concurrent_motion(
    sdk_config,
) -> None:
    class BlockingProbeAdapter(MockRobotAdapter):
        def __init__(self) -> None:
            super().__init__([0.0] * 6)
            self.probe_entered = threading.Event()
            self.release_probe = threading.Event()
            self.write_count = 0

        def probe_identity(self, context: CommandContext) -> HardwareIdentity:
            self.probe_entered.set()
            if not self.release_probe.wait(timeout=1.0):
                raise AssertionError("probe release timed out")
            return super().probe_identity(context)

        def write_joint_target(
            self,
            target: JointTarget,
            context: CommandContext,
        ) -> CommandResult:
            self.write_count += 1
            return super().write_joint_target(target, context)

    adapter = BlockingProbeAdapter()
    real_intent = replace(
        sdk_config,
        adapter=AdapterConfig(adapter_type="vendor_serial"),
    )
    robot = (
        RobotSessionBuilder(real_intent)
        .with_adapter_factory(lambda _config, _mapper: adapter)
        .with_scheduler(VirtualScheduler())
        .build()
    )
    connect_errors = []
    motion_results = []
    motion_errors = []
    motion_attempted = threading.Event()
    connector = threading.Thread(
        target=lambda: _capture_call(robot.connect, connect_errors)
    )
    connector.start()
    assert adapter.probe_entered.wait(timeout=1.0)

    def attempt_motion() -> None:
        motion_attempted.set()
        _capture_result(
            lambda: robot.move_joints(
                [0.05, 0.0, 0.0, 0.0, 0.0, 0.0],
                MotionProfile(duration_s=1.0),
            ),
            motion_results,
            motion_errors,
        )

    mover = threading.Thread(
        target=attempt_motion,
    )
    mover.start()
    assert motion_attempted.wait(timeout=1.0)
    time.sleep(0.02)
    assert mover.is_alive()
    assert adapter.write_count == 0

    adapter.release_probe.set()
    connector.join(timeout=1.0)
    mover.join(timeout=1.0)
    assert not connector.is_alive()
    assert not mover.is_alive()
    assert connect_errors == []
    assert motion_errors == []
    assert motion_results[0].succeeded
    assert adapter.write_count > 0
    robot.close()


@pytest.mark.parametrize(
    "stop_state",
    [CapabilityState.UNVERIFIED, CapabilityState.UNSUPPORTED],
)
def test_real_motion_requires_stop_verified_after_probe(
    sdk_config,
    stop_state: CapabilityState,
) -> None:
    class StopCapabilityAdapter(MockRobotAdapter):
        def __init__(self) -> None:
            super().__init__([0.0] * 6)
            self.read_count = 0
            self.write_count = 0

        def capabilities(self) -> AdapterCapabilities:
            return AdapterCapabilities(stop=stop_state)

        def read_joint_state(self, context: CommandContext):
            self.read_count += 1
            return super().read_joint_state(context)

        def write_joint_target(
            self,
            target: JointTarget,
            context: CommandContext,
        ) -> CommandResult:
            self.write_count += 1
            return super().write_joint_target(target, context)

    adapter = StopCapabilityAdapter()
    real_intent = replace(
        sdk_config,
        adapter=AdapterConfig(adapter_type="vendor_serial"),
    )
    robot = (
        RobotSessionBuilder(real_intent)
        .with_adapter_factory(lambda _config, _mapper: adapter)
        .with_scheduler(VirtualScheduler())
        .build()
    )
    with robot:
        probe_read_count = adapter.read_count
        result = robot.move_joints(
            [0.05, 0.0, 0.0, 0.0, 0.0, 0.0],
            MotionProfile(duration_s=1.0),
        )
        assert result.status is CommandStatus.REJECTED
        assert result.error_code == "STOP_CAPABILITY_NOT_VERIFIED"
        assert adapter.read_count == probe_read_count
        assert adapter.write_count == 0


def test_failed_connect_returns_to_disconnected_and_can_retry(
    sdk_config,
) -> None:
    class FlakyConnectAdapter(MockRobotAdapter):
        def __init__(self) -> None:
            super().__init__([0.0] * 6)
            self.connect_count = 0

        def connect(self) -> None:
            self.connect_count += 1
            if self.connect_count == 1:
                raise HardwareConnectionError("transient open failure")
            super().connect()

    adapter = FlakyConnectAdapter()
    robot = (
        RobotSessionBuilder(sdk_config)
        .with_adapter_factory(lambda _config, _mapper: adapter)
        .build()
    )

    with pytest.raises(HardwareConnectionError, match="transient"):
        robot.connect()
    assert robot.state is DriverState.DISCONNECTED
    with pytest.raises(InvalidDriverStateError, match="connected, ready"):
        robot.read_joint_state()

    robot.connect()
    assert robot.state is DriverState.IDLE
    assert robot.read_joint_state().source == "mock"
    robot.close()


def _capture_call(call, errors) -> None:
    try:
        call()
    except Exception as error:
        errors.append(error)


def _capture_result(call, results, errors) -> None:
    try:
        results.append(call())
    except Exception as error:
        errors.append(error)
