from typing import Optional

import pytest
from myarm_m750_core.adapters import MockRobotAdapter
from myarm_m750_core.api.session import RobotSession
from myarm_m750_core.application.robot_controller import RobotController
from myarm_m750_core.domain.errors import HardwareStopError, InvalidDriverStateError
from myarm_m750_core.domain.models import CommandResult
from myarm_m750_core.runtime.state_machine import (
    DriverEvent,
    DriverState,
    DriverStateMachine,
)


class _TrackingAdapter(MockRobotAdapter):
    def __init__(self, disconnect_error: Optional[Exception] = None) -> None:
        super().__init__((0.0,) * 6)
        self.disconnect_count = 0
        self._disconnect_error = disconnect_error

    def disconnect(self) -> None:
        self.disconnect_count += 1
        if self._disconnect_error is not None:
            raise self._disconnect_error
        super().disconnect()


class _StopExecutor:
    def __init__(
        self,
        result: Optional[CommandResult] = None,
        error: Optional[Exception] = None,
    ) -> None:
        self.result = result
        self.error = error
        self.stop_count = 0

    def stop_now(self) -> CommandResult:
        self.stop_count += 1
        if self.error is not None:
            raise self.error
        if self.result is None:
            raise AssertionError("Test executor requires a stop result.")
        return self.result


def _controller(
    adapter: _TrackingAdapter,
    executor: _StopExecutor,
    state_machine: DriverStateMachine,
) -> RobotController:
    return RobotController(
        joint_names=tuple(f"joint_{index}" for index in range(6)),
        hardware=adapter,
        kinematics=object(),
        trajectory_generator=object(),
        trajectory_executor=executor,
        state_machine=state_machine,
        read_timeout_s=1.0,
    )


def _enter_execution(
    controller: RobotController,
    state_machine: DriverStateMachine,
) -> None:
    controller.connect()
    state_machine.apply(
        DriverEvent.EXECUTION_ACCEPTED,
        reason="shutdown regression test",
        command_id="active-command",
    )


def test_controller_surfaces_stop_failure_after_disconnect_cleanup() -> None:
    adapter = _TrackingAdapter()
    executor = _StopExecutor(
        result=CommandResult.failed(
            "firmware rejected stop",
            "STOP_REJECTED",
            command_id="active-command",
        )
    )
    state_machine = DriverStateMachine()
    controller = _controller(adapter, executor, state_machine)
    _enter_execution(controller, state_machine)

    with pytest.raises(HardwareStopError) as captured:
        controller.disconnect()

    assert captured.value.command_id == "active-command"
    assert captured.value.error_code == "STOP_REJECTED"
    assert executor.stop_count == 1
    assert adapter.disconnect_count == 1
    assert not adapter.read_hardware_status().connected
    assert controller.state is DriverState.DISCONNECTED


def test_session_clears_connected_state_after_surfaced_stop_failure(
    sdk_config,
) -> None:
    adapter = _TrackingAdapter()
    executor = _StopExecutor(
        result=CommandResult.rejected(
            "stop capability is unverified",
            "CAPABILITY_NOT_VERIFIED",
            command_id="active-command",
        )
    )
    state_machine = DriverStateMachine()
    controller = _controller(adapter, executor, state_machine)
    session = RobotSession(sdk_config, controller)
    session.connect()
    state_machine.apply(
        DriverEvent.EXECUTION_ACCEPTED,
        reason="shutdown regression test",
        command_id="active-command",
    )

    with pytest.raises(HardwareStopError, match="unverified"):
        session.close()

    assert session.state is DriverState.DISCONNECTED
    assert not adapter.read_hardware_status().connected
    with pytest.raises(InvalidDriverStateError, match="connected session"):
        session.probe_hardware()
    session.close()
    assert adapter.disconnect_count == 1


def test_stop_exception_is_typed_and_cleanup_is_still_attempted() -> None:
    adapter = _TrackingAdapter()
    executor = _StopExecutor(error=RuntimeError("stop transport failed"))
    state_machine = DriverStateMachine()
    controller = _controller(adapter, executor, state_machine)
    _enter_execution(controller, state_machine)

    with pytest.raises(HardwareStopError, match="RuntimeError") as captured:
        controller.disconnect()

    assert captured.value.error_code == "STOP_EXCEPTION"
    assert adapter.disconnect_count == 1
    assert controller.state is DriverState.DISCONNECTED


def test_combined_stop_and_disconnect_failure_preserves_connected_state() -> None:
    adapter = _TrackingAdapter(disconnect_error=OSError("close failed"))
    executor = _StopExecutor(
        result=CommandResult.failed(
            "stop failed",
            "STOP_FAILED",
            command_id="active-command",
        )
    )
    state_machine = DriverStateMachine()
    controller = _controller(adapter, executor, state_machine)
    _enter_execution(controller, state_machine)

    with pytest.raises(HardwareStopError, match="disconnect also failed") as captured:
        controller.disconnect()

    assert isinstance(captured.value.__cause__, OSError)
    assert adapter.disconnect_count == 1
    assert adapter.read_hardware_status().connected
    assert controller.state is DriverState.EXECUTING
