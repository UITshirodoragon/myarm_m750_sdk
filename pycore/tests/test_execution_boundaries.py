import threading
import time
from types import SimpleNamespace
from typing import Callable, Optional

import pytest
from myarm_m750_core.adapters.mock_robot import MockRobotAdapter
from myarm_m750_core.domain.models import (
    AdmittedTrajectory,
    CommandContext,
    CommandResult,
    CommandStatus,
    JointState,
    JointTarget,
    JointTrajectory,
    JointTrajectoryPoint,
)
from myarm_m750_core.runtime.admission import AdmissionResult, CommandAdmission
from myarm_m750_core.runtime.executor import TrajectoryExecutor
from myarm_m750_core.runtime.scheduler import Scheduler, VirtualScheduler
from myarm_m750_core.runtime.state_machine import (
    DriverEvent,
    DriverState,
    DriverStateMachine,
)


class _ControlledRobot(MockRobotAdapter):
    def __init__(
        self,
        *,
        write_result: Optional[CommandResult] = None,
        write_error: Optional[Exception] = None,
        stop_result: Optional[CommandResult] = None,
        stop_error: Optional[Exception] = None,
        read_state: Optional[JointState] = None,
    ) -> None:
        super().__init__((0.0,) * 6)
        self._write_result = write_result
        self._write_error = write_error
        self._stop_result = stop_result
        self._stop_error = stop_error
        self._read_state = read_state
        self.read_count = 0
        self.stop_count = 0

    def read_joint_state(self, context: CommandContext) -> JointState:
        self.read_count += 1
        if self._read_state is not None:
            return self._read_state
        return super().read_joint_state(context)

    def write_joint_target(
        self, target: JointTarget, context: CommandContext
    ) -> CommandResult:
        if self._write_error is not None:
            raise self._write_error
        if self._write_result is not None:
            return self._write_result
        return super().write_joint_target(target, context)

    def stop(self, context: CommandContext) -> CommandResult:
        self.stop_count += 1
        if self._stop_error is not None:
            raise self._stop_error
        if self._stop_result is not None:
            return self._stop_result
        return super().stop(context)


class _OverrunScheduler(Scheduler):
    def __init__(self) -> None:
        self.current_s = 0.0

    def now(self) -> float:
        return self.current_s

    def wait_until(
        self,
        deadline_monotonic_s: float,
        cancellation_requested: Callable[[], bool],
    ) -> float:
        if not cancellation_requested():
            self.current_s = max(self.current_s, deadline_monotonic_s) + 2.0
        return self.current_s - deadline_monotonic_s


class _CancellationScheduler(Scheduler):
    def __init__(self) -> None:
        self.cancel_hook = lambda: None  # type: Callable[[], None]

    def now(self) -> float:
        return 0.0

    def wait_until(
        self,
        deadline_monotonic_s: float,
        cancellation_requested: Callable[[], bool],
    ) -> float:
        del cancellation_requested
        self.cancel_hook()
        return -deadline_monotonic_s


def _idle_state_machine() -> DriverStateMachine:
    machine = DriverStateMachine()
    machine.apply(DriverEvent.CONNECT_REQUESTED, reason="test connect")
    machine.apply(DriverEvent.CONNECT_SUCCEEDED, reason="test ready")
    return machine


def _trajectory(joint_names) -> JointTrajectory:
    return JointTrajectory(
        joint_names=tuple(joint_names),
        points=(
            JointTrajectoryPoint(
                position_rad=(0.0,) * 6,
                velocity_rad_s=(0.0,) * 6,
                acceleration_rad_s2=(0.0,) * 6,
                time_from_start_s=0.0,
            ),
            JointTrajectoryPoint(
                position_rad=(0.0,) * 6,
                velocity_rad_s=(0.0,) * 6,
                acceleration_rad_s2=(0.0,) * 6,
                time_from_start_s=1.0,
            ),
        ),
    )


def _executor(
    trajectory_validator,
    *,
    hardware: Optional[_ControlledRobot] = None,
    scheduler: Optional[Scheduler] = None,
):
    selected_hardware = hardware or _ControlledRobot()
    selected_scheduler = scheduler or VirtualScheduler()
    state_machine = _idle_state_machine()
    selected_hardware.connect()
    admission = CommandAdmission(
        trajectory_validator,
        lambda: state_machine.state,
        selected_scheduler,
    )
    executor = TrajectoryExecutor(
        hardware=selected_hardware,
        admission=admission,
        state_machine=state_machine,
        scheduler=selected_scheduler,
        command_timeout_s=1.0,
        stop_timeout_s=1.0,
        operation_clock=lambda: 1.0,
    )
    return executor, state_machine


def test_admission_result_requires_exactly_one_outcome(
    sdk_config,
) -> None:
    state = JointState(position_rad=(0.0,) * 6)
    trajectory = _trajectory(sdk_config.robot.joint_names)
    admitted = AdmittedTrajectory(
        command_id="command",
        trajectory=trajectory,
        initial_state=state,
        admitted_monotonic_s=0.0,
        model_fingerprint="model",
        limit_provenance="test",
    )
    rejection = CommandResult.rejected("rejected", "TEST")

    with pytest.raises(ValueError, match="exactly one"):
        AdmissionResult()
    with pytest.raises(ValueError, match="exactly one"):
        AdmissionResult(admitted=admitted, rejection=rejection)


def test_admission_rejects_non_idle_and_invalid_trajectory(
    sdk_config, trajectory_validator
) -> None:
    trajectory = _trajectory(sdk_config.robot.joint_names)
    state = JointState(position_rad=(0.0,) * 6)
    non_idle = CommandAdmission(
        trajectory_validator,
        lambda: DriverState.DISCONNECTED,
        VirtualScheduler(),
    )
    assert non_idle.admit("not-idle", trajectory, state).rejection.error_code == (
        "DRIVER_NOT_IDLE"
    )

    invalid = JointTrajectory(
        joint_names=trajectory.joint_names,
        points=(
            JointTrajectoryPoint(
                position_rad=(0.2, 0.0, 0.0, 0.0, 0.0, 0.0),
                time_from_start_s=1.0,
            ),
        ),
    )
    idle = CommandAdmission(
        trajectory_validator,
        lambda: DriverState.IDLE,
        VirtualScheduler(),
    )
    rejection = idle.admit("unsafe", invalid, state).rejection
    assert rejection is not None
    assert rejection.error_code == "SAFETY_VALIDATION_FAILED"


def test_executor_no_active_cancel_stop_and_busy_boundaries(
    sdk_config, trajectory_validator
) -> None:
    executor, _ = _executor(trajectory_validator)
    no_active = executor.cancel_current_command()
    assert no_active.status is CommandStatus.REJECTED
    assert no_active.error_code == "NO_ACTIVE_COMMAND"
    assert executor.stop_now().succeeded

    assert executor._execution_lock.acquire(blocking=False)
    try:
        busy = executor.execute(
            _trajectory(sdk_config.robot.joint_names),
            JointState(position_rad=(0.0,) * 6),
        )
    finally:
        executor._execution_lock.release()
    assert busy.status is CommandStatus.REJECTED
    assert busy.error_code == "COMMAND_BUSY"


def test_executor_counts_stale_admission_rejection(
    sdk_config, trajectory_validator
) -> None:
    executor, _ = _executor(trajectory_validator)
    stale = JointState(
        position_rad=(0.0,) * 6,
        received_monotonic_s=time.monotonic() - 10.0,
    )
    result = executor.execute(_trajectory(sdk_config.robot.joint_names), stale)
    assert result.status is CommandStatus.REJECTED
    assert result.error_code == "SAFETY_VALIDATION_FAILED"
    assert executor.metrics_snapshot().stale_state_count == 1


def test_executor_progress_feedback_and_overrun_metrics(
    sdk_config, trajectory_validator
) -> None:
    scheduler = _OverrunScheduler()
    executor, state_machine = _executor(trajectory_validator, scheduler=scheduler)
    feedback = []
    result = executor.execute(
        _trajectory(sdk_config.robot.joint_names),
        JointState(position_rad=(0.0,) * 6),
        lambda index, point, actual: feedback.append(
            (index, point.time_from_start_s, actual.sequence)
        ),
    )

    assert result.succeeded
    assert state_machine.state is DriverState.IDLE
    assert len(feedback) == 2
    metrics = executor.metrics_snapshot()
    assert metrics.waypoint_count == 2
    assert metrics.overrun_count == 1


def test_executor_always_reads_state_and_faults_on_stale_watchdog(
    sdk_config, trajectory_validator
) -> None:
    hardware = _ControlledRobot(
        read_state=JointState(
            position_rad=(0.0,) * 6,
            received_monotonic_s=-10.0,
        )
    )
    executor, state_machine = _executor(trajectory_validator, hardware=hardware)

    result = executor.execute(
        _trajectory(sdk_config.robot.joint_names),
        JointState(position_rad=(0.0,) * 6),
    )

    assert result.status is CommandStatus.FAILED
    assert result.error_code == "STALE_STATE_WATCHDOG"
    assert hardware.read_count == 1
    assert hardware.stop_count == 1
    assert executor.metrics_snapshot().stale_state_count == 1
    assert state_machine.state is DriverState.FAULT


def test_executor_transitions_to_fault_on_write_rejection(
    sdk_config, trajectory_validator
) -> None:
    hardware = _ControlledRobot(
        write_result=CommandResult.failed("write rejected", "WRITE_REJECTED")
    )
    executor, state_machine = _executor(trajectory_validator, hardware=hardware)
    result = executor.execute(
        _trajectory(sdk_config.robot.joint_names),
        JointState(position_rad=(0.0,) * 6),
    )

    assert result.status is CommandStatus.FAILED
    assert result.error_code == "WRITE_REJECTED"
    assert state_machine.state is DriverState.FAULT


def test_executor_contains_write_and_stop_exceptions(
    sdk_config, trajectory_validator
) -> None:
    hardware = _ControlledRobot(
        write_error=RuntimeError("write exploded"),
        stop_error=RuntimeError("stop exploded"),
    )
    executor, state_machine = _executor(trajectory_validator, hardware=hardware)
    result = executor.execute(
        _trajectory(sdk_config.robot.joint_names),
        JointState(position_rad=(0.0,) * 6),
    )

    assert result.status is CommandStatus.FAILED
    assert result.error_code == "RuntimeError+STOP_EXCEPTION"
    assert "write exploded" in result.message
    assert "stop exploded" in result.message
    assert state_machine.state is DriverState.FAULT


def test_executor_contains_invalid_admission_result(
    sdk_config, trajectory_validator
) -> None:
    executor, state_machine = _executor(trajectory_validator)
    executor._admission = SimpleNamespace(
        admit=lambda *_args: SimpleNamespace(rejection=None, admitted=None)
    )
    result = executor.execute(
        _trajectory(sdk_config.robot.joint_names),
        JointState(position_rad=(0.0,) * 6),
    )

    assert result.status is CommandStatus.FAILED
    assert result.error_code == "RuntimeError"
    assert state_machine.state is DriverState.FAULT


def test_executor_surfaces_failed_stop_during_cancellation(
    sdk_config, trajectory_validator
) -> None:
    scheduler = _CancellationScheduler()
    hardware = _ControlledRobot(
        stop_result=CommandResult.failed("stop rejected", "STOP_REJECTED")
    )
    executor, state_machine = _executor(
        trajectory_validator,
        hardware=hardware,
        scheduler=scheduler,
    )

    def accept_cancellation_generation() -> None:
        executor._cancellation_generation += 1

    scheduler.cancel_hook = accept_cancellation_generation
    result = executor.execute(
        _trajectory(sdk_config.robot.joint_names),
        JointState(position_rad=(0.0,) * 6),
    )

    assert result.status is CommandStatus.FAILED
    assert result.error_code == "STOP_REJECTED"
    assert state_machine.state is DriverState.FAULT


@pytest.mark.parametrize("stop_status", [CommandStatus.FAILED, CommandStatus.REJECTED])
def test_executor_replays_failed_stop_outcome_without_second_stop(
    sdk_config, trajectory_validator, stop_status: CommandStatus
) -> None:
    scheduler = _CancellationScheduler()
    if stop_status is CommandStatus.FAILED:
        stop_result = CommandResult.failed("stop failed", "STOP_FAILED")
    else:
        stop_result = CommandResult.rejected("stop rejected", "STOP_REJECTED")
    hardware = _ControlledRobot(stop_result=stop_result)
    executor, state_machine = _executor(
        trajectory_validator,
        hardware=hardware,
        scheduler=scheduler,
    )
    cancellation_results = []

    def cancel_twice() -> None:
        cancellation_results.append(executor.cancel_current_command())
        cancellation_results.append(executor.cancel_current_command())

    scheduler.cancel_hook = cancel_twice
    result = executor.execute(
        _trajectory(sdk_config.robot.joint_names),
        JointState(position_rad=(0.0,) * 6),
    )

    assert cancellation_results[0] is stop_result
    assert cancellation_results[1] is stop_result
    assert hardware.stop_count == 1
    assert result.status is CommandStatus.FAILED
    assert result.error_code == stop_result.error_code
    assert state_machine.state is DriverState.FAULT


def test_last_waypoint_cancel_and_success_have_one_linearized_outcome(
    sdk_config,
    trajectory_validator,
) -> None:
    hardware = _ControlledRobot()
    executor, state_machine = _executor(
        trajectory_validator,
        hardware=hardware,
    )
    final_feedback_entered = threading.Event()
    release_feedback = threading.Event()
    execution_results = []

    def block_final_feedback(
        point_index: int,
        _point: JointTrajectoryPoint,
        _actual: JointState,
    ) -> None:
        if point_index == 1:
            final_feedback_entered.set()
            assert release_feedback.wait(timeout=1.0)

    worker = threading.Thread(
        target=lambda: execution_results.append(
            executor.execute(
                _trajectory(sdk_config.robot.joint_names),
                JointState(position_rad=(0.0,) * 6),
                block_final_feedback,
            )
        )
    )
    worker.start()
    assert final_feedback_entered.wait(timeout=1.0)

    cancellation = executor.cancel_current_command()
    release_feedback.set()
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert cancellation.succeeded
    assert execution_results[0].status is CommandStatus.CANCELED
    assert hardware.stop_count == 1
    assert state_machine.state is DriverState.IDLE


def test_primary_execution_and_bounded_stop_failures_are_both_reported(
    sdk_config,
    trajectory_validator,
) -> None:
    hardware = _ControlledRobot(
        write_result=CommandResult.failed("write rejected", "WRITE_REJECTED"),
        stop_result=CommandResult.failed("stop rejected", "STOP_REJECTED"),
    )
    executor, state_machine = _executor(
        trajectory_validator,
        hardware=hardware,
    )

    result = executor.execute(
        _trajectory(sdk_config.robot.joint_names),
        JointState(position_rad=(0.0,) * 6),
    )

    assert result.status is CommandStatus.FAILED
    assert result.error_code == "WRITE_REJECTED+STOP_REJECTED"
    assert "Hardware rejected point 0" in result.message
    assert "Bounded stop also failed (STOP_REJECTED)" in result.message
    assert state_machine.state is DriverState.FAULT
    assert state_machine.history[-1].reason == result.message
