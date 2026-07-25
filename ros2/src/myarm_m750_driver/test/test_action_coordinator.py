"""Action terminal-state, feedback, and tolerance regression tests."""

import threading
import time
import unittest

from builtin_interfaces.msg import Time
from control_msgs.action import FollowJointTrajectory
from myarm_m750_driver.action_coordinator import TrajectoryActionCoordinator
from myarm_m750_driver.contracts import (
    CoreCommandOutcome,
    CoreJointSample,
    DriverLifecycleState,
    TrajectoryErrorCode,
)
from rclpy.action import GoalResponse
from trajectory_msgs.msg import JointTrajectoryPoint

_JOINT_NAMES = ("j1", "j2", "j3", "j4", "j5", "j6")


class _Logger:
    def warning(self, message):
        del message

    def error(self, message):
        del message

    def exception(self, message):
        del message


class _Now:
    nanoseconds = int(time.time() * 1_000_000_000)

    @staticmethod
    def to_msg():
        message = Time()
        message.sec = _Now.nanoseconds // 1_000_000_000
        message.nanosec = _Now.nanoseconds % 1_000_000_000
        return message


class _Clock:
    @staticmethod
    def now():
        return _Now()


class _Node:
    @staticmethod
    def get_logger():
        return _Logger()

    @staticmethod
    def get_clock():
        return _Clock()


class _Facade:
    joint_names = _JOINT_NAMES

    def __init__(self, actual_position_rad=(0.0,) * 6):
        self.actual_position_rad = actual_position_rad
        self.cancel_count = 0
        self.stop_count = 0

    def execute_trajectory(self, trajectory, progress_callback=None):
        desired = trajectory.points[-1]
        actual = self.read_joint_state()
        if progress_callback is not None:
            progress_callback(len(trajectory.points) - 1, desired, actual)
        return CoreCommandOutcome(
            status="succeeded",
            message="executed",
            command_id="command-1",
            error_code=None,
        )

    def read_joint_state(self):
        return CoreJointSample(
            position_rad=self.actual_position_rad,
            sample_wall_time_s=time.time(),
            received_monotonic_s=time.monotonic(),
            source="fake",
            sequence=1,
        )

    def cancel_current_command(self):
        self.cancel_count += 1
        return CoreCommandOutcome(
            status="canceled",
            message="canceled",
            command_id="command-1",
            error_code="COMMAND_CANCELED",
        )

    def stop(self):
        self.stop_count += 1
        return CoreCommandOutcome(
            status="succeeded",
            message="stopped",
            command_id="command-1",
            error_code=None,
        )


class _Lifecycle:
    state = DriverLifecycleState.ACTIVE

    def __init__(self, facade):
        self.facade = facade
        self.fault = None

    def record_runtime_fault(self, error):
        self.fault = error
        self.state = DriverLifecycleState.FAULT


class _GoalHandle:
    is_cancel_requested = False

    def __init__(self):
        self.feedback = []
        self.terminal_state = ""

    def publish_feedback(self, feedback):
        self.feedback.append(feedback)

    def succeed(self):
        self.terminal_state = "succeeded"

    def abort(self):
        self.terminal_state = "aborted"

    def canceled(self):
        self.terminal_state = "canceled"


def _goal(position_rad=(0.0,) * 6):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(_JOINT_NAMES)
    point = JointTrajectoryPoint()
    point.positions = list(position_rad)
    point.time_from_start.sec = 0
    goal.trajectory.points = [point]
    return goal


def _coordinator(facade, goal_time_tolerance_s=0.0):
    return TrajectoryActionCoordinator(
        node=_Node(),
        lifecycle=_Lifecycle(facade),
        enable_command_interfaces=True,
        maximum_trajectory_points=1000,
        default_path_tolerance_rad=0.2,
        default_goal_tolerance_rad=0.05,
        default_goal_time_tolerance_s=goal_time_tolerance_s,
        old_header_tolerance_s=0.5,
    )


class ActionCoordinatorTest(unittest.TestCase):
    """Verify success feedback and path-tolerance terminal mapping."""

    def test_success_publishes_desired_actual_error_feedback(self) -> None:
        facade = _Facade(actual_position_rad=(0.0,) * 6)
        coordinator = _coordinator(facade)
        goal_handle = _GoalHandle()
        try:
            self.assertIs(coordinator.goal_callback(_goal()), GoalResponse.ACCEPT)
            result = coordinator.execute_callback(goal_handle)
        finally:
            coordinator.shutdown()

        self.assertEqual(goal_handle.terminal_state, "succeeded")
        self.assertEqual(result.error_code, TrajectoryErrorCode.SUCCESSFUL.value)
        self.assertEqual(len(goal_handle.feedback), 1)
        self.assertEqual(tuple(goal_handle.feedback[0].actual.positions), (0.0,) * 6)

    def test_path_tolerance_violation_aborts_goal(self) -> None:
        facade = _Facade(actual_position_rad=(1.0,) * 6)
        coordinator = _coordinator(facade)
        goal_handle = _GoalHandle()
        try:
            self.assertIs(coordinator.goal_callback(_goal()), GoalResponse.ACCEPT)
            result = coordinator.execute_callback(goal_handle)
        finally:
            coordinator.shutdown()

        self.assertEqual(goal_handle.terminal_state, "aborted")
        self.assertEqual(
            result.error_code,
            TrajectoryErrorCode.PATH_TOLERANCE_VIOLATED.value,
        )

    def test_cancel_retries_no_active_command_race(self) -> None:
        class PendingCancelFacade(_Facade):
            def __init__(self):
                super().__init__()
                self.command_active = False
                self.execution_started = threading.Event()
                self.cancel_accepted = threading.Event()

            def execute_trajectory(self, trajectory, progress_callback=None):
                del trajectory, progress_callback
                self.execution_started.set()
                time.sleep(0.06)
                self.command_active = True
                self.cancel_accepted.wait(timeout=0.5)
                self.command_active = False
                return CoreCommandOutcome(
                    status="canceled",
                    message="canceled after admission",
                    command_id="command-1",
                    error_code="COMMAND_CANCELED",
                )

            def cancel_current_command(self):
                self.cancel_count += 1
                if not self.command_active:
                    return CoreCommandOutcome(
                        status="rejected",
                        message="no active command",
                        command_id="none",
                        error_code="NO_ACTIVE_COMMAND",
                    )
                self.cancel_accepted.set()
                return CoreCommandOutcome(
                    status="succeeded",
                    message="cancel accepted",
                    command_id="command-1",
                    error_code=None,
                )

        facade = PendingCancelFacade()
        coordinator = _coordinator(facade)
        goal_handle = _GoalHandle()
        result_holder = []
        try:
            self.assertIs(coordinator.goal_callback(_goal()), GoalResponse.ACCEPT)
            worker = threading.Thread(
                target=lambda: result_holder.append(
                    coordinator.execute_callback(goal_handle)
                )
            )
            worker.start()
            self.assertTrue(facade.execution_started.wait(timeout=0.5))
            goal_handle.is_cancel_requested = True
            worker.join(timeout=1.0)
            self.assertFalse(worker.is_alive())
        finally:
            coordinator.shutdown()

        self.assertEqual(goal_handle.terminal_state, "canceled")
        self.assertEqual(
            result_holder[0].error_code,
            TrajectoryErrorCode.SUCCESSFUL.value,
        )
        self.assertGreaterEqual(facade.cancel_count, 2)

    def test_non_retryable_cancel_rejection_faults_and_aborts(self) -> None:
        class RejectedCancelFacade(_Facade):
            def __init__(self):
                super().__init__()
                self.execution_started = threading.Event()

            def execute_trajectory(self, trajectory, progress_callback=None):
                del trajectory, progress_callback
                self.execution_started.set()
                time.sleep(0.06)
                return CoreCommandOutcome(
                    status="succeeded",
                    message="unexpected completion",
                    command_id="command-1",
                    error_code=None,
                )

            def cancel_current_command(self):
                self.cancel_count += 1
                return CoreCommandOutcome(
                    status="rejected",
                    message="cancel rejected",
                    command_id="command-1",
                    error_code="CANCEL_REJECTED",
                )

        facade = RejectedCancelFacade()
        lifecycle = _Lifecycle(facade)
        coordinator = TrajectoryActionCoordinator(
            node=_Node(),
            lifecycle=lifecycle,
            enable_command_interfaces=True,
            maximum_trajectory_points=1000,
            default_path_tolerance_rad=0.2,
            default_goal_tolerance_rad=0.05,
            default_goal_time_tolerance_s=0.0,
            old_header_tolerance_s=0.5,
        )
        goal_handle = _GoalHandle()
        result_holder = []
        try:
            self.assertIs(coordinator.goal_callback(_goal()), GoalResponse.ACCEPT)
            worker = threading.Thread(
                target=lambda: result_holder.append(
                    coordinator.execute_callback(goal_handle)
                )
            )
            worker.start()
            self.assertTrue(facade.execution_started.wait(timeout=0.5))
            goal_handle.is_cancel_requested = True
            worker.join(timeout=1.0)
            self.assertFalse(worker.is_alive())
        finally:
            coordinator.shutdown()

        self.assertEqual(goal_handle.terminal_state, "aborted")
        self.assertEqual(
            result_holder[0].error_code,
            TrajectoryErrorCode.INVALID_GOAL.value,
        )
        self.assertIs(lifecycle.state, DriverLifecycleState.FAULT)

    def test_goal_tolerance_settle_honors_late_cancel(self) -> None:
        class SettlingFacade(_Facade):
            def __init__(self):
                super().__init__(actual_position_rad=(1.0,) * 6)
                self.settle_read = threading.Event()

            def execute_trajectory(self, trajectory, progress_callback=None):
                del trajectory, progress_callback
                return CoreCommandOutcome(
                    status="succeeded",
                    message="execution complete",
                    command_id="command-1",
                    error_code=None,
                )

            def read_joint_state(self):
                self.settle_read.set()
                return super().read_joint_state()

        facade = SettlingFacade()
        coordinator = _coordinator(facade, goal_time_tolerance_s=0.5)
        goal_handle = _GoalHandle()
        result_holder = []
        self.assertIs(coordinator.goal_callback(_goal()), GoalResponse.ACCEPT)
        worker = threading.Thread(
            target=lambda: result_holder.append(
                coordinator.execute_callback(goal_handle)
            )
        )
        worker.start()
        self.assertTrue(facade.settle_read.wait(timeout=0.5))
        goal_handle.is_cancel_requested = True
        worker.join(timeout=1.0)
        try:
            self.assertFalse(worker.is_alive())
            self.assertEqual(goal_handle.terminal_state, "canceled")
            self.assertEqual(facade.stop_count, 1)
            self.assertEqual(
                result_holder[0].error_code,
                TrajectoryErrorCode.SUCCESSFUL.value,
            )
        finally:
            coordinator.shutdown()
