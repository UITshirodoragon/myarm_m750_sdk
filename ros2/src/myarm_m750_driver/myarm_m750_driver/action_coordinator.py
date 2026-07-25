"""Single-goal ``FollowJointTrajectory`` execution coordinator."""

from __future__ import annotations

import concurrent.futures
import threading
import time
from typing import Any, Optional, Tuple

from control_msgs.action import FollowJointTrajectory
from rclpy.action import CancelResponse, GoalResponse
from trajectory_msgs.msg import JointTrajectoryPoint

from myarm_m750_driver.contracts import (
    AcceptedTrajectory,
    CanonicalTrajectoryPoint,
    CoreCommandOutcome,
    CoreJointSample,
    DriverLifecycleState,
    GoalConversionError,
    TrajectoryErrorCode,
)
from myarm_m750_driver.core_facade import CoreRobotFacade
from myarm_m750_driver.lifecycle_manager import DriverLifecycleManager
from myarm_m750_driver.trajectory_converter import (
    convert_follow_joint_trajectory_goal,
    violated_joint_names,
)

_ACTION_POLL_PERIOD_S = 0.02


class TrajectoryActionCoordinator:
    """Validate, serialize, monitor, and cancel one trajectory goal at a time."""

    def __init__(
        self,
        node: Any,
        lifecycle: DriverLifecycleManager,
        enable_command_interfaces: bool,
        maximum_trajectory_points: int,
        default_path_tolerance_rad: float,
        default_goal_tolerance_rad: float,
        default_goal_time_tolerance_s: float,
        old_header_tolerance_s: float,
    ) -> None:
        self._node = node
        self._lifecycle = lifecycle
        self._enable_command_interfaces = enable_command_interfaces
        if (
            isinstance(maximum_trajectory_points, bool)
            or not isinstance(maximum_trajectory_points, int)
            or maximum_trajectory_points <= 0
        ):
            raise ValueError(
                "maximum_trajectory_points must be a positive integer."
            )
        self._maximum_trajectory_points = maximum_trajectory_points
        self._default_path_tolerance_rad = default_path_tolerance_rad
        self._default_goal_tolerance_rad = default_goal_tolerance_rad
        self._default_goal_time_tolerance_s = default_goal_time_tolerance_s
        self._old_header_tolerance_s = old_header_tolerance_s
        self._goal_lock = threading.Lock()
        self._goal_reserved = False
        self._accepted_goal: Optional[AcceptedTrajectory] = None
        self._path_violation: Tuple[str, ...] = ()
        self._worker = concurrent.futures.ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="myarm-m750-core-command",
        )

    def goal_callback(self, goal_request: Any) -> GoalResponse:
        """Validate and reserve a goal without opening or commanding hardware."""
        if not self._enable_command_interfaces:
            self._node.get_logger().warning(
                "Rejected goal: command interfaces are disabled."
            )
            return GoalResponse.REJECT
        if self._lifecycle.state is not DriverLifecycleState.ACTIVE:
            self._node.get_logger().warning(
                "Rejected goal: driver lifecycle state is not ACTIVE."
            )
            return GoalResponse.REJECT
        facade = self._lifecycle.facade
        if facade is None:
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._goal_reserved:
                self._node.get_logger().warning(
                    "Rejected goal: another trajectory is active; preemption is disabled."
                )
                return GoalResponse.REJECT
            try:
                accepted = convert_follow_joint_trajectory_goal(
                    goal_request,
                    canonical_joint_names=facade.joint_names,
                    maximum_trajectory_points=(
                        self._maximum_trajectory_points
                    ),
                    now_ros_s=self._now_ros_s(),
                    default_path_tolerance_rad=self._default_path_tolerance_rad,
                    default_goal_tolerance_rad=self._default_goal_tolerance_rad,
                    default_goal_time_tolerance_s=(
                        self._default_goal_time_tolerance_s
                    ),
                    old_header_tolerance_s=self._old_header_tolerance_s,
                )
            except GoalConversionError as error:
                self._node.get_logger().warning(
                    f"Rejected trajectory goal: {error}"
                )
                return GoalResponse.REJECT
            self._goal_reserved = True
            self._accepted_goal = accepted
            self._path_violation = ()
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: Any) -> CancelResponse:
        """Accept cancellation; the execute loop invokes the core cancel path."""
        del goal_handle
        with self._goal_lock:
            is_active = self._goal_reserved
        return CancelResponse.ACCEPT if is_active else CancelResponse.REJECT

    def execute_callback(self, goal_handle: Any) -> FollowJointTrajectory.Result:
        """Execute one accepted goal and map its terminal state to ROS."""
        result_message = FollowJointTrajectory.Result()
        accepted = self._take_accepted_goal()
        if accepted is None:
            goal_handle.abort()
            return self._set_result(
                result_message,
                TrajectoryErrorCode.INVALID_GOAL,
                "Accepted goal metadata was not available.",
            )
        facade = self._lifecycle.facade
        if facade is None or self._lifecycle.state is not DriverLifecycleState.ACTIVE:
            self._release_goal()
            goal_handle.abort()
            return self._set_result(
                result_message,
                TrajectoryErrorCode.INVALID_GOAL,
                "Driver became inactive before execution.",
            )

        try:
            if not self._wait_for_start(goal_handle, accepted):
                if bool(goal_handle.is_cancel_requested):
                    goal_handle.canceled()
                    return self._set_result(
                        result_message,
                        TrajectoryErrorCode.SUCCESSFUL,
                        "Goal canceled before its requested start time.",
                    )
                goal_handle.abort()
                return self._set_result(
                    result_message,
                    TrajectoryErrorCode.INVALID_GOAL,
                    "Driver became inactive before the requested start time.",
                )
            future = self._worker.submit(
                facade.execute_trajectory,
                accepted.trajectory,
                lambda point_index, desired, actual: self._progress_callback(
                    goal_handle,
                    accepted,
                    point_index,
                    desired,
                    actual,
                ),
            )
            outcome = self._wait_for_core_result(goal_handle, facade, future)
            path_violation = self._get_path_violation()
            if path_violation:
                goal_handle.abort()
                return self._set_result(
                    result_message,
                    TrajectoryErrorCode.PATH_TOLERANCE_VIOLATED,
                    f"Path position tolerance violated by: "
                    f"{', '.join(path_violation)}.",
                )
            if bool(goal_handle.is_cancel_requested) or outcome.status == "canceled":
                goal_handle.canceled()
                return self._set_result(
                    result_message,
                    TrajectoryErrorCode.SUCCESSFUL,
                    outcome.message,
                )
            if not outcome.succeeded:
                goal_handle.abort()
                return self._set_result(
                    result_message,
                    TrajectoryErrorCode.INVALID_GOAL,
                    outcome.message,
                )
            goal_violation, canceled_during_settle = (
                self._wait_for_goal_tolerance(
                    goal_handle,
                    facade,
                    accepted,
                )
            )
            if canceled_during_settle:
                goal_handle.canceled()
                return self._set_result(
                    result_message,
                    TrajectoryErrorCode.SUCCESSFUL,
                    "Goal canceled during tolerance settling.",
                )
            if goal_violation:
                goal_handle.abort()
                return self._set_result(
                    result_message,
                    TrajectoryErrorCode.GOAL_TOLERANCE_VIOLATED,
                    f"Goal position tolerance violated by: "
                    f"{', '.join(goal_violation)}.",
                )
            if bool(goal_handle.is_cancel_requested):
                self._stop_after_execution(facade)
                goal_handle.canceled()
                return self._set_result(
                    result_message,
                    TrajectoryErrorCode.SUCCESSFUL,
                    "Goal canceled after trajectory execution.",
                )
            goal_handle.succeed()
            return self._set_result(
                result_message,
                TrajectoryErrorCode.SUCCESSFUL,
                outcome.message,
            )
        except Exception as error:
            self._lifecycle.record_runtime_fault(error)
            goal_handle.abort()
            self._node.get_logger().exception("Trajectory action failed.")
            return self._set_result(
                result_message,
                TrajectoryErrorCode.INVALID_GOAL,
                f"Driver exception: {error}",
            )
        finally:
            self._release_goal()

    def shutdown(self) -> None:
        """Stop accepting worker tasks during node destruction."""
        facade = self._lifecycle.facade
        if facade is not None:
            try:
                facade.cancel_current_command()
            except Exception as error:
                self._node.get_logger().error(
                    f"Core cancellation during shutdown failed: {error}"
                )
        self._worker.shutdown(wait=True)

    def _progress_callback(
        self,
        goal_handle: Any,
        accepted: AcceptedTrajectory,
        point_index: int,
        desired: CanonicalTrajectoryPoint,
        actual: CoreJointSample,
    ) -> None:
        del point_index
        violation = violated_joint_names(
            actual.position_rad,
            desired.position_rad,
            accepted.tolerance.path_position_rad,
            accepted.trajectory.joint_names,
        )
        if violation:
            with self._goal_lock:
                self._path_violation = violation
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self._node.get_clock().now().to_msg()
        feedback.joint_names = list(accepted.trajectory.joint_names)
        feedback.desired = self._to_ros_point(desired)
        feedback.actual = JointTrajectoryPoint()
        feedback.actual.positions = list(actual.position_rad)
        feedback.error = JointTrajectoryPoint()
        feedback.error.positions = [
            desired_value - actual_value
            for desired_value, actual_value in zip(
                desired.position_rad, actual.position_rad
            )
        ]
        goal_handle.publish_feedback(feedback)

    def _wait_for_core_result(
        self,
        goal_handle: Any,
        facade: CoreRobotFacade,
        future: concurrent.futures.Future[CoreCommandOutcome],
    ) -> CoreCommandOutcome:
        cancel_sent = False
        while not future.done():
            should_cancel = bool(goal_handle.is_cancel_requested) or bool(
                self._get_path_violation()
            )
            should_cancel = should_cancel or (
                self._lifecycle.state is not DriverLifecycleState.ACTIVE
            )
            if should_cancel and not cancel_sent:
                cancel_outcome = facade.cancel_current_command()
                if self._cancel_was_accepted(cancel_outcome):
                    cancel_sent = True
                elif (
                    cancel_outcome.status == "rejected"
                    and cancel_outcome.error_code == "NO_ACTIVE_COMMAND"
                ):
                    # The worker owns admission and may not have published its
                    # active command ID yet. Keep cancellation pending.
                    pass
                else:
                    raise RuntimeError(
                        "Core cancel was not accepted: "
                        f"{cancel_outcome.message}"
                    )
            time.sleep(_ACTION_POLL_PERIOD_S)
        outcome = future.result()
        needs_late_stop = (
            bool(goal_handle.is_cancel_requested)
            or bool(self._get_path_violation())
        )
        if needs_late_stop and not cancel_sent and outcome.status != "canceled":
            self._stop_after_execution(facade)
        return outcome

    def _wait_for_start(
        self, goal_handle: Any, accepted: AcceptedTrajectory
    ) -> bool:
        requested_start_s = accepted.trajectory.start_time_ros_s
        while (
            requested_start_s is not None
            and self._now_ros_s() < requested_start_s
        ):
            if (
                bool(goal_handle.is_cancel_requested)
                or self._lifecycle.state is not DriverLifecycleState.ACTIVE
            ):
                return False
            time.sleep(_ACTION_POLL_PERIOD_S)
        return not bool(goal_handle.is_cancel_requested)

    def _wait_for_goal_tolerance(
        self,
        goal_handle: Any,
        facade: CoreRobotFacade,
        accepted: AcceptedTrajectory,
    ) -> Tuple[Tuple[str, ...], bool]:
        deadline_s = (
            time.monotonic() + accepted.tolerance.goal_time_tolerance_s
        )
        target_position_rad = accepted.trajectory.points[-1].position_rad
        while True:
            if bool(goal_handle.is_cancel_requested):
                self._stop_after_execution(facade)
                return (), True
            actual = facade.read_joint_state()
            violation = violated_joint_names(
                actual.position_rad,
                target_position_rad,
                accepted.tolerance.goal_position_rad,
                accepted.trajectory.joint_names,
            )
            if not violation:
                return (), False
            if time.monotonic() >= deadline_s:
                return violation, False
            time.sleep(_ACTION_POLL_PERIOD_S)

    @staticmethod
    def _cancel_was_accepted(outcome: CoreCommandOutcome) -> bool:
        return outcome.succeeded or outcome.status == "canceled"

    def _stop_after_execution(self, facade: CoreRobotFacade) -> None:
        outcome = facade.stop()
        if not self._cancel_was_accepted(outcome):
            raise RuntimeError(
                f"Bounded stop after late cancel failed: {outcome.message}"
            )

    def _take_accepted_goal(self) -> Optional[AcceptedTrajectory]:
        with self._goal_lock:
            accepted = self._accepted_goal
            self._accepted_goal = None
            return accepted

    def _release_goal(self) -> None:
        with self._goal_lock:
            self._goal_reserved = False
            self._accepted_goal = None
            self._path_violation = ()

    def _get_path_violation(self) -> Tuple[str, ...]:
        with self._goal_lock:
            return self._path_violation

    def _now_ros_s(self) -> float:
        return float(self._node.get_clock().now().nanoseconds) * 1.0e-9

    @staticmethod
    def _to_ros_point(point: CanonicalTrajectoryPoint) -> JointTrajectoryPoint:
        message = JointTrajectoryPoint()
        message.positions = list(point.position_rad)
        if point.velocity_rad_s is not None:
            message.velocities = list(point.velocity_rad_s)
        if point.acceleration_rad_s2 is not None:
            message.accelerations = list(point.acceleration_rad_s2)
        seconds = int(point.time_from_start_s)
        message.time_from_start.sec = seconds
        message.time_from_start.nanosec = int(
            (point.time_from_start_s - seconds) * 1_000_000_000
        )
        return message

    @staticmethod
    def _set_result(
        result: FollowJointTrajectory.Result,
        code: TrajectoryErrorCode,
        message: str,
    ) -> FollowJointTrajectory.Result:
        result.error_code = code.value
        result.error_string = message
        return result
