"""Headless MoveIt planning, collision, execution, and cancellation probe.

``mock-execution`` is the passing release gate for planning, collision
rejection, and one successful ``ExecuteTrajectory`` request. ``mock-cancel``
is intentionally separate because MoveIt Foxy currently terminates the goal
before servicing the cancellation request; it remains a tracked blocker and
must not be folded into the passing execution evidence.
"""

from __future__ import annotations

import argparse
import json
import time
from typing import Any, Dict, List, Optional, Sequence, Set

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningScene,
    RobotState,
    RobotTrajectory,
)
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

_ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_flex_joint",
    "forearm_roll_joint",
    "wrist_flex_joint",
    "wrist_roll_joint",
)
_MODEL_JOINTS = _ARM_JOINTS + ("left_gripper_joint",)
_START_POSITION_RAD = (0.0,) * 7
_TARGET_POSITION_RAD = (0.12, -0.12, 0.18, 0.08, -0.08, 0.06)
_COLLISION_OBJECT_ID = "runtime_probe_blocker"
_PLANNING_SCALING = 0.05


class MoveItRuntimeProbe:
    """Drive only standard MoveIt service/action contracts."""

    def __init__(self, node: Node, timeout_s: float) -> None:
        self._node = node
        self._timeout_s = timeout_s
        self._plan_client = node.create_client(
            GetMotionPlan,
            "/plan_kinematic_path",
        )
        self._validity_client = node.create_client(
            GetStateValidity,
            "/check_state_validity",
        )
        self._scene_client = node.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
        )
        self._execute_client = ActionClient(
            node,
            ExecuteTrajectory,
            "/execute_trajectory",
        )
        self._move_group_client = ActionClient(
            node,
            MoveGroup,
            "/move_action",
        )
        self._latest_joint_names: Set[str] = set()
        self._joint_state_subscription = node.create_subscription(
            JointState,
            "/joint_states",
            self._record_joint_state,
            5,
        )

    def run(self, execute: bool) -> Dict[str, object]:
        """Verify planning and collision rejection, then optional execution."""
        self._wait_for_endpoints(
            require_execute=execute,
            require_move_group=False,
        )
        if execute:
            self._wait_for_active_driver()
        start_state = _robot_state(_START_POSITION_RAD)
        initial_validity = self._state_validity(start_state)
        if not initial_validity.valid:
            contacts = [
                f"{contact.contact_body_1}<->{contact.contact_body_2}"
                for contact in initial_validity.contacts
            ]
            raise RuntimeError(
                "Canonical zero start state is unexpectedly invalid; "
                f"contacts={contacts}."
            )

        planned = self._plan(start_state, _TARGET_POSITION_RAD)
        planning_response = planned.motion_plan_response
        if planning_response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                "Collision-free planning failed with MoveIt code "
                f"{planning_response.error_code.val}."
            )
        trajectory = planning_response.trajectory
        point_count = len(trajectory.joint_trajectory.points)
        if point_count < 2:
            raise RuntimeError(
                "MoveIt returned an empty or degenerate trajectory."
            )

        self._apply_blocking_collision()
        try:
            collision_state_valid = bool(
                self._state_validity(start_state).valid
            )
            collision_plan = self._plan(
                start_state,
                _TARGET_POSITION_RAD,
            ).motion_plan_response
        finally:
            self._remove_blocking_collision()
        if collision_state_valid:
            raise RuntimeError(
                "Blocking collision did not invalidate start state."
            )
        if collision_plan.error_code.val == MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                "MoveIt planned through an explicitly blocking object."
            )

        report = {
            "planning_error_code": planning_response.error_code.val,
            "planning_point_count": point_count,
            "planning_times_s": [
                _duration_seconds(point.time_from_start)
                for point in trajectory.joint_trajectory.points
            ],
            "collision_state_valid": collision_state_valid,
            "collision_planning_error_code": collision_plan.error_code.val,
        }  # type: Dict[str, object]
        if execute:
            report.update(self._execute_success(trajectory))
        return report

    def run_cancel_only(self) -> Dict[str, object]:
        """Cancel one slow current-state MoveGroup execution."""
        self._wait_for_endpoints(
            require_execute=True,
            require_move_group=True,
        )
        canceled = self._cancel_move_group_execution(
            _current_robot_state(),
            _TARGET_POSITION_RAD,
        )
        if canceled["status"] != GoalStatus.STATUS_CANCELED:
            raise RuntimeError(
                "MoveGroup cancel-only probe was not canceled; action status="
                f"{canceled['status']}, MoveIt code={canceled['error_code']}."
            )
        return {
            "cancel_status": canceled["status"],
            "cancel_error_code": canceled["error_code"],
            "cancel_feedback_states": canceled["feedback_states"],
        }

    def _wait_for_endpoints(
        self,
        require_execute: bool,
        require_move_group: bool,
    ) -> None:
        endpoints = (
            (self._plan_client, "/plan_kinematic_path"),
            (self._validity_client, "/check_state_validity"),
            (self._scene_client, "/apply_planning_scene"),
        )
        for client, name in endpoints:
            if not client.wait_for_service(timeout_sec=self._timeout_s):
                raise RuntimeError(
                    f"Timed out waiting for MoveIt service {name}."
                )
        if require_execute and not self._execute_client.wait_for_server(
            timeout_sec=self._timeout_s
        ):
            raise RuntimeError("Timed out waiting for /execute_trajectory.")
        if require_move_group and not self._move_group_client.wait_for_server(
            timeout_sec=self._timeout_s
        ):
            raise RuntimeError("Timed out waiting for /move_action.")

    def _plan(
        self,
        start_state: RobotState,
        target_position_rad: Sequence[float],
    ) -> GetMotionPlan.Response:
        request = GetMotionPlan.Request()
        request.motion_plan_request = self._motion_request(
            start_state,
            target_position_rad,
            # The core also checks conservative waypoint finite differences;
            # keep MoveIt's time parameterization inside that admission bound.
            velocity_scaling=_PLANNING_SCALING,
        )
        return self._call_service(self._plan_client, request)

    @staticmethod
    def _motion_request(
        start_state: RobotState,
        target_position_rad: Sequence[float],
        velocity_scaling: float,
    ) -> Any:
        request = GetMotionPlan.Request().motion_plan_request
        motion_request = request
        motion_request.group_name = "arm"
        motion_request.num_planning_attempts = 2
        motion_request.allowed_planning_time = 5.0
        motion_request.max_velocity_scaling_factor = velocity_scaling
        motion_request.max_acceleration_scaling_factor = velocity_scaling
        motion_request.start_state = start_state
        constraints = Constraints()
        constraints.name = "runtime_probe_target"
        constraints.joint_constraints = [
            _joint_constraint(name, position)
            for name, position in zip(_ARM_JOINTS, target_position_rad)
        ]
        motion_request.goal_constraints = [constraints]
        return motion_request

    def _state_validity(self, state: RobotState) -> GetStateValidity.Response:
        request = GetStateValidity.Request()
        request.robot_state = state
        request.group_name = "arm"
        return self._call_service(self._validity_client, request)

    def destroy(self) -> None:
        """Release the action client before its owning node is destroyed."""
        self._node.destroy_subscription(self._joint_state_subscription)
        self._move_group_client.destroy()
        self._execute_client.destroy()

    def _record_joint_state(self, message: JointState) -> None:
        self._latest_joint_names = set(message.name)

    def _wait_for_active_driver(self) -> None:
        """Require one complete state sample before requesting execution."""
        deadline_s = time.monotonic() + self._timeout_s
        required_names = set(_MODEL_JOINTS)
        while time.monotonic() < deadline_s:
            if required_names.issubset(self._latest_joint_names):
                return
            rclpy.spin_once(self._node, timeout_sec=0.05)
        missing = sorted(required_names - self._latest_joint_names)
        raise RuntimeError(
            "Timed out waiting for active driver joint state; "
            f"missing={missing}."
        )

    def _apply_blocking_collision(self) -> None:
        collision = CollisionObject()
        collision.header.frame_id = "base_link"
        collision.id = _COLLISION_OBJECT_ID
        collision.operation = CollisionObject.ADD
        collision.pose.orientation.w = 1.0
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [2.0, 2.0, 2.0]
        pose = Pose()
        pose.position.z = 0.25
        pose.orientation.w = 1.0
        collision.primitives = [primitive]
        collision.primitive_poses = [pose]
        self._apply_scene_object(collision)

    def _remove_blocking_collision(self) -> None:
        collision = CollisionObject()
        collision.header.frame_id = "base_link"
        collision.id = _COLLISION_OBJECT_ID
        collision.operation = CollisionObject.REMOVE
        self._apply_scene_object(collision)

    def _apply_scene_object(self, collision: CollisionObject) -> None:
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [collision]
        request = ApplyPlanningScene.Request()
        request.scene = scene
        response = self._call_service(self._scene_client, request)
        if not response.success:
            raise RuntimeError(
                f"MoveIt rejected planning-scene update for {collision.id}."
            )

    def _execute_success(
        self,
        trajectory: RobotTrajectory,
    ) -> Dict[str, object]:
        """Execute a planned trajectory and require a successful result."""
        succeeded = self._execute(trajectory)
        if succeeded["status"] != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                "Mock execution did not succeed; action status="
                f"{succeeded['status']}, "
                f"MoveIt code={succeeded['error_code']}."
            )
        if succeeded["error_code"] != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"Mock execution failed with code {succeeded['error_code']}."
            )
        return {
            "execution_status": succeeded["status"],
            "execution_error_code": succeeded["error_code"],
        }

    def _cancel_move_group_execution(
        self,
        start_state: RobotState,
        target_position_rad: Sequence[float],
    ) -> Dict[str, object]:
        goal = MoveGroup.Goal()
        goal.request = self._motion_request(
            start_state,
            target_position_rad,
            velocity_scaling=0.03,
        )
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        feedback_states = []  # type: List[str]

        def record_feedback(feedback: Any) -> None:
            feedback_states.append(str(feedback.feedback.state))

        goal_future = self._move_group_client.send_goal_async(
            goal,
            feedback_callback=record_feedback,
        )
        self._wait_for_future(goal_future, "MoveGroup goal acceptance")
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("MoveIt rejected MoveGroup cancel probe goal.")
        result_future = goal_handle.get_result_async()
        # Foxy reports PLANNING and IDLE but does not emit a MONITOR feedback
        # state while its trajectory execution manager is active.  The low
        # speed scaling makes execution multi-second; spin for one second so
        # planning completes and the controller goal is active before cancel.
        cancel_at_s = time.monotonic() + 1.0
        while time.monotonic() < cancel_at_s:
            if result_future.done():
                wrapped = result_future.result()
                raise RuntimeError(
                    "MoveGroup cancel probe finished before cancellation; "
                    f"status={wrapped.status}, states={feedback_states}."
                )
            rclpy.spin_once(self._node, timeout_sec=0.05)
        cancel_future = goal_handle.cancel_goal_async()
        self._wait_for_future(cancel_future, "MoveGroup cancellation")
        cancel_response = cancel_future.result()
        if cancel_response is None or not cancel_response.goals_canceling:
            return_code = (
                None
                if cancel_response is None
                else int(cancel_response.return_code)
            )
            raise RuntimeError(
                "MoveIt rejected MoveGroup cancellation; "
                f"return_code={return_code}, states={feedback_states}."
            )
        self._wait_for_future(result_future, "MoveGroup canceled result")
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError("MoveGroup cancellation returned no result.")
        return {
            "status": int(wrapped.status),
            "error_code": int(wrapped.result.error_code.val),
            "feedback_states": feedback_states,
        }

    def _execute(
        self,
        trajectory: RobotTrajectory,
    ) -> Dict[str, int]:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        goal_future = self._execute_client.send_goal_async(goal)
        self._wait_for_future(goal_future, "execute goal acceptance")
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("MoveIt rejected ExecuteTrajectory goal.")
        result_future = goal_handle.get_result_async()
        self._wait_for_future(result_future, "execute result")
        wrapped_result = result_future.result()
        if wrapped_result is None:
            raise RuntimeError("ExecuteTrajectory returned no result.")
        return {
            "status": int(wrapped_result.status),
            "error_code": int(wrapped_result.result.error_code.val),
        }

    def _call_service(self, client: Any, request: Any) -> Any:
        future = client.call_async(request)
        self._wait_for_future(future, "service response")
        response = future.result()
        if response is None:
            raise RuntimeError("MoveIt service returned no response.")
        return response

    def _wait_for_future(self, future: Any, operation: str) -> None:
        rclpy.spin_until_future_complete(
            self._node,
            future,
            timeout_sec=self._timeout_s,
        )
        if not future.done():
            raise RuntimeError(f"Timed out waiting for {operation}.")
        exception = future.exception()
        if exception is not None:
            raise RuntimeError(
                f"{operation} failed: {exception}"
            ) from exception


def _robot_state(position_rad: Sequence[float]) -> RobotState:
    state = RobotState()
    state.joint_state.name = list(_MODEL_JOINTS)
    state.joint_state.position = list(position_rad)
    state.is_diff = False
    return state


def _current_robot_state() -> RobotState:
    state = RobotState()
    state.is_diff = True
    return state


def _joint_constraint(name: str, position_rad: float) -> JointConstraint:
    constraint = JointConstraint()
    constraint.joint_name = name
    constraint.position = float(position_rad)
    constraint.tolerance_above = 0.002
    constraint.tolerance_below = 0.002
    constraint.weight = 1.0
    return constraint


def _duration_seconds(duration: Any) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1.0e-9


def main(argv: Optional[List[str]] = None) -> int:
    """Run one probe mode and emit machine-readable evidence."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode",
        choices=("plan-only", "mock-execution", "mock-cancel"),
        required=True,
    )
    parser.add_argument("--timeout-s", type=float, default=30.0)
    arguments = parser.parse_args(argv)
    if arguments.timeout_s <= 0.0:
        raise ValueError("--timeout-s must be positive.")

    rclpy.init()
    node = rclpy.create_node("myarm_m750_moveit_runtime_probe")
    probe = MoveItRuntimeProbe(node, arguments.timeout_s)
    try:
        if arguments.mode == "mock-cancel":
            report = probe.run_cancel_only()
        else:
            report = probe.run(
                execute=arguments.mode == "mock-execution"
            )
        report["mode"] = arguments.mode
        print(json.dumps(report, sort_keys=True))
    finally:
        probe.destroy()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
