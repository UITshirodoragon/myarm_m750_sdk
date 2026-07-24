"""ROS 2 bridge for the ROS-independent MyArm M750 Python Core."""

from __future__ import annotations

from typing import List, Optional

import rclpy
from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from myarm_m750_core import JointTrajectory, JointTrajectoryPoint, RobotSession
from myarm_m750_core.domain.models import CommandStatus


class MyArmM750DriverNode(Node):
    """Publish measured state and expose ``FollowJointTrajectory``.

    The node owns ROS interfaces only. Kinematics, validation, trajectory
    execution, firmware mapping, and adapter behavior remain in Python Core.
    The hardware adapter serializes vendor calls, so state polling may run in a
    second callback group while an action waits between trajectory points.
    """

    def __init__(self) -> None:
        super().__init__("myarm_m750_driver")
        self.declare_parameter("core_config_file", "")
        self.declare_parameter("state_rate_hz", 5.0)
        self.declare_parameter("publish_diagnostics", True)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter(
            "follow_joint_trajectory_action",
            "/myarm_m750/follow_joint_trajectory",
        )

        core_config_file = str(self.get_parameter("core_config_file").value)
        if not core_config_file:
            raise RuntimeError(
                "Parameter 'core_config_file' is required. Pass an absolute path "
                "to pycore/config/default.yaml or default_real.yaml."
            )
        state_rate_hz = float(self.get_parameter("state_rate_hz").value)
        if state_rate_hz <= 0.0:
            raise RuntimeError("state_rate_hz must be positive.")

        self._session = RobotSession.from_config(core_config_file)
        self._session.connect()
        self._joint_names = tuple(self._session.joint_names)
        self._publish_diagnostics = bool(
            self.get_parameter("publish_diagnostics").value
        )
        self._last_error = ""

        state_group = MutuallyExclusiveCallbackGroup()
        action_group = ReentrantCallbackGroup()
        joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        action_name = str(
            self.get_parameter("follow_joint_trajectory_action").value
        )
        self._joint_state_publisher = self.create_publisher(
            JointState, joint_state_topic, 10
        )
        self._diagnostic_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )

        # MyArmMControl blocks for write-then-read replies. A separate callback
        # group and executor thread keep ROS timers responsive; the adapter lock
        # remains the single serial critical section.
        self._state_timer = self.create_timer(
            1.0 / state_rate_hz,
            self._publish_state,
            callback_group=state_group,
        )
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=self._execute_trajectory,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=action_group,
        )
        self.get_logger().info(
            "MyArm M750 driver connected with config: {0}".format(core_config_file)
        )

    def _publish_state(self) -> None:
        try:
            state = self._session.get_state()
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(self._joint_names)
            message.position = list(state.position_rad)
            self._joint_state_publisher.publish(message)
            self._last_error = ""
        except Exception as error:  # ROS boundary converts failures to diagnostics.
            self._last_error = repr(error)
            self.get_logger().error("State poll failed: {0}".format(error))
        if self._publish_diagnostics:
            self._publish_driver_diagnostics()

    def _publish_driver_diagnostics(self) -> None:
        hardware_status = self._session.get_hardware_status()
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "myarm_m750/driver"
        status.hardware_id = "myarm_m750"
        is_healthy = hardware_status.connected and not self._last_error
        status.level = DiagnosticStatus.OK if is_healthy else DiagnosticStatus.ERROR
        status.message = self._last_error or hardware_status.message
        status.values = [
            KeyValue(key="runtime_state", value=self._session.state.value),
            KeyValue(key="adapter_state", value=hardware_status.state),
            KeyValue(key="connected", value=str(hardware_status.connected).lower()),
            KeyValue(key="joint_count", value=str(len(self._joint_names))),
            KeyValue(
                key="protocol_error_count",
                value=str(hardware_status.protocol_error_count),
            ),
            KeyValue(key="timeout_count", value=str(hardware_status.timeout_count)),
        ]
        message.status = [status]
        self._diagnostic_publisher.publish(message)

    def _goal_callback(self, goal_request: FollowJointTrajectory.Goal) -> GoalResponse:
        names = tuple(goal_request.trajectory.joint_names)
        if names != self._joint_names:
            self.get_logger().warning(
                "Rejected goal: joint order does not match canonical model."
            )
            return GoalResponse.REJECT
        if not goal_request.trajectory.points:
            self.get_logger().warning("Rejected goal: trajectory has no points.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: object) -> CancelResponse:
        del goal_handle
        return CancelResponse.ACCEPT

    @staticmethod
    def _duration_to_seconds(duration: object) -> float:
        return float(duration.sec) + float(duration.nanosec) * 1.0e-9

    def _to_core_trajectory(self, ros_trajectory: object) -> JointTrajectory:
        points: List[JointTrajectoryPoint] = []
        for point in ros_trajectory.points:
            points.append(
                JointTrajectoryPoint(
                    position_rad=tuple(point.positions),
                    time_from_start_s=self._duration_to_seconds(
                        point.time_from_start
                    ),
                )
            )
        return JointTrajectory(
            joint_names=tuple(ros_trajectory.joint_names),
            points=tuple(points),
        )

    def _execute_trajectory(self, goal_handle: object) -> FollowJointTrajectory.Result:
        result_message = FollowJointTrajectory.Result()
        try:
            trajectory = self._to_core_trajectory(goal_handle.request.trajectory)
            result = self._session.execute_trajectory(
                trajectory,
                cancel_requested=lambda: bool(goal_handle.is_cancel_requested),
            )
        except Exception as error:
            goal_handle.abort()
            result_message.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result_message.error_string = "Driver exception: {0}".format(error)
            self.get_logger().exception("Trajectory action failed.")
            return result_message

        if result.status is CommandStatus.SUCCEEDED:
            goal_handle.succeed()
            result_message.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        elif result.status is CommandStatus.CANCELED:
            goal_handle.canceled()
            result_message.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        else:
            goal_handle.abort()
            result_message.error_code = FollowJointTrajectory.Result.INVALID_GOAL
        result_message.error_string = result.message
        return result_message

    def destroy_node(self) -> bool:
        self._action_server.destroy()
        self._session.close()
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    """Run the driver with separate ROS callback threads."""
    rclpy.init(args=args)
    node: Optional[MyArmM750DriverNode] = None
    executor = MultiThreadedExecutor(num_threads=3)
    try:
        node = MyArmM750DriverNode()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
