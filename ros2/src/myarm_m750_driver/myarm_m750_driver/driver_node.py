"""Thin ROS 2 Foxy bridge for the ROS-independent MyArm M750 core."""

from __future__ import annotations

from typing import Callable, List, Optional, Sequence, Tuple

import rclpy
from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from myarm_m750_driver.action_coordinator import TrajectoryActionCoordinator
from myarm_m750_driver.contracts import DriverLifecycleState
from myarm_m750_driver.lifecycle_manager import DriverLifecycleManager

_STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
_DIAGNOSTIC_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
# The canonical model has one independent passive gripper joint.  The right
# finger is a URDF mimic joint, so robot_state_publisher derives it from this
# single neutral value.
_NEUTRAL_PASSIVE_JOINTS = (("left_gripper_joint", 0.0),)


def _complete_model_joint_state(
    joint_names: Sequence[str],
    position_rad: Sequence[float],
) -> Tuple[List[str], List[float]]:
    if len(joint_names) != len(position_rad):
        raise ValueError("Core joint-state name and position dimensions differ.")
    names = list(joint_names)
    positions = list(position_rad)
    for joint_name, neutral_position_rad in _NEUTRAL_PASSIVE_JOINTS:
        if joint_name not in names:
            names.append(joint_name)
            positions.append(neutral_position_rad)
    return names, positions


class MyArmM750DriverNode(Node):
    """Compose lifecycle, ROS interfaces, and the core facade."""

    def __init__(self) -> None:
        super().__init__("myarm_m750_driver")
        self._declare_parameters()
        state_rate_hz = self._positive_parameter("state_rate_hz")
        diagnostic_rate_hz = self._positive_parameter("diagnostic_rate_hz")

        core_config_file = str(self.get_parameter("core_config_file").value)
        use_real_hardware = bool(self.get_parameter("use_real_hardware").value)
        enable_commands = bool(
            self.get_parameter("enable_command_interfaces").value
        )
        self._lifecycle = DriverLifecycleManager(
            config_file=core_config_file,
            use_real_hardware=use_real_hardware,
            require_supported_stop=enable_commands,
        )
        self._use_real_hardware = use_real_hardware
        self._enable_commands = enable_commands
        self._last_state_error = ""

        state_group = MutuallyExclusiveCallbackGroup()
        lifecycle_group = MutuallyExclusiveCallbackGroup()
        action_group = ReentrantCallbackGroup()
        self._joint_state_publisher = self.create_publisher(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            _STATE_QOS,
        )
        self._diagnostic_publisher = self.create_publisher(
            DiagnosticArray,
            "/diagnostics",
            _DIAGNOSTIC_QOS,
        )
        self._state_timer = self.create_timer(
            1.0 / state_rate_hz,
            self._publish_state,
            callback_group=state_group,
        )
        self._diagnostic_timer = self.create_timer(
            1.0 / diagnostic_rate_hz,
            self._publish_diagnostics,
            callback_group=state_group,
        )
        self._create_lifecycle_services(lifecycle_group)

        self._action_coordinator = TrajectoryActionCoordinator(
            node=self,
            lifecycle=self._lifecycle,
            enable_command_interfaces=enable_commands,
            maximum_trajectory_points=self._positive_integer_parameter(
                "maximum_trajectory_points"
            ),
            default_path_tolerance_rad=self._non_negative_parameter(
                "default_path_tolerance_rad"
            ),
            default_goal_tolerance_rad=self._non_negative_parameter(
                "default_goal_tolerance_rad"
            ),
            default_goal_time_tolerance_s=self._non_negative_parameter(
                "default_goal_time_tolerance_s"
            ),
            old_header_tolerance_s=self._non_negative_parameter(
                "old_header_tolerance_s"
            ),
        )
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            str(
                self.get_parameter(
                    "follow_joint_trajectory_action"
                ).value
            ),
            execute_callback=self._action_coordinator.execute_callback,
            goal_callback=self._action_coordinator.goal_callback,
            cancel_callback=self._action_coordinator.cancel_callback,
            callback_group=action_group,
        )
        self._apply_automatic_transitions()

    def _declare_parameters(self) -> None:
        self.declare_parameter("core_config_file", "")
        self.declare_parameter("state_rate_hz", 5.0)
        self.declare_parameter("diagnostic_rate_hz", 1.0)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter(
            "follow_joint_trajectory_action",
            "/myarm_m750/follow_joint_trajectory",
        )
        self.declare_parameter("use_real_hardware", False)
        self.declare_parameter("enable_command_interfaces", False)
        self.declare_parameter("auto_configure", False)
        self.declare_parameter("auto_activate", False)
        self.declare_parameter("maximum_trajectory_points", 1000)
        self.declare_parameter("default_path_tolerance_rad", 0.2)
        self.declare_parameter("default_goal_tolerance_rad", 0.05)
        self.declare_parameter("default_goal_time_tolerance_s", 0.5)
        self.declare_parameter("old_header_tolerance_s", 0.5)

    def _create_lifecycle_services(self, callback_group: object) -> None:
        transitions = (
            ("~/configure", self._lifecycle.configure),
            ("~/activate", self._lifecycle.activate),
            ("~/deactivate", self._lifecycle.deactivate),
            ("~/cleanup", self._lifecycle.cleanup),
            ("~/recover", self._lifecycle.recover),
        )
        self._lifecycle_services = [
            self.create_service(
                Trigger,
                service_name,
                self._trigger_callback(transition),
                callback_group=callback_group,
            )
            for service_name, transition in transitions
        ]

    @staticmethod
    def _trigger_callback(
        transition: Callable[[], Tuple[bool, str]]
    ) -> Callable[[Trigger.Request, Trigger.Response], Trigger.Response]:
        def callback(
            request: Trigger.Request, response: Trigger.Response
        ) -> Trigger.Response:
            del request
            response.success, response.message = transition()
            return response

        return callback

    def _apply_automatic_transitions(self) -> None:
        auto_configure = bool(self.get_parameter("auto_configure").value)
        auto_activate = bool(self.get_parameter("auto_activate").value)
        if auto_activate and not auto_configure:
            raise RuntimeError("auto_activate=true requires auto_configure=true.")
        if not auto_configure:
            self.get_logger().info(
                "Driver ready in UNCONFIGURED state; lifecycle services are available."
            )
            return
        configured, configure_message = self._lifecycle.configure()
        self._log_transition(configured, configure_message)
        if configured and auto_activate:
            activated, activate_message = self._lifecycle.activate()
            self._log_transition(activated, activate_message)

    def _publish_state(self) -> None:
        if self._lifecycle.state is not DriverLifecycleState.ACTIVE:
            return
        facade = self._lifecycle.facade
        if facade is None:
            return
        try:
            state = facade.read_joint_state()
            message = JointState()
            seconds = int(state.sample_wall_time_s)
            message.header.stamp.sec = seconds
            message.header.stamp.nanosec = int(
                (state.sample_wall_time_s - seconds) * 1_000_000_000
            )
            message.name, message.position = _complete_model_joint_state(
                facade.joint_names,
                state.position_rad,
            )
            self._joint_state_publisher.publish(message)
            self._last_state_error = ""
        except Exception as error:
            self._last_state_error = repr(error)
            self._lifecycle.record_runtime_fault(error)
            self.get_logger().error(f"State poll failed: {error}")

    def _publish_diagnostics(self) -> None:
        lifecycle_state = self._lifecycle.state
        status = DiagnosticStatus()
        status.name = "myarm_m750/driver"
        status.hardware_id = "myarm_m750"
        status.level = (
            DiagnosticStatus.OK
            if lifecycle_state is DriverLifecycleState.ACTIVE
            else DiagnosticStatus.WARN
        )
        status.message = self._lifecycle.last_error or lifecycle_state.value
        values = [
            KeyValue(key="lifecycle_state", value=lifecycle_state.value),
            KeyValue(
                key="use_real_hardware",
                value=str(self._use_real_hardware).lower(),
            ),
            KeyValue(
                key="command_interfaces_enabled",
                value=str(self._enable_commands).lower(),
            ),
        ]
        facade = self._lifecycle.facade
        if facade is not None:
            values.extend(
                [
                    KeyValue(key="adapter_kind", value=facade.adapter_kind),
                    KeyValue(key="runtime_state", value=facade.runtime_state),
                    KeyValue(
                        key="model_contract_sha256",
                        value=facade.model_contract_sha256,
                    ),
                    KeyValue(
                        key="joint_count", value=str(len(facade.joint_names))
                    ),
                ]
            )
            identity = self._lifecycle.hardware_identity
            if identity is not None:
                values.append(
                    KeyValue(
                        key="capability_verification_reference",
                        value=identity.capability_verification_reference,
                    )
                )
            try:
                hardware_status = facade.read_hardware_status()
                values.extend(
                    [
                        KeyValue(
                            key="adapter_state", value=hardware_status.state
                        ),
                        KeyValue(
                            key="connected",
                            value=str(hardware_status.connected).lower(),
                        ),
                        KeyValue(
                            key="protocol_error_count",
                            value=str(hardware_status.protocol_error_count),
                        ),
                        KeyValue(
                            key="timeout_count",
                            value=str(hardware_status.timeout_count),
                        ),
                        KeyValue(
                            key="retry_count",
                            value=str(hardware_status.retry_count),
                        ),
                    ]
                )
                if (
                    lifecycle_state is DriverLifecycleState.ACTIVE
                    and not hardware_status.connected
                ):
                    status.level = DiagnosticStatus.ERROR
                    status.message = hardware_status.message
            except Exception as error:
                status.level = DiagnosticStatus.ERROR
                status.message = f"Diagnostic read failed: {error}"
        if lifecycle_state is DriverLifecycleState.FAULT:
            status.level = DiagnosticStatus.ERROR
        if self._last_state_error:
            status.level = DiagnosticStatus.ERROR
            status.message = self._last_state_error
        status.values = values
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self._diagnostic_publisher.publish(message)

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            raise RuntimeError(f"{name} must be positive.")
        return value

    def _non_negative_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if value < 0.0:
            raise RuntimeError(f"{name} must be non-negative.")
        return value

    def _positive_integer_parameter(self, name: str) -> int:
        value = self.get_parameter(name).value
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise RuntimeError(f"{name} must be a positive integer.")
        return value

    def _log_transition(self, succeeded: bool, message: str) -> None:
        if succeeded:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)

    def destroy_node(self) -> bool:
        self._action_coordinator.shutdown()
        self._action_server.destroy()
        self._lifecycle.shutdown()
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    """Run the driver with independent lifecycle/state/action callbacks."""
    rclpy.init(args=args)
    node: Optional[MyArmM750DriverNode] = None
    executor = MultiThreadedExecutor(num_threads=4)
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
