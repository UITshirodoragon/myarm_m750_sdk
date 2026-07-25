#!/usr/bin/env python3
"""Bounded live ROS 2 gate for the mock driver and two-camera bridge."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import signal
import subprocess
import time
import xml.etree.ElementTree as ElementTree
from pathlib import Path
from typing import Any, Callable, Dict, Optional, Sequence, Set, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectoryPoint

_ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_flex_joint",
    "forearm_roll_joint",
    "wrist_flex_joint",
    "wrist_roll_joint",
)
_CAMERA_NAMES = ("mock_wrist_01", "mock_shoulder_02")
_DRIVER_ACTION = "/myarm_m750/follow_joint_trajectory"
_MINIMUM_NETWORK_SAMPLES = 100
_NETWORK_REPORT_TIMEOUT_S = 35.0
_CANONICAL_LINKS = {
    "base_link",
    "shoulder_link",
    "upper_arm_link",
    "lower_arm_link",
    "forearm_link",
    "wrist_link",
    "flange_link",
    "tool0",
    "gripper_base_link",
    "left_gripper_link",
    "right_gripper_link",
}
_CANONICAL_JOINTS = {
    "shoulder_pan_joint": ("revolute", "base_link", "shoulder_link"),
    "shoulder_lift_joint": (
        "revolute",
        "shoulder_link",
        "upper_arm_link",
    ),
    "elbow_flex_joint": (
        "revolute",
        "upper_arm_link",
        "lower_arm_link",
    ),
    "forearm_roll_joint": (
        "revolute",
        "lower_arm_link",
        "forearm_link",
    ),
    "wrist_flex_joint": (
        "revolute",
        "forearm_link",
        "wrist_link",
    ),
    "wrist_roll_joint": ("revolute", "wrist_link", "flange_link"),
    "flange_to_tool0_joint": ("fixed", "flange_link", "tool0"),
    "gripper_mount_joint": (
        "fixed",
        "flange_link",
        "gripper_base_link",
    ),
    "left_gripper_joint": (
        "prismatic",
        "gripper_base_link",
        "left_gripper_link",
    ),
    "right_gripper_joint": (
        "prismatic",
        "gripper_base_link",
        "right_gripper_link",
    ),
}
_EXPECTED_DYNAMIC_TF = {
    (parent, child)
    for joint_type, parent, child in _CANONICAL_JOINTS.values()
    if joint_type != "fixed"
}
_EXPECTED_STATIC_TF = {
    (parent, child)
    for joint_type, parent, child in _CANONICAL_JOINTS.values()
    if joint_type == "fixed"
}
_RELIABLE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
_IMAGE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
_DYNAMIC_TF_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
_STATIC_TF_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class GateFailure(RuntimeError):
    """A live ROS contract was not observed before its deadline."""


class ManagedLaunch:
    """Own an isolated launch process group and its diagnostic log."""

    def __init__(
        self,
        package: str,
        launch_file: str,
        arguments: Sequence[str],
        log_file: Path,
    ) -> None:
        command = ["ros2", "launch", package, launch_file]
        command.extend(arguments)
        log_file.parent.mkdir(parents=True, exist_ok=True)
        self._log_file = log_file
        self._stream = log_file.open("w", encoding="utf-8")
        self._process = subprocess.Popen(
            command,
            stdout=self._stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    @property
    def return_code(self) -> Optional[int]:
        """Return the launch return code without blocking."""
        return self._process.poll()

    def assert_running(self, context: str) -> None:
        """Fail with recent launch output when the process exits early."""
        return_code = self.return_code
        if return_code is None:
            return
        self._stream.flush()
        recent_output = self._log_file.read_text(
            encoding="utf-8", errors="replace"
        )[-4000:]
        raise GateFailure(
            f"{context}: launch exited with code {return_code}.\n"
            f"{recent_output}"
        )

    def stop(self, timeout_s: float = 8.0) -> float:
        """Request SIGINT and require the complete launch group to stop."""
        started_s = time.monotonic()
        if self._process.poll() is None:
            # Signal launch once; it owns orderly signaling of child nodes.
            self._process.send_signal(signal.SIGINT)
            try:
                self._process.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired as error:
                os.killpg(self._process.pid, signal.SIGTERM)
                try:
                    self._process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    os.killpg(self._process.pid, signal.SIGKILL)
                    self._process.wait(timeout=2.0)
                raise GateFailure(
                    f"ROS launch did not stop within {timeout_s:.1f}s."
                ) from error
        self._stream.close()
        return time.monotonic() - started_s


class DriverProbe:
    """Exercise lifecycle, state, diagnostics, feedback, and cancellation."""

    def __init__(
        self,
        node: Node,
        launch: ManagedLaunch,
        log_directory: Path,
        expected_description: str,
        expected_description_sha256: str,
    ) -> None:
        self._node = node
        self._launch = launch
        self._log_directory = log_directory
        self._expected_description = expected_description
        self._expected_description_sha256 = expected_description_sha256
        self._latest_joint_state: Optional[JointState] = None
        self._joint_state_count = 0
        self._diagnostic_count = 0
        self._lifecycle_state = ""
        self._capability_verification_reference = ""
        self._feedback_count = 0
        self._feedback_has_full_contract = False
        self._description_received = False
        self._dynamic_tf_message_count = 0
        self._dynamic_tf_transform_count = 0
        self._dynamic_tf_frames = set()  # type: Set[Tuple[str, str]]
        self._static_tf_message_count = 0
        self._static_tf_transform_count = 0
        self._static_tf_frames = set()  # type: Set[Tuple[str, str]]
        self._state_subscription = node.create_subscription(
            JointState,
            "/joint_states",
            self._record_joint_state,
            _RELIABLE_QOS,
        )
        self._diagnostic_subscription = node.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            self._record_diagnostics,
            _RELIABLE_QOS,
        )
        self._description_subscription = node.create_subscription(
            String,
            "/robot_description",
            self._record_robot_description,
            _STATIC_TF_QOS,
        )
        self._dynamic_tf_subscription = node.create_subscription(
            TFMessage,
            "/tf",
            self._record_dynamic_tf,
            _DYNAMIC_TF_QOS,
        )
        self._static_tf_subscription = node.create_subscription(
            TFMessage,
            "/tf_static",
            self._record_static_tf,
            _STATIC_TF_QOS,
        )
        self._configure = node.create_client(
            Trigger, "/myarm_m750_driver/configure"
        )
        self._activate = node.create_client(
            Trigger, "/myarm_m750_driver/activate"
        )
        self._deactivate = node.create_client(
            Trigger, "/myarm_m750_driver/deactivate"
        )
        self._cleanup = node.create_client(
            Trigger, "/myarm_m750_driver/cleanup"
        )
        self._action = ActionClient(
            node,
            FollowJointTrajectory,
            _DRIVER_ACTION,
        )

    def run(self, timeout_s: float) -> Dict[str, object]:
        """Run the complete mock-driver scenario."""
        self._wait_for(
            lambda: self._lifecycle_state == "unconfigured",
            timeout_s,
            "diagnostics while UNCONFIGURED",
        )
        self._call_transition(self._configure, "configure", timeout_s)
        self._wait_for(
            lambda: self._lifecycle_state == "inactive",
            timeout_s,
            "INACTIVE diagnostics",
        )
        self._call_transition(self._activate, "activate", timeout_s)
        self._wait_for(
            lambda: self._latest_joint_state is not None,
            timeout_s,
            "measured JointState after activation",
        )
        self._wait_for(
            self._has_remote_visualization_contract,
            timeout_s,
            "canonical robot description and dynamic/static TF",
        )
        self._assert_driver_qos()
        network_launch = self._start_network_probe()
        try:
            action_report = self._run_canceled_action(timeout_s)
            network_report = self._wait_for_network_report(
                network_launch,
                timeout_s,
            )
        finally:
            network_shutdown_s = network_launch.stop()
        self._call_transition(self._deactivate, "deactivate", timeout_s)
        self._wait_for(
            lambda: self._lifecycle_state == "inactive",
            timeout_s,
            "INACTIVE diagnostics after deactivation",
        )
        self._call_transition(self._cleanup, "cleanup", timeout_s)
        self._wait_for(
            lambda: self._lifecycle_state == "unconfigured",
            timeout_s,
            "UNCONFIGURED diagnostics after cleanup",
        )
        if not self._capability_verification_reference:
            raise GateFailure(
                "Driver diagnostics omitted stop-capability evidence."
            )
        report = {
            "joint_state_count": self._joint_state_count,
            "diagnostic_count": self._diagnostic_count,
            "final_lifecycle_state": self._lifecycle_state,
            "capability_verification_reference": (
                self._capability_verification_reference
            ),
            "robot_description_received": self._description_received,
            "robot_description_sha256": self._expected_description_sha256,
            "dynamic_tf_message_count": self._dynamic_tf_message_count,
            "dynamic_tf_transform_count": self._dynamic_tf_transform_count,
            "dynamic_tf_frames": _format_frame_edges(
                self._dynamic_tf_frames
            ),
            "static_tf_message_count": self._static_tf_message_count,
            "static_tf_transform_count": self._static_tf_transform_count,
            "static_tf_frames": _format_frame_edges(self._static_tf_frames),
            "network_probe": network_report,
            "network_probe_shutdown_s": network_shutdown_s,
        }
        report.update(action_report)
        return report

    def _start_network_probe(self) -> ManagedLaunch:
        report_json = self._log_directory / "network_probe.json"
        report_csv = self._log_directory / "network_probe.csv"
        return ManagedLaunch(
            "myarm_m750_visualization",
            "rviz_host.launch.py",
            (
                "headless:=true",
                "enable_network_probe:=true",
                f"report_json_file:={report_json}",
                f"report_csv_file:={report_csv}",
                "report_interval_s:=1.0",
            ),
            self._log_directory / "network_probe.log",
        )

    def _wait_for_network_report(
        self,
        network_launch: ManagedLaunch,
        timeout_s: float,
    ) -> Dict[str, object]:
        report_json = self._log_directory / "network_probe.json"
        report_csv = self._log_directory / "network_probe.csv"
        parsed = {}  # type: Dict[str, object]

        def report_passed() -> bool:
            nonlocal parsed
            network_launch.assert_running("local network probe report")
            if not report_json.is_file() or not report_csv.is_file():
                return False
            try:
                payload = json.loads(report_json.read_text(encoding="utf-8"))
            except (json.JSONDecodeError, OSError):
                return False
            if not isinstance(payload, dict) or not payload.get(
                "budget_passed"
            ):
                return False
            joint_metrics = payload.get("joint_state")
            if not isinstance(joint_metrics, dict):
                return False
            sample_count = joint_metrics.get("sample_count")
            if (
                not isinstance(sample_count, int)
                or isinstance(sample_count, bool)
                or sample_count < _MINIMUM_NETWORK_SAMPLES
            ):
                return False
            parsed = payload
            return True

        self._wait_for(
            report_passed,
            max(timeout_s, _NETWORK_REPORT_TIMEOUT_S),
            "passing local network-probe JSON/CSV report",
        )
        joint = parsed["joint_state"]
        control = parsed["control"]
        clock_sync = parsed["clock_sync"]
        if not isinstance(joint, dict) or not isinstance(control, dict):
            raise GateFailure("Network report metric sections are invalid.")
        if not isinstance(clock_sync, dict):
            raise GateFailure("Network report clock_sync section is invalid.")
        if clock_sync.get("source") != "local_loopback_same_clock":
            raise GateFailure(
                "Local network report has no explicit clock source."
            )
        if clock_sync.get("absolute_clock_offset_ms") != 0.0:
            raise GateFailure(
                "Local loopback clock offset must be exactly zero."
            )
        if float(joint["effective_rate_hz"]) < 4.5:
            raise GateFailure("Local joint-state rate is below 4.5 Hz.")
        if float(joint["message_age_p95_ms"]) > 250.0:
            raise GateFailure("Local p95 message age exceeds 250 ms.")
        if float(joint["maximum_gap_s"]) > 1.0:
            raise GateFailure("Local maximum state gap exceeds 1 second.")
        if float(control["bandwidth_mbit_s"]) > 1.0:
            raise GateFailure("Local control bandwidth exceeds 1 Mbit/s.")
        csv_header = report_csv.read_text(
            encoding="utf-8",
        ).splitlines()[0]
        if "clock_sync.source" not in csv_header:
            raise GateFailure(
                "Network CSV omitted explicit clock-sync fields."
            )
        return {
            "sample_count": joint["sample_count"],
            "effective_rate_hz": joint["effective_rate_hz"],
            "message_age_p95_ms": joint["message_age_p95_ms"],
            "message_age_p99_ms": joint["message_age_p99_ms"],
            "maximum_gap_s": joint["maximum_gap_s"],
            "bandwidth_mbit_s": control["bandwidth_mbit_s"],
            "clock_offset_source": clock_sync["source"],
            "absolute_clock_offset_ms": clock_sync[
                "absolute_clock_offset_ms"
            ],
            "budget_passed": parsed["budget_passed"],
        }

    def destroy(self) -> None:
        """Release the action client before destroying its node."""
        self._action.destroy()

    def _run_canceled_action(self, timeout_s: float) -> Dict[str, object]:
        if not self._action.wait_for_server(timeout_sec=timeout_s):
            raise GateFailure(f"Timed out waiting for {_DRIVER_ACTION}.")
        current = self._canonical_position()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(_ARM_JOINTS)
        first = JointTrajectoryPoint()
        first.positions = list(current)
        first.time_from_start.sec = 0
        first.time_from_start.nanosec = 300_000_000
        second = JointTrajectoryPoint()
        target = list(current)
        target[0] += 0.04
        second.positions = target
        second.time_from_start.sec = 3
        goal.trajectory.points = [first, second]

        goal_future = self._action.send_goal_async(
            goal,
            feedback_callback=self._record_feedback,
        )
        goal_handle = self._wait_future(
            goal_future, timeout_s, "trajectory goal acceptance"
        )
        if goal_handle is None or not goal_handle.accepted:
            raise GateFailure("Mock driver rejected a valid trajectory goal.")
        joint_count_at_accept = self._joint_state_count
        diagnostic_count_at_accept = self._diagnostic_count
        self._wait_for(
            lambda: (
                self._feedback_count > 0
                and self._joint_state_count >= joint_count_at_accept + 2
                and self._diagnostic_count > diagnostic_count_at_accept
            ),
            2.5,
            "state/diagnostics/feedback during active trajectory",
        )
        result_future = goal_handle.get_result_async()
        if result_future.done():
            raise GateFailure(
                "Trajectory finished before cancellation was sent."
            )
        cancel_future = goal_handle.cancel_goal_async()
        cancel_response = self._wait_future(
            cancel_future, timeout_s, "trajectory cancellation response"
        )
        if cancel_response is None or not cancel_response.goals_canceling:
            raise GateFailure("Driver did not accept trajectory cancellation.")
        wrapped_result = self._wait_future(
            result_future, timeout_s, "canceled trajectory terminal result"
        )
        if wrapped_result.status != GoalStatus.STATUS_CANCELED:
            raise GateFailure(
                "Canceled trajectory reached unexpected action status "
                f"{wrapped_result.status}."
            )
        if wrapped_result.result.error_code != 0:
            raise GateFailure(
                "Canceled trajectory returned non-success protocol code "
                f"{wrapped_result.result.error_code}."
            )
        if not self._feedback_has_full_contract:
            raise GateFailure(
                "FollowJointTrajectory feedback omitted desired/actual/error."
            )
        return {
            "action_terminal_status": wrapped_result.status,
            "action_feedback_count": self._feedback_count,
            "state_updates_during_action": (
                self._joint_state_count - joint_count_at_accept
            ),
            "diagnostics_during_action": (
                self._diagnostic_count - diagnostic_count_at_accept
            ),
        }

    def _canonical_position(self) -> Sequence[float]:
        state = self._latest_joint_state
        if state is None:
            raise GateFailure("No measured state is available.")
        values = dict(zip(state.name, state.position))
        missing = [name for name in _ARM_JOINTS if name not in values]
        if missing:
            raise GateFailure(
                f"JointState omitted canonical joints: {missing}."
            )
        return tuple(float(values[name]) for name in _ARM_JOINTS)

    def _record_joint_state(self, message: JointState) -> None:
        self._latest_joint_state = message
        self._joint_state_count += 1

    def _record_diagnostics(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name != "myarm_m750/driver":
                continue
            values = {entry.key: entry.value for entry in status.values}
            self._lifecycle_state = values.get("lifecycle_state", "")
            reference = values.get("capability_verification_reference", "")
            if reference:
                self._capability_verification_reference = reference
            self._diagnostic_count += 1

    def _record_robot_description(self, message: String) -> None:
        if message.data != self._expected_description:
            actual_sha256 = hashlib.sha256(
                message.data.encode("utf-8")
            ).hexdigest()
            raise GateFailure(
                "Published robot_description does not match the installed "
                "canonical full variant: "
                f"expected {self._expected_description_sha256}, "
                f"got {actual_sha256}."
            )
        _validate_canonical_robot_description(message.data)
        self._description_received = True

    def _record_dynamic_tf(self, message: TFMessage) -> None:
        self._dynamic_tf_message_count += 1
        self._dynamic_tf_transform_count += len(message.transforms)
        self._dynamic_tf_frames.update(
            _transform_edge(transform) for transform in message.transforms
        )

    def _record_static_tf(self, message: TFMessage) -> None:
        self._static_tf_message_count += 1
        self._static_tf_transform_count += len(message.transforms)
        self._static_tf_frames.update(
            _transform_edge(transform) for transform in message.transforms
        )

    def _has_remote_visualization_contract(self) -> bool:
        return (
            self._description_received
            and _EXPECTED_DYNAMIC_TF.issubset(self._dynamic_tf_frames)
            and _EXPECTED_STATIC_TF.issubset(self._static_tf_frames)
        )

    def _record_feedback(self, wrapped_feedback: Any) -> None:
        feedback = wrapped_feedback.feedback
        self._feedback_count += 1
        joint_count = len(feedback.joint_names)
        self._feedback_has_full_contract = (
            joint_count == len(_ARM_JOINTS)
            and len(feedback.desired.positions) == joint_count
            and len(feedback.actual.positions) == joint_count
            and len(feedback.error.positions) == joint_count
        )

    def _call_transition(
        self, client: Any, name: str, timeout_s: float
    ) -> None:
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise GateFailure(f"Timed out waiting for lifecycle {name}.")
        response = self._wait_future(
            client.call_async(Trigger.Request()),
            timeout_s,
            f"lifecycle {name}",
        )
        if response is None or not response.success:
            message = "" if response is None else response.message
            raise GateFailure(f"Lifecycle {name} failed: {message}")

    def _wait_for(
        self,
        predicate: Callable[[], bool],
        timeout_s: float,
        context: str,
    ) -> None:
        deadline_s = time.monotonic() + timeout_s
        while time.monotonic() < deadline_s:
            self._launch.assert_running(context)
            if predicate():
                return
            rclpy.spin_once(self._node, timeout_sec=0.05)
        raise GateFailure(f"Timed out waiting for {context}.")

    def _wait_future(
        self, future: Any, timeout_s: float, context: str
    ) -> Any:
        self._wait_for(lambda: bool(future.done()), timeout_s, context)
        error = future.exception()
        if error is not None:
            raise GateFailure(f"{context} raised: {error}") from error
        return future.result()

    def _assert_driver_qos(self) -> None:
        _assert_publisher_qos(
            self._node,
            "/joint_states",
            ReliabilityPolicy.RELIABLE,
            DurabilityPolicy.VOLATILE,
            5,
        )
        _assert_publisher_qos(
            self._node,
            "/robot_description",
            ReliabilityPolicy.RELIABLE,
            DurabilityPolicy.TRANSIENT_LOCAL,
            1,
        )
        _assert_publisher_qos(
            self._node,
            "/tf_static",
            ReliabilityPolicy.RELIABLE,
            DurabilityPolicy.TRANSIENT_LOCAL,
            1,
        )
        _assert_publisher_qos(
            self._node,
            "/tf",
            ReliabilityPolicy.RELIABLE,
            DurabilityPolicy.VOLATILE,
            100,
        )
        _assert_publisher_qos(
            self._node,
            "/diagnostics",
            ReliabilityPolicy.RELIABLE,
            DurabilityPolicy.VOLATILE,
            5,
        )


class CameraProbe:
    """Observe both mock cameras, calibration, TF, diagnostics, and QoS."""

    def __init__(self, node: Node, launch: ManagedLaunch) -> None:
        self._node = node
        self._launch = launch
        self._image_count = {name: 0 for name in _CAMERA_NAMES}
        self._info_count = {name: 0 for name in _CAMERA_NAMES}
        self._diagnostic_names: Set[str] = set()
        self._tf_children: Set[str] = set()
        self._subscriptions = []
        for camera_name in _CAMERA_NAMES:
            self._subscriptions.append(
                node.create_subscription(
                    Image,
                    f"/{camera_name}/image_raw",
                    lambda message, name=camera_name: self._record_image(
                        name, message
                    ),
                    _IMAGE_QOS,
                )
            )
            self._subscriptions.append(
                node.create_subscription(
                    CameraInfo,
                    f"/{camera_name}/camera_info",
                    lambda message, name=camera_name: self._record_info(
                        name, message
                    ),
                    _RELIABLE_QOS,
                )
            )
        self._subscriptions.append(
            node.create_subscription(
                DiagnosticArray,
                "/diagnostics",
                self._record_diagnostics,
                _RELIABLE_QOS,
            )
        )
        self._subscriptions.append(
            node.create_subscription(
                TFMessage,
                "/tf_static",
                self._record_static_tf,
                _STATIC_TF_QOS,
            )
        )

    def run(self, timeout_s: float) -> Dict[str, object]:
        """Require complete messages from two independent mock workers."""
        expected_diagnostics = {
            f"myarm_m750/camera/{name}" for name in _CAMERA_NAMES
        }
        expected_frames = {
            "mock_wrist_01_link",
            "mock_wrist_01_optical_frame",
            "mock_shoulder_02_link",
            "mock_shoulder_02_optical_frame",
        }
        self._wait_for(
            lambda: (
                all(count >= 2 for count in self._image_count.values())
                and all(count >= 2 for count in self._info_count.values())
                and expected_diagnostics.issubset(self._diagnostic_names)
                and expected_frames.issubset(self._tf_children)
            ),
            timeout_s,
            "two-camera Image/CameraInfo/TF/diagnostics contract",
        )
        self._assert_camera_qos()
        return {
            "image_count": dict(self._image_count),
            "camera_info_count": dict(self._info_count),
            "diagnostic_names": sorted(self._diagnostic_names),
            "tf_children": sorted(self._tf_children),
        }

    def _record_image(self, camera_name: str, message: Image) -> None:
        if (
            message.width != 640
            or message.height != 480
            or message.encoding != "bgr8"
            or len(message.data) != 640 * 480 * 3
        ):
            raise GateFailure(
                f"{camera_name} published an invalid raw Image contract."
            )
        self._image_count[camera_name] += 1

    def _record_info(self, camera_name: str, message: CameraInfo) -> None:
        if (
            message.width != 640
            or message.height != 480
            or len(message.k) != 9
            or len(message.p) != 12
        ):
            raise GateFailure(
                f"{camera_name} published an invalid CameraInfo contract."
            )
        self._info_count[camera_name] += 1

    def _record_diagnostics(self, message: DiagnosticArray) -> None:
        self._diagnostic_names.update(status.name for status in message.status)

    def _record_static_tf(self, message: TFMessage) -> None:
        self._tf_children.update(
            transform.child_frame_id for transform in message.transforms
        )

    def _wait_for(
        self,
        predicate: Callable[[], bool],
        timeout_s: float,
        context: str,
    ) -> None:
        deadline_s = time.monotonic() + timeout_s
        while time.monotonic() < deadline_s:
            self._launch.assert_running(context)
            if predicate():
                return
            rclpy.spin_once(self._node, timeout_sec=0.05)
        raise GateFailure(f"Timed out waiting for {context}.")

    def _assert_camera_qos(self) -> None:
        for camera_name in _CAMERA_NAMES:
            _assert_publisher_qos(
                self._node,
                f"/{camera_name}/image_raw",
                ReliabilityPolicy.BEST_EFFORT,
                DurabilityPolicy.VOLATILE,
                1,
            )
            _assert_publisher_qos(
                self._node,
                f"/{camera_name}/camera_info",
                ReliabilityPolicy.RELIABLE,
                DurabilityPolicy.VOLATILE,
                5,
            )
        _assert_publisher_qos(
            self._node,
            "/tf_static",
            ReliabilityPolicy.RELIABLE,
            DurabilityPolicy.TRANSIENT_LOCAL,
            1,
        )


def _assert_publisher_qos(
    node: Node,
    topic: str,
    reliability: Any,
    durability: Any,
    depth: int,
) -> None:
    endpoints = node.get_publishers_info_by_topic(topic)
    if not endpoints:
        raise GateFailure(f"No live publisher endpoint found for {topic}.")
    if not any(
        endpoint.qos_profile.reliability == reliability
        and endpoint.qos_profile.durability == durability
        # Foxy/Fast DDS reports depth zero through endpoint introspection even
        # when the matched publisher was created with an explicit depth.
        and endpoint.qos_profile.depth in (0, depth)
        for endpoint in endpoints
    ):
        observed = [
            {
                "reliability": int(endpoint.qos_profile.reliability),
                "durability": int(endpoint.qos_profile.durability),
                "depth": endpoint.qos_profile.depth,
            }
            for endpoint in endpoints
        ]
        raise GateFailure(
            f"{topic} QoS does not match the release contract: {observed}."
        )


def _load_installed_robot_description() -> Tuple[str, str]:
    description_share = Path(
        get_package_share_directory("myarm_m750_description")
    )
    manifest_file = description_share / "config" / "model_manifest.json"
    manifest = json.loads(manifest_file.read_text(encoding="utf-8"))
    full_variant = manifest["variants"]["full"]
    description_file = description_share / str(full_variant["path"])
    description_bytes = description_file.read_bytes()
    actual_sha256 = hashlib.sha256(description_bytes).hexdigest()
    expected_sha256 = str(full_variant["artifact_sha256"])
    if actual_sha256 != expected_sha256:
        raise GateFailure(
            "Installed full robot-description artifact does not match its "
            f"manifest: expected {expected_sha256}, got {actual_sha256}."
        )
    try:
        description = description_bytes.decode("utf-8")
    except UnicodeDecodeError as error:
        raise GateFailure(
            "Installed robot description is not valid UTF-8."
        ) from error
    _validate_canonical_robot_description(description)
    return description, expected_sha256


def _validate_canonical_robot_description(description: str) -> None:
    try:
        root = ElementTree.fromstring(description)
    except ElementTree.ParseError as error:
        raise GateFailure(
            f"Published robot_description is invalid XML: {error}."
        ) from error
    if root.tag != "robot" or root.get("name") != "myarm_m750":
        raise GateFailure(
            "Published robot_description has the wrong robot identity."
        )
    link_names = {
        str(link.get("name")) for link in root.findall("link")
    }
    if link_names != _CANONICAL_LINKS:
        raise GateFailure(
            "Published robot_description link set is not canonical: "
            f"{sorted(link_names)}."
        )
    joints = {}  # type: Dict[str, Tuple[str, str, str]]
    joint_elements = {}
    for joint in root.findall("joint"):
        name = str(joint.get("name"))
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            raise GateFailure(
                f"Published joint {name!r} has no parent/child contract."
            )
        joints[name] = (
            str(joint.get("type")),
            str(parent.get("link")),
            str(child.get("link")),
        )
        joint_elements[name] = joint
    if joints != _CANONICAL_JOINTS:
        raise GateFailure(
            "Published robot_description joint/frame contract is not "
            f"canonical: {joints}."
        )
    right_gripper = joint_elements["right_gripper_joint"]
    mimic = right_gripper.find("mimic")
    if mimic is None or mimic.get("joint") != "left_gripper_joint":
        raise GateFailure(
            "Published robot_description omitted the canonical gripper mimic."
        )


def _transform_edge(transform: Any) -> Tuple[str, str]:
    return (
        str(transform.header.frame_id),
        str(transform.child_frame_id),
    )


def _format_frame_edges(edges: Set[Tuple[str, str]]) -> Sequence[str]:
    return [
        f"{parent}->{child}" for parent, child in sorted(edges)
    ]


def _run_driver(
    core_config: Path,
    log_directory: Path,
    timeout_s: float,
) -> Dict[str, object]:
    description, description_sha256 = _load_installed_robot_description()
    launch = ManagedLaunch(
        "myarm_m750_bringup",
        "robot_local.launch.py",
        (
            f"core_config_file:={core_config}",
            "enable_command_interfaces:=true",
            "auto_configure:=false",
            "auto_activate:=false",
        ),
        log_directory / "driver.log",
    )
    node = Node("myarm_m750_driver_runtime_probe")
    probe = DriverProbe(
        node,
        launch,
        log_directory,
        description,
        description_sha256,
    )
    try:
        report = probe.run(timeout_s)
    finally:
        probe.destroy()
        node.destroy_node()
        report_shutdown_s = launch.stop()
    report["shutdown_s"] = report_shutdown_s
    return report


def _run_camera(
    camera_config: Path,
    log_directory: Path,
    timeout_s: float,
) -> Dict[str, object]:
    launch = ManagedLaunch(
        "myarm_m750_camera",
        "camera_bridge.launch.py",
        (
            "enable_camera:=true",
            f"core_camera_config_file:={camera_config}",
        ),
        log_directory / "camera.log",
    )
    node = Node("myarm_m750_camera_runtime_probe")
    try:
        report = CameraProbe(node, launch).run(timeout_s)
    finally:
        node.destroy_node()
        report_shutdown_s = launch.stop()
    report["shutdown_s"] = report_shutdown_s
    return report


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Run both live probes and print one machine-readable report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--core-config", required=True, type=Path)
    parser.add_argument("--camera-config", required=True, type=Path)
    parser.add_argument("--log-directory", required=True, type=Path)
    parser.add_argument("--timeout-s", type=float, default=12.0)
    parser.add_argument("--domain-id", type=int)
    arguments = parser.parse_args(argv)
    if arguments.timeout_s <= 0.0:
        raise ValueError("--timeout-s must be positive.")
    domain_id = (
        arguments.domain_id
        if arguments.domain_id is not None
        else 42
    )
    if domain_id < 0 or domain_id > 232:
        raise ValueError("--domain-id must be in [0, 232].")
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    ros_log_directory = arguments.log_directory.resolve() / "ros"
    ros_log_directory.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_LOG_DIR"] = str(ros_log_directory)

    rclpy.init(args=None)
    try:
        report = {
            "ros_domain_id": domain_id,
            "driver": _run_driver(
                arguments.core_config.resolve(),
                arguments.log_directory.resolve(),
                arguments.timeout_s,
            ),
            "camera": _run_camera(
                arguments.camera_config.resolve(),
                arguments.log_directory.resolve(),
                arguments.timeout_s,
            ),
        }  # type: Dict[str, object]
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
