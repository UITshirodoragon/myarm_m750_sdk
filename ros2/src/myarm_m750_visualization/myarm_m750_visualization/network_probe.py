"""Measure remote ROS 2 observation quality without commanding the robot."""

from __future__ import annotations

import csv
import json
import math
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.serialization import serialize_message
from sensor_msgs.msg import JointState

from myarm_m750_visualization.network_metrics import NetworkMetrics, NetworkSnapshot

_OBSERVATION_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


@dataclass(frozen=True)
class ClockSyncMeasurement:
    """Explicit external clock-sync measurement; never inferred from messages."""

    available: bool
    measured_clock_offset_ms: Optional[float]
    absolute_clock_offset_ms: Optional[float]
    source: str

    def as_dict(self) -> Dict[str, object]:
        """Return a JSON/CSV-ready mapping."""
        return asdict(self)


class NetworkProbeNode(Node):
    """Subscribe to state/diagnostics and emit bounded JSON/CSV measurements."""

    def __init__(self) -> None:
        super().__init__("myarm_m750_network_probe")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("diagnostic_topic", "/diagnostics")
        self.declare_parameter("report_interval_s", 5.0)
        self.declare_parameter("report_json_file", "")
        self.declare_parameter("report_csv_file", "")
        self.declare_parameter("reconnect_gap_s", 1.0)
        self.declare_parameter("minimum_joint_state_rate_hz", 4.5)
        self.declare_parameter("maximum_p95_age_ms", 250.0)
        self.declare_parameter("maximum_p99_age_ms", 500.0)
        self.declare_parameter("maximum_gap_s", 1.0)
        self.declare_parameter("maximum_reconnect_s", 15.0)
        self.declare_parameter("maximum_clock_offset_ms", 20.0)
        self.declare_parameter("clock_offset_source", "")
        self.declare_parameter("measured_clock_offset_ms", 0.0)
        self.declare_parameter("require_clock_offset_measurement", True)
        self.declare_parameter("maximum_control_bandwidth_mbit_s", 1.0)

        start_monotonic_s = time.monotonic()
        self._joint_metrics = NetworkMetrics(
            start_monotonic_s=start_monotonic_s,
            reconnect_gap_s=self._positive_parameter("reconnect_gap_s"),
        )
        self._all_control_metrics = NetworkMetrics(
            start_monotonic_s=start_monotonic_s,
            reconnect_gap_s=self._positive_parameter("reconnect_gap_s"),
        )
        self._metrics_lock = threading.Lock()
        self._clock_sync = explicit_clock_sync_measurement(
            measured_clock_offset_ms=float(
                self.get_parameter("measured_clock_offset_ms").value
            ),
            source=str(self.get_parameter("clock_offset_source").value),
        )
        self._require_clock_offset_measurement = bool(
            self.get_parameter("require_clock_offset_measurement").value
        )
        self._report_json_file = str(
            self.get_parameter("report_json_file").value
        )
        self._report_csv_file = str(self.get_parameter("report_csv_file").value)
        self._joint_subscription = self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._joint_state_callback,
            _OBSERVATION_QOS,
        )
        self._diagnostic_subscription = self.create_subscription(
            DiagnosticArray,
            str(self.get_parameter("diagnostic_topic").value),
            self._diagnostic_callback,
            _OBSERVATION_QOS,
        )
        self._report_timer = self.create_timer(
            self._positive_parameter("report_interval_s"),
            self._periodic_report,
        )

    def _joint_state_callback(self, message: JointState) -> None:
        receive_monotonic_s = time.monotonic()
        receive_ros_s = self._now_ros_s()
        source_stamp_ros_s = _stamp_seconds(message.header.stamp)
        serialized_size = len(serialize_message(message))
        with self._metrics_lock:
            self._joint_metrics.observe(
                receive_monotonic_s,
                receive_ros_s,
                source_stamp_ros_s,
                serialized_size,
            )
            self._all_control_metrics.observe(
                receive_monotonic_s,
                receive_ros_s,
                source_stamp_ros_s,
                serialized_size,
            )

    def _diagnostic_callback(self, message: DiagnosticArray) -> None:
        receive_monotonic_s = time.monotonic()
        receive_ros_s = self._now_ros_s()
        source_stamp_ros_s = _stamp_seconds(message.header.stamp)
        with self._metrics_lock:
            self._all_control_metrics.observe(
                receive_monotonic_s,
                receive_ros_s,
                source_stamp_ros_s,
                len(serialize_message(message)),
            )

    def _periodic_report(self) -> None:
        report = self._create_report()
        joint = report["joint_state"]
        control = report["control"]
        passed = str(report["budget_passed"]).lower()
        self.get_logger().info(
            f"network_probe samples={joint['sample_count']} "
            f"rate_hz={joint['effective_rate_hz']:.3f} "
            f"p95_age_ms={joint['message_age_p95_ms']:.3f} "
            f"max_gap_s={joint['maximum_gap_s']:.3f} "
            f"silence_s={joint['current_silence_s']:.3f} "
            f"stale={str(joint['stale']).lower()} "
            f"bandwidth_mbit_s={control['bandwidth_mbit_s']:.4f} "
            f"passed={passed}"
        )
        self._write_reports(report)

    def _create_report(self) -> Dict[str, Any]:
        now_monotonic_s = time.monotonic()
        with self._metrics_lock:
            joint_snapshot = self._joint_metrics.snapshot(now_monotonic_s)
            control_snapshot = self._all_control_metrics.snapshot(now_monotonic_s)
        violations = self._budget_violations(
            joint_snapshot,
            control_snapshot,
            self._clock_sync,
        )
        return {
            "generated_wall_time_s": time.time(),
            "joint_state": joint_snapshot.as_dict(),
            "control": control_snapshot.as_dict(),
            "clock_sync": self._clock_sync.as_dict(),
            "budget_passed": not violations,
            "budget_violations": violations,
        }

    def _budget_violations(
        self,
        joint: NetworkSnapshot,
        control: NetworkSnapshot,
        clock_sync: ClockSyncMeasurement,
    ) -> List[str]:
        limits = {
            "effective_rate_hz": (
                joint.effective_rate_hz,
                float(
                    self.get_parameter("minimum_joint_state_rate_hz").value
                ),
                "minimum",
            ),
            "message_age_p95_ms": (
                joint.message_age_p95_ms,
                float(self.get_parameter("maximum_p95_age_ms").value),
                "maximum",
            ),
            "message_age_p99_ms": (
                joint.message_age_p99_ms,
                float(self.get_parameter("maximum_p99_age_ms").value),
                "maximum",
            ),
            "maximum_gap_s": (
                joint.maximum_gap_s,
                float(self.get_parameter("maximum_gap_s").value),
                "maximum",
            ),
            "maximum_reconnect_s": (
                joint.maximum_reconnect_s,
                float(self.get_parameter("maximum_reconnect_s").value),
                "maximum",
            ),
            "bandwidth_mbit_s": (
                control.bandwidth_mbit_s,
                float(
                    self.get_parameter(
                        "maximum_control_bandwidth_mbit_s"
                    ).value
                ),
                "maximum",
            ),
        }
        violations = []
        for name, (measured, limit, direction) in limits.items():
            if math.isnan(measured):
                violations.append(f"{name}: no timestamped samples")
            elif direction == "minimum" and measured < limit:
                violations.append(
                    f"{name}: {measured:.6g} < {limit:.6g}"
                )
            elif direction == "maximum" and measured > limit:
                violations.append(
                    f"{name}: {measured:.6g} > {limit:.6g}"
                )
        clock_violation = clock_offset_budget_violation(
            clock_sync,
            maximum_absolute_offset_ms=float(
                self.get_parameter("maximum_clock_offset_ms").value
            ),
            required=self._require_clock_offset_measurement,
        )
        if clock_violation is not None:
            violations.append(clock_violation)
        return violations

    def _write_reports(self, report: Dict[str, Any]) -> None:
        if self._report_json_file:
            path = Path(self._report_json_file).expanduser().resolve()
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(
                json.dumps(
                    _json_safe(report),
                    allow_nan=False,
                    indent=2,
                    sort_keys=True,
                ),
                encoding="utf-8",
            )
        if self._report_csv_file:
            flattened = _flatten_report(report)
            path = Path(self._report_csv_file).expanduser().resolve()
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=sorted(flattened))
                writer.writeheader()
                writer.writerow(flattened)

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            raise RuntimeError(f"{name} must be positive.")
        return value

    def _now_ros_s(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1.0e-9

    def destroy_node(self) -> bool:
        self._write_reports(self._create_report())
        return super().destroy_node()


def _stamp_seconds(stamp: Any) -> Optional[float]:
    stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
    return None if stamp_s == 0.0 else stamp_s


def _flatten_report(report: Dict[str, Any]) -> Dict[str, object]:
    flattened: Dict[str, object] = {
        "generated_wall_time_s": report["generated_wall_time_s"],
        "budget_passed": report["budget_passed"],
        "budget_violations": "; ".join(report["budget_violations"]),
    }
    for section in ("joint_state", "control", "clock_sync"):
        values = report[section]
        for key, value in values.items():
            flattened[f"{section}.{key}"] = value
    return flattened


def _json_safe(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value


def explicit_clock_sync_measurement(
    measured_clock_offset_ms: float,
    source: str,
) -> ClockSyncMeasurement:
    """Build an explicit clock measurement or an unavailable marker."""
    normalized_source = source.strip()
    if not normalized_source:
        return ClockSyncMeasurement(
            available=False,
            measured_clock_offset_ms=None,
            absolute_clock_offset_ms=None,
            source="unavailable",
        )
    offset_ms = float(measured_clock_offset_ms)
    if not math.isfinite(offset_ms):
        raise ValueError(
            "measured_clock_offset_ms must be finite when a source is declared."
        )
    return ClockSyncMeasurement(
        available=True,
        measured_clock_offset_ms=offset_ms,
        absolute_clock_offset_ms=abs(offset_ms),
        source=normalized_source,
    )


def clock_offset_budget_violation(
    measurement: ClockSyncMeasurement,
    maximum_absolute_offset_ms: float,
    required: bool,
) -> Optional[str]:
    """Return one explicit clock-budget violation, if any."""
    if not math.isfinite(maximum_absolute_offset_ms):
        raise ValueError("maximum_absolute_offset_ms must be finite.")
    if maximum_absolute_offset_ms <= 0.0:
        raise ValueError("maximum_absolute_offset_ms must be positive.")
    if not measurement.available:
        return (
            "clock_offset_ms: explicit chrony/NTP measurement unavailable"
            if required
            else None
        )
    absolute_offset_ms = measurement.absolute_clock_offset_ms
    if absolute_offset_ms is None:
        raise ValueError("Available clock measurement has no offset value.")
    if absolute_offset_ms > maximum_absolute_offset_ms:
        return (
            f"clock_offset_ms: {absolute_offset_ms:.6g} > "
            f"{maximum_absolute_offset_ms:.6g}"
        )
    return None


def main(args: Optional[List[str]] = None) -> None:
    """Run the read-only network probe."""
    rclpy.init(args=args)
    node = NetworkProbeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
