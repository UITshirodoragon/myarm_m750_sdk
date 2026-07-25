"""Bounded, ROS-independent observation-channel metric aggregation."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import asdict, dataclass
from typing import Deque, Dict, Optional


@dataclass(frozen=True)
class NetworkSnapshot:
    """Serializable network-observation metrics."""

    sample_count: int
    discovery_time_s: Optional[float]
    reconnect_count: int
    maximum_reconnect_s: float
    effective_rate_hz: float
    message_age_p50_ms: float
    message_age_p95_ms: float
    message_age_p99_ms: float
    maximum_gap_s: float
    current_silence_s: float
    stale: bool
    source_stamp_skew_p99_ms: float
    bandwidth_mbit_s: float

    def as_dict(self) -> Dict[str, object]:
        """Return a JSON/CSV-ready mapping."""
        return asdict(self)


class NetworkMetrics:
    """Aggregate a bounded window while retaining whole-run counters."""

    def __init__(
        self,
        start_monotonic_s: float,
        reconnect_gap_s: float,
        window_size: int = 10_000,
    ) -> None:
        if reconnect_gap_s <= 0.0:
            raise ValueError("reconnect_gap_s must be positive.")
        if window_size < 2:
            raise ValueError("window_size must be at least two.")
        self._start_monotonic_s = start_monotonic_s
        self._reconnect_gap_s = reconnect_gap_s
        self._sample_times_s: Deque[float] = deque(maxlen=window_size)
        self._age_ms: Deque[float] = deque(maxlen=window_size)
        self._source_stamp_skew_ms: Deque[float] = deque(
            maxlen=window_size
        )
        self._sample_count = 0
        self._serialized_bytes = 0
        self._first_sample_s: Optional[float] = None
        self._last_sample_s: Optional[float] = None
        self._maximum_gap_s = 0.0
        self._reconnect_count = 0
        self._maximum_reconnect_s = 0.0

    def observe(
        self,
        receive_monotonic_s: float,
        receive_ros_s: float,
        source_stamp_ros_s: Optional[float],
        serialized_size_bytes: int,
    ) -> None:
        """Record one state/diagnostic message observation."""
        if serialized_size_bytes < 0:
            raise ValueError("serialized_size_bytes must be non-negative.")
        if self._first_sample_s is None:
            self._first_sample_s = receive_monotonic_s
        if self._last_sample_s is not None:
            gap_s = max(0.0, receive_monotonic_s - self._last_sample_s)
            self._maximum_gap_s = max(self._maximum_gap_s, gap_s)
            if gap_s > self._reconnect_gap_s:
                self._reconnect_count += 1
                self._maximum_reconnect_s = max(
                    self._maximum_reconnect_s, gap_s
                )
        self._last_sample_s = receive_monotonic_s
        self._sample_times_s.append(receive_monotonic_s)
        self._sample_count += 1
        self._serialized_bytes += serialized_size_bytes
        if source_stamp_ros_s is not None and source_stamp_ros_s > 0.0:
            signed_offset_ms = (receive_ros_s - source_stamp_ros_s) * 1_000.0
            self._age_ms.append(max(0.0, signed_offset_ms))
            # This combines source/receiver clock skew and transport age. It
            # is not an independent NTP/PTP clock-offset measurement.
            self._source_stamp_skew_ms.append(abs(signed_offset_ms))

    def snapshot(self, now_monotonic_s: float) -> NetworkSnapshot:
        """Return a deterministic metrics snapshot."""
        discovery_time_s = (
            None
            if self._first_sample_s is None
            else max(0.0, self._first_sample_s - self._start_monotonic_s)
        )
        current_silence_s = (
            max(0.0, now_monotonic_s - self._start_monotonic_s)
            if self._last_sample_s is None
            else max(0.0, now_monotonic_s - self._last_sample_s)
        )
        stale = (
            self._last_sample_s is None
            or current_silence_s > self._reconnect_gap_s
        )
        window_duration_s = 0.0
        if len(self._sample_times_s) >= 2:
            window_duration_s = max(
                0.0,
                now_monotonic_s - self._sample_times_s[0],
            )
        effective_rate_hz = (
            0.0
            if window_duration_s <= 0.0
            else (len(self._sample_times_s) - 1) / window_duration_s
        )
        run_duration_s = max(0.0, now_monotonic_s - self._start_monotonic_s)
        bandwidth_mbit_s = (
            0.0
            if run_duration_s <= 0.0
            else self._serialized_bytes * 8.0 / run_duration_s / 1_000_000.0
        )
        maximum_gap_s = max(self._maximum_gap_s, current_silence_s)
        maximum_reconnect_s = max(
            self._maximum_reconnect_s,
            current_silence_s if stale else 0.0,
        )
        return NetworkSnapshot(
            sample_count=self._sample_count,
            discovery_time_s=discovery_time_s,
            reconnect_count=self._reconnect_count,
            maximum_reconnect_s=maximum_reconnect_s,
            effective_rate_hz=effective_rate_hz,
            message_age_p50_ms=_percentile(self._age_ms, 50.0),
            message_age_p95_ms=_percentile(self._age_ms, 95.0),
            message_age_p99_ms=_percentile(self._age_ms, 99.0),
            maximum_gap_s=maximum_gap_s,
            current_silence_s=current_silence_s,
            stale=stale,
            source_stamp_skew_p99_ms=_percentile(
                self._source_stamp_skew_ms,
                99.0,
            ),
            bandwidth_mbit_s=bandwidth_mbit_s,
        )


def _percentile(values: Deque[float], percentile: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * percentile / 100.0
    lower_index = int(math.floor(rank))
    upper_index = int(math.ceil(rank))
    if lower_index == upper_index:
        return ordered[lower_index]
    fraction = rank - lower_index
    return (
        ordered[lower_index] * (1.0 - fraction)
        + ordered[upper_index] * fraction
    )
