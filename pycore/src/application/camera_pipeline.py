"""Independent latest-frame camera worker with bounded reconnect/shutdown."""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

from myarm_m750_core.domain.camera import (
    CameraConfig,
    CameraFrame,
    CameraMetricsSnapshot,
    CameraState,
)
from myarm_m750_core.domain.errors import (
    CameraCaptureError,
    CameraTimeoutError,
)
from myarm_m750_core.ports.camera import CameraCapturePort


class CameraWorker:
    """Own one camera thread and a latest-frame queue of depth one."""

    def __init__(
        self,
        config: CameraConfig,
        capture: CameraCapturePort,
        monotonic_clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._config = config
        self._capture = capture
        self._clock = monotonic_clock
        self._state = CameraState.CLOSED
        self._thread = None  # type: Optional[threading.Thread]
        self._stop_event = threading.Event()
        self._condition = threading.Condition(threading.RLock())
        self._latest_frame = None  # type: Optional[CameraFrame]
        self._consumed_sequence = 0
        self._frames_captured = 0
        self._read_timeouts = 0
        self._capture_errors = 0
        self._reconnect_count = 0
        self._queue_overflow_count = 0
        self._last_error = ""

    @property
    def state(self) -> CameraState:
        """Return current worker state."""
        with self._condition:
            return self._state

    def start(self) -> None:
        """Start the worker once; no other camera or robot is touched."""
        with self._condition:
            if self._thread is not None and self._thread.is_alive():
                return
            if not self._config.enabled:
                raise CameraCaptureError(
                    f"Camera is disabled: {self._config.hardware_name}"
                )
            self._stop_event.clear()
            self._state = CameraState.OPENING
            self._thread = threading.Thread(
                target=self._run,
                name=f"camera-{self._config.hardware_name}",
                daemon=True,
            )
            self._thread.start()

    def latest_frame(
        self,
        timeout_s: float,
        after_sequence: Optional[int] = None,
    ) -> CameraFrame:
        """Return the latest/newer frame or raise a typed timeout."""
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive.")
        deadline_s = self._clock() + timeout_s
        with self._condition:
            while True:
                frame = self._latest_frame
                if frame is not None and (
                    after_sequence is None or frame.sequence > after_sequence
                ):
                    self._consumed_sequence = frame.sequence
                    return frame
                remaining_s = deadline_s - self._clock()
                if remaining_s <= 0.0:
                    raise CameraTimeoutError(
                        f"No frame from {self._config.hardware_name} within "
                        f"{timeout_s:.3f}s."
                    )
                if self._state in (CameraState.CLOSED, CameraState.FAULT):
                    raise CameraCaptureError(
                        f"Camera {self._config.hardware_name} is "
                        f"{self._state.value}: {self._last_error}"
                    )
                self._condition.wait(timeout=remaining_s)

    def metrics_snapshot(self) -> CameraMetricsSnapshot:
        """Return an immutable per-camera metrics snapshot."""
        with self._condition:
            age_s = 0.0
            if self._latest_frame is not None:
                age_s = max(
                    0.0,
                    self._clock() - self._latest_frame.acquisition_monotonic_s,
                )
            return CameraMetricsSnapshot(
                frames_captured=self._frames_captured,
                read_timeouts=self._read_timeouts,
                capture_errors=self._capture_errors,
                reconnect_count=self._reconnect_count,
                queue_overflow_count=self._queue_overflow_count,
                last_frame_age_s=age_s,
                last_error=self._last_error,
            )

    def close(self, timeout_s: float = 2.0) -> None:
        """Signal shutdown, close capture, and join within a bounded timeout."""
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive.")
        with self._condition:
            thread = self._thread
            if thread is None:
                self._capture.close()
                self._state = CameraState.CLOSED
                return
            self._state = CameraState.STOPPING
            self._stop_event.set()
            self._condition.notify_all()
        self._capture.close()
        thread.join(timeout=timeout_s)
        with self._condition:
            if thread.is_alive():
                self._state = CameraState.FAULT
                self._last_error = "camera_shutdown_timeout"
                raise CameraCaptureError(
                    f"Camera worker did not stop within {timeout_s:.3f}s."
                )
            self._thread = None
            self._state = CameraState.CLOSED
            self._condition.notify_all()

    def _set_state(self, state: CameraState, error: str = "") -> None:
        with self._condition:
            self._state = state
            if error:
                self._last_error = error
            self._condition.notify_all()

    def _publish(self, frame: CameraFrame) -> None:
        with self._condition:
            previous = self._latest_frame
            if previous is not None and previous.sequence > self._consumed_sequence:
                self._queue_overflow_count += 1
            self._latest_frame = frame
            self._frames_captured += 1
            self._last_error = ""
            self._condition.notify_all()

    def _run(self) -> None:
        backoff_s = self._config.reconnect.initial_backoff_s
        attempts = 0
        frame_period_s = 1.0 / self._config.stream.fps_hz
        while not self._stop_event.is_set():
            try:
                self._set_state(CameraState.OPENING)
                self._capture.open(self._config)
                self._set_state(CameraState.STREAMING)
                while not self._stop_event.is_set():
                    frame_started_s = self._clock()
                    frame = self._capture.read_frame(self._config.reconnect.read_timeout_s)
                    self._publish(frame)
                    attempts = 0
                    backoff_s = self._config.reconnect.initial_backoff_s
                    remaining_s = frame_period_s - (self._clock() - frame_started_s)
                    if remaining_s > 0.0:
                        self._stop_event.wait(remaining_s)
            except CameraTimeoutError as error:
                with self._condition:
                    self._read_timeouts += 1
                self._handle_capture_failure(str(error))
            except Exception as error:
                with self._condition:
                    self._capture_errors += 1
                self._handle_capture_failure(str(error))
            finally:
                self._capture.close()
            if self._stop_event.is_set():
                break
            attempts += 1
            maximum = self._config.reconnect.maximum_attempts
            if maximum and attempts > maximum:
                self._set_state(CameraState.FAULT, "reconnect_attempts_exhausted")
                return
            with self._condition:
                self._reconnect_count += 1
            self._set_state(CameraState.BACKOFF)
            self._stop_event.wait(backoff_s)
            backoff_s = min(
                self._config.reconnect.maximum_backoff_s,
                max(
                    self._config.reconnect.initial_backoff_s,
                    backoff_s * self._config.reconnect.multiplier,
                ),
            )
        self._set_state(CameraState.CLOSED)

    def _handle_capture_failure(self, error: str) -> None:
        with self._condition:
            self._last_error = error
            self._condition.notify_all()
