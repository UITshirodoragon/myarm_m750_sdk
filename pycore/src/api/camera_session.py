"""Public multi-camera session backed by independent workers."""

from __future__ import annotations

import threading
from typing import Mapping, Optional, Tuple

from myarm_m750_core.application.camera_pipeline import CameraWorker
from myarm_m750_core.domain.camera import (
    CameraConfig,
    CameraFrame,
    CameraMetricsSnapshot,
    CameraState,
)
from myarm_m750_core.domain.errors import CameraCaptureError, ConfigurationError


class CameraSession:
    """Own multiple cameras without coupling failures to robot state."""

    def __init__(
        self,
        configs: Mapping[str, CameraConfig],
        workers: Mapping[str, CameraWorker],
    ) -> None:
        self._configs = dict(configs)
        self._workers = dict(workers)
        self._started = False
        self._lock = threading.RLock()

    @property
    def camera_names(self) -> Tuple[str, ...]:
        """Return stable hardware identity topic names."""
        return tuple(self._workers)

    def config(self, camera_name: str) -> CameraConfig:
        """Return one immutable camera/calibration/extrinsic contract."""
        try:
            return self._configs[camera_name]
        except KeyError as error:
            raise ConfigurationError(f"Unknown camera: {camera_name}") from error

    def state(self, camera_name: str) -> CameraState:
        """Return one independent camera worker state."""
        return self._worker(camera_name).state

    def start(self) -> None:
        """Start every configured worker; roll back if startup itself fails."""
        with self._lock:
            if self._started:
                return
            started = []
            try:
                for worker in self._workers.values():
                    worker.start()
                    started.append(worker)
            except Exception:
                for worker in started:
                    worker.close()
                raise
            self._started = True

    def latest_frame(
        self,
        camera_name: str,
        timeout_s: float = 1.0,
        after_sequence: Optional[int] = None,
    ) -> CameraFrame:
        """Read the depth-one latest-frame queue for one hardware identity."""
        return self._worker(camera_name).latest_frame(
            timeout_s=timeout_s,
            after_sequence=after_sequence,
        )

    def metrics_snapshot(self, camera_name: str) -> CameraMetricsSnapshot:
        """Return one camera metrics snapshot."""
        return self._worker(camera_name).metrics_snapshot()

    def close(self, timeout_s: float = 2.0) -> None:
        """Bound and join every worker, preserving independent close attempts."""
        with self._lock:
            errors = []
            for camera_name, worker in self._workers.items():
                try:
                    worker.close(timeout_s=timeout_s)
                except Exception as error:
                    errors.append(f"{camera_name}: {error}")
            self._started = False
            if errors:
                raise CameraCaptureError(f"Camera shutdown failures: {'; '.join(errors)}")

    def __enter__(self) -> CameraSession:
        self.start()
        return self

    def __exit__(
        self, exception_type: object, exception: object, traceback: object
    ) -> None:
        del exception_type, exception, traceback
        self.close()

    def _worker(self, camera_name: str) -> CameraWorker:
        try:
            return self._workers[camera_name]
        except KeyError as error:
            raise ConfigurationError(f"Unknown camera: {camera_name}") from error
