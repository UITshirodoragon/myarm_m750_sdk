"""Lifecycle-equivalent manager for ROS 2 Foxy Python nodes."""

from __future__ import annotations

import threading
from typing import Callable, Optional, Tuple

from myarm_m750_driver.contracts import (
    CoreCommandOutcome,
    CoreHardwareIdentity,
    DriverLifecycleState,
)
from myarm_m750_driver.core_facade import CoreRobotFacade

FacadeFactory = Callable[[str], CoreRobotFacade]


class DriverLifecycleManager:
    """Own core composition and explicit lifecycle transitions."""

    def __init__(
        self,
        config_file: str,
        use_real_hardware: bool,
        require_supported_stop: bool = False,
        facade_factory: FacadeFactory = CoreRobotFacade,
    ) -> None:
        self._config_file = config_file
        self._use_real_hardware = use_real_hardware
        self._require_supported_stop = require_supported_stop
        self._facade_factory = facade_factory
        self._facade: Optional[CoreRobotFacade] = None
        self._hardware_identity: Optional[CoreHardwareIdentity] = None
        self._state = DriverLifecycleState.UNCONFIGURED
        self._last_error = ""
        self._lock = threading.RLock()

    @property
    def state(self) -> DriverLifecycleState:
        """Return the explicit lifecycle-equivalent state."""
        with self._lock:
            return self._state

    @property
    def last_error(self) -> str:
        """Return the most recent transition/runtime error."""
        with self._lock:
            return self._last_error

    @property
    def facade(self) -> Optional[CoreRobotFacade]:
        """Return the configured facade for bounded ROS callbacks."""
        with self._lock:
            return self._facade

    @property
    def hardware_identity(self) -> Optional[CoreHardwareIdentity]:
        """Return the most recently verified configured hardware identity."""
        with self._lock:
            return self._hardware_identity

    def configure(self) -> Tuple[bool, str]:
        """Build and validate a disconnected session."""
        with self._lock:
            if self._state is not DriverLifecycleState.UNCONFIGURED:
                return False, "configure requires UNCONFIGURED state."
            try:
                facade = self._facade_factory(self._config_file)
                facade.configure()
                self._validate_adapter_intent(facade.adapter_kind)
            except Exception as error:
                self._record_fault(error)
                return False, f"Configure failed: {error}"
            self._facade = facade
            self._state = DriverLifecycleState.INACTIVE
            self._last_error = ""
            return True, "Driver configured; hardware remains closed."

    def activate(self) -> Tuple[bool, str]:
        """Connect, probe identity/state, then enable state publication."""
        with self._lock:
            if self._state is not DriverLifecycleState.INACTIVE:
                return False, "activate requires INACTIVE state."
            facade = self._require_facade()
            try:
                facade.connect()
                identity = facade.probe_hardware()
                self._validate_command_capabilities(facade, identity)
            except Exception as error:
                try:
                    facade.close()
                except Exception as cleanup_error:
                    error = RuntimeError(
                        f"{error}; activation cleanup failed: {cleanup_error}"
                    )
                self._record_fault(error)
                return False, f"Activation failed: {error}"
            self._hardware_identity = identity
            self._state = DriverLifecycleState.ACTIVE
            self._last_error = ""
            return True, "Driver active."

    def deactivate(self) -> Tuple[bool, str]:
        """Cancel motion and close hardware while preserving configuration."""
        with self._lock:
            if self._state is not DriverLifecycleState.ACTIVE:
                return False, "deactivate requires ACTIVE state."
            facade = self._require_facade()
            try:
                outcome = facade.cancel_current_command()
                self._require_safe_cancel(outcome)
                facade.close()
            except Exception as error:
                self._record_fault(error)
                return False, f"Deactivation failed: {error}"
            self._state = DriverLifecycleState.INACTIVE
            self._last_error = ""
            return True, "Driver inactive; hardware closed."

    def cleanup(self) -> Tuple[bool, str]:
        """Release the session and return to UNCONFIGURED."""
        with self._lock:
            if self._state not in (
                DriverLifecycleState.INACTIVE,
                DriverLifecycleState.FAULT,
            ):
                return False, "cleanup requires INACTIVE or FAULT state."
            if self._facade is not None:
                try:
                    self._facade.close()
                except Exception as error:
                    self._record_fault(error)
                    return False, f"Cleanup failed: {error}"
            self._facade = None
            self._hardware_identity = None
            self._state = DriverLifecycleState.UNCONFIGURED
            self._last_error = ""
            return True, "Driver resources cleaned."

    def recover(self) -> Tuple[bool, str]:
        """Reopen and re-probe hardware after any ROS/core boundary fault."""
        with self._lock:
            if self._state is not DriverLifecycleState.FAULT:
                return False, "recover requires FAULT state."
            if self._facade is None:
                return (
                    False,
                    "No configured session; use cleanup then configure.",
                )
            try:
                # A ROS boundary fault (for example a state-poll exception)
                # does not necessarily move the core state machine to FAULT. A
                # reconnect and identity/state probe is therefore the single
                # recovery policy for both ROS-only and core-originated faults.
                self._facade.close()
                self._facade.connect()
                identity = self._facade.probe_hardware()
                self._validate_command_capabilities(self._facade, identity)
            except Exception as error:
                try:
                    self._facade.close()
                except Exception as cleanup_error:
                    error = RuntimeError(
                        f"{error}; recovery cleanup failed: {cleanup_error}"
                    )
                self._record_fault(error)
                return False, f"Recovery failed: {error}"
            self._hardware_identity = identity
            self._state = DriverLifecycleState.ACTIVE
            self._last_error = ""
            return True, "Driver recovered after reconnect and hardware probe."

    def record_runtime_fault(self, error: Exception) -> None:
        """Record an unexpected core boundary failure for diagnostics."""
        with self._lock:
            self._record_fault(error)

    def shutdown(self) -> None:
        """Best-effort release used only during process shutdown."""
        with self._lock:
            errors = []
            if self._facade is not None:
                try:
                    outcome = self._facade.cancel_current_command()
                    self._require_safe_cancel(outcome)
                except Exception as error:
                    errors.append(f"cancel failed: {error!r}")
                try:
                    self._facade.close()
                except Exception as error:
                    errors.append(f"close failed: {error!r}")
            self._last_error = "; ".join(errors)
            if errors:
                self._state = DriverLifecycleState.FAULT
                return
            self._facade = None
            self._hardware_identity = None
            self._state = DriverLifecycleState.UNCONFIGURED

    def _validate_adapter_intent(self, adapter_kind: str) -> None:
        adapter_is_real = adapter_kind not in ("mock", "replay")
        if adapter_is_real != self._use_real_hardware:
            requested = str(self._use_real_hardware).lower()
            raise RuntimeError(
                f"use_real_hardware={requested} conflicts with "
                f"core adapter_kind='{adapter_kind}'."
            )

    def _validate_command_capabilities(
        self,
        facade: CoreRobotFacade,
        identity: CoreHardwareIdentity,
    ) -> None:
        if not self._require_supported_stop:
            return
        if facade.adapter_capabilities.stop != "supported":
            raise RuntimeError(
                "Command interfaces require a verified SUPPORTED stop "
                "capability."
            )
        if not identity.capability_verification_reference.strip():
            raise RuntimeError(
                "Command interfaces require a non-empty stop capability "
                "verification reference."
            )

    def _require_facade(self) -> CoreRobotFacade:
        if self._facade is None:
            raise RuntimeError("Driver is not configured.")
        return self._facade

    def _record_fault(self, error: Exception) -> None:
        self._state = DriverLifecycleState.FAULT
        self._last_error = repr(error)

    @staticmethod
    def _require_safe_cancel(outcome: CoreCommandOutcome) -> None:
        if outcome.succeeded:
            return
        if (
            outcome.status == "rejected"
            and outcome.error_code == "NO_ACTIVE_COMMAND"
        ):
            return
        error_code = outcome.error_code or "UNKNOWN"
        raise RuntimeError(
            f"motion cancellation was not confirmed ({error_code}): "
            f"{outcome.message}"
        )
