"""Vendor serial adapter for the coordinate-firmware ``MyArmMControl`` API."""

from __future__ import annotations

import importlib
import logging
import threading
import time
from typing import Any, Callable, Optional, Sequence, Tuple

from myarm_m750_core.adapters.joint_mapping import JointMapper
from myarm_m750_core.domain.errors import (
    HardwareConnectionError,
    InvalidDriverStateError,
    ProtocolError,
)
from myarm_m750_core.domain.models import (
    CommandResult,
    HardwareStatus,
    JointState,
    JointTarget,
    RobotCapabilities,
)
from myarm_m750_core.ports.robot_hardware import RobotHardwarePort

_LOGGER = logging.getLogger(__name__)


def _load_vendor_class() -> Any:
    """Load ``MyArmMControl`` without importing the vendor SDK in core domain."""
    candidates = (
        ("pymycobot.myarmm_control", "MyArmMControl"),
        ("pymycobot", "MyArmMControl"),
    )
    errors = []
    for module_name, class_name in candidates:
        try:
            module = importlib.import_module(module_name)
            return getattr(module, class_name)
        except (ImportError, AttributeError) as error:
            errors.append("{0}: {1}".format(module_name, error))
    raise HardwareConnectionError(
        "Could not import pymycobot MyArmMControl. "
        "Install the vendor package in the runtime environment. Attempts: {0}".format(
            "; ".join(errors)
        )
    )


class VendorSerialRobotAdapter(RobotHardwarePort):
    """Translate canonical joint commands to the blocking vendor serial API.

    ``MyArmMControl`` performs write-then-read for both GET and SET commands.
    All vendor calls are serialized with one lock and retries are bounded. A
    firmware ``-1`` is converted to ``ProtocolError`` instead of escaping into
    application code.
    """

    def __init__(
        self,
        port: str,
        baudrate: int,
        timeout_s: float,
        firmware_speed: int,
        mapper: JointMapper,
        max_retries: int = 1,
        retry_delay_s: float = 0.05,
        debug: bool = False,
        vendor_factory: Optional[Callable[..., Any]] = None,
    ) -> None:
        if not 1 <= int(firmware_speed) <= 100:
            raise ValueError("firmware_speed must be in the range 1..100.")
        if int(max_retries) < 0:
            raise ValueError("max_retries must be non-negative.")
        self._port = str(port)
        self._baudrate = int(baudrate)
        self._timeout_s = float(timeout_s)
        self._firmware_speed = int(firmware_speed)
        self._mapper = mapper
        self._max_retries = int(max_retries)
        self._retry_delay_s = float(retry_delay_s)
        self._debug = bool(debug)
        self._vendor_factory = vendor_factory
        self._vendor: Optional[Any] = None
        self._sequence = 0
        self._protocol_error_count = 0
        self._timeout_count = 0
        self._lock = threading.RLock()

    def connect(self) -> None:
        with self._lock:
            if self._vendor is not None:
                return
            factory = self._vendor_factory or _load_vendor_class()
            try:
                self._vendor = factory(
                    self._port,
                    baudrate=self._baudrate,
                    timeout=self._timeout_s,
                    debug=self._debug,
                )
            except Exception as error:
                raise HardwareConnectionError(
                    "Failed to open MyArm M750 serial port {0}: {1}".format(
                        self._port, error
                    )
                ) from error
            _LOGGER.info(
                "vendor_serial_connected",
                extra={
                    "port": self._port,
                    "baudrate": self._baudrate,
                    "timeout_s": self._timeout_s,
                },
            )

    def disconnect(self) -> None:
        with self._lock:
            vendor = self._vendor
            self._vendor = None
            if vendor is None:
                return
            close_method = getattr(vendor, "close", None)
            if callable(close_method):
                close_method()
            else:
                serial_port = getattr(vendor, "_serial_port", None)
                serial_close = getattr(serial_port, "close", None)
                if callable(serial_close):
                    serial_close()
            _LOGGER.info("vendor_serial_disconnected", extra={"port": self._port})

    def _require_vendor(self) -> Any:
        if self._vendor is None:
            raise InvalidDriverStateError("Vendor serial adapter is disconnected.")
        return self._vendor

    def _call_with_retry(self, operation_name: str, operation: Callable[[], Any]) -> Any:
        attempts = self._max_retries + 1
        last_error: Optional[BaseException] = None
        for attempt_index in range(attempts):
            try:
                response = operation()
                if response == -1 or response is None:
                    raise ProtocolError(
                        "Firmware operation '{0}' returned {1!r}.".format(
                            operation_name, response
                        )
                    )
                return response
            except ProtocolError as error:
                self._protocol_error_count += 1
                last_error = error
            except TimeoutError as error:
                self._timeout_count += 1
                last_error = error
            except Exception as error:
                # pyserial timeout exceptions are not guaranteed to inherit the
                # built-in TimeoutError across vendor SDK versions. Preserve a
                # diagnostic count using the exception type name as a fallback.
                if "timeout" in type(error).__name__.lower():
                    self._timeout_count += 1
                last_error = error
            _LOGGER.warning(
                "vendor_operation_retry",
                extra={
                    "operation": operation_name,
                    "attempt": attempt_index + 1,
                    "attempts": attempts,
                    "error": repr(last_error),
                },
            )
            if attempt_index + 1 < attempts:
                time.sleep(self._retry_delay_s)
        raise ProtocolError(
            "Firmware operation '{0}' failed after {1} attempt(s): {2}".format(
                operation_name, attempts, last_error
            )
        ) from last_error

    @staticmethod
    def _validate_firmware_angles(response: Any) -> Tuple[float, ...]:
        if not isinstance(response, (list, tuple)) or len(response) < 6:
            raise ProtocolError(
                "get_angles() returned invalid data: {0!r}".format(response)
            )
        try:
            return tuple(float(value) for value in response[:6])
        except (TypeError, ValueError) as error:
            raise ProtocolError(
                "get_angles() returned non-numeric data: {0!r}".format(response)
            ) from error

    def read_state(self) -> JointState:
        with self._lock:
            vendor = self._require_vendor()
            response = self._call_with_retry("get_angles", vendor.get_angles)
            firmware_position_deg = self._validate_firmware_angles(response)
            core_position_rad = self._mapper.firmware_deg_to_core_rad(
                firmware_position_deg
            )
            self._sequence += 1
            return JointState(
                position_rad=core_position_rad,
                timestamp_s=time.time(),
                source="vendor_serial",
                sequence=self._sequence,
            )

    def write_joint_target(self, target: JointTarget) -> CommandResult:
        with self._lock:
            vendor = self._require_vendor()
            firmware_position_deg = self._mapper.core_rad_to_firmware_deg(
                target.position_rad
            )

            # WARNING: MyArmMControl waits for a reply even for SET commands.
            # Keep this call outside ROS timer callbacks that must remain bounded.
            self._call_with_retry(
                "write_angles",
                lambda: vendor.write_angles(
                    list(firmware_position_deg), self._firmware_speed
                ),
            )
            _LOGGER.info(
                "joint_target_sent",
                extra={
                    "core_position_rad": list(target.position_rad),
                    "firmware_position_deg": list(firmware_position_deg),
                    "firmware_speed": self._firmware_speed,
                },
            )
            return CommandResult.success("Joint target acknowledged by firmware.")

    def _command(self, method_name: str) -> CommandResult:
        with self._lock:
            vendor = self._require_vendor()
            method = getattr(vendor, method_name, None)
            if not callable(method):
                return CommandResult.rejected(
                    "Vendor SDK does not expose {0}().".format(method_name),
                    "CAPABILITY_NOT_SUPPORTED",
                )
            self._call_with_retry(method_name, method)
            return CommandResult.success(
                "Firmware command {0} acknowledged.".format(method_name)
            )

    def stop(self) -> CommandResult:
        return self._command("stop")

    def pause(self) -> CommandResult:
        return self._command("pause")

    def resume(self) -> CommandResult:
        return self._command("resume")

    def capabilities(self) -> RobotCapabilities:
        return RobotCapabilities(
            supports_pause=True,
            supports_resume=True,
            supports_stop=True,
            supports_power_control=False,
        )

    def status(self) -> HardwareStatus:
        return HardwareStatus(
            connected=self._vendor is not None,
            state="idle" if self._vendor is not None else "disconnected",
            message="Blocking MyArmMControl serial adapter.",
            protocol_error_count=self._protocol_error_count,
            timeout_count=self._timeout_count,
        )
