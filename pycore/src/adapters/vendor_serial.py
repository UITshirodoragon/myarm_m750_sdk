"""Vendor serial adapter for the coordinate-firmware ``MyArmMControl`` API."""

from __future__ import annotations

import importlib
import importlib.metadata
import logging
import math
import threading
import time
from typing import Any, Callable, Optional, Tuple

from myarm_m750_core.adapters.joint_mapping import JointMapper
from myarm_m750_core.domain.errors import (
    HardwareConnectionError,
    HardwareTimeoutError,
    InvalidDriverStateError,
    ProtocolError,
)
from myarm_m750_core.domain.models import (
    AdapterCapabilities,
    CapabilityState,
    CommandContext,
    CommandResult,
    HardwareIdentity,
    HardwareStatus,
    JointState,
    JointTarget,
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
            errors.append(f"{module_name}: {error}")
    raise HardwareConnectionError(
        "Could not import pymycobot MyArmMControl. "
        "Install the vendor package in the runtime environment. "
        f"Attempts: {'; '.join(errors)}"
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
        expected_model: str = "MyArm M750",
        expected_firmware_version: str = "",
        mapping_fingerprint: str = "",
        capability_verification_reference: str = "",
        verified_capabilities: Optional[AdapterCapabilities] = None,
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
        self._expected_model = str(expected_model)
        self._expected_firmware_version = str(expected_firmware_version)
        actual_mapping_fingerprint = self._mapper.contract_fingerprint
        expected_mapping_fingerprint = str(mapping_fingerprint)
        if (
            expected_mapping_fingerprint
            and expected_mapping_fingerprint != actual_mapping_fingerprint
        ):
            raise ValueError(
                "Expected mapping fingerprint does not match the JointMapper contract."
            )
        self._mapping_fingerprint = actual_mapping_fingerprint
        self._declared_capabilities = verified_capabilities or AdapterCapabilities(
            stop=CapabilityState.UNVERIFIED
        )
        verification_reference = str(capability_verification_reference).strip()
        declared_states = (
            self._declared_capabilities.stop,
            self._declared_capabilities.pause,
            self._declared_capabilities.resume,
            self._declared_capabilities.power_control,
        )
        if (
            any(state is CapabilityState.SUPPORTED for state in declared_states)
            and not verification_reference
        ):
            raise ValueError(
                "capability_verification_reference is required when a capability "
                "is declared SUPPORTED."
            )
        self._capability_verification_reference = (
            verification_reference or "none://no-supported-capabilities"
        )
        self._observed_capabilities = self._pre_probe_capabilities(
            self._declared_capabilities
        )
        self._debug = bool(debug)
        self._vendor_factory = vendor_factory
        self._vendor: Optional[Any] = None
        self._sequence = 0
        self._protocol_error_count = 0
        self._timeout_count = 0
        self._retry_count = 0
        self._identity: Optional[HardwareIdentity] = None
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
                    f"Failed to open MyArm M750 serial port {self._port}: {error}"
                ) from error
            self._identity = None
            self._observed_capabilities = self._pre_probe_capabilities(
                self._declared_capabilities
            )
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
            self._identity = None
            self._observed_capabilities = self._pre_probe_capabilities(
                self._declared_capabilities
            )
            if vendor is None:
                return
            close_method = getattr(vendor, "close", None)
            if callable(close_method):
                close_method()
            else:
                # WORKAROUND(pymycobot==4.0.5): the vendor class has no public
                # close() in this pinned release. Remove when its public
                # lifecycle API is available and covered by conformance tests.
                try:
                    vendor_version = importlib.metadata.version("pymycobot")
                except importlib.metadata.PackageNotFoundError:
                    vendor_version = ""
                if vendor_version == "4.0.5":
                    serial_port = getattr(vendor, "_serial_port", None)
                    serial_close = getattr(serial_port, "close", None)
                    if callable(serial_close):
                        serial_close()
            _LOGGER.info("vendor_serial_disconnected", extra={"port": self._port})

    def _require_vendor(self) -> Any:
        if self._vendor is None:
            raise InvalidDriverStateError("Vendor serial adapter is disconnected.")
        return self._vendor

    def _call_with_retry(
        self,
        operation_name: str,
        operation: Callable[[], Any],
        context: CommandContext,
    ) -> Any:
        attempts = self._max_retries + 1
        last_error: Optional[BaseException] = None
        for attempt_index in range(attempts):
            if time.monotonic() >= context.deadline_monotonic_s:
                self._timeout_count += 1
                raise HardwareTimeoutError(
                    f"Firmware operation '{operation_name}' exceeded deadline "
                    f"for {context.command_id}."
                )
            try:
                response = operation()
                if time.monotonic() > context.deadline_monotonic_s:
                    self._timeout_count += 1
                    raise HardwareTimeoutError(
                        f"Firmware operation '{operation_name}' completed "
                        "after its deadline."
                    )
                if response == -1 or response is None:
                    raise ProtocolError(
                        f"Firmware operation '{operation_name}' returned {response!r}."
                    )
                return response
            except ProtocolError as error:
                self._protocol_error_count += 1
                last_error = error
            except TimeoutError as error:
                self._timeout_count += 1
                last_error = error
            except HardwareTimeoutError:
                raise
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
                    "command_id": context.command_id,
                    "point_index": context.trajectory_point_index,
                },
            )
            if attempt_index + 1 < attempts:
                self._retry_count += 1
                remaining_s = context.deadline_monotonic_s - time.monotonic()
                if remaining_s <= 0.0:
                    continue
                time.sleep(min(self._retry_delay_s, remaining_s))
        raise ProtocolError(
            f"Firmware operation '{operation_name}' failed after "
            f"{attempts} attempt(s): {last_error}"
        ) from last_error

    @staticmethod
    def _validate_firmware_angles(
        response: Any,
        expected_joint_count: int,
    ) -> Tuple[float, ...]:
        if (
            not isinstance(response, (list, tuple))
            or len(response) != expected_joint_count
        ):
            raise ProtocolError(
                "get_angles() must return exactly "
                f"{expected_joint_count} canonical joints: {response!r}"
            )
        try:
            angles = tuple(float(value) for value in response)
        except (TypeError, ValueError) as error:
            raise ProtocolError(
                f"get_angles() returned non-numeric data: {response!r}"
            ) from error
        if not all(math.isfinite(value) for value in angles):
            raise ProtocolError(
                f"get_angles() returned non-finite data: {response!r}"
            )
        return angles

    def read_joint_state(self, context: CommandContext) -> JointState:
        with self._lock:
            vendor = self._require_vendor()
            response = self._call_with_retry("get_angles", vendor.get_angles, context)
            firmware_position_deg = self._validate_firmware_angles(
                response,
                self._mapper.joint_count,
            )
            core_position_rad = self._mapper.firmware_deg_to_core_rad(firmware_position_deg)
            self._sequence += 1
            return JointState(
                position_rad=core_position_rad,
                sample_wall_time_s=time.time(),
                received_monotonic_s=time.monotonic(),
                source="vendor_serial",
                sequence=self._sequence,
            )

    def write_joint_target(
        self, target: JointTarget, context: CommandContext
    ) -> CommandResult:
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
                context,
            )
            _LOGGER.info(
                "joint_target_sent",
                extra={
                    "core_position_rad": list(target.position_rad),
                    "firmware_position_deg": list(firmware_position_deg),
                    "firmware_speed": self._firmware_speed,
                    "command_id": context.command_id,
                    "point_index": context.trajectory_point_index,
                },
            )
            return CommandResult.success(
                "Joint target acknowledged by firmware.",
                command_id=context.command_id,
            )

    def _command(self, method_name: str, context: CommandContext) -> CommandResult:
        with self._lock:
            vendor = self._require_vendor()
            method = getattr(vendor, method_name, None)
            if not callable(method):
                return CommandResult.rejected(
                    f"Vendor SDK does not expose {method_name}().",
                    "CAPABILITY_NOT_SUPPORTED",
                    command_id=context.command_id,
                )
            self._call_with_retry(method_name, method, context)
            return CommandResult.success(
                f"Firmware command {method_name} acknowledged.",
                command_id=context.command_id,
            )

    def stop(self, context: CommandContext) -> CommandResult:
        stop_state = self._observed_capabilities.stop
        if stop_state is CapabilityState.UNSUPPORTED:
            return CommandResult.rejected(
                "Firmware stop is explicitly unsupported.",
                "CAPABILITY_NOT_SUPPORTED",
                command_id=context.command_id,
            )
        if stop_state is not CapabilityState.SUPPORTED:
            return CommandResult.rejected(
                "Firmware stop semantics are not verified.",
                "CAPABILITY_NOT_VERIFIED",
                command_id=context.command_id,
            )
        return self._command("stop", context)

    def capabilities(self) -> AdapterCapabilities:
        return self._observed_capabilities

    def read_hardware_status(self) -> HardwareStatus:
        return HardwareStatus(
            connected=self._vendor is not None,
            state="idle" if self._vendor is not None else "disconnected",
            message="Blocking MyArmMControl serial adapter.",
            protocol_error_count=self._protocol_error_count,
            timeout_count=self._timeout_count,
            retry_count=self._retry_count,
            identity=self._identity,
        )

    def probe_identity(self, context: CommandContext) -> HardwareIdentity:
        """Read model/version identity; never issue a motion command.

        The mapping fingerprint is derived from the immutable local
        ``JointMapper`` contract. It prevents configuration echo/mismatch but
        cannot certify physical joint direction or offset; that remains a
        robot hardware calibration gate.
        """
        with self._lock:
            vendor = self._require_vendor()
            version_method = getattr(vendor, "get_system_version", None)
            if not callable(version_method):
                version_method = getattr(vendor, "get_basic_version", None)
            if not callable(version_method):
                raise ProtocolError(
                    "Pinned vendor API exposes no readable firmware identity."
                )
            observed_version = str(
                self._call_with_retry("get_system_version", version_method, context)
            )
            if observed_version != self._expected_firmware_version:
                raise ProtocolError(
                    "Firmware version mismatch: expected "
                    f"{self._expected_firmware_version}, observed {observed_version}."
                )
            model_method = getattr(vendor, "get_robot_type", None)
            if not callable(model_method):
                model_method = getattr(vendor, "get_robot_model", None)
            if not callable(model_method):
                raise ProtocolError(
                    "Pinned vendor API exposes no readable robot model identity."
                )
            observed_model = str(
                self._call_with_retry("get_robot_type", model_method, context)
            )
            if observed_model != self._expected_model:
                raise ProtocolError(
                    f"Robot model mismatch: expected {self._expected_model}, "
                    f"observed {observed_model}."
                )
            identity = HardwareIdentity(
                adapter="vendor_serial",
                model=observed_model,
                firmware_version=observed_version,
                serial_resource=self._port,
                mapping_fingerprint=self._mapping_fingerprint,
                capability_verification_reference=(
                    self._capability_verification_reference
                ),
            )
            self._observed_capabilities = self._verify_declared_capabilities(
                vendor
            )
            self._identity = identity
            return identity

    @staticmethod
    def _pre_probe_capabilities(
        declared: AdapterCapabilities,
    ) -> AdapterCapabilities:
        """Downgrade claims until identity and method presence are observed."""

        def before_probe(state: CapabilityState) -> CapabilityState:
            return (
                CapabilityState.UNSUPPORTED
                if state is CapabilityState.UNSUPPORTED
                else CapabilityState.UNVERIFIED
            )

        return AdapterCapabilities(
            stop=before_probe(declared.stop),
            pause=before_probe(declared.pause),
            resume=before_probe(declared.resume),
            power_control=before_probe(declared.power_control),
        )

    def _verify_declared_capabilities(
        self,
        vendor: Any,
    ) -> AdapterCapabilities:
        """Confirm declared supported methods after the identity probe.

        Method presence does not certify firmware semantics by itself.  A
        deployment may declare ``SUPPORTED`` only after its firmware/version
        evidence and stop behavior have passed the hardware checklist.
        """
        method_contracts = {
            "stop": ("stop",),
            "pause": ("pause",),
            "resume": ("resume",),
            "power_control": ("power_on", "power_off"),
        }
        states = {
            "stop": self._declared_capabilities.stop,
            "pause": self._declared_capabilities.pause,
            "resume": self._declared_capabilities.resume,
            "power_control": self._declared_capabilities.power_control,
        }
        for capability_name, state in states.items():
            if state is not CapabilityState.SUPPORTED:
                continue
            missing_methods = [
                method_name
                for method_name in method_contracts[capability_name]
                if not callable(getattr(vendor, method_name, None))
            ]
            if missing_methods:
                raise ProtocolError(
                    f"Capability '{capability_name}' is declared supported, "
                    f"but vendor methods are missing: {missing_methods}."
                )
        return self._declared_capabilities
