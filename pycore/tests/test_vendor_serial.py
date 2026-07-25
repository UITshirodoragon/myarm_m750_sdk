import pytest
from myarm_m750_core.adapters import JointMapper, VendorSerialRobotAdapter
from myarm_m750_core.adapters import vendor_serial as vendor_module
from myarm_m750_core.domain.errors import (
    HardwareConnectionError,
    HardwareTimeoutError,
    ProtocolError,
)
from myarm_m750_core.domain.models import (
    AdapterCapabilities,
    CapabilityState,
    CommandContext,
    JointTarget,
)


class FakeVendor:
    def __init__(self, *args, **kwargs):
        del args, kwargs
        self.last_angles = None

    def get_angles(self):
        return [0.0, 10.0, -10.0, 0.0, 0.0, 0.0]

    def write_angles(self, angles, speed):
        self.last_angles = (angles, speed)
        return 1

    def get_system_version(self):
        return "fake-1"

    def get_robot_type(self):
        return "MyArm M750"

    def stop(self):
        return 1

    def pause(self):
        return 1

    def resume(self):
        return 1


class ErrorVendor(FakeVendor):
    def get_angles(self):
        return -1


def test_vendor_adapter_converts_mapping_and_ack(sdk_config) -> None:
    mapper = JointMapper(sdk_config.robot.joint_names, sdk_config.robot.joint_mapping)
    adapter = VendorSerialRobotAdapter(
        port="fake",
        baudrate=1_000_000,
        timeout_s=0.1,
        firmware_speed=30,
        mapper=mapper,
        max_retries=0,
        expected_firmware_version="fake-1",
        mapping_fingerprint=mapper.contract_fingerprint,
        capability_verification_reference="test://firmware-fake-1/stop",
        verified_capabilities=AdapterCapabilities(stop=CapabilityState.SUPPORTED),
        vendor_factory=FakeVendor,
    )
    adapter.connect()
    context = CommandContext.with_timeout(1.0)
    assert adapter.capabilities().stop is CapabilityState.UNVERIFIED
    assert adapter.read_joint_state(context).position_rad == (0.0,) * 6
    assert adapter.write_joint_target(JointTarget((0.0,) * 6), context).succeeded
    assert adapter.probe_identity(context).firmware_version == "fake-1"
    assert adapter.capabilities().stop is CapabilityState.SUPPORTED


def test_vendor_minus_one_becomes_protocol_error(sdk_config) -> None:
    mapper = JointMapper(sdk_config.robot.joint_names, sdk_config.robot.joint_mapping)
    adapter = VendorSerialRobotAdapter(
        port="fake",
        baudrate=1_000_000,
        timeout_s=0.1,
        firmware_speed=30,
        mapper=mapper,
        max_retries=0,
        vendor_factory=ErrorVendor,
    )
    adapter.connect()
    with pytest.raises(ProtocolError):
        adapter.read_joint_state(CommandContext.with_timeout(1.0))


class TimeoutVendor(FakeVendor):
    def get_angles(self):
        raise TimeoutError("serial read timed out")


def test_vendor_timeout_is_counted_and_reported(sdk_config) -> None:
    mapper = JointMapper(sdk_config.robot.joint_names, sdk_config.robot.joint_mapping)
    adapter = VendorSerialRobotAdapter(
        port="fake",
        baudrate=1_000_000,
        timeout_s=0.1,
        firmware_speed=30,
        mapper=mapper,
        max_retries=0,
        vendor_factory=TimeoutVendor,
    )
    adapter.connect()
    with pytest.raises(ProtocolError):
        adapter.read_joint_state(CommandContext.with_timeout(1.0))
    assert adapter.read_hardware_status().timeout_count == 1


def _adapter(sdk_config, factory=FakeVendor, **overrides):
    mapper = JointMapper(sdk_config.robot.joint_names, sdk_config.robot.joint_mapping)
    arguments = {
        "port": "/dev/serial/by-id/fake",
        "baudrate": 1_000_000,
        "timeout_s": 0.1,
        "firmware_speed": 30,
        "mapper": mapper,
        "max_retries": 0,
        "expected_firmware_version": "fake-1",
        "mapping_fingerprint": mapper.contract_fingerprint,
        "capability_verification_reference": "test://firmware-fake-1/stop",
        "verified_capabilities": AdapterCapabilities(stop=CapabilityState.SUPPORTED),
        "vendor_factory": factory,
    }
    arguments.update(overrides)
    return VendorSerialRobotAdapter(**arguments)


def test_vendor_loader_and_constructor_fail_before_io(sdk_config, monkeypatch) -> None:
    def fail_import(name):
        raise ImportError(name)

    monkeypatch.setattr(vendor_module.importlib, "import_module", fail_import)
    with pytest.raises(HardwareConnectionError, match="Attempts"):
        vendor_module._load_vendor_class()

    with pytest.raises(ValueError, match="firmware_speed"):
        _adapter(sdk_config, firmware_speed=0)
    with pytest.raises(ValueError, match="max_retries"):
        _adapter(sdk_config, max_retries=-1)
    with pytest.raises(ValueError, match="mapping fingerprint"):
        _adapter(sdk_config, mapping_fingerprint="0" * 64)
    with pytest.raises(ValueError, match="capability_verification_reference"):
        _adapter(sdk_config, capability_verification_reference="")

    def fail_factory(*args, **kwargs):
        del args, kwargs
        raise OSError("port busy")

    with pytest.raises(HardwareConnectionError, match="port busy"):
        _adapter(sdk_config, factory=fail_factory).connect()


def test_vendor_private_close_workaround_is_version_guarded(
    sdk_config, monkeypatch
) -> None:
    class SerialPort:
        closed = False

        def close(self):
            self.closed = True

    class VendorWithoutClose(FakeVendor):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self._serial_port = SerialPort()

    adapter = _adapter(sdk_config, factory=VendorWithoutClose)
    adapter.connect()
    vendor = adapter._vendor
    monkeypatch.setattr(vendor_module.importlib.metadata, "version", lambda _name: "4.0.5")
    adapter.disconnect()
    assert vendor._serial_port.closed

    adapter = _adapter(sdk_config, factory=VendorWithoutClose)
    adapter.connect()
    vendor = adapter._vendor

    def missing_distribution(_name):
        raise vendor_module.importlib.metadata.PackageNotFoundError

    monkeypatch.setattr(
        vendor_module.importlib.metadata,
        "version",
        missing_distribution,
    )
    adapter.disconnect()
    assert not vendor._serial_port.closed


def test_vendor_deadline_capabilities_and_non_numeric_reply(sdk_config) -> None:
    adapter = _adapter(sdk_config)
    adapter.connect()
    with pytest.raises(HardwareTimeoutError):
        adapter.read_joint_state(CommandContext("expired", deadline_monotonic_s=0.0))

    unverified = _adapter(
        sdk_config,
        verified_capabilities=AdapterCapabilities(stop=CapabilityState.UNVERIFIED),
    )
    unverified.connect()
    result = unverified.stop(CommandContext.with_timeout(1.0))
    assert not result.succeeded
    assert result.error_code == "CAPABILITY_NOT_VERIFIED"
    assert unverified.capabilities().stop is CapabilityState.UNVERIFIED

    class NonNumericVendor(FakeVendor):
        def get_angles(self):
            return [0.0, 0.0, 0.0, 0.0, 0.0, object()]

    invalid = _adapter(sdk_config, factory=NonNumericVendor)
    invalid.connect()
    with pytest.raises(ProtocolError, match="non-numeric"):
        invalid.read_joint_state(CommandContext.with_timeout(1.0))

    class NonFiniteVendor(FakeVendor):
        def get_angles(self):
            return [0.0, 0.0, float("nan"), 0.0, 0.0, 0.0]

    nonfinite = _adapter(sdk_config, factory=NonFiniteVendor)
    nonfinite.connect()
    with pytest.raises(ProtocolError, match="non-finite"):
        nonfinite.read_joint_state(CommandContext.with_timeout(1.0))

    class ExtraJointVendor(FakeVendor):
        def get_angles(self):
            return [0.0] * 7

    extra = _adapter(sdk_config, factory=ExtraJointVendor)
    extra.connect()
    with pytest.raises(ProtocolError, match="exactly 6"):
        extra.read_joint_state(CommandContext.with_timeout(1.0))


def test_vendor_generic_timeout_and_missing_stop_are_typed(sdk_config) -> None:
    class SerialTimeout(Exception):
        pass

    class GenericTimeoutVendor(FakeVendor):
        def get_angles(self):
            raise SerialTimeout("vendor-specific timeout")

    adapter = _adapter(sdk_config, factory=GenericTimeoutVendor)
    adapter.connect()
    with pytest.raises(ProtocolError):
        adapter.read_joint_state(CommandContext.with_timeout(1.0))
    assert adapter.read_hardware_status().timeout_count == 1

    class NoStopVendor(FakeVendor):
        stop = None

    adapter = _adapter(sdk_config, factory=NoStopVendor)
    adapter.connect()
    assert (
        adapter.stop(CommandContext.with_timeout(1.0)).error_code
        == "CAPABILITY_NOT_VERIFIED"
    )
    with pytest.raises(ProtocolError, match="methods are missing"):
        adapter.probe_identity(CommandContext.with_timeout(1.0))

    unsupported = _adapter(
        sdk_config,
        factory=NoStopVendor,
        verified_capabilities=AdapterCapabilities(
            stop=CapabilityState.UNSUPPORTED
        ),
    )
    unsupported.connect()
    unsupported.probe_identity(CommandContext.with_timeout(1.0))
    assert unsupported.capabilities().stop is CapabilityState.UNSUPPORTED
    assert (
        unsupported.stop(CommandContext.with_timeout(1.0)).error_code
        == "CAPABILITY_NOT_SUPPORTED"
    )


@pytest.mark.parametrize(
    "vendor_type, expected_message",
    [
        (
            type(
                "MissingVersionVendor",
                (FakeVendor,),
                {"get_system_version": None, "get_basic_version": None},
            ),
            "firmware identity",
        ),
        (
            type(
                "WrongVersionVendor",
                (FakeVendor,),
                {"get_system_version": lambda self: "wrong"},
            ),
            "Firmware version mismatch",
        ),
        (
            type(
                "MissingModelVendor",
                (FakeVendor,),
                {"get_robot_type": None, "get_robot_model": None},
            ),
            "robot model identity",
        ),
        (
            type(
                "WrongModelVendor",
                (FakeVendor,),
                {"get_robot_type": lambda self: "another-arm"},
            ),
            "Robot model mismatch",
        ),
    ],
)
def test_vendor_probe_rejects_unverified_identity(
    sdk_config, vendor_type, expected_message
) -> None:
    adapter = _adapter(sdk_config, factory=vendor_type)
    adapter.connect()
    with pytest.raises(ProtocolError, match=expected_message):
        adapter.probe_identity(CommandContext.with_timeout(1.0))


def test_vendor_probe_supports_pinned_fallback_identity_methods(sdk_config) -> None:
    class FallbackIdentityVendor(FakeVendor):
        get_system_version = None
        get_robot_type = None

        def get_basic_version(self):
            return "fake-1"

        def get_robot_model(self):
            return "MyArm M750"

    adapter = _adapter(sdk_config, factory=FallbackIdentityVendor)
    adapter.connect()
    identity = adapter.probe_identity(CommandContext.with_timeout(1.0))
    assert identity.model == "MyArm M750"
    expected_mapping_fingerprint = JointMapper(
        sdk_config.robot.joint_names,
        sdk_config.robot.joint_mapping,
    ).contract_fingerprint
    assert identity.mapping_fingerprint == expected_mapping_fingerprint
    assert (
        identity.capability_verification_reference
        == "test://firmware-fake-1/stop"
    )
