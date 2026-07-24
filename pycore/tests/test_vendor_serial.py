import pytest

from myarm_m750_core.adapters import JointMapper, VendorSerialRobotAdapter
from myarm_m750_core.domain.errors import ProtocolError
from myarm_m750_core.domain.models import JointTarget


class FakeVendor:
    def __init__(self, *args, **kwargs):
        del args, kwargs
        self.last_angles = None

    def get_angles(self):
        return [0.0, 10.0, -10.0, 0.0, 0.0, 0.0]

    def write_angles(self, angles, speed):
        self.last_angles = (angles, speed)
        return 1

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
        vendor_factory=FakeVendor,
    )
    adapter.connect()
    assert adapter.read_state().position_rad == (0.0,) * 6
    assert adapter.write_joint_target(JointTarget((0.0,) * 6)).succeeded


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
        adapter.read_state()


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
        adapter.read_state()
    assert adapter.status().timeout_count == 1
