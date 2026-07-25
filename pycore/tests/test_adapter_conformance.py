import json
import logging

import pytest
from myarm_m750_core.adapters import (
    JointMapper,
    MockRobotAdapter,
    ReplayRobotAdapter,
    VendorSerialRobotAdapter,
)
from myarm_m750_core.domain.errors import (
    ConfigurationError,
    HardwareTimeoutError,
    InvalidDriverStateError,
    ProtocolError,
)
from myarm_m750_core.domain.models import (
    AdapterCapabilities,
    CapabilityState,
    CommandContext,
    JointTarget,
)


class ConformantVendor:
    def __init__(self, *args, **kwargs):
        del args, kwargs
        self.closed = False

    def get_angles(self):
        return [0.0, 10.0, -10.0, 0.0, 0.0, 0.0]

    def get_system_version(self):
        return "fake-1"

    def get_robot_type(self):
        return "MyArm M750"

    def write_angles(self, _angles, _speed):
        return 1

    def stop(self):
        return 1

    def close(self):
        self.closed = True


def _vendor(sdk_config, factory=ConformantVendor, retries=0):
    mapper = JointMapper(
        sdk_config.robot.joint_names,
        sdk_config.robot.joint_mapping,
    )
    return VendorSerialRobotAdapter(
        port="/dev/serial/by-id/fake",
        baudrate=1_000_000,
        timeout_s=0.1,
        firmware_speed=30,
        mapper=mapper,
        max_retries=retries,
        retry_delay_s=0.0,
        expected_firmware_version="fake-1",
        mapping_fingerprint=mapper.contract_fingerprint,
        capability_verification_reference="test://firmware-fake-1/stop",
        verified_capabilities=AdapterCapabilities(stop=CapabilityState.SUPPORTED),
        vendor_factory=factory,
    )


def test_mock_lifecycle_is_idempotent_and_deadline_is_enforced() -> None:
    adapter = MockRobotAdapter([0.0] * 6)
    adapter.connect()
    adapter.connect()
    assert adapter.probe_identity(CommandContext.with_timeout(1.0)).adapter == "mock"
    expired = CommandContext("expired", deadline_monotonic_s=0.0)
    with pytest.raises(HardwareTimeoutError):
        adapter.read_joint_state(expired)
    adapter.disconnect()
    adapter.disconnect()
    with pytest.raises(InvalidDriverStateError):
        adapter.read_joint_state(CommandContext.with_timeout(1.0))


def test_replay_lifecycle_and_read_only_contract(tmp_path) -> None:
    replay_path = tmp_path / "states.jsonl"
    replay_path.write_text(
        json.dumps({"position_rad": [0.0] * 6}) + "\n",
        encoding="utf-8",
    )
    adapter = ReplayRobotAdapter(str(replay_path), loop=True)
    adapter.connect()
    adapter.connect()
    context = CommandContext.with_timeout(1.0)
    assert adapter.read_joint_state(context).sequence == 0
    assert (
        adapter.write_joint_target(JointTarget((0.0,) * 6), context).error_code
        == "REPLAY_READ_ONLY"
    )
    assert adapter.stop(context).succeeded
    assert adapter.capabilities().stop is CapabilityState.SUPPORTED
    assert adapter.read_hardware_status().state == "replay"
    assert adapter.probe_identity(context).adapter == "replay"
    adapter.disconnect()
    adapter.disconnect()
    assert adapter.read_hardware_status().state == "disconnected"
    with pytest.raises(InvalidDriverStateError):
        adapter.read_joint_state(CommandContext.with_timeout(1.0))


@pytest.mark.parametrize(
    "contents, expected_message",
    [
        ("", "no joint samples"),
        ("not-json\n", "Invalid replay sample"),
        (json.dumps({"sequence": 1}) + "\n", "Invalid replay sample"),
    ],
)
def test_replay_rejects_missing_or_malformed_samples(
    tmp_path, contents, expected_message
) -> None:
    replay_path = tmp_path / "invalid.jsonl"
    replay_path.write_text(contents, encoding="utf-8")
    with pytest.raises(ConfigurationError, match=expected_message):
        ReplayRobotAdapter(str(replay_path)).connect()

    with pytest.raises(ConfigurationError, match="does not exist"):
        ReplayRobotAdapter(str(tmp_path / "missing.jsonl")).connect()


def test_replay_non_looping_cursor_and_deadlines(tmp_path) -> None:
    replay_path = tmp_path / "states.jsonl"
    replay_path.write_text(
        "\n".join(
            [
                json.dumps({"position_rad": [0.0] * 6, "sequence": 10}),
                json.dumps({"position_rad": [0.1] * 6, "sequence": 11}),
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    adapter = ReplayRobotAdapter(str(replay_path))
    adapter.connect()
    assert adapter.read_joint_state(CommandContext.with_timeout(1.0)).sequence == 10
    assert adapter.read_joint_state(CommandContext.with_timeout(1.0)).sequence == 11
    assert adapter.read_joint_state(CommandContext.with_timeout(1.0)).sequence == 11
    expired = CommandContext("expired", deadline_monotonic_s=0.0)
    with pytest.raises(HardwareTimeoutError):
        adapter.read_joint_state(expired)
    with pytest.raises(HardwareTimeoutError):
        adapter.stop(expired)


def test_vendor_fake_probe_disconnect_and_stop(sdk_config) -> None:
    adapter = _vendor(sdk_config)
    adapter.connect()
    adapter.connect()
    context = CommandContext.with_timeout(1.0)
    identity = adapter.probe_identity(context)
    expected_mapping_fingerprint = JointMapper(
        sdk_config.robot.joint_names,
        sdk_config.robot.joint_mapping,
    ).contract_fingerprint
    assert identity.mapping_fingerprint == expected_mapping_fingerprint
    assert (
        identity.capability_verification_reference
        == "test://firmware-fake-1/stop"
    )
    assert adapter.stop(context).succeeded
    adapter.disconnect()
    adapter.disconnect()
    with pytest.raises(InvalidDriverStateError):
        adapter.read_joint_state(CommandContext.with_timeout(1.0))


def test_vendor_malformed_reply_and_stop_failure_are_typed(sdk_config) -> None:
    class BrokenVendor(ConformantVendor):
        def get_angles(self):
            return ["bad"]

        def stop(self):
            return -1

    adapter = _vendor(sdk_config, BrokenVendor)
    adapter.connect()
    adapter.probe_identity(CommandContext.with_timeout(1.0))
    with pytest.raises(ProtocolError):
        adapter.read_joint_state(CommandContext.with_timeout(1.0))
    with pytest.raises(ProtocolError):
        adapter.stop(CommandContext.with_timeout(1.0))


def test_vendor_retry_log_carries_command_context(sdk_config, caplog) -> None:
    class RetryVendor(ConformantVendor):
        calls = 0

        def get_angles(self):
            self.calls += 1
            if self.calls == 1:
                return -1
            return super().get_angles()

    adapter = _vendor(sdk_config, RetryVendor, retries=1)
    adapter.connect()
    with caplog.at_level(logging.WARNING):
        state = adapter.read_joint_state(
            CommandContext.with_timeout(1.0, command_id="trace-123")
        )
    assert state.position_rad == (0.0,) * 6
    records = [
        record for record in caplog.records if record.msg == "vendor_operation_retry"
    ]
    assert records
    assert records[0].command_id == "trace-123"
