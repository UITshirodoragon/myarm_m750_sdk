"""Lifecycle-equivalent state transition tests."""

import unittest
from types import SimpleNamespace

from myarm_m750_driver.contracts import (
    CoreAdapterCapabilities,
    CoreCommandOutcome,
    CoreHardwareIdentity,
    DriverLifecycleState,
)
from myarm_m750_driver.lifecycle_manager import DriverLifecycleManager


class FakeFacade:
    """Deterministic facade for lifecycle tests."""

    adapter_kind = "mock"

    def __init__(self, config_file):
        self.config_file = config_file
        self.configured = False
        self.connected = False
        self.connect_count = 0
        self.close_count = 0
        self.probe_count = 0

    def configure(self):
        self.configured = True

    @property
    def adapter_capabilities(self):
        return CoreAdapterCapabilities(
            stop="supported",
            pause="unsupported",
            resume="unsupported",
            power_control="unsupported",
        )

    def connect(self):
        self.connect_count += 1
        self.connected = True

    def probe_hardware(self):
        self.probe_count += 1
        return CoreHardwareIdentity(
            adapter="mock",
            model="mock",
            firmware_version="mock-1",
            serial_resource="memory://mock",
            mapping_fingerprint="mock-mapping",
            capability_verification_reference="builtin://mock/stop",
        )

    def close(self):
        self.close_count += 1
        self.connected = False

    def cancel_current_command(self):
        return CoreCommandOutcome(
            status="rejected",
            message="no active command",
            command_id="",
            error_code="NO_ACTIVE_COMMAND",
        )

    def recover(self):
        return SimpleNamespace(succeeded=True, message="recovered")


class LifecycleManagerTest(unittest.TestCase):
    """Verify allowed transitions and real-hardware intent matching."""

    def test_configure_activate_deactivate_cleanup(self) -> None:
        manager = DriverLifecycleManager(
            config_file="/tmp/mock.yaml",
            use_real_hardware=False,
            facade_factory=FakeFacade,
        )

        self.assertTrue(manager.configure()[0])
        self.assertIs(manager.state, DriverLifecycleState.INACTIVE)
        self.assertTrue(manager.activate()[0])
        self.assertIs(manager.state, DriverLifecycleState.ACTIVE)
        self.assertEqual(manager.facade.probe_count, 1)
        self.assertTrue(manager.deactivate()[0])
        self.assertIs(manager.state, DriverLifecycleState.INACTIVE)
        self.assertTrue(manager.cleanup()[0])
        self.assertIs(manager.state, DriverLifecycleState.UNCONFIGURED)

    def test_rejects_real_flag_adapter_mismatch(self) -> None:
        manager = DriverLifecycleManager(
            config_file="/tmp/mock.yaml",
            use_real_hardware=True,
            facade_factory=FakeFacade,
        )

        succeeded, message = manager.configure()

        self.assertFalse(succeeded)
        self.assertIn("conflicts", message)
        self.assertIs(manager.state, DriverLifecycleState.FAULT)

    def test_activation_probe_failure_closes_and_faults(self) -> None:
        class ProbeFailureFacade(FakeFacade):
            def probe_hardware(self):
                self.probe_count += 1
                raise RuntimeError("firmware identity mismatch")

        manager = DriverLifecycleManager(
            config_file="/tmp/mock.yaml",
            use_real_hardware=False,
            facade_factory=ProbeFailureFacade,
        )
        self.assertTrue(manager.configure()[0])

        succeeded, message = manager.activate()

        self.assertFalse(succeeded)
        self.assertIn("identity mismatch", message)
        self.assertIs(manager.state, DriverLifecycleState.FAULT)
        self.assertFalse(manager.facade.connected)
        self.assertTrue(manager.cleanup()[0])
        self.assertIs(manager.state, DriverLifecycleState.UNCONFIGURED)

    def test_command_activation_requires_verified_supported_stop(self) -> None:
        class UnverifiedStopFacade(FakeFacade):
            @property
            def adapter_capabilities(self):
                return CoreAdapterCapabilities(
                    stop="unverified",
                    pause="unsupported",
                    resume="unsupported",
                    power_control="unsupported",
                )

        manager = DriverLifecycleManager(
            config_file="/tmp/mock.yaml",
            use_real_hardware=False,
            require_supported_stop=True,
            facade_factory=UnverifiedStopFacade,
        )
        self.assertTrue(manager.configure()[0])

        succeeded, message = manager.activate()

        self.assertFalse(succeeded)
        self.assertIn("SUPPORTED stop", message)
        self.assertIs(manager.state, DriverLifecycleState.FAULT)
        self.assertFalse(manager.facade.connected)

    def test_command_activation_rejects_missing_stop_evidence(self) -> None:
        class MissingEvidenceFacade(FakeFacade):
            def probe_hardware(self):
                self.probe_count += 1
                return CoreHardwareIdentity(
                    adapter="mock",
                    model="mock",
                    firmware_version="mock-1",
                    serial_resource="memory://mock",
                    mapping_fingerprint="mock-mapping",
                    capability_verification_reference="",
                )

        manager = DriverLifecycleManager(
            config_file="/tmp/mock.yaml",
            use_real_hardware=False,
            require_supported_stop=True,
            facade_factory=MissingEvidenceFacade,
        )
        self.assertTrue(manager.configure()[0])

        succeeded, message = manager.activate()

        self.assertFalse(succeeded)
        self.assertIn("verification reference", message)
        self.assertIs(manager.state, DriverLifecycleState.FAULT)

    def test_state_poll_fault_recovery_reconnects_and_reprobes(self) -> None:
        manager = DriverLifecycleManager(
            config_file="/tmp/mock.yaml",
            use_real_hardware=False,
            facade_factory=FakeFacade,
        )
        self.assertTrue(manager.configure()[0])
        self.assertTrue(manager.activate()[0])
        facade = manager.facade
        self.assertIsNotNone(facade)

        manager.record_runtime_fault(RuntimeError("state poll failed"))
        succeeded, message = manager.recover()

        self.assertTrue(succeeded)
        self.assertIn("reconnect", message)
        self.assertIs(manager.state, DriverLifecycleState.ACTIVE)
        self.assertTrue(facade.connected)
        self.assertEqual(facade.connect_count, 2)
        self.assertEqual(facade.close_count, 1)
        self.assertEqual(facade.probe_count, 2)

    def test_deactivation_stop_failure_faults_without_closing(self) -> None:
        class StopFailureFacade(FakeFacade):
            def cancel_current_command(self):
                return CoreCommandOutcome(
                    status="failed",
                    message="adapter stop timed out",
                    command_id="command-7",
                    error_code="STOP_FAILED",
                )

        manager = DriverLifecycleManager(
            config_file="/tmp/mock.yaml",
            use_real_hardware=False,
            facade_factory=StopFailureFacade,
        )
        self.assertTrue(manager.configure()[0])
        self.assertTrue(manager.activate()[0])
        facade = manager.facade
        self.assertIsNotNone(facade)

        succeeded, message = manager.deactivate()

        self.assertFalse(succeeded)
        self.assertIn("STOP_FAILED", message)
        self.assertIs(manager.state, DriverLifecycleState.FAULT)
        self.assertTrue(facade.connected)
        self.assertEqual(facade.close_count, 0)

    def test_shutdown_records_cancel_failure_and_still_closes(self) -> None:
        class StopFailureFacade(FakeFacade):
            def cancel_current_command(self):
                return CoreCommandOutcome(
                    status="failed",
                    message="adapter stop timed out",
                    command_id="command-8",
                    error_code="STOP_FAILED",
                )

        manager = DriverLifecycleManager(
            config_file="/tmp/mock.yaml",
            use_real_hardware=False,
            facade_factory=StopFailureFacade,
        )
        self.assertTrue(manager.configure()[0])
        self.assertTrue(manager.activate()[0])
        facade = manager.facade
        self.assertIsNotNone(facade)

        manager.shutdown()

        self.assertIs(manager.state, DriverLifecycleState.FAULT)
        self.assertIs(manager.facade, facade)
        self.assertIn("STOP_FAILED", manager.last_error)
        self.assertFalse(facade.connected)
        self.assertEqual(facade.close_count, 1)
