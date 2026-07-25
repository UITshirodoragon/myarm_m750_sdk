"""Core facade normalization regression tests."""

import unittest
from types import SimpleNamespace

from myarm_m750_driver.contracts import CoreAdapterCapabilities
from myarm_m750_driver.core_facade import CoreRobotFacade


class CoreFacadeTest(unittest.TestCase):
    """Keep core enums and implementation objects behind ROS DTOs."""

    def test_normalizes_three_state_adapter_capabilities(self) -> None:
        facade = CoreRobotFacade("/tmp/mock.yaml")
        facade._session = SimpleNamespace(  # noqa: SLF001
            adapter_capabilities=lambda: SimpleNamespace(
                stop=SimpleNamespace(value="supported"),
                pause=SimpleNamespace(value="unsupported"),
                resume=SimpleNamespace(value="unverified"),
                power_control=SimpleNamespace(value="unverified"),
            )
        )

        capabilities = facade.adapter_capabilities

        self.assertEqual(
            capabilities,
            CoreAdapterCapabilities(
                stop="supported",
                pause="unsupported",
                resume="unverified",
                power_control="unverified",
            ),
        )

    def test_normalizes_capability_evidence_in_hardware_identity(self) -> None:
        facade = CoreRobotFacade("/tmp/mock.yaml")
        facade._session = SimpleNamespace(  # noqa: SLF001
            probe_hardware=lambda: SimpleNamespace(
                adapter="mock",
                model="mock",
                firmware_version="mock-1",
                serial_resource="memory://mock",
                mapping_fingerprint="mock-mapping",
                capability_verification_reference="builtin://mock/stop",
            )
        )

        identity = facade.probe_hardware()

        self.assertEqual(
            identity.capability_verification_reference,
            "builtin://mock/stop",
        )


if __name__ == "__main__":
    unittest.main()
