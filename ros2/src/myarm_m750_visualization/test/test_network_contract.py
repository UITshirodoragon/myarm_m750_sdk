"""Network contract validation and Fast DDS rendering tests."""

import tempfile
import unittest
from pathlib import Path

import yaml
from myarm_m750_visualization.network_contract import (
    environment_for_contract,
    load_network_contract,
    render_fastdds_profile,
)


def _document(role="host", discovery_mode="peer"):
    return {
        "schema_version": 1,
        "network": {
            "role": role,
            "rmw_implementation": "rmw_fastrtps_cpp",
            "ros_domain_id": 42,
            "wlan_interface": "wlan0",
            "interface_address": "192.168.50.20",
            "discovery": {
                "mode": discovery_mode,
                "peer_addresses": ["192.168.50.10"],
                "server": None,
            },
        },
        "budget": {
            "minimum_joint_state_rate_hz": 4.5,
            "maximum_p95_age_ms": 250.0,
            "maximum_p99_age_ms": 500.0,
            "maximum_gap_s": 1.0,
            "maximum_reconnect_s": 15.0,
            "maximum_clock_offset_ms": 20.0,
            "maximum_control_bandwidth_mbit_s": 1.0,
        },
    }


def _write(directory, document):
    path = Path(directory) / "network.yaml"
    path.write_text(yaml.safe_dump(document), encoding="utf-8")
    return path


class NetworkContractTest(unittest.TestCase):
    """Verify strict schema, role separation, and peer profile rendering."""

    def test_renders_machine_specific_peer_profile(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            contract = load_network_contract(
                str(_write(directory, _document())), expected_role="host"
            )
            profile = render_fastdds_profile(contract)

            self.assertIn("192.168.50.20", profile)
            self.assertIn("192.168.50.10", profile)
            self.assertIn("myarm_m750_wlan_udp", profile)
            self.assertIn(
                "<avoid_builtin_multicast>true</avoid_builtin_multicast>",
                profile,
            )
            self.assertIn("<initialPeersList>", profile)
            environment = environment_for_contract(
                contract, str(Path(directory) / "fastdds.xml")
            )
            self.assertEqual(
                environment["RMW_IMPLEMENTATION"], "rmw_fastrtps_cpp"
            )
            self.assertEqual(environment["ROS_DOMAIN_ID"], "42")

    def test_multicast_profile_keeps_builtin_multicast(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            contract = load_network_contract(
                str(_write(directory, _document(discovery_mode="multicast"))),
                expected_role="host",
            )
            profile = render_fastdds_profile(contract)

            self.assertIn(
                "<avoid_builtin_multicast>false</avoid_builtin_multicast>",
                profile,
            )
            self.assertNotIn("<initialPeersList>", profile)

    def test_discovery_server_uses_foxy_environment_contract(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            document = _document(discovery_mode="discovery_server")
            document["network"]["discovery"]["server"] = "192.168.50.30:11811"
            contract = load_network_contract(
                str(_write(directory, document)),
                expected_role="host",
            )

            environment = environment_for_contract(
                contract,
                str(Path(directory) / "fastdds.xml"),
            )

            self.assertEqual(
                environment["ROS_DISCOVERY_SERVER"],
                "192.168.50.30:11811",
            )

    def test_rejects_role_and_unknown_fields(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = _write(directory, _document(role="jetson"))
            with self.assertRaisesRegex(ValueError, "expected role"):
                load_network_contract(str(path), expected_role="host")

            document = _document()
            document["network"]["implicit_fallback"] = True
            path = _write(directory, document)
            with self.assertRaisesRegex(ValueError, "unknown fields"):
                load_network_contract(str(path), expected_role="host")

    def test_peer_mode_requires_explicit_peer(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            document = _document()
            document["network"]["discovery"]["peer_addresses"] = []
            with self.assertRaisesRegex(ValueError, "at least one peer"):
                load_network_contract(
                    str(_write(directory, document)), expected_role="host"
                )
