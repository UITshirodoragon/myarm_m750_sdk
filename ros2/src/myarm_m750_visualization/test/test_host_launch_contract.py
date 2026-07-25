"""Read-only Host launch ownership regression test."""

import importlib.util
import unittest
from pathlib import Path

import yaml
from myarm_m750_visualization.network_contract import load_network_contract

_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
_LAUNCH_PATH = _PACKAGE_ROOT / "launch" / "remote_observe.launch.py"
_SPEC = importlib.util.spec_from_file_location(
    "myarm_remote_observe_launch",
    _LAUNCH_PATH,
)
assert _SPEC is not None
assert _SPEC.loader is not None
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)


class HostLaunchContractTest(unittest.TestCase):
    """Keep WLAN observation independent from core and command ownership."""

    def test_remote_observe_has_no_core_or_action_server(self) -> None:
        launch_source = (
            Path(__file__).resolve().parents[1]
            / "launch"
            / "remote_observe.launch.py"
        ).read_text(encoding="utf-8")

        self.assertNotIn("myarm_m750_core", launch_source)
        self.assertNotIn("ActionServer", launch_source)
        self.assertNotIn("follow_joint_trajectory", launch_source)
        self.assertIn('executable="network_probe"', launch_source)
        self.assertIn('package="rviz2"', launch_source)

        local_launch_source = (
            _PACKAGE_ROOT / "launch" / "rviz_host.launch.py"
        ).read_text(encoding="utf-8")
        self.assertIn('"report_json_file"', local_launch_source)
        self.assertIn('"report_csv_file"', local_launch_source)
        self.assertIn('"local_loopback_same_clock"', local_launch_source)

    def test_validated_contract_owns_every_probe_budget(self) -> None:
        contract = load_network_contract(
            str(_PACKAGE_ROOT / "config" / "network_host_local.yaml"),
            expected_role="host",
        )

        parameters = _MODULE._network_budget_parameters(contract)

        self.assertEqual(
            set(parameters),
            {
                "minimum_joint_state_rate_hz",
                "maximum_p95_age_ms",
                "maximum_p99_age_ms",
                "maximum_gap_s",
                "maximum_reconnect_s",
                "maximum_clock_offset_ms",
                "maximum_control_bandwidth_mbit_s",
            },
        )
        self.assertEqual(
            parameters["minimum_joint_state_rate_hz"],
            contract.budget.minimum_joint_state_rate_hz,
        )
        self.assertEqual(
            parameters["maximum_clock_offset_ms"],
            contract.budget.maximum_clock_offset_ms,
        )

    def test_generic_probe_yaml_does_not_shadow_contract_budget(self) -> None:
        document = yaml.safe_load(
            (_PACKAGE_ROOT / "config" / "network_probe.yaml").read_text(
                encoding="utf-8"
            )
        )
        parameters = document["myarm_m750_network_probe"][
            "ros__parameters"
        ]
        contract_keys = set(
            _MODULE._network_budget_parameters(
                load_network_contract(
                    str(
                        _PACKAGE_ROOT
                        / "config"
                        / "network_host_local.yaml"
                    ),
                    expected_role="host",
                )
            )
        )

        self.assertFalse(contract_keys.intersection(parameters))
        self.assertNotIn("clock_offset_source", parameters)
        self.assertNotIn("measured_clock_offset_ms", parameters)
        self.assertNotIn("require_clock_offset_measurement", parameters)
        self.assertNotIn("report_interval_s", parameters)


if __name__ == "__main__":
    unittest.main()
