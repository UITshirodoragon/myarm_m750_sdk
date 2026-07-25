"""Tests for the generated network-environment artifact boundary."""

import importlib.util
import tempfile
import unittest
from pathlib import Path

_LAUNCH_PATH = Path(__file__).parents[1] / "launch" / "robot.launch.py"
_SPEC = importlib.util.spec_from_file_location("myarm_robot_launch", _LAUNCH_PATH)
assert _SPEC is not None
assert _SPEC.loader is not None
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)


def _write_environment(directory, **overrides):
    directory_path = Path(directory)
    profile = directory_path / "fastdds.xml"
    profile.write_text("<profiles/>", encoding="utf-8")
    values = {
        "MYARM_M750_ROLE": "jetson",
        "MYARM_M750_WLAN_INTERFACE": "lo",
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
        "ROS_DOMAIN_ID": "42",
        "FASTRTPS_DEFAULT_PROFILES_FILE": "fastdds.xml",
    }
    values.update(overrides)
    environment_file = directory_path / "network.env"
    environment_file.write_text(
        "".join(f"{key}={value}\n" for key, value in values.items()),
        encoding="utf-8",
    )
    return environment_file


class NetworkEnvironmentTest(unittest.TestCase):
    """Verify allowlisted keys, roles, domains, and profile resolution."""

    def test_resolves_profile_relative_to_environment(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            environment = _MODULE._load_network_environment(
                str(_write_environment(directory))
            )

        self.assertEqual(environment["ROS_DOMAIN_ID"], "42")
        self.assertTrue(
            Path(environment["FASTRTPS_DEFAULT_PROFILES_FILE"]).is_absolute()
        )

    def test_rejects_wrong_role_and_unknown_key(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(RuntimeError, "role"):
                _MODULE._load_network_environment(
                    str(
                        _write_environment(
                            directory, MYARM_M750_ROLE="host"
                        )
                    )
                )
            with self.assertRaisesRegex(RuntimeError, "Unsupported"):
                _MODULE._load_network_environment(
                    str(
                        _write_environment(
                            directory, LD_PRELOAD="/tmp/injected.so"
                        )
                    )
                )


if __name__ == "__main__":
    unittest.main()
