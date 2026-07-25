"""Camera QoS and launch safety contract tests."""

import unittest
from pathlib import Path

import yaml
from myarm_m750_camera.camera_bridge import IMAGE_QOS, RELIABLE_QOS
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)


class CameraRosContractTest(unittest.TestCase):
    """Keep camera transport low-latency and opt-in over WLAN."""

    def test_qos_contract(self) -> None:
        self.assertEqual(
            IMAGE_QOS.reliability,
            QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.assertEqual(IMAGE_QOS.history, QoSHistoryPolicy.KEEP_LAST)
        self.assertEqual(IMAGE_QOS.depth, 1)
        self.assertEqual(
            RELIABLE_QOS.reliability,
            QoSReliabilityPolicy.RELIABLE,
        )
        self.assertEqual(RELIABLE_QOS.history, QoSHistoryPolicy.KEEP_LAST)
        self.assertEqual(RELIABLE_QOS.depth, 5)

    def test_launch_is_disabled_without_explicit_camera_profile(self) -> None:
        launch_source = (
            Path(__file__).resolve().parents[1]
            / "launch"
            / "camera_bridge.launch.py"
        ).read_text(encoding="utf-8")

        self.assertIn(
            'DeclareLaunchArgument("enable_camera", default_value="false")',
            launch_source,
        )
        self.assertIn(
            'DeclareLaunchArgument("core_camera_config_file"',
            launch_source,
        )
        self.assertIn("condition=IfCondition(enable)", launch_source)
        self.assertIn('"config",', launch_source)
        self.assertIn('"camera_bridge.yaml"', launch_source)
        self.assertNotIn('"publish_rate_hz": 15.0', launch_source)
        self.assertNotIn("cv_bridge", launch_source)

        parameters = yaml.safe_load(
            (
                Path(__file__).resolve().parents[1]
                / "config"
                / "camera_bridge.yaml"
            ).read_text(encoding="utf-8")
        )["myarm_m750_camera_bridge"]["ros__parameters"]
        self.assertNotIn("core_camera_config_file", parameters)


if __name__ == "__main__":
    unittest.main()
