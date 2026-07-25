"""Explicit clock-sync measurement contract tests."""

import math
import unittest

from myarm_m750_visualization.network_probe import (
    clock_offset_budget_violation,
    explicit_clock_sync_measurement,
)


class ClockSyncTest(unittest.TestCase):
    """Keep external clock offset distinct from message timestamp skew."""

    def test_missing_measurement_fails_required_wlan_budget(self) -> None:
        measurement = explicit_clock_sync_measurement(0.0, "")

        self.assertFalse(measurement.available)
        self.assertIsNone(measurement.measured_clock_offset_ms)
        self.assertIn(
            "unavailable",
            clock_offset_budget_violation(
                measurement,
                maximum_absolute_offset_ms=20.0,
                required=True,
            ),
        )
        self.assertIsNone(
            clock_offset_budget_violation(
                measurement,
                maximum_absolute_offset_ms=20.0,
                required=False,
            )
        )

    def test_signed_measurement_uses_absolute_budget(self) -> None:
        measurement = explicit_clock_sync_measurement(
            -12.5,
            "chronyc_tracking_host_vs_jetson",
        )

        self.assertTrue(measurement.available)
        self.assertEqual(measurement.measured_clock_offset_ms, -12.5)
        self.assertEqual(measurement.absolute_clock_offset_ms, 12.5)
        self.assertIsNone(
            clock_offset_budget_violation(
                measurement,
                maximum_absolute_offset_ms=20.0,
                required=True,
            )
        )
        self.assertIn(
            "25",
            clock_offset_budget_violation(
                explicit_clock_sync_measurement(25.0, "chrony"),
                maximum_absolute_offset_ms=20.0,
                required=True,
            ),
        )

    def test_rejects_nonfinite_explicit_measurement(self) -> None:
        with self.assertRaisesRegex(ValueError, "finite"):
            explicit_clock_sync_measurement(math.nan, "chrony")


if __name__ == "__main__":
    unittest.main()
