"""Deterministic tests for read-only network metrics."""

import unittest

from myarm_m750_visualization.network_metrics import NetworkMetrics


class NetworkMetricsTest(unittest.TestCase):
    """Verify discovery, reconnect, latency, rate, and bandwidth math."""

    def test_reports_rate_age_gap_reconnect_and_bandwidth(self) -> None:
        metrics = NetworkMetrics(start_monotonic_s=10.0, reconnect_gap_s=1.0)
        metrics.observe(10.2, 100.2, 100.1, 100)
        metrics.observe(10.4, 100.4, 100.2, 100)
        metrics.observe(12.0, 102.0, 101.7, 100)

        snapshot = metrics.snapshot(12.0)

        self.assertEqual(snapshot.sample_count, 3)
        self.assertAlmostEqual(snapshot.discovery_time_s, 0.2)
        self.assertAlmostEqual(snapshot.effective_rate_hz, 2.0 / 1.8)
        self.assertAlmostEqual(snapshot.maximum_gap_s, 1.6)
        self.assertEqual(snapshot.reconnect_count, 1)
        self.assertAlmostEqual(snapshot.maximum_reconnect_s, 1.6)
        self.assertAlmostEqual(snapshot.message_age_p50_ms, 200.0)
        self.assertAlmostEqual(snapshot.bandwidth_mbit_s, 0.0012)
        self.assertFalse(snapshot.stale)
        self.assertAlmostEqual(snapshot.current_silence_s, 0.0)

    def test_current_outage_degrades_gap_rate_and_staleness(self) -> None:
        metrics = NetworkMetrics(start_monotonic_s=10.0, reconnect_gap_s=1.0)
        metrics.observe(10.2, 100.2, 100.1, 100)
        metrics.observe(10.4, 100.4, 100.3, 100)

        healthy = metrics.snapshot(10.4)
        outage = metrics.snapshot(13.4)

        self.assertFalse(healthy.stale)
        self.assertAlmostEqual(healthy.effective_rate_hz, 5.0)
        self.assertTrue(outage.stale)
        self.assertAlmostEqual(outage.current_silence_s, 3.0)
        self.assertAlmostEqual(outage.maximum_gap_s, 3.0)
        self.assertAlmostEqual(outage.maximum_reconnect_s, 3.0)
        self.assertAlmostEqual(outage.effective_rate_hz, 1.0 / 3.2)
        self.assertLess(
            outage.effective_rate_hz,
            healthy.effective_rate_hz,
        )

    def test_rejects_invalid_constructor_values(self) -> None:
        with self.assertRaises(ValueError):
            NetworkMetrics(start_monotonic_s=0.0, reconnect_gap_s=0.0)
        with self.assertRaises(ValueError):
            NetworkMetrics(
                start_monotonic_s=0.0,
                reconnect_gap_s=1.0,
                window_size=1,
            )
