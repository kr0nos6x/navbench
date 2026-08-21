from __future__ import annotations

import math
import unittest

from navbench.metrics import Pose2, distance_to_polyline, summarize_run


class MetricTests(unittest.TestCase):
    def test_exact_run_has_zero_errors(self) -> None:
        truth = [Pose2(0.0, 0.0, 0.0), Pose2(1.0, 0.0, 0.0)]
        summary = summarize_run(
            truth=truth,
            estimates=truth,
            route=[(0.0, 0.0), (1.0, 0.0)],
            final_speed_mps=0.0,
            nis_values=[1.0, 3.0],
            max_position_rmse_m=0.01,
            max_cross_track_rmse_m=0.01,
            max_final_stop_error_m=0.01,
            max_final_speed_mps=0.01,
        )
        self.assertEqual(summary.sample_count, 2)
        self.assertEqual(summary.position_rmse_m, 0.0)
        self.assertEqual(summary.heading_rmse_rad, 0.0)
        self.assertEqual(summary.cross_track_rmse_m, 0.0)
        self.assertEqual(summary.final_stop_error_m, 0.0)
        self.assertEqual(summary.nis_mean, 2.0)
        self.assertTrue(summary.success)

    def test_position_heading_and_cross_track_are_computed(self) -> None:
        truth = [Pose2(0.0, 1.0, math.pi - 0.1)]
        estimate = [Pose2(3.0, 5.0, -math.pi + 0.1)]
        summary = summarize_run(
            truth=truth,
            estimates=estimate,
            route=[(0.0, 0.0), (2.0, 0.0)],
            final_speed_mps=0.2,
        )
        self.assertAlmostEqual(summary.position_rmse_m, 5.0)
        self.assertAlmostEqual(summary.heading_rmse_rad, 0.2)
        self.assertAlmostEqual(summary.cross_track_rmse_m, 1.0)
        self.assertAlmostEqual(summary.final_stop_error_m, math.sqrt(5.0))

    def test_aggregate_nis_is_weighted_without_repeating_health_samples(self) -> None:
        pose = Pose2(0.0, 0.0, 0.0)
        summary = summarize_run(
            truth=[pose],
            estimates=[pose],
            route=[(0.0, 0.0), (1.0, 0.0)],
            final_speed_mps=0.0,
            nis_evaluated_count=4,
            nis_sum=10.0,
            nis_maximum=6.0,
        )
        self.assertEqual(summary.nis_mean, 2.5)
        self.assertEqual(summary.nis_max, 6.0)

    def test_polyline_distance_handles_projection_and_zero_segment(self) -> None:
        route = [(0.0, 0.0), (0.0, 0.0), (2.0, 0.0)]
        self.assertAlmostEqual(distance_to_polyline(1.0, 1.0, route), 1.0)
        self.assertAlmostEqual(distance_to_polyline(3.0, 0.0, route), 1.0)

    def test_invalid_inputs_are_rejected(self) -> None:
        pose = Pose2(0.0, 0.0, 0.0)
        with self.assertRaises(ValueError):
            summarize_run(
                truth=[pose],
                estimates=[],
                route=[(0.0, 0.0), (1.0, 0.0)],
                final_speed_mps=0.0,
            )
        with self.assertRaises(ValueError):
            summarize_run(
                truth=[pose],
                estimates=[pose],
                route=[(0.0, 0.0)],
                final_speed_mps=0.0,
            )
        with self.assertRaises(ValueError):
            summarize_run(
                truth=[pose],
                estimates=[pose],
                route=[(0.0, 0.0), (1.0, 0.0)],
                final_speed_mps=0.0,
                nis_values=[math.nan],
            )
        with self.assertRaises(ValueError):
            summarize_run(
                truth=[pose],
                estimates=[pose],
                route=[(0.0, 0.0), (1.0, 0.0)],
                final_speed_mps=0.0,
                nis_evaluated_count=1,
                nis_sum=1.0,
            )
        with self.assertRaises(ValueError):
            summarize_run(
                truth=[pose],
                estimates=[pose],
                route=[(0.0, 0.0), (1.0, 0.0)],
                final_speed_mps=0.0,
                nis_values=[1.0],
                nis_evaluated_count=1,
                nis_sum=1.0,
                nis_maximum=1.0,
            )


if __name__ == "__main__":
    unittest.main()
