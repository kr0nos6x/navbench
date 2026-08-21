from __future__ import annotations

import math
import unittest

import numpy as np

from navbench.reference import (
    ReferenceEkf6,
    ReferenceNavigationMode,
    ReferenceState,
    UpdateStatus,
)


class ReferenceEkfTests(unittest.TestCase):
    def test_prediction_jacobian_matches_finite_difference(self) -> None:
        state = ReferenceState(1.2, -0.7, 0.4, 2.1, 0.12, 0.03)
        acceleration = 0.45
        dt_s = 0.02
        _, analytic = ReferenceEkf6.prediction_model(
            state, acceleration, dt_s
        )
        base = state.as_array()
        numeric = np.zeros((6, 6))
        epsilon = 1.0e-6
        for column in range(6):
            plus = base.copy()
            minus = base.copy()
            plus[column] += epsilon
            minus[column] -= epsilon
            plus_state, _ = ReferenceEkf6.prediction_model(
                ReferenceState.from_array(plus), acceleration, dt_s
            )
            minus_state, _ = ReferenceEkf6.prediction_model(
                ReferenceState.from_array(minus), acceleration, dt_s
            )
            delta = plus_state.as_array() - minus_state.as_array()
            delta[2] = _angle_difference(
                plus_state.heading_rad, minus_state.heading_rad
            )
            numeric[:, column] = delta / (2.0 * epsilon)
        np.testing.assert_allclose(analytic, numeric, rtol=1e-6, atol=2e-7)

    def test_landmark_jacobian_matches_finite_difference(self) -> None:
        state = ReferenceState(1.0, -2.0, 0.3, 1.0, 0.1, 0.02)
        landmark = (5.0, 3.0)
        _, analytic = ReferenceEkf6.landmark_model_and_jacobian(
            state, *landmark
        )
        base = state.as_array()
        numeric = np.zeros((2, 6))
        epsilon = 1.0e-6
        for column in range(6):
            plus = base.copy()
            minus = base.copy()
            plus[column] += epsilon
            minus[column] -= epsilon
            plus_measurement, _ = ReferenceEkf6.landmark_model_and_jacobian(
                ReferenceState.from_array(plus), *landmark
            )
            minus_measurement, _ = ReferenceEkf6.landmark_model_and_jacobian(
                ReferenceState.from_array(minus), *landmark
            )
            delta = plus_measurement - minus_measurement
            delta[1] = _angle_difference(
                float(plus_measurement[1]), float(minus_measurement[1])
            )
            numeric[:, column] = delta / (2.0 * epsilon)
        np.testing.assert_allclose(analytic, numeric, rtol=1e-6, atol=2e-7)

    def test_updates_preserve_symmetric_finite_covariance(self) -> None:
        ekf = ReferenceEkf6()
        ekf.initialize(ReferenceState(speed_mps=1.0))
        for step in range(1, 101):
            now_ms = step * 20
            self.assertEqual(
                ekf.predict(
                    longitudinal_accel_mps2=0.01,
                    yaw_rate_rad_s=0.02,
                    timestamp_ms=now_ms,
                    dt_s=0.02,
                ).status,
                UpdateStatus.ACCEPTED,
            )
            if step % 5 == 0:
                ekf.update_wheel(1.0 + 0.0001 * step, now_ms)
            if step % 50 == 0:
                ekf.update_gnss(0.02 * step, 0.0, now_ms)
        self.assertTrue(ekf.healthy)
        self.assertTrue(np.all(np.isfinite(ekf.covariance)))
        np.testing.assert_allclose(
            ekf.covariance,
            ekf.covariance.T,
            rtol=0.0,
            atol=1.0e-12,
        )
        self.assertTrue(np.all(np.diag(ekf.covariance) > 0.0))

    def test_outliers_are_rejected_without_changing_state(self) -> None:
        ekf = ReferenceEkf6()
        ekf.initialize(ReferenceState())
        before = ekf.state.copy()
        outcome = ekf.update_gnss(1000.0, -1000.0, 10)
        self.assertEqual(outcome.status, UpdateStatus.REJECTED_GATE)
        np.testing.assert_array_equal(before, ekf.state)

    def test_navigation_modes_follow_freshness(self) -> None:
        ekf = ReferenceEkf6()
        ekf.initialize(ReferenceState())
        self.assertEqual(
            ekf.navigation_mode(0), ReferenceNavigationMode.UNAVAILABLE
        )
        ekf.predict(
            longitudinal_accel_mps2=0.0,
            yaw_rate_rad_s=0.0,
            timestamp_ms=20,
            dt_s=0.02,
        )
        self.assertEqual(
            ekf.navigation_mode(20), ReferenceNavigationMode.DEGRADED
        )
        ekf.update_wheel(0.0, 20)
        self.assertEqual(
            ekf.navigation_mode(20), ReferenceNavigationMode.DEAD_RECKONING
        )
        ekf.update_landmark(
            landmark_x_m=5.0,
            landmark_y_m=0.0,
            range_m=5.0,
            bearing_rad=0.0,
            timestamp_ms=20,
        )
        self.assertEqual(
            ekf.navigation_mode(20), ReferenceNavigationMode.LANDMARK_AIDED
        )
        ekf.update_gnss(0.0, 0.0, 20)
        self.assertEqual(
            ekf.navigation_mode(20), ReferenceNavigationMode.GNSS_AIDED
        )
        self.assertEqual(
            ekf.navigation_mode(2000), ReferenceNavigationMode.UNAVAILABLE
        )

    def test_yaw_rate_tracks_a_steering_transient_without_false_gating(self) -> None:
        ekf = ReferenceEkf6()
        ekf.initialize(ReferenceState(speed_mps=1.5))
        for step in range(1, 101):
            outcome = ekf.predict(
                longitudinal_accel_mps2=0.0,
                yaw_rate_rad_s=1.5 * step / 100.0,
                timestamp_ms=step * 20,
                dt_s=0.02,
            )
            self.assertEqual(outcome.status, UpdateStatus.ACCEPTED)
        self.assertAlmostEqual(float(ekf.state[4]), 1.5, delta=0.10)
        self.assertNotIn("imu", ekf.stats.rejected)


def _angle_difference(first: float, second: float) -> float:
    return (first - second + math.pi) % (2.0 * math.pi) - math.pi


if __name__ == "__main__":
    unittest.main()
