from __future__ import annotations

import os
import subprocess
import unittest
from pathlib import Path

import numpy as np

from navbench.reference import (
    ReferenceEkf6,
    ReferenceState,
    UpdateOutcome,
    UpdateStatus,
)


ROOT = Path(__file__).resolve().parents[2]
FIXTURE = ROOT / "test" / "fixtures" / "ekf_v1_fixture.tsv"
RUNNER = os.environ.get("NAVBENCH_EKF_FIXTURE_RUNNER")


@unittest.skipUnless(RUNNER, "native EKF fixture runner not configured")
class EkfCrossLanguageParityTests(unittest.TestCase):
    def test_cpp_float32_matches_python_float64_reference(self) -> None:
        completed = subprocess.run(
            [RUNNER or "", str(FIXTURE)],
            check=True,
            capture_output=True,
            text=True,
        )
        native = {
            int(fields[0]): fields
            for fields in (
                line.split("\t")
                for line in completed.stdout.splitlines()
                if line
            )
        }
        reference = ReferenceEkf6()
        exercised: set[str] = set()
        for line_number, raw in enumerate(
            FIXTURE.read_text(encoding="ascii").splitlines(), start=1
        ):
            if not raw or raw.startswith("#"):
                continue
            fields = raw.split("\t")
            operation = fields[0]
            timestamp_ms = int(fields[1])
            values = [float(value) for value in fields[2:-1]]
            expected_status = int(fields[-1])
            exercised.add(operation)
            if operation == "INIT":
                reference.initialize(ReferenceState(*values), timestamp_ms)
                outcome = UpdateOutcome(UpdateStatus.ACCEPTED)
            elif operation == "IMU":
                dt_s, acceleration, yaw_rate = values
                outcome = reference.predict(
                    longitudinal_accel_mps2=acceleration,
                    yaw_rate_rad_s=yaw_rate,
                    timestamp_ms=timestamp_ms,
                    dt_s=dt_s,
                )
            elif operation == "WHEEL":
                outcome = reference.update_wheel(values[0], timestamp_ms)
            elif operation == "GNSS":
                outcome = reference.update_gnss(*values, timestamp_ms)
            elif operation == "LANDMARK":
                outcome = reference.update_landmark(
                    landmark_x_m=values[0],
                    landmark_y_m=values[1],
                    range_m=values[2],
                    bearing_rad=values[3],
                    timestamp_ms=timestamp_ms,
                )
            elif operation == "MODE":
                outcome = UpdateOutcome(UpdateStatus.ACCEPTED)
            else:  # pragma: no cover - fixture parser is deliberately exhaustive.
                self.fail(f"unknown fixture operation: {operation}")

            row = native[line_number]
            native_status = int(row[1])
            native_nis = float(row[2])
            native_state = np.asarray(row[3:9], dtype=np.float64)
            native_covariance = np.asarray(row[9:15], dtype=np.float64)
            native_mode = int(row[15])
            self.assertEqual(native_status, expected_status, raw)
            self.assertEqual(native_status, int(outcome.status), raw)
            self.assertAlmostEqual(native_nis, outcome.nis, delta=0.05, msg=raw)
            np.testing.assert_allclose(
                native_state,
                reference.state,
                rtol=2.0e-5,
                atol=2.0e-5,
                err_msg=raw,
            )
            np.testing.assert_allclose(
                native_covariance,
                np.diag(reference.covariance),
                rtol=4.0e-5,
                atol=2.0e-5,
                err_msg=raw,
            )
            self.assertEqual(
                native_mode,
                int(reference.navigation_mode(timestamp_ms)),
                raw,
            )
        self.assertEqual(
            exercised,
            {"INIT", "IMU", "WHEEL", "GNSS", "LANDMARK", "MODE"},
        )


if __name__ == "__main__":
    unittest.main()
