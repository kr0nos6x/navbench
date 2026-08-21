from __future__ import annotations

import dataclasses
import unittest

from navbench.sensors import (
    FaultSpec,
    GnssConfig,
    ImuConfig,
    ImuMeasurement,
    Landmark,
    LandmarkConfig,
    LandmarkMeasurement,
    SensorPipeline,
    SensorSuiteConfig,
    WheelSpeedConfig,
    WheelSpeedMeasurement,
)
from navbench.simulator import VehicleParameters, VehicleState


def zero_noise_config() -> SensorSuiteConfig:
    return SensorSuiteConfig(
        imu=ImuConfig(
            rate_hz=50.0,
            acceleration_noise_std_mps2=0.0,
            yaw_rate_noise_std_rad_s=0.0,
        ),
        wheel_speed=WheelSpeedConfig(rate_hz=25.0, noise_std_mps=0.0),
        gnss=GnssConfig(rate_hz=5.0, position_noise_std_m=0.0, latency_s=0.10),
        landmark=LandmarkConfig(
            rate_hz=10.0,
            range_noise_std_m=0.0,
            bearing_noise_std_rad=0.0,
            latency_s=0.04,
            max_range_m=20.0,
            field_of_view_rad=6.283185307179586,
            outlier_range_std_m=1.0,
            outlier_bearing_std_rad=0.5,
        ),
    )


class SensorPipelineTests(unittest.TestCase):
    def make_pipeline(self, **kwargs: object) -> SensorPipeline:
        values: dict[str, object] = {
            "dt_s": 0.02,
            "vehicle_parameters": VehicleParameters(),
            "config": zero_noise_config(),
            "landmarks": (Landmark(1, 3.0, 4.0),),
            "faults": (),
            "seed": 1234,
        }
        values.update(kwargs)
        return SensorPipeline(**values)  # type: ignore[arg-type]

    def test_rates_and_latency_are_step_based(self) -> None:
        pipeline = self.make_pipeline()
        delivered = []
        state = VehicleState(speed_mps=1.0)
        for step in range(51):
            delivered.extend(pipeline.step(step, step * 0.02, state))
        stats = pipeline.statistics
        self.assertEqual(stats["imu"]["generated"], 51)
        self.assertEqual(stats["wheel_speed"]["generated"], 26)
        self.assertEqual(stats["gnss"]["generated"], 6)
        self.assertEqual(stats["landmark"]["generated"], 11)
        gnss = [item for item in delivered if item.__class__.__name__ == "GnssMeasurement"]
        self.assertEqual(gnss[0].sample_step_id, 0)
        self.assertEqual(gnss[0].delivery_step_id, 5)
        self.assertEqual(gnss[0].delivery_time_s, 0.1)

    def test_same_seed_produces_identical_measurements(self) -> None:
        noisy = SensorSuiteConfig()
        first = self.make_pipeline(config=noisy)
        second = self.make_pipeline(config=noisy)
        state = VehicleState(x_m=1.0, y_m=2.0, speed_mps=1.2, steering_rad=0.1)
        first_values = []
        second_values = []
        for step in range(101):
            first_values.extend(first.step(step, step * 0.02, state))
            second_values.extend(second.step(step, step * 0.02, state))
        self.assertEqual(first_values, second_values)

    def test_different_seed_changes_noise(self) -> None:
        first = self.make_pipeline(config=SensorSuiteConfig(), seed=1)
        second = self.make_pipeline(config=SensorSuiteConfig(), seed=2)
        state = VehicleState(speed_mps=1.0)
        self.assertNotEqual(first.step(0, 0.0, state), second.step(0, 0.0, state))

    def test_measurements_do_not_contain_ground_truth_state(self) -> None:
        pipeline = self.make_pipeline()
        measurements = pipeline.step(0, 0.0, VehicleState(speed_mps=1.0))
        self.assertTrue(measurements)
        for measurement in measurements:
            names = {field.name for field in dataclasses.fields(measurement)}
            self.assertNotIn("state", names)
            self.assertNotIn("ground_truth", names)
            self.assertFalse(
                any(isinstance(value, VehicleState) for value in dataclasses.astuple(measurement))
            )

    def test_forced_gnss_dropout_and_fault_events(self) -> None:
        fault = FaultSpec("gnss", "dropout", 0.2, 0.4)
        pipeline = self.make_pipeline(faults=(fault,))
        state = VehicleState()
        events = []
        for step in range(26):
            pipeline.step(step, step * 0.02, state)
            events.extend(pipeline.drain_events())
        self.assertEqual(pipeline.statistics["gnss"]["dropped"], 1)
        self.assertEqual(
            [(event.event_type, event.step_id) for event in events],
            [("fault_started", 10), ("fault_ended", 20)],
        )

    def test_imu_bias_and_wheel_slip_faults_apply_only_in_window(self) -> None:
        faults = (
            FaultSpec("imu", "bias_acceleration_mps2", 0.02, 0.04, 0.5),
            FaultSpec("wheel_speed", "scale_error", 0.04, 0.08, 0.25),
        )
        pipeline = self.make_pipeline(faults=faults)
        state = VehicleState(speed_mps=2.0, acceleration_mps2=0.2)
        by_step = {}
        for step in range(5):
            by_step[step] = pipeline.step(step, step * 0.02, state)
        imu = next(item for item in by_step[1] if isinstance(item, ImuMeasurement))
        wheel = next(item for item in by_step[2] if isinstance(item, WheelSpeedMeasurement))
        self.assertAlmostEqual(imu.acceleration_mps2, 0.7)
        self.assertAlmostEqual(wheel.speed_mps, 2.5)

    def test_landmark_geometry_visibility_and_forced_outlier(self) -> None:
        fault = FaultSpec("landmark", "outlier", 0.0, 0.02)
        pipeline = self.make_pipeline(faults=(fault,))
        delivered = []
        for step in range(3):
            delivered.extend(pipeline.step(step, step * 0.02, VehicleState()))
        measurement = next(item for item in delivered if isinstance(item, LandmarkMeasurement))
        self.assertEqual(measurement.delivery_step_id, 2)
        self.assertEqual(len(measurement.observations), 1)
        self.assertTrue(measurement.observations[0].is_outlier)
        self.assertEqual(pipeline.statistics["landmark"]["outliers"], 1)

    def test_latency_spike_changes_delivery_not_sample_timestamp(self) -> None:
        config = zero_noise_config()
        config = dataclasses.replace(config, gnss=dataclasses.replace(config.gnss, latency_s=0.0))
        pipeline = self.make_pipeline(
            config=config,
            faults=(FaultSpec("gnss", "latency_spike", 0.0, 0.02, 0.11),),
        )
        delivered = []
        for step in range(7):
            delivered.extend(pipeline.step(step, step * 0.02, VehicleState()))
        gnss = next(item for item in delivered if item.__class__.__name__ == "GnssMeasurement")
        self.assertEqual(gnss.sample_step_id, 0)
        self.assertEqual(gnss.sample_time_s, 0.0)
        self.assertEqual(gnss.delivery_step_id, 6)

    def test_invalid_rate_and_noncontiguous_steps_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "integer divisor"):
            self.make_pipeline(config=SensorSuiteConfig(imu=ImuConfig(rate_hz=30.0)))
        pipeline = self.make_pipeline()
        with self.assertRaisesRegex(ValueError, "contiguous"):
            pipeline.step(1, 0.02, VehicleState())


if __name__ == "__main__":
    unittest.main()
