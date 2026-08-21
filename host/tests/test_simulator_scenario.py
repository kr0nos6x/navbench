from __future__ import annotations

import math
import tempfile
import unittest
from pathlib import Path

import yaml

from navbench.scenario import load_scenario, run_scenario, scenario_to_mapping
from navbench.simulator import (
    ControlCommand,
    VehicleModel,
    VehicleParameters,
    VehicleState,
    run_open_loop,
    save_csv,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


class SimulatorTests(unittest.TestCase):
    def test_fixed_step_time_semantics_and_determinism(self) -> None:
        scenario = load_scenario(REPOSITORY_ROOT / "scenarios" / "straight.yaml")
        first = run_scenario(scenario)
        second = run_scenario(scenario)
        self.assertEqual(first, second)
        self.assertEqual(len(first), scenario.step_count + 1)
        self.assertEqual(first[0].step_id, 0)
        self.assertEqual(first[0].time_s, 0.0)
        self.assertEqual(first[-1].step_id, scenario.step_count)
        self.assertEqual(first[-1].time_s, scenario.duration_s)
        self.assertEqual(first[0].state, scenario.initial_state)

    def test_command_interval_is_left_closed_right_open(self) -> None:
        scenario = load_scenario(REPOSITORY_ROOT / "scenarios" / "straight.yaml")
        self.assertEqual(scenario.command_at(1.98).acceleration_mps2, 1.0)
        self.assertEqual(scenario.command_at(2.0).acceleration_mps2, 0.0)

    def test_actuators_and_speed_are_saturated(self) -> None:
        params = VehicleParameters(
            max_steering_rad=0.2,
            max_acceleration_mps2=1.0,
            max_deceleration_mps2=2.0,
            max_speed_mps=1.5,
        )
        model = VehicleModel(params)
        state = VehicleState()
        command = ControlCommand(acceleration_mps2=100.0, steering_rad=2.0)
        for _ in range(500):
            state = model.step(state, command, 0.02)
        self.assertLessEqual(state.speed_mps, params.max_speed_mps)
        self.assertLessEqual(state.acceleration_mps2, params.max_acceleration_mps2)
        self.assertLessEqual(state.steering_rad, params.max_steering_rad)
        self.assertAlmostEqual(state.speed_mps, params.max_speed_mps)

    def test_braking_never_reverses_vehicle(self) -> None:
        model = VehicleModel()
        state = VehicleState(speed_mps=0.1)
        for _ in range(100):
            state = model.step(state, ControlCommand(-100.0, 0.0), 0.02)
        self.assertEqual(state.speed_mps, 0.0)
        self.assertEqual(state.acceleration_mps2, 0.0)

    def test_invalid_finite_values_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            VehicleParameters(wheelbase_m=float("nan"))
        with self.assertRaises(ValueError):
            VehicleParameters(max_steering_rad=math.pi / 2)
        with self.assertRaises(ValueError):
            VehicleModel().step(VehicleState(), ControlCommand(0.0, 0.0), 0.0)
        with self.assertRaises(TypeError):
            ControlCommand(True, 0.0)

    def test_legacy_open_loop_uses_exact_step_policy(self) -> None:
        self.assertEqual(len(run_open_loop(1.0, 0.02)), 51)
        with self.assertRaises(ValueError):
            run_open_loop(1.0, 0.03)

    def test_direct_csv_export_does_not_silently_overwrite(self) -> None:
        samples = run_open_loop(0.1, 0.02)
        with tempfile.TemporaryDirectory(dir="/tmp") as directory:
            path = Path(directory) / "ground_truth.csv"
            save_csv(samples, path)
            original = path.read_bytes()
            with self.assertRaises(FileExistsError):
                save_csv(samples, path)
            self.assertEqual(path.read_bytes(), original)


class ScenarioTests(unittest.TestCase):
    def test_all_smoke_scenarios_validate_and_execute(self) -> None:
        expected = {
            "straight.yaml",
            "constant_turn.yaml",
            "s_curve.yaml",
            "saturation.yaml",
            "stop.yaml",
            "fault_smoke.yaml",
        }
        found = {path.name for path in (REPOSITORY_ROOT / "scenarios").glob("*.yaml")}
        self.assertTrue(expected <= found)
        for name in sorted(expected):
            with self.subTest(name=name):
                scenario = load_scenario(REPOSITORY_ROOT / "scenarios" / name)
                samples = run_scenario(scenario)
                self.assertEqual(samples[-1].time_s, scenario.duration_s)
                self.assertTrue(all(math.isfinite(sample.state.x_m) for sample in samples))

    def test_vehicle_route_landmarks_faults_and_seed_are_loaded(self) -> None:
        scenario = load_scenario(REPOSITORY_ROOT / "scenarios" / "fault_smoke.yaml")
        self.assertEqual(scenario.seed, 424242)
        self.assertEqual(scenario.vehicle.wheelbase_m, 0.32)
        self.assertEqual(len(scenario.route), 3)
        self.assertEqual(len(scenario.landmarks), 3)
        self.assertEqual(len(scenario.faults), 6)
        self.assertEqual(scenario.vehicle_parameters, scenario.vehicle)

    def test_scenario_snapshot_is_reloadable(self) -> None:
        original = load_scenario(REPOSITORY_ROOT / "scenarios" / "fault_smoke.yaml")
        with tempfile.TemporaryDirectory(dir="/tmp") as directory:
            path = Path(directory) / "snapshot.yaml"
            path.write_text(
                yaml.safe_dump(scenario_to_mapping(original), sort_keys=True),
                encoding="utf-8",
            )
            restored = load_scenario(path)
        self.assertEqual(restored.name, original.name)
        self.assertEqual(restored.seed, original.seed)
        self.assertEqual(restored.commands, original.commands)
        self.assertEqual(restored.faults, original.faults)
        self.assertEqual(restored.route, original.route)

    def test_non_integral_duration_is_rejected(self) -> None:
        raw = {
            "version": 1,
            "name": "invalid_duration",
            "duration_s": 1.0,
            "dt_s": 0.03,
            "commands": [
                {"until_s": 1.0, "acceleration_mps2": 0.0, "steering_deg": 0.0}
            ],
        }
        with self.assertRaisesRegex(ValueError, "exact integer multiple"):
            self._load_raw(raw)

    def test_unknown_and_wrong_type_fields_are_rejected(self) -> None:
        raw = {
            "version": 1,
            "name": "bad_type",
            "duration_s": "1.0",
            "dt_s": 0.02,
            "commands": [
                {"until_s": 1.0, "acceleration_mps2": 0.0, "steering_deg": 0.0}
            ],
        }
        with self.assertRaisesRegex(ValueError, "finite number"):
            self._load_raw(raw)
        raw["duration_s"] = 1.0
        raw["typo"] = 1
        with self.assertRaisesRegex(ValueError, "unknown field"):
            self._load_raw(raw)

    def test_command_and_fault_boundaries_must_align_to_timestep(self) -> None:
        raw = {
            "version": 1,
            "name": "misaligned",
            "duration_s": 1.0,
            "dt_s": 0.02,
            "commands": [
                {"until_s": 0.31, "acceleration_mps2": 0.0, "steering_deg": 0.0},
                {"until_s": 1.0, "acceleration_mps2": 0.0, "steering_deg": 0.0},
            ],
        }
        with self.assertRaisesRegex(ValueError, "exact integer multiple"):
            self._load_raw(raw)

    @staticmethod
    def _load_raw(raw: dict[str, object]):
        with tempfile.TemporaryDirectory(dir="/tmp") as directory:
            path = Path(directory) / "scenario.yaml"
            path.write_text(yaml.safe_dump(raw), encoding="utf-8")
            return load_scenario(path)


if __name__ == "__main__":
    unittest.main()
