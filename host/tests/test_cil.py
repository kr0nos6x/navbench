from __future__ import annotations

import math
import os
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

from navbench.cil import replay_native_run, run_closed_loop
from navbench.protocol import ControlMode
from navbench.runlog import RunReplay
from navbench.scenario import load_scenario
from navbench.sensors import FaultSpec


ROOT = Path(__file__).resolve().parents[2]
NATIVE_EXECUTABLE = os.environ.get("NAVBENCH_NATIVE_FIRMWARE")


@unittest.skipUnless(NATIVE_EXECUTABLE, "native firmware executable not configured")
class ControllerInTheLoopTests(unittest.TestCase):
    @property
    def native(self) -> Path:
        return Path(NATIVE_EXECUTABLE or "")

    def test_nominal_straight_turn_s_curve_and_final_stop(self) -> None:
        limits = {
            "straight": (0.15, 0.25, 0.8),
            "constant_turn": (0.15, 0.50, 0.7),
            "s_curve": (0.15, 0.35, 0.7),
            "stop": (0.15, 0.10, 0.2),
        }
        for name, (position_limit, cross_track_limit, stop_limit) in limits.items():
            with self.subTest(scenario=name):
                scenario = load_scenario(ROOT / "scenarios" / f"{name}.yaml")
                result = run_closed_loop(
                    scenario,
                    native_executable=self.native,
                    create_dashboard=False,
                )
                self.assertLess(result.metric_summary.position_rmse_m, position_limit)
                self.assertLess(result.metric_summary.cross_track_rmse_m, cross_track_limit)
                self.assertLess(result.metric_summary.final_stop_error_m, stop_limit)
                self.assertLess(
                    result.metric_summary.final_stop_error_m,
                    scenario.route[-1].acceptance_radius_m,
                )
                self.assertLessEqual(result.metric_summary.final_speed_mps, 0.10)
                self.assertLess(
                    result.summary["native_process_round_trip_ms_p99"],
                    scenario.dt_s * 1000.0,
                )
                final_command = result.controller_commands[-1].command
                self.assertIs(final_command.mode, ControlMode.TRACKING)
                self.assertEqual(final_command.flags & 0x01, 0x01)
                self.assertEqual(final_command.target_speed_mps, 0.0)
                self.assertEqual(final_command.steering_rad, 0.0)
                self.assertEqual(final_command.acceleration_mps2, 0.0)
                self.assertEqual(result.summary["protocol_error_messages"], 0)
                self.assertEqual(result.summary["parser_statistics"]["crc_errors"], 0)

    def test_same_seed_reproduces_closed_loop_state_estimate_and_commands(self) -> None:
        scenario = load_scenario(ROOT / "scenarios" / "constant_turn.yaml")
        first = run_closed_loop(
            scenario, native_executable=self.native, create_dashboard=False
        )
        second = run_closed_loop(
            scenario, native_executable=self.native, create_dashboard=False
        )
        self.assertEqual(first.plant_samples, second.plant_samples)
        self.assertEqual(first.estimates, second.estimates)
        self.assertEqual(first.controller_commands, second.controller_commands)

    def test_sensor_fault_smoke_exercises_gnss_denied_modes(self) -> None:
        result = run_closed_loop(
            load_scenario(ROOT / "scenarios" / "fault_smoke.yaml"),
            native_executable=self.native,
            create_dashboard=False,
        )
        modes = result.summary["navigation_mode_counts"]
        states = result.summary["safety_state_counts"]
        active = result.summary["active_fault_steps"]
        self.assertGreater(modes.get("gnss_aided", 0), 0)
        self.assertGreater(modes.get("landmark_aided", 0), 0)
        self.assertGreater(modes.get("dead_reckoning", 0), 0)
        self.assertGreater(states.get("degraded", 0), 0)
        for fault in (
            "gnss:dropout",
            "landmark:dropout",
            "imu:bias_acceleration_mps2",
            "wheel_speed:scale_error",
            "landmark:outlier",
            "gnss:latency_spike",
        ):
            self.assertGreater(active.get(fault, 0), 0)
        self.assertTrue(result.metric_summary.success)

    def test_transport_faults_are_counted_and_disconnect_safe_stops(self) -> None:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        result = run_closed_loop(
            load_scenario(ROOT / "scenarios" / "cil_faults.yaml"),
            native_executable=self.native,
            output_root=Path(temporary.name),
            run_name="cil_fault_replay",
            create_dashboard=False,
        )
        link = result.summary["link_statistics"]
        health = result.summary["firmware_health_counters"]
        states = result.summary["safety_state_counts"]
        self.assertGreater(link["frames_corrupted"], 0)
        self.assertGreater(link["frames_dropped"], 0)
        self.assertGreater(link["frames_delayed"], 0)
        self.assertGreater(link["stale_frames_replayed"], 0)
        self.assertGreater(link["rx_frames_corrupted"], 0)
        self.assertGreater(link["rx_frames_dropped"], 0)
        self.assertGreater(link["rx_frames_delayed"], 0)
        self.assertGreater(link["rx_stale_frames_replayed"], 0)
        self.assertGreater(link["rx_disconnect_drops"], 0)
        self.assertGreater(health["rx_crc_errors"] + health["rx_decode_errors"], 0)
        self.assertGreater(health["rx_missing"], 0)
        self.assertGreater(health["rx_duplicates"] + health["rx_out_of_order"], 0)
        self.assertGreater(states.get("safe_stop", 0), 0)
        self.assertEqual(result.summary["session_statistics"]["reconnects"], 1)
        self.assertGreater(result.summary["host_command_guard_steps"], 0)
        guarded_window = tuple(
            sample.command.mode
            for sample in result.controller_commands
            if 2.7 <= sample.time_s < 3.2
        )
        self.assertTrue(guarded_window)
        self.assertTrue(all(mode is ControlMode.SAFE_STOP for mode in guarded_window))
        self.assertEqual(result.metric_summary.final_speed_mps, 0.0)
        self.assertIsNotNone(result.run_path)
        replay = RunReplay(result.run_path or Path(temporary.name))
        event_types = {event["event_type"] for event in replay.events()}
        self.assertIn("session_reconnect", event_types)
        self.assertIn("host_command_guard_transition", event_types)
        self.assertIn("safety_state_transition", event_types)
        native_replay = replay_native_run(
            replay,
            native_executable=self.native,
        )
        self.assertTrue(native_replay.deterministic_match)
        self.assertEqual(native_replay.estimate_mismatches, 0)
        self.assertEqual(native_replay.command_mismatches, 0)

    def test_manual_safe_stop_is_latched(self) -> None:
        scenario = load_scenario(ROOT / "scenarios" / "straight.yaml")
        scenario = replace(
            scenario,
            faults=(FaultSpec("runtime", "manual_safe_stop", 0.5, 0.52),),
        )
        result = run_closed_loop(
            scenario, native_executable=self.native, create_dashboard=False
        )
        self.assertGreater(result.summary["safety_state_counts"].get("safe_stop", 0), 0)
        self.assertEqual(result.metric_summary.final_speed_mps, 0.0)

    def test_delayed_command_expires_from_its_source_step(self) -> None:
        scenario = load_scenario(ROOT / "scenarios" / "straight.yaml")
        scenario = replace(
            scenario,
            faults=(
                FaultSpec("transport", "latency_spike", 0.5, 0.52, 0.10),
                FaultSpec("transport", "packet_loss", 0.52, 0.8),
            ),
        )
        result = run_closed_loop(
            scenario,
            native_executable=self.native,
            create_dashboard=False,
        )
        delayed = next(
            sample
            for sample in result.controller_commands
            if math.isclose(sample.time_s, 0.62, abs_tol=1.0e-12)
        )
        self.assertIs(delayed.command.mode, ControlMode.SAFE_STOP)
        self.assertGreater(result.summary["host_command_guard_steps"], 0)

    def test_complete_artifact_dashboard_and_hardware_free_replay(self) -> None:
        scenario = load_scenario(ROOT / "scenarios" / "straight.yaml")
        with tempfile.TemporaryDirectory() as temporary:
            result = run_closed_loop(
                scenario,
                native_executable=self.native,
                output_root=Path(temporary),
                run_name="replay_fixture",
                create_dashboard=True,
            )
            self.assertIsNotNone(result.run_path)
            path = result.run_path or Path(temporary)
            self.assertTrue((path / "dashboard.png").is_file())
            replay = RunReplay(path)
            self.assertTrue(replay.is_complete)
            self.assertEqual(len(tuple(replay.ground_truth())), scenario.step_count + 1)
            self.assertEqual(len(tuple(replay.estimates())), scenario.step_count + 1)
            self.assertGreater(len(tuple(replay.measurements())), scenario.step_count)
            native_replay = replay_native_run(
                replay,
                native_executable=self.native,
            )
            self.assertTrue(native_replay.deterministic_match)
            self.assertEqual(native_replay.estimate_mismatches, 0)
            self.assertEqual(native_replay.command_mismatches, 0)
            self.assertEqual(native_replay.steps_replayed, scenario.step_count + 1)


if __name__ == "__main__":
    unittest.main()
