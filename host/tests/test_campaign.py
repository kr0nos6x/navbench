from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from navbench.campaign import inspect_campaign, navigation_variant, run_campaign
from navbench.metrics import MetricSummary
from navbench.runlog import RunLogger
from navbench.scenario import load_scenario


ROOT = Path(__file__).resolve().parents[2]


class CampaignTests(unittest.TestCase):
    def setUp(self) -> None:
        self.scenario = load_scenario(ROOT / "scenarios" / "straight.yaml")

    def test_navigation_variants_share_initial_gnss_then_deny_selected_sensors(self) -> None:
        gnss = navigation_variant(self.scenario, "gnss_aided", 7)
        landmark = navigation_variant(self.scenario, "landmark_aided", 7)
        dead = navigation_variant(self.scenario, "dead_reckoning", 7)
        self.assertEqual(
            [(fault.target, fault.kind) for fault in gnss.faults],
            [
                ("imu", "bias_yaw_rate_rad_s"),
                ("wheel_speed", "scale_error"),
            ],
        )
        self.assertEqual(
            [(fault.target, fault.kind, fault.start_s) for fault in landmark.faults],
            [
                ("imu", "bias_yaw_rate_rad_s", 1.0),
                ("wheel_speed", "scale_error", 1.0),
                ("gnss", "dropout", 1.0),
            ],
        )
        self.assertEqual(
            [(fault.target, fault.kind) for fault in dead.faults],
            [
                ("imu", "bias_yaw_rate_rad_s"),
                ("wheel_speed", "scale_error"),
                ("gnss", "dropout"),
                ("landmark", "dropout"),
            ],
        )
        self.assertEqual((gnss.seed, landmark.seed, dead.seed), (7, 7, 7))

    def test_campaign_aggregates_matrix_and_inspection_detects_missing_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            executable = root / "native"
            executable.write_bytes(b"test")

            def fake_run(scenario: object, **kwargs: object) -> object:
                run_name = str(kwargs["run_name"])
                seed = int(getattr(scenario, "seed"))
                faults = getattr(scenario, "faults")
                mode_penalty = len(faults) * 0.1
                metric = MetricSummary(
                    sample_count=10,
                    position_rmse_m=seed * 0.01 + mode_penalty,
                    heading_rmse_rad=0.01,
                    cross_track_rmse_m=0.02,
                    cross_track_max_m=0.03,
                    final_stop_error_m=0.04,
                    final_speed_mps=0.0,
                    nis_mean=1.0,
                    nis_max=2.0,
                    success=True,
                )
                summary = {
                    "native_process_round_trip_ms_p99": 0.25 + seed,
                    "session_statistics": {"tx_packets": 10, "rx_rejected": seed},
                    "parser_statistics": {"crc_errors": seed},
                    "sequence_statistics": {"missing": seed},
                    "link_statistics": {"frames_dropped": seed, "rx_frames_corrupted": 1},
                    "firmware_health_counters": {"queue_overflows": 0},
                    "nis_statistics": {
                        "evaluated_count": 12,
                        "gate_rejected_count": 2,
                        "sum": 9.0,
                        "max": 2.0,
                    },
                }
                logger = RunLogger(
                    root / "campaign", scenario, run_name=run_name,
                    git_commit="test", git_dirty=False,
                    source_tree_sha256="0" * 64,
                    controller_binary_sha256="1" * 64,
                )
                run_path = logger.finalize(summary)
                return SimpleNamespace(metric_summary=metric, run_path=run_path, summary=summary)

            with patch("navbench.campaign.run_closed_loop", side_effect=fake_run):
                result = run_campaign(
                    self.scenario,
                    native_executable=executable,
                    output_root=root,
                    campaign_name="campaign",
                    seeds=(1, 2),
                )
            self.assertEqual(result.expected_runs, 6)
            self.assertEqual(result.completed_runs, 6)
            self.assertEqual(result.failed_runs, 0)
            self.assertEqual(result.aggregate["gnss_aided"]["runs"], 2)
            self.assertEqual(result.aggregate["gnss_aided"]["nis_mean_mean"], 1.0)
            self.assertEqual(result.aggregate["gnss_aided"]["nis_max_max"], 2.0)
            self.assertEqual(result.aggregate["gnss_aided"]["session_tx_packets_total"], 20.0)
            self.assertEqual(result.aggregate["gnss_aided"]["parser_crc_errors_total"], 3.0)
            self.assertEqual(result.aggregate["gnss_aided"]["nis_evaluated_count_total"], 24.0)
            self.assertTrue(result.acceptance_passed)
            summary = json.loads(
                (result.path / "campaign_summary.json").read_text(encoding="utf-8")
            )
            self.assertEqual(summary["status"], "complete")
            missing_name = summary["records"][0]["run_name"]
            (result.path / missing_name / "manifest.json").unlink()
            inspected = inspect_campaign(result.path)
            self.assertEqual(inspected["artifact_missing_runs"], [missing_name])
            self.assertFalse(inspected["acceptance_passed"])

    def test_metric_failure_and_landmark_comparison_are_acceptance_failures(self) -> None:
        from navbench.campaign import _acceptance_failures

        names = ("landmark", "dead")
        records = [
            {"run_name": "landmark", "status": "complete", "success": False},
            {"run_name": "dead", "status": "complete", "success": True},
        ]
        failures = _acceptance_failures(
            expected_names=names,
            records=records,
            missing_runs=(), artifact_missing_runs=(),
            artifact_incomplete_runs=(), artifact_invalid_runs=(),
            comparison={"landmark_lower_mean_position_rmse": False},
        )
        self.assertIn("metric_failed:landmark", failures)
        self.assertIn(
            "landmark_mean_position_rmse_not_better_than_dead_reckoning", failures
        )

    def test_strict_inspection_rejects_truncated_complete_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "campaign"
            path.mkdir()
            name = "run"
            (path / "campaign_config.json").write_text(
                json.dumps({"expected_runs": [name]}), encoding="utf-8"
            )
            logger = RunLogger(
                path, self.scenario, run_name=name, git_commit="test",
                git_dirty=False, source_tree_sha256="0" * 64,
            )
            run_path = logger.finalize({"success": True})
            with (run_path / "commands.csv").open("a", encoding="utf-8") as output:
                output.write("corrupt,row\n")
            (path / "campaign_summary.json").write_text(
                json.dumps({"acceptance_passed": True, "acceptance_failures": []}),
                encoding="utf-8",
            )
            inspected = inspect_campaign(path)
            self.assertEqual(inspected["artifact_invalid_runs"], [name])
            self.assertFalse(inspected["acceptance_passed"])

    def test_interrupted_campaign_is_inspectable_before_summary_exists(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "interrupted"
            path.mkdir()
            (path / "INCOMPLETE").write_text("campaign is incomplete\n", encoding="ascii")
            (path / "campaign_config.json").write_text(
                json.dumps(
                    {
                        "expected_runs": ["done", "partial", "not_started"],
                    }
                ),
                encoding="utf-8",
            )
            done = path / "done"
            done.mkdir()
            (done / "manifest.json").write_text(
                '{"status":"complete"}', encoding="ascii"
            )
            partial = path / "partial"
            partial.mkdir()
            (partial / "INCOMPLETE").write_text("run is incomplete\n", encoding="ascii")

            inspected = inspect_campaign(path)

            self.assertEqual(inspected["status"], "incomplete")
            self.assertFalse(inspected["campaign_summary_present"])
            self.assertTrue(inspected["campaign_incomplete"])
            self.assertEqual(inspected["completed_runs"], 0)
            self.assertEqual(
                inspected["artifact_missing_runs"], ["not_started", "partial"]
            )
            self.assertEqual(inspected["artifact_incomplete_runs"], ["partial"])
            self.assertEqual(inspected["artifact_invalid_runs"], ["done"])

    def test_invalid_seed_and_existing_campaign_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            executable = root / "native"
            executable.write_bytes(b"test")
            with self.assertRaises(ValueError):
                run_campaign(
                    self.scenario,
                    native_executable=executable,
                    output_root=root,
                    campaign_name="campaign",
                    seeds=(1, 1),
                )
            (root / "campaign").mkdir()
            with self.assertRaises(FileExistsError):
                run_campaign(
                    self.scenario,
                    native_executable=executable,
                    output_root=root,
                    campaign_name="campaign",
                    seeds=(1,),
                )


if __name__ == "__main__":
    unittest.main()
