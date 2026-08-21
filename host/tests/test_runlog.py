from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path

from navbench.runlog import (
    CommandMode,
    CorruptRunError,
    IncompleteRunError,
    RunEvent,
    RunLogger,
    RunReplay,
    deserialize_measurement,
    discover_runs,
    serialize_measurement,
)
from navbench.scenario import load_scenario, run_scenario
from navbench.sensors import (
    GnssMeasurement,
    ImuMeasurement,
    LandmarkMeasurement,
    LandmarkObservation,
    SensorPipeline,
    WheelSpeedMeasurement,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


class MeasurementSerializationTests(unittest.TestCase):
    def test_all_measurement_types_round_trip(self) -> None:
        values = (
            ImuMeasurement(1, 0.02, 1, 0.02, 0.3, -0.2),
            WheelSpeedMeasurement(2, 0.04, 3, 0.06, 1.2),
            GnssMeasurement(3, 0.06, 8, 0.16, 4.0, -5.0),
            LandmarkMeasurement(
                4,
                0.08,
                6,
                0.12,
                (LandmarkObservation(7, 2.5, -0.4, True),),
            ),
        )
        for measurement in values:
            with self.subTest(measurement=type(measurement).__name__):
                self.assertEqual(
                    deserialize_measurement(serialize_measurement(measurement)),
                    measurement,
                )

    def test_corrupt_measurement_fields_are_rejected(self) -> None:
        record = serialize_measurement(GnssMeasurement(0, 0.0, 0, 0.0, 1.0, 2.0))
        record["extra"] = 1
        with self.assertRaises(CorruptRunError):
            deserialize_measurement(record)
        record = serialize_measurement(GnssMeasurement(0, 0.0, 0, 0.0, 1.0, 2.0))
        record["x_m"] = math.nan
        with self.assertRaises(CorruptRunError):
            deserialize_measurement(record)


class RunLogTests(unittest.TestCase):
    def setUp(self) -> None:
        self.scenario = load_scenario(REPOSITORY_ROOT / "scenarios" / "straight.yaml")
        self.temporary = tempfile.TemporaryDirectory(dir="/tmp")
        self.root = Path(self.temporary.name)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def test_complete_run_contains_all_artifacts_and_replays(self) -> None:
        samples = run_scenario(self.scenario)[:21]
        pipeline = SensorPipeline(
            dt_s=self.scenario.dt_s,
            vehicle_parameters=self.scenario.vehicle,
            config=self.scenario.sensors,
            landmarks=self.scenario.landmarks,
            faults=self.scenario.faults,
            seed=self.scenario.seed,
        )
        logger = RunLogger(
            self.root,
            self.scenario,
            git_commit="0" * 40,
            git_dirty=True,
            source_tree_sha256="1" * 64,
            controller_binary_sha256="2" * 64,
        )
        measurement_count = 0
        for sample in samples:
            logger.record_ground_truth(sample)
            logger.record_command(
                sample.step_id,
                sample.time_s,
                sample.command,
                source="embedded_cpp",
                target_speed_mps=1.25,
                mode=CommandMode.TRACKING,
                flags=3,
            )
            for measurement in pipeline.step(sample.step_id, sample.time_s, sample.state):
                logger.record_measurement(measurement)
                measurement_count += 1
            for event in pipeline.drain_events():
                logger.record_event(event)
            logger.record_timing(sample.step_id, sample.time_s, "simulation", 0.0001)
        logger.record_estimate({"step_id": 20, "time_s": 0.4, "x_m": 1.0})
        logger.record_event(RunEvent(20, 0.4, "smoke_complete", "test"))
        path = logger.finalize({"measurement_count": measurement_count})

        expected = {
            "manifest.json", "config.yaml", "ground_truth.csv", "measurements.jsonl",
            "estimates.jsonl", "commands.csv", "events.jsonl", "timing.csv", "summary.json",
        }
        self.assertEqual({item.name for item in path.iterdir()}, expected)
        replay = RunReplay(path)
        self.assertTrue(replay.is_complete)
        self.assertEqual(len(list(replay.ground_truth())), len(samples))
        self.assertEqual(len(list(replay.measurements())), measurement_count)
        command = next(replay.commands())
        self.assertEqual(command.target_speed_mps, 1.25)
        self.assertIs(command.mode, CommandMode.TRACKING)
        self.assertEqual(command.flags, 3)
        self.assertEqual(replay.summary()["measurement_count"], measurement_count)
        replayed = []
        self.assertEqual(replay.replay_measurements(replayed.append), measurement_count)
        self.assertEqual(replayed, list(replay.measurements()))
        manifest = json.loads((path / "manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(manifest["status"], "complete")
        self.assertEqual(manifest["record_counts"]["ground_truth"], len(samples))
        self.assertTrue(manifest["git_dirty"])
        self.assertEqual(manifest["source_tree_sha256"], "1" * 64)
        self.assertEqual(manifest["controller_binary_sha256"], "2" * 64)

    def test_complete_run_rejects_truncation_and_full_record_deletion(self) -> None:
        sample = run_scenario(self.scenario)[0]

        truncated = RunLogger(
            self.root,
            self.scenario,
            run_name="truncated",
            git_commit="0" * 40,
        )
        truncated.record_measurement(
            GnssMeasurement(0, 0.0, 0, 0.0, 1.0, 2.0)
        )
        truncated_path = truncated.finalize({})
        measurements_path = truncated_path / "measurements.jsonl"
        text = measurements_path.read_text(encoding="utf-8")
        measurements_path.write_text(text[:-4], encoding="utf-8")
        with self.assertRaisesRegex(CorruptRunError, "measurements.jsonl"):
            RunReplay(truncated_path)

        deleted = RunLogger(
            self.root,
            self.scenario,
            run_name="deleted-record",
            git_commit="0" * 40,
        )
        deleted.record_ground_truth(sample)
        deleted_path = deleted.finalize({})
        ground_truth_path = deleted_path / "ground_truth.csv"
        header = ground_truth_path.read_text(encoding="utf-8").splitlines(keepends=True)[0]
        ground_truth_path.write_text(header, encoding="utf-8")
        with self.assertRaisesRegex(CorruptRunError, "record count mismatch"):
            RunReplay(deleted_path)

    def test_complete_run_requires_exact_manifest_record_count_schema(self) -> None:
        logger = RunLogger(
            self.root,
            self.scenario,
            run_name="bad-counts",
            git_commit="0" * 40,
        )
        path = logger.finalize({})
        manifest_path = path / "manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest["record_counts"].pop("events")
        manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
        with self.assertRaisesRegex(CorruptRunError, "record_counts fields"):
            RunReplay(path)

    def test_existing_run_is_never_overwritten(self) -> None:
        first = RunLogger(self.root, self.scenario, git_commit="0" * 40)
        with self.assertRaises(FileExistsError):
            RunLogger(self.root, self.scenario, git_commit="0" * 40)
        first.close_incomplete("test complete")

    def test_incomplete_run_is_detected_and_can_be_inspected(self) -> None:
        logger = RunLogger(self.root, self.scenario, git_commit="0" * 40)
        sample = run_scenario(self.scenario)[0]
        logger.record_ground_truth(sample)
        logger.close_incomplete("simulated interruption")
        with self.assertRaises(IncompleteRunError):
            RunReplay(logger.path)
        replay = RunReplay(logger.path, allow_incomplete=True)
        self.assertFalse(replay.is_complete)
        self.assertEqual(len(list(replay.ground_truth())), 1)
        statuses = discover_runs(self.root)
        self.assertEqual([(status.path.name, status.status) for status in statuses], [("straight", "incomplete")])

    def test_config_tampering_is_detected(self) -> None:
        logger = RunLogger(self.root, self.scenario, git_commit="0" * 40)
        path = logger.finalize({})
        with (path / "config.yaml").open("a", encoding="utf-8") as file:
            file.write("tampered: true\n")
        with self.assertRaisesRegex(CorruptRunError, "hash"):
            RunReplay(path)

    def test_truncated_jsonl_is_only_tolerated_for_explicit_partial_recovery(self) -> None:
        logger = RunLogger(self.root, self.scenario, git_commit="0" * 40)
        logger.record_measurement(GnssMeasurement(0, 0.0, 0, 0.0, 1.0, 2.0))
        logger.close_incomplete("power loss")
        with (logger.path / "measurements.jsonl").open("a", encoding="utf-8") as file:
            file.write('{"type":"gnss"')
        replay = RunReplay(logger.path, allow_incomplete=True)
        self.assertEqual(len(list(replay.measurements())), 1)

    def test_nonfinite_estimate_and_summary_are_rejected(self) -> None:
        logger = RunLogger(self.root, self.scenario, git_commit="0" * 40)
        with self.assertRaises(ValueError):
            logger.record_estimate({"step_id": 0, "time_s": 0.0, "x_m": math.nan})
        with self.assertRaises(ValueError):
            logger.finalize({"rmse": math.inf})
        logger.close_incomplete("invalid metric")


if __name__ == "__main__":
    unittest.main()
