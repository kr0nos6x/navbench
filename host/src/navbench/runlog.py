"""Recoverable NavBench run artifacts and hardware-free measurement replay."""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
import subprocess
import tempfile
from collections.abc import Callable, Iterator, Mapping
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from enum import Enum
from pathlib import Path
from typing import Any, TextIO

import yaml

from navbench import __version__
from navbench.scenario import Scenario, scenario_to_mapping
from navbench.sensors import (
    GnssMeasurement,
    ImuMeasurement,
    LandmarkMeasurement,
    LandmarkObservation,
    Measurement,
    SensorEvent,
    WheelSpeedMeasurement,
    measurement_kind,
)
from navbench.simulator import ControlCommand, SimulationSample, VehicleState


RUN_FORMAT_VERSION = 1
_INCOMPLETE_MARKER = "INCOMPLETE"


class RunLogError(RuntimeError):
    pass


class IncompleteRunError(RunLogError):
    pass


class CorruptRunError(RunLogError):
    pass


class CommandMode(str, Enum):
    """Semantic origin/mode of an actuator command stored in a run artifact."""

    OPEN_LOOP = "open_loop"
    NEUTRAL = "neutral"
    TRACKING = "tracking"
    SAFE_STOP = "safe_stop"


@dataclass(frozen=True, slots=True)
class RunEvent:
    step_id: int
    time_s: float
    event_type: str
    source: str
    detail: str = ""
    severity: str = "info"

    def __post_init__(self) -> None:
        _validate_step_time(self.step_id, self.time_s)
        if not all(
            isinstance(value, str)
            for value in (self.event_type, self.source, self.detail, self.severity)
        ):
            raise TypeError("event text fields must be strings")
        if not self.event_type or not self.source:
            raise ValueError("event_type and source cannot be empty")
        if self.severity not in {"debug", "info", "warning", "error", "critical"}:
            raise ValueError("event severity is invalid")


@dataclass(frozen=True, slots=True)
class CommandRecord:
    step_id: int
    time_s: float
    command: ControlCommand
    target_speed_mps: float
    mode: CommandMode
    flags: int
    source: str


@dataclass(frozen=True, slots=True)
class TimingRecord:
    step_id: int
    time_s: float
    component: str
    duration_s: float


@dataclass(frozen=True, slots=True)
class RunStatus:
    path: Path
    status: str
    scenario_name: str | None
    config_sha256: str | None


def serialize_measurement(measurement: Measurement) -> dict[str, Any]:
    """Serialize a typed measurement without exposing plant state."""

    result: dict[str, Any] = {
        "type": measurement_kind(measurement),
        "sample_step_id": measurement.sample_step_id,
        "sample_time_s": measurement.sample_time_s,
        "delivery_step_id": measurement.delivery_step_id,
        "delivery_time_s": measurement.delivery_time_s,
    }
    if isinstance(measurement, ImuMeasurement):
        result.update(
            acceleration_mps2=measurement.acceleration_mps2,
            yaw_rate_rad_s=measurement.yaw_rate_rad_s,
        )
    elif isinstance(measurement, WheelSpeedMeasurement):
        result["speed_mps"] = measurement.speed_mps
    elif isinstance(measurement, GnssMeasurement):
        result.update(x_m=measurement.x_m, y_m=measurement.y_m)
    elif isinstance(measurement, LandmarkMeasurement):
        result["observations"] = [asdict(item) for item in measurement.observations]
    else:  # pragma: no cover - measurement_kind already rejects this
        raise TypeError(f"unsupported measurement: {type(measurement).__name__}")
    _json_text(result)
    return result


def deserialize_measurement(record: Mapping[str, Any]) -> Measurement:
    """Deserialize and strictly validate a runlog measurement record."""

    kind = record.get("type")
    common = {
        "sample_step_id": _record_int(record, "sample_step_id"),
        "sample_time_s": _record_float(record, "sample_time_s"),
        "delivery_step_id": _record_int(record, "delivery_step_id"),
        "delivery_time_s": _record_float(record, "delivery_time_s"),
    }
    if common["sample_step_id"] < 0 or common["delivery_step_id"] < common["sample_step_id"]:
        raise CorruptRunError("measurement step ids are invalid")
    if common["sample_time_s"] < 0.0 or common["delivery_time_s"] < common["sample_time_s"]:
        raise CorruptRunError("measurement times are invalid")

    if kind == "imu":
        _require_fields(record, set(common) | {"type", "acceleration_mps2", "yaw_rate_rad_s"})
        return ImuMeasurement(
            **common,
            acceleration_mps2=_record_float(record, "acceleration_mps2"),
            yaw_rate_rad_s=_record_float(record, "yaw_rate_rad_s"),
        )
    if kind == "wheel_speed":
        _require_fields(record, set(common) | {"type", "speed_mps"})
        speed = _record_float(record, "speed_mps")
        if speed < 0.0:
            raise CorruptRunError("wheel speed cannot be negative")
        return WheelSpeedMeasurement(**common, speed_mps=speed)
    if kind == "gnss":
        _require_fields(record, set(common) | {"type", "x_m", "y_m"})
        return GnssMeasurement(
            **common,
            x_m=_record_float(record, "x_m"),
            y_m=_record_float(record, "y_m"),
        )
    if kind == "landmark":
        _require_fields(record, set(common) | {"type", "observations"})
        raw_observations = record["observations"]
        if not isinstance(raw_observations, list):
            raise CorruptRunError("landmark observations must be a list")
        observations: list[LandmarkObservation] = []
        for raw in raw_observations:
            if not isinstance(raw, dict):
                raise CorruptRunError("landmark observation must be a mapping")
            _require_fields(raw, {"landmark_id", "range_m", "bearing_rad", "is_outlier"})
            landmark_id = _record_int(raw, "landmark_id")
            range_m = _record_float(raw, "range_m")
            bearing = _record_float(raw, "bearing_rad")
            is_outlier = raw["is_outlier"]
            if landmark_id < 0 or landmark_id > 65535 or range_m < 0.0:
                raise CorruptRunError("landmark observation contains an invalid id or range")
            if not isinstance(is_outlier, bool):
                raise CorruptRunError("landmark is_outlier must be a boolean")
            observations.append(LandmarkObservation(landmark_id, range_m, bearing, is_outlier))
        return LandmarkMeasurement(**common, observations=tuple(observations))
    raise CorruptRunError(f"unknown measurement type: {kind!r}")


class RunLogger:
    """Append-only artifact writer with explicit incomplete/complete states."""

    GROUND_TRUTH_FIELDS = (
        "step_id", "time_s", "x_m", "y_m", "heading_rad", "speed_mps",
        "steering_rad", "acceleration_mps2", "command_acceleration_mps2",
        "command_steering_rad",
    )
    COMMAND_FIELDS = (
        "step_id", "time_s", "acceleration_mps2", "steering_rad",
        "target_speed_mps", "mode", "flags", "source",
    )
    TIMING_FIELDS = ("step_id", "time_s", "component", "duration_s")

    def __init__(
        self,
        output_root: Path,
        scenario: Scenario,
        *,
        run_name: str | None = None,
        software_version: str | None = None,
        git_commit: str | None = None,
        git_dirty: bool | None = None,
        source_tree_sha256: str | None = None,
        controller_binary_sha256: str | None = None,
    ) -> None:
        name = run_name or scenario.name
        if not name or Path(name).name != name or name in {".", ".."}:
            raise ValueError("run_name must be a single safe path component")
        if git_dirty is not None and not isinstance(git_dirty, bool):
            raise TypeError("git_dirty must be a boolean or None")
        source_tree_sha256 = _optional_sha256(
            source_tree_sha256, "source_tree_sha256"
        )
        controller_binary_sha256 = _optional_sha256(
            controller_binary_sha256, "controller_binary_sha256"
        )
        resolved_git_dirty = git_dirty if git_dirty is not None else detect_git_dirty()
        resolved_source_tree_sha256 = (
            source_tree_sha256
            if source_tree_sha256 is not None
            else detect_source_tree_sha256()
        )
        output_root.mkdir(parents=True, exist_ok=True)
        self.path = output_root / name
        self.path.mkdir(exist_ok=False)
        self._closed = False
        self._counts = {
            "ground_truth": 0,
            "measurements": 0,
            "estimates": 0,
            "commands": 0,
            "events": 0,
            "timing": 0,
        }
        self._config = scenario_to_mapping(scenario)
        self._config_sha256 = config_sha256(self._config)
        self._manifest: dict[str, Any] = {
            "run_format_version": RUN_FORMAT_VERSION,
            "status": "incomplete",
            "scenario_name": scenario.name,
            "seed": scenario.seed,
            "config_sha256": self._config_sha256,
            "software_version": software_version or _software_version(),
            "git_commit": git_commit if git_commit is not None else detect_git_commit(),
            "git_dirty": resolved_git_dirty,
            "source_tree_sha256": resolved_source_tree_sha256,
            "controller_binary_sha256": controller_binary_sha256,
            "started_utc": _utc_now(),
        }
        _atomic_write_text(self.path / _INCOMPLETE_MARKER, "run is incomplete\n")
        _atomic_write_text(
            self.path / "config.yaml",
            yaml.safe_dump(self._config, sort_keys=True, allow_unicode=True),
        )
        _atomic_write_json(self.path / "manifest.json", self._manifest)
        _atomic_write_json(self.path / "summary.json", {"status": "incomplete"})

        self._ground_truth_file, self._ground_truth = self._open_csv(
            "ground_truth.csv", self.GROUND_TRUTH_FIELDS
        )
        self._command_file, self._commands = self._open_csv(
            "commands.csv", self.COMMAND_FIELDS
        )
        self._timing_file, self._timing = self._open_csv(
            "timing.csv", self.TIMING_FIELDS
        )
        self._measurements = self._open_text("measurements.jsonl")
        self._estimates = self._open_text("estimates.jsonl")
        self._events = self._open_text("events.jsonl")

    @property
    def is_complete(self) -> bool:
        return self._closed and self._manifest.get("status") == "complete"

    def record_ground_truth(self, sample: SimulationSample) -> None:
        self._ensure_open()
        self._ground_truth.writerow(
            {
                "step_id": sample.step_id,
                "time_s": sample.time_s,
                "x_m": sample.state.x_m,
                "y_m": sample.state.y_m,
                "heading_rad": sample.state.heading_rad,
                "speed_mps": sample.state.speed_mps,
                "steering_rad": sample.state.steering_rad,
                "acceleration_mps2": sample.state.acceleration_mps2,
                "command_acceleration_mps2": sample.command.acceleration_mps2,
                "command_steering_rad": sample.command.steering_rad,
            }
        )
        self._ground_truth_file.flush()
        self._counts["ground_truth"] += 1

    add_ground_truth = record_ground_truth

    def record_measurement(self, measurement: Measurement) -> None:
        self._ensure_open()
        self._write_jsonl(self._measurements, serialize_measurement(measurement))
        self._counts["measurements"] += 1

    add_measurement = record_measurement

    def record_estimate(self, record: Mapping[str, Any]) -> None:
        self._ensure_open()
        normalized = _normalized_record(record, "estimate")
        if "step_id" not in normalized or "time_s" not in normalized:
            raise ValueError("estimate record requires step_id and time_s")
        self._write_jsonl(self._estimates, normalized)
        self._counts["estimates"] += 1

    add_estimate = record_estimate

    def record_command(
        self,
        step_id: int,
        time_s: float,
        command: ControlCommand,
        *,
        source: str,
        target_speed_mps: float = 0.0,
        mode: CommandMode = CommandMode.NEUTRAL,
        flags: int = 0,
    ) -> None:
        self._ensure_open()
        _validate_step_time(step_id, time_s)
        target_speed_mps = _finite(target_speed_mps, "target_speed_mps")
        if not isinstance(source, str) or not source:
            raise ValueError("command source cannot be empty")
        if target_speed_mps < 0.0:
            raise ValueError("target_speed_mps cannot be negative")
        if not isinstance(mode, CommandMode):
            raise TypeError("command mode must be a CommandMode")
        if isinstance(flags, bool) or not isinstance(flags, int) or not 0 <= flags <= 255:
            raise ValueError("command flags must be an integer in [0, 255]")
        self._commands.writerow(
            {
                "step_id": step_id,
                "time_s": time_s,
                "acceleration_mps2": command.acceleration_mps2,
                "steering_rad": command.steering_rad,
                "target_speed_mps": target_speed_mps,
                "mode": mode.value,
                "flags": flags,
                "source": source,
            }
        )
        self._command_file.flush()
        self._counts["commands"] += 1

    add_command = record_command

    def record_event(self, event: RunEvent | SensorEvent | Mapping[str, Any]) -> None:
        self._ensure_open()
        if isinstance(event, RunEvent):
            record = asdict(event)
        elif isinstance(event, SensorEvent):
            record = {
                "step_id": event.step_id,
                "time_s": event.time_s,
                "event_type": event.event_type,
                "source": event.sensor,
                "detail": event.detail,
                "severity": "info",
            }
        else:
            record = _normalized_record(event, "event")
            required = {"step_id", "time_s", "event_type", "source"}
            missing = required - set(record)
            if missing:
                raise ValueError(f"event is missing fields: {', '.join(sorted(missing))}")
            record.setdefault("detail", "")
            record.setdefault("severity", "info")
            RunEvent(
                step_id=record["step_id"],
                time_s=record["time_s"],
                event_type=record["event_type"],
                source=record["source"],
                detail=record["detail"],
                severity=record["severity"],
            )
        self._write_jsonl(self._events, record)
        self._counts["events"] += 1

    add_event = record_event

    def record_timing(
        self, step_id: int, time_s: float, component: str, duration_s: float
    ) -> None:
        self._ensure_open()
        _validate_step_time(step_id, time_s)
        duration_s = _finite(duration_s, "duration_s")
        if not isinstance(component, str) or duration_s < 0.0 or not component:
            raise ValueError("timing component must be set and duration cannot be negative")
        self._timing.writerow(
            {
                "step_id": step_id,
                "time_s": time_s,
                "component": component,
                "duration_s": duration_s,
            }
        )
        self._timing_file.flush()
        self._counts["timing"] += 1

    add_timing = record_timing

    def checkpoint(self) -> None:
        self._ensure_open()
        for file in self._files:
            file.flush()
            os.fsync(file.fileno())

    def finalize(self, summary: Mapping[str, Any]) -> Path:
        self._ensure_open()
        normalized_summary = _normalized_record(summary, "summary")
        self.checkpoint()
        self._close_files()
        final_summary = {**normalized_summary, "status": "complete"}
        _atomic_write_json(self.path / "summary.json", final_summary)
        self._manifest.update(
            status="complete",
            completed_utc=_utc_now(),
            record_counts=self._counts.copy(),
        )
        _atomic_write_json(self.path / "manifest.json", self._manifest)
        (self.path / _INCOMPLETE_MARKER).unlink()
        self._closed = True
        return self.path

    def close_incomplete(self, reason: str | None = None) -> None:
        if self._closed:
            return
        for file in self._files:
            file.flush()
        self._close_files()
        if reason:
            self._manifest["incomplete_reason"] = reason
        self._manifest["record_counts"] = self._counts.copy()
        _atomic_write_json(self.path / "manifest.json", self._manifest)
        self._closed = True

    def __enter__(self) -> RunLogger:
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        if not self._closed:
            self.close_incomplete(str(exc) if exc else "finalize was not called")

    @property
    def _files(self) -> tuple[TextIO, ...]:
        return (
            self._ground_truth_file,
            self._command_file,
            self._timing_file,
            self._measurements,
            self._estimates,
            self._events,
        )

    def _open_csv(self, name: str, fields: tuple[str, ...]) -> tuple[TextIO, csv.DictWriter]:
        file = self._open_text(name, newline="")
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        file.flush()
        return file, writer

    def _open_text(self, name: str, newline: str | None = None) -> TextIO:
        return (self.path / name).open(
            "x", encoding="utf-8", newline=newline, buffering=1
        )

    @staticmethod
    def _write_jsonl(file: TextIO, record: Mapping[str, Any]) -> None:
        file.write(_json_text(record) + "\n")
        file.flush()

    def _close_files(self) -> None:
        for file in self._files:
            if not file.closed:
                file.close()

    def _ensure_open(self) -> None:
        if self._closed:
            raise RunLogError("run logger is closed")


class RunReplay:
    """Read a complete run or explicitly inspect a recoverable partial run."""

    def __init__(self, path: Path, *, allow_incomplete: bool = False) -> None:
        self.path = path
        self.allow_incomplete = allow_incomplete
        try:
            self.manifest = _read_json(path / "manifest.json")
        except (OSError, json.JSONDecodeError) as error:
            raise CorruptRunError("manifest is missing or invalid") from error
        if self.manifest.get("run_format_version") != RUN_FORMAT_VERSION:
            raise CorruptRunError("unsupported run format version")
        incomplete = (
            self.manifest.get("status") != "complete"
            or (path / _INCOMPLETE_MARKER).exists()
        )
        if incomplete and not allow_incomplete:
            raise IncompleteRunError(f"run is incomplete: {path}")
        try:
            with (path / "config.yaml").open(encoding="utf-8") as file:
                self.config = yaml.safe_load(file)
        except (OSError, yaml.YAMLError) as error:
            raise CorruptRunError("config snapshot is missing or invalid") from error
        if config_sha256(self.config) != self.manifest.get("config_sha256"):
            raise CorruptRunError("config snapshot hash does not match manifest")
        if self.is_complete:
            self._validate_complete_artifacts()

    @property
    def is_complete(self) -> bool:
        return self.manifest.get("status") == "complete" and not (
            self.path / _INCOMPLETE_MARKER
        ).exists()

    def measurements(self) -> Iterator[Measurement]:
        for record in self._jsonl("measurements.jsonl"):
            yield deserialize_measurement(record)

    def measurement_batches(self) -> Iterator[tuple[int, tuple[Measurement, ...]]]:
        current_step: int | None = None
        batch: list[Measurement] = []
        for measurement in self.measurements():
            if current_step is None:
                current_step = measurement.delivery_step_id
            if measurement.delivery_step_id < current_step:
                raise CorruptRunError("measurements are not ordered by delivery step")
            if measurement.delivery_step_id != current_step:
                yield current_step, tuple(batch)
                current_step = measurement.delivery_step_id
                batch = []
            batch.append(measurement)
        if current_step is not None:
            yield current_step, tuple(batch)

    def replay_measurements(self, consumer: Callable[[Measurement], None]) -> int:
        count = 0
        for measurement in self.measurements():
            consumer(measurement)
            count += 1
        return count

    def ground_truth(self) -> Iterator[SimulationSample]:
        try:
            file = (self.path / "ground_truth.csv").open(newline="", encoding="utf-8")
        except OSError as error:
            raise CorruptRunError("ground_truth.csv is missing") from error
        with file:
            reader = csv.DictReader(file)
            if tuple(reader.fieldnames or ()) != RunLogger.GROUND_TRUTH_FIELDS:
                raise CorruptRunError("ground_truth.csv header is invalid")
            for row in reader:
                try:
                    yield SimulationSample(
                        step_id=int(row["step_id"]),
                        time_s=float(row["time_s"]),
                        state=VehicleState(
                            x_m=float(row["x_m"]),
                            y_m=float(row["y_m"]),
                            heading_rad=float(row["heading_rad"]),
                            speed_mps=float(row["speed_mps"]),
                            steering_rad=float(row["steering_rad"]),
                            acceleration_mps2=float(row["acceleration_mps2"]),
                        ),
                        command=ControlCommand(
                            acceleration_mps2=float(row["command_acceleration_mps2"]),
                            steering_rad=float(row["command_steering_rad"]),
                        ),
                    )
                except (KeyError, TypeError, ValueError) as error:
                    raise CorruptRunError("ground_truth.csv contains an invalid row") from error

    def events(self) -> Iterator[dict[str, Any]]:
        yield from self._jsonl("events.jsonl")

    def estimates(self) -> Iterator[dict[str, Any]]:
        yield from self._jsonl("estimates.jsonl")

    def commands(self) -> Iterator[CommandRecord]:
        for row in self._csv_rows("commands.csv", RunLogger.COMMAND_FIELDS):
            try:
                step_id = int(row["step_id"])
                time_s = float(row["time_s"])
                source = row["source"]
                _validate_step_time(step_id, time_s)
                if not source:
                    raise ValueError("empty source")
                target_speed_mps = float(row["target_speed_mps"])
                if not math.isfinite(target_speed_mps) or target_speed_mps < 0.0:
                    raise ValueError("invalid target speed")
                flags = int(row["flags"])
                if flags < 0 or flags > 255 or str(flags) != row["flags"]:
                    raise ValueError("invalid flags")
                yield CommandRecord(
                    step_id=step_id,
                    time_s=time_s,
                    command=ControlCommand(
                        acceleration_mps2=float(row["acceleration_mps2"]),
                        steering_rad=float(row["steering_rad"]),
                    ),
                    target_speed_mps=target_speed_mps,
                    mode=CommandMode(row["mode"]),
                    flags=flags,
                    source=source,
                )
            except (KeyError, TypeError, ValueError) as error:
                raise CorruptRunError("commands.csv contains an invalid row") from error

    def timing(self) -> Iterator[TimingRecord]:
        for row in self._csv_rows("timing.csv", RunLogger.TIMING_FIELDS):
            try:
                record = TimingRecord(
                    step_id=int(row["step_id"]),
                    time_s=float(row["time_s"]),
                    component=row["component"],
                    duration_s=float(row["duration_s"]),
                )
                _validate_step_time(record.step_id, record.time_s)
                if not record.component or not math.isfinite(record.duration_s) or record.duration_s < 0.0:
                    raise ValueError("invalid timing")
                yield record
            except (KeyError, TypeError, ValueError) as error:
                raise CorruptRunError("timing.csv contains an invalid row") from error

    def summary(self) -> dict[str, Any]:
        try:
            return _read_json(self.path / "summary.json")
        except (OSError, json.JSONDecodeError) as error:
            raise CorruptRunError("summary.json is missing or invalid") from error

    def _validate_complete_artifacts(self) -> None:
        """Reject a nominally complete run whose files disagree with its manifest."""

        expected_names = (
            "ground_truth",
            "measurements",
            "estimates",
            "commands",
            "events",
            "timing",
        )
        raw_counts = self.manifest.get("record_counts")
        if not isinstance(raw_counts, dict) or set(raw_counts) != set(expected_names):
            raise CorruptRunError("manifest record_counts fields are invalid")
        expected: dict[str, int] = {}
        for name in expected_names:
            value = raw_counts[name]
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise CorruptRunError(
                    f"manifest record_counts.{name} must be a non-negative integer"
                )
            expected[name] = value

        actual = {
            "ground_truth": sum(1 for _ in self.ground_truth()),
            "measurements": sum(1 for _ in self.measurements()),
            "estimates": sum(1 for _ in self.estimates()),
            "commands": sum(1 for _ in self.commands()),
            "events": sum(1 for _ in self.events()),
            "timing": sum(1 for _ in self.timing()),
        }
        mismatches = [
            f"{name}: manifest={expected[name]}, file={actual[name]}"
            for name in expected_names
            if expected[name] != actual[name]
        ]
        if mismatches:
            raise CorruptRunError(
                "run artifact record count mismatch (" + "; ".join(mismatches) + ")"
            )
        if self.summary().get("status") != "complete":
            raise CorruptRunError("complete run summary status is not complete")

    def _jsonl(self, name: str) -> Iterator[dict[str, Any]]:
        try:
            file = (self.path / name).open(encoding="utf-8")
        except OSError as error:
            raise CorruptRunError(f"{name} is missing") from error
        with file:
            line = file.readline()
            line_number = 0
            while line:
                line_number += 1
                next_line = file.readline()
                if not line.strip():
                    line = next_line
                    continue
                try:
                    record = json.loads(line)
                except json.JSONDecodeError as error:
                    if self.allow_incomplete and not next_line and not line.endswith("\n"):
                        return
                    raise CorruptRunError(f"{name}:{line_number} is invalid JSON") from error
                if not isinstance(record, dict):
                    raise CorruptRunError(f"{name}:{line_number} is not a JSON object")
                yield record
                line = next_line

    def _csv_rows(
        self, name: str, expected_fields: tuple[str, ...]
    ) -> Iterator[dict[str, str]]:
        try:
            file = (self.path / name).open(newline="", encoding="utf-8")
        except OSError as error:
            raise CorruptRunError(f"{name} is missing") from error
        with file:
            reader = csv.DictReader(file)
            if tuple(reader.fieldnames or ()) != expected_fields:
                raise CorruptRunError(f"{name} header is invalid")
            yield from reader


def discover_runs(output_root: Path) -> list[RunStatus]:
    if not output_root.exists():
        return []
    result: list[RunStatus] = []
    for path in sorted(item for item in output_root.iterdir() if item.is_dir()):
        try:
            manifest = _read_json(path / "manifest.json")
            status = str(manifest.get("status", "invalid"))
            if (path / _INCOMPLETE_MARKER).exists():
                status = "incomplete"
            result.append(
                RunStatus(
                    path=path,
                    status=status,
                    scenario_name=manifest.get("scenario_name"),
                    config_sha256=manifest.get("config_sha256"),
                )
            )
        except (OSError, json.JSONDecodeError):
            result.append(RunStatus(path, "invalid", None, None))
    return result


def config_sha256(config: object) -> str:
    return hashlib.sha256(_json_text(config).encode("utf-8")).hexdigest()


def detect_git_commit(repository_path: Path | None = None) -> str | None:
    """Return HEAD without modifying Git state, or ``None`` outside a repository."""

    try:
        result = subprocess.run(
            ["git", "rev-parse", "--verify", "HEAD"],
            cwd=repository_path,
            env={**os.environ, "GIT_OPTIONAL_LOCKS": "0"},
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=2.0,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    commit = result.stdout.strip()
    if len(commit) == 40 and all(character in "0123456789abcdef" for character in commit.lower()):
        return commit
    return None


def detect_git_dirty(repository_path: Path | None = None) -> bool | None:
    """Return whether tracked or untracked source state differs from ``HEAD``."""

    try:
        result = subprocess.run(
            ["git", "status", "--porcelain=v1", "--untracked-files=normal"],
            cwd=repository_path,
            env={**os.environ, "GIT_OPTIONAL_LOCKS": "0"},
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=3.0,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return bool(result.stdout)


def detect_source_tree_sha256(repository_path: Path | None = None) -> str | None:
    """Hash relevant tracked and untracked repository file contents deterministically."""

    excluded_parts = {
        ".git",
        ".pio",
        ".venv",
        ".pytest_cache",
        ".mypy_cache",
        ".ruff_cache",
        "__pycache__",
        "build",
        "runs",
        "venv",
    }
    environment = {**os.environ, "GIT_OPTIONAL_LOCKS": "0"}
    try:
        root_result = subprocess.run(
            ["git", "rev-parse", "--show-toplevel"],
            cwd=repository_path,
            env=environment,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=3.0,
        )
        root = Path(root_result.stdout.strip())
        files_result = subprocess.run(
            ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
            cwd=root,
            env=environment,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=5.0,
        )
    except (OSError, subprocess.SubprocessError):
        return None

    relative_paths: list[Path] = []
    for raw_path in files_result.stdout.split(b"\0"):
        if not raw_path:
            continue
        relative = Path(os.fsdecode(raw_path))
        if relative.is_absolute() or ".." in relative.parts:
            return None
        if any(part in excluded_parts for part in relative.parts):
            continue
        relative_paths.append(relative)

    digest = hashlib.sha256()
    try:
        for relative in sorted(relative_paths, key=lambda item: item.as_posix()):
            path_bytes = os.fsencode(relative.as_posix())
            full_path = root / relative
            digest.update(len(path_bytes).to_bytes(8, "little"))
            digest.update(path_bytes)
            if full_path.is_symlink():
                target = os.fsencode(os.readlink(full_path))
                digest.update(b"L")
                digest.update(len(target).to_bytes(8, "little"))
                digest.update(target)
                continue
            if not full_path.is_file():
                # Deleted tracked files remain in ``git ls-files`` and must affect identity.
                digest.update(b"M")
                continue
            initial_stat = full_path.stat()
            digest.update(b"F")
            digest.update(initial_stat.st_size.to_bytes(8, "little"))
            with full_path.open("rb") as source:
                while chunk := source.read(1024 * 1024):
                    digest.update(chunk)
            final_stat = full_path.stat()
            if (
                initial_stat.st_size != final_stat.st_size
                or initial_stat.st_mtime_ns != final_stat.st_mtime_ns
            ):
                return None
    except (OSError, OverflowError):
        return None
    return digest.hexdigest()


def _software_version() -> str:
    return __version__


def _utc_now() -> str:
    return datetime.now(UTC).isoformat(timespec="milliseconds")


def _json_text(value: object) -> str:
    try:
        return json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        )
    except (TypeError, ValueError) as error:
        raise ValueError("record must contain finite JSON-compatible values") from error


def _optional_sha256(value: str | None, name: str) -> str | None:
    if value is None:
        return None
    if not isinstance(value, str):
        raise TypeError(f"{name} must be a hexadecimal string or None")
    normalized = value.lower()
    if len(normalized) != 64 or any(character not in "0123456789abcdef" for character in normalized):
        raise ValueError(f"{name} must contain exactly 64 hexadecimal characters")
    return normalized


def _atomic_write_json(path: Path, value: object) -> None:
    _atomic_write_text(path, _json_text(value) + "\n")


def _atomic_write_text(path: Path, text: str) -> None:
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.tmp-", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as file:
            file.write(text)
            file.flush()
            os.fsync(file.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def _read_json(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as file:
        value = json.load(file)
    if not isinstance(value, dict):
        raise json.JSONDecodeError("expected JSON object", "", 0)
    return value


def _record_int(record: Mapping[str, Any], key: str) -> int:
    value = record.get(key)
    if isinstance(value, bool) or not isinstance(value, int):
        raise CorruptRunError(f"{key} must be an integer")
    return value


def _record_float(record: Mapping[str, Any], key: str) -> float:
    try:
        return _finite(record[key], key)
    except (KeyError, TypeError, ValueError) as error:
        raise CorruptRunError(f"{key} must be finite") from error


def _require_fields(record: Mapping[str, Any], expected: set[str]) -> None:
    actual = set(record)
    if actual != expected:
        raise CorruptRunError(
            f"record fields differ; missing={sorted(expected - actual)}, unknown={sorted(actual - expected)}"
        )


def _normalized_record(record: Mapping[str, Any], name: str) -> dict[str, Any]:
    if not isinstance(record, Mapping):
        raise TypeError(f"{name} must be a mapping")
    normalized = dict(record)
    _json_text(normalized)
    return normalized


def _validate_step_time(step_id: int, time_s: float) -> None:
    if isinstance(step_id, bool) or not isinstance(step_id, int) or step_id < 0:
        raise ValueError("step_id must be a non-negative integer")
    time_s = _finite(time_s, "time_s")
    if time_s < 0.0:
        raise ValueError("time_s cannot be negative")


def _finite(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a finite real number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite real number")
    return result
