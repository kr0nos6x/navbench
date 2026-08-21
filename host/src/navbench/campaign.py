"""Deterministic controller-in-the-loop experiment campaign management."""

from __future__ import annotations

import csv
import json
import math
import os
import tempfile
from collections import defaultdict
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Iterable

from navbench.cil import run_closed_loop
from navbench.runlog import CorruptRunError, IncompleteRunError, RunReplay
from navbench.scenario import Scenario
from navbench.sensors import FaultSpec


CAMPAIGN_FORMAT_VERSION = 1
DEFAULT_MODES = ("gnss_aided", "landmark_aided", "dead_reckoning")


@dataclass(frozen=True, slots=True)
class CampaignResult:
    path: Path
    expected_runs: int
    completed_runs: int
    failed_runs: int
    missing_runs: tuple[str, ...]
    aggregate: dict[str, dict[str, float | int | bool | None]]
    acceptance_passed: bool
    acceptance_failures: tuple[str, ...]
    artifact_missing_runs: tuple[str, ...]
    artifact_incomplete_runs: tuple[str, ...]
    artifact_invalid_runs: tuple[str, ...]


_SUMMARY_GROUPS = {
    "session_statistics": "session",
    "parser_statistics": "parser",
    "sequence_statistics": "sequence",
    "link_statistics": "link",
    "firmware_health_counters": "firmware_health",
}
_COUNTER_FIELDS = tuple(
    f"{prefix}_{name}"
    for prefix, names in (
        ("session", ("tx_packets", "tx_bytes", "rx_packets", "rx_bytes", "rx_rejected", "rx_payload_errors", "timeouts", "reconnects", "transport_errors")),
        ("parser", ("bytes_received", "frames_received", "packets_accepted", "empty_delimiters", "oversized_frames", "truncated_frames", "cobs_errors", "crc_errors", "version_errors", "type_errors", "length_errors", "other_errors")),
        ("sequence", ("accepted", "missing", "duplicates", "out_of_order", "stale")),
        ("link", ("frames_forwarded", "frames_dropped", "frames_corrupted", "frames_delayed", "stale_frames_replayed", "disconnect_drops", "rx_frames_forwarded", "rx_frames_dropped", "rx_frames_corrupted", "rx_frames_delayed", "rx_stale_frames_replayed", "rx_disconnect_drops")),
        ("firmware_health", ("rx_frames", "rx_crc_errors", "rx_decode_errors", "rx_missing", "rx_duplicates", "rx_out_of_order", "rx_stale", "queue_overflows", "scheduler_overruns", "max_loop_us")),
    )
    for name in names
)
_NIS_FIELDS = ("nis_evaluated_count", "nis_gate_rejected_count", "nis_sum", "nis_max")


def navigation_variant(base: Scenario, mode: str, seed: int) -> Scenario:
    """Create one comparable mode variant after a common GNSS initialization."""

    if mode not in DEFAULT_MODES:
        raise ValueError(f"unsupported navigation campaign mode: {mode}")
    if base.duration_s <= 1.0:
        raise ValueError("campaign scenarios must last longer than one second")
    faults = list(base.faults)
    # All variants see the same mild post-initialization odometry challenge;
    # only the aiding source differs. Values are synthetic campaign inputs,
    # never claims about physical sensor performance.
    faults.extend(
        (
            FaultSpec(
                "imu",
                "bias_yaw_rate_rad_s",
                1.0,
                base.duration_s,
                0.015,
            ),
            FaultSpec(
                "wheel_speed",
                "scale_error",
                1.0,
                base.duration_s,
                0.02,
            ),
        )
    )
    if mode in {"landmark_aided", "dead_reckoning"}:
        faults.append(FaultSpec("gnss", "dropout", 1.0, base.duration_s))
    if mode == "dead_reckoning":
        faults.append(FaultSpec("landmark", "dropout", 1.0, base.duration_s))
    return replace(base, seed=seed, faults=tuple(faults))


def run_campaign(
    base: Scenario,
    *,
    native_executable: Path,
    output_root: Path,
    campaign_name: str,
    seeds: Iterable[int],
    modes: tuple[str, ...] = DEFAULT_MODES,
    create_dashboards: bool = False,
) -> CampaignResult:
    seed_values = tuple(seeds)
    if not seed_values or len(set(seed_values)) != len(seed_values):
        raise ValueError("campaign seeds must be non-empty and unique")
    if any(isinstance(seed, bool) or not isinstance(seed, int) for seed in seed_values):
        raise TypeError("campaign seeds must be integers")
    if any(seed < 0 or seed > 2**64 - 1 for seed in seed_values):
        raise ValueError("campaign seeds must fit uint64")
    if not modes or len(set(modes)) != len(modes):
        raise ValueError("campaign modes must be non-empty and unique")
    for mode in modes:
        navigation_variant(base, mode, seed_values[0])
    if not campaign_name or Path(campaign_name).name != campaign_name:
        raise ValueError("campaign_name must be one safe path component")
    if not native_executable.is_file():
        raise ValueError("native firmware executable does not exist")

    output_root.mkdir(parents=True, exist_ok=True)
    path = output_root / campaign_name
    path.mkdir(exist_ok=False)
    marker = path / "INCOMPLETE"
    marker.write_text("campaign is incomplete\n", encoding="ascii")
    expected_names = tuple(
        _run_name(base.name, mode, seed) for mode in modes for seed in seed_values
    )
    config = {
        "campaign_format_version": CAMPAIGN_FORMAT_VERSION,
        "scenario": base.name,
        "modes": list(modes),
        "seeds": list(seed_values),
        "expected_runs": list(expected_names),
    }
    _atomic_json(path / "campaign_config.json", config)

    records: list[dict[str, object]] = []
    for mode in modes:
        for seed in seed_values:
            run_name = _run_name(base.name, mode, seed)
            scenario = navigation_variant(base, mode, seed)
            try:
                result = run_closed_loop(
                    scenario,
                    native_executable=native_executable,
                    output_root=path,
                    run_name=run_name,
                    create_dashboard=create_dashboards,
                )
            except Exception as error:
                records.append(
                    {
                        "run_name": run_name,
                        "mode": mode,
                        "seed": seed,
                        "status": "failed",
                        "error": f"{type(error).__name__}: {error}",
                    }
                )
                continue
            records.append(
                {
                    "run_name": run_name,
                    "mode": mode,
                    "seed": seed,
                    "status": "complete",
                    "run_path": result.run_path.name if result.run_path else run_name,
                    **result.metric_summary.to_dict(),
                    **_summary_measurements(result.summary),
                }
            )

    present = {str(record["run_name"]) for record in records}
    missing = tuple(name for name in expected_names if name not in present)
    artifact_status = {
        name: _run_artifact_status(path / name) for name in expected_names
    }
    for record in records:
        if record["status"] == "complete":
            status = artifact_status[str(record["run_name"])]
            if status != "complete":
                record["status"] = f"artifact_{status}"
    completed = [record for record in records if record["status"] == "complete"]
    failed = [record for record in records if record["status"] == "failed"]
    artifact_missing = tuple(
        name for name, status in artifact_status.items() if status == "missing"
    )
    artifact_incomplete = tuple(
        name for name, status in artifact_status.items() if status == "incomplete"
    )
    artifact_invalid = tuple(
        name for name, status in artifact_status.items() if status == "invalid"
    )
    aggregate = _aggregate(completed, modes)
    comparison = _landmark_dead_reckoning_comparison(completed, seed_values)
    acceptance_failures = _acceptance_failures(
        expected_names=expected_names,
        records=records,
        missing_runs=missing,
        artifact_missing_runs=artifact_missing,
        artifact_incomplete_runs=artifact_incomplete,
        artifact_invalid_runs=artifact_invalid,
        comparison=comparison,
    )
    acceptance_passed = not acceptance_failures
    summary = {
        "status": (
            "complete"
            if not failed and not missing and not artifact_missing
            and not artifact_incomplete and not artifact_invalid
            else "failed"
        ),
        "expected_runs": len(expected_names),
        "completed_runs": len(completed),
        "failed_runs": len(failed),
        "missing_runs": list(missing),
        "artifact_missing_runs": list(artifact_missing),
        "artifact_incomplete_runs": list(artifact_incomplete),
        "artifact_invalid_runs": list(artifact_invalid),
        "acceptance_passed": acceptance_passed,
        "acceptance_failures": list(acceptance_failures),
        "aggregate": aggregate,
        "landmark_vs_dead_reckoning": comparison,
        "records": records,
    }
    _write_metrics_csv(path / "metrics.csv", records)
    _atomic_json(path / "campaign_summary.json", summary)
    marker.unlink()
    return CampaignResult(
        path=path,
        expected_runs=len(expected_names),
        completed_runs=len(completed),
        failed_runs=len(failed),
        missing_runs=missing,
        aggregate=aggregate,
        acceptance_passed=acceptance_passed,
        acceptance_failures=acceptance_failures,
        artifact_missing_runs=artifact_missing,
        artifact_incomplete_runs=artifact_incomplete,
        artifact_invalid_runs=artifact_invalid,
    )


def inspect_campaign(path: Path) -> dict[str, object]:
    """Validate campaign completion and explicitly report missing run artifacts."""

    config = _read_json(path / "campaign_config.json")
    expected = config.get("expected_runs")
    if not isinstance(expected, list) or not all(isinstance(item, str) for item in expected):
        raise ValueError("campaign expected_runs is invalid")
    if any(
        not name or Path(name).name != name or name in {".", ".."}
        for name in expected
    ):
        raise ValueError("campaign expected_runs contains an unsafe run name")
    if len(set(expected)) != len(expected):
        raise ValueError("campaign expected_runs contains duplicates")
    marker_present = (path / "INCOMPLETE").exists()
    summary_path = path / "campaign_summary.json"
    summary_present = summary_path.is_file()
    if summary_present:
        summary = _read_json(summary_path)
    elif marker_present:
        summary = {
            "status": "incomplete",
            "expected_runs": len(expected),
            "completed_runs": 0,
            "failed_runs": None,
            "missing_runs": list(expected),
        }
    else:
        raise ValueError("campaign summary is missing without an INCOMPLETE marker")

    missing: list[str] = []
    incomplete: list[str] = []
    invalid: list[str] = []
    completed_count = 0
    for name in expected:
        run_path = path / name
        manifest_path = run_path / "manifest.json"
        if not manifest_path.is_file():
            missing.append(name)
            if (run_path / "INCOMPLETE").exists():
                incomplete.append(name)
            continue
        artifact_status = _run_artifact_status(run_path)
        if artifact_status == "incomplete":
            incomplete.append(name)
        elif artifact_status == "invalid":
            invalid.append(name)
        elif artifact_status == "complete":
            completed_count += 1

    if not summary_present:
        summary["completed_runs"] = completed_count
        summary["missing_runs"] = sorted(missing)
    acceptance_failures = list(summary.get("acceptance_failures", []))
    if not isinstance(acceptance_failures, list) or not all(
        isinstance(item, str) for item in acceptance_failures
    ):
        acceptance_failures = ["campaign_summary_acceptance_invalid"]
    for category, names in (
        ("artifact_missing", missing),
        ("artifact_incomplete", incomplete),
        ("artifact_invalid", invalid),
    ):
        acceptance_failures.extend(f"{category}:{name}" for name in names)
    acceptance_failures = sorted(set(acceptance_failures))
    acceptance_passed = bool(summary.get("acceptance_passed", False))
    if not summary_present:
        acceptance_failures.append("campaign_incomplete")
    acceptance_passed = acceptance_passed and not acceptance_failures
    return {
        **summary,
        "artifact_missing_runs": sorted(missing),
        "artifact_incomplete_runs": sorted(set(incomplete)),
        "artifact_invalid_runs": sorted(invalid),
        "campaign_config_present": True,
        "campaign_summary_present": summary_present,
        "campaign_incomplete": marker_present,
        "acceptance_passed": acceptance_passed,
        "acceptance_failures": sorted(set(acceptance_failures)),
    }


def _run_name(scenario_name: str, mode: str, seed: int) -> str:
    return f"{scenario_name}__{mode}__seed_{seed:08d}"


def _aggregate(
    completed: list[dict[str, object]],
    modes: tuple[str, ...],
) -> dict[str, dict[str, float | int | bool | None]]:
    grouped: defaultdict[str, list[dict[str, object]]] = defaultdict(list)
    for record in completed:
        grouped[str(record["mode"])].append(record)
    result: dict[str, dict[str, float | int | bool | None]] = {}
    metric_names = (
        "position_rmse_m",
        "heading_rmse_rad",
        "cross_track_rmse_m",
        "final_stop_error_m",
        "final_speed_mps",
        "nis_mean",
        "nis_max",
    )
    for mode in modes:
        values = grouped[mode]
        row: dict[str, float | int | bool | None] = {
            "runs": len(values),
            "successes": sum(bool(item["success"]) for item in values),
        }
        for metric in metric_names:
            numbers = [
                float(item[metric])
                for item in values
                if item.get(metric) is not None
            ]
            row[f"{metric}_mean"] = (
                math.fsum(numbers) / len(numbers) if numbers else None
            )
            row[f"{metric}_max"] = max(numbers) if numbers else None
        latency = _numbers(values, "native_process_round_trip_ms_p99")
        row["native_process_round_trip_ms_p99_mean"] = (
            math.fsum(latency) / len(latency) if latency else None
        )
        row["native_process_round_trip_ms_p99_max"] = (
            max(latency) if latency else None
        )
        for counter in _COUNTER_FIELDS:
            numbers = _numbers(values, counter)
            row[f"{counter}_total"] = sum(numbers) if numbers else None
            row[f"{counter}_max"] = max(numbers) if numbers else None
        for field in ("nis_evaluated_count", "nis_gate_rejected_count", "nis_sum"):
            numbers = _numbers(values, field)
            row[f"{field}_total"] = math.fsum(numbers) if numbers else None
        nis_maxima = _numbers(values, "nis_max")
        row["nis_max_max"] = max(nis_maxima) if nis_maxima else None
        result[mode] = row
    return result


def _landmark_dead_reckoning_comparison(
    completed: list[dict[str, object]],
    seeds: tuple[int, ...],
) -> dict[str, object]:
    lookup = {
        (str(record["mode"]), int(record["seed"])): record
        for record in completed
    }
    paired = []
    for seed in seeds:
        landmark = lookup.get(("landmark_aided", seed))
        dead = lookup.get(("dead_reckoning", seed))
        if landmark is None or dead is None:
            continue
        paired.append(
            float(dead["position_rmse_m"]) - float(landmark["position_rmse_m"])
        )
    return {
        "paired_seed_count": len(paired),
        "landmark_lower_rmse_seed_count": sum(value > 0.0 for value in paired),
        "mean_dead_minus_landmark_position_rmse_m": (
            math.fsum(paired) / len(paired) if paired else None
        ),
        "landmark_lower_mean_position_rmse": (
            math.fsum(paired) > 0.0 if paired else None
        ),
    }


def _write_metrics_csv(path: Path, records: list[dict[str, object]]) -> None:
    fields = (
        "run_name",
        "mode",
        "seed",
        "status",
        "success",
        "position_rmse_m",
        "heading_rmse_rad",
        "cross_track_rmse_m",
        "cross_track_max_m",
        "final_stop_error_m",
        "final_speed_mps",
        "nis_mean",
        "nis_max",
        "native_process_round_trip_ms_p99",
        *_COUNTER_FIELDS,
        *_NIS_FIELDS,
        "error",
    )
    with path.open("x", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for record in records:
            writer.writerow(record)


def _atomic_json(path: Path, value: object) -> None:
    text = json.dumps(value, sort_keys=True, indent=2, allow_nan=False) + "\n"
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", dir=path.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as output:
            output.write(text)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary_name, path)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def _read_json(path: Path) -> dict[str, object]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path.name} must contain an object")
    return value


def _summary_measurements(summary: object) -> dict[str, int | float | None]:
    if not isinstance(summary, dict):
        return {}
    flattened: dict[str, int | float | None] = {
        "native_process_round_trip_ms_p99": _number_or_none(
            summary.get("native_process_round_trip_ms_p99")
        )
    }
    for group_name, prefix in _SUMMARY_GROUPS.items():
        group = summary.get(group_name)
        if isinstance(group, dict):
            for name, value in group.items():
                flattened[f"{prefix}_{name}"] = _number_or_none(value)
    nis = summary.get("nis_statistics")
    if isinstance(nis, dict):
        flattened.update(
            nis_evaluated_count=_number_or_none(nis.get("evaluated_count")),
            nis_gate_rejected_count=_number_or_none(nis.get("gate_rejected_count")),
            nis_sum=_number_or_none(nis.get("sum")),
            nis_max=_number_or_none(nis.get("max")),
        )
    return flattened


def _number_or_none(value: object) -> int | float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    if not math.isfinite(value):
        return None
    return value


def _numbers(records: list[dict[str, object]], name: str) -> list[float]:
    result: list[float] = []
    for record in records:
        value = _number_or_none(record.get(name))
        if value is not None:
            result.append(float(value))
    return result


def _run_artifact_status(path: Path) -> str:
    if not path.is_dir() or not (path / "manifest.json").is_file():
        return "missing"
    try:
        RunReplay(path)
    except IncompleteRunError:
        return "incomplete"
    except (CorruptRunError, OSError, ValueError):
        return "invalid"
    return "complete"


def _acceptance_failures(
    *,
    expected_names: tuple[str, ...],
    records: list[dict[str, object]],
    missing_runs: tuple[str, ...],
    artifact_missing_runs: tuple[str, ...],
    artifact_incomplete_runs: tuple[str, ...],
    artifact_invalid_runs: tuple[str, ...],
    comparison: dict[str, object],
) -> tuple[str, ...]:
    failures: list[str] = []
    by_name = {str(record["run_name"]): record for record in records}
    failures.extend(f"missing_record:{name}" for name in missing_runs)
    failures.extend(
        f"execution_failed:{name}"
        for name, record in by_name.items()
        if record.get("status") == "failed"
    )
    for category, names in (
        ("artifact_missing", artifact_missing_runs),
        ("artifact_incomplete", artifact_incomplete_runs),
        ("artifact_invalid", artifact_invalid_runs),
    ):
        failures.extend(f"{category}:{name}" for name in names)
    failures.extend(
        f"metric_failed:{name}"
        for name in expected_names
        if name in by_name
        and by_name[name].get("status") == "complete"
        and by_name[name].get("success") is not True
    )
    if comparison.get("landmark_lower_mean_position_rmse") is not True:
        failures.append("landmark_mean_position_rmse_not_better_than_dead_reckoning")
    return tuple(sorted(set(failures)))
