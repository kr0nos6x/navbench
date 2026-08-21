"""Deterministic experiment metrics independent of ground-truth consumers.

Ground truth enters this module only after a run.  No estimator or controller
API imports this module, which keeps evaluation data out of embedded inputs.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Sequence
from dataclasses import asdict, dataclass


@dataclass(frozen=True, slots=True)
class Pose2:
    x_m: float
    y_m: float
    heading_rad: float


@dataclass(frozen=True, slots=True)
class MetricSummary:
    sample_count: int
    position_rmse_m: float
    heading_rmse_rad: float
    cross_track_rmse_m: float
    cross_track_max_m: float
    final_stop_error_m: float
    final_speed_mps: float
    nis_mean: float | None
    nis_max: float | None
    success: bool

    def to_dict(self) -> dict[str, int | float | bool | None]:
        return asdict(self)


def summarize_run(
    *,
    truth: Sequence[Pose2],
    estimates: Sequence[Pose2],
    route: Sequence[tuple[float, float]],
    final_speed_mps: float,
    nis_values: Iterable[float] = (),
    nis_evaluated_count: int | None = None,
    nis_sum: float | None = None,
    nis_maximum: float | None = None,
    max_position_rmse_m: float = math.inf,
    max_cross_track_rmse_m: float = math.inf,
    max_final_stop_error_m: float = math.inf,
    max_final_speed_mps: float = math.inf,
) -> MetricSummary:
    if not truth:
        raise ValueError("truth samples cannot be empty")
    if len(truth) != len(estimates):
        raise ValueError("truth and estimate sample counts must match")
    if len(route) < 2:
        raise ValueError("route must contain at least two points")

    finite_limits = (
        max_position_rmse_m,
        max_cross_track_rmse_m,
        max_final_stop_error_m,
        max_final_speed_mps,
    )
    if any(math.isnan(value) or value < 0.0 for value in finite_limits):
        raise ValueError("metric limits must be non-negative")
    if not math.isfinite(final_speed_mps) or final_speed_mps < 0.0:
        raise ValueError("final_speed_mps must be finite and non-negative")

    position_squared: list[float] = []
    heading_squared: list[float] = []
    cross_track_squared: list[float] = []
    cross_track_max = 0.0
    for truth_pose, estimate in zip(truth, estimates, strict=True):
        _validate_pose(truth_pose)
        _validate_pose(estimate)
        dx = estimate.x_m - truth_pose.x_m
        dy = estimate.y_m - truth_pose.y_m
        position_squared.append(dx * dx + dy * dy)
        heading_error = wrap_angle(estimate.heading_rad - truth_pose.heading_rad)
        heading_squared.append(heading_error * heading_error)
        cross_track = distance_to_polyline(
            truth_pose.x_m,
            truth_pose.y_m,
            route,
        )
        cross_track_squared.append(cross_track * cross_track)
        cross_track_max = max(cross_track_max, cross_track)

    position_rmse = _rms_squared(position_squared)
    heading_rmse = _rms_squared(heading_squared)
    cross_track_rmse = _rms_squared(cross_track_squared)
    final_x, final_y = route[-1]
    final_stop_error = math.hypot(
        truth[-1].x_m - final_x,
        truth[-1].y_m - final_y,
    )

    accepted_nis = []
    for value in nis_values:
        if not math.isfinite(value) or value < 0.0:
            raise ValueError("NIS values must be finite and non-negative")
        accepted_nis.append(value)
    aggregate_values = (nis_evaluated_count, nis_sum, nis_maximum)
    if any(value is not None for value in aggregate_values):
        if any(value is None for value in aggregate_values):
            raise ValueError("aggregate NIS count, sum, and maximum are required")
        if accepted_nis:
            raise ValueError("NIS samples and aggregate summary are mutually exclusive")
        assert nis_evaluated_count is not None
        assert nis_sum is not None
        assert nis_maximum is not None
        if (
            isinstance(nis_evaluated_count, bool)
            or not isinstance(nis_evaluated_count, int)
            or nis_evaluated_count < 0
            or not math.isfinite(nis_sum)
            or nis_sum < 0.0
            or not math.isfinite(nis_maximum)
            or nis_maximum < 0.0
            or (nis_evaluated_count == 0 and (nis_sum != 0.0 or nis_maximum != 0.0))
            or (nis_evaluated_count > 0 and nis_maximum > nis_sum)
        ):
            raise ValueError("invalid aggregate NIS summary")
        nis_mean = (
            nis_sum / nis_evaluated_count
            if nis_evaluated_count > 0
            else None
        )
        nis_max = nis_maximum if nis_evaluated_count > 0 else None
    else:
        nis_mean = (
            math.fsum(accepted_nis) / len(accepted_nis)
            if accepted_nis
            else None
        )
        nis_max = max(accepted_nis) if accepted_nis else None

    success = (
        position_rmse <= max_position_rmse_m
        and cross_track_rmse <= max_cross_track_rmse_m
        and final_stop_error <= max_final_stop_error_m
        and final_speed_mps <= max_final_speed_mps
    )
    return MetricSummary(
        sample_count=len(truth),
        position_rmse_m=position_rmse,
        heading_rmse_rad=heading_rmse,
        cross_track_rmse_m=cross_track_rmse,
        cross_track_max_m=cross_track_max,
        final_stop_error_m=final_stop_error,
        final_speed_mps=final_speed_mps,
        nis_mean=nis_mean,
        nis_max=nis_max,
        success=success,
    )


def distance_to_polyline(
    x_m: float,
    y_m: float,
    route: Sequence[tuple[float, float]],
) -> float:
    if not math.isfinite(x_m) or not math.isfinite(y_m):
        raise ValueError("point must be finite")
    if len(route) < 2:
        raise ValueError("route must contain at least two points")

    minimum = math.inf
    previous_x, previous_y = _validate_point(route[0])
    for raw_point in route[1:]:
        next_x, next_y = _validate_point(raw_point)
        dx = next_x - previous_x
        dy = next_y - previous_y
        length_squared = dx * dx + dy * dy
        if length_squared == 0.0:
            distance = math.hypot(x_m - previous_x, y_m - previous_y)
        else:
            projection = (
                (x_m - previous_x) * dx + (y_m - previous_y) * dy
            ) / length_squared
            projection = max(0.0, min(1.0, projection))
            closest_x = previous_x + projection * dx
            closest_y = previous_y + projection * dy
            distance = math.hypot(x_m - closest_x, y_m - closest_y)
        minimum = min(minimum, distance)
        previous_x, previous_y = next_x, next_y
    return minimum


def wrap_angle(angle_rad: float) -> float:
    if not math.isfinite(angle_rad):
        raise ValueError("angle must be finite")
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def _rms_squared(squared_values: Sequence[float]) -> float:
    return math.sqrt(math.fsum(squared_values) / len(squared_values))


def _validate_pose(pose: Pose2) -> None:
    if not all(
        math.isfinite(value)
        for value in (pose.x_m, pose.y_m, pose.heading_rad)
    ):
        raise ValueError("pose values must be finite")


def _validate_point(point: tuple[float, float]) -> tuple[float, float]:
    if len(point) != 2:
        raise ValueError("route point must contain x and y")
    x_m, y_m = point
    if not math.isfinite(x_m) or not math.isfinite(y_m):
        raise ValueError("route points must be finite")
    return float(x_m), float(y_m)
