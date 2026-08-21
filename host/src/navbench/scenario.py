"""Validated YAML scenarios and the canonical open-loop execution adapter."""

from __future__ import annotations

import math
import re
from dataclasses import asdict, dataclass
from decimal import Decimal, InvalidOperation
from pathlib import Path
from typing import Any

import yaml

from navbench.sensors import (
    FaultSpec,
    GnssConfig,
    ImuConfig,
    Landmark,
    LandmarkConfig,
    SensorSuiteConfig,
    WheelSpeedConfig,
)
from navbench.simulator import (
    ControlCommand,
    SimulationSample,
    VehicleModel,
    VehicleParameters,
    VehicleState,
    simulate_fixed_step,
)


_SCENARIO_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")
_MISSING = object()


def _finite(value: object, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{path} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{path} must be a finite number")
    return result


def _number(
    mapping: dict[str, Any],
    key: str,
    path: str,
    default: object = _MISSING,
) -> float:
    if key not in mapping:
        if default is _MISSING:
            raise ValueError(f"{path}.{key} is required")
        return _finite(default, f"{path}.{key}")
    return _finite(mapping[key], f"{path}.{key}")


def _integer(
    mapping: dict[str, Any],
    key: str,
    path: str,
    default: object = _MISSING,
) -> int:
    if key not in mapping:
        if default is _MISSING:
            raise ValueError(f"{path}.{key} is required")
        value = default
    else:
        value = mapping[key]
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{path}.{key} must be an integer")
    return value


def _boolean(
    mapping: dict[str, Any], key: str, path: str, default: bool
) -> bool:
    value = mapping.get(key, default)
    if not isinstance(value, bool):
        raise ValueError(f"{path}.{key} must be a boolean")
    return value


def _mapping(
    value: object,
    path: str,
    *,
    allowed: set[str] | None = None,
) -> dict[str, Any]:
    if not isinstance(value, dict) or not all(isinstance(k, str) for k in value):
        raise ValueError(f"{path} must be a mapping with string keys")
    result: dict[str, Any] = value
    if allowed is not None:
        unknown = sorted(set(result) - allowed)
        if unknown:
            raise ValueError(f"{path} contains unknown field(s): {', '.join(unknown)}")
    return result


def _list(value: object, path: str) -> list[Any]:
    if not isinstance(value, list):
        raise ValueError(f"{path} must be a list")
    return value


def _exact_steps(duration_s: float, dt_s: float, path: str = "duration_s") -> int:
    try:
        duration = Decimal(str(duration_s))
        timestep = Decimal(str(dt_s))
    except InvalidOperation as error:
        raise ValueError(f"{path} and dt_s must be decimal-compatible") from error
    if duration <= 0 or timestep <= 0:
        raise ValueError(f"{path} and dt_s must be greater than zero")
    quotient, remainder = divmod(duration, timestep)
    if remainder != 0:
        raise ValueError(f"{path} must be an exact integer multiple of dt_s")
    steps = int(quotient)
    if steps < 1 or steps > 10_000_000:
        raise ValueError("scenario step count must be in [1, 10000000]")
    return steps


def _require_aligned_time(time_s: float, dt_s: float, path: str) -> None:
    """Require a non-negative time to lie exactly on the decimal step grid."""

    try:
        time = Decimal(str(time_s))
        timestep = Decimal(str(dt_s))
    except InvalidOperation as error:
        raise ValueError(f"{path} and dt_s must be decimal-compatible") from error
    if time < 0 or timestep <= 0 or time % timestep != 0:
        raise ValueError(f"{path} must be a non-negative integer multiple of dt_s")


@dataclass(frozen=True, slots=True)
class CommandSegment:
    until_s: float
    acceleration_mps2: float
    steering_rad: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "until_s", _finite(self.until_s, "command until_s"))
        object.__setattr__(
            self,
            "acceleration_mps2",
            _finite(self.acceleration_mps2, "command acceleration_mps2"),
        )
        object.__setattr__(
            self, "steering_rad", _finite(self.steering_rad, "command steering_rad")
        )
        if abs(self.acceleration_mps2) > 100.0:
            raise ValueError("command acceleration magnitude cannot exceed 100 m/s^2")
        if abs(self.steering_rad) > 2.0 * math.pi:
            raise ValueError("command steering magnitude cannot exceed 2*pi")


@dataclass(frozen=True, slots=True)
class Waypoint:
    x_m: float
    y_m: float
    target_speed_mps: float
    acceptance_radius_m: float = 0.35

    def __post_init__(self) -> None:
        for name in ("x_m", "y_m", "target_speed_mps", "acceptance_radius_m"):
            object.__setattr__(self, name, _finite(getattr(self, name), f"waypoint {name}"))
        if self.target_speed_mps < 0.0:
            raise ValueError("waypoint target_speed_mps cannot be negative")
        if self.acceptance_radius_m <= 0.0:
            raise ValueError("waypoint acceptance_radius_m must be greater than zero")


@dataclass(frozen=True, slots=True)
class Scenario:
    name: str
    duration_s: float
    dt_s: float
    initial_state: VehicleState
    commands: tuple[CommandSegment, ...]
    vehicle: VehicleParameters = VehicleParameters()
    sensors: SensorSuiteConfig = SensorSuiteConfig()
    landmarks: tuple[Landmark, ...] = ()
    faults: tuple[FaultSpec, ...] = ()
    route: tuple[Waypoint, ...] = ()
    seed: int = 0

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not _SCENARIO_NAME.fullmatch(self.name):
            raise ValueError(
                "scenario name must be 1-64 safe filename characters (letters, digits, '.', '_' or '-')"
            )
        object.__setattr__(self, "duration_s", _finite(self.duration_s, "duration_s"))
        object.__setattr__(self, "dt_s", _finite(self.dt_s, "dt_s"))
        if isinstance(self.seed, bool) or not isinstance(self.seed, int):
            raise ValueError("seed must be an integer")
        if self.seed < 0 or self.seed > 2**64 - 1:
            raise ValueError("seed must be in [0, 2^64 - 1]")
        if not isinstance(self.commands, tuple) or not self.commands:
            raise ValueError("commands must be a non-empty tuple")
        if not all(isinstance(item, CommandSegment) for item in self.commands):
            raise TypeError("commands must contain CommandSegment values")
        if not all(isinstance(item, Landmark) for item in self.landmarks):
            raise TypeError("landmarks must contain Landmark values")
        if not all(isinstance(item, FaultSpec) for item in self.faults):
            raise TypeError("faults must contain FaultSpec values")
        if not all(isinstance(item, Waypoint) for item in self.route):
            raise TypeError("route must contain Waypoint values")
        _validate_scenario(self)

    @property
    def step_count(self) -> int:
        return _exact_steps(self.duration_s, self.dt_s)

    @property
    def vehicle_parameters(self) -> VehicleParameters:
        """Compatibility alias for callers using the longer field name."""

        return self.vehicle

    def command_at(self, time_s: float) -> ControlCommand:
        time_s = _finite(time_s, "time_s")
        if time_s < 0.0 or time_s > self.duration_s + 1e-12:
            raise ValueError("time_s is outside the scenario")
        for segment in self.commands:
            if time_s < segment.until_s:
                return ControlCommand(segment.acceleration_mps2, segment.steering_rad)
        final_segment = self.commands[-1]
        return ControlCommand(final_segment.acceleration_mps2, final_segment.steering_rad)


def load_scenario(path: Path) -> Scenario:
    with path.open(encoding="utf-8") as scenario_file:
        raw = yaml.safe_load(scenario_file)
    return scenario_from_mapping(raw)


def scenario_from_mapping(raw: object) -> Scenario:
    """Parse and validate a scenario mapping, including run config snapshots."""

    root = _mapping(
        raw,
        "scenario",
        allowed={
            "version",
            "name",
            "seed",
            "duration_s",
            "dt_s",
            "vehicle",
            "initial_state",
            "commands",
            "sensors",
            "landmarks",
            "faults",
            "route",
        },
    )
    version = _integer(root, "version", "scenario", 1)
    if version != 1:
        raise ValueError(f"unsupported scenario version: {version}")
    name = root.get("name")
    if not isinstance(name, str):
        raise ValueError("scenario.name must be a string")

    vehicle = _parse_vehicle(root.get("vehicle", {}))
    initial_state = _parse_initial_state(root.get("initial_state", {}))
    commands = _parse_commands(root.get("commands", []))
    sensors = _parse_sensors(root.get("sensors", {}))
    landmarks = _parse_landmarks(root.get("landmarks", []))
    faults = _parse_faults(root.get("faults", []))
    route = _parse_route(root.get("route", []))

    scenario = Scenario(
        name=name,
        duration_s=_number(root, "duration_s", "scenario"),
        dt_s=_number(root, "dt_s", "scenario"),
        initial_state=initial_state,
        commands=commands,
        vehicle=vehicle,
        sensors=sensors,
        landmarks=landmarks,
        faults=faults,
        route=route,
        seed=_integer(root, "seed", "scenario", 0),
    )
    return scenario


def run_scenario(scenario: Scenario) -> list[SimulationSample]:
    """Execute a validated scenario through the canonical plant loop."""

    return simulate_fixed_step(
        model=VehicleModel(scenario.vehicle),
        initial_state=scenario.initial_state,
        dt_s=scenario.dt_s,
        step_count=scenario.step_count,
        command_at=lambda _step_id, time_s: scenario.command_at(time_s),
    )


def scenario_to_mapping(scenario: Scenario) -> dict[str, Any]:
    """Return a stable, reloadable normalized scenario mapping."""

    return {
        "version": 1,
        "name": scenario.name,
        "seed": scenario.seed,
        "duration_s": scenario.duration_s,
        "dt_s": scenario.dt_s,
        "vehicle": {
            "wheelbase_m": scenario.vehicle.wheelbase_m,
            "steering_time_constant_s": scenario.vehicle.steering_time_constant_s,
            "acceleration_time_constant_s": scenario.vehicle.acceleration_time_constant_s,
            "max_steering_deg": math.degrees(scenario.vehicle.max_steering_rad),
            "max_acceleration_mps2": scenario.vehicle.max_acceleration_mps2,
            "max_deceleration_mps2": scenario.vehicle.max_deceleration_mps2,
            "max_speed_mps": scenario.vehicle.max_speed_mps,
        },
        "initial_state": {
            "x_m": scenario.initial_state.x_m,
            "y_m": scenario.initial_state.y_m,
            "heading_deg": math.degrees(scenario.initial_state.heading_rad),
            "speed_mps": scenario.initial_state.speed_mps,
            "steering_deg": math.degrees(scenario.initial_state.steering_rad),
            "acceleration_mps2": scenario.initial_state.acceleration_mps2,
        },
        "commands": [
            {
                "until_s": command.until_s,
                "acceleration_mps2": command.acceleration_mps2,
                "steering_deg": math.degrees(command.steering_rad),
            }
            for command in scenario.commands
        ],
        "sensors": _sensors_to_mapping(scenario.sensors),
        "landmarks": [asdict(landmark) for landmark in scenario.landmarks],
        "faults": [asdict(fault) for fault in scenario.faults],
        "route": [asdict(waypoint) for waypoint in scenario.route],
    }


def _parse_vehicle(raw: object) -> VehicleParameters:
    values = _mapping(
        raw,
        "vehicle",
        allowed={
            "wheelbase_m",
            "steering_time_constant_s",
            "acceleration_time_constant_s",
            "max_steering_deg",
            "max_acceleration_mps2",
            "max_deceleration_mps2",
            "max_speed_mps",
        },
    )
    defaults = VehicleParameters()
    return VehicleParameters(
        wheelbase_m=_number(values, "wheelbase_m", "vehicle", defaults.wheelbase_m),
        steering_time_constant_s=_number(
            values,
            "steering_time_constant_s",
            "vehicle",
            defaults.steering_time_constant_s,
        ),
        acceleration_time_constant_s=_number(
            values,
            "acceleration_time_constant_s",
            "vehicle",
            defaults.acceleration_time_constant_s,
        ),
        max_steering_rad=math.radians(
            _number(values, "max_steering_deg", "vehicle", math.degrees(defaults.max_steering_rad))
        ),
        max_acceleration_mps2=_number(
            values, "max_acceleration_mps2", "vehicle", defaults.max_acceleration_mps2
        ),
        max_deceleration_mps2=_number(
            values, "max_deceleration_mps2", "vehicle", defaults.max_deceleration_mps2
        ),
        max_speed_mps=_number(values, "max_speed_mps", "vehicle", defaults.max_speed_mps),
    )


def _parse_initial_state(raw: object) -> VehicleState:
    values = _mapping(
        raw,
        "initial_state",
        allowed={
            "x_m",
            "y_m",
            "heading_deg",
            "speed_mps",
            "steering_deg",
            "acceleration_mps2",
        },
    )
    return VehicleState(
        x_m=_number(values, "x_m", "initial_state", 0.0),
        y_m=_number(values, "y_m", "initial_state", 0.0),
        heading_rad=math.radians(_number(values, "heading_deg", "initial_state", 0.0)),
        speed_mps=_number(values, "speed_mps", "initial_state", 0.0),
        steering_rad=math.radians(_number(values, "steering_deg", "initial_state", 0.0)),
        acceleration_mps2=_number(values, "acceleration_mps2", "initial_state", 0.0),
    )


def _parse_commands(raw: object) -> tuple[CommandSegment, ...]:
    items = _list(raw, "commands")
    if not items:
        raise ValueError("commands must be a non-empty list")
    commands: list[CommandSegment] = []
    for index, item in enumerate(items):
        path = f"commands[{index}]"
        values = _mapping(
            item,
            path,
            allowed={"until_s", "acceleration_mps2", "steering_deg"},
        )
        commands.append(
            CommandSegment(
                until_s=_number(values, "until_s", path),
                acceleration_mps2=_number(values, "acceleration_mps2", path),
                steering_rad=math.radians(_number(values, "steering_deg", path)),
            )
        )
    return tuple(commands)


def _parse_sensors(raw: object) -> SensorSuiteConfig:
    values = _mapping(raw, "sensors", allowed={"imu", "wheel_speed", "gnss", "landmark"})
    return SensorSuiteConfig(
        imu=_parse_imu(values.get("imu", {})),
        wheel_speed=_parse_wheel(values.get("wheel_speed", {})),
        gnss=_parse_gnss(values.get("gnss", {})),
        landmark=_parse_landmark_sensor(values.get("landmark", {})),
    )


def _parse_imu(raw: object) -> ImuConfig:
    path = "sensors.imu"
    values = _mapping(
        raw,
        path,
        allowed={
            "enabled", "rate_hz", "acceleration_noise_std_mps2",
            "yaw_rate_noise_std_deg_s", "acceleration_bias_mps2",
            "yaw_rate_bias_deg_s", "latency_s", "dropout_probability",
        },
    )
    defaults = ImuConfig()
    return ImuConfig(
        enabled=_boolean(values, "enabled", path, defaults.enabled),
        rate_hz=_number(values, "rate_hz", path, defaults.rate_hz),
        acceleration_noise_std_mps2=_number(
            values, "acceleration_noise_std_mps2", path, defaults.acceleration_noise_std_mps2
        ),
        yaw_rate_noise_std_rad_s=math.radians(
            _number(
                values,
                "yaw_rate_noise_std_deg_s",
                path,
                math.degrees(defaults.yaw_rate_noise_std_rad_s),
            )
        ),
        acceleration_bias_mps2=_number(
            values, "acceleration_bias_mps2", path, defaults.acceleration_bias_mps2
        ),
        yaw_rate_bias_rad_s=math.radians(
            _number(
                values,
                "yaw_rate_bias_deg_s",
                path,
                math.degrees(defaults.yaw_rate_bias_rad_s),
            )
        ),
        latency_s=_number(values, "latency_s", path, defaults.latency_s),
        dropout_probability=_number(
            values, "dropout_probability", path, defaults.dropout_probability
        ),
    )


def _parse_wheel(raw: object) -> WheelSpeedConfig:
    path = "sensors.wheel_speed"
    values = _mapping(
        raw,
        path,
        allowed={"enabled", "rate_hz", "noise_std_mps", "scale_error", "latency_s", "dropout_probability"},
    )
    defaults = WheelSpeedConfig()
    return WheelSpeedConfig(
        enabled=_boolean(values, "enabled", path, defaults.enabled),
        rate_hz=_number(values, "rate_hz", path, defaults.rate_hz),
        noise_std_mps=_number(values, "noise_std_mps", path, defaults.noise_std_mps),
        scale_error=_number(values, "scale_error", path, defaults.scale_error),
        latency_s=_number(values, "latency_s", path, defaults.latency_s),
        dropout_probability=_number(
            values, "dropout_probability", path, defaults.dropout_probability
        ),
    )


def _parse_gnss(raw: object) -> GnssConfig:
    path = "sensors.gnss"
    values = _mapping(
        raw,
        path,
        allowed={"enabled", "rate_hz", "position_noise_std_m", "latency_s", "dropout_probability"},
    )
    defaults = GnssConfig()
    return GnssConfig(
        enabled=_boolean(values, "enabled", path, defaults.enabled),
        rate_hz=_number(values, "rate_hz", path, defaults.rate_hz),
        position_noise_std_m=_number(
            values, "position_noise_std_m", path, defaults.position_noise_std_m
        ),
        latency_s=_number(values, "latency_s", path, defaults.latency_s),
        dropout_probability=_number(
            values, "dropout_probability", path, defaults.dropout_probability
        ),
    )


def _parse_landmark_sensor(raw: object) -> LandmarkConfig:
    path = "sensors.landmark"
    values = _mapping(
        raw,
        path,
        allowed={
            "enabled", "rate_hz", "range_noise_std_m", "bearing_noise_std_deg",
            "max_range_m", "field_of_view_deg", "latency_s", "dropout_probability",
            "outlier_probability", "outlier_range_std_m", "outlier_bearing_std_deg",
        },
    )
    defaults = LandmarkConfig()
    return LandmarkConfig(
        enabled=_boolean(values, "enabled", path, defaults.enabled),
        rate_hz=_number(values, "rate_hz", path, defaults.rate_hz),
        range_noise_std_m=_number(
            values, "range_noise_std_m", path, defaults.range_noise_std_m
        ),
        bearing_noise_std_rad=math.radians(
            _number(
                values,
                "bearing_noise_std_deg",
                path,
                math.degrees(defaults.bearing_noise_std_rad),
            )
        ),
        max_range_m=_number(values, "max_range_m", path, defaults.max_range_m),
        field_of_view_rad=math.radians(
            _number(
                values,
                "field_of_view_deg",
                path,
                math.degrees(defaults.field_of_view_rad),
            )
        ),
        latency_s=_number(values, "latency_s", path, defaults.latency_s),
        dropout_probability=_number(
            values, "dropout_probability", path, defaults.dropout_probability
        ),
        outlier_probability=_number(
            values, "outlier_probability", path, defaults.outlier_probability
        ),
        outlier_range_std_m=_number(
            values, "outlier_range_std_m", path, defaults.outlier_range_std_m
        ),
        outlier_bearing_std_rad=math.radians(
            _number(
                values,
                "outlier_bearing_std_deg",
                path,
                math.degrees(defaults.outlier_bearing_std_rad),
            )
        ),
    )


def _parse_landmarks(raw: object) -> tuple[Landmark, ...]:
    result: list[Landmark] = []
    for index, item in enumerate(_list(raw, "landmarks")):
        path = f"landmarks[{index}]"
        values = _mapping(item, path, allowed={"landmark_id", "x_m", "y_m"})
        result.append(
            Landmark(
                landmark_id=_integer(values, "landmark_id", path),
                x_m=_number(values, "x_m", path),
                y_m=_number(values, "y_m", path),
            )
        )
    return tuple(result)


def _parse_faults(raw: object) -> tuple[FaultSpec, ...]:
    result: list[FaultSpec] = []
    for index, item in enumerate(_list(raw, "faults")):
        path = f"faults[{index}]"
        values = _mapping(item, path, allowed={"target", "kind", "start_s", "end_s", "value"})
        target = values.get("target")
        kind = values.get("kind")
        if not isinstance(target, str) or not isinstance(kind, str):
            raise ValueError(f"{path}.target and {path}.kind must be strings")
        result.append(
            FaultSpec(
                target=target,
                kind=kind,
                start_s=_number(values, "start_s", path),
                end_s=_number(values, "end_s", path),
                value=_number(values, "value", path, 0.0),
            )
        )
    return tuple(result)


def _parse_route(raw: object) -> tuple[Waypoint, ...]:
    result: list[Waypoint] = []
    for index, item in enumerate(_list(raw, "route")):
        path = f"route[{index}]"
        values = _mapping(
            item,
            path,
            allowed={"x_m", "y_m", "target_speed_mps", "acceptance_radius_m"},
        )
        result.append(
            Waypoint(
                x_m=_number(values, "x_m", path),
                y_m=_number(values, "y_m", path),
                target_speed_mps=_number(values, "target_speed_mps", path),
                acceptance_radius_m=_number(values, "acceptance_radius_m", path, 0.35),
            )
        )
    return tuple(result)


def _validate_scenario(scenario: Scenario) -> None:
    _exact_steps(scenario.duration_s, scenario.dt_s)
    params = scenario.vehicle
    state = scenario.initial_state
    if state.speed_mps < 0.0 or state.speed_mps > params.max_speed_mps:
        raise ValueError("initial speed is outside vehicle limits")
    if abs(state.steering_rad) > params.max_steering_rad:
        raise ValueError("initial steering is outside vehicle limits")
    if not -params.max_deceleration_mps2 <= state.acceleration_mps2 <= params.max_acceleration_mps2:
        raise ValueError("initial acceleration is outside vehicle limits")

    previous_end = 0.0
    for segment in scenario.commands:
        if segment.until_s <= previous_end:
            raise ValueError("command end times must increase")
        _exact_steps(segment.until_s, scenario.dt_s, "command until_s")
        previous_end = segment.until_s
    if previous_end != scenario.duration_s:
        raise ValueError("the final command must end exactly at duration_s")

    for name, config in (
        ("imu", scenario.sensors.imu),
        ("wheel_speed", scenario.sensors.wheel_speed),
        ("gnss", scenario.sensors.gnss),
        ("landmark", scenario.sensors.landmark),
    ):
        if not config.enabled:
            continue
        ratio = 1.0 / (config.rate_hz * scenario.dt_s)
        if round(ratio) < 1 or not math.isclose(ratio, round(ratio), rel_tol=0.0, abs_tol=1e-10):
            raise ValueError(f"{name} rate must be an integer divisor of the scenario timestep")
    if len({landmark.landmark_id for landmark in scenario.landmarks}) != len(scenario.landmarks):
        raise ValueError("landmark ids must be unique")
    for fault in scenario.faults:
        if fault.end_s > scenario.duration_s:
            raise ValueError("fault windows must be inside scenario duration")
        _require_aligned_time(fault.start_s, scenario.dt_s, "fault start_s")
        _require_aligned_time(fault.end_s, scenario.dt_s, "fault end_s")
    for waypoint in scenario.route:
        if waypoint.target_speed_mps > params.max_speed_mps:
            raise ValueError("waypoint target speed exceeds vehicle max_speed_mps")


def _sensors_to_mapping(sensors: SensorSuiteConfig) -> dict[str, Any]:
    return {
        "imu": {
            "enabled": sensors.imu.enabled,
            "rate_hz": sensors.imu.rate_hz,
            "acceleration_noise_std_mps2": sensors.imu.acceleration_noise_std_mps2,
            "yaw_rate_noise_std_deg_s": math.degrees(sensors.imu.yaw_rate_noise_std_rad_s),
            "acceleration_bias_mps2": sensors.imu.acceleration_bias_mps2,
            "yaw_rate_bias_deg_s": math.degrees(sensors.imu.yaw_rate_bias_rad_s),
            "latency_s": sensors.imu.latency_s,
            "dropout_probability": sensors.imu.dropout_probability,
        },
        "wheel_speed": {
            "enabled": sensors.wheel_speed.enabled,
            "rate_hz": sensors.wheel_speed.rate_hz,
            "noise_std_mps": sensors.wheel_speed.noise_std_mps,
            "scale_error": sensors.wheel_speed.scale_error,
            "latency_s": sensors.wheel_speed.latency_s,
            "dropout_probability": sensors.wheel_speed.dropout_probability,
        },
        "gnss": {
            "enabled": sensors.gnss.enabled,
            "rate_hz": sensors.gnss.rate_hz,
            "position_noise_std_m": sensors.gnss.position_noise_std_m,
            "latency_s": sensors.gnss.latency_s,
            "dropout_probability": sensors.gnss.dropout_probability,
        },
        "landmark": {
            "enabled": sensors.landmark.enabled,
            "rate_hz": sensors.landmark.rate_hz,
            "range_noise_std_m": sensors.landmark.range_noise_std_m,
            "bearing_noise_std_deg": math.degrees(sensors.landmark.bearing_noise_std_rad),
            "max_range_m": sensors.landmark.max_range_m,
            "field_of_view_deg": math.degrees(sensors.landmark.field_of_view_rad),
            "latency_s": sensors.landmark.latency_s,
            "dropout_probability": sensors.landmark.dropout_probability,
            "outlier_probability": sensors.landmark.outlier_probability,
            "outlier_range_std_m": sensors.landmark.outlier_range_std_m,
            "outlier_bearing_std_deg": math.degrees(sensors.landmark.outlier_bearing_std_rad),
        },
    }
