"""Deterministic virtual sensors with explicit sample and delivery times."""

from __future__ import annotations

import hashlib
import heapq
import math
import random
from dataclasses import dataclass, fields
from typing import TypeAlias

from navbench.simulator import VehicleParameters, VehicleState, wrap_angle


def _finite(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a finite real number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite real number")
    return result


def _probability(value: object, name: str) -> float:
    result = _finite(value, name)
    if result < 0.0 or result > 1.0:
        raise ValueError(f"{name} must be in [0, 1]")
    return result


def _validate_common(config: object) -> None:
    for item in fields(config):
        value = getattr(config, item.name)
        if item.name == "enabled":
            if not isinstance(value, bool):
                raise TypeError("enabled must be a boolean")
        elif item.name == "dropout_probability":
            object.__setattr__(config, item.name, _probability(value, item.name))
        elif isinstance(value, (int, float)) and not isinstance(value, bool):
            object.__setattr__(config, item.name, _finite(value, item.name))

    if getattr(config, "rate_hz") <= 0.0:
        raise ValueError("rate_hz must be greater than zero")
    if getattr(config, "latency_s") < 0.0:
        raise ValueError("latency_s cannot be negative")


@dataclass(frozen=True, slots=True)
class ImuConfig:
    enabled: bool = True
    rate_hz: float = 50.0
    acceleration_noise_std_mps2: float = 0.02
    yaw_rate_noise_std_rad_s: float = math.radians(0.15)
    acceleration_bias_mps2: float = 0.0
    yaw_rate_bias_rad_s: float = 0.0
    latency_s: float = 0.0
    dropout_probability: float = 0.0

    def __post_init__(self) -> None:
        _validate_common(self)
        if self.acceleration_noise_std_mps2 < 0.0:
            raise ValueError("acceleration noise cannot be negative")
        if self.yaw_rate_noise_std_rad_s < 0.0:
            raise ValueError("yaw-rate noise cannot be negative")


@dataclass(frozen=True, slots=True)
class WheelSpeedConfig:
    enabled: bool = True
    rate_hz: float = 25.0
    noise_std_mps: float = 0.01
    scale_error: float = 0.0
    latency_s: float = 0.0
    dropout_probability: float = 0.0

    def __post_init__(self) -> None:
        _validate_common(self)
        if self.noise_std_mps < 0.0:
            raise ValueError("wheel-speed noise cannot be negative")
        if self.scale_error <= -1.0:
            raise ValueError("scale_error must be greater than -1")


@dataclass(frozen=True, slots=True)
class GnssConfig:
    enabled: bool = True
    rate_hz: float = 5.0
    position_noise_std_m: float = 0.25
    latency_s: float = 0.10
    dropout_probability: float = 0.0

    def __post_init__(self) -> None:
        _validate_common(self)
        if self.position_noise_std_m < 0.0:
            raise ValueError("GNSS position noise cannot be negative")


@dataclass(frozen=True, slots=True)
class LandmarkConfig:
    enabled: bool = True
    rate_hz: float = 10.0
    range_noise_std_m: float = 0.03
    bearing_noise_std_rad: float = math.radians(0.5)
    max_range_m: float = 15.0
    field_of_view_rad: float = math.radians(160.0)
    latency_s: float = 0.04
    dropout_probability: float = 0.0
    outlier_probability: float = 0.0
    outlier_range_std_m: float = 3.0
    outlier_bearing_std_rad: float = math.radians(25.0)

    def __post_init__(self) -> None:
        _validate_common(self)
        object.__setattr__(
            self,
            "outlier_probability",
            _probability(self.outlier_probability, "outlier_probability"),
        )
        nonnegative = (
            "range_noise_std_m",
            "bearing_noise_std_rad",
            "outlier_range_std_m",
            "outlier_bearing_std_rad",
        )
        for name in nonnegative:
            if getattr(self, name) < 0.0:
                raise ValueError(f"{name} cannot be negative")
        if self.max_range_m <= 0.0:
            raise ValueError("max_range_m must be greater than zero")
        if self.field_of_view_rad <= 0.0 or self.field_of_view_rad > 2.0 * math.pi:
            raise ValueError("field_of_view_rad must be in (0, 2*pi]")


@dataclass(frozen=True, slots=True)
class SensorSuiteConfig:
    imu: ImuConfig = ImuConfig()
    wheel_speed: WheelSpeedConfig = WheelSpeedConfig()
    gnss: GnssConfig = GnssConfig()
    landmark: LandmarkConfig = LandmarkConfig()

    def __post_init__(self) -> None:
        expected = (
            ("imu", ImuConfig),
            ("wheel_speed", WheelSpeedConfig),
            ("gnss", GnssConfig),
            ("landmark", LandmarkConfig),
        )
        for name, expected_type in expected:
            if not isinstance(getattr(self, name), expected_type):
                raise TypeError(f"{name} must be {expected_type.__name__}")


@dataclass(frozen=True, slots=True)
class Landmark:
    landmark_id: int
    x_m: float
    y_m: float

    def __post_init__(self) -> None:
        if isinstance(self.landmark_id, bool) or not isinstance(self.landmark_id, int):
            raise TypeError("landmark_id must be an integer")
        if self.landmark_id < 0 or self.landmark_id > 65535:
            raise ValueError("landmark_id must be in [0, 65535]")
        object.__setattr__(self, "x_m", _finite(self.x_m, "landmark x_m"))
        object.__setattr__(self, "y_m", _finite(self.y_m, "landmark y_m"))


_FAULT_KINDS = {
    "dropout",
    "bias_acceleration_mps2",
    "bias_yaw_rate_rad_s",
    "scale_error",
    "outlier",
    "latency_spike",
    "packet_corruption",
    "packet_loss",
    "stale_frame",
    "disconnect",
    "manual_safe_stop",
}
_FAULT_TARGETS = {
    "imu",
    "wheel_speed",
    "gnss",
    "landmark",
    "transport",
    "host",
    "runtime",
}


@dataclass(frozen=True, slots=True)
class FaultSpec:
    target: str
    kind: str
    start_s: float
    end_s: float
    value: float = 0.0

    def __post_init__(self) -> None:
        if self.target not in _FAULT_TARGETS:
            raise ValueError(f"unsupported fault target: {self.target!r}")
        if self.kind not in _FAULT_KINDS:
            raise ValueError(f"unsupported fault kind: {self.kind!r}")
        object.__setattr__(self, "start_s", _finite(self.start_s, "fault start_s"))
        object.__setattr__(self, "end_s", _finite(self.end_s, "fault end_s"))
        object.__setattr__(self, "value", _finite(self.value, "fault value"))
        if self.start_s < 0.0:
            raise ValueError("fault start_s cannot be negative")
        if self.end_s <= self.start_s:
            raise ValueError("fault end_s must be greater than start_s")
        if self.kind == "latency_spike" and self.value < 0.0:
            raise ValueError("latency_spike value cannot be negative")
        valid = {
            "imu": {"dropout", "bias_acceleration_mps2", "bias_yaw_rate_rad_s", "latency_spike"},
            "wheel_speed": {"dropout", "scale_error", "latency_spike"},
            "gnss": {"dropout", "latency_spike"},
            "landmark": {"dropout", "outlier", "latency_spike"},
            "transport": {
                "packet_corruption",
                "packet_loss",
                "latency_spike",
                "stale_frame",
            },
            "host": {"disconnect"},
            "runtime": {"manual_safe_stop"},
        }
        if self.kind not in valid[self.target]:
            raise ValueError(f"fault {self.kind!r} is invalid for {self.target!r}")

    def active(self, time_s: float) -> bool:
        return self.start_s <= time_s < self.end_s


@dataclass(frozen=True, slots=True)
class ImuMeasurement:
    sample_step_id: int
    sample_time_s: float
    delivery_step_id: int
    delivery_time_s: float
    acceleration_mps2: float
    yaw_rate_rad_s: float

    def __post_init__(self) -> None:
        _validate_measurement_common(self)
        object.__setattr__(
            self,
            "acceleration_mps2",
            _finite(self.acceleration_mps2, "acceleration_mps2"),
        )
        object.__setattr__(
            self, "yaw_rate_rad_s", _finite(self.yaw_rate_rad_s, "yaw_rate_rad_s")
        )


@dataclass(frozen=True, slots=True)
class WheelSpeedMeasurement:
    sample_step_id: int
    sample_time_s: float
    delivery_step_id: int
    delivery_time_s: float
    speed_mps: float

    def __post_init__(self) -> None:
        _validate_measurement_common(self)
        object.__setattr__(self, "speed_mps", _finite(self.speed_mps, "speed_mps"))
        if self.speed_mps < 0.0:
            raise ValueError("speed_mps cannot be negative")


@dataclass(frozen=True, slots=True)
class GnssMeasurement:
    sample_step_id: int
    sample_time_s: float
    delivery_step_id: int
    delivery_time_s: float
    x_m: float
    y_m: float

    def __post_init__(self) -> None:
        _validate_measurement_common(self)
        object.__setattr__(self, "x_m", _finite(self.x_m, "x_m"))
        object.__setattr__(self, "y_m", _finite(self.y_m, "y_m"))


@dataclass(frozen=True, slots=True)
class LandmarkObservation:
    landmark_id: int
    range_m: float
    bearing_rad: float
    is_outlier: bool = False

    def __post_init__(self) -> None:
        if isinstance(self.landmark_id, bool) or not isinstance(self.landmark_id, int):
            raise TypeError("landmark_id must be an integer")
        if self.landmark_id < 0 or self.landmark_id > 65535:
            raise ValueError("landmark_id must be in [0, 65535]")
        object.__setattr__(self, "range_m", _finite(self.range_m, "range_m"))
        object.__setattr__(self, "bearing_rad", _finite(self.bearing_rad, "bearing_rad"))
        if self.range_m < 0.0:
            raise ValueError("range_m cannot be negative")
        if not isinstance(self.is_outlier, bool):
            raise TypeError("is_outlier must be a boolean")


@dataclass(frozen=True, slots=True)
class LandmarkMeasurement:
    sample_step_id: int
    sample_time_s: float
    delivery_step_id: int
    delivery_time_s: float
    observations: tuple[LandmarkObservation, ...]

    def __post_init__(self) -> None:
        _validate_measurement_common(self)
        if not isinstance(self.observations, tuple) or not all(
            isinstance(item, LandmarkObservation) for item in self.observations
        ):
            raise TypeError("observations must be a tuple of LandmarkObservation values")


Measurement: TypeAlias = (
    ImuMeasurement
    | WheelSpeedMeasurement
    | GnssMeasurement
    | LandmarkMeasurement
)


@dataclass(frozen=True, slots=True)
class SensorEvent:
    step_id: int
    time_s: float
    event_type: str
    sensor: str
    detail: str

    def __post_init__(self) -> None:
        if isinstance(self.step_id, bool) or not isinstance(self.step_id, int) or self.step_id < 0:
            raise ValueError("event step_id must be a non-negative integer")
        object.__setattr__(self, "time_s", _finite(self.time_s, "event time_s"))
        if self.time_s < 0.0:
            raise ValueError("event time_s cannot be negative")
        if not isinstance(self.event_type, str) or not isinstance(self.sensor, str):
            raise TypeError("event_type and sensor must be strings")
        if not isinstance(self.detail, str):
            raise TypeError("event detail must be a string")
        if not self.event_type or not self.sensor:
            raise ValueError("event_type and sensor cannot be empty")


class SensorPipeline:
    """Generate seeded measurements and release them after configured latency."""

    def __init__(
        self,
        *,
        dt_s: float,
        vehicle_parameters: VehicleParameters,
        config: SensorSuiteConfig | None = None,
        landmarks: tuple[Landmark, ...] = (),
        faults: tuple[FaultSpec, ...] = (),
        seed: int,
    ) -> None:
        self.dt_s = _finite(dt_s, "dt_s")
        if self.dt_s <= 0.0:
            raise ValueError("dt_s must be greater than zero")
        if isinstance(seed, bool) or not isinstance(seed, int):
            raise TypeError("seed must be an integer")
        if seed < 0 or seed > (2**64 - 1):
            raise ValueError("seed must be in [0, 2^64 - 1]")

        self.vehicle_parameters = vehicle_parameters
        self.config = config or SensorSuiteConfig()
        self.landmarks = tuple(landmarks)
        self.faults = tuple(faults)
        if not isinstance(self.vehicle_parameters, VehicleParameters):
            raise TypeError("vehicle_parameters must be VehicleParameters")
        if not isinstance(self.config, SensorSuiteConfig):
            raise TypeError("config must be SensorSuiteConfig")
        if not all(isinstance(item, Landmark) for item in self.landmarks):
            raise TypeError("landmarks must contain Landmark values")
        if not all(isinstance(item, FaultSpec) for item in self.faults):
            raise TypeError("faults must contain FaultSpec values")
        if len({landmark.landmark_id for landmark in self.landmarks}) != len(self.landmarks):
            raise ValueError("landmark ids must be unique")

        self._period_steps = {
            "imu": self._sample_period(self.config.imu.rate_hz),
            "wheel_speed": self._sample_period(self.config.wheel_speed.rate_hz),
            "gnss": self._sample_period(self.config.gnss.rate_hz),
            "landmark": self._sample_period(self.config.landmark.rate_hz),
        }
        self._rngs = {
            name: random.Random(_derived_seed(seed, name))
            for name in self._period_steps
        }
        self._pending: list[tuple[int, int, Measurement]] = []
        self._ordinal = 0
        self._last_step = -1
        self._events: list[SensorEvent] = []
        self._active_faults: set[int] = set()
        self._statistics = {
            name: {"generated": 0, "delivered": 0, "dropped": 0, "outliers": 0}
            for name in self._period_steps
        }

    @property
    def pending_count(self) -> int:
        return len(self._pending)

    @property
    def statistics(self) -> dict[str, dict[str, int]]:
        return {name: values.copy() for name, values in self._statistics.items()}

    def drain_events(self) -> list[SensorEvent]:
        events = self._events
        self._events = []
        return events

    def drain_pending(self) -> list[Measurement]:
        """Release delayed end-of-run samples without generating new data."""

        delivered: list[Measurement] = []
        while self._pending:
            _delivery_step, _ordinal, measurement = heapq.heappop(self._pending)
            delivered.append(measurement)
            self._statistics[measurement_kind(measurement)]["delivered"] += 1
        return delivered

    def step(
        self,
        step_id: int,
        time_s: float,
        state: VehicleState,
    ) -> list[Measurement]:
        if isinstance(step_id, bool) or not isinstance(step_id, int):
            raise TypeError("step_id must be an integer")
        if step_id != self._last_step + 1:
            raise ValueError("SensorPipeline.step requires contiguous step_id values")
        time_s = _finite(time_s, "time_s")
        if not isinstance(state, VehicleState):
            raise TypeError("state must be VehicleState")
        expected_time_s = step_id * self.dt_s
        if not math.isclose(time_s, expected_time_s, rel_tol=0.0, abs_tol=1e-12):
            raise ValueError("time_s must equal step_id * dt_s")
        self._last_step = step_id
        self._update_fault_events(step_id, time_s)

        if self.config.imu.enabled and step_id % self._period_steps["imu"] == 0:
            self._sample_imu(step_id, time_s, state)
        if self.config.wheel_speed.enabled and step_id % self._period_steps["wheel_speed"] == 0:
            self._sample_wheel(step_id, time_s, state)
        if self.config.gnss.enabled and step_id % self._period_steps["gnss"] == 0:
            self._sample_gnss(step_id, time_s, state)
        if self.config.landmark.enabled and step_id % self._period_steps["landmark"] == 0:
            self._sample_landmarks(step_id, time_s, state)

        delivered: list[Measurement] = []
        while self._pending and self._pending[0][0] <= step_id:
            _delivery_step, _ordinal, measurement = heapq.heappop(self._pending)
            delivered.append(measurement)
            self._statistics[measurement_kind(measurement)]["delivered"] += 1
        return delivered

    def _sample_period(self, rate_hz: float) -> int:
        ratio = 1.0 / (rate_hz * self.dt_s)
        period = round(ratio)
        if period < 1 or not math.isclose(ratio, period, rel_tol=0.0, abs_tol=1e-10):
            raise ValueError(
                f"sensor rate {rate_hz:g} Hz must be an integer divisor of the {self.dt_s:g} s timestep"
            )
        return period

    def _sample_imu(self, step_id: int, time_s: float, state: VehicleState) -> None:
        name = "imu"
        config = self.config.imu
        rng = self._rngs[name]
        dropout_draw = rng.random()
        acceleration_noise = rng.gauss(0.0, config.acceleration_noise_std_mps2)
        yaw_noise = rng.gauss(0.0, config.yaw_rate_noise_std_rad_s)
        acceleration_bias = config.acceleration_bias_mps2 + self._fault_sum(
            name, "bias_acceleration_mps2", time_s
        )
        yaw_bias = config.yaw_rate_bias_rad_s + self._fault_sum(
            name, "bias_yaw_rate_rad_s", time_s
        )
        yaw_rate = state.speed_mps * math.tan(state.steering_rad) / self.vehicle_parameters.wheelbase_m
        latency = config.latency_s + self._fault_sum(name, "latency_spike", time_s)
        delivery_step, delivery_time = self._delivery(step_id, latency)
        measurement = ImuMeasurement(
            step_id,
            time_s,
            delivery_step,
            delivery_time,
            state.acceleration_mps2 + acceleration_bias + acceleration_noise,
            yaw_rate + yaw_bias + yaw_noise,
        )
        self._record_or_drop(name, measurement, dropout_draw, config.dropout_probability, time_s)

    def _sample_wheel(self, step_id: int, time_s: float, state: VehicleState) -> None:
        name = "wheel_speed"
        config = self.config.wheel_speed
        rng = self._rngs[name]
        dropout_draw = rng.random()
        noise = rng.gauss(0.0, config.noise_std_mps)
        scale_error = config.scale_error + self._fault_sum(name, "scale_error", time_s)
        latency = config.latency_s + self._fault_sum(name, "latency_spike", time_s)
        delivery_step, delivery_time = self._delivery(step_id, latency)
        measurement = WheelSpeedMeasurement(
            step_id,
            time_s,
            delivery_step,
            delivery_time,
            max(0.0, state.speed_mps * (1.0 + scale_error) + noise),
        )
        self._record_or_drop(name, measurement, dropout_draw, config.dropout_probability, time_s)

    def _sample_gnss(self, step_id: int, time_s: float, state: VehicleState) -> None:
        name = "gnss"
        config = self.config.gnss
        rng = self._rngs[name]
        dropout_draw = rng.random()
        noise_x = rng.gauss(0.0, config.position_noise_std_m)
        noise_y = rng.gauss(0.0, config.position_noise_std_m)
        latency = config.latency_s + self._fault_sum(name, "latency_spike", time_s)
        delivery_step, delivery_time = self._delivery(step_id, latency)
        measurement = GnssMeasurement(
            step_id,
            time_s,
            delivery_step,
            delivery_time,
            state.x_m + noise_x,
            state.y_m + noise_y,
        )
        self._record_or_drop(name, measurement, dropout_draw, config.dropout_probability, time_s)

    def _sample_landmarks(self, step_id: int, time_s: float, state: VehicleState) -> None:
        name = "landmark"
        config = self.config.landmark
        rng = self._rngs[name]
        dropout_draw = rng.random()
        forced_outlier = self._fault_active(name, "outlier", time_s)
        observations: list[LandmarkObservation] = []
        for landmark in self.landmarks:
            dx = landmark.x_m - state.x_m
            dy = landmark.y_m - state.y_m
            true_range = math.hypot(dx, dy)
            true_bearing = wrap_angle(math.atan2(dy, dx) - state.heading_rad)
            if true_range > config.max_range_m or abs(true_bearing) > config.field_of_view_rad / 2.0:
                continue

            outlier_draw = rng.random()
            nominal_range_noise = rng.gauss(0.0, config.range_noise_std_m)
            nominal_bearing_noise = rng.gauss(0.0, config.bearing_noise_std_rad)
            outlier_range_noise = rng.gauss(0.0, config.outlier_range_std_m)
            outlier_bearing_noise = rng.gauss(0.0, config.outlier_bearing_std_rad)
            is_outlier = forced_outlier or outlier_draw < config.outlier_probability
            range_noise = outlier_range_noise if is_outlier else nominal_range_noise
            bearing_noise = outlier_bearing_noise if is_outlier else nominal_bearing_noise
            observations.append(
                LandmarkObservation(
                    landmark_id=landmark.landmark_id,
                    range_m=max(0.0, true_range + range_noise),
                    bearing_rad=wrap_angle(true_bearing + bearing_noise),
                    is_outlier=is_outlier,
                )
            )
            if is_outlier:
                self._statistics[name]["outliers"] += 1

        latency = config.latency_s + self._fault_sum(name, "latency_spike", time_s)
        delivery_step, delivery_time = self._delivery(step_id, latency)
        measurement = LandmarkMeasurement(
            step_id,
            time_s,
            delivery_step,
            delivery_time,
            tuple(observations),
        )
        self._record_or_drop(name, measurement, dropout_draw, config.dropout_probability, time_s)

    def _record_or_drop(
        self,
        name: str,
        measurement: Measurement,
        dropout_draw: float,
        dropout_probability: float,
        time_s: float,
    ) -> None:
        self._statistics[name]["generated"] += 1
        if self._fault_active(name, "dropout", time_s) or dropout_draw < dropout_probability:
            self._statistics[name]["dropped"] += 1
            return
        self._ordinal += 1
        heapq.heappush(
            self._pending,
            (measurement.delivery_step_id, self._ordinal, measurement),
        )

    def _delivery(self, sample_step: int, latency_s: float) -> tuple[int, float]:
        delay_steps = max(0, math.ceil(latency_s / self.dt_s - 1e-12))
        delivery_step = sample_step + delay_steps
        return delivery_step, delivery_step * self.dt_s

    def _fault_active(self, target: str, kind: str, time_s: float) -> bool:
        return any(
            fault.target == target and fault.kind == kind and fault.active(time_s)
            for fault in self.faults
        )

    def _fault_sum(self, target: str, kind: str, time_s: float) -> float:
        return sum(
            fault.value
            for fault in self.faults
            if fault.target == target and fault.kind == kind and fault.active(time_s)
        )

    def _update_fault_events(self, step_id: int, time_s: float) -> None:
        now_active = {
            index for index, fault in enumerate(self.faults) if fault.active(time_s)
        }
        for index in sorted(now_active - self._active_faults):
            fault = self.faults[index]
            self._events.append(
                SensorEvent(step_id, time_s, "fault_started", fault.target, fault.kind)
            )
        for index in sorted(self._active_faults - now_active):
            fault = self.faults[index]
            self._events.append(
                SensorEvent(step_id, time_s, "fault_ended", fault.target, fault.kind)
            )
        self._active_faults = now_active


def measurement_kind(measurement: Measurement) -> str:
    if isinstance(measurement, ImuMeasurement):
        return "imu"
    if isinstance(measurement, WheelSpeedMeasurement):
        return "wheel_speed"
    if isinstance(measurement, GnssMeasurement):
        return "gnss"
    if isinstance(measurement, LandmarkMeasurement):
        return "landmark"
    raise TypeError(f"unsupported measurement type: {type(measurement).__name__}")


def _derived_seed(seed: int, stream_name: str) -> int:
    material = f"navbench-sensor-v1:{seed}:{stream_name}".encode("ascii")
    return int.from_bytes(hashlib.sha256(material).digest()[:8], "little")


def _validate_measurement_common(measurement: object) -> None:
    sample_step = getattr(measurement, "sample_step_id")
    delivery_step = getattr(measurement, "delivery_step_id")
    if (
        isinstance(sample_step, bool)
        or not isinstance(sample_step, int)
        or sample_step < 0
    ):
        raise ValueError("sample_step_id must be a non-negative integer")
    if (
        isinstance(delivery_step, bool)
        or not isinstance(delivery_step, int)
        or delivery_step < sample_step
    ):
        raise ValueError("delivery_step_id cannot precede sample_step_id")
    sample_time = _finite(getattr(measurement, "sample_time_s"), "sample_time_s")
    delivery_time = _finite(
        getattr(measurement, "delivery_time_s"), "delivery_time_s"
    )
    if sample_time < 0.0 or delivery_time < sample_time:
        raise ValueError("measurement times are negative or reversed")
    object.__setattr__(measurement, "sample_time_s", sample_time)
    object.__setattr__(measurement, "delivery_time_s", delivery_time)
