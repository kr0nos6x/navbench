"""Float64 reference implementation of the firmware six-state EKF.

This module is an executable mathematical oracle for deterministic fixtures.  It
is not used by the controller-in-the-loop data path and never receives plant
ground truth except when tests explicitly construct comparison measurements.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import IntEnum

import numpy as np


STATE_SIZE = 6


class UpdateStatus(IntEnum):
    ACCEPTED = 0
    REJECTED_GATE = 1
    INVALID_MEASUREMENT = 2
    NOT_INITIALIZED = 3
    NUMERICAL_FAILURE = 4


class ReferenceNavigationMode(IntEnum):
    UNAVAILABLE = 0
    DEAD_RECKONING = 1
    LANDMARK_AIDED = 2
    GNSS_AIDED = 3
    DEGRADED = 4


@dataclass(frozen=True, slots=True)
class UpdateOutcome:
    status: UpdateStatus
    nis: float = 0.0


@dataclass(frozen=True, slots=True)
class ReferenceState:
    x_m: float = 0.0
    y_m: float = 0.0
    heading_rad: float = 0.0
    speed_mps: float = 0.0
    yaw_rate_rad_s: float = 0.0
    accel_bias_mps2: float = 0.0

    def as_array(self) -> np.ndarray:
        return np.array(
            [
                self.x_m,
                self.y_m,
                self.heading_rad,
                self.speed_mps,
                self.yaw_rate_rad_s,
                self.accel_bias_mps2,
            ],
            dtype=np.float64,
        )

    @classmethod
    def from_array(cls, values: np.ndarray) -> ReferenceState:
        return cls(*(float(value) for value in values))


@dataclass(frozen=True, slots=True)
class ReferenceEkfConfig:
    initial_variance: tuple[float, ...] = (
        4.0,
        4.0,
        0.25,
        1.0,
        0.09,
        0.25,
    )
    process_noise_per_second: tuple[float, ...] = (
        0.0025,
        0.0025,
        0.001,
        0.04,
        1.0,
        0.0001,
    )
    imu_yaw_rate_variance: float = 0.0025
    wheel_speed_variance: float = 0.04
    gnss_x_variance: float = 1.0
    gnss_y_variance: float = 1.0
    landmark_range_variance: float = 0.09
    landmark_bearing_variance: float = 0.0025
    imu_yaw_nis_gate: float = 6.635
    wheel_nis_gate: float = 6.635
    gnss_nis_gate: float = 9.210
    landmark_nis_gate: float = 9.210
    minimum_landmark_range_m: float = 0.05
    maximum_prediction_dt_s: float = 0.25
    covariance_floor: float = 1.0e-8
    maximum_covariance: float = 1.0e6
    imu_timeout_ms: int = 250
    wheel_timeout_ms: int = 500
    gnss_timeout_ms: int = 1500
    landmark_timeout_ms: int = 1000

    def __post_init__(self) -> None:
        if len(self.initial_variance) != STATE_SIZE:
            raise ValueError("initial_variance must contain six values")
        if len(self.process_noise_per_second) != STATE_SIZE:
            raise ValueError("process noise must contain six values")
        positive = (
            *self.initial_variance,
            self.imu_yaw_rate_variance,
            self.wheel_speed_variance,
            self.gnss_x_variance,
            self.gnss_y_variance,
            self.landmark_range_variance,
            self.landmark_bearing_variance,
            self.imu_yaw_nis_gate,
            self.wheel_nis_gate,
            self.gnss_nis_gate,
            self.landmark_nis_gate,
            self.minimum_landmark_range_m,
            self.maximum_prediction_dt_s,
            self.covariance_floor,
            self.maximum_covariance,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive):
            raise ValueError("EKF positive configuration values are invalid")
        if any(
            not math.isfinite(value) or value < 0.0
            for value in self.process_noise_per_second
        ):
            raise ValueError("process noise must be finite and non-negative")
        timeouts = (
            self.imu_timeout_ms,
            self.wheel_timeout_ms,
            self.gnss_timeout_ms,
            self.landmark_timeout_ms,
        )
        if any(value <= 0 for value in timeouts):
            raise ValueError("sensor timeouts must be positive")


@dataclass(slots=True)
class ReferenceStats:
    predictions: int = 0
    accepted: dict[str, int] = field(default_factory=dict)
    rejected: dict[str, int] = field(default_factory=dict)
    last_nis: dict[str, float] = field(default_factory=dict)


class ReferenceEkf6:
    def __init__(self, config: ReferenceEkfConfig | None = None) -> None:
        self.config = config or ReferenceEkfConfig()
        self.state = np.zeros(STATE_SIZE, dtype=np.float64)
        self.covariance = np.zeros((STATE_SIZE, STATE_SIZE), dtype=np.float64)
        self.initialized = False
        self.healthy = False
        self.stats = ReferenceStats()
        self._last_sensor_ms: dict[str, int] = {}

    def initialize(
        self,
        state: ReferenceState,
        timestamp_ms: int = 0,
        covariance_diagonal: tuple[float, ...] | None = None,
    ) -> None:
        del timestamp_ms
        values = state.as_array()
        diagonal = np.asarray(
            covariance_diagonal or self.config.initial_variance,
            dtype=np.float64,
        )
        if diagonal.shape != (STATE_SIZE,):
            raise ValueError("covariance diagonal must contain six values")
        if not np.all(np.isfinite(values)):
            raise ValueError("state must be finite")
        if not np.all(np.isfinite(diagonal)) or np.any(diagonal <= 0.0):
            raise ValueError("covariance diagonal must be finite and positive")
        if np.any(diagonal > self.config.maximum_covariance):
            raise ValueError("initial covariance exceeds configured maximum")
        values[2] = wrap_angle(float(values[2]))
        self.state = values
        self.covariance = np.diag(diagonal)
        self.initialized = True
        self.healthy = True
        self.stats = ReferenceStats()
        self._last_sensor_ms.clear()

    @staticmethod
    def prediction_model(
        state: ReferenceState,
        longitudinal_accel_mps2: float,
        dt_s: float,
    ) -> tuple[ReferenceState, np.ndarray]:
        values = state.as_array()
        if not np.all(np.isfinite(values)):
            raise ValueError("state must be finite")
        if not math.isfinite(longitudinal_accel_mps2):
            raise ValueError("acceleration must be finite")
        if not math.isfinite(dt_s) or dt_s <= 0.0:
            raise ValueError("dt_s must be finite and positive")

        heading = values[2]
        cosine = math.cos(heading)
        sine = math.sin(heading)
        acceleration = longitudinal_accel_mps2 - values[5]
        distance = values[3] * dt_s + 0.5 * acceleration * dt_s * dt_s
        predicted = values.copy()
        predicted[0] += distance * cosine
        predicted[1] += distance * sine
        predicted[2] = wrap_angle(values[2] + values[4] * dt_s)
        predicted[3] += acceleration * dt_s

        jacobian = np.eye(STATE_SIZE, dtype=np.float64)
        jacobian[0, 2] = -distance * sine
        jacobian[0, 3] = dt_s * cosine
        jacobian[0, 5] = -0.5 * dt_s * dt_s * cosine
        jacobian[1, 2] = distance * cosine
        jacobian[1, 3] = dt_s * sine
        jacobian[1, 5] = -0.5 * dt_s * dt_s * sine
        jacobian[2, 4] = dt_s
        jacobian[3, 5] = -dt_s
        return ReferenceState.from_array(predicted), jacobian

    @staticmethod
    def landmark_model_and_jacobian(
        state: ReferenceState,
        landmark_x_m: float,
        landmark_y_m: float,
        minimum_range_m: float = 0.05,
    ) -> tuple[np.ndarray, np.ndarray]:
        values = state.as_array()
        if not all(
            math.isfinite(value)
            for value in (landmark_x_m, landmark_y_m, minimum_range_m)
        ) or minimum_range_m <= 0.0:
            raise ValueError("landmark inputs are invalid")
        dx = landmark_x_m - values[0]
        dy = landmark_y_m - values[1]
        range_squared = dx * dx + dy * dy
        if range_squared < minimum_range_m * minimum_range_m:
            raise ValueError("landmark is too close to linearize")
        range_m = math.sqrt(range_squared)
        predicted = np.array(
            [range_m, wrap_angle(math.atan2(dy, dx) - values[2])],
            dtype=np.float64,
        )
        jacobian = np.zeros((2, STATE_SIZE), dtype=np.float64)
        jacobian[0, 0] = -dx / range_m
        jacobian[0, 1] = -dy / range_m
        jacobian[1, 0] = dy / range_squared
        jacobian[1, 1] = -dx / range_squared
        jacobian[1, 2] = -1.0
        return predicted, jacobian

    def predict(
        self,
        *,
        longitudinal_accel_mps2: float,
        yaw_rate_rad_s: float,
        timestamp_ms: int,
        dt_s: float,
    ) -> UpdateOutcome:
        if not self.initialized:
            return UpdateOutcome(UpdateStatus.NOT_INITIALIZED)
        if not self.healthy:
            return UpdateOutcome(UpdateStatus.NUMERICAL_FAILURE)
        if (
            not math.isfinite(dt_s)
            or dt_s <= 0.0
            or dt_s > self.config.maximum_prediction_dt_s
            or not math.isfinite(longitudinal_accel_mps2)
            or not math.isfinite(yaw_rate_rad_s)
        ):
            return UpdateOutcome(UpdateStatus.INVALID_MEASUREMENT)
        predicted, jacobian = self.prediction_model(
            ReferenceState.from_array(self.state),
            longitudinal_accel_mps2,
            dt_s,
        )
        self.state = predicted.as_array()
        process = np.asarray(
            self.config.process_noise_per_second,
            dtype=np.float64,
        ) * dt_s
        self.covariance = (
            jacobian @ self.covariance @ jacobian.T + np.diag(process)
        )
        if not self._enforce_health():
            return UpdateOutcome(UpdateStatus.NUMERICAL_FAILURE)
        self.stats.predictions += 1
        self._last_sensor_ms["imu"] = timestamp_ms
        outcome = self._linear_update(
            np.array([yaw_rate_rad_s - self.state[4]]),
            np.eye(1, STATE_SIZE, 4),
            np.array([[self.config.imu_yaw_rate_variance]]),
            self.config.imu_yaw_nis_gate,
        )
        self._record("imu", outcome)
        return outcome

    def update_wheel(self, speed_mps: float, timestamp_ms: int) -> UpdateOutcome:
        if not self.initialized:
            return UpdateOutcome(UpdateStatus.NOT_INITIALIZED)
        if not math.isfinite(speed_mps):
            return UpdateOutcome(UpdateStatus.INVALID_MEASUREMENT)
        h = np.eye(1, STATE_SIZE, 3)
        outcome = self._linear_update(
            np.array([speed_mps - self.state[3]]),
            h,
            np.array([[self.config.wheel_speed_variance]]),
            self.config.wheel_nis_gate,
        )
        if outcome.status is UpdateStatus.ACCEPTED:
            self._last_sensor_ms["wheel"] = timestamp_ms
        self._record("wheel", outcome)
        return outcome

    def update_gnss(
        self,
        x_m: float,
        y_m: float,
        timestamp_ms: int,
    ) -> UpdateOutcome:
        if not self.initialized:
            return UpdateOutcome(UpdateStatus.NOT_INITIALIZED)
        if not math.isfinite(x_m) or not math.isfinite(y_m):
            return UpdateOutcome(UpdateStatus.INVALID_MEASUREMENT)
        h = np.zeros((2, STATE_SIZE), dtype=np.float64)
        h[0, 0] = 1.0
        h[1, 1] = 1.0
        outcome = self._linear_update(
            np.array([x_m - self.state[0], y_m - self.state[1]]),
            h,
            np.diag(
                [self.config.gnss_x_variance, self.config.gnss_y_variance]
            ),
            self.config.gnss_nis_gate,
        )
        if outcome.status is UpdateStatus.ACCEPTED:
            self._last_sensor_ms["gnss"] = timestamp_ms
        self._record("gnss", outcome)
        return outcome

    def update_landmark(
        self,
        *,
        landmark_x_m: float,
        landmark_y_m: float,
        range_m: float,
        bearing_rad: float,
        timestamp_ms: int,
    ) -> UpdateOutcome:
        if not self.initialized:
            return UpdateOutcome(UpdateStatus.NOT_INITIALIZED)
        if not all(
            math.isfinite(value)
            for value in (
                landmark_x_m,
                landmark_y_m,
                range_m,
                bearing_rad,
            )
        ) or range_m < self.config.minimum_landmark_range_m:
            return UpdateOutcome(UpdateStatus.INVALID_MEASUREMENT)
        try:
            predicted, h = self.landmark_model_and_jacobian(
                ReferenceState.from_array(self.state),
                landmark_x_m,
                landmark_y_m,
                self.config.minimum_landmark_range_m,
            )
        except ValueError:
            return UpdateOutcome(UpdateStatus.INVALID_MEASUREMENT)
        innovation = np.array(
            [range_m - predicted[0], wrap_angle(bearing_rad - predicted[1])]
        )
        outcome = self._linear_update(
            innovation,
            h,
            np.diag(
                [
                    self.config.landmark_range_variance,
                    self.config.landmark_bearing_variance,
                ]
            ),
            self.config.landmark_nis_gate,
            normalize_second=True,
        )
        if outcome.status is UpdateStatus.ACCEPTED:
            self._last_sensor_ms["landmark"] = timestamp_ms
        self._record("landmark", outcome)
        return outcome

    def navigation_mode(self, now_ms: int) -> ReferenceNavigationMode:
        if not self.initialized or not self.healthy:
            return ReferenceNavigationMode.UNAVAILABLE
        if self._fresh("gnss", now_ms, self.config.gnss_timeout_ms):
            return ReferenceNavigationMode.GNSS_AIDED
        if self._fresh("landmark", now_ms, self.config.landmark_timeout_ms):
            return ReferenceNavigationMode.LANDMARK_AIDED
        imu = self._fresh("imu", now_ms, self.config.imu_timeout_ms)
        wheel = self._fresh("wheel", now_ms, self.config.wheel_timeout_ms)
        if imu and wheel:
            return ReferenceNavigationMode.DEAD_RECKONING
        if imu or wheel:
            return ReferenceNavigationMode.DEGRADED
        return ReferenceNavigationMode.UNAVAILABLE

    def _linear_update(
        self,
        innovation: np.ndarray,
        h: np.ndarray,
        measurement_covariance: np.ndarray,
        gate: float,
        *,
        normalize_second: bool = False,
    ) -> UpdateOutcome:
        if not self.initialized:
            return UpdateOutcome(UpdateStatus.NOT_INITIALIZED)
        if not self.healthy or not np.all(np.isfinite(innovation)):
            return UpdateOutcome(UpdateStatus.NUMERICAL_FAILURE)
        innovation = innovation.copy()
        if normalize_second and len(innovation) > 1:
            innovation[1] = wrap_angle(float(innovation[1]))
        innovation_covariance = h @ self.covariance @ h.T + measurement_covariance
        try:
            inverse = np.linalg.inv(innovation_covariance)
        except np.linalg.LinAlgError:
            return UpdateOutcome(UpdateStatus.NUMERICAL_FAILURE)
        nis = float(innovation.T @ inverse @ innovation)
        if not math.isfinite(nis) or nis < 0.0:
            return UpdateOutcome(UpdateStatus.NUMERICAL_FAILURE)
        if nis > gate:
            return UpdateOutcome(UpdateStatus.REJECTED_GATE, nis)

        gain = self.covariance @ h.T @ inverse
        previous_state = self.state.copy()
        previous_covariance = self.covariance.copy()
        self.state = self.state + gain @ innovation
        self.state[2] = wrap_angle(float(self.state[2]))
        identity_minus_gain_h = np.eye(STATE_SIZE) - gain @ h
        self.covariance = (
            identity_minus_gain_h
            @ self.covariance
            @ identity_minus_gain_h.T
            + gain @ measurement_covariance @ gain.T
        )
        if not self._enforce_health():
            self.state = previous_state
            self.covariance = previous_covariance
            self.healthy = False
            return UpdateOutcome(UpdateStatus.NUMERICAL_FAILURE, nis)
        return UpdateOutcome(UpdateStatus.ACCEPTED, nis)

    def _enforce_health(self) -> bool:
        if not np.all(np.isfinite(self.state)) or not np.all(
            np.isfinite(self.covariance)
        ):
            self.healthy = False
            return False
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        diagonal = np.diag(self.covariance).copy()
        if np.any(diagonal < -self.config.covariance_floor) or np.any(
            diagonal > self.config.maximum_covariance
        ):
            self.healthy = False
            return False
        for index, value in enumerate(diagonal):
            if value < self.config.covariance_floor:
                self.covariance[index, index] = self.config.covariance_floor
        self.healthy = True
        return True

    def _fresh(self, name: str, now_ms: int, timeout_ms: int) -> bool:
        last = self._last_sensor_ms.get(name)
        return last is not None and 0 <= now_ms - last <= timeout_ms

    def _record(self, name: str, outcome: UpdateOutcome) -> None:
        self.stats.last_nis[name] = outcome.nis
        target = (
            self.stats.accepted
            if outcome.status is UpdateStatus.ACCEPTED
            else self.stats.rejected
        )
        target[name] = target.get(name, 0) + 1


def wrap_angle(angle_rad: float) -> float:
    if not math.isfinite(angle_rad):
        raise ValueError("angle must be finite")
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi
