"""Deterministic fixed-step ground-vehicle plant.

Time semantics are deliberately explicit: sample ``k`` is the state at
``k * dt`` and its command is held over ``[k * dt, (k + 1) * dt)``.  The
final sample is therefore a state observation and is not integrated again.
"""

from __future__ import annotations

import csv
import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass, fields
from decimal import Decimal
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

from matplotlib import pyplot as plt


_FINITE_ERROR = "{name} must be a finite real number"


def _finite(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(_FINITE_ERROR.format(name=name))
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(_FINITE_ERROR.format(name=name))
    return result


@dataclass(frozen=True, slots=True)
class VehicleParameters:
    wheelbase_m: float = 0.32
    steering_time_constant_s: float = 0.18
    acceleration_time_constant_s: float = 0.25
    max_steering_rad: float = math.radians(30.0)
    max_acceleration_mps2: float = 2.0
    max_deceleration_mps2: float = 4.0
    max_speed_mps: float = 8.0

    def __post_init__(self) -> None:
        for item in fields(self):
            value = _finite(getattr(self, item.name), item.name)
            object.__setattr__(self, item.name, value)

        positive = (
            "wheelbase_m",
            "steering_time_constant_s",
            "acceleration_time_constant_s",
            "max_steering_rad",
            "max_acceleration_mps2",
            "max_deceleration_mps2",
            "max_speed_mps",
        )
        for name in positive:
            if getattr(self, name) <= 0.0:
                raise ValueError(f"{name} must be greater than zero")
        if self.max_steering_rad >= math.pi / 2.0:
            raise ValueError("max_steering_rad must be less than pi/2")


@dataclass(frozen=True, slots=True)
class VehicleState:
    x_m: float = 0.0
    y_m: float = 0.0
    heading_rad: float = 0.0
    speed_mps: float = 0.0
    steering_rad: float = 0.0
    acceleration_mps2: float = 0.0

    def __post_init__(self) -> None:
        for item in fields(self):
            value = _finite(getattr(self, item.name), item.name)
            object.__setattr__(self, item.name, value)


@dataclass(frozen=True, slots=True)
class ControlCommand:
    acceleration_mps2: float
    steering_rad: float

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "acceleration_mps2",
            _finite(self.acceleration_mps2, "acceleration_mps2"),
        )
        object.__setattr__(
            self,
            "steering_rad",
            _finite(self.steering_rad, "steering_rad"),
        )


@dataclass(frozen=True, slots=True)
class SimulationSample:
    time_s: float
    state: VehicleState
    command: ControlCommand
    step_id: int = 0

    def __post_init__(self) -> None:
        object.__setattr__(self, "time_s", _finite(self.time_s, "time_s"))
        if isinstance(self.step_id, bool) or not isinstance(self.step_id, int):
            raise TypeError("step_id must be an integer")
        if self.step_id < 0:
            raise ValueError("step_id cannot be negative")
        if not isinstance(self.state, VehicleState):
            raise TypeError("state must be VehicleState")
        if not isinstance(self.command, ControlCommand):
            raise TypeError("command must be ControlCommand")


class VehicleModel:
    """Kinematic bicycle model with first-order actuator dynamics."""

    def __init__(self, parameters: VehicleParameters | None = None) -> None:
        self.parameters = parameters or VehicleParameters()

    def step(
        self,
        state: VehicleState,
        command: ControlCommand,
        dt_s: float,
    ) -> VehicleState:
        dt_s = _finite(dt_s, "dt_s")
        if dt_s <= 0.0:
            raise ValueError("dt_s must be greater than zero")
        if not isinstance(state, VehicleState):
            raise TypeError("state must be VehicleState")
        if not isinstance(command, ControlCommand):
            raise TypeError("command must be ControlCommand")
        self._validate_state(state)

        params = self.parameters
        requested_acceleration = _clamp(
            command.acceleration_mps2,
            -params.max_deceleration_mps2,
            params.max_acceleration_mps2,
        )
        requested_steering = _clamp(
            command.steering_rad,
            -params.max_steering_rad,
            params.max_steering_rad,
        )

        acceleration_response = -math.expm1(
            -dt_s / params.acceleration_time_constant_s
        )
        steering_response = -math.expm1(
            -dt_s / params.steering_time_constant_s
        )
        acceleration = state.acceleration_mps2 + acceleration_response * (
            requested_acceleration - state.acceleration_mps2
        )
        acceleration = _clamp(
            acceleration,
            -params.max_deceleration_mps2,
            params.max_acceleration_mps2,
        )
        steering = state.steering_rad + steering_response * (
            requested_steering - state.steering_rad
        )
        steering = _clamp(
            steering, -params.max_steering_rad, params.max_steering_rad
        )

        speed_unclamped = state.speed_mps + acceleration * dt_s
        speed = _clamp(speed_unclamped, 0.0, params.max_speed_mps)
        if (speed == 0.0 and acceleration < 0.0) or (
            speed == params.max_speed_mps and acceleration > 0.0
        ):
            acceleration = 0.0

        yaw_rate_rad_s = speed * math.tan(steering) / params.wheelbase_m
        heading_midpoint = state.heading_rad + 0.5 * yaw_rate_rad_s * dt_s
        x_m = state.x_m + speed * math.cos(heading_midpoint) * dt_s
        y_m = state.y_m + speed * math.sin(heading_midpoint) * dt_s
        heading_rad = wrap_angle(
            state.heading_rad + yaw_rate_rad_s * dt_s
        )

        next_state = VehicleState(
            x_m=x_m,
            y_m=y_m,
            heading_rad=heading_rad,
            speed_mps=speed,
            steering_rad=steering,
            acceleration_mps2=acceleration,
        )
        self._validate_state(next_state)
        return next_state

    def _validate_state(self, state: VehicleState) -> None:
        params = self.parameters
        tolerance = 1e-12
        if state.speed_mps < -tolerance or state.speed_mps > (
            params.max_speed_mps + tolerance
        ):
            raise ValueError("state speed is outside vehicle limits")
        if abs(state.steering_rad) > params.max_steering_rad + tolerance:
            raise ValueError("state steering is outside vehicle limits")
        if state.acceleration_mps2 < -params.max_deceleration_mps2 - tolerance:
            raise ValueError("state acceleration is outside vehicle limits")
        if state.acceleration_mps2 > params.max_acceleration_mps2 + tolerance:
            raise ValueError("state acceleration is outside vehicle limits")


def simulate_fixed_step(
    *,
    model: VehicleModel,
    initial_state: VehicleState,
    dt_s: float,
    step_count: int,
    command_at: Callable[[int, float], ControlCommand],
) -> list[SimulationSample]:
    """Run the single canonical plant execution path."""

    dt_s = _finite(dt_s, "dt_s")
    if dt_s <= 0.0:
        raise ValueError("dt_s must be greater than zero")
    if isinstance(step_count, bool) or not isinstance(step_count, int):
        raise TypeError("step_count must be an integer")
    if step_count < 1:
        raise ValueError("step_count must be at least one")

    state = initial_state
    samples: list[SimulationSample] = []
    for step_id in range(step_count + 1):
        time_s = step_id * dt_s
        command = command_at(step_id, time_s)
        if not isinstance(command, ControlCommand):
            raise TypeError("command_at must return ControlCommand")
        samples.append(
            SimulationSample(
                step_id=step_id,
                time_s=time_s,
                state=state,
                command=command,
            )
        )
        if step_id != step_count:
            state = model.step(state, command, dt_s)
    return samples


def scripted_command(time_s: float) -> ControlCommand:
    """Legacy demo command provider retained for API compatibility."""

    time_s = _finite(time_s, "time_s")
    if time_s < 4.0:
        return ControlCommand(1.0, 0.0)
    if time_s < 9.0:
        return ControlCommand(0.0, math.radians(18.0))
    if time_s < 14.0:
        return ControlCommand(0.0, math.radians(-18.0))
    if time_s < 18.0:
        return ControlCommand(-0.8, 0.0)
    return ControlCommand(-2.0, 0.0)


def run_open_loop(
    duration_s: float = 20.0,
    dt_s: float = 0.02,
) -> list[SimulationSample]:
    """Run the legacy demo through :func:`simulate_fixed_step`."""

    duration_s = _finite(duration_s, "duration_s")
    dt_s = _finite(dt_s, "dt_s")
    if duration_s <= 0.0 or dt_s <= 0.0:
        raise ValueError("duration_s and dt_s must be greater than zero")
    duration_decimal = Decimal(str(duration_s))
    dt_decimal = Decimal(str(dt_s))
    step_count_decimal, remainder = divmod(duration_decimal, dt_decimal)
    if remainder != 0:
        raise ValueError("duration_s must be an exact integer multiple of dt_s")
    step_count = int(step_count_decimal)
    return simulate_fixed_step(
        model=VehicleModel(),
        initial_state=VehicleState(),
        dt_s=dt_s,
        step_count=step_count,
        command_at=lambda _step_id, time_s: scripted_command(time_s),
    )


def save_csv(
    samples: Sequence[SimulationSample],
    output_path: Path,
    *,
    overwrite: bool = False,
) -> None:
    if not samples:
        raise ValueError("samples cannot be empty")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "step_id",
        "time_s",
        "x_m",
        "y_m",
        "heading_rad",
        "speed_mps",
        "steering_rad",
        "acceleration_mps2",
        "command_acceleration_mps2",
        "command_steering_rad",
    ]
    mode = "w" if overwrite else "x"
    with output_path.open(mode, newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for sample in samples:
            writer.writerow(
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


def save_plot(
    samples: Sequence[SimulationSample],
    output_path: Path,
    *,
    overwrite: bool = False,
) -> None:
    if not samples:
        raise ValueError("samples cannot be empty")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if output_path.exists() and not overwrite:
        raise FileExistsError(output_path)
    times = [sample.time_s for sample in samples]
    x_positions = [sample.state.x_m for sample in samples]
    y_positions = [sample.state.y_m for sample in samples]
    speeds = [sample.state.speed_mps for sample in samples]

    figure, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    axes[0].plot(x_positions, y_positions, linewidth=2)
    axes[0].scatter(
        [x_positions[0], x_positions[-1]],
        [y_positions[0], y_positions[-1]],
        s=45,
    )
    axes[0].set_title("Ground-Truth Trajectory")
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    axes[0].axis("equal")
    axes[0].grid(True)
    axes[1].plot(times, speeds, linewidth=2)
    axes[1].set_title("Vehicle Speed")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("speed [m/s]")
    axes[1].grid(True)
    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def run_demo(output_directory: Path) -> None:
    samples = run_open_loop()
    csv_path = output_directory / "ground_truth.csv"
    plot_path = output_directory / "trajectory.png"
    save_csv(samples, csv_path)
    save_plot(samples, plot_path)
    final_state = samples[-1].state
    print("SIMULATION_COMPLETE")
    print(f"SAMPLES={len(samples)}")
    print(f"FINAL_X_M={final_state.x_m:.6f}")
    print(f"FINAL_Y_M={final_state.y_m:.6f}")
    print(f"FINAL_SPEED_MPS={final_state.speed_mps:.6f}")
    print(f"CSV={csv_path}")
    print(f"PLOT={plot_path}")


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


def wrap_angle(angle_rad: float) -> float:
    angle_rad = _finite(angle_rad, "angle_rad")
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


_wrap_angle = wrap_angle
