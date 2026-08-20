from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

from matplotlib import pyplot as plt


@dataclass(frozen=True, slots=True)
class VehicleParameters:
    wheelbase_m: float = 0.32
    steering_time_constant_s: float = 0.18
    acceleration_time_constant_s: float = 0.25
    max_steering_rad: float = math.radians(30.0)
    max_acceleration_mps2: float = 2.0
    max_deceleration_mps2: float = 4.0
    max_speed_mps: float = 8.0


@dataclass(frozen=True, slots=True)
class VehicleState:
    x_m: float = 0.0
    y_m: float = 0.0
    heading_rad: float = 0.0
    speed_mps: float = 0.0
    steering_rad: float = 0.0
    acceleration_mps2: float = 0.0


@dataclass(frozen=True, slots=True)
class ControlCommand:
    acceleration_mps2: float
    steering_rad: float


@dataclass(frozen=True, slots=True)
class SimulationSample:
    time_s: float
    state: VehicleState
    command: ControlCommand


class VehicleModel:
    def __init__(
        self,
        parameters: VehicleParameters | None = None,
    ) -> None:
        self.parameters = parameters or VehicleParameters()

    def step(
        self,
        state: VehicleState,
        command: ControlCommand,
        dt_s: float,
    ) -> VehicleState:
        if not math.isfinite(dt_s) or dt_s <= 0.0:
            raise ValueError("dt_s must be finite and greater than zero")

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

        acceleration_response = 1.0 - math.exp(
            -dt_s / params.acceleration_time_constant_s
        )
        steering_response = 1.0 - math.exp(
            -dt_s / params.steering_time_constant_s
        )

        acceleration = state.acceleration_mps2 + acceleration_response * (
            requested_acceleration - state.acceleration_mps2
        )
        steering = state.steering_rad + steering_response * (
            requested_steering - state.steering_rad
        )

        speed = _clamp(
            state.speed_mps + acceleration * dt_s,
            0.0,
            params.max_speed_mps,
        )

        yaw_rate_rad_s = speed * math.tan(steering) / params.wheelbase_m
        heading_midpoint = state.heading_rad + 0.5 * yaw_rate_rad_s * dt_s

        x_m = state.x_m + speed * math.cos(heading_midpoint) * dt_s
        y_m = state.y_m + speed * math.sin(heading_midpoint) * dt_s
        heading_rad = _wrap_angle(
            state.heading_rad + yaw_rate_rad_s * dt_s
        )

        return VehicleState(
            x_m=x_m,
            y_m=y_m,
            heading_rad=heading_rad,
            speed_mps=speed,
            steering_rad=steering,
            acceleration_mps2=acceleration,
        )


def scripted_command(time_s: float) -> ControlCommand:
    if time_s < 4.0:
        return ControlCommand(
            acceleration_mps2=1.0,
            steering_rad=0.0,
        )

    if time_s < 9.0:
        return ControlCommand(
            acceleration_mps2=0.0,
            steering_rad=math.radians(18.0),
        )

    if time_s < 14.0:
        return ControlCommand(
            acceleration_mps2=0.0,
            steering_rad=math.radians(-18.0),
        )

    if time_s < 18.0:
        return ControlCommand(
            acceleration_mps2=-0.8,
            steering_rad=0.0,
        )

    return ControlCommand(
        acceleration_mps2=-2.0,
        steering_rad=0.0,
    )


def run_open_loop(
    duration_s: float = 20.0,
    dt_s: float = 0.02,
) -> list[SimulationSample]:
    model = VehicleModel()
    state = VehicleState()
    sample_count = round(duration_s / dt_s)

    samples: list[SimulationSample] = []

    for step_index in range(sample_count + 1):
        time_s = step_index * dt_s
        command = scripted_command(time_s)

        samples.append(
            SimulationSample(
                time_s=time_s,
                state=state,
                command=command,
            )
        )

        if step_index < sample_count:
            state = model.step(state, command, dt_s)

    return samples


def save_csv(
    samples: list[SimulationSample],
    output_path: Path,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)

    fieldnames = [
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

    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        for sample in samples:
            writer.writerow(
                {
                    "time_s": sample.time_s,
                    "x_m": sample.state.x_m,
                    "y_m": sample.state.y_m,
                    "heading_rad": sample.state.heading_rad,
                    "speed_mps": sample.state.speed_mps,
                    "steering_rad": sample.state.steering_rad,
                    "acceleration_mps2": sample.state.acceleration_mps2,
                    "command_acceleration_mps2": (
                        sample.command.acceleration_mps2
                    ),
                    "command_steering_rad": sample.command.steering_rad,
                }
            )


def save_plot(
    samples: list[SimulationSample],
    output_path: Path,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)

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


def _wrap_angle(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi
