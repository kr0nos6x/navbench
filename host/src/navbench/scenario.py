from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import yaml

from navbench.simulator import (
    ControlCommand,
    SimulationSample,
    VehicleModel,
    VehicleState,
)


@dataclass(frozen=True, slots=True)
class CommandSegment:
    until_s: float
    acceleration_mps2: float
    steering_rad: float


@dataclass(frozen=True, slots=True)
class Scenario:
    name: str
    duration_s: float
    dt_s: float
    initial_state: VehicleState
    commands: tuple[CommandSegment, ...]

    def command_at(self, time_s: float) -> ControlCommand:
        for segment in self.commands:
            if time_s < segment.until_s:
                return ControlCommand(
                    acceleration_mps2=segment.acceleration_mps2,
                    steering_rad=segment.steering_rad,
                )

        final_segment = self.commands[-1]
        return ControlCommand(
            acceleration_mps2=final_segment.acceleration_mps2,
            steering_rad=final_segment.steering_rad,
        )


def load_scenario(path: Path) -> Scenario:
    with path.open(encoding="utf-8") as scenario_file:
        raw = yaml.safe_load(scenario_file)

    if not isinstance(raw, dict):
        raise ValueError("scenario must contain a YAML mapping")

    initial = raw.get("initial_state", {})
    command_data = raw.get("commands", [])

    if not isinstance(initial, dict):
        raise ValueError("initial_state must be a mapping")

    if not isinstance(command_data, list) or not command_data:
        raise ValueError("commands must be a non-empty list")

    commands = tuple(
        CommandSegment(
            until_s=float(command["until_s"]),
            acceleration_mps2=float(command["acceleration_mps2"]),
            steering_rad=math.radians(float(command["steering_deg"])),
        )
        for command in command_data
    )

    scenario = Scenario(
        name=str(raw["name"]),
        duration_s=float(raw["duration_s"]),
        dt_s=float(raw["dt_s"]),
        initial_state=VehicleState(
            x_m=float(initial.get("x_m", 0.0)),
            y_m=float(initial.get("y_m", 0.0)),
            heading_rad=math.radians(
                float(initial.get("heading_deg", 0.0))
            ),
            speed_mps=float(initial.get("speed_mps", 0.0)),
            steering_rad=math.radians(
                float(initial.get("steering_deg", 0.0))
            ),
            acceleration_mps2=float(
                initial.get("acceleration_mps2", 0.0)
            ),
        ),
        commands=commands,
    )

    _validate_scenario(scenario)
    return scenario


def run_scenario(scenario: Scenario) -> list[SimulationSample]:
    model = VehicleModel()
    state = scenario.initial_state
    step_count = round(scenario.duration_s / scenario.dt_s)

    samples: list[SimulationSample] = []

    for step_index in range(step_count + 1):
        time_s = step_index * scenario.dt_s
        command = scenario.command_at(time_s)

        samples.append(
            SimulationSample(
                time_s=time_s,
                state=state,
                command=command,
            )
        )

        if step_index < step_count:
            state = model.step(
                state=state,
                command=command,
                dt_s=scenario.dt_s,
            )

    return samples


def _validate_scenario(scenario: Scenario) -> None:
    if not scenario.name:
        raise ValueError("scenario name cannot be empty")

    if scenario.duration_s <= 0.0:
        raise ValueError("duration_s must be greater than zero")

    if scenario.dt_s <= 0.0:
        raise ValueError("dt_s must be greater than zero")

    previous_end_s = 0.0

    for segment in scenario.commands:
        if segment.until_s <= previous_end_s:
            raise ValueError("command end times must increase")

        previous_end_s = segment.until_s

    if scenario.commands[-1].until_s < scenario.duration_s:
        raise ValueError(
            "the final command must cover the scenario duration"
        )
