"""Deterministic host plant to native C++ controller-in-the-loop runner."""

from __future__ import annotations

import math
import time
import hashlib
from collections import Counter
from dataclasses import asdict, dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

from matplotlib import pyplot as plt

from navbench.metrics import MetricSummary, Pose2, summarize_run
from navbench.native import NativeFirmwareTransport
from navbench.protocol import (
    ControlCommandPayload,
    ControlMode,
    ErrorPayload,
    GnssSample,
    HealthStatusPayload,
    ImuSample,
    LandmarkSample,
    MessageType,
    NavigationMode,
    RouteWaypoint,
    SensorFramePayload,
    SensorMask,
    StateEstimatePayload,
    WheelSpeedSample,
)
from navbench.runlog import CommandMode, CommandRecord, RunLogger, RunReplay
from navbench.scenario import Scenario, scenario_from_mapping
from navbench.sensors import (
    FaultSpec,
    GnssMeasurement,
    ImuMeasurement,
    LandmarkMeasurement,
    Measurement,
    SensorPipeline,
    WheelSpeedMeasurement,
)
from navbench.session import HostSession, SessionEvent, SessionState
from navbench.simulator import (
    ControlCommand,
    SimulationSample,
    VehicleModel,
    VehicleState,
)
from navbench.transport import (
    DeterministicFaultTransport,
    LinkFaultState,
)


_HOST_COMMAND_TIMEOUT_S = 0.10


@dataclass(frozen=True, slots=True)
class EstimateSample:
    step_id: int
    time_s: float
    estimate: StateEstimatePayload


@dataclass(frozen=True, slots=True)
class ControllerSample:
    step_id: int
    time_s: float
    command: ControlCommandPayload


@dataclass(frozen=True, slots=True)
class CilResult:
    scenario_name: str
    seed: int
    plant_samples: tuple[SimulationSample, ...]
    estimates: tuple[EstimateSample, ...]
    controller_commands: tuple[ControllerSample, ...]
    metric_summary: MetricSummary
    summary: dict[str, object]
    run_path: Path | None = None


@dataclass(frozen=True, slots=True)
class NativeReplayResult:
    """Comparison of a recorded sensor stream with the native firmware core."""

    steps_replayed: int
    measurements_replayed: int
    post_run_measurements_ignored: int
    estimate_mismatches: int
    command_mismatches: int
    maximum_estimate_error: float
    maximum_command_error: float

    @property
    def deterministic_match(self) -> bool:
        return self.estimate_mismatches == 0 and self.command_mismatches == 0

    def to_dict(self) -> dict[str, object]:
        return {
            "steps_replayed": self.steps_replayed,
            "measurements_replayed": self.measurements_replayed,
            "post_run_measurements_ignored": self.post_run_measurements_ignored,
            "estimate_mismatches": self.estimate_mismatches,
            "command_mismatches": self.command_mismatches,
            "maximum_estimate_error": self.maximum_estimate_error,
            "maximum_command_error": self.maximum_command_error,
            "deterministic_match": self.deterministic_match,
            "input_scope": "recorded sensor measurements and route/reference only",
        }


def run_closed_loop(
    scenario: Scenario,
    *,
    native_executable: Path,
    output_root: Path | None = None,
    run_name: str | None = None,
    create_dashboard: bool = True,
) -> CilResult:
    """Run one lockstep plant/sensor/Protocol-v1/native-controller experiment."""

    if not scenario.route:
        raise ValueError("closed-loop scenario requires at least one route waypoint")
    if len(scenario.route) > 32:
        raise ValueError("firmware route capacity is 32 waypoints")

    model = VehicleModel(scenario.vehicle)
    state = scenario.initial_state
    sensors = SensorPipeline(
        dt_s=scenario.dt_s,
        vehicle_parameters=scenario.vehicle,
        config=scenario.sensors,
        landmarks=scenario.landmarks,
        faults=scenario.faults,
        seed=scenario.seed,
    )
    landmark_map = {
        landmark.landmark_id: landmark for landmark in scenario.landmarks
    }
    logger = (
        RunLogger(
            output_root,
            scenario,
            run_name=run_name,
            controller_binary_sha256=_file_sha256(native_executable),
        )
        if output_root is not None
        else None
    )

    plant_samples: list[SimulationSample] = []
    estimates: list[EstimateSample] = []
    controller_commands: list[ControllerSample] = []
    health_samples: list[HealthStatusPayload] = []
    native_round_trip_ms: list[float] = []
    safety_counts: Counter[str] = Counter()
    navigation_counts: Counter[str] = Counter()
    active_fault_steps: Counter[str] = Counter()
    protocol_errors = 0
    manual_stop_sent = False
    command_timeout_steps = max(
        1,
        math.ceil(_HOST_COMMAND_TIMEOUT_S / scenario.dt_s - 1.0e-12),
    )
    last_command_step_id: int | None = None
    command_guard_active = False
    command_guard_steps = 0
    stale_controller_packets = 0
    last_reported_safety_state: str | None = None
    last_reported_navigation_mode: str | None = None
    last_estimate = StateEstimatePayload(
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        navigation_mode=NavigationMode.UNAVAILABLE,
    )
    last_command = ControlCommandPayload(
        steering_rad=0.0,
        acceleration_mps2=0.0,
        target_speed_mps=0.0,
        mode=ControlMode.NEUTRAL,
    )

    native = NativeFirmwareTransport(native_executable)
    fault_link = DeterministicFaultTransport(native)
    session = HostSession(fault_link)
    run_path: Path | None = None
    try:
        native.set_time(0)
        fault_link.configure(0, LinkFaultState())
        session.start(0)
        handshake_events = session.poll(0)
        if session.state is not SessionState.ACTIVE:
            raise RuntimeError("native firmware Protocol v1 handshake failed")
        _record_session_events(
            handshake_events,
            0,
            0.0,
            logger,
        )
        _send_scenario_route(session, scenario)

        for step_id in range(scenario.step_count + 1):
            time_s = step_id * scenario.dt_s
            now_ms = round(time_s * 1000.0)
            native.set_time(now_ms)
            link_state = _link_fault_state(
                scenario.faults,
                time_s,
                scenario.dt_s,
            )
            for fault in scenario.faults:
                if fault.active(time_s):
                    active_fault_steps[f"{fault.target}:{fault.kind}"] += 1
            fault_link.configure(step_id, link_state, link_state)

            if (
                session.state is SessionState.TIMED_OUT
                and _link_is_healthy(link_state)
            ):
                session.reconnect(now_ms)
                reconnect_events = session.poll(now_ms)
                _record_transition_event(
                    logger,
                    step_id,
                    time_s,
                    "session_reconnect",
                    "timed_out",
                    session.state.name.lower(),
                )
                _record_session_events(
                    reconnect_events,
                    step_id,
                    time_s,
                    logger,
                )
                if session.state is SessionState.ACTIVE:
                    _send_scenario_route(session, scenario, step_id=step_id)

            delivered = sensors.step(step_id, time_s, state)
            if logger is not None:
                for measurement in delivered:
                    logger.record_measurement(measurement)
                for event in sensors.drain_events():
                    logger.record_event(event)
            else:
                sensors.drain_events()

            if _fault_active(
                scenario.faults,
                "runtime",
                "manual_safe_stop",
                time_s,
            ) and not manual_stop_sent:
                session.request_safe_stop(step_id=step_id)
                manual_stop_sent = True

            exchange_started = time.perf_counter_ns()
            events: tuple[SessionEvent, ...]
            if (
                link_state.disconnected
                or manual_stop_sent
                or session.state is not SessionState.ACTIVE
            ):
                native.tick(now_ms, step_id)
                events = session.poll(now_ms)
            else:
                payload = _sensor_payload(
                    delivered,
                    landmark_map,
                    scenario.faults,
                    time_s,
                    step_id,
                )
                if payload.present_mask:
                    session.send_sensor(step_id, payload)
                    events = session.poll(now_ms)
                else:
                    native.tick(now_ms, step_id)
                    events = session.poll(now_ms)

            if not any(
                event.packet.message_type is MessageType.CONTROL_COMMAND
                for event in events
            ):
                native.tick(now_ms, step_id)
                events = events + session.poll(now_ms)
            native_round_trip_ms.append(
                (time.perf_counter_ns() - exchange_started) / 1_000_000.0
            )

            for event in events:
                if isinstance(event.payload, ControlCommandPayload):
                    if not _controller_event_is_fresh(
                        event,
                        step_id,
                        command_timeout_steps,
                    ):
                        stale_controller_packets += 1
                        continue
                    last_command = event.payload
                    last_command_step_id = event.packet.step_id
                elif isinstance(event.payload, StateEstimatePayload):
                    if not _controller_event_is_fresh(
                        event,
                        step_id,
                        command_timeout_steps,
                    ):
                        stale_controller_packets += 1
                        continue
                    last_estimate = event.payload
                elif isinstance(event.payload, HealthStatusPayload):
                    health_samples.append(event.payload)
                    safety_name = event.payload.runtime_state.name.lower()
                    navigation_name = event.payload.navigation_mode.name.lower()
                    safety_counts[safety_name] += 1
                    navigation_counts[navigation_name] += 1
                    if safety_name != last_reported_safety_state:
                        _record_transition_event(
                            logger,
                            step_id,
                            time_s,
                            "safety_state_transition",
                            last_reported_safety_state,
                            safety_name,
                        )
                        last_reported_safety_state = safety_name
                    if navigation_name != last_reported_navigation_mode:
                        _record_transition_event(
                            logger,
                            step_id,
                            time_s,
                            "navigation_mode_transition",
                            last_reported_navigation_mode,
                            navigation_name,
                        )
                        last_reported_navigation_mode = navigation_name
                elif isinstance(event.payload, ErrorPayload):
                    protocol_errors += 1
            _record_session_events(events, step_id, time_s, logger)

            command_is_fresh = (
                last_command_step_id is not None
                and step_id - last_command_step_id <= command_timeout_steps
            )
            if command_is_fresh:
                effective_command = last_command
                command_source = "embedded_cpp"
            else:
                effective_command = _host_command_timeout_safe_stop(
                    last_estimate,
                    scenario.vehicle.max_deceleration_mps2,
                )
                command_source = "host_command_timeout_safe_stop"
                command_guard_steps += 1
            if command_is_fresh == command_guard_active:
                _record_transition_event(
                    logger,
                    step_id,
                    time_s,
                    "host_command_guard_transition",
                    "active" if command_guard_active else "inactive",
                    "inactive" if command_is_fresh else "active",
                )
                command_guard_active = not command_is_fresh

            plant_command = ControlCommand(
                acceleration_mps2=effective_command.acceleration_mps2,
                steering_rad=effective_command.steering_rad,
            )
            sample = SimulationSample(
                step_id=step_id,
                time_s=time_s,
                state=state,
                command=plant_command,
            )
            plant_samples.append(sample)
            estimates.append(EstimateSample(step_id, time_s, last_estimate))
            controller_commands.append(
                ControllerSample(step_id, time_s, effective_command)
            )
            if logger is not None:
                logger.record_ground_truth(sample)
                logger.record_estimate(
                    {
                        "step_id": step_id,
                        "time_s": time_s,
                        "x_m": last_estimate.x_m,
                        "y_m": last_estimate.y_m,
                        "heading_rad": last_estimate.heading_rad,
                        "speed_mps": last_estimate.speed_mps,
                        "yaw_rate_rad_s": last_estimate.yaw_rate_rps,
                        "acceleration_bias_mps2": (
                            last_estimate.acceleration_bias_mps2
                        ),
                        "navigation_mode": last_estimate.navigation_mode.name,
                        "covariance_diagonal": list(
                            last_estimate.covariance_diagonal
                        ),
                    }
                )
                logger.record_command(
                    step_id,
                    time_s,
                    plant_command,
                    source=command_source,
                    target_speed_mps=effective_command.target_speed_mps,
                    mode=_runlog_command_mode(effective_command.mode),
                    flags=effective_command.flags,
                )
                logger.record_timing(
                    step_id,
                    time_s,
                    "native_process_round_trip",
                    native_round_trip_ms[-1] / 1000.0,
                )

            if step_id < scenario.step_count:
                state = model.step(state, plant_command, scenario.dt_s)

        if logger is not None:
            for measurement in sensors.drain_pending():
                logger.record_measurement(measurement)

        nis_summary = _nis_summary(health_samples)
        metric_summary = summarize_run(
            truth=[
                Pose2(
                    sample.state.x_m,
                    sample.state.y_m,
                    sample.state.heading_rad,
                )
                for sample in plant_samples
            ],
            estimates=[
                Pose2(
                    sample.estimate.x_m,
                    sample.estimate.y_m,
                    sample.estimate.heading_rad,
                )
                for sample in estimates
            ],
            route=[
                (scenario.initial_state.x_m, scenario.initial_state.y_m),
                *((point.x_m, point.y_m) for point in scenario.route),
            ],
            final_speed_mps=plant_samples[-1].state.speed_mps,
            nis_evaluated_count=nis_summary["evaluated_count"],
            nis_sum=nis_summary["sum"],
            nis_maximum=nis_summary["max"],
            max_position_rmse_m=2.0,
            max_cross_track_rmse_m=2.0,
            max_final_stop_error_m=1.0,
            max_final_speed_mps=0.25,
        )
        summary: dict[str, object] = {
            **metric_summary.to_dict(),
            "smoke_thresholds": {
                "position_rmse_m": 2.0,
                "cross_track_rmse_m": 2.0,
                "final_stop_error_m": 1.0,
                "final_speed_mps": 0.25,
            },
            "scenario": scenario.name,
            "seed": scenario.seed,
            "plant_samples": len(plant_samples),
            "sensor_statistics": sensors.statistics,
            "session_statistics": asdict(session.stats),
            "parser_statistics": asdict(session.parser.stats),
            "sequence_statistics": asdict(session.rx_sequence.stats),
            "link_statistics": asdict(fault_link.stats),
            "firmware_health_counters": _firmware_health_counters(
                health_samples
            ),
            "nis_statistics": nis_summary,
            "safety_state_counts": dict(sorted(safety_counts.items())),
            "navigation_mode_counts": dict(sorted(navigation_counts.items())),
            "active_fault_steps": dict(sorted(active_fault_steps.items())),
            "protocol_error_messages": protocol_errors,
            "host_command_timeout_s": _HOST_COMMAND_TIMEOUT_S,
            "host_command_guard_steps": command_guard_steps,
            "stale_controller_packets": stale_controller_packets,
            "native_process_round_trip_ms_p50": _percentile(
                native_round_trip_ms, 0.50
            ),
            "native_process_round_trip_ms_p99": _percentile(
                native_round_trip_ms, 0.99
            ),
            "latency_scope": "host-to-local-native-process round trip",
        }
        if logger is not None:
            run_path = logger.finalize(summary)
        result = CilResult(
            scenario_name=scenario.name,
            seed=scenario.seed,
            plant_samples=tuple(plant_samples),
            estimates=tuple(estimates),
            controller_commands=tuple(controller_commands),
            metric_summary=metric_summary,
            summary=summary,
            run_path=run_path,
        )
        if create_dashboard and run_path is not None:
            save_dashboard(result, run_path / "dashboard.png")
        return result
    except BaseException as error:
        if logger is not None:
            logger.close_incomplete(str(error))
        raise
    finally:
        session.close()


def replay_native_run(
    replay: RunReplay,
    *,
    native_executable: Path,
    numeric_tolerance: float = 1.0e-6,
) -> NativeReplayResult:
    """Replay logged measurements through the common native firmware session.

    The comparison deliberately does not open or iterate ``ground_truth.csv``.
    It reconstructs only the scenario's route/reference map and reuses the
    recorded sensor delivery stream, including its original timestamps.
    """

    if not replay.is_complete:
        raise ValueError("native controller replay requires a complete run")
    if not math.isfinite(numeric_tolerance) or numeric_tolerance < 0.0:
        raise ValueError("numeric_tolerance must be finite and non-negative")
    scenario = scenario_from_mapping(replay.config)
    if not scenario.route:
        raise ValueError("native controller replay requires a recorded route")
    if len(scenario.route) > 32:
        raise ValueError("recorded route exceeds firmware capacity")
    recorded_binary_sha256 = replay.manifest.get("controller_binary_sha256")
    supplied_binary_sha256 = _file_sha256(native_executable)
    if (
        recorded_binary_sha256 is not None
        and recorded_binary_sha256 != supplied_binary_sha256
    ):
        raise ValueError("native executable hash does not match the run manifest")

    measurements = tuple(replay.measurements())
    delivered_by_step: dict[int, list[Measurement]] = {}
    for measurement in measurements:
        delivered_by_step.setdefault(measurement.delivery_step_id, []).append(
            measurement
        )
    logged_estimates = _indexed_estimates(replay, scenario.step_count)
    logged_commands = _indexed_commands(replay, scenario.step_count)
    landmark_map = {
        landmark.landmark_id: landmark for landmark in scenario.landmarks
    }

    last_estimate = StateEstimatePayload(
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        navigation_mode=NavigationMode.UNAVAILABLE,
    )
    last_command = ControlCommandPayload(
        steering_rad=0.0,
        acceleration_mps2=0.0,
        target_speed_mps=0.0,
        mode=ControlMode.NEUTRAL,
    )
    estimate_mismatches = 0
    command_mismatches = 0
    maximum_estimate_error = 0.0
    maximum_command_error = 0.0
    measurements_replayed = 0
    manual_stop_sent = False
    command_timeout_steps = max(
        1,
        math.ceil(_HOST_COMMAND_TIMEOUT_S / scenario.dt_s - 1.0e-12),
    )
    last_command_step_id: int | None = None

    native = NativeFirmwareTransport(native_executable)
    fault_link = DeterministicFaultTransport(native)
    session = HostSession(fault_link)
    try:
        native.set_time(0)
        fault_link.configure(0, LinkFaultState())
        session.start(0)
        session.poll(0)
        if session.state is not SessionState.ACTIVE:
            raise RuntimeError("native firmware Protocol v1 replay handshake failed")
        _send_scenario_route(session, scenario)

        for step_id in range(scenario.step_count + 1):
            time_s = step_id * scenario.dt_s
            now_ms = round(time_s * 1000.0)
            native.set_time(now_ms)
            link_state = _link_fault_state(
                scenario.faults,
                time_s,
                scenario.dt_s,
            )
            fault_link.configure(step_id, link_state, link_state)
            if (
                session.state is SessionState.TIMED_OUT
                and _link_is_healthy(link_state)
            ):
                session.reconnect(now_ms)
                session.poll(now_ms)
                if session.state is SessionState.ACTIVE:
                    _send_scenario_route(session, scenario, step_id=step_id)
            delivered = delivered_by_step.get(step_id, [])
            measurements_replayed += len(delivered)

            if _fault_active(
                scenario.faults,
                "runtime",
                "manual_safe_stop",
                time_s,
            ) and not manual_stop_sent:
                session.request_safe_stop(step_id=step_id)
                manual_stop_sent = True

            if (
                link_state.disconnected
                or manual_stop_sent
                or session.state is not SessionState.ACTIVE
            ):
                native.tick(now_ms, step_id)
                events = session.poll(now_ms)
            else:
                payload = _sensor_payload(
                    delivered,
                    landmark_map,
                    scenario.faults,
                    time_s,
                    step_id,
                )
                if payload.present_mask:
                    session.send_sensor(step_id, payload)
                    events = session.poll(now_ms)
                else:
                    native.tick(now_ms, step_id)
                    events = session.poll(now_ms)
            if not any(
                event.packet.message_type is MessageType.CONTROL_COMMAND
                for event in events
            ):
                native.tick(now_ms, step_id)
                events = events + session.poll(now_ms)

            for event in events:
                if isinstance(event.payload, ControlCommandPayload):
                    if not _controller_event_is_fresh(
                        event,
                        step_id,
                        command_timeout_steps,
                    ):
                        continue
                    last_command = event.payload
                    last_command_step_id = event.packet.step_id
                elif isinstance(event.payload, StateEstimatePayload):
                    if not _controller_event_is_fresh(
                        event,
                        step_id,
                        command_timeout_steps,
                    ):
                        continue
                    last_estimate = event.payload

            if (
                last_command_step_id is None
                or step_id - last_command_step_id > command_timeout_steps
            ):
                effective_command = _host_command_timeout_safe_stop(
                    last_estimate,
                    scenario.vehicle.max_deceleration_mps2,
                )
                effective_source = "host_command_timeout_safe_stop"
            else:
                effective_command = last_command
                effective_source = "embedded_cpp"

            estimate_error, estimate_mode_matches = _estimate_replay_error(
                last_estimate,
                logged_estimates[step_id],
            )
            command_error, command_semantics_match = _command_replay_error(
                effective_command,
                effective_source,
                logged_commands[step_id],
            )
            maximum_estimate_error = max(maximum_estimate_error, estimate_error)
            maximum_command_error = max(maximum_command_error, command_error)
            if estimate_error > numeric_tolerance or not estimate_mode_matches:
                estimate_mismatches += 1
            if command_error > numeric_tolerance or not command_semantics_match:
                command_mismatches += 1
    finally:
        session.close()

    post_run = len(measurements) - measurements_replayed
    return NativeReplayResult(
        steps_replayed=scenario.step_count + 1,
        measurements_replayed=measurements_replayed,
        post_run_measurements_ignored=post_run,
        estimate_mismatches=estimate_mismatches,
        command_mismatches=command_mismatches,
        maximum_estimate_error=maximum_estimate_error,
        maximum_command_error=maximum_command_error,
    )


def save_dashboard(result: CilResult, output_path: Path) -> None:
    times = [sample.time_s for sample in result.plant_samples]
    truth_x = [sample.state.x_m for sample in result.plant_samples]
    truth_y = [sample.state.y_m for sample in result.plant_samples]
    estimate_x = [sample.estimate.x_m for sample in result.estimates]
    estimate_y = [sample.estimate.y_m for sample in result.estimates]
    speed = [sample.state.speed_mps for sample in result.plant_samples]
    target_speed = [
        sample.command.target_speed_mps for sample in result.controller_commands
    ]
    position_error = [
        math.hypot(ex - tx, ey - ty)
        for tx, ty, ex, ey in zip(
            truth_x,
            truth_y,
            estimate_x,
            estimate_y,
            strict=True,
        )
    ]

    figure, axes = plt.subplots(2, 2, figsize=(11, 8))
    axes[0, 0].plot(truth_x, truth_y, label="ground truth")
    axes[0, 0].plot(estimate_x, estimate_y, label="estimate", alpha=0.8)
    axes[0, 0].set_title("Trajectory")
    axes[0, 0].axis("equal")
    axes[0, 0].legend()
    axes[0, 1].plot(times, position_error)
    axes[0, 1].set_title("Position error [m]")
    axes[1, 0].plot(times, speed, label="plant")
    axes[1, 0].plot(times, target_speed, label="target")
    axes[1, 0].set_title("Speed [m/s]")
    axes[1, 0].legend()
    axes[1, 1].axis("off")
    axes[1, 1].text(
        0.0,
        1.0,
        "\n".join(
            [
                f"scenario: {result.scenario_name}",
                f"seed: {result.seed}",
                f"position RMSE: {result.metric_summary.position_rmse_m:.3f} m",
                f"cross-track RMSE: {result.metric_summary.cross_track_rmse_m:.3f} m",
                f"final stop error: {result.metric_summary.final_stop_error_m:.3f} m",
                f"final speed: {result.metric_summary.final_speed_mps:.3f} m/s",
            ]
        ),
        va="top",
        family="monospace",
    )
    for axis in axes.flat:
        if axis.axison:
            axis.grid(True)
            axis.set_xlabel("time [s]" if axis is not axes[0, 0] else "x [m]")
    axes[0, 0].set_ylabel("y [m]")
    figure.tight_layout()
    figure.savefig(output_path, dpi=150)
    plt.close(figure)


def _controller_event_is_fresh(
    event: SessionEvent,
    current_step_id: int,
    maximum_age_steps: int,
) -> bool:
    packet_step_id = event.packet.step_id
    if packet_step_id > current_step_id:
        return False
    return current_step_id - packet_step_id <= maximum_age_steps


def _host_command_timeout_safe_stop(
    estimate: StateEstimatePayload,
    maximum_deceleration_mps2: float,
) -> ControlCommandPayload:
    acceleration = (
        -maximum_deceleration_mps2 if abs(estimate.speed_mps) > 0.02 else 0.0
    )
    return ControlCommandPayload(
        steering_rad=0.0,
        acceleration_mps2=acceleration,
        target_speed_mps=0.0,
        mode=ControlMode.SAFE_STOP,
        flags=0,
    )


def _record_transition_event(
    logger: RunLogger | None,
    step_id: int,
    time_s: float,
    event_type: str,
    previous: str | None,
    current: str,
) -> None:
    if logger is None:
        return
    logger.record_event(
        {
            "step_id": step_id,
            "time_s": time_s,
            "event_type": event_type,
            "source": "host_session",
            "detail": f"{previous or 'none'}->{current}",
            "severity": "warning" if current in {"safe_stop", "fault", "active"} else "info",
        }
    )


def _sensor_payload(
    delivered: list[Measurement],
    landmark_map: dict[int, object],
    faults: tuple[FaultSpec, ...],
    time_s: float,
    step_id: int,
) -> SensorFramePayload:
    imu_measurements = [item for item in delivered if isinstance(item, ImuMeasurement)]
    wheel_measurements = [
        item for item in delivered if isinstance(item, WheelSpeedMeasurement)
    ]
    gnss_measurements = [
        item for item in delivered if isinstance(item, GnssMeasurement)
    ]
    landmark_measurements = [
        item for item in delivered if isinstance(item, LandmarkMeasurement)
    ]
    present = SensorMask(0)
    fault_mask = SensorMask(0)
    imu = ImuSample()
    wheel = WheelSpeedSample()
    gnss = GnssSample()
    landmark = LandmarkSample()

    if imu_measurements:
        item = max(imu_measurements, key=lambda value: value.sample_step_id)
        present |= SensorMask.IMU
        imu = ImuSample(
            item.sample_step_id,
            _timestamp_us(item.sample_time_s),
            item.acceleration_mps2,
            item.yaw_rate_rad_s,
        )
    if wheel_measurements:
        item = max(wheel_measurements, key=lambda value: value.sample_step_id)
        present |= SensorMask.WHEEL_SPEED
        wheel = WheelSpeedSample(
            item.sample_step_id,
            _timestamp_us(item.sample_time_s),
            item.speed_mps,
        )
    if gnss_measurements:
        item = max(gnss_measurements, key=lambda value: value.sample_step_id)
        present |= SensorMask.GNSS
        gnss = GnssSample(
            item.sample_step_id,
            _timestamp_us(item.sample_time_s),
            item.x_m,
            item.y_m,
        )

    observations: list[tuple[LandmarkMeasurement, object]] = []
    for measurement in landmark_measurements:
        for observation in measurement.observations:
            observations.append((measurement, observation))
    if observations:
        observations.sort(
            key=lambda pair: (
                pair[0].sample_step_id,
                pair[1].landmark_id,
            )
        )
        measurement, observation = observations[step_id % len(observations)]
        reference = landmark_map[observation.landmark_id]
        present |= SensorMask.LANDMARK
        if observation.is_outlier:
            fault_mask |= SensorMask.LANDMARK
        landmark = LandmarkSample(
            measurement.sample_step_id,
            _timestamp_us(measurement.sample_time_s),
            observation.landmark_id,
            reference.x_m,
            reference.y_m,
            observation.range_m,
            observation.bearing_rad,
        )

    for fault in faults:
        if not fault.active(time_s):
            continue
        if fault.target == "imu":
            fault_mask |= SensorMask.IMU
        elif fault.target == "wheel_speed":
            fault_mask |= SensorMask.WHEEL_SPEED
        elif fault.target == "gnss":
            fault_mask |= SensorMask.GNSS
        elif fault.target == "landmark":
            fault_mask |= SensorMask.LANDMARK
    return SensorFramePayload(present, fault_mask, imu, wheel, gnss, landmark)


def _send_scenario_route(
    session: HostSession,
    scenario: Scenario,
    *,
    step_id: int = 0,
) -> None:
    session.send_route(
        route_id=1,
        step_id=step_id,
        points=tuple(
            RouteWaypoint(
                point.x_m,
                point.y_m,
                point.target_speed_mps,
                max(0.05, point.acceptance_radius_m),
            )
            for point in scenario.route
        ),
    )


def _link_fault_state(
    faults: tuple[FaultSpec, ...],
    time_s: float,
    dt_s: float,
) -> LinkFaultState:
    active = [
        fault
        for fault in faults
        if fault.active(time_s) and fault.target in {"transport", "host"}
    ]
    delay_s = sum(
        fault.value
        for fault in active
        if fault.target == "transport" and fault.kind == "latency_spike"
    )
    return LinkFaultState(
        packet_loss=any(fault.kind == "packet_loss" for fault in active),
        packet_corruption=any(
            fault.kind == "packet_corruption" for fault in active
        ),
        delay_steps=max(0, math.ceil(delay_s / dt_s - 1.0e-12)),
        stale_frame=any(fault.kind == "stale_frame" for fault in active),
        disconnected=any(fault.kind == "disconnect" for fault in active),
    )


def _link_is_healthy(state: LinkFaultState) -> bool:
    return not (
        state.packet_loss
        or state.packet_corruption
        or state.delay_steps
        or state.stale_frame
        or state.disconnected
    )


def _fault_active(
    faults: tuple[FaultSpec, ...],
    target: str,
    kind: str,
    time_s: float,
) -> bool:
    return any(
        fault.target == target and fault.kind == kind and fault.active(time_s)
        for fault in faults
    )


def _record_session_events(
    events: tuple[SessionEvent, ...],
    step_id: int,
    time_s: float,
    logger: RunLogger | None,
) -> None:
    if logger is None:
        return
    for event in events:
        if isinstance(event.payload, ErrorPayload):
            logger.record_event(
                {
                    "step_id": step_id,
                    "time_s": time_s,
                    "event_type": "protocol_error",
                    "source": "firmware",
                    "detail": (
                        f"code={event.payload.code.name},detail={event.payload.detail}"
                    ),
                    "severity": "warning",
                }
            )


def _nis_summary(health_samples: list[HealthStatusPayload]) -> dict[str, int | float]:
    names = ("imu_yaw", "wheel", "gnss", "landmark")
    if not health_samples:
        return {
            "evaluated_count": 0,
            "gate_rejected_count": 0,
            "sum": 0.0,
            "max": 0.0,
        }
    latest = health_samples[-1]
    evaluated_count = sum(
        getattr(latest, f"{name}_nis_evaluated_count") for name in names
    )
    gate_rejected_count = sum(
        getattr(latest, f"{name}_nis_gate_rejected_count") for name in names
    )
    nis_sum = math.fsum(
        getattr(latest, f"{name}_nis_sum") for name in names
    )
    nis_max = max(getattr(latest, f"{name}_nis_max") for name in names)
    return {
        "evaluated_count": evaluated_count,
        "gate_rejected_count": gate_rejected_count,
        "sum": nis_sum,
        "max": nis_max,
    }


def _firmware_health_counters(
    health_samples: list[HealthStatusPayload],
) -> dict[str, int]:
    names = (
        "rx_frames",
        "rx_crc_errors",
        "rx_decode_errors",
        "rx_missing",
        "rx_duplicates",
        "rx_out_of_order",
        "rx_stale",
        "queue_overflows",
        "scheduler_overruns",
        "max_loop_us",
    )
    return {
        name: max((getattr(health, name) for health in health_samples), default=0)
        for name in names
    }


def _indexed_estimates(
    replay: RunReplay,
    step_count: int,
) -> dict[int, dict[str, object]]:
    indexed: dict[int, dict[str, object]] = {}
    required = {
        "step_id",
        "time_s",
        "x_m",
        "y_m",
        "heading_rad",
        "speed_mps",
        "yaw_rate_rad_s",
        "acceleration_bias_mps2",
        "navigation_mode",
        "covariance_diagonal",
    }
    for record in replay.estimates():
        if set(record) != required:
            raise ValueError("native replay requires the CIL estimate schema")
        step_id = record["step_id"]
        if isinstance(step_id, bool) or not isinstance(step_id, int):
            raise ValueError("recorded estimate step_id is invalid")
        if step_id in indexed:
            raise ValueError("recorded estimates contain duplicate steps")
        _finite_replay_value(record, "time_s")
        for name in (
            "x_m",
            "y_m",
            "heading_rad",
            "speed_mps",
            "yaw_rate_rad_s",
            "acceleration_bias_mps2",
        ):
            _finite_replay_value(record, name)
        covariance = record["covariance_diagonal"]
        if not isinstance(covariance, list) or len(covariance) != 6:
            raise ValueError("recorded estimate covariance is invalid")
        for value in covariance:
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
            ):
                raise ValueError("recorded estimate covariance is invalid")
        if not isinstance(record["navigation_mode"], str):
            raise ValueError("recorded navigation mode is invalid")
        indexed[step_id] = record
    expected = set(range(step_count + 1))
    if set(indexed) != expected:
        raise ValueError("recorded estimates do not cover every scenario step")
    return indexed


def _indexed_commands(
    replay: RunReplay,
    step_count: int,
) -> dict[int, CommandRecord]:
    indexed: dict[int, CommandRecord] = {}
    for record in replay.commands():
        if record.source not in {
            "embedded_cpp",
            "host_command_timeout_safe_stop",
        }:
            raise ValueError("native replay found an unsupported command source")
        if record.step_id in indexed:
            raise ValueError("recorded commands contain duplicate steps")
        indexed[record.step_id] = record
    expected = set(range(step_count + 1))
    if set(indexed) != expected:
        raise ValueError("recorded commands do not cover every scenario step")
    return indexed


def _estimate_replay_error(
    actual: StateEstimatePayload,
    expected: dict[str, object],
) -> tuple[float, bool]:
    actual_values = (
        actual.x_m,
        actual.y_m,
        actual.heading_rad,
        actual.speed_mps,
        actual.yaw_rate_rps,
        actual.acceleration_bias_mps2,
        *actual.covariance_diagonal,
    )
    expected_values = tuple(
        _finite_replay_value(expected, name)
        for name in (
            "x_m",
            "y_m",
            "heading_rad",
            "speed_mps",
            "yaw_rate_rad_s",
            "acceleration_bias_mps2",
        )
    ) + tuple(float(value) for value in expected["covariance_diagonal"])
    maximum = max(
        abs(actual_value - expected_value)
        for actual_value, expected_value in zip(
            actual_values,
            expected_values,
            strict=True,
        )
    )
    return maximum, actual.navigation_mode.name == expected["navigation_mode"]


def _command_replay_error(
    actual: ControlCommandPayload,
    actual_source: str,
    expected: CommandRecord,
) -> tuple[float, bool]:
    maximum = max(
        abs(actual.steering_rad - expected.command.steering_rad),
        abs(actual.acceleration_mps2 - expected.command.acceleration_mps2),
        abs(actual.target_speed_mps - expected.target_speed_mps),
    )
    semantics_match = (
        _runlog_command_mode(actual.mode) is expected.mode
        and actual.flags == expected.flags
        and actual_source == expected.source
    )
    return maximum, semantics_match


def _finite_replay_value(record: dict[str, object], name: str) -> float:
    value = record[name]
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
    ):
        raise ValueError(f"recorded estimate {name} is invalid")
    return float(value)


def _runlog_command_mode(mode: ControlMode) -> CommandMode:
    mapping = {
        ControlMode.NEUTRAL: CommandMode.NEUTRAL,
        ControlMode.TRACKING: CommandMode.TRACKING,
        ControlMode.SAFE_STOP: CommandMode.SAFE_STOP,
    }
    return mapping[mode]


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        while chunk := file.read(128 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def _timestamp_us(time_s: float) -> int:
    value = round(time_s * 1_000_000.0)
    if value < 0 or value > 0xFFFFFFFF:
        raise ValueError("Protocol v1 timestamp_us exceeded uint32 range")
    return value


def _percentile(values: list[float], quantile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    position = (len(ordered) - 1) * quantile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction
