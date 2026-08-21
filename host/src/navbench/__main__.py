from __future__ import annotations

import argparse
import json
from pathlib import Path
from time import perf_counter

from navbench.campaign import inspect_campaign, run_campaign
from navbench.cil import replay_native_run, run_closed_loop
from navbench.hardware import (
    HardwareConfig,
    run_physical_validation,
    run_serial_diagnostic,
)
from navbench.runlog import CommandMode, RunEvent, RunLogger, RunReplay
from navbench.scenario import load_scenario, run_scenario
from navbench.simulator import save_plot


def main() -> int:
    parser = argparse.ArgumentParser(
        description="NavBench deterministic simulation and controller-in-the-loop tools."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    open_loop = subparsers.add_parser("open-loop", help="run only the host plant")
    _scenario_argument(open_loop, "scenarios/open_loop_s_curve.yaml")
    open_loop.add_argument("--output-root", type=Path, default=Path("runs"))
    open_loop.add_argument("--run-name")
    open_loop.set_defaults(handler=_open_loop)

    cil = subparsers.add_parser("cil", help="run the native C++ controller in lockstep")
    _scenario_argument(cil, "scenarios/straight.yaml")
    cil.add_argument("--native", type=Path, required=True)
    cil.add_argument("--output-root", type=Path, default=Path("runs"))
    cil.add_argument("--run-name")
    cil.add_argument("--no-dashboard", action="store_true")
    cil.set_defaults(handler=_cil)

    campaign = subparsers.add_parser(
        "campaign", help="run GNSS/landmark/dead-reckoning comparison matrix"
    )
    _scenario_argument(campaign, "scenarios/s_curve.yaml")
    campaign.add_argument("--native", type=Path, required=True)
    campaign.add_argument("--output-root", type=Path, default=Path("runs"))
    campaign.add_argument("--name", default="campaign_20_seed")
    campaign.add_argument(
        "--seeds",
        default=",".join(str(seed) for seed in range(1001, 1021)),
        help="comma-separated unique uint64 seeds",
    )
    campaign.add_argument("--dashboards", action="store_true")
    campaign.set_defaults(handler=_campaign)

    replay = subparsers.add_parser("replay", help="validate and replay a run artifact")
    replay.add_argument("run", type=Path)
    replay.add_argument("--allow-incomplete", action="store_true")
    replay.add_argument(
        "--native",
        type=Path,
        help="replay recorded sensors through the native C++ firmware and compare outputs",
    )
    replay.set_defaults(handler=_replay)

    inspect = subparsers.add_parser(
        "inspect-campaign", help="report failed, missing, or incomplete campaign runs"
    )
    inspect.add_argument("campaign", type=Path)
    inspect.set_defaults(handler=_inspect_campaign)

    hardware = subparsers.add_parser(
        "hardware",
        help="validate Protocol v1 against a physical serial device",
        epilog=(
            "exit codes: 0 success, 10 port open, 20 handshake, "
            "30 normal exchange, 40 watchdog, 50 diagnostic"
        ),
    )
    hardware.add_argument(
        "--port",
        required=True,
        help="serial device, for example /dev/cu.usbmodemXXXX",
    )
    hardware.add_argument("--baud", type=int, default=115200)
    hardware.add_argument(
        "--startup-delay",
        type=float,
        default=3.0,
        metavar="SECONDS",
        help="wait after opening USB CDC before HELLO (default: 3.0)",
    )
    hardware_mode = hardware.add_mutually_exclusive_group()
    hardware_mode.add_argument(
        "--watchdog-check",
        action="store_true",
        help="stop sensor traffic for 600 ms and require SAFE_STOP evidence",
    )
    hardware_mode.add_argument(
        "--diagnostic",
        choices=("usb", "protocol"),
        help="read diagnostic firmware frames and test the selected path",
    )
    hardware.set_defaults(handler=_hardware)

    arguments = parser.parse_args()
    result = arguments.handler(arguments)
    return result if isinstance(result, int) else 0


def _scenario_argument(parser: argparse.ArgumentParser, default: str) -> None:
    parser.add_argument("--scenario", type=Path, default=Path(default))


def _open_loop(arguments: argparse.Namespace) -> int:
    scenario = load_scenario(arguments.scenario)
    with RunLogger(
        arguments.output_root,
        scenario,
        run_name=arguments.run_name,
    ) as logger:
        logger.record_event(RunEvent(0, 0.0, "run_started", "open_loop"))
        started = perf_counter()
        samples = run_scenario(scenario)
        elapsed_s = perf_counter() - started
        for sample in samples:
            logger.record_ground_truth(sample)
            logger.record_command(
                sample.step_id,
                sample.time_s,
                sample.command,
                source="scenario",
                target_speed_mps=0.0,
                mode=CommandMode.OPEN_LOOP,
                flags=0,
            )
        final_sample = samples[-1]
        logger.record_timing(
            final_sample.step_id,
            final_sample.time_s,
            "open_loop_simulation",
            elapsed_s,
        )
        plot_path = logger.path / "trajectory.png"
        save_plot(samples, plot_path)
        logger.record_event(
            RunEvent(
                final_sample.step_id,
                final_sample.time_s,
                "run_completed",
                "open_loop",
            )
        )
        final_state = final_sample.state
        summary = {
            "scenario": scenario.name,
            "samples": len(samples),
            "final_x_m": final_state.x_m,
            "final_y_m": final_state.y_m,
            "final_heading_rad": final_state.heading_rad,
            "final_speed_mps": final_state.speed_mps,
        }
        run_path = logger.finalize(summary)
    print(
        json.dumps(
            {
                "status": "complete",
                **summary,
                "run": str(run_path),
                "csv": str(run_path / "ground_truth.csv"),
                "plot": str(plot_path),
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


def _cil(arguments: argparse.Namespace) -> int:
    scenario = load_scenario(arguments.scenario)
    result = run_closed_loop(
        scenario,
        native_executable=arguments.native,
        output_root=arguments.output_root,
        run_name=arguments.run_name,
        create_dashboard=not arguments.no_dashboard,
    )
    print(json.dumps(result.summary, indent=2, sort_keys=True, allow_nan=False))
    return 0 if result.metric_summary.success else 1


def _campaign(arguments: argparse.Namespace) -> int:
    result = run_campaign(
        load_scenario(arguments.scenario),
        native_executable=arguments.native,
        output_root=arguments.output_root,
        campaign_name=arguments.name,
        seeds=_parse_seeds(arguments.seeds),
        create_dashboards=arguments.dashboards,
    )
    inspected = inspect_campaign(result.path)
    print(json.dumps(inspected, indent=2, sort_keys=True, allow_nan=False))
    return 0 if bool(inspected.get("acceptance_passed")) else 1


def _replay(arguments: argparse.Namespace) -> int:
    replay = RunReplay(arguments.run, allow_incomplete=arguments.allow_incomplete)
    counts = {
        "ground_truth": sum(1 for _ in replay.ground_truth()),
        "measurements": sum(1 for _ in replay.measurements()),
        "estimates": sum(1 for _ in replay.estimates()),
        "commands": sum(1 for _ in replay.commands()),
        "events": sum(1 for _ in replay.events()),
        "timing": sum(1 for _ in replay.timing()),
    }
    result = {
        "status": "complete" if replay.is_complete else "incomplete",
        "run": str(arguments.run),
        "records_replayed": counts,
        "summary": replay.summary(),
    }
    deterministic_match = True
    if arguments.native is not None:
        native_result = replay_native_run(
            replay,
            native_executable=arguments.native,
        )
        result["native_controller_replay"] = native_result.to_dict()
        deterministic_match = native_result.deterministic_match
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0 if replay.is_complete and deterministic_match else 1


def _inspect_campaign(arguments: argparse.Namespace) -> int:
    result = inspect_campaign(arguments.campaign)
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0 if bool(result.get("acceptance_passed")) else 1


def _hardware(arguments: argparse.Namespace) -> int:
    config = HardwareConfig(
        port=arguments.port,
        baud=arguments.baud,
        watchdog_check=arguments.watchdog_check,
        startup_delay_s=arguments.startup_delay,
    )
    result = (
        run_serial_diagnostic(config, arguments.diagnostic)
        if arguments.diagnostic
        else run_physical_validation(config)
    )
    print(json.dumps(result.to_dict(), indent=2, sort_keys=True, allow_nan=False))
    return int(result.exit_code)


def _parse_seeds(text: str) -> tuple[int, ...]:
    try:
        values = tuple(int(value.strip()) for value in text.split(",") if value.strip())
    except ValueError as error:
        raise argparse.ArgumentTypeError("seeds must be comma-separated integers") from error
    if not values:
        raise argparse.ArgumentTypeError("at least one seed is required")
    return values


if __name__ == "__main__":
    raise SystemExit(main())
