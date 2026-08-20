from __future__ import annotations

import argparse
from pathlib import Path

from navbench.scenario import load_scenario, run_scenario
from navbench.simulator import save_csv, save_plot


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run a deterministic NavBench vehicle scenario."
    )
    parser.add_argument(
        "--scenario",
        type=Path,
        default=Path("scenarios/open_loop_s_curve.yaml"),
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("runs"),
    )
    arguments = parser.parse_args()

    scenario = load_scenario(arguments.scenario)
    samples = run_scenario(scenario)

    output_directory = arguments.output_root / scenario.name
    csv_path = output_directory / "ground_truth.csv"
    plot_path = output_directory / "trajectory.png"

    save_csv(samples, csv_path)
    save_plot(samples, plot_path)

    final_state = samples[-1].state

    print("SIMULATION_COMPLETE")
    print(f"SCENARIO={scenario.name}")
    print(f"SAMPLES={len(samples)}")
    print(f"FINAL_X_M={final_state.x_m:.6f}")
    print(f"FINAL_Y_M={final_state.y_m:.6f}")
    print(f"FINAL_HEADING_RAD={final_state.heading_rad:.6f}")
    print(f"FINAL_SPEED_MPS={final_state.speed_mps:.6f}")
    print(f"CSV={csv_path}")
    print(f"PLOT={plot_path}")


if __name__ == "__main__":
    main()
