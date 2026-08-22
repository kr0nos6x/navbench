#!/usr/bin/env python3
"""Run every computer-only NavBench v1 verification gate."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / "build" / "native"
CORE = [
    "src/protocol.cpp",
    "src/ekf.cpp",
    "src/control.cpp",
    "src/runtime.cpp",
    "src/firmware_session.cpp",
]
COMMON_FLAGS = [
    "-std=c++11", "-Os", "-fno-exceptions", "-fno-rtti",
    "-Wall", "-Wextra", "-Wpedantic", "-Werror", "-Iinclude",
]


def run(command: list[str], *, env: dict[str, str] | None = None) -> None:
    print("+", " ".join(command), flush=True)
    subprocess.run(command, cwd=ROOT, env=env, check=True)


def compile_and_run(cxx: str, name: str, sources: list[str],
                    *, define: str | None = None, execute: bool = True) -> Path:
    output = BUILD / name
    command = [cxx, *COMMON_FLAGS]
    if define:
        command.append(f"-D{define}")
    command.extend([*sources, "-o", str(output)])
    run(command)
    if execute:
        run([str(output)])
    return output


def platformio_executable() -> str:
    executable = shutil.which("platformio") or shutil.which("pio")
    if executable:
        return executable
    candidate = Path.home() / ".platformio" / "penv" / "bin" / "platformio"
    if candidate.is_file():
        return str(candidate)
    raise RuntimeError("PlatformIO executable was not found")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run NavBench Python, native, replay, campaign, and UNO build gates."
    )
    parser.add_argument("--skip-platformio", action="store_true")
    parser.add_argument("--skip-campaign", action="store_true")
    args = parser.parse_args()

    BUILD.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    environment.setdefault("UV_CACHE_DIR", str(ROOT / "build" / "uv-cache"))
    environment.setdefault("MPLCONFIGDIR", str(ROOT / "build" / "matplotlib"))
    cxx = environment.get("CXX", "c++")

    compile_and_run(cxx, "test_protocol",
                    ["src/protocol.cpp", "test/native/test_protocol.cpp"])
    compile_and_run(cxx, "test_ekf", ["src/ekf.cpp", "test/native/test_ekf.cpp"])
    compile_and_run(cxx, "test_control",
                    ["src/ekf.cpp", "src/control.cpp", "test/native/test_control.cpp"])
    compile_and_run(cxx, "test_runtime",
                    ["src/ekf.cpp", "src/control.cpp", "src/runtime.cpp",
                     "test/native/test_runtime.cpp"])
    compile_and_run(cxx, "test_firmware_session",
                    [*CORE, "test/native/test_firmware_session.cpp"])
    compile_and_run(cxx, "test_firmware_session_diagnostic",
                    [*CORE, "test/native/test_firmware_session.cpp"],
                    define="NAVBENCH_SERIAL_DIAGNOSTIC=1")
    compile_and_run(cxx, "test_hmi", ["src/hmi.cpp", "test/native/test_hmi.cpp"])
    compile_and_run(cxx, "test_serial_io",
                    [*CORE, "src/serial_io.cpp", "test/native/test_serial_io.cpp"])
    native = compile_and_run(cxx, "navbench_native_firmware",
                             [*CORE, "test/native/native_firmware.cpp"], execute=False)
    ekf_runner = compile_and_run(cxx, "ekf_fixture_runner",
                                 ["src/ekf.cpp", "test/native/ekf_fixture_runner.cpp"],
                                 execute=False)

    run(["uv", "lock", "--project", "host", "--check"], env=environment)
    run(["uv", "sync", "--project", "host", "--locked"], env=environment)
    distribution = ROOT / "build" / "dist"
    if distribution.exists():
        shutil.rmtree(distribution)
    distribution.mkdir(parents=True)
    run(["uv", "build", "--project", "host", "--out-dir", str(distribution)],
        env=environment)
    wheel = next(distribution.glob("navbench-*.whl"))
    with tempfile.TemporaryDirectory(prefix="navbench-install-",
                                     dir=ROOT / "build") as install_temp:
        virtualenv = Path(install_temp) / "venv"
        run(["uv", "venv", str(virtualenv)], env=environment)
        # Dependencies are already locked/synced and exercised below. This
        # isolated no-dependency install verifies that the built wheel itself
        # contains importable package metadata without requiring network I/O.
        run(["uv", "pip", "install", "--offline", "--no-deps", "--python",
             str(virtualenv / "bin" / "python"), str(wheel)], env=environment)
        run([str(virtualenv / "bin" / "python"), "-c",
             "import navbench; print(navbench.__version__)"], env=environment)
    run(["uv", "run", "--project", "host", "--locked", "python", "-m",
         "compileall", "-q", "host/src", "host/tests"], env=environment)
    test_environment = environment | {
        "NAVBENCH_NATIVE_FIRMWARE": str(native),
        "NAVBENCH_EKF_FIXTURE_RUNNER": str(ekf_runner),
        "PYTHONPATH": str(ROOT / "host" / "src"),
    }
    run(["uv", "run", "--project", "host", "--locked", "python", "-m",
         "unittest", "discover", "-s", "host/tests", "-v"], env=test_environment)
    run(["uv", "run", "--project", "host", "--locked", "python", "-m",
         "navbench", "--help"], env=test_environment)
    run(["uv", "run", "--project", "host", "--locked", "navbench", "--help"],
        env=test_environment)

    with tempfile.TemporaryDirectory(prefix="navbench-verify-",
                                     dir=ROOT / "build") as temporary:
        output = Path(temporary)
        run(["uv", "run", "--project", "host", "--locked", "python", "-m",
             "navbench", "cil", "--scenario", "scenarios/straight.yaml",
             "--native", str(native), "--output-root", str(output),
             "--run-name", "replay_smoke", "--no-dashboard"], env=test_environment)
        run(["uv", "run", "--project", "host", "--locked", "python", "-m",
             "navbench", "replay", str(output / "replay_smoke"),
             "--native", str(native)], env=test_environment)
        if not args.skip_campaign:
            run(["uv", "run", "--project", "host", "--locked", "python", "-m",
                 "navbench", "campaign", "--scenario", "scenarios/s_curve.yaml",
                 "--native", str(native), "--output-root", str(output),
                 "--name", "campaign_smoke", "--seeds", "1001,1002"],
                env=test_environment)

    if not args.skip_platformio:
        pio = platformio_executable()
        for profile in ("uno_r4_wifi", "uno_r4_wifi_serial_diagnostic"):
            run([pio, "run", "-e", profile, "-t", "clean"], env=environment)
            run([pio, "run", "-e", profile], env=environment)

    run(["git", "diff", "--check"])
    print("NavBench software verification: PASS")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, subprocess.CalledProcessError, RuntimeError) as error:
        print(f"NavBench software verification: FAIL: {error}", file=sys.stderr)
        raise SystemExit(1)
