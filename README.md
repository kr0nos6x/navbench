# NavBench

NavBench is a deterministic controller-in-the-loop testbed for a simulated
planar vehicle and an Arduino UNO R4 WiFi controller. The host owns plant truth,
virtual sensors, artifacts, replay, metrics, and campaigns. The embedded side
owns Protocol v1 validation, estimation, route following, control, safety, and
the logical HMI. Ground truth is never sent to the controller.

The repository is a `1.0.0rc1` software candidate, not a hardware-qualified
release. See [requirements closure](docs/REQUIREMENTS.md) and [current status](PROJECT.md).

## Software verification

Python 3.13, a C++11 compiler, `uv`, and PlatformIO are required. One command
runs locked-package/build checks, all Python and native C++ tests, shared
Protocol v1 vectors and soak, deterministic native replay, a two-seed campaign
smoke, and clean production/diagnostic UNO R4 cross-builds:

```sh
uv run --project host --locked python tools/verify.py
```

All CLI surfaces, including native CIL, replay, campaign, physical serial, and
binary serial diagnostics, are listed by:

```sh
uv run --project host --locked python -m navbench --help
```

Generated `build/` and `runs/` data are ignored by Git.

## Hardware validation pending

Computer-only and native-process checks do not qualify real USB/UART timing,
UNO runtime stack high-water, physical OLED/LED behavior, or external buttons,
buzzer, and servo wiring. The SG90 adapter is disabled by default and cannot be
enabled without an explicit compile-time physical power/common-ground
qualification flag. No physical pin is assumed for the unqualified modules.

## Technical references

- [Architecture](docs/ARCHITECTURE.md)
- [Protocol v1](docs/PROTOCOL_V1.md)
- [Plant, EKF, guidance, and control](docs/MODEL_EKF.md)
- [Scenario format](docs/SCENARIOS.md)
- [Product requirements](docs/PRD.md)
