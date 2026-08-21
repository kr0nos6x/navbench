# NavBench

NavBench is a deterministic controller-in-the-loop testbed for a simulated
planar ground vehicle and an Arduino UNO R4 WiFi controller. The host owns the
plant, virtual sensors, experiment artifacts, replay, and metrics. The embedded
side owns estimation, route following, control, and safety. Vehicle ground truth
is never encoded in Protocol v1 or passed to the controller.

## Status

The repository is a `1.0.0rc1` software candidate. It contains the fixed-step
plant and scenario engine, seeded sensor and fault models, recoverable run logs,
Protocol v1 codecs, host session/transports, a fixed-memory C++ firmware core,
native controller-in-the-loop execution, bidirectional frame-aware link faults,
a host stale-command safe-stop guard, campaign aggregation, and automated
Python/C++ checks. Complete CIL artifacts record strict stream counts plus
source-tree and controller-binary identity hashes; native replay enforces the
recorded binary hash and typed command semantics as well as numerical output.

This status does not qualify real-board serial behavior, end-to-end hardware
latency, runtime stack high-water/memory margin, physical HMI devices, or
navigation performance on hardware. Static flash/global-RAM use is reported by
the build; the remaining items are hardware gates.

## Run and verify

Use Python 3.13 and the locked dependencies in `host/uv.lock`.

```sh
uv sync --project host --locked

mkdir -p build/native
c++ -std=c++11 -O2 -Wall -Wextra -Wpedantic -Werror -Iinclude \
  src/protocol.cpp src/ekf.cpp src/control.cpp src/runtime.cpp \
  src/firmware_session.cpp test/native/native_firmware.cpp \
  -o build/native/navbench_native_firmware
c++ -std=c++11 -O2 -Wall -Wextra -Wpedantic -Werror -Iinclude \
  src/ekf.cpp test/native/ekf_fixture_runner.cpp \
  -o build/native/ekf_fixture_runner

mkdir -p build/native-tests
for source in test/native/test_*.cpp; do
  name=${source##*/}
  name=${name%.cpp}
  c++ -std=c++11 -O2 -Wall -Wextra -Wpedantic -Werror -Iinclude \
    src/protocol.cpp src/ekf.cpp src/control.cpp src/runtime.cpp \
    src/firmware_session.cpp "$source" -o "build/native-tests/$name"
  "build/native-tests/$name"
done

PYTHONPATH=host/src \
NAVBENCH_NATIVE_FIRMWARE=build/native/navbench_native_firmware \
NAVBENCH_EKF_FIXTURE_RUNNER=build/native/ekf_fixture_runner \
uv run --project host --locked python -m unittest discover -s host/tests -v

PYTHONPATH=host/src uv run --project host --locked python -m navbench cil \
  --scenario scenarios/straight.yaml \
  --native build/native/navbench_native_firmware

PYTHONPATH=host/src uv run --project host --locked python -m navbench replay \
  runs/straight --native build/native/navbench_native_firmware

pio run -e uno_r4_wifi -t clean
pio run -e uno_r4_wifi
```

Native C++ tests are standalone programs. The PlatformIO commands compile only;
firmware upload and serial-port access are separate, explicit hardware
operations.

## Documentation

- [Architecture](docs/ARCHITECTURE.md)
- [Protocol v1](docs/PROTOCOL_V1.md)
- [Plant, EKF, guidance, and control](docs/MODEL_EKF.md)
- [Scenario format](docs/SCENARIOS.md)
- [Product requirements](docs/PRD.md)
- [Current acceptance status](PROJECT.md)
