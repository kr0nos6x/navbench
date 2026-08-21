# NavBench Architecture

## System boundary

NavBench separates the simulated world from the controller at the Protocol v1
wire boundary.

```text
scenario/config + explicit seed
             |
             v
host plant -> virtual sensors -> Protocol v1/session -> FirmwareSession
     ^                                              |       |       |
     |                                              EKF  guidance  safety
     +--------------- CONTROL_COMMAND <-------------+--- control ---+

ground truth ------> run artifacts and metrics only
```

The host sends timestamped sensor measurements, landmark map coordinates, and
route waypoints. It does not send vehicle pose, plant actuator state, or any
other ground-truth field. The firmware estimator and controller consume only
measurements and reference data. Host metrics compare returned estimates and
commands with truth after the control decision.

## Host responsibilities

`simulator.py` is the canonical plant. Sample `k` is the state at `k * dt`; the
command is held over `[k * dt, (k + 1) * dt)`, and the final sample is not
integrated again. The plant is a kinematic bicycle model with exact first-order
actuator responses and bounded acceleration, steering, and speed.

`scenario.py` validates YAML before execution. Duration, command boundaries,
fault boundaries, and sensor periods must align to the fixed step grid. A
duration of `N * dt` produces `N + 1` state samples. Open-loop and closed-loop
execution both call the same plant step; only the command provider differs.

`sensors.py` owns independent seeded random streams for IMU, wheel speed, GNSS,
and landmarks. Measurement records contain sample and delivery step/time. The
plant state and measurement types are separate, and sensor events describe
dropout, outlier, latency, and configured faults.

`runlog.py` creates a new run directory and writes an `INCOMPLETE` marker before
streaming artifacts. A run contains a normalized config snapshot and hash,
manifest, truth, measurements, estimates, commands, events, timing, and summary.
The manifest records Git state plus, when available, a deterministic
relevant-source-tree digest and, for CIL, the native controller binary digest.
Finalization writes exact counts for all six record streams and removes the
marker. Strict replay parses each stream, requires the exact count schema and
values, validates the config hash and complete summary, and rejects
incomplete/corrupt runs unless partial inspection is explicitly enabled.

With `replay RUN --native NATIVE_EXECUTABLE`, the normalized config snapshot is
parsed back into a validated scenario, its route/reference and recorded sensor
delivery stream are fed through the shared C++ `FirmwareSession`, and regenerated
estimates/commands are compared with the log. A recorded controller-binary hash
must match the supplied executable. Command comparison covers steering,
acceleration, target speed, mode, flags, and source, not only actuator floats.
This controller replay deliberately does not open or iterate `ground_truth.csv`.

`metrics.py` computes pose, heading, cross-track, final-stop, and NIS summaries.
Firmware health reports cumulative evaluated/rejected counts, sum, and maximum
NIS for each sensor family. The host combines only the latest cumulative
snapshot across sensor families, so periodic health frames are not counted
repeatedly; mean NIS is the combined sum divided by combined evaluated count.
`campaign.py` expands fixed seeds across GNSS-aided, landmark-aided, and
dead-reckoning variants, writes aggregate CSV/JSON, and reports missing or
incomplete run artifacts. `cil.py` also produces a minimal PNG dashboard when
requested.

## Protocol, transport, and session

[Protocol v1](PROTOCOL_V1.md) defines COBS framing, CRC-16, versioned typed
messages, bounded payloads, sequence/step identifiers, and parser error
counters. Python and fixed-buffer C++ codecs share golden and invalid fixtures.

The session layer owns HELLO negotiation, sequence acceptance, typed message
dispatch, timeout state, reconnect reset, and statistics. It depends only on a
small byte-transport interface:

- `InMemoryEndpoint` provides deterministic fragmented/combined read tests.
- `DeterministicFaultTransport` buffers through each COBS `0x00` delimiter, then
  applies loss, bit corruption, step delay, stale replay, or disconnect once per
  complete frame. Host-to-controller and controller-to-host directions have
  separate state and counters; CIL scenario faults configure both directions.
- `NativeFirmwareTransport` runs the C++ firmware executable as a persistent
  subprocess without replacing the firmware codec/runtime path.
- `PosixSerialTransport` is a dependency-free, non-blocking raw serial adapter
  for macOS/Linux. Constructing it opens a device; real-device behavior is not
  qualified by computer-only tests and no automatic port discovery is used.

## Firmware responsibilities

`FirmwareSession` is shared by `src/main.cpp` and the native CIL executable. It
owns the incremental parser, sequence tracker, an eight-frame fixed-capacity TX
queue, a four-frame fixed-capacity sensor queue, route assembly, typed dispatch,
periodic health/heartbeat emission, and the `EmbeddedControllerCore`. Route,
sensor, and heartbeat input is rejected until HELLO succeeds. Invalid, corrupt,
duplicate, out-of-order, stale, and oversized input is counted and cannot
directly replace active estimator, route, or control state. Queue overflow
transitions the runtime to safe stop. Sensor frames are consumed at the 20 ms
estimator/control scheduler release, at most once per release.
For each present sensor, `FirmwareSession` rejects a sample from the future or
beyond its configured `sample_step_id` lag relative to the enclosing frame step.
After that source-age gate, the EKF uses controller receive time for sensor-mode
freshness. Wire timestamps remain source-clock metadata and are not compared
with board `millis()`.

The runtime uses no heap allocation, Arduino `String`, exceptions, or dynamic
containers. The controller core uses fixed arrays, a 32-waypoint route, bounded
landmark storage, and cooperative scheduler counters. Its safety states are:

```text
STARTUP -> SELF_TEST -> READY -> RUNNING <-> DEGRADED
                         |          |            |
                         +----------+------------+-> SAFE_STOP
                                    numerical fault -> FAULT
```

READY emits neutral output. RUNNING and DEGRADED permit control while the
estimator is healthy and navigation is available. Host timeout, unavailable
navigation after its grace period, queue overflow, or a latched manual request
causes SAFE_STOP. A numerical estimator failure causes FAULT. Safe-stop and
fault outputs request zero steering/target speed and bounded braking.

The board entry point performs bounded non-blocking serial reads/writes and a
non-blocking built-in LED heartbeat. Board-specific OLED, button, buzzer, and
servo pin mappings are intentionally outside the controller core. The current
firmware session completes its software self-test during reset; it does not
perform or claim a peripheral/electrical self-test.

## Deterministic CIL schedule

For each scenario step the host advances logical time, configures deterministic
link faults, generates/delivers due sensors, sends one aggregate sensor frame or
ticks the runtime, drains controller messages, applies the newest control
command, records artifacts, then advances the plant. Controller estimates and
commands whose packet step is in the future or older than the configured command
window are ignored. If no fresh command remains, the host applies and logs a
typed safe-stop command with zero steering/target speed and bounded braking,
preventing a delayed or replayed controller frame from driving the plant.
Randomness comes only from the scenario seed. Logical outputs are reproducible
for a pinned toolchain; recorded UTC metadata and measured native-process
durations are intentionally not byte-deterministic.

## Qualification boundary

Native CIL exercises the actual C++ protocol, firmware session, EKF, guidance,
control, and safety sources, but it does not measure UNO R4 execution time,
USB-serial latency, runtime stack high-water, electrical behavior, or physical HMI.
The first hardware gate is a clean firmware build followed by an explicit upload
and a bounded Protocol v1 HELLO/route/sensor/safe-stop exchange on a user-selected
serial device.
