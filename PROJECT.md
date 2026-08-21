# NavBench v1.0rc1 Technical Status

NavBench currently has one host implementation and one C++ controller core.
The same C++ `FirmwareSession` is used by the native controller-in-the-loop
executable and the Arduino entry point. The release candidate is suitable for
computer-only verification; it is not yet a hardware-qualified release.

## Implemented software

| Area | Current implementation | Evidence in the repository |
|---|---|---|
| Plant and scenarios | Validated, exact fixed-step kinematic bicycle plant with actuator dynamics and saturations | `host/src/navbench/simulator.py`, `scenario.py`, scenario tests |
| Virtual sensors | Seeded IMU, wheel-speed, GNSS, and range-bearing landmark models with sampling, noise, bias/error, latency, dropout, and outliers | `sensors.py`, sensor tests |
| Artifacts and replay | Config snapshot/hash, exact stream counts, source-tree/controller-binary hashes, typed commands, incomplete marker, and strict replay | `runlog.py`, replay tests |
| Protocol v1 | COBS, CRC-16, typed payloads, bounded incremental parsing, sequence policy, shared golden/invalid vectors | Python/C++ protocol implementations and fixtures |
| Host link | Codec-independent transport/session, bidirectional COBS-frame-aware deterministic faults, native-process adapter, POSIX serial adapter | transport/session/native tests |
| Firmware core | Fixed queues, cooperative safety runtime, step-age validation, receive-clock freshness, six-state EKF, Pure Pursuit, and PI speed control | `include/navbench`, `src`, native tests |
| CIL and faults | Step-driven Protocol v1 loop with a host guard that replaces stale/missing controller commands with bounded safe stop | `cil.py`, `cil_faults.yaml`, CIL tests |
| Metrics and campaign | Estimation/tracking/final-stop and cumulative NIS summaries, dashboards, three-mode campaign, missing-run inspection | `metrics.py`, `campaign.py`, tests |
| Embedded HMI | Navigation-independent non-blocking state presenter, 2 Hz bounded SSD1306 adapter, LED patterns, optional configured buttons/buzzer/user LED, physically gated disabled-by-default SG90 | `hmi.hpp`, `arduino_hmi.hpp`, HMI native tests |
| Automation and CI | One software-only verification command plus read-only GitHub Actions build/test gate | `tools/verify.py`, `.github/workflows/software-verification.yml` |

The Python reference EKF exists only as a numerical oracle for fixtures. It is
not on the controller-in-the-loop control path.

## Acceptance checklist

`Software verified` means the final computer/native gate passed in this
worktree. No item below claims a physical measurement unless explicitly
labelled as a hardware gate.

| Requirement | Status | Remaining gate |
|---|---|---|
| Repeated scenario/seed determinism | Software verified | Deterministic scenario and typed native replay passed |
| Python/C++ Protocol v1 conformance and 1,000-frame soak | Software verified | Shared golden/rejection vectors and soak passed |
| Six-state EKF Jacobians, covariance health, NIS gating, and cross-language fixture parity | Software verified | Python/native suites and parity fixture passed |
| Route following, PI speed control, limits, final stop, and safety transitions | Software verified | Native and four-scenario closed-loop gates passed |
| Bidirectional communication, stale-command, and sensor fault behavior | Software verified | Fault and safety regression gates passed |
| Campaign completeness and three-mode aggregation | Software verified | 20 seeds × 3 modes completed and acceptance passed in the final local run |
| UNO R4 firmware compilation | Build gate | Run a clean `uno_r4_wifi` PlatformIO build |
| Real Protocol v1 handshake and sustained CIL over USB serial | Hardware gate | Upload, open the selected device explicitly, and execute a bounded smoke run |
| Actual serial round-trip timing and loop timing | Hardware gate | Measure on the board; native-process timing is not a substitute |
| Static flash/global RAM footprint | Build gate | Record the linker/build memory report for the final image |
| Runtime stack high-water and actual memory margin | Hardware gate | Measure on the board under a sustained bounded session |
| HMI logical behavior and mock HAL | Software verified | Native mock-HAL test passed |
| SSD1306 at `0x3C` and built-in LED | Hardware gate | Build/upload and observe on the board |
| Buttons/buzzer/user LED/SG90 | Hardware gate | Define explicit board macros, qualify wiring/polarity; externally power SG90 with common ground before enabling |

## Known limitations

- Delayed sensor records preserve their original sample step and timestamp, but
  the EKF applies an accepted late measurement to the current state. There is no
  out-of-sequence rewind, state history, or repropagation in v1. Firmware bounds
  source age in the step domain and uses controller receive time for mode
  freshness; wire timestamps remain source metadata.
- Protocol v1 provides corruption detection, not authentication or encryption.
- A sensor frame carries at most one landmark observation; additional visible
  landmarks require later frames.
- The native process validates software integration, not board timing, USB
  driver behavior, electrical connections, or real serial reconnect behavior.
- `FirmwareSession` currently advances its software self-test to passed during
  reset; it does not qualify board peripherals or electrical health.
- OLED communication failure is isolated from navigation/control and remains a
  physical HMI diagnostic; it does not silently assert controller readiness.
- `commands` in scenario YAML drives open-loop simulation only. Closed-loop
  simulation applies controller commands returned by the C++ firmware path.
- CIL smoke thresholds are regression gates, not a claim of real vehicle or
  hardware performance. The PRD campaign target is judged only from generated
  campaign artifacts.

The project remains simulation-based and is not an automotive safety system or
a controller for a physical autonomous vehicle.
