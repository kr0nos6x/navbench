# NavBench v1.0.0 Technical Status

The NavBench v1.0.0 implementation is complete for its defined
controller-in-the-loop testbed scope. It has one Python host implementation and
one fixed-memory C++ controller core. The same `FirmwareSession` and Protocol
v1 implementation are used by the native controller-in-the-loop executable and
the Arduino UNO R4 WiFi firmware. The vehicle remains simulated; estimation,
guidance, control, communications, safety, and HMI logic execute through the
embedded code path.

## Implemented system

| Area | v1.0.0 implementation | Verification |
|---|---|---|
| Plant and scenarios | Exact fixed-step kinematic bicycle plant, actuator dynamics, limits, strict scenario validation | Determinism, boundary, saturation, and scenario tests |
| Virtual sensors and faults | Seeded IMU, wheel, GNSS, and landmark models with rate, noise, bias, latency, dropout, slip, outliers, and link/runtime faults | Sensor and fault regression suites |
| Artifacts and replay | Config/hash manifest, typed streams, recoverable incomplete runs, strict replay, metrics, dashboards, and campaign aggregation | Artifact, replay, metrics, and campaign tests |
| Protocol v1 | COBS, CRC-16, typed payloads, bounded incremental parsing, sequence/step policy, HELLO negotiation | Shared Python/C++ golden and rejection vectors plus 1,000-frame soak |
| Host link | In-memory, deterministic fault, native-process, and POSIX serial transports behind one session interface | Transport, session, CIL, and physical serial validation |
| Estimation | Six-state EKF with IMU prediction, wheel/GNSS/landmark corrections, analytic Jacobians, Joseph update, and NIS gating | Jacobian, covariance, gating, and Python/C++ parity tests |
| Guidance and control | Fixed-capacity route manager, Pure Pursuit, PI speed control, anti-windup, saturation, and final stop | Native and closed-loop scenario tests |
| Firmware safety | Cooperative runtime, fixed queues, stale/duplicate rejection, watchdog, startup/ready/running/degraded/safe-stop/fault states | Native safety/fault tests and physical watchdog validation |
| Embedded HMI | Non-blocking OLED and LED presentation plus compile-time button, buzzer, user LED, and safely gated SG90 adapters | Mock-HAL tests; physical OLED and built-in LED validation |
| Automation | Single local verification command and read-only CI workflow | Packaging, Python/native, replay, campaign, and UNO cross-build gates |

The Python reference EKF is a numerical fixture oracle and is not part of the
controller-in-the-loop control path. Firmware estimator, guidance, and control
consume only sensor measurements and route/reference data.

## v1.0.0 verification record

- Python: 105 tests passed.
- Native protocol: 3,393 checks, 13 golden vectors, 10 rejection vectors, and
  a 1,000-frame soak passed.
- Native serial I/O: 10,047 checks passed, including partial/zero writes,
  framing continuity, and diagnostic rate limiting.
- Native EKF, control, runtime, firmware-session, HMI, sanitizer, replay, and
  campaign gates passed.
- Production clean build: 9,216 bytes RAM and 68,448 bytes flash.
- Serial-diagnostic clean build: 9,280 bytes RAM and 68,752 bytes flash.
- Physical diagnostic: 28-byte HELLO accepted; one response; zero drops and
  zero COBS, CRC, or length errors.
- Physical production: 49/49 packets accepted; zero reconnects, timeouts,
  rejections, parser errors, or sequence errors.
- Watchdog: SAFE_STOP observed in `CONTROL_COMMAND` and `HEALTH_STATUS` after
  565 ms without sensor traffic.

NavBench is a controller-in-the-loop embedded testbed. Its measurements and
regression thresholds describe the simulated plant, native process, UNO R4
firmware, and tested serial path; they are not claims of autonomous-vehicle or
automotive safety certification.
