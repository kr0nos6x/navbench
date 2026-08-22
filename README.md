# NavBench

NavBench v1.0.0 is the completed release of a deterministic
controller-in-the-loop navigation and control testbed. A macOS host simulates a
planar vehicle and its sensors while an Arduino UNO R4 WiFi runs the embedded
estimator, route guidance, control, safety runtime, and status interface.
NavBench is an embedded algorithm testbed, not a controller for a physical
autonomous vehicle.

The host owns ground truth, seeded sensor and fault generation, run artifacts,
replay, metrics, plots, and experiment campaigns. The controller receives only
typed sensor measurements and route/reference data through binary Protocol v1;
ground truth is never used by embedded estimation or control.

## Capabilities

- Fixed-step kinematic bicycle plant with actuator dynamics and saturation.
- Configurable IMU, wheel-speed, GNSS, and range-bearing landmark sensors with
  noise, bias, latency, dropout, slip, and outlier models.
- Versioned COBS/CRC-16 binary protocol with typed Python/C++ codecs, bounded
  incremental parsing, sequence/step policy, and session negotiation.
- Fixed-memory UNO R4 firmware with cooperative scheduling, input freshness,
  watchdog, safety FSM, and deterministic SAFE_STOP behavior.
- Six-state planar EKF with GNSS, wheel-speed, IMU, and nonlinear landmark
  updates, analytic Jacobians, Joseph covariance update, and NIS gating.
- GNSS-aided, landmark-aided, dead-reckoning, degraded, and unavailable modes.
- Waypoint route manager, Pure Pursuit lateral guidance, PI speed control,
  anti-windup, command limiting, and final-stop behavior.
- Deterministic native controller-in-the-loop execution, logging, replay,
  fault injection, campaign aggregation, metrics, and minimal dashboards.
- Non-blocking fixed-memory HMI support for SSD1306 OLED, built-in/user LEDs,
  configured buttons and buzzer, and a safely disabled-by-default SG90 steering
  indicator.

## Verification

The primary v1.0.0 verification command covers locked packaging, Python and
native C++ tests, shared Protocol v1 vectors and 1,000-frame soak,
deterministic replay, campaign smoke, and clean production/diagnostic UNO R4
builds:

```sh
uv run --project host --locked python tools/verify.py
```

The release verification completed 105 Python tests. Native checks and
ASan/UBSan runs covered the protocol, EKF, control, runtime, firmware session,
serial I/O, and HMI. Final clean builds used 9,216 bytes RAM and 68,448 bytes
flash for production, and 9,280 bytes RAM and 68,752 bytes flash for the
serial-diagnostic image.

Physical UNO R4 WiFi validation confirmed the SSD1306 display at I2C address
`0x3C`, built-in LED behavior, Protocol v1 HELLO negotiation, normal binary
sensor/control exchange, and watchdog SAFE_STOP. The diagnostic exchange
received the complete 28-byte HELLO and reported one response with no drops or
COBS/CRC/length errors. The production exchange accepted 49 of 49 packets with
no reconnects, timeouts, rejections, sequence errors, or parser errors;
SAFE_STOP was observed through both `CONTROL_COMMAND` and `HEALTH_STATUS` after
565 ms without sensor traffic.

All command surfaces are listed by:

```sh
uv run --project host --locked python -m navbench --help
```

Generated `build/`, `.pio/`, and `runs/` data are ignored by Git. Servo output
remains compile-time disabled unless its external 5 V supply and common ground
have been explicitly qualified.

## Technical references

- [Architecture](docs/ARCHITECTURE.md)
- [Protocol v1](docs/PROTOCOL_V1.md)
- [Plant, EKF, guidance, and control](docs/MODEL_EKF.md)
- [Scenario format](docs/SCENARIOS.md)
- [Product requirements](docs/PRD.md)
- [Requirement verification](docs/REQUIREMENTS.md)
