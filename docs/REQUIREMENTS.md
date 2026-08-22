# NavBench v1.0.0 Requirement Verification

The v1.0 requirements are implemented and covered by repository tests, build
evidence, or the recorded UNO R4 validation. Software verification exercises
the deterministic host and the same fixed-memory C++ controller core used by
the production firmware.

| ID | Status | Verification evidence |
|---|---|---|
| SIM-001 | Verified | Fixed-step and explicit-seed validation in simulator/scenario tests |
| SIM-002 | Verified | Repeated open/closed-loop determinism and native replay |
| SIM-003 | Verified | Vehicle actuator dynamics, limits, and saturation tests |
| SEN-001 | Verified | Seeded rate/noise/bias/latency/dropout/fault tests |
| SEN-002 | Verified | Typed IMU, wheel-speed, GNSS, and landmark pipeline tests |
| COM-001 | Verified | Shared Protocol v1 Python/C++ implementation and session tests |
| COM-002 | Verified | 13 golden vectors, 10 rejection vectors, and 1,000-frame soak |
| COM-003 | Verified | Parser/session rejection and control-state isolation tests |
| COM-004 | Verified | Firmware watchdog, host stale-command guard, and physical SAFE_STOP observation |
| EST-001 | Verified | Six-state IMU prediction and cross-language parity fixture |
| EST-002 | Verified | Wheel-speed/GNSS corrections and covariance tests |
| EST-003 | Verified | Nonlinear landmark correction and analytic Jacobian tests |
| EST-004 | Verified | Innovation/NIS gating and outlier rejection tests |
| EST-005 | Verified | Finite, symmetric covariance and numerical-fault tests |
| NAV-001 | Verified | GNSS, landmark, dead-reckoning, degraded, and unavailable mode tests |
| GNC-001 | Verified | Fixed-capacity waypoint route and final-stop tests |
| GNC-002 | Verified | Native replay consumes only measurements and route/reference data |
| GNC-003 | Verified | Pure Pursuit/PI limits, rate limiting, and anti-windup tests |
| SAF-001 | Verified | Startup, self-test, ready, running, degraded, safe-stop, and fault tests |
| SAF-002 | Verified | Timeout, invalid input, queue overflow, manual stop, and link-fault tests |
| LOG-001 | Verified | Managed artifacts, hashes, events, and strict stream counts |
| LOG-002 | Verified | Hardware-free typed replay and incomplete/corrupt artifact detection |
| HMI-001 | Verified | Non-blocking mock-HAL tests, physical SSD1306/built-in LED validation, and compile-time-gated button/buzzer/user-LED/SG90 adapters |

## Release evidence

The release gate passed 105 Python tests and all native protocol, EKF, control,
runtime, firmware-session, serial-I/O, HMI, sanitizer, replay, and campaign
checks. Production and diagnostic clean builds remained within the UNO R4 WiFi
memory limits at 9,216/68,448 bytes and 9,280/68,752 bytes of RAM/flash,
respectively.

Physical Protocol v1 validation accepted the complete 28-byte HELLO and one
HELLO_ACK with no drops or COBS/CRC/length errors. The production exchange
accepted 49/49 packets with no reconnects, timeouts, rejections, parser errors,
or sequence errors. Removing sensor traffic produced SAFE_STOP evidence in both
`CONTROL_COMMAND` and `HEALTH_STATUS` after 565 ms.

These results qualify NavBench as the specified controller-in-the-loop embedded
testbed. They do not characterize a physical vehicle or constitute automotive
safety certification.
