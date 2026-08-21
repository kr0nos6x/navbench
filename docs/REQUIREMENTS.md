# NavBench v1 Requirement Closure

`software-verified` means an automated computer/native test exercises the
implemented path. `complete` means the software implementation is present but
the remaining acceptance evidence is physical. `hardware-pending` is never a
claim of board behavior.

| ID | Classification | Evidence or remaining gate |
|---|---|---|
| SIM-001 | software-verified | Fixed-step/seed validation in simulator and scenario tests |
| SIM-002 | software-verified | Repeated open/closed-loop determinism and native replay tests |
| SIM-003 | software-verified | Plant actuator dynamics and saturation tests |
| SEN-001 | software-verified | Seeded rate/noise/bias/latency/dropout/fault tests |
| SEN-002 | software-verified | Typed IMU, wheel, GNSS, and landmark pipeline tests |
| COM-001 | software-verified | One Protocol v1 Python/C++ implementation and session tests |
| COM-002 | software-verified | Shared golden/rejection vectors and 1,000-frame soak |
| COM-003 | software-verified | Parser/session rejection and control-state isolation tests |
| COM-004 | software-verified | Firmware watchdog and host stale-command guard tests |
| EST-001 | software-verified | Six-state IMU prediction and parity fixture |
| EST-002 | software-verified | Wheel/GNSS corrections and covariance tests |
| EST-003 | software-verified | Nonlinear landmark correction and analytic Jacobian test |
| EST-004 | software-verified | Innovation/NIS gate and outlier rejection tests |
| EST-005 | software-verified | Finite/symmetric covariance and numerical fault tests |
| NAV-001 | software-verified | GNSS, landmark, dead-reckoning, unavailable mode tests |
| GNC-001 | software-verified | Fixed waypoint route and final-stop tests |
| GNC-002 | software-verified | Native replay consumes only measurements/reference data |
| GNC-003 | software-verified | Pure Pursuit/PI limits, rate limit, and anti-windup tests |
| SAF-001 | software-verified | Startup/self-test/ready/running/degraded/safe-stop/fault tests |
| SAF-002 | software-verified | Timeout, invalid, queue overflow, manual stop and link-fault tests |
| LOG-001 | software-verified | Managed artifacts, config/source hashes, events, strict counts |
| LOG-002 | software-verified | Hardware-free typed replay and incomplete/corrupt detection |
| HMI-001 | complete; hardware-pending | Mock HAL verifies non-blocking logical states; OLED/LED/buttons/buzzer/SG90 require physical observation/qualification |

## Acceptance boundary

Determinism, protocol soak/rejections, estimator health, closed-loop route/final
stop, native-process latency, link faults, replay, campaign aggregation, and
cross-build resource limits are software gates. The final 20-seed comparison is
valid only when its generated campaign summary reports acceptance. Real serial
round-trip/loop timing, sustained stack high-water, USB/UART behavior, and all
peripheral electrical behavior remain hardware gates.
