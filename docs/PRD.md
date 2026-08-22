# NavBench v1.0.0 Product Requirements

## 1. Summary

NavBench is a controller-in-the-loop testbed for developing and evaluating embedded navigation and control algorithms under GNSS-aided and GNSS-denied conditions. Version 1.0.0 implements the requirements in this document and is verified by the repository test/build gates and recorded UNO R4 serial validation.

A host computer simulates a ground vehicle and its sensors. An Arduino UNO R4 WiFi receives sensor measurements, estimates the vehicle state, generates control commands, and supervises system safety.

## 2. Problem

Navigation and control algorithms are often evaluated entirely in simulation or introduced directly onto physical hardware. The first approach does not expose embedded limitations, while the second makes communication, estimation, control, and electrical problems difficult to isolate.

NavBench provides an intermediate environment in which the vehicle remains simulated but the navigation and control software executes on a real microcontroller with limited memory, processing capacity, and communication bandwidth.

## 3. Goals

- Create a deterministic and reproducible vehicle simulation.
- Execute estimation, guidance, control, and safety logic on real embedded hardware.
- Evaluate GNSS-aided and GNSS-denied navigation using repeatable scenarios.
- Compare landmark-aided localization against dead reckoning.
- Measure estimation error, tracking error, latency, and resource usage.
- Reproduce failures using recorded logs and deterministic replay.
- Demonstrate safe behavior during sensor, communication, and runtime faults.

## 4. System Responsibilities

### Host Computer

The host computer shall:

- Simulate vehicle motion and ground truth.
- Generate virtual sensor measurements.
- Send timestamped measurements to the Arduino.
- Apply returned control commands to the simulated vehicle.
- Record configurations, measurements, commands, events, and ground truth.
- Replay recorded experiments without requiring the Arduino.
- Generate plots and summary metrics.

### Embedded Controller

The Arduino shall:

- Validate incoming protocol frames.
- Reject corrupt, duplicated, invalid, or stale data.
- Estimate vehicle state using an Extended Kalman Filter.
- Determine navigation availability from sensor health.
- Execute waypoint guidance and path-following control.
- Enforce actuator limits and safety states.
- Return control commands, estimates, health data, and timing information.
- Drive the embedded status interface without disturbing control timing.

## 5. Functional Requirements

| ID | Requirement |
|---|---|
| SIM-001 | The simulator shall use a fixed timestep and configurable random seed. |
| SIM-002 | Repeating the same scenario, seed, and input sequence shall reproduce the same trajectory. |
| SIM-003 | The vehicle model shall include steering and speed limits plus basic actuator dynamics. |
| SEN-001 | Virtual sensors shall support configurable rate, noise, bias, latency, and dropout. |
| SEN-002 | Supported measurements shall include IMU, wheel speed, GNSS position, and landmark range-bearing observations. |
| COM-001 | Host and firmware shall use a versioned binary serial protocol. |
| COM-002 | Frames shall include message type, payload length, sequence information, and integrity protection. |
| COM-003 | Invalid or corrupt frames shall be rejected without changing the active control state. |
| COM-004 | Stale commands and lost host communication shall be detected by watchdog logic. |
| EST-001 | The embedded estimator shall perform state prediction using motion and IMU information. |
| EST-002 | The estimator shall support wheel-speed and GNSS correction steps. |
| EST-003 | The estimator shall support nonlinear range-bearing landmark corrections. |
| EST-004 | Landmark innovations shall be checked before a measurement is accepted. |
| EST-005 | State and covariance health shall be monitored for invalid numerical values. |
| NAV-001 | The system shall distinguish GNSS-aided, landmark-aided, dead-reckoning, and unavailable navigation modes. |
| GNC-001 | The route manager shall support ordered waypoints and final-stop behavior. |
| GNC-002 | Guidance shall generate references without accessing simulator ground truth. |
| GNC-003 | Control outputs shall respect configured speed, steering, and rate limits. |
| SAF-001 | The firmware shall implement explicit startup, ready, running, degraded, safe-stop, and fault states. |
| SAF-002 | Communication timeout, invalid data, queue overflow, and manual stop shall produce defined responses. |
| LOG-001 | Every run shall record its scenario, seed, software versions, configuration, and event history. |
| LOG-002 | Recorded runs shall be replayable without connected hardware. |
| HMI-001 | The OLED, LEDs, buzzer, buttons, and servo indicator shall report system status without blocking the control loop. |

## 6. Quality Requirements

- Simulator ground truth shall never be used by embedded estimation, guidance, or control.
- Firmware shall avoid heap allocation during normal runtime.
- Long-running firmware tasks shall not use blocking delays.
- Protocol behavior shall be verified using shared Python and C++ test vectors.
- Hardware-independent firmware logic shall be testable on the host.
- Experiment outputs shall identify the exact scenario and software configuration.
- Failures shall be reported explicitly rather than replaced with silent fallback behavior.
- Repository claims shall be supported by measured results.

## 7. v1.0 Acceptance Targets

The v1.0 release shall satisfy all of the following:

- Identical scenario and seed inputs reproduce identical host-side results on the pinned toolchain.
- A 1,000-frame nominal protocol test completes without an undetected corrupt frame.
- Injected corrupt frames are rejected and counted.
- Estimator state remains finite and covariance remains numerically healthy in accepted scenarios.
- Across the final 20-seed GNSS-denied campaign, landmark aiding achieves lower position RMSE than the dead-reckoning baseline.
- The vehicle completes the reference route and reaches the defined final-stop region.
- Nominal command round-trip latency remains below the configured control period at the 99th percentile.
- Host disconnect and stale-input faults lead to a documented safe state.
- No tested fault causes uncontrolled stale command application.
- Firmware flash and SRAM usage remain within the Arduino UNO R4 WiFi limits with recorded headroom.
- All required unit, integration, replay, and fault tests pass.
- Final reports can be regenerated from versioned configuration and aggregated result files.

Exact scenario geometry, sensor parameters, control period, and metric thresholds are versioned in the validated scenario and campaign configuration. NavBench v1.0.0 evaluates the simulated vehicle through embedded controller hardware; it is not a physical autonomous-driving product or a safety-certification platform.
