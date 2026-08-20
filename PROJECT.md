# NavBench

## What Is NavBench?

NavBench is an embedded controller-in-the-loop testbed for GNSS-denied ground-vehicle navigation and control.

A deterministic vehicle simulation runs on a host computer, while an Arduino UNO R4 WiFi executes the embedded estimation, guidance, control, and safety logic. The two sides communicate over a versioned USB serial protocol.

## How It Works

The host computer simulates:

- Vehicle motion
- IMU measurements
- Wheel-speed measurements
- GNSS measurements
- Range-bearing landmark observations
- Sensor noise, bias, latency, and dropout

The Arduino processes these measurements and returns control commands. The host applies the commands to the simulated vehicle, records ground truth, and produces logs and visualizations.

Ground truth is used only for evaluation and is not provided to the embedded navigation or control algorithms.

## v1.0 Goals

- Build a deterministic and repeatable vehicle simulator.
- Design a reliable binary serial protocol.
- Implement an Extended Kalman Filter on the Arduino.
- Support GNSS-aided and GNSS-denied navigation.
- Add nonlinear landmark-based localization.
- Implement waypoint guidance and path-following control.
- Detect communication, sensor, and runtime faults.
- Record and replay complete experiments.
- Measure estimation error, tracking error, latency, and embedded resource usage.

## Hardware

The embedded system is based on an Arduino UNO R4 WiFi. An OLED display, buttons, LEDs, a buzzer, and a servo indicator provide basic physical interaction and status feedback.

## Scope

NavBench v1.0 is a simulation-based embedded testbed. It is not intended to control a physical autonomous vehicle, provide automotive-grade HIL capability, or serve as a production safety system.

## Current Status

Milestone 0 — Development environment and repository foundation.

No navigation, estimation, communication, or control capability has been implemented yet.
