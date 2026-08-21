# Scenario Format v1

Scenarios are strict YAML mappings loaded by `host/src/navbench/scenario.py`.
Unknown fields, non-finite numbers, invalid ranges, unaligned time boundaries,
and unsupported fault combinations are rejected before execution.

## Minimal example

```yaml
version: 1
name: straight_example
seed: 1001
duration_s: 8.0
dt_s: 0.02

vehicle:
  wheelbase_m: 0.32
  steering_time_constant_s: 0.18
  acceleration_time_constant_s: 0.25
  max_steering_deg: 30.0
  max_acceleration_mps2: 2.0
  max_deceleration_mps2: 4.0
  max_speed_mps: 5.0

initial_state:
  x_m: 0.0
  y_m: 0.0
  heading_deg: 0.0
  speed_mps: 0.0

commands:
  - {until_s: 4.0, acceleration_mps2: 1.0, steering_deg: 0.0}
  - {until_s: 8.0, acceleration_mps2: -1.0, steering_deg: 0.0}

route:
  - {x_m: 4.0, y_m: 0.0, target_speed_mps: 1.5}
  - {x_m: 8.0, y_m: 0.0, target_speed_mps: 0.0, acceptance_radius_m: 0.5}

landmarks:
  - {landmark_id: 1, x_m: 3.0, y_m: 2.0}

faults: []
```

`commands` is required and drives open-loop simulation. Controller-in-the-loop
execution ignores those open-loop commands and applies `CONTROL_COMMAND`
messages from the C++ controller. `route` is required only for CIL execution.

## Top-level fields

| Field | Required | Meaning |
|---|---:|---|
| `version` | no | Schema version; default and only supported value is `1` |
| `name` | yes | Safe 1-64 character run name |
| `seed` | no | Explicit unsigned 64-bit random seed; default `0` |
| `duration_s`, `dt_s` | yes | Positive duration and fixed plant step |
| `vehicle` | no | Plant geometry, actuator constants, and limits |
| `initial_state` | no | Initial global pose and actuator state |
| `commands` | yes | Piecewise-constant open-loop command schedule |
| `sensors` | no | Per-sensor enable/rate/error/latency configuration |
| `landmarks` | no | Unique global landmark map entries |
| `faults` | no | Step-aligned injected fault windows |
| `route` | no | Ordered controller waypoints |

All omitted nested values use the typed defaults in `simulator.py` and
`sensors.py`. Angles in scenario YAML are degrees; internal and wire angles are
radians.

## Timing policy

- `duration_s / dt_s` must be an exact decimal integer. A duration of `N * dt`
  yields states `0..N`, or `N + 1` samples.
- Command `until_s` values strictly increase, align to `dt_s`, and the final
  command ends exactly at `duration_s`.
- Fault start/end values align to `dt_s`, use `[start_s, end_s)`, and remain
  inside the scenario.
- Each enabled sensor sample period must be an integer number of plant steps.
- Sensor latency is converted to a deterministic delivery step without changing
  the sample step or timestamp.
- Firmware independently bounds sample lag using `sample_step_id` versus the
  enclosing sensor-frame step; a configured latency spike can therefore produce
  an explicit stale-sample rejection instead of refreshing estimator health.

## Vehicle and route

`vehicle` accepts:

```text
wheelbase_m
steering_time_constant_s
acceleration_time_constant_s
max_steering_deg
max_acceleration_mps2
max_deceleration_mps2
max_speed_mps
```

`initial_state` accepts `x_m`, `y_m`, `heading_deg`, `speed_mps`,
`steering_deg`, and `acceleration_mps2`; initial actuator state must already be
inside vehicle limits.

Each route entry accepts `x_m`, `y_m`, `target_speed_mps`, and optional
`acceptance_radius_m` (default `0.35`). Target speed cannot exceed the vehicle
limit. Firmware accepts at most 32 waypoints and does not accept loop routes.

## Sensors

Every sensor supports `enabled`, `rate_hz`, `latency_s`, and
`dropout_probability`.

| Sensor | Additional fields |
|---|---|
| `imu` | `acceleration_noise_std_mps2`, `yaw_rate_noise_std_deg_s`, `acceleration_bias_mps2`, `yaw_rate_bias_deg_s` |
| `wheel_speed` | `noise_std_mps`, `scale_error` |
| `gnss` | `position_noise_std_m` |
| `landmark` | `range_noise_std_m`, `bearing_noise_std_deg`, `max_range_m`, `field_of_view_deg`, `outlier_probability`, `outlier_range_std_m`, `outlier_bearing_std_deg` |

Each sensor has an independent stream derived from the scenario seed, so
enabling or changing one sensor does not consume randomness from another. The
same validated scenario, seed, and toolchain reproduce logical plant and sensor
outputs. UTC manifest fields and measured process timing are not deterministic.

## Faults

Each entry contains `target`, `kind`, `start_s`, `end_s`, and an optional numeric
`value`.

| Target | Supported kinds |
|---|---|
| `imu` | `dropout`, `bias_acceleration_mps2`, `bias_yaw_rate_rad_s`, `latency_spike` |
| `wheel_speed` | `dropout`, `scale_error`, `latency_spike` |
| `gnss` | `dropout`, `latency_spike` |
| `landmark` | `dropout`, `outlier`, `latency_spike` |
| `transport` | `packet_corruption`, `packet_loss`, `latency_spike`, `stale_frame` |
| `host` | `disconnect` |
| `runtime` | `manual_safe_stop` |

The sensor suite uses faults targeting sensors; CIL additionally maps transport,
host, and runtime faults into the link/session path. Transport processing waits
for complete COBS-delimited frames and applies each configured fault in both
host-to-controller and controller-to-host directions, with separate counters.
`value` is the additive bias, scale-error increment, or extra latency in seconds,
depending on kind. If return-path faults leave no fresh controller command, the
host plant uses a bounded safe-stop command rather than reusing stale output.

## Included scenarios and commands

The repository includes straight, constant-turn, S-curve, saturation, stop,
sensor-fault, and communication/safety fault cases in `scenarios/`.

```sh
PYTHONPATH=host/src uv run --project host --locked python -m navbench open-loop \
  --scenario scenarios/open_loop_s_curve.yaml --output-root runs

PYTHONPATH=host/src uv run --project host --locked python -m navbench cil \
  --scenario scenarios/straight.yaml \
  --native build/native/navbench_native_firmware --output-root runs

PYTHONPATH=host/src uv run --project host --locked python -m navbench campaign \
  --scenario scenarios/s_curve.yaml \
  --native build/native/navbench_native_firmware \
  --output-root runs --name campaign_smoke --seeds 1001,1002

PYTHONPATH=host/src uv run --project host --locked python -m navbench \
  inspect-campaign runs/campaign_smoke
```

Open-loop, CIL, and campaign runs refuse an existing run directory and retain an
`INCOMPLETE` marker after interruption. Raw `runs/` artifacts are ignored by
Git. Complete replay checks the config hash, exact stream counts, typed command
fields, and (for native replay) the recorded controller-binary hash.
