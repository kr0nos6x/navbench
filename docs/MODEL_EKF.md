# Plant, Estimator, Guidance, and Control

## Frames, units, and state

The global map frame uses metres. Heading and bearing are radians and are
normalized to `[-pi, pi)`. Speed is metres per second, longitudinal acceleration
is metres per second squared, and yaw rate is radians per second.

The embedded EKF state is

```text
x = [p_x, p_y, psi, v, omega, b_a]^T
```

where `p_x,p_y` are global position, `psi` is heading, `v` is forward speed,
`omega` is yaw rate, and `b_a` is longitudinal accelerometer bias. Six states
retain the quantities required by guidance while keeping the firmware matrices
fixed and small.

## Host plant

The host uses a kinematic bicycle plant. For command `a_c, delta_c`, limits are
applied first. Actuator states use an exact discretization of first-order
responses:

```text
alpha_a     = 1 - exp(-dt / tau_a)
a[k+1]      = a[k]     + alpha_a     (sat(a_c)     - a[k])
alpha_delta = 1 - exp(-dt / tau_delta)
delta[k+1]  = delta[k] + alpha_delta (sat(delta_c) - delta[k])
v[k+1]      = sat(v[k] + a[k+1] dt)
omega_p     = v[k+1] tan(delta[k+1]) / wheelbase
psi[k+1]    = wrap(psi[k] + omega_p dt)
```

Position uses the midpoint heading `psi[k] + omega_p dt / 2`. Speed is bounded
to `[0, max_speed]`; steering, acceleration, and deceleration use scenario
limits.

## EKF prediction

Given IMU longitudinal acceleration `a_m`, define

```text
a = a_m - b_a
d = v dt + 0.5 a dt^2
```

The prediction is

```text
p_x' = p_x + d cos(psi)
p_y' = p_y + d sin(psi)
psi' = wrap(psi + omega dt)
v'   = v + a dt
omega' = omega
b_a'   = b_a
P' = F P F^T + diag(q dt)
```

`F` is identity with these non-zero off-diagonal terms:

```text
F[x,psi] = -d sin(psi)       F[x,v] = dt cos(psi)
F[x,b_a] = -0.5 dt^2 cos(psi)
F[y,psi] =  d cos(psi)       F[y,v] = dt sin(psi)
F[y,b_a] = -0.5 dt^2 sin(psi)
F[psi,omega] = dt            F[v,b_a] = -dt
```

The IMU yaw-rate value is then a scalar correction of `omega`. The estimator is
initialized on the first valid GNSS frame: GNSS supplies position, wheel speed
supplies initial speed when present, and IMU supplies initial yaw rate when
present. Initial heading and acceleration bias are zero unless a caller invokes
the lower-level explicit initializer.

## Measurement updates

Wheel speed observes `v`; GNSS observes `[p_x,p_y]`. A landmark at global
position `[l_x,l_y]` predicts

```text
dx = l_x - p_x
dy = l_y - p_y
r_hat = sqrt(dx^2 + dy^2)
beta_hat = wrap(atan2(dy, dx) - psi)
```

The landmark Jacobian rows are

```text
H_r    = [-dx/r, -dy/r,  0, 0, 0, 0]
H_beta = [ dy/r^2, -dx/r^2, -1, 0, 0, 0]
```

Prediction and landmark analytic Jacobians are checked against finite
differences in the Python reference tests. A deterministic shared fixture also
compares the Python float64 reference with the C++ float32 implementation.

## Gating and covariance safety

For innovation `nu` and innovation covariance `S`, every correction computes

```text
NIS = nu^T S^-1 nu
```

The configurable per-sensor NIS threshold rejects an outlier before the state
update. Accepted updates use the Joseph form

```text
P = (I - K H) P (I - K H)^T + K R K^T
```

and explicitly restore covariance symmetry. State/covariance finiteness,
positive covariance diagonals, configured floors/ceilings, measurement ranges,
and maximum prediction interval are checked. A numerical failure moves runtime
safety to FAULT; a gated measurement is counted without being treated as a
numerical fault.

Since estimator reset, each sensor family keeps cumulative NIS
`evaluated_count`, `gate_rejected_count`, `sum`, and `max` values, with a
defensive accumulator reset on counter/sum rollover. Evaluated count and sum
include both accepted innovations and NIS-gated rejections; invalid measurements
and numerical failures do not contribute. Periodic health frames are snapshots
of these accumulators, not independent samples. Run metrics use the latest
health snapshot, sum counts/sums across the four sensor families, take their
maximum, and compute the weighted mean as total sum divided by total evaluated
count.

`Q`, all measurement variances, NIS gates, numerical bounds, and freshness
timeouts live in `EkfConfig`. The navigation mode is selected from accepted,
fresh observations in this priority order:

1. `GNSS_AIDED`
2. `LANDMARK_AIDED`
3. `DEAD_RECKONING` when IMU and wheel speed are fresh
4. `DEGRADED` when only one dead-reckoning source is fresh
5. `UNAVAILABLE`

## Route guidance and speed control

The fixed-capacity route manager accepts up to 32 ordered waypoints. Each point
has global `x,y`, target speed, and an optional acceptance radius. Progress
advances only when the active waypoint enters its own acceptance region. After
the first point, the lookahead target is projected from the active route segment
and walked forward across subsequent polyline segments, bounded by the final
waypoint.

Pure Pursuit computes

```text
delta = atan2(2 wheelbase sin(heading_error), target_distance)
```

then applies steering magnitude and steering-rate limits. Longitudinal control
is PI with integral clamping and conditional integration anti-windup, followed
by acceleration/deceleration saturation. The final waypoint applies a braking
speed bound and slowdown ramp, latches zero target speed inside its acceptance
region, and completes only when position and speed tolerances are both met.
Guidance and control use only the EKF estimate and route reference.

## Known estimator limitation

Sensor latency is represented correctly at the host and on the wire: every
measurement carries its original sample step and timestamp. Firmware first
checks sample age in the step domain against the enclosing frame step and rejects
future/excessively old samples. For accepted input, EKF navigation freshness is
measured from controller receive time, avoiding comparison between the source
clock/32-bit wire timestamp and board `millis()`.

The v1 EKF does not keep a state history or rewind/repropagate for an
out-of-sequence measurement. An accepted delayed GNSS or landmark observation is
therefore applied to the current state. Scenario latency must remain within the
configured step-age envelope, and hardware latency requires separate
measurement.
