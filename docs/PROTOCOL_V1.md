# NavBench Protocol v1

Protocol v1 is the binary boundary between the macOS host and the embedded
controller. It defines framing and typed messages only. Serial-port ownership,
handshake retries, timeouts, and reconnect policy belong to the session layer.

## Wire frame

Every multi-byte integer and every IEEE-754 binary32 value is little-endian.
One raw packet has this layout:

| Offset | Field | Type | Meaning |
|---:|---|---|---|
| 0 | `version` | `u8` | Exactly `1` |
| 1 | `message_type` | `u8` | One value from the message table |
| 2 | `payload_length` | `u16` | `0..128` |
| 4 | `sequence` | `u32` | Per-sender session sequence |
| 8 | `step_id` | `u32` | Simulation/control step associated with the message |
| 12 | `payload` | bytes | Typed payload, `payload_length` bytes |
| `12+n` | `crc16` | `u16` | CRC over header and payload |

CRC is CRC-16/CCITT-FALSE: polynomial `0x1021`, initial value `0xffff`, no
reflection, and xorout `0x0000`. The check value for ASCII `123456789` is
`0x29b1`.

The raw packet is COBS encoded and followed by one `0x00` delimiter. A raw
packet is at most 142 bytes; the encoded portion is at most 143 bytes; the
delimiter-inclusive wire frame is at most 144 bytes. A packet decoder accepts
an encoded frame with or without its single trailing delimiter. Embedded or
interior zero delimiters, malformed COBS blocks, unknown types or versions,
length mismatches, oversized payloads, and CRC mismatches are rejected.

Packet decoding validates framing but does not infer an application payload
layout. The matching typed-payload decoder must be called before a payload is
used. Reserved fields must be transmitted as zero. All received floats must be
finite; state-estimate covariance diagonal entries must also be non-negative.

## Message types

| ID | Name | Direction | Payload |
|---:|---|---|---|
| `0x01` | `HELLO` | either | 12-byte hello |
| `0x02` | `HELLO_ACK` | either | 12-byte negotiation result |
| `0x10` | `SENSOR_FRAME` | host to controller | 76-byte sensor record |
| `0x11` | `ROUTE_CHUNK` | host to controller | bounded waypoint chunk |
| `0x20` | `CONTROL_COMMAND` | controller to host | 16-byte actuator command |
| `0x21` | `STATE_ESTIMATE` | controller to host | 52-byte estimator telemetry |
| `0x22` | `HEALTH_STATUS` | controller to host | 116-byte health and NIS telemetry |
| `0x23` | `HEARTBEAT` | either | 12-byte liveness record |
| `0x7e` | `ERROR` | either | 12-byte structured error |
| `0x7f` | `SAFE_STOP` | either | 4-byte safe-stop request/status |

Direction states intended ownership. `HELLO`, `HELLO_ACK`, `HEARTBEAT`,
`ERROR`, and `SAFE_STOP` can be emitted by either endpoint where session policy
permits it.

## Negotiation payloads

`HELLO`:

| Offset | Field | Type |
|---:|---|---|
| 0 | `role` | `u8` |
| 1 | `min_version` | `u8` |
| 2 | `max_version` | `u8` |
| 3 | reserved | `u8` |
| 4 | `capabilities` | `u32` bit mask |
| 8 | `max_payload` | `u16` |
| 10 | `heartbeat_timeout_ms` | `u16`, ms |

`role` is host `1` or controller `2`. The version range must be nonempty and
start above zero. `max_payload` is `1..128`, and the timeout is nonzero.

`HELLO_ACK` has the same size:

| Offset | Field | Type |
|---:|---|---|
| 0 | `accepted_version` | `u8` |
| 1 | `status` | `u8` |
| 2 | `role` | `u8` |
| 3 | reserved | `u8` |
| 4 | `capabilities` | `u32` bit mask |
| 8 | `max_payload` | `u16` |
| 10 | `heartbeat_timeout_ms` | `u16`, ms |

Status values are OK `0`, version mismatch `1`, busy `2`, and rejected `3`.
An OK response carries accepted version `1`; any rejection carries accepted
version `0`. Capability bits are sensor frame `0`, route chunk `1`, state
estimate `2`, health status `3`, and safe stop `4`.

NavBench v1 firmware requires all five capabilities, a host receive ceiling of
at least 116 bytes, and the configured 500 ms watchdog timeout. It rejects an
incompatible HELLO and does not accept route, sensor, or heartbeat input before
a successful negotiation. A pre-session SAFE_STOP remains admissible.

## Sensor frame

`SENSOR_FRAME` is a fixed 76-byte aggregate. It never carries plant ground
truth. Each measurement records its source sampling step and timestamp, so a
delayed delivery does not alter measurement time.

| Offset | Field | Type/unit |
|---:|---|---|
| 0 | `present_mask` | `u16` |
| 2 | `fault_mask` | `u16` |
| 4 | IMU `sample_step_id` | `u32` |
| 8 | IMU `timestamp_us` | `u32`, us |
| 12 | IMU longitudinal acceleration | `f32`, m/s² |
| 16 | IMU yaw rate | `f32`, rad/s |
| 20 | wheel `sample_step_id` | `u32` |
| 24 | wheel `timestamp_us` | `u32`, us |
| 28 | wheel speed | `f32`, m/s |
| 32 | GNSS `sample_step_id` | `u32` |
| 36 | GNSS `timestamp_us` | `u32`, us |
| 40 | GNSS x | `f32`, m, global frame |
| 44 | GNSS y | `f32`, m, global frame |
| 48 | landmark `sample_step_id` | `u32` |
| 52 | landmark `timestamp_us` | `u32`, us |
| 56 | `landmark_id` | `u16` |
| 58 | reserved | `u16` |
| 60 | landmark global x | `f32`, m |
| 64 | landmark global y | `f32`, m |
| 68 | landmark range | `f32`, m |
| 72 | landmark bearing | `f32`, rad, body-frame signed angle |

Mask bits are IMU `0`, wheel speed `1`, GNSS `2`, and landmark `3`.
`present_mask` is authoritative; fields for an absent sample are ignored and
should be zeroed by senders. `fault_mask` marks a known injected or detected
quality fault and may be set even when dropout clears the corresponding present
bit. One frame contains at most one landmark observation. Multiple landmark
observations use multiple packets with the same outer `step_id` and distinct
sequences. Landmark global coordinates are map/reference data; they do not
contain vehicle ground truth.

## Route chunk

The payload starts with an 8-byte prefix followed by zero to five fixed
waypoint records:

| Offset | Field | Type |
|---:|---|---|
| 0 | `route_id` | `u16` |
| 2 | `start_index` | `u16` |
| 4 | `total_count` | `u16` |
| 6 | `point_count` | `u8`, `0..5` |
| 7 | `flags` | `u8` |
| 8 | waypoint records | `point_count * 16` bytes |

Each waypoint is x metres, y metres, target speed m/s, and waypoint acceptance
radius metres as four `f32` values. Pure Pursuit lookahead is a controller
configuration value and is not encoded per waypoint. Flag bits are clear existing route `0`, final chunk `1`, and
loop route `2`. A nonempty chunk must satisfy
`start_index + point_count <= total_count`. An empty chunk is valid only with
the clear flag. Route assembly, duplicate chunks, capacity, and atomic route
activation are session/runtime responsibilities.

## Controller output and telemetry

`CONTROL_COMMAND`:

| Offset | Field | Type/unit |
|---:|---|---|
| 0 | steering | `f32`, rad |
| 4 | acceleration | `f32`, m/s² |
| 8 | target speed | `f32`, m/s |
| 12 | mode | `u8` |
| 13 | flags | `u8` |
| 14 | reserved | `u16` |

Modes are neutral `0`, tracking `1`, and safe stop `2`.

`STATE_ESTIMATE` exposes the six estimator states followed by its covariance
diagonal:

| Offset | Field | Type/unit |
|---:|---|---|
| 0 | x | `f32`, m |
| 4 | y | `f32`, m |
| 8 | heading | `f32`, rad |
| 12 | speed | `f32`, m/s |
| 16 | yaw rate | `f32`, rad/s |
| 20 | longitudinal acceleration bias | `f32`, m/s² |
| 24 | covariance diagonal | six `f32`, state units squared |
| 48 | navigation mode | `u8` |
| 49 | flags | `u8` |
| 50 | reserved | `u16` |

Navigation modes are unavailable `0`, dead reckoning `1`, landmark aided `2`,
GNSS aided `3`, and degraded `4`.

`HEALTH_STATUS` starts with runtime state `u8`, navigation mode `u8`, and flags
`u16`. Twelve `u32` values follow at offsets 4 through 48: uptime ms, last
sensor age ms, received frames, CRC errors, decode errors, missing sequences,
duplicates, out-of-order packets, stale packets, queue overflows, scheduler
overruns, and maximum loop time us. Bytes 52 through 115 contain four
cumulative 16-byte NIS summary groups:

| Group offset | Update |
|---:|---|
| 52 | IMU yaw-rate correction |
| 68 | wheel-speed correction |
| 84 | GNSS position correction |
| 100 | landmark range-bearing correction |

Every group has the same relative layout:

| Relative offset | Field | Type |
|---:|---|---|
| 0 | `evaluated_count` | `u32` |
| 4 | `gate_rejected_count` | `u32` |
| 8 | `nis_sum` | finite non-negative `f32` |
| 12 | `nis_max` | finite non-negative `f32` |

`evaluated_count` includes exactly accepted and innovation-gate-rejected
updates. Invalid, not-initialized, and numerical-failure attempts are excluded.
`nis_sum / evaluated_count` is therefore the cumulative mean; periodic health
frames repeat a snapshot and must not themselves be averaged as independent NIS
samples. `gate_rejected_count` cannot exceed `evaluated_count`, and `nis_max`
cannot exceed `nis_sum`. A group with no evaluated samples is all zero.

All four accumulators reset when the EKF resets or initializes. If a `u32`
evaluation count would wrap, or adding a finite NIS would make the `f32` sum
non-finite, that sensor's group atomically starts a new epoch containing the
current sample. The resulting count decrease is observable to the host. This
keeps every transmitted count, sum, and maximum internally consistent and
finite. Runtime states are startup `0`, self-test `1`, ready `2`, running `3`,
degraded `4`, safe stop `5`, and fault `6`.

## Liveness, errors, and safe stop

`HEARTBEAT` contains uptime ms `u32`, monotonic ms `u32`, runtime state `u8`,
and three reserved zero bytes.

`ERROR` contains application error code `u16`, detail `u16`, related sequence
`u32`, and context `u32`. Defined codes are none `0`, bad payload `1`,
unsupported message `2`, route rejected `3`, not ready `4`, and internal fault
`5`. Detail and context are code-specific numeric values; they are not strings.

`SAFE_STOP` contains reason `u8`, latch `u8` restricted to `0` or `1`, and
detail `u16`. Reasons are none `0`, manual `1`, host timeout `2`, stale sensor
`3`, protocol error `4`, and internal fault `5`. Clearing a latched stop is a
safety-state-machine decision, not an implicit consequence of receiving a
reason of none.

## Stream and ordering policy

The incremental parsers accept arbitrary fragmented or combined byte reads.
They retain at most 143 encoded bytes. Once that bound is exceeded they report
one oversized-frame error, discard through the next delimiter, and then resume
synchronization. An explicit end-of-stream operation reports a pending frame as
truncated. Parsers count accepted packets and classified COBS, CRC, version,
type, length, oversize, and truncation failures.

Sequence comparison uses modulo-2³² arithmetic and resets after a successful
new session handshake. The supplied tracker implements this policy:

- the first sequence is accepted;
- a delta of one is in order;
- a forward delta below 2³¹ is accepted, with missing packets counted;
- an exact repeat is rejected as duplicate;
- a behind sequence within the configured reorder window is rejected as
  out-of-order;
- an older sequence outside that window is rejected as stale;
- an otherwise forward packet carrying an older `step_id` is rejected as stale.

Only accepted packets advance the tracker. Message-specific handling may apply
stricter freshness constraints. Callers apply the optional `step_id` comparison
only to step-bound data; handshake, heartbeat, health, error, and safe-stop
messages use sequence-only observation so their conventional step value of zero
cannot make a safety message stale. Corrupt, duplicate, out-of-order, stale, or
malformed input must not mutate estimator, controller, route, or safety state.

## Implementations and conformance

The Python codec is in `host/src/navbench/protocol.py`. The fixed-buffer C++ API
is in `include/navbench/protocol.hpp` and `src/protocol.cpp`; it performs no heap
allocation and uses no exceptions or dynamic containers. Golden packet vectors
shared by Python and C++ are stored in
`test/fixtures/protocol_v1_golden.tsv`. They cover empty and ordinary payloads,
embedded zero bytes, every typed message, and the 128-byte maximum payload.
`test/fixtures/protocol_v1_invalid.tsv` holds shared rejection vectors for
empty, malformed COBS, interior delimiter, short, corrupt CRC, unsupported
version/type, mismatched length, oversized payload declaration, and oversized
frame cases.

Protocol v1 provides integrity against accidental corruption, not
authentication, confidentiality, or protection from a malicious peer.
