"""NavBench Protocol v1 wire codec.

The packet layer is deliberately independent from serial I/O and session policy.
All integers and IEEE-754 binary32 values are little-endian on the wire.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from enum import IntEnum, IntFlag
from typing import TypeAlias


PROTOCOL_VERSION = 1
MAX_PAYLOAD_SIZE = 128

_HEADER = struct.Struct("<BBHII")
_CRC = struct.Struct("<H")

HEADER_SIZE = _HEADER.size
CRC_SIZE = _CRC.size
MAX_RAW_PACKET_SIZE = HEADER_SIZE + MAX_PAYLOAD_SIZE + CRC_SIZE
# MAX_RAW_PACKET_SIZE is below 254, so COBS adds exactly one code byte.
MAX_ENCODED_FRAME_SIZE = MAX_RAW_PACKET_SIZE + 1
MAX_WIRE_FRAME_SIZE = MAX_ENCODED_FRAME_SIZE + 1
_FLOAT32_MAX = 3.4028234663852886e38


class MessageType(IntEnum):
    HELLO = 0x01
    HELLO_ACK = 0x02
    SENSOR_FRAME = 0x10
    ROUTE_CHUNK = 0x11
    ROUTE_REFERENCE = 0x11  # Descriptive alias retained for callers.
    CONTROL_COMMAND = 0x20
    STATE_ESTIMATE = 0x21
    HEALTH_STATUS = 0x22
    HEARTBEAT = 0x23
    ERROR = 0x7E
    SAFE_STOP = 0x7F


class EndpointRole(IntEnum):
    HOST = 1
    CONTROLLER = 2


class HelloStatus(IntEnum):
    OK = 0
    VERSION_MISMATCH = 1
    BUSY = 2
    REJECTED = 3


class Capability(IntFlag):
    SENSOR_FRAME = 1 << 0
    ROUTE_CHUNK = 1 << 1
    STATE_ESTIMATE = 1 << 2
    HEALTH_STATUS = 1 << 3
    SAFE_STOP = 1 << 4


class SensorMask(IntFlag):
    IMU = 1 << 0
    WHEEL_SPEED = 1 << 1
    GNSS = 1 << 2
    LANDMARK = 1 << 3


class RouteFlag(IntFlag):
    CLEAR_EXISTING = 1 << 0
    FINAL_CHUNK = 1 << 1
    LOOP_ROUTE = 1 << 2


class ControlMode(IntEnum):
    NEUTRAL = 0
    TRACKING = 1
    SAFE_STOP = 2


class RuntimeState(IntEnum):
    STARTUP = 0
    SELF_TEST = 1
    READY = 2
    RUNNING = 3
    DEGRADED = 4
    SAFE_STOP = 5
    FAULT = 6


class NavigationMode(IntEnum):
    UNAVAILABLE = 0
    DEAD_RECKONING = 1
    LANDMARK_AIDED = 2
    GNSS_AIDED = 3
    DEGRADED = 4


class SafeStopReason(IntEnum):
    NONE = 0
    MANUAL = 1
    HOST_TIMEOUT = 2
    STALE_SENSOR = 3
    PROTOCOL_ERROR = 4
    INTERNAL_FAULT = 5


class ApplicationErrorCode(IntEnum):
    NONE = 0
    BAD_PAYLOAD = 1
    UNSUPPORTED_MESSAGE = 2
    ROUTE_REJECTED = 3
    NOT_READY = 4
    INTERNAL_FAULT = 5
    DIAGNOSTIC = 6


class ProtocolErrorCode(IntEnum):
    EMPTY_FRAME = 1
    UNEXPECTED_DELIMITER = 2
    COBS_MALFORMED = 3
    PACKET_TOO_SHORT = 4
    CRC_MISMATCH = 5
    UNSUPPORTED_VERSION = 6
    UNKNOWN_MESSAGE_TYPE = 7
    PAYLOAD_LENGTH_MISMATCH = 8
    PAYLOAD_TOO_LARGE = 9
    INVALID_VALUE = 10
    OVERSIZED_FRAME = 11
    TRUNCATED_FRAME = 12


class ProtocolError(ValueError):
    """A classified packet, payload, or stream decoding failure."""

    def __init__(self, code: ProtocolErrorCode, message: str) -> None:
        super().__init__(message)
        self.code = code


@dataclass(frozen=True, slots=True)
class Packet:
    message_type: MessageType
    sequence: int
    step_id: int
    payload: bytes = b""


@dataclass(frozen=True, slots=True)
class HelloPayload:
    role: EndpointRole
    min_version: int = PROTOCOL_VERSION
    max_version: int = PROTOCOL_VERSION
    capabilities: int = 0
    max_payload: int = MAX_PAYLOAD_SIZE
    heartbeat_timeout_ms: int = 500


@dataclass(frozen=True, slots=True)
class HelloAckPayload:
    accepted_version: int
    status: HelloStatus
    role: EndpointRole
    capabilities: int = 0
    max_payload: int = MAX_PAYLOAD_SIZE
    heartbeat_timeout_ms: int = 500


@dataclass(frozen=True, slots=True)
class ImuSample:
    sample_step_id: int = 0
    timestamp_us: int = 0
    longitudinal_acceleration_mps2: float = 0.0
    yaw_rate_rps: float = 0.0


@dataclass(frozen=True, slots=True)
class WheelSpeedSample:
    sample_step_id: int = 0
    timestamp_us: int = 0
    speed_mps: float = 0.0


@dataclass(frozen=True, slots=True)
class GnssSample:
    sample_step_id: int = 0
    timestamp_us: int = 0
    x_m: float = 0.0
    y_m: float = 0.0


@dataclass(frozen=True, slots=True)
class LandmarkSample:
    sample_step_id: int = 0
    timestamp_us: int = 0
    landmark_id: int = 0
    landmark_x_m: float = 0.0
    landmark_y_m: float = 0.0
    range_m: float = 0.0
    bearing_rad: float = 0.0


@dataclass(frozen=True, slots=True)
class SensorFramePayload:
    present_mask: SensorMask = SensorMask(0)
    fault_mask: SensorMask = SensorMask(0)
    imu: ImuSample = ImuSample()
    wheel_speed: WheelSpeedSample = WheelSpeedSample()
    gnss: GnssSample = GnssSample()
    landmark: LandmarkSample = LandmarkSample()


@dataclass(frozen=True, slots=True)
class RouteWaypoint:
    x_m: float
    y_m: float
    target_speed_mps: float
    acceptance_radius_m: float


MAX_ROUTE_POINTS_PER_CHUNK = 5


@dataclass(frozen=True, slots=True)
class RouteChunkPayload:
    route_id: int
    start_index: int
    total_count: int
    flags: RouteFlag
    points: tuple[RouteWaypoint, ...]


@dataclass(frozen=True, slots=True)
class ControlCommandPayload:
    steering_rad: float
    acceleration_mps2: float
    target_speed_mps: float
    mode: ControlMode
    flags: int = 0


@dataclass(frozen=True, slots=True)
class StateEstimatePayload:
    x_m: float
    y_m: float
    heading_rad: float
    speed_mps: float
    yaw_rate_rps: float
    acceleration_bias_mps2: float
    covariance_diagonal: tuple[float, float, float, float, float, float]
    navigation_mode: NavigationMode
    flags: int = 0


@dataclass(frozen=True, slots=True)
class HealthStatusPayload:
    runtime_state: RuntimeState
    navigation_mode: NavigationMode
    flags: int
    uptime_ms: int
    last_sensor_age_ms: int
    rx_frames: int
    rx_crc_errors: int
    rx_decode_errors: int
    rx_missing: int
    rx_duplicates: int
    rx_out_of_order: int
    rx_stale: int
    queue_overflows: int
    scheduler_overruns: int
    max_loop_us: int
    imu_yaw_nis_evaluated_count: int
    imu_yaw_nis_gate_rejected_count: int
    imu_yaw_nis_sum: float
    imu_yaw_nis_max: float
    wheel_nis_evaluated_count: int
    wheel_nis_gate_rejected_count: int
    wheel_nis_sum: float
    wheel_nis_max: float
    gnss_nis_evaluated_count: int
    gnss_nis_gate_rejected_count: int
    gnss_nis_sum: float
    gnss_nis_max: float
    landmark_nis_evaluated_count: int
    landmark_nis_gate_rejected_count: int
    landmark_nis_sum: float
    landmark_nis_max: float


@dataclass(frozen=True, slots=True)
class HeartbeatPayload:
    uptime_ms: int
    monotonic_ms: int
    runtime_state: RuntimeState


@dataclass(frozen=True, slots=True)
class ErrorPayload:
    code: ApplicationErrorCode
    detail: int = 0
    related_sequence: int = 0
    context: int = 0


@dataclass(frozen=True, slots=True)
class SafeStopPayload:
    reason: SafeStopReason
    latch: bool = True
    detail: int = 0


TypedPayload: TypeAlias = (
    HelloPayload
    | HelloAckPayload
    | SensorFramePayload
    | RouteChunkPayload
    | ControlCommandPayload
    | StateEstimatePayload
    | HealthStatusPayload
    | HeartbeatPayload
    | ErrorPayload
    | SafeStopPayload
)


_HELLO = struct.Struct("<BBBBIHH")
_HELLO_ACK = struct.Struct("<BBBBIHH")
_SENSOR_PREFIX = struct.Struct("<HH")
_IMU_SAMPLE = struct.Struct("<IIff")
_WHEEL_SAMPLE = struct.Struct("<IIf")
_GNSS_SAMPLE = struct.Struct("<IIff")
_LANDMARK_SAMPLE = struct.Struct("<IIHHffff")
_ROUTE_PREFIX = struct.Struct("<HHHBB")
_ROUTE_POINT = struct.Struct("<ffff")
_CONTROL_COMMAND = struct.Struct("<fffBBH")
_STATE_ESTIMATE = struct.Struct("<12fBBH")
_HEALTH_STATUS = struct.Struct("<BBH12I" + "IIff" * 4)
_HEARTBEAT = struct.Struct("<IIB3x")
_ERROR_PAYLOAD = struct.Struct("<HHII")
_SAFE_STOP = struct.Struct("<BBH")


def _error(code: ProtocolErrorCode, message: str) -> ProtocolError:
    return ProtocolError(code, message)


def _uint(name: str, value: int, bits: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise _error(ProtocolErrorCode.INVALID_VALUE, f"{name} must be an integer")
    maximum = (1 << bits) - 1
    if not 0 <= value <= maximum:
        raise _error(
            ProtocolErrorCode.INVALID_VALUE,
            f"{name} must fit in uint{bits}",
        )
    return value


def _finite(name: str, value: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise _error(ProtocolErrorCode.INVALID_VALUE, f"{name} must be numeric")
    result = float(value)
    if not math.isfinite(result) or abs(result) > _FLOAT32_MAX:
        raise _error(
            ProtocolErrorCode.INVALID_VALUE,
            f"{name} must be finite and representable as binary32",
        )
    return result


def _enum_value(name: str, value: IntEnum, enum_type: type[IntEnum]) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise _error(
            ProtocolErrorCode.INVALID_VALUE,
            f"{name} is not a valid {enum_type.__name__}",
        )
    try:
        return int(enum_type(value))
    except (TypeError, ValueError) as exc:
        raise _error(
            ProtocolErrorCode.INVALID_VALUE,
            f"{name} is not a valid {enum_type.__name__}",
        ) from exc


def _mask(name: str, value: IntFlag, allowed: int) -> int:
    raw = _uint(name, value, 16)
    if raw & ~allowed:
        raise _error(ProtocolErrorCode.INVALID_VALUE, f"{name} has unknown bits")
    return raw


def _message_type(value: MessageType | int) -> MessageType:
    if isinstance(value, bool) or not isinstance(value, int):
        raise _error(
            ProtocolErrorCode.UNKNOWN_MESSAGE_TYPE,
            f"unknown message type: {value!r}",
        )
    try:
        return MessageType(value)
    except ValueError as exc:
        raise _error(
            ProtocolErrorCode.UNKNOWN_MESSAGE_TYPE,
            f"unknown message type: {value!r}",
        ) from exc


def _nis_summary_fields(
    name: str,
    evaluated_count: int,
    gate_rejected_count: int,
    nis_sum: float,
    nis_max: float,
) -> tuple[int, int, float, float]:
    evaluated = _uint(f"{name}_nis_evaluated_count", evaluated_count, 32)
    rejected = _uint(
        f"{name}_nis_gate_rejected_count", gate_rejected_count, 32
    )
    total = _finite(f"{name}_nis_sum", nis_sum)
    maximum = _finite(f"{name}_nis_max", nis_max)
    if rejected > evaluated:
        raise _error(
            ProtocolErrorCode.INVALID_VALUE,
            f"{name} NIS rejected count exceeds evaluated count",
        )
    if total < 0.0 or maximum < 0.0:
        raise _error(
            ProtocolErrorCode.INVALID_VALUE,
            f"{name} NIS summary must be non-negative",
        )
    if evaluated == 0:
        if rejected != 0 or total != 0.0 or maximum != 0.0:
            raise _error(
                ProtocolErrorCode.INVALID_VALUE,
                f"empty {name} NIS summary must be all zero",
            )
    elif maximum > total:
        raise _error(
            ProtocolErrorCode.INVALID_VALUE,
            f"{name} NIS maximum exceeds sum",
        )
    return evaluated, rejected, total, maximum


def encode_packet(packet: Packet) -> bytes:
    """Encode one packet, including its trailing zero delimiter."""

    message_type = _message_type(packet.message_type)

    if not isinstance(packet.payload, (bytes, bytearray, memoryview)):
        raise _error(ProtocolErrorCode.INVALID_VALUE, "payload must be bytes-like")
    payload = bytes(packet.payload)
    payload_size = len(payload)
    if payload_size > MAX_PAYLOAD_SIZE:
        raise _error(
            ProtocolErrorCode.PAYLOAD_TOO_LARGE,
            f"payload exceeds {MAX_PAYLOAD_SIZE} bytes",
        )

    sequence = _uint("sequence", packet.sequence, 32)
    step_id = _uint("step_id", packet.step_id, 32)
    header = _HEADER.pack(
        PROTOCOL_VERSION,
        int(message_type),
        payload_size,
        sequence,
        step_id,
    )
    packet_without_crc = header + payload
    raw_packet = packet_without_crc + _CRC.pack(crc16_ccitt(packet_without_crc))
    return cobs_encode(raw_packet) + b"\x00"


def decode_packet(frame: bytes | bytearray | memoryview) -> Packet:
    """Decode exactly one COBS frame, with or without one trailing delimiter."""

    if not isinstance(frame, (bytes, bytearray, memoryview)):
        raise _error(ProtocolErrorCode.INVALID_VALUE, "frame must be bytes-like")
    encoded = bytes(frame)
    if encoded.endswith(b"\x00"):
        encoded = encoded[:-1]
    if not encoded:
        raise _error(ProtocolErrorCode.EMPTY_FRAME, "frame is empty")
    if b"\x00" in encoded:
        raise _error(
            ProtocolErrorCode.UNEXPECTED_DELIMITER,
            "frame contains an unexpected delimiter",
        )
    if len(encoded) > MAX_ENCODED_FRAME_SIZE:
        raise _error(ProtocolErrorCode.OVERSIZED_FRAME, "encoded frame is oversized")

    raw_packet = cobs_decode(encoded)
    if len(raw_packet) < HEADER_SIZE + CRC_SIZE:
        raise _error(
            ProtocolErrorCode.PACKET_TOO_SHORT,
            "packet is shorter than the header and CRC",
        )
    if len(raw_packet) > MAX_RAW_PACKET_SIZE:
        raise _error(ProtocolErrorCode.PAYLOAD_TOO_LARGE, "decoded packet is oversized")

    packet_without_crc = raw_packet[:-CRC_SIZE]
    received_crc = _CRC.unpack(raw_packet[-CRC_SIZE:])[0]
    calculated_crc = crc16_ccitt(packet_without_crc)
    if received_crc != calculated_crc:
        raise _error(ProtocolErrorCode.CRC_MISMATCH, "CRC mismatch")

    version, type_value, payload_size, sequence, step_id = _HEADER.unpack(
        raw_packet[:HEADER_SIZE]
    )
    if version != PROTOCOL_VERSION:
        raise _error(
            ProtocolErrorCode.UNSUPPORTED_VERSION,
            f"unsupported protocol version: {version}",
        )
    if payload_size > MAX_PAYLOAD_SIZE:
        raise _error(
            ProtocolErrorCode.PAYLOAD_TOO_LARGE,
            "payload exceeds protocol limit",
        )
    payload = raw_packet[HEADER_SIZE:-CRC_SIZE]
    if len(payload) != payload_size:
        raise _error(
            ProtocolErrorCode.PAYLOAD_LENGTH_MISMATCH,
            "payload length mismatch",
        )
    try:
        message_type = MessageType(type_value)
    except ValueError as exc:
        raise _error(
            ProtocolErrorCode.UNKNOWN_MESSAGE_TYPE,
            f"unknown message type: {type_value}",
        ) from exc
    return Packet(message_type, sequence, step_id, payload)


def encode_typed_payload(message_type: MessageType, payload: TypedPayload) -> bytes:
    """Encode the exact Protocol v1 payload associated with ``message_type``."""

    message_type = _message_type(message_type)

    if message_type is MessageType.HELLO and isinstance(payload, HelloPayload):
        minimum = _uint("min_version", payload.min_version, 8)
        maximum = _uint("max_version", payload.max_version, 8)
        limit = _uint("max_payload", payload.max_payload, 16)
        timeout = _uint("heartbeat_timeout_ms", payload.heartbeat_timeout_ms, 16)
        if minimum == 0 or minimum > maximum:
            raise _error(
                ProtocolErrorCode.INVALID_VALUE,
                "HELLO version range is invalid",
            )
        if not 1 <= limit <= MAX_PAYLOAD_SIZE or timeout == 0:
            raise _error(
                ProtocolErrorCode.INVALID_VALUE,
                "HELLO limits must be nonzero and within Protocol v1 bounds",
            )
        return _HELLO.pack(
            _enum_value("role", payload.role, EndpointRole),
            minimum,
            maximum,
            0,
            _uint("capabilities", payload.capabilities, 32),
            limit,
            timeout,
        )
    if message_type is MessageType.HELLO_ACK and isinstance(payload, HelloAckPayload):
        accepted = _uint("accepted_version", payload.accepted_version, 8)
        status = _enum_value("status", payload.status, HelloStatus)
        limit = _uint("max_payload", payload.max_payload, 16)
        timeout = _uint("heartbeat_timeout_ms", payload.heartbeat_timeout_ms, 16)
        expected_version = PROTOCOL_VERSION if status == int(HelloStatus.OK) else 0
        if accepted != expected_version:
            raise _error(
                ProtocolErrorCode.INVALID_VALUE,
                "HELLO_ACK accepted_version and status disagree",
            )
        if not 1 <= limit <= MAX_PAYLOAD_SIZE or timeout == 0:
            raise _error(
                ProtocolErrorCode.INVALID_VALUE,
                "HELLO_ACK limits must be nonzero and within Protocol v1 bounds",
            )
        return _HELLO_ACK.pack(
            accepted,
            status,
            _enum_value("role", payload.role, EndpointRole),
            0,
            _uint("capabilities", payload.capabilities, 32),
            _uint("max_payload", payload.max_payload, 16),
            _uint("heartbeat_timeout_ms", payload.heartbeat_timeout_ms, 16),
        )
    if message_type is MessageType.SENSOR_FRAME and isinstance(
        payload, SensorFramePayload
    ):
        allowed = int(
            SensorMask.IMU
            | SensorMask.WHEEL_SPEED
            | SensorMask.GNSS
            | SensorMask.LANDMARK
        )
        data = bytearray(
            _SENSOR_PREFIX.pack(
                _mask("present_mask", payload.present_mask, allowed),
                _mask("fault_mask", payload.fault_mask, allowed),
            )
        )
        data.extend(
            _IMU_SAMPLE.pack(
                _uint("imu.sample_step_id", payload.imu.sample_step_id, 32),
                _uint("imu.timestamp_us", payload.imu.timestamp_us, 32),
                _finite(
                    "imu.longitudinal_acceleration_mps2",
                    payload.imu.longitudinal_acceleration_mps2,
                ),
                _finite("imu.yaw_rate_rps", payload.imu.yaw_rate_rps),
            )
        )
        data.extend(
            _WHEEL_SAMPLE.pack(
                _uint(
                    "wheel_speed.sample_step_id",
                    payload.wheel_speed.sample_step_id,
                    32,
                ),
                _uint(
                    "wheel_speed.timestamp_us", payload.wheel_speed.timestamp_us, 32
                ),
                _finite("wheel_speed.speed_mps", payload.wheel_speed.speed_mps),
            )
        )
        data.extend(
            _GNSS_SAMPLE.pack(
                _uint("gnss.sample_step_id", payload.gnss.sample_step_id, 32),
                _uint("gnss.timestamp_us", payload.gnss.timestamp_us, 32),
                _finite("gnss.x_m", payload.gnss.x_m),
                _finite("gnss.y_m", payload.gnss.y_m),
            )
        )
        data.extend(
            _LANDMARK_SAMPLE.pack(
                _uint("landmark.sample_step_id", payload.landmark.sample_step_id, 32),
                _uint("landmark.timestamp_us", payload.landmark.timestamp_us, 32),
                _uint("landmark.landmark_id", payload.landmark.landmark_id, 16),
                0,
                _finite("landmark.landmark_x_m", payload.landmark.landmark_x_m),
                _finite("landmark.landmark_y_m", payload.landmark.landmark_y_m),
                _finite("landmark.range_m", payload.landmark.range_m),
                _finite("landmark.bearing_rad", payload.landmark.bearing_rad),
            )
        )
        return bytes(data)
    if message_type is MessageType.ROUTE_CHUNK and isinstance(
        payload, RouteChunkPayload
    ):
        if len(payload.points) > MAX_ROUTE_POINTS_PER_CHUNK:
            raise _error(
                ProtocolErrorCode.INVALID_VALUE,
                f"route chunk exceeds {MAX_ROUTE_POINTS_PER_CHUNK} points",
            )
        route_id = _uint("route_id", payload.route_id, 16)
        start_index = _uint("start_index", payload.start_index, 16)
        total_count = _uint("total_count", payload.total_count, 16)
        flags = _mask(
            "route flags",
            payload.flags,
            int(RouteFlag.CLEAR_EXISTING | RouteFlag.FINAL_CHUNK | RouteFlag.LOOP_ROUTE),
        )
        count = len(payload.points)
        if count and (total_count == 0 or start_index + count > total_count):
            raise _error(
                ProtocolErrorCode.INVALID_VALUE,
                "route chunk indices exceed total_count",
            )
        if not count and not (flags & int(RouteFlag.CLEAR_EXISTING)):
            raise _error(
                ProtocolErrorCode.INVALID_VALUE,
                "an empty route chunk must clear the existing route",
            )
        data = bytearray(
            _ROUTE_PREFIX.pack(route_id, start_index, total_count, count, flags)
        )
        for index, point in enumerate(payload.points):
            data.extend(
                _ROUTE_POINT.pack(
                    _finite(f"points[{index}].x_m", point.x_m),
                    _finite(f"points[{index}].y_m", point.y_m),
                    _finite(
                        f"points[{index}].target_speed_mps", point.target_speed_mps
                    ),
                    _finite(
                        f"points[{index}].acceptance_radius_m",
                        point.acceptance_radius_m,
                    ),
                )
            )
        return bytes(data)
    if message_type is MessageType.CONTROL_COMMAND and isinstance(
        payload, ControlCommandPayload
    ):
        return _CONTROL_COMMAND.pack(
            _finite("steering_rad", payload.steering_rad),
            _finite("acceleration_mps2", payload.acceleration_mps2),
            _finite("target_speed_mps", payload.target_speed_mps),
            _enum_value("mode", payload.mode, ControlMode),
            _uint("flags", payload.flags, 8),
            0,
        )
    if message_type is MessageType.STATE_ESTIMATE and isinstance(
        payload, StateEstimatePayload
    ):
        if len(payload.covariance_diagonal) != 6:
            raise _error(
                ProtocolErrorCode.INVALID_VALUE,
                "covariance_diagonal must contain six values",
            )
        values = (
            _finite("x_m", payload.x_m),
            _finite("y_m", payload.y_m),
            _finite("heading_rad", payload.heading_rad),
            _finite("speed_mps", payload.speed_mps),
            _finite("yaw_rate_rps", payload.yaw_rate_rps),
            _finite("acceleration_bias_mps2", payload.acceleration_bias_mps2),
            *(
                _finite(f"covariance_diagonal[{index}]", value)
                for index, value in enumerate(payload.covariance_diagonal)
            ),
        )
        if any(value < 0.0 for value in values[6:]):
            raise _error(
                ProtocolErrorCode.INVALID_VALUE,
                "covariance diagonal values must be non-negative",
            )
        return _STATE_ESTIMATE.pack(
            *values,
            _enum_value("navigation_mode", payload.navigation_mode, NavigationMode),
            _uint("flags", payload.flags, 8),
            0,
        )
    if message_type is MessageType.HEALTH_STATUS and isinstance(
        payload, HealthStatusPayload
    ):
        return _HEALTH_STATUS.pack(
            _enum_value("runtime_state", payload.runtime_state, RuntimeState),
            _enum_value("navigation_mode", payload.navigation_mode, NavigationMode),
            _uint("flags", payload.flags, 16),
            *(
                _uint(name, value, 32)
                for name, value in (
                    ("uptime_ms", payload.uptime_ms),
                    ("last_sensor_age_ms", payload.last_sensor_age_ms),
                    ("rx_frames", payload.rx_frames),
                    ("rx_crc_errors", payload.rx_crc_errors),
                    ("rx_decode_errors", payload.rx_decode_errors),
                    ("rx_missing", payload.rx_missing),
                    ("rx_duplicates", payload.rx_duplicates),
                    ("rx_out_of_order", payload.rx_out_of_order),
                    ("rx_stale", payload.rx_stale),
                    ("queue_overflows", payload.queue_overflows),
                    ("scheduler_overruns", payload.scheduler_overruns),
                    ("max_loop_us", payload.max_loop_us),
                )
            ),
            *_nis_summary_fields(
                "imu_yaw",
                payload.imu_yaw_nis_evaluated_count,
                payload.imu_yaw_nis_gate_rejected_count,
                payload.imu_yaw_nis_sum,
                payload.imu_yaw_nis_max,
            ),
            *_nis_summary_fields(
                "wheel",
                payload.wheel_nis_evaluated_count,
                payload.wheel_nis_gate_rejected_count,
                payload.wheel_nis_sum,
                payload.wheel_nis_max,
            ),
            *_nis_summary_fields(
                "gnss",
                payload.gnss_nis_evaluated_count,
                payload.gnss_nis_gate_rejected_count,
                payload.gnss_nis_sum,
                payload.gnss_nis_max,
            ),
            *_nis_summary_fields(
                "landmark",
                payload.landmark_nis_evaluated_count,
                payload.landmark_nis_gate_rejected_count,
                payload.landmark_nis_sum,
                payload.landmark_nis_max,
            ),
        )
    if message_type is MessageType.HEARTBEAT and isinstance(payload, HeartbeatPayload):
        return _HEARTBEAT.pack(
            _uint("uptime_ms", payload.uptime_ms, 32),
            _uint("monotonic_ms", payload.monotonic_ms, 32),
            _enum_value("runtime_state", payload.runtime_state, RuntimeState),
        )
    if message_type is MessageType.ERROR and isinstance(payload, ErrorPayload):
        return _ERROR_PAYLOAD.pack(
            _enum_value("code", payload.code, ApplicationErrorCode),
            _uint("detail", payload.detail, 16),
            _uint("related_sequence", payload.related_sequence, 32),
            _uint("context", payload.context, 32),
        )
    if message_type is MessageType.SAFE_STOP and isinstance(payload, SafeStopPayload):
        if not isinstance(payload.latch, bool):
            raise _error(ProtocolErrorCode.INVALID_VALUE, "latch must be bool")
        return _SAFE_STOP.pack(
            _enum_value("reason", payload.reason, SafeStopReason),
            int(payload.latch),
            _uint("detail", payload.detail, 16),
        )
    raise _error(
        ProtocolErrorCode.INVALID_VALUE,
        f"payload type does not match {message_type.name}",
    )


def _require_payload_size(payload: bytes, expected: int, name: str) -> None:
    if len(payload) != expected:
        raise _error(
            ProtocolErrorCode.PAYLOAD_LENGTH_MISMATCH,
            f"{name} payload must be {expected} bytes, got {len(payload)}",
        )


def _decode_enum(name: str, value: int, enum_type: type[IntEnum]) -> IntEnum:
    try:
        return enum_type(value)
    except ValueError as exc:
        raise _error(
            ProtocolErrorCode.INVALID_VALUE,
            f"{name} has unknown value {value}",
        ) from exc


def decode_typed_payload(message_type: MessageType, payload: bytes) -> TypedPayload:
    """Decode and validate an application payload for ``message_type``."""

    message_type = _message_type(message_type)
    if not isinstance(payload, (bytes, bytearray, memoryview)):
        raise _error(ProtocolErrorCode.INVALID_VALUE, "payload must be bytes-like")
    data = bytes(payload)

    if message_type is MessageType.HELLO:
        _require_payload_size(data, _HELLO.size, "HELLO")
        role, minimum, maximum, reserved, capabilities, limit, timeout = _HELLO.unpack(data)
        if reserved:
            raise _error(ProtocolErrorCode.INVALID_VALUE, "HELLO reserved byte is nonzero")
        result = HelloPayload(
            _decode_enum("role", role, EndpointRole),  # type: ignore[arg-type]
            minimum,
            maximum,
            capabilities,
            limit,
            timeout,
        )
    elif message_type is MessageType.HELLO_ACK:
        _require_payload_size(data, _HELLO_ACK.size, "HELLO_ACK")
        accepted, status, role, reserved, capabilities, limit, timeout = _HELLO_ACK.unpack(data)
        if reserved:
            raise _error(
                ProtocolErrorCode.INVALID_VALUE, "HELLO_ACK reserved byte is nonzero"
            )
        result = HelloAckPayload(
            accepted,
            _decode_enum("status", status, HelloStatus),  # type: ignore[arg-type]
            _decode_enum("role", role, EndpointRole),  # type: ignore[arg-type]
            capabilities,
            limit,
            timeout,
        )
    elif message_type is MessageType.SENSOR_FRAME:
        expected = (
            _SENSOR_PREFIX.size
            + _IMU_SAMPLE.size
            + _WHEEL_SAMPLE.size
            + _GNSS_SAMPLE.size
            + _LANDMARK_SAMPLE.size
        )
        _require_payload_size(data, expected, "SENSOR_FRAME")
        offset = 0
        present, faults = _SENSOR_PREFIX.unpack_from(data, offset)
        offset += _SENSOR_PREFIX.size
        imu_step, imu_time, accel, yaw_rate = _IMU_SAMPLE.unpack_from(data, offset)
        offset += _IMU_SAMPLE.size
        wheel_step, wheel_time, speed = _WHEEL_SAMPLE.unpack_from(data, offset)
        offset += _WHEEL_SAMPLE.size
        gnss_step, gnss_time, x_m, y_m = _GNSS_SAMPLE.unpack_from(data, offset)
        offset += _GNSS_SAMPLE.size
        (
            landmark_step,
            landmark_time,
            landmark_id,
            reserved,
            landmark_x_m,
            landmark_y_m,
            range_m,
            bearing,
        ) = _LANDMARK_SAMPLE.unpack_from(data, offset)
        if reserved:
            raise _error(
                ProtocolErrorCode.INVALID_VALUE,
                "SENSOR_FRAME landmark reserved field is nonzero",
            )
        result = SensorFramePayload(
            SensorMask(present),
            SensorMask(faults),
            ImuSample(imu_step, imu_time, accel, yaw_rate),
            WheelSpeedSample(wheel_step, wheel_time, speed),
            GnssSample(gnss_step, gnss_time, x_m, y_m),
            LandmarkSample(
                landmark_step,
                landmark_time,
                landmark_id,
                landmark_x_m,
                landmark_y_m,
                range_m,
                bearing,
            ),
        )
    elif message_type is MessageType.ROUTE_CHUNK:
        if len(data) < _ROUTE_PREFIX.size:
            raise _error(
                ProtocolErrorCode.PAYLOAD_LENGTH_MISMATCH,
                "ROUTE_CHUNK payload is shorter than its prefix",
            )
        route_id, start, total, count, flags = _ROUTE_PREFIX.unpack_from(data)
        if count > MAX_ROUTE_POINTS_PER_CHUNK:
            raise _error(ProtocolErrorCode.INVALID_VALUE, "route point count exceeds limit")
        expected = _ROUTE_PREFIX.size + count * _ROUTE_POINT.size
        _require_payload_size(data, expected, "ROUTE_CHUNK")
        points = tuple(
            RouteWaypoint(
                *_ROUTE_POINT.unpack_from(data, _ROUTE_PREFIX.size + index * _ROUTE_POINT.size)
            )
            for index in range(count)
        )
        result = RouteChunkPayload(route_id, start, total, RouteFlag(flags), points)
    elif message_type is MessageType.CONTROL_COMMAND:
        _require_payload_size(data, _CONTROL_COMMAND.size, "CONTROL_COMMAND")
        steering, acceleration, target_speed, mode, flags, reserved = (
            _CONTROL_COMMAND.unpack(data)
        )
        if reserved:
            raise _error(
                ProtocolErrorCode.INVALID_VALUE,
                "CONTROL_COMMAND reserved field is nonzero",
            )
        result = ControlCommandPayload(
            steering,
            acceleration,
            target_speed,
            _decode_enum("mode", mode, ControlMode),  # type: ignore[arg-type]
            flags,
        )
    elif message_type is MessageType.STATE_ESTIMATE:
        _require_payload_size(data, _STATE_ESTIMATE.size, "STATE_ESTIMATE")
        values = _STATE_ESTIMATE.unpack(data)
        if values[14]:
            raise _error(
                ProtocolErrorCode.INVALID_VALUE,
                "STATE_ESTIMATE reserved field is nonzero",
            )
        result = StateEstimatePayload(
            *values[:6],
            tuple(values[6:12]),  # type: ignore[arg-type]
            _decode_enum("navigation_mode", values[12], NavigationMode),  # type: ignore[arg-type]
            values[13],
        )
    elif message_type is MessageType.HEALTH_STATUS:
        _require_payload_size(data, _HEALTH_STATUS.size, "HEALTH_STATUS")
        values = _HEALTH_STATUS.unpack(data)
        result = HealthStatusPayload(
            _decode_enum("runtime_state", values[0], RuntimeState),  # type: ignore[arg-type]
            _decode_enum("navigation_mode", values[1], NavigationMode),  # type: ignore[arg-type]
            *values[2:],
        )
    elif message_type is MessageType.HEARTBEAT:
        _require_payload_size(data, _HEARTBEAT.size, "HEARTBEAT")
        uptime, monotonic, state = _HEARTBEAT.unpack(data)
        result = HeartbeatPayload(
            uptime,
            monotonic,
            _decode_enum("runtime_state", state, RuntimeState),  # type: ignore[arg-type]
        )
    elif message_type is MessageType.ERROR:
        _require_payload_size(data, _ERROR_PAYLOAD.size, "ERROR")
        code, detail, related, context = _ERROR_PAYLOAD.unpack(data)
        result = ErrorPayload(
            _decode_enum("code", code, ApplicationErrorCode),  # type: ignore[arg-type]
            detail,
            related,
            context,
        )
    elif message_type is MessageType.SAFE_STOP:
        _require_payload_size(data, _SAFE_STOP.size, "SAFE_STOP")
        reason, latch, detail = _SAFE_STOP.unpack(data)
        if latch > 1:
            raise _error(ProtocolErrorCode.INVALID_VALUE, "SAFE_STOP latch is not boolean")
        result = SafeStopPayload(
            _decode_enum("reason", reason, SafeStopReason),  # type: ignore[arg-type]
            bool(latch),
            detail,
        )
    else:  # pragma: no cover - MessageType conversion above makes this exhaustive.
        raise _error(ProtocolErrorCode.UNKNOWN_MESSAGE_TYPE, "unknown message type")

    # Re-encoding centralizes finite/range/mask validation for decoded values.
    encode_typed_payload(message_type, result)
    return result


def make_packet(
    message_type: MessageType,
    sequence: int,
    step_id: int,
    payload: TypedPayload,
) -> Packet:
    return Packet(
        _message_type(message_type),
        sequence,
        step_id,
        encode_typed_payload(message_type, payload),
    )


def decode_packet_payload(packet: Packet) -> TypedPayload:
    return decode_typed_payload(packet.message_type, packet.payload)


def crc16_ccitt(data: bytes | bytearray | memoryview) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xffff, no reflection/xorout."""

    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = (
                ((crc << 1) ^ 0x1021) & 0xFFFF
                if crc & 0x8000
                else (crc << 1) & 0xFFFF
            )
    return crc


def cobs_encode(data: bytes | bytearray | memoryview) -> bytes:
    encoded = bytearray([0])
    code_index = 0
    code = 1
    for value in data:
        if value == 0:
            encoded[code_index] = code
            code_index = len(encoded)
            encoded.append(0)
            code = 1
        else:
            encoded.append(value)
            code += 1
            if code == 0xFF:
                encoded[code_index] = code
                code_index = len(encoded)
                encoded.append(0)
                code = 1
    encoded[code_index] = code
    return bytes(encoded)


def cobs_decode(data: bytes | bytearray | memoryview) -> bytes:
    if not data:
        raise _error(ProtocolErrorCode.COBS_MALFORMED, "COBS frame is empty")
    decoded = bytearray()
    index = 0
    while index < len(data):
        code = data[index]
        if code == 0:
            raise _error(
                ProtocolErrorCode.COBS_MALFORMED,
                "invalid zero byte in COBS frame",
            )
        index += 1
        block_end = index + code - 1
        if block_end > len(data):
            raise _error(
                ProtocolErrorCode.COBS_MALFORMED,
                "COBS block exceeds frame length",
            )
        decoded.extend(data[index:block_end])
        index = block_end
        if code != 0xFF and index < len(data):
            decoded.append(0)
    return bytes(decoded)


@dataclass(slots=True)
class ParserStats:
    bytes_received: int = 0
    frames_received: int = 0
    packets_accepted: int = 0
    empty_delimiters: int = 0
    oversized_frames: int = 0
    truncated_frames: int = 0
    cobs_errors: int = 0
    crc_errors: int = 0
    version_errors: int = 0
    type_errors: int = 0
    length_errors: int = 0
    other_errors: int = 0


@dataclass(frozen=True, slots=True)
class ParseFailure:
    code: ProtocolErrorCode
    message: str


@dataclass(frozen=True, slots=True)
class ParseBatch:
    packets: tuple[Packet, ...] = ()
    errors: tuple[ParseFailure, ...] = ()


class StreamParser:
    """Incremental bounded-frame parser for fragmented or combined reads."""

    def __init__(self) -> None:
        self.stats = ParserStats()
        self._encoded = bytearray()
        self._discard_until_delimiter = False

    def reset(self, *, clear_stats: bool = False) -> None:
        self._encoded.clear()
        self._discard_until_delimiter = False
        if clear_stats:
            self.stats = ParserStats()

    def _count_error(self, code: ProtocolErrorCode) -> None:
        if code is ProtocolErrorCode.COBS_MALFORMED:
            self.stats.cobs_errors += 1
        elif code is ProtocolErrorCode.CRC_MISMATCH:
            self.stats.crc_errors += 1
        elif code is ProtocolErrorCode.UNSUPPORTED_VERSION:
            self.stats.version_errors += 1
        elif code is ProtocolErrorCode.UNKNOWN_MESSAGE_TYPE:
            self.stats.type_errors += 1
        elif code in (
            ProtocolErrorCode.PACKET_TOO_SHORT,
            ProtocolErrorCode.PAYLOAD_LENGTH_MISMATCH,
            ProtocolErrorCode.PAYLOAD_TOO_LARGE,
        ):
            self.stats.length_errors += 1
        else:
            self.stats.other_errors += 1

    def feed(self, data: bytes | bytearray | memoryview) -> ParseBatch:
        if not isinstance(data, (bytes, bytearray, memoryview)):
            raise _error(ProtocolErrorCode.INVALID_VALUE, "stream input must be bytes-like")
        packets: list[Packet] = []
        errors: list[ParseFailure] = []
        self.stats.bytes_received += len(data)
        for value in data:
            if value == 0:
                if self._discard_until_delimiter:
                    self._discard_until_delimiter = False
                    self._encoded.clear()
                    continue
                if not self._encoded:
                    self.stats.empty_delimiters += 1
                    continue
                self.stats.frames_received += 1
                encoded = bytes(self._encoded)
                self._encoded.clear()
                try:
                    packet = decode_packet(encoded)
                except ProtocolError as exc:
                    self._count_error(exc.code)
                    errors.append(ParseFailure(exc.code, str(exc)))
                else:
                    self.stats.packets_accepted += 1
                    packets.append(packet)
                continue
            if self._discard_until_delimiter:
                continue
            if len(self._encoded) >= MAX_ENCODED_FRAME_SIZE:
                self._encoded.clear()
                self._discard_until_delimiter = True
                self.stats.oversized_frames += 1
                errors.append(
                    ParseFailure(
                        ProtocolErrorCode.OVERSIZED_FRAME,
                        "encoded frame exceeded protocol limit",
                    )
                )
                continue
            self._encoded.append(value)
        return ParseBatch(tuple(packets), tuple(errors))

    def finish(self) -> ParseBatch:
        """Declare end-of-stream and classify a pending unterminated frame."""

        if not self._encoded and not self._discard_until_delimiter:
            return ParseBatch()
        self._encoded.clear()
        self._discard_until_delimiter = False
        self.stats.truncated_frames += 1
        return ParseBatch(
            errors=(
                ParseFailure(
                    ProtocolErrorCode.TRUNCATED_FRAME,
                    "stream ended with an unterminated frame",
                ),
            )
        )


class SequenceDisposition(IntEnum):
    FIRST = 0
    IN_ORDER = 1
    GAP = 2
    DUPLICATE = 3
    OUT_OF_ORDER = 4
    STALE = 5


@dataclass(frozen=True, slots=True)
class SequenceResult:
    disposition: SequenceDisposition
    accepted: bool
    missing: int = 0


@dataclass(slots=True)
class SequenceStats:
    accepted: int = 0
    missing: int = 0
    duplicates: int = 0
    out_of_order: int = 0
    stale: int = 0


class SequenceTracker:
    """Track a single endpoint's wrapping uint32 sequence stream.

    Exact repeats are duplicates. A behind value within ``reorder_window`` is
    out-of-order; older values are stale. Forward gaps are accepted and counted.
    An optional older ``step_id`` makes an otherwise forward packet stale.
    """

    def __init__(self, reorder_window: int = 32) -> None:
        if not 1 <= reorder_window < 0x80000000:
            raise _error(
                ProtocolErrorCode.INVALID_VALUE,
                "reorder_window must be in [1, 2^31)",
            )
        self.reorder_window = reorder_window
        self.stats = SequenceStats()
        self._last_sequence: int | None = None
        self._last_step_id: int | None = None

    @staticmethod
    def _is_older(value: int, reference: int) -> bool:
        delta = (value - reference) & 0xFFFFFFFF
        return delta >= 0x80000000 and value != reference

    def reset(self, *, clear_stats: bool = False) -> None:
        self._last_sequence = None
        self._last_step_id = None
        if clear_stats:
            self.stats = SequenceStats()

    def observe(self, sequence: int, step_id: int | None = None) -> SequenceResult:
        sequence = _uint("sequence", sequence, 32)
        if step_id is not None:
            step_id = _uint("step_id", step_id, 32)
        if self._last_sequence is None:
            self._last_sequence = sequence
            self._last_step_id = step_id
            self.stats.accepted += 1
            return SequenceResult(SequenceDisposition.FIRST, True)

        delta = (sequence - self._last_sequence) & 0xFFFFFFFF
        if delta == 0:
            self.stats.duplicates += 1
            return SequenceResult(SequenceDisposition.DUPLICATE, False)
        if delta < 0x80000000:
            if (
                step_id is not None
                and self._last_step_id is not None
                and self._is_older(step_id, self._last_step_id)
            ):
                self.stats.stale += 1
                return SequenceResult(SequenceDisposition.STALE, False)
            missing = delta - 1
            disposition = (
                SequenceDisposition.IN_ORDER if not missing else SequenceDisposition.GAP
            )
            self._last_sequence = sequence
            if step_id is not None:
                self._last_step_id = step_id
            self.stats.accepted += 1
            self.stats.missing += missing
            return SequenceResult(disposition, True, missing)

        backwards = (self._last_sequence - sequence) & 0xFFFFFFFF
        if backwards <= self.reorder_window:
            self.stats.out_of_order += 1
            return SequenceResult(SequenceDisposition.OUT_OF_ORDER, False)
        self.stats.stale += 1
        return SequenceResult(SequenceDisposition.STALE, False)
