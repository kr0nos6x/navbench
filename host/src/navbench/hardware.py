"""Physical Protocol v1 serial validation without firmware upload behavior."""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import asdict, dataclass, field
from enum import IntEnum
from typing import Any

from navbench.protocol import (
    ApplicationErrorCode,
    ControlCommandPayload,
    ControlMode,
    EndpointRole,
    ErrorPayload,
    GnssSample,
    HealthStatusPayload,
    HelloAckPayload,
    HelloPayload,
    HelloStatus,
    ImuSample,
    MessageType,
    NavigationMode,
    PROTOCOL_VERSION,
    RouteWaypoint,
    RuntimeState,
    SensorFramePayload,
    SensorMask,
    StateEstimatePayload,
    StreamParser,
    WheelSpeedSample,
    decode_packet_payload,
    encode_packet,
    make_packet,
)
from navbench.session import (
    HostSession,
    SessionConfig,
    SessionError,
    SessionEvent,
    SessionState,
    SUPPORTED_CAPABILITIES,
)
from navbench.transport import (
    ByteTransport,
    PosixSerialTransport,
    SerialConfig,
    TransportError,
)


class HardwareExitCode(IntEnum):
    SUCCESS = 0
    PORT_OPEN_FAILED = 10
    HANDSHAKE_FAILED = 20
    EXCHANGE_FAILED = 30
    WATCHDOG_FAILED = 40
    DIAGNOSTIC_FAILED = 50


@dataclass(frozen=True, slots=True)
class HardwareConfig:
    port: str
    baud: int = 115200
    watchdog_check: bool = False
    startup_delay_s: float = 3.0


@dataclass(frozen=True, slots=True)
class HardwareValidationResult:
    exit_code: HardwareExitCode
    phase: str
    message: str
    summary: dict[str, Any] = field(default_factory=dict)

    @property
    def succeeded(self) -> bool:
        return self.exit_code is HardwareExitCode.SUCCESS

    def to_dict(self) -> dict[str, Any]:
        return {
            "status": "success" if self.succeeded else "error",
            "exit_code": int(self.exit_code),
            "phase": self.phase,
            "message": self.message,
            **self.summary,
        }


TransportFactory = Callable[[SerialConfig], ByteTransport]
Clock = Callable[[], float]
Sleeper = Callable[[float], None]

_SESSION_TIMEOUT_MS = 500
_HANDSHAKE_DEADLINE_S = 3.0
_FRAME_PERIOD_S = 0.020
_NORMAL_FRAME_COUNT = 10
_POLL_PERIOD_S = 0.005
_WATCHDOG_QUIET_S = 0.600
_WATCHDOG_MINIMUM_S = 0.500
_MAXIMUM_RESPONSE_LAG_STEPS = 5
_DIAGNOSTIC_DEADLINE_S = 3.0
_DIAGNOSTIC_MAGIC = 0x4E424447
_DIAGNOSTIC_USB_PROBE = b"\x01\x00"


def run_physical_validation(
    config: HardwareConfig,
    *,
    transport_factory: TransportFactory = PosixSerialTransport,
    clock: Clock = time.monotonic,
    sleep: Sleeper = time.sleep,
) -> HardwareValidationResult:
    """Open one configured device and run the existing Protocol v1 session."""

    if (
        isinstance(config.startup_delay_s, bool)
        or not isinstance(config.startup_delay_s, (int, float))
        or not math.isfinite(config.startup_delay_s)
        or config.startup_delay_s < 0.0
        or config.startup_delay_s > 30.0
    ):
        return _failure(
            HardwareExitCode.PORT_OPEN_FAILED,
            "port_open",
            "startup delay must be finite and between 0 and 30 seconds",
            port=config.port,
            baud=config.baud,
        )
    try:
        serial_config = SerialConfig(config.port, config.baud)
        transport = transport_factory(serial_config)
    except (OSError, TransportError, TypeError, ValueError) as error:
        return _failure(
            HardwareExitCode.PORT_OPEN_FAILED,
            "port_open",
            str(error),
            port=config.port,
            baud=config.baud,
        )

    try:
        if config.startup_delay_s > 0.0:
            sleep(config.startup_delay_s)
        reset_input = getattr(transport, "reset_input_buffer", None)
        if callable(reset_input):
            reset_input()
        return validate_transport(
            transport,
            config=config,
            clock=clock,
            sleep=sleep,
        )
    except (OSError, TransportError, TypeError, ValueError) as error:
        return _failure(
            HardwareExitCode.PORT_OPEN_FAILED,
            "port_prepare",
            str(error),
            port=config.port,
            baud=config.baud,
            startup_delay_s=config.startup_delay_s,
        )
    finally:
        transport.close()


def run_serial_diagnostic(
    config: HardwareConfig,
    mode: str,
    *,
    transport_factory: TransportFactory = PosixSerialTransport,
    clock: Clock = time.monotonic,
    sleep: Sleeper = time.sleep,
) -> HardwareValidationResult:
    """Run the diagnostic firmware path without opening the port twice."""

    if mode not in ("usb", "protocol"):
        return _failure(
            HardwareExitCode.DIAGNOSTIC_FAILED,
            "diagnostic",
            "diagnostic mode must be 'usb' or 'protocol'",
        )
    if (
        isinstance(config.startup_delay_s, bool)
        or not isinstance(config.startup_delay_s, (int, float))
        or not math.isfinite(config.startup_delay_s)
        or config.startup_delay_s < 0.0
        or config.startup_delay_s > 30.0
    ):
        return _failure(
            HardwareExitCode.PORT_OPEN_FAILED,
            "port_open",
            "startup delay must be finite and between 0 and 30 seconds",
        )
    try:
        transport = transport_factory(SerialConfig(config.port, config.baud))
    except (OSError, TransportError, TypeError, ValueError) as error:
        return _failure(
            HardwareExitCode.PORT_OPEN_FAILED,
            "port_open",
            str(error),
            port=config.port,
            baud=config.baud,
        )

    try:
        if config.startup_delay_s > 0.0:
            sleep(config.startup_delay_s)
        reset_input = getattr(transport, "reset_input_buffer", None)
        if callable(reset_input):
            reset_input()
        return validate_serial_diagnostic(
            transport,
            config=config,
            mode=mode,
            clock=clock,
            sleep=sleep,
        )
    except (OSError, TransportError, TypeError, ValueError) as error:
        return _failure(
            HardwareExitCode.DIAGNOSTIC_FAILED,
            "diagnostic",
            str(error),
            port=config.port,
            baud=config.baud,
            startup_delay_s=config.startup_delay_s,
        )
    finally:
        transport.close()


def validate_serial_diagnostic(
    transport: ByteTransport,
    *,
    config: HardwareConfig,
    mode: str,
    clock: Clock = time.monotonic,
    sleep: Sleeper = time.sleep,
) -> HardwareValidationResult:
    """Decode build-gated diagnostics carried in ordinary Protocol v1 frames."""

    parser = StreamParser()
    status = _DiagnosticStatus()
    deadline = clock() + _DIAGNOSTIC_DEADLINE_S
    while clock() <= deadline and not status.beacon_seen:
        _poll_diagnostic(transport, parser, status)
        sleep(_POLL_PERIOD_S)
    if not status.beacon_seen:
        return _diagnostic_failure(
            config,
            mode,
            "a valid diagnostic beacon was not received",
            parser,
            status,
        )

    baseline_rx = status.serial_rx_bytes
    baseline_errors = status.parser_errors
    baseline_parser_frames = status.parser_frames_received
    baseline_host_errors = _host_parser_error_snapshot(parser)
    if mode == "usb":
        _write_all(
            transport,
            _DIAGNOSTIC_USB_PROBE,
            clock=clock,
            sleep=sleep,
        )
        deadline = clock() + _DIAGNOSTIC_DEADLINE_S
        while clock() <= deadline:
            _poll_diagnostic(transport, parser, status)
            if (
                status.serial_rx_bytes >= baseline_rx + len(_DIAGNOSTIC_USB_PROBE)
                and status.parser_errors > baseline_errors
            ):
                return _diagnostic_success(config, mode, parser, status)
            sleep(_POLL_PERIOD_S)
        return _diagnostic_failure(
            config,
            mode,
            "UART bridge RX byte counter did not observe the binary probe",
            parser,
            status,
        )

    hello = HelloPayload(
        role=EndpointRole.HOST,
        capabilities=SUPPORTED_CAPABILITIES,
        heartbeat_timeout_ms=_SESSION_TIMEOUT_MS,
    )
    hello_frame = encode_packet(make_packet(MessageType.HELLO, 0, 0, hello))
    _write_all(transport, hello_frame, clock=clock, sleep=sleep)
    deadline = clock() + _DIAGNOSTIC_DEADLINE_S
    last_error = "HELLO parse result and HELLO_ACK were not both observed"
    while clock() <= deadline:
        _poll_diagnostic(transport, parser, status)
        if status.hello_ack is not None or status.hello_result != 0:
            error = _diagnostic_hello_error(
                status.hello_ack,
                status,
                minimum_rx=baseline_rx + len(hello_frame),
                minimum_parser_frames=baseline_parser_frames + 1,
            )
            if _host_parser_error_snapshot(parser) != baseline_host_errors:
                last_error = (
                    "host parser observed a new invalid frame during the "
                    "HELLO exchange"
                )
            elif error is None:
                return _diagnostic_success(config, mode, parser, status)
            else:
                last_error = error
        sleep(_POLL_PERIOD_S)
    return _diagnostic_failure(
        config,
        mode,
        last_error,
        parser,
        status,
    )


@dataclass(slots=True)
class _DiagnosticStatus:
    beacon_seen: bool = False
    tx_proven: bool = False
    write_diagnostic_seen: bool = False
    uptime_ms: int = 0
    serial_rx_bytes: int = 0
    parser_frames_received: int = 0
    parser_status: int = 0
    parser_errors: int = 0
    hello_result: int = 0
    hello_packets: int = 0
    response_frames_created: int = 0
    response_frames_dropped: int = 0
    response_frames_pending: int = 0
    serial_tx_bytes: int = 0
    last_write_requested: int = 0
    last_write_result: int = 0
    diagnostic_frames: int = 0
    hello_ack: HelloAckPayload | None = None

    def summary(self) -> dict[str, Any]:
        hello_names = {
            0: "NOT_SEEN",
            1: "ACCEPTED",
            2: "PAYLOAD_DECODE_ERROR",
            3: "WRONG_ROLE",
            4: "VERSION_MISMATCH",
            5: "CAPABILITY_MISMATCH",
            6: "PAYLOAD_LIMIT",
            7: "TIMEOUT_MISMATCH",
        }
        parser_names = {
            0: "OK",
            1: "NULL_ARGUMENT",
            2: "BUFFER_TOO_SMALL",
            3: "EMPTY_FRAME",
            4: "UNEXPECTED_DELIMITER",
            5: "COBS_MALFORMED",
            6: "PACKET_TOO_SHORT",
            7: "CRC_MISMATCH",
            8: "UNSUPPORTED_VERSION",
            9: "UNKNOWN_MESSAGE_TYPE",
            10: "PAYLOAD_LENGTH_MISMATCH",
            11: "PAYLOAD_TOO_LARGE",
            12: "INVALID_VALUE",
            13: "OVERSIZED_FRAME",
            14: "TRUNCATED_FRAME",
        }
        return {
            "beacon_seen": self.beacon_seen,
            "tx_proven": self.tx_proven,
            "write_diagnostic_seen": self.write_diagnostic_seen,
            "uptime_ms": self.uptime_ms,
            "serial_rx_bytes": self.serial_rx_bytes,
            "parser_frames_received": self.parser_frames_received,
            "parser_status": parser_names.get(
                self.parser_status, f"UNKNOWN_{self.parser_status}"
            ),
            "parser_errors": self.parser_errors,
            "hello_result": hello_names.get(
                self.hello_result, f"UNKNOWN_{self.hello_result}"
            ),
            "hello_packets": self.hello_packets,
            "response_frames_created": self.response_frames_created,
            "response_frames_dropped": self.response_frames_dropped,
            "response_frames_pending": self.response_frames_pending,
            "serial_tx_bytes": self.serial_tx_bytes,
            "last_write_requested": self.last_write_requested,
            "last_write_result": self.last_write_result,
            "diagnostic_frames": self.diagnostic_frames,
            "hello_ack": (
                {
                    "status": self.hello_ack.status.name,
                    "accepted_version": self.hello_ack.accepted_version,
                    "role": self.hello_ack.role.name,
                    "capabilities": self.hello_ack.capabilities,
                    "max_payload": self.hello_ack.max_payload,
                    "heartbeat_timeout_ms": self.hello_ack.heartbeat_timeout_ms,
                }
                if self.hello_ack is not None
                else None
            ),
        }


def _poll_diagnostic(
    transport: ByteTransport,
    parser: StreamParser,
    status: _DiagnosticStatus,
) -> None:
    for _ in range(32):
        data = transport.read()
        if not data:
            return
        batch = parser.feed(data)
        for packet in batch.packets:
            try:
                payload = decode_packet_payload(packet)
            except ValueError:
                continue
            if isinstance(payload, HelloAckPayload):
                status.hello_ack = payload
                continue
            if not (
                isinstance(payload, ErrorPayload)
                and payload.code is ApplicationErrorCode.DIAGNOSTIC
            ):
                continue
            status.diagnostic_frames += 1
            if payload.detail == 1 and payload.related_sequence == _DIAGNOSTIC_MAGIC:
                status.beacon_seen = True
                # A frame that passed Protocol v1 COBS, CRC, length, type and
                # payload validation is direct proof of firmware-to-host TX.
                # Its payload cannot report the write that is still in flight.
                status.tx_proven = True
                status.uptime_ms = payload.context
            elif payload.detail == 2:
                status.serial_rx_bytes = max(
                    status.serial_rx_bytes, payload.related_sequence
                )
                status.parser_frames_received = max(
                    status.parser_frames_received, payload.context
                )
            elif payload.detail == 3:
                hello_packets = payload.related_sequence & 0xFFFF
                if hello_packets >= status.hello_packets:
                    status.hello_result = (
                        payload.related_sequence >> 24
                    ) & 0xFF
                    status.parser_status = (
                        payload.related_sequence >> 16
                    ) & 0xFF
                    status.hello_packets = hello_packets
                    status.parser_errors = max(
                        status.parser_errors, payload.context
                    )
            elif payload.detail == 4:
                status.write_diagnostic_seen = True
                status.last_write_requested = (
                    payload.related_sequence >> 16
                ) & 0xFFFF
                status.last_write_result = payload.related_sequence & 0xFFFF
                status.serial_tx_bytes = max(status.serial_tx_bytes, payload.context)
            elif payload.detail == 5:
                status.response_frames_created = max(
                    status.response_frames_created, payload.related_sequence
                )
                status.response_frames_dropped = max(
                    status.response_frames_dropped,
                    (payload.context >> 16) & 0xFFFF,
                )
                status.response_frames_pending = payload.context & 0xFFFF


def _write_all(
    transport: ByteTransport,
    data: bytes,
    *,
    clock: Clock,
    sleep: Sleeper,
) -> None:
    offset = 0
    deadline = clock() + 1.0
    while offset < len(data):
        written = transport.write(data[offset:])
        if written < 0 or written > len(data) - offset:
            raise TransportError("serial transport returned an invalid write count")
        offset += written
        if offset == len(data):
            return
        if clock() > deadline:
            raise TransportError("serial diagnostic write timed out")
        sleep(_POLL_PERIOD_S)


def _diagnostic_hello_error(
    ack: HelloAckPayload | None,
    status: _DiagnosticStatus,
    *,
    minimum_rx: int,
    minimum_parser_frames: int,
) -> str | None:
    if status.serial_rx_bytes < minimum_rx:
        return "firmware RX counter did not include the complete HELLO frame"
    if status.hello_result != 1 or status.parser_status != 0:
        return "firmware diagnostic did not report an accepted HELLO"
    if status.hello_packets != 1:
        return "firmware diagnostic did not report exactly one HELLO packet"
    if status.parser_frames_received < minimum_parser_frames:
        return "firmware parser did not report accepting the HELLO frame"
    if status.response_frames_created != 1:
        return "firmware did not report exactly one HELLO response"
    if status.response_frames_dropped != 0:
        return "firmware dropped a HELLO response frame"
    if status.response_frames_pending != 0:
        return "firmware HELLO response remained pending"
    if status.write_diagnostic_seen and (
        status.last_write_requested != 0 or status.last_write_result != 0
    ):
        if (
            status.last_write_requested == 0
            or status.last_write_result != status.last_write_requested
        ):
            return "firmware reported an incomplete serial write"
    if ack is None:
        return "HELLO_ACK was not observed"
    if not (
        ack.status is HelloStatus.OK
        and ack.accepted_version == PROTOCOL_VERSION
        and ack.role is EndpointRole.CONTROLLER
        and ack.capabilities & SUPPORTED_CAPABILITIES == SUPPORTED_CAPABILITIES
        and ack.max_payload >= 116
        and ack.heartbeat_timeout_ms == _SESSION_TIMEOUT_MS
    ):
        return "HELLO_ACK fields are incompatible with the host"
    return None


def _host_parser_error_snapshot(parser: StreamParser) -> tuple[int, ...]:
    stats = parser.stats
    return (
        stats.cobs_errors,
        stats.crc_errors,
        stats.version_errors,
        stats.type_errors,
        stats.length_errors,
        stats.oversized_frames,
        stats.truncated_frames,
        stats.other_errors,
    )


def _diagnostic_success(
    config: HardwareConfig,
    mode: str,
    parser: StreamParser,
    status: _DiagnosticStatus,
) -> HardwareValidationResult:
    return HardwareValidationResult(
        HardwareExitCode.SUCCESS,
        f"diagnostic_{mode}",
        f"diagnostic {mode} path verified",
        {
            "port": config.port,
            "baud": config.baud,
            "startup_delay_s": config.startup_delay_s,
            "diagnostic": status.summary(),
            "host_parser_statistics": asdict(parser.stats),
        },
    )


def _diagnostic_failure(
    config: HardwareConfig,
    mode: str,
    message: str,
    parser: StreamParser,
    status: _DiagnosticStatus,
) -> HardwareValidationResult:
    return _failure(
        HardwareExitCode.DIAGNOSTIC_FAILED,
        f"diagnostic_{mode}",
        message,
        port=config.port,
        baud=config.baud,
        startup_delay_s=config.startup_delay_s,
        diagnostic=status.summary(),
        host_parser_statistics=asdict(parser.stats),
    )


def validate_transport(
    transport: ByteTransport,
    *,
    config: HardwareConfig,
    clock: Clock = time.monotonic,
    sleep: Sleeper = time.sleep,
) -> HardwareValidationResult:
    """Validate a supplied transport; tests use this without opening a port."""

    started_s = clock()

    def now_ms() -> int:
        elapsed_s = clock() - started_s
        if not math.isfinite(elapsed_s) or elapsed_s < 0.0:
            raise ValueError("clock must be finite and monotonic")
        return int(elapsed_s * 1000.0)

    session = HostSession(
        transport,
        SessionConfig(timeout_ms=_SESSION_TIMEOUT_MS),
    )
    try:
        session.start(now_ms())
        handshake_deadline = clock() + _HANDSHAKE_DEADLINE_S
        while clock() <= handshake_deadline:
            session.poll(now_ms())
            if session.state is SessionState.ACTIVE:
                break
            if session.state is SessionState.ERROR:
                return _failure(
                    HardwareExitCode.HANDSHAKE_FAILED,
                    "handshake",
                    "HELLO_ACK was rejected by Protocol v1 session policy",
                    port=config.port,
                    baud=config.baud,
                    startup_delay_s=config.startup_delay_s,
                    **_session_summary(session),
                )
            if session.state is SessionState.TIMED_OUT:
                session.reconnect(now_ms())
            sleep(_POLL_PERIOD_S)
        if session.state is not SessionState.ACTIVE:
            return _failure(
                HardwareExitCode.HANDSHAKE_FAILED,
                "handshake",
                "compatible HELLO_ACK was not received before the deadline",
                port=config.port,
                baud=config.baud,
                startup_delay_s=config.startup_delay_s,
                **_session_summary(session),
            )
    except (SessionError, TransportError, ValueError) as error:
        return _failure(
            HardwareExitCode.HANDSHAKE_FAILED,
            "handshake",
            str(error),
            port=config.port,
            baud=config.baud,
            startup_delay_s=config.startup_delay_s,
            **_session_summary(session),
        )

    handshake = {
        "accepted_version": PROTOCOL_VERSION,
        "peer_capabilities": session.peer_capabilities,
        "negotiated_max_payload": session.negotiated_max_payload,
        "negotiated_timeout_ms": session.negotiated_timeout_ms,
    }
    responses = _ResponseAccumulator()
    last_sensor_sent_s = clock()
    try:
        session.send_route(
            route_id=1,
            points=(
                RouteWaypoint(0.0, 0.0, 0.0, 0.25),
                RouteWaypoint(1.0, 0.0, 0.0, 0.25),
            ),
        )
        for step_id in range(1, _NORMAL_FRAME_COUNT + 1):
            last_sensor_sent_s = clock()
            session.send_sensor(step_id, _stationary_sensor_frame(step_id))
            deadline = clock() + _FRAME_PERIOD_S
            while clock() < deadline:
                error = responses.consume(
                    session.poll(now_ms()),
                    current_step_id=step_id,
                )
                if error is not None:
                    raise SessionError(error)
                sleep(min(_POLL_PERIOD_S, max(0.0, deadline - clock())))
            error = responses.consume(
                session.poll(now_ms()),
                current_step_id=step_id,
            )
            if error is not None:
                raise SessionError(error)
    except (SessionError, TransportError, ValueError) as error:
        return _failure(
            HardwareExitCode.EXCHANGE_FAILED,
            "normal_exchange",
            str(error),
            handshake=handshake,
            **_session_summary(session),
        )

    exchange_error = _normal_exchange_error(session, responses)
    if exchange_error is not None:
        return _failure(
            HardwareExitCode.EXCHANGE_FAILED,
            "normal_exchange",
            exchange_error,
            handshake=handshake,
            responses=responses.summary(),
            **_session_summary(session),
        )

    summary: dict[str, Any] = {
        "port": config.port,
        "baud": config.baud,
        "startup_delay_s": config.startup_delay_s,
        "handshake": handshake,
        "normal_exchange": {
            "sensor_frames_sent": _NORMAL_FRAME_COUNT,
            "frame_period_ms": int(_FRAME_PERIOD_S * 1000.0),
            "responses": responses.summary(),
        },
        **_session_summary(session),
    }
    if not config.watchdog_check:
        return HardwareValidationResult(
            HardwareExitCode.SUCCESS,
            "complete",
            "handshake and normal binary exchange verified",
            summary,
        )

    quiet_started_s = last_sensor_sent_s
    watchdog_evidence: list[str] = []
    watchdog_observed_ms: int | None = None
    try:
        while clock() - quiet_started_s < _WATCHDOG_QUIET_S:
            elapsed_s = clock() - quiet_started_s
            events = session.poll(now_ms())
            if elapsed_s >= _WATCHDOG_MINIMUM_S:
                evidence = _safe_stop_evidence(
                    events,
                    current_step_id=_NORMAL_FRAME_COUNT,
                )
                if evidence and watchdog_observed_ms is None:
                    watchdog_observed_ms = int(elapsed_s * 1000.0)
                watchdog_evidence.extend(evidence)
            sleep(
                min(
                    _POLL_PERIOD_S,
                    max(0.0, _WATCHDOG_QUIET_S - (clock() - quiet_started_s)),
                )
            )
        elapsed_s = clock() - quiet_started_s
        if elapsed_s >= _WATCHDOG_MINIMUM_S:
            evidence = _safe_stop_evidence(
                session.poll(now_ms()),
                current_step_id=_NORMAL_FRAME_COUNT,
            )
            if evidence and watchdog_observed_ms is None:
                watchdog_observed_ms = int(elapsed_s * 1000.0)
            watchdog_evidence.extend(evidence)
    except (SessionError, TransportError, ValueError) as error:
        return _failure(
            HardwareExitCode.WATCHDOG_FAILED,
            "watchdog_check",
            str(error),
            **summary,
        )

    if not watchdog_evidence:
        return _failure(
            HardwareExitCode.WATCHDOG_FAILED,
            "watchdog_check",
            "SAFE_STOP was not proven after 600 ms without sensor traffic",
            watchdog={
                "quiet_period_ms": int(_WATCHDOG_QUIET_S * 1000.0),
                "observed_after_ms": None,
                "evidence": [],
            },
            **summary,
        )
    summary["watchdog"] = {
        "quiet_period_ms": int(_WATCHDOG_QUIET_S * 1000.0),
        "observed_after_ms": watchdog_observed_ms,
        "evidence": sorted(set(watchdog_evidence)),
    }
    summary.update(_session_summary(session))
    return HardwareValidationResult(
        HardwareExitCode.SUCCESS,
        "complete",
        "handshake, normal exchange, and watchdog SAFE_STOP verified",
        summary,
    )


@dataclass(slots=True)
class _ResponseAccumulator:
    command: ControlCommandPayload | None = None
    estimate: StateEstimatePayload | None = None
    health: HealthStatusPayload | None = None
    command_count: int = 0
    estimate_count: int = 0
    health_count: int = 0

    def consume(
        self,
        events: tuple[SessionEvent, ...],
        *,
        current_step_id: int,
    ) -> str | None:
        for event in events:
            if isinstance(event.payload, ErrorPayload):
                return (
                    "firmware ERROR "
                    f"{event.payload.code.name} detail={event.payload.detail}"
                )
            if event.packet.message_type in (
                MessageType.CONTROL_COMMAND,
                MessageType.STATE_ESTIMATE,
            ):
                if event.packet.step_id > current_step_id:
                    return "controller response step_id is in the future"
                if (
                    current_step_id - event.packet.step_id
                    > _MAXIMUM_RESPONSE_LAG_STEPS
                ):
                    return "controller response step_id is stale"
            if isinstance(event.payload, ControlCommandPayload):
                self.command = event.payload
                self.command_count += 1
            elif isinstance(event.payload, StateEstimatePayload):
                self.estimate = event.payload
                self.estimate_count += 1
            elif isinstance(event.payload, HealthStatusPayload):
                self.health = event.payload
                self.health_count += 1
        return None

    def summary(self) -> dict[str, Any]:
        return {
            "control_command_count": self.command_count,
            "state_estimate_count": self.estimate_count,
            "health_status_count": self.health_count,
            "last_control_command": (
                {
                    "mode": self.command.mode.name,
                    "steering_rad": self.command.steering_rad,
                    "acceleration_mps2": self.command.acceleration_mps2,
                    "target_speed_mps": self.command.target_speed_mps,
                    "flags": self.command.flags,
                }
                if self.command is not None
                else None
            ),
            "last_state_estimate": (
                {
                    "x_m": self.estimate.x_m,
                    "y_m": self.estimate.y_m,
                    "heading_rad": self.estimate.heading_rad,
                    "speed_mps": self.estimate.speed_mps,
                    "navigation_mode": self.estimate.navigation_mode.name,
                }
                if self.estimate is not None
                else None
            ),
            "last_health_status": (
                {
                    "runtime_state": self.health.runtime_state.name,
                    "navigation_mode": self.health.navigation_mode.name,
                    "last_sensor_age_ms": self.health.last_sensor_age_ms,
                    "rx_frames": self.health.rx_frames,
                    "rx_crc_errors": self.health.rx_crc_errors,
                    "rx_decode_errors": self.health.rx_decode_errors,
                    "rx_stale": self.health.rx_stale,
                    "queue_overflows": self.health.queue_overflows,
                }
                if self.health is not None
                else None
            ),
        }


def _stationary_sensor_frame(step_id: int) -> SensorFramePayload:
    timestamp_us = step_id * int(_FRAME_PERIOD_S * 1_000_000.0)
    return SensorFramePayload(
        present_mask=SensorMask.IMU | SensorMask.WHEEL_SPEED | SensorMask.GNSS,
        imu=ImuSample(step_id, timestamp_us, 0.0, 0.0),
        wheel_speed=WheelSpeedSample(step_id, timestamp_us, 0.0),
        gnss=GnssSample(step_id, timestamp_us, 0.0, 0.0),
    )


def _normal_exchange_error(
    session: HostSession,
    responses: _ResponseAccumulator,
) -> str | None:
    if responses.command is None:
        return "CONTROL_COMMAND was not received"
    if responses.estimate is None:
        return "STATE_ESTIMATE was not received"
    if responses.health is None:
        return "HEALTH_STATUS was not received"
    if responses.command.mode is ControlMode.SAFE_STOP:
        return "normal exchange unexpectedly returned SAFE_STOP"
    if responses.estimate.navigation_mode is not NavigationMode.GNSS_AIDED:
        return "normal exchange did not reach GNSS_AIDED navigation"
    if responses.health.runtime_state is not RuntimeState.RUNNING:
        return "normal exchange did not reach RUNNING state"
    parser = session.parser.stats
    if (
        parser.cobs_errors
        or parser.crc_errors
        or parser.length_errors
        or parser.oversized_frames
        or parser.truncated_frames
        or session.stats.rx_payload_errors
        or session.stats.rx_rejected
    ):
        return "normal exchange contained rejected, corrupt, or stale responses"
    return None


def _safe_stop_evidence(
    events: tuple[SessionEvent, ...],
    *,
    current_step_id: int,
) -> list[str]:
    evidence: list[str] = []
    for event in events:
        if (
            event.packet.step_id > current_step_id
            or current_step_id - event.packet.step_id
            > _MAXIMUM_RESPONSE_LAG_STEPS
        ):
            continue
        if (
            isinstance(event.payload, ControlCommandPayload)
            and event.payload.mode is ControlMode.SAFE_STOP
        ):
            evidence.append("CONTROL_COMMAND.mode=SAFE_STOP")
        if (
            isinstance(event.payload, HealthStatusPayload)
            and event.payload.runtime_state is RuntimeState.SAFE_STOP
        ):
            evidence.append("HEALTH_STATUS.runtime_state=SAFE_STOP")
    return evidence


def _session_summary(session: HostSession) -> dict[str, Any]:
    return {
        "session_state": session.state.name,
        "session_statistics": asdict(session.stats),
        "parser_statistics": asdict(session.parser.stats),
        "sequence_statistics": asdict(session.rx_sequence.stats),
    }


def _failure(
    exit_code: HardwareExitCode,
    phase: str,
    message: str,
    **summary: Any,
) -> HardwareValidationResult:
    return HardwareValidationResult(exit_code, phase, message, summary)
