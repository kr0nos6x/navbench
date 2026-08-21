"""Deterministic host-side Protocol v1 session policy."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

from navbench.protocol import (
    MAX_ROUTE_POINTS_PER_CHUNK,
    MAX_PAYLOAD_SIZE,
    Capability,
    EndpointRole,
    HeartbeatPayload,
    HelloAckPayload,
    HelloPayload,
    HelloStatus,
    MessageType,
    Packet,
    ProtocolError,
    RouteChunkPayload,
    RouteFlag,
    RouteWaypoint,
    RuntimeState,
    SafeStopPayload,
    SafeStopReason,
    SensorFramePayload,
    SequenceDisposition,
    SequenceTracker,
    StreamParser,
    TypedPayload,
    decode_packet_payload,
    encode_packet,
    make_packet,
)
from navbench.transport import ByteTransport, TransportError


SUPPORTED_CAPABILITIES = int(
    Capability.SENSOR_FRAME
    | Capability.ROUTE_CHUNK
    | Capability.STATE_ESTIMATE
    | Capability.HEALTH_STATUS
    | Capability.SAFE_STOP
)
DEFAULT_REQUIRED_CAPABILITIES = SUPPORTED_CAPABILITIES
# A controller advertising SENSOR_FRAME must be able to receive the fixed-size
# aggregate payload.  Route chunks are reduced to fit a smaller negotiated
# limit, but SENSOR_FRAME cannot be split.
SENSOR_FRAME_PAYLOAD_SIZE = len(
    make_packet(
        MessageType.SENSOR_FRAME,
        0,
        0,
        SensorFramePayload(),
    ).payload
)
ROUTE_PREFIX_PAYLOAD_SIZE = len(
    make_packet(
        MessageType.ROUTE_CHUNK,
        0,
        0,
        RouteChunkPayload(
            route_id=0,
            start_index=0,
            total_count=1,
            flags=RouteFlag.CLEAR_EXISTING | RouteFlag.FINAL_CHUNK,
            points=(RouteWaypoint(0.0, 0.0, 0.0, 0.1),),
        ),
    ).payload
) - 16


class SessionState(IntEnum):
    DISCONNECTED = 0
    HELLO_SENT = 1
    ACTIVE = 2
    TIMED_OUT = 3
    SAFE_STOP = 4
    ERROR = 5
    CLOSED = 6


class SessionError(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class SessionConfig:
    timeout_ms: int = 500
    max_reads_per_poll: int = 32
    max_tx_buffer: int = 4096
    required_capabilities: int = DEFAULT_REQUIRED_CAPABILITIES

    def __post_init__(self) -> None:
        if self.timeout_ms <= 0:
            raise ValueError("timeout_ms must be greater than zero")
        if self.max_reads_per_poll <= 0:
            raise ValueError("max_reads_per_poll must be greater than zero")
        if self.max_tx_buffer < 256:
            raise ValueError("max_tx_buffer must be at least 256 bytes")
        if (
            isinstance(self.required_capabilities, bool)
            or not isinstance(self.required_capabilities, int)
        ):
            raise TypeError("required_capabilities must be an integer mask")
        if (
            self.required_capabilities < 0
            or self.required_capabilities & ~SUPPORTED_CAPABILITIES
        ):
            raise ValueError("required_capabilities contains unsupported bits")


@dataclass(slots=True)
class SessionStats:
    tx_packets: int = 0
    tx_bytes: int = 0
    rx_packets: int = 0
    rx_bytes: int = 0
    rx_rejected: int = 0
    rx_payload_errors: int = 0
    timeouts: int = 0
    reconnects: int = 0
    transport_errors: int = 0


@dataclass(frozen=True, slots=True)
class SessionEvent:
    packet: Packet
    payload: TypedPayload
    sequence_disposition: SequenceDisposition


class HostSession:
    """Host handshake, sequencing, timeout, and typed packet exchange.

    Time is supplied by the caller in integer milliseconds.  No wall-clock call
    appears here, keeping fake-transport tests and replay byte-for-byte stable.
    """

    def __init__(
        self,
        transport: ByteTransport,
        config: SessionConfig | None = None,
    ) -> None:
        self.transport = transport
        self.config = config or SessionConfig()
        self.state = SessionState.DISCONNECTED
        self.stats = SessionStats()
        self.parser = StreamParser()
        self.rx_sequence = SequenceTracker()
        self._tx_sequence = 0
        self._tx_buffer = bytearray()
        self._last_rx_ms: int | None = None
        self._last_now_ms: int | None = None
        self._peer_capabilities: int | None = None
        self._negotiated_max_payload: int | None = None
        self._negotiated_timeout_ms: int | None = None

    @property
    def active(self) -> bool:
        return self.state is SessionState.ACTIVE

    @property
    def pending_tx_bytes(self) -> int:
        return len(self._tx_buffer)

    @property
    def peer_capabilities(self) -> int | None:
        return self._peer_capabilities

    @property
    def negotiated_max_payload(self) -> int | None:
        return self._negotiated_max_payload

    @property
    def negotiated_timeout_ms(self) -> int | None:
        """Effective receive timeout after HELLO_ACK.

        The stricter of the locally requested timeout and the controller's
        nonzero advertised timeout is used.  Before activation, the local
        timeout remains the HELLO response deadline.
        """

        return self._negotiated_timeout_ms

    def start(self, now_ms: int = 0) -> None:
        self._validate_now(now_ms)
        if self.state is SessionState.CLOSED:
            raise SessionError("closed session cannot be restarted")
        self.parser.reset()
        self.rx_sequence.reset()
        self._tx_buffer.clear()
        self._tx_sequence = 0
        self._last_rx_ms = now_ms
        self._peer_capabilities = None
        self._negotiated_max_payload = None
        self._negotiated_timeout_ms = None
        hello = HelloPayload(
            role=EndpointRole.HOST,
            capabilities=SUPPORTED_CAPABILITIES,
            heartbeat_timeout_ms=self.config.timeout_ms,
        )
        self._send(MessageType.HELLO, 0, hello)
        self.state = SessionState.HELLO_SENT

    def reconnect(self, now_ms: int) -> None:
        if self.state is SessionState.CLOSED:
            raise SessionError("closed session cannot reconnect")
        self.stats.reconnects += 1
        self._tx_buffer.clear()
        self.start(now_ms)

    def send_sensor(self, step_id: int, payload: SensorFramePayload) -> None:
        self._require_active()
        self._send(MessageType.SENSOR_FRAME, step_id, payload)

    def send_heartbeat(
        self,
        *,
        step_id: int,
        uptime_ms: int,
        monotonic_ms: int,
        runtime_state: RuntimeState,
    ) -> None:
        """Send a typed host heartbeat on an active Protocol v1 session."""

        self._require_active()
        self._send(
            MessageType.HEARTBEAT,
            step_id,
            HeartbeatPayload(
                uptime_ms=uptime_ms,
                monotonic_ms=monotonic_ms,
                runtime_state=runtime_state,
            ),
        )

    def send_route(
        self,
        *,
        route_id: int,
        points: tuple[RouteWaypoint, ...],
        step_id: int = 0,
        loop_route: bool = False,
    ) -> None:
        self._require_active()
        if not points:
            raise ValueError("route must contain at least one waypoint")
        if len(points) > 0xFFFF:
            raise ValueError("route cannot exceed 65535 waypoints")
        negotiated_limit = self._negotiated_max_payload
        if negotiated_limit is None:
            raise SessionError("session payload limit has not been negotiated")
        points_per_chunk = min(
            MAX_ROUTE_POINTS_PER_CHUNK,
            (negotiated_limit - ROUTE_PREFIX_PAYLOAD_SIZE) // 16,
        )
        if points_per_chunk <= 0:
            raise SessionError("negotiated payload limit cannot carry a route point")
        for start in range(0, len(points), points_per_chunk):
            chunk = points[start : start + points_per_chunk]
            flags = RouteFlag(0)
            if start == 0:
                flags |= RouteFlag.CLEAR_EXISTING
            if start + len(chunk) == len(points):
                flags |= RouteFlag.FINAL_CHUNK
            if loop_route:
                flags |= RouteFlag.LOOP_ROUTE
            self._send(
                MessageType.ROUTE_CHUNK,
                step_id,
                RouteChunkPayload(
                    route_id=route_id,
                    start_index=start,
                    total_count=len(points),
                    flags=flags,
                    points=chunk,
                ),
            )

    def request_safe_stop(
        self,
        *,
        reason: SafeStopReason = SafeStopReason.MANUAL,
        detail: int = 0,
        step_id: int = 0,
    ) -> None:
        if self.state in (SessionState.CLOSED, SessionState.ERROR):
            raise SessionError("session cannot send safe-stop")
        self._send(
            MessageType.SAFE_STOP,
            step_id,
            SafeStopPayload(reason=reason, latch=True, detail=detail),
        )
        self.state = SessionState.SAFE_STOP

    def poll(self, now_ms: int) -> tuple[SessionEvent, ...]:
        self._validate_now(now_ms)
        if self.state is SessionState.CLOSED:
            return ()
        self._flush_tx()

        events: list[SessionEvent] = []
        for _ in range(self.config.max_reads_per_poll):
            try:
                data = self.transport.read()
            except TransportError:
                self.stats.transport_errors += 1
                self.state = SessionState.ERROR
                raise
            if not data:
                break
            self.stats.rx_bytes += len(data)
            batch = self.parser.feed(data)
            self.stats.rx_rejected += len(batch.errors)
            for packet in batch.packets:
                if packet.message_type in (
                    MessageType.CONTROL_COMMAND,
                    MessageType.STATE_ESTIMATE,
                ):
                    sequence = self.rx_sequence.observe(
                        packet.sequence,
                        packet.step_id,
                    )
                else:
                    sequence = self.rx_sequence.observe(packet.sequence)
                if not sequence.accepted:
                    self.stats.rx_rejected += 1
                    continue
                try:
                    payload = decode_packet_payload(packet)
                except ProtocolError:
                    self.stats.rx_payload_errors += 1
                    self.stats.rx_rejected += 1
                    if packet.message_type is MessageType.HELLO_ACK:
                        self.state = SessionState.ERROR
                    continue
                self.stats.rx_packets += 1
                self._last_rx_ms = now_ms
                self._handle_session_packet(packet.message_type, payload)
                events.append(
                    SessionEvent(packet, payload, sequence.disposition)
                )

        if (
            self.state in (SessionState.HELLO_SENT, SessionState.ACTIVE)
            and self._last_rx_ms is not None
            and now_ms - self._last_rx_ms > self._current_timeout_ms()
        ):
            self.stats.timeouts += 1
            self.state = SessionState.TIMED_OUT
        return tuple(events)

    def close(self) -> None:
        self.transport.close()
        self._tx_buffer.clear()
        self.state = SessionState.CLOSED

    def _send(
        self,
        message_type: MessageType,
        step_id: int,
        payload: TypedPayload,
    ) -> None:
        packet = make_packet(
            message_type,
            self._tx_sequence,
            step_id,
            payload,
        )
        if (
            message_type is not MessageType.HELLO
            and self._negotiated_max_payload is not None
            and len(packet.payload) > self._negotiated_max_payload
        ):
            raise SessionError(
                "payload exceeds the controller's negotiated maximum"
            )
        frame = encode_packet(packet)
        if len(self._tx_buffer) + len(frame) > self.config.max_tx_buffer:
            raise SessionError("session transmit buffer overflow")
        self._tx_buffer.extend(frame)
        self._tx_sequence = (self._tx_sequence + 1) & 0xFFFFFFFF
        self.stats.tx_packets += 1
        self._flush_tx()

    def _flush_tx(self) -> None:
        while self._tx_buffer:
            try:
                written = self.transport.write(bytes(self._tx_buffer))
            except TransportError:
                self.stats.transport_errors += 1
                self.state = SessionState.ERROR
                raise
            if written < 0 or written > len(self._tx_buffer):
                self.state = SessionState.ERROR
                raise SessionError("transport returned an invalid write count")
            if written == 0:
                break
            del self._tx_buffer[:written]
            self.stats.tx_bytes += written

    def _handle_session_packet(
        self,
        message_type: MessageType,
        payload: TypedPayload,
    ) -> None:
        if message_type is MessageType.HELLO_ACK:
            if not isinstance(payload, HelloAckPayload):
                self.state = SessionState.ERROR
                return
            required = self.config.required_capabilities
            minimum_payload = self._minimum_peer_payload(required)
            if not (
                self.state is SessionState.HELLO_SENT
                and payload.status is HelloStatus.OK
                and payload.accepted_version == 1
                and payload.role is EndpointRole.CONTROLLER
                and payload.capabilities & required == required
                and minimum_payload <= payload.max_payload <= MAX_PAYLOAD_SIZE
                and payload.heartbeat_timeout_ms > 0
            ):
                self.state = SessionState.ERROR
                return
            self._peer_capabilities = payload.capabilities
            self._negotiated_max_payload = payload.max_payload
            self._negotiated_timeout_ms = min(
                self.config.timeout_ms,
                payload.heartbeat_timeout_ms,
            )
            self.state = SessionState.ACTIVE

    def _require_active(self) -> None:
        if self.state is not SessionState.ACTIVE:
            raise SessionError("session handshake is not active")

    def _validate_now(self, now_ms: int) -> None:
        if isinstance(now_ms, bool) or not isinstance(now_ms, int):
            raise TypeError("now_ms must be an integer")
        if now_ms < 0:
            raise ValueError("now_ms cannot be negative")
        if self._last_now_ms is not None and now_ms < self._last_now_ms:
            raise ValueError("now_ms must be monotonic")
        self._last_now_ms = now_ms

    def _current_timeout_ms(self) -> int:
        if self.state is SessionState.ACTIVE:
            negotiated = self._negotiated_timeout_ms
            if negotiated is None:
                raise SessionError("active session has no negotiated timeout")
            return negotiated
        return self.config.timeout_ms

    @staticmethod
    def _minimum_peer_payload(required_capabilities: int) -> int:
        minimum = 1
        if required_capabilities & int(Capability.SENSOR_FRAME):
            minimum = max(minimum, SENSOR_FRAME_PAYLOAD_SIZE)
        if required_capabilities & int(Capability.ROUTE_CHUNK):
            minimum = max(minimum, ROUTE_PREFIX_PAYLOAD_SIZE + 16)
        return minimum
