from __future__ import annotations

import struct
import unittest

from navbench.protocol import (
    MAX_PAYLOAD_SIZE,
    Capability,
    EndpointRole,
    HeartbeatPayload,
    HelloAckPayload,
    HelloStatus,
    MessageType,
    Packet,
    RouteWaypoint,
    RuntimeState,
    StreamParser,
    decode_packet_payload,
    encode_packet,
    make_packet,
)
from navbench.session import (
    DEFAULT_REQUIRED_CAPABILITIES,
    HostSession,
    SENSOR_FRAME_PAYLOAD_SIZE,
    SessionConfig,
    SessionError,
    SessionState,
)
from navbench.transport import InMemoryEndpoint


def activate_session(
    *, read_limit: int | None = None
) -> tuple[HostSession, InMemoryEndpoint]:
    host_transport, controller_transport = InMemoryEndpoint.pair(
        first_read_limit=read_limit,
        second_read_limit=read_limit,
    )
    session = HostSession(host_transport, SessionConfig(timeout_ms=100))
    session.start(0)

    parser = StreamParser()
    hello_packets = []
    while controller_transport.pending_bytes:
        hello_packets.extend(
            parser.feed(controller_transport.read(512)).packets
        )
    assert len(hello_packets) == 1
    hello = decode_packet_payload(hello_packets[0])
    assert hello.role is EndpointRole.HOST

    ack = make_packet(
        MessageType.HELLO_ACK,
        sequence=0,
        step_id=0,
        payload=HelloAckPayload(
            accepted_version=1,
            status=HelloStatus.OK,
            role=EndpointRole.CONTROLLER,
            capabilities=DEFAULT_REQUIRED_CAPABILITIES,
        ),
    )
    controller_transport.write(encode_packet(ack))
    session.poll(1)
    assert session.state is SessionState.ACTIVE
    return session, controller_transport


class SessionTests(unittest.TestCase):
    def test_fragmented_handshake_becomes_active(self) -> None:
        session, _ = activate_session(read_limit=3)
        self.assertTrue(session.active)
        self.assertEqual(session.stats.rx_packets, 1)

    def test_typed_heartbeat_requires_active_session_and_uses_sequence(self) -> None:
        host, _ = InMemoryEndpoint.pair()
        inactive = HostSession(host)
        with self.assertRaises(SessionError):
            inactive.send_heartbeat(
                step_id=7,
                uptime_ms=123,
                monotonic_ms=456,
                runtime_state=RuntimeState.READY,
            )

        session, controller = activate_session()
        session.send_heartbeat(
            step_id=7,
            uptime_ms=123,
            monotonic_ms=456,
            runtime_state=RuntimeState.RUNNING,
        )
        packets = StreamParser().feed(controller.read(512)).packets
        self.assertEqual(len(packets), 1)
        self.assertEqual(packets[0].message_type, MessageType.HEARTBEAT)
        self.assertEqual(packets[0].sequence, 1)
        self.assertEqual(packets[0].step_id, 7)
        self.assertEqual(
            decode_packet_payload(packets[0]),
            HeartbeatPayload(123, 456, RuntimeState.RUNNING),
        )

    def test_route_is_chunked_without_exceeding_protocol_limit(self) -> None:
        session, controller = activate_session()
        points = tuple(
            RouteWaypoint(float(index), 0.0, 1.0, 0.5)
            for index in range(12)
        )
        session.send_route(route_id=7, points=points)
        parser = StreamParser()
        batch = parser.feed(controller.read(4096))
        self.assertEqual(
            [packet.message_type for packet in batch.packets],
            [
                MessageType.ROUTE_CHUNK,
                MessageType.ROUTE_CHUNK,
                MessageType.ROUTE_CHUNK,
            ],
        )
        chunks = [decode_packet_payload(packet) for packet in batch.packets]
        self.assertEqual([len(chunk.points) for chunk in chunks], [5, 5, 2])

    def test_duplicate_packet_is_rejected(self) -> None:
        session, controller = activate_session()
        heartbeat = encode_packet(
            make_packet(
                MessageType.HEARTBEAT,
                sequence=1,
                step_id=1,
                payload=HeartbeatPayload(1, 1, RuntimeState.RUNNING),
            )
        )
        controller.write(heartbeat + heartbeat)
        events = session.poll(2)
        self.assertEqual(len(events), 1)
        self.assertEqual(session.rx_sequence.stats.duplicates, 1)

    def test_timeout_uses_caller_supplied_monotonic_time(self) -> None:
        session, _ = activate_session()
        session.poll(101)
        self.assertEqual(session.state, SessionState.ACTIVE)
        session.poll(102)
        self.assertEqual(session.state, SessionState.TIMED_OUT)
        self.assertEqual(session.stats.timeouts, 1)

    def test_ack_requires_controller_role_and_required_capabilities(self) -> None:
        cases = (
            (
                EndpointRole.HOST,
                DEFAULT_REQUIRED_CAPABILITIES,
            ),
            (
                EndpointRole.CONTROLLER,
                DEFAULT_REQUIRED_CAPABILITIES
                & ~int(Capability.SAFE_STOP),
            ),
        )
        for role, capabilities in cases:
            with self.subTest(role=role, capabilities=capabilities):
                host, controller = InMemoryEndpoint.pair()
                session = HostSession(host)
                session.start(0)
                controller.read(512)
                controller.write(
                    encode_packet(
                        make_packet(
                            MessageType.HELLO_ACK,
                            sequence=0,
                            step_id=0,
                            payload=HelloAckPayload(
                                accepted_version=1,
                                status=HelloStatus.OK,
                                role=role,
                                capabilities=capabilities,
                            ),
                        )
                    )
                )
                session.poll(1)
                self.assertEqual(session.state, SessionState.ERROR)

    def test_ack_rejects_small_payload_and_zero_timeout(self) -> None:
        host, controller = InMemoryEndpoint.pair()
        session = HostSession(host)
        session.start(0)
        controller.read(512)
        controller.write(
            encode_packet(
                make_packet(
                    MessageType.HELLO_ACK,
                    sequence=0,
                    step_id=0,
                    payload=HelloAckPayload(
                        accepted_version=1,
                        status=HelloStatus.OK,
                        role=EndpointRole.CONTROLLER,
                        capabilities=DEFAULT_REQUIRED_CAPABILITIES,
                        max_payload=SENSOR_FRAME_PAYLOAD_SIZE - 1,
                    ),
                )
            )
        )
        session.poll(1)
        self.assertEqual(session.state, SessionState.ERROR)

        host, controller = InMemoryEndpoint.pair()
        session = HostSession(host)
        session.start(0)
        controller.read(512)
        raw_ack = struct.pack(
            "<BBBBIHH",
            1,
            int(HelloStatus.OK),
            int(EndpointRole.CONTROLLER),
            0,
            DEFAULT_REQUIRED_CAPABILITIES,
            MAX_PAYLOAD_SIZE,
            0,
        )
        controller.write(
            encode_packet(
                Packet(MessageType.HELLO_ACK, 0, 0, raw_ack)
            )
        )
        session.poll(1)
        self.assertEqual(session.state, SessionState.ERROR)
        self.assertEqual(session.stats.rx_payload_errors, 1)

    def test_negotiated_payload_and_timeout_are_enforced(self) -> None:
        host, controller = InMemoryEndpoint.pair()
        session = HostSession(host, SessionConfig(timeout_ms=100))
        session.start(0)
        controller.read(512)
        controller.write(
            encode_packet(
                make_packet(
                    MessageType.HELLO_ACK,
                    sequence=0,
                    step_id=0,
                    payload=HelloAckPayload(
                        accepted_version=1,
                        status=HelloStatus.OK,
                        role=EndpointRole.CONTROLLER,
                        capabilities=DEFAULT_REQUIRED_CAPABILITIES,
                        max_payload=SENSOR_FRAME_PAYLOAD_SIZE,
                        heartbeat_timeout_ms=40,
                    ),
                )
            )
        )
        session.poll(1)
        self.assertTrue(session.active)
        self.assertEqual(
            session.peer_capabilities,
            DEFAULT_REQUIRED_CAPABILITIES,
        )
        self.assertEqual(
            session.negotiated_max_payload,
            SENSOR_FRAME_PAYLOAD_SIZE,
        )
        self.assertEqual(session.negotiated_timeout_ms, 40)

        session.send_route(
            route_id=3,
            points=tuple(
                RouteWaypoint(float(index), 0.0, 1.0, 0.5)
                for index in range(12)
            ),
        )
        packets = StreamParser().feed(controller.read(4096)).packets
        chunks = [decode_packet_payload(packet) for packet in packets]
        self.assertEqual([len(chunk.points) for chunk in chunks], [4, 4, 4])
        self.assertTrue(
            all(len(packet.payload) <= SENSOR_FRAME_PAYLOAD_SIZE for packet in packets)
        )

        session.poll(41)
        self.assertEqual(session.state, SessionState.ACTIVE)
        session.poll(42)
        self.assertEqual(session.state, SessionState.TIMED_OUT)

    def test_time_cannot_move_backwards(self) -> None:
        session, _ = activate_session()
        session.poll(5)
        with self.assertRaises(ValueError):
            session.poll(4)

    def test_reconnect_resets_both_session_sequence_streams(self) -> None:
        session, controller = activate_session()
        session.send_route(
            route_id=1,
            points=(RouteWaypoint(1.0, 0.0, 0.0, 0.5),),
        )
        first_session = StreamParser().feed(controller.read(4096)).packets
        self.assertEqual(first_session[0].sequence, 1)

        session.reconnect(10)
        reconnect_packets = StreamParser().feed(controller.read(4096)).packets
        self.assertEqual(len(reconnect_packets), 1)
        self.assertEqual(reconnect_packets[0].message_type, MessageType.HELLO)
        self.assertEqual(reconnect_packets[0].sequence, 0)
        controller.write(
            encode_packet(
                make_packet(
                    MessageType.HELLO_ACK,
                    sequence=0,
                    step_id=0,
                    payload=HelloAckPayload(
                        accepted_version=1,
                        status=HelloStatus.OK,
                        role=EndpointRole.CONTROLLER,
                        capabilities=DEFAULT_REQUIRED_CAPABILITIES,
                    ),
                )
            )
        )
        session.poll(11)
        self.assertEqual(session.state, SessionState.ACTIVE)
        self.assertEqual(session.stats.reconnects, 1)


if __name__ == "__main__":
    unittest.main()
