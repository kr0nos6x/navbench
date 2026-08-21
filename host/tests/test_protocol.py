from __future__ import annotations

import unittest
from pathlib import Path

from navbench.protocol import (
    MAX_PAYLOAD_SIZE,
    MAX_WIRE_FRAME_SIZE,
    MessageType,
    Packet,
    ProtocolError,
    ProtocolErrorCode,
    SequenceDisposition,
    SequenceTracker,
    StreamParser,
    crc16_ccitt,
    decode_packet,
    decode_packet_payload,
    decode_typed_payload,
    encode_packet,
    encode_typed_payload,
)


ROOT = Path(__file__).resolve().parents[2]
GOLDEN_PATH = ROOT / "test" / "fixtures" / "protocol_v1_golden.tsv"
INVALID_PATH = ROOT / "test" / "fixtures" / "protocol_v1_invalid.tsv"


def _records(path: Path) -> list[list[str]]:
    return [
        line.split("\t")
        for line in path.read_text(encoding="ascii").splitlines()
        if line and not line.startswith("#")
    ]


def _hex(value: str) -> bytes:
    return b"" if value == "-" else bytes.fromhex(value)


class ProtocolGoldenVectorTests(unittest.TestCase):
    def test_python_codec_matches_all_shared_golden_vectors(self) -> None:
        records = _records(GOLDEN_PATH)
        self.assertEqual(len(records), 13)
        for name, type_text, sequence_text, step_text, payload_hex, frame_hex in records:
            with self.subTest(name=name):
                packet = Packet(
                    MessageType(int(type_text)),
                    int(sequence_text),
                    int(step_text),
                    _hex(payload_hex),
                )
                frame = _hex(frame_hex)
                self.assertEqual(encode_packet(packet), frame)
                self.assertEqual(decode_packet(frame), packet)
                if name not in {
                    "empty_payload",
                    "arbitrary_zero_payload",
                    "maximum_payload",
                }:
                    typed = decode_packet_payload(packet)
                    self.assertEqual(
                        encode_typed_payload(packet.message_type, typed),
                        packet.payload,
                    )
        maximum = next(row for row in records if row[0] == "maximum_payload")
        self.assertEqual(len(_hex(maximum[4])), MAX_PAYLOAD_SIZE)
        self.assertEqual(len(_hex(maximum[5])), MAX_WIRE_FRAME_SIZE)

    def test_all_shared_rejection_vectors_have_the_expected_error(self) -> None:
        records = _records(INVALID_PATH)
        self.assertEqual(len(records), 10)
        for name, error_name, frame_hex in records:
            with self.subTest(name=name):
                with self.assertRaises(ProtocolError) as captured:
                    decode_packet(_hex(frame_hex))
                self.assertEqual(captured.exception.code, ProtocolErrorCode[error_name])

    def test_fragmented_and_combined_stream_reads(self) -> None:
        frames = [_hex(row[5]) for row in _records(GOLDEN_PATH)]
        combined = b"".join(frames)
        parser = StreamParser()
        packets = []
        errors = []
        sizes = (1, 2, 7, 3, 19, 5)
        offset = 0
        chunk = 0
        while offset < len(combined):
            size = min(sizes[chunk % len(sizes)], len(combined) - offset)
            batch = parser.feed(combined[offset : offset + size])
            packets.extend(batch.packets)
            errors.extend(batch.errors)
            offset += size
            chunk += 1
        self.assertEqual(errors, [])
        self.assertEqual(len(packets), len(frames))
        self.assertEqual(parser.stats.packets_accepted, len(frames))
        self.assertFalse(parser.finish().errors)

        parser.reset(clear_stats=True)
        parser.feed(frames[0][:-1])
        finished = parser.finish()
        self.assertEqual(finished.errors[0].code, ProtocolErrorCode.TRUNCATED_FRAME)
        self.assertEqual(parser.stats.truncated_frames, 1)

    def test_crc_sequence_and_one_thousand_frame_soak(self) -> None:
        self.assertEqual(crc16_ccitt(b"123456789"), 0x29B1)
        parser = StreamParser()
        combined = bytearray()
        expected: list[Packet] = []
        for index in range(1000):
            payload = bytes((index & 0xFF, 0, (index >> 8) & 0xFF))
            packet = Packet(MessageType.ERROR, index, index * 2, payload)
            expected.append(packet)
            combined.extend(encode_packet(packet))
        actual = []
        for offset in range(0, len(combined), 37):
            batch = parser.feed(combined[offset : offset + 37])
            self.assertFalse(batch.errors)
            actual.extend(batch.packets)
        self.assertEqual(actual, expected)
        self.assertEqual(parser.stats.packets_accepted, 1000)

        tracker = SequenceTracker(32)
        self.assertEqual(tracker.observe(0xFFFFFFFE).disposition, SequenceDisposition.FIRST)
        self.assertEqual(tracker.observe(0xFFFFFFFF).disposition, SequenceDisposition.IN_ORDER)
        self.assertEqual(tracker.observe(0).disposition, SequenceDisposition.IN_ORDER)
        self.assertEqual(tracker.observe(0).disposition, SequenceDisposition.DUPLICATE)
        self.assertEqual(tracker.observe(0xFFFFFFFF).disposition, SequenceDisposition.OUT_OF_ORDER)
        self.assertEqual(tracker.observe(0xFFFFFF00).disposition, SequenceDisposition.STALE)
        self.assertEqual(tracker.observe(3).missing, 2)
        self.assertTrue(tracker.observe(4, 100).accepted)
        self.assertTrue(tracker.observe(5).accepted)
        self.assertEqual(tracker.observe(6, 99).disposition, SequenceDisposition.STALE)

    def test_nonfinite_typed_values_and_oversized_payload_are_rejected(self) -> None:
        from navbench.protocol import HealthStatusPayload, NavigationMode, RuntimeState

        with self.assertRaises(ProtocolError) as captured:
            encode_typed_payload(
                MessageType.HEALTH_STATUS,
                HealthStatusPayload(
                    runtime_state=RuntimeState.RUNNING,
                    navigation_mode=NavigationMode.GNSS_AIDED,
                    flags=0,
                    uptime_ms=0,
                    last_sensor_age_ms=0,
                    rx_frames=0,
                    rx_crc_errors=0,
                    rx_decode_errors=0,
                    rx_missing=0,
                    rx_duplicates=0,
                    rx_out_of_order=0,
                    rx_stale=0,
                    queue_overflows=0,
                    scheduler_overruns=0,
                    max_loop_us=0,
                    imu_yaw_nis_evaluated_count=0,
                    imu_yaw_nis_gate_rejected_count=0,
                    imu_yaw_nis_sum=0.0,
                    imu_yaw_nis_max=0.0,
                    wheel_nis_evaluated_count=0,
                    wheel_nis_gate_rejected_count=0,
                    wheel_nis_sum=0.0,
                    wheel_nis_max=0.0,
                    gnss_nis_evaluated_count=1,
                    gnss_nis_gate_rejected_count=0,
                    gnss_nis_sum=float("nan"),
                    gnss_nis_max=0.0,
                    landmark_nis_evaluated_count=0,
                    landmark_nis_gate_rejected_count=0,
                    landmark_nis_sum=0.0,
                    landmark_nis_max=0.0,
                ),
            )
        self.assertEqual(captured.exception.code, ProtocolErrorCode.INVALID_VALUE)
        with self.assertRaises(ProtocolError) as captured:
            encode_packet(
                Packet(MessageType.ERROR, 0, 0, bytes(MAX_PAYLOAD_SIZE + 1))
            )
        self.assertEqual(captured.exception.code, ProtocolErrorCode.PAYLOAD_TOO_LARGE)
        with self.assertRaises(ProtocolError):
            decode_typed_payload(MessageType.SAFE_STOP, b"\x01\x02\x00\x00")


if __name__ == "__main__":
    unittest.main()
