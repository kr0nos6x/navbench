from __future__ import annotations

import unittest

from navbench.transport import (
    DeterministicFaultTransport,
    InMemoryEndpoint,
    LinkFaultState,
    SerialConfig,
    TransportError,
)


class InMemoryTransportTests(unittest.TestCase):
    def test_fragmented_reads_preserve_every_byte(self) -> None:
        sender, receiver = InMemoryEndpoint.pair(second_read_limit=3)
        self.assertEqual(sender.write(b"abcdefgh"), 8)
        self.assertEqual(receiver.read(), b"abc")
        self.assertEqual(receiver.read(), b"def")
        self.assertEqual(receiver.read(), b"gh")
        self.assertEqual(receiver.read(), b"")

    def test_combined_writes_can_be_read_as_one_chunk(self) -> None:
        sender, receiver = InMemoryEndpoint.pair()
        sender.write(b"first")
        sender.write(b"second")
        self.assertEqual(receiver.pending_bytes, 11)
        self.assertEqual(receiver.read(), b"firstsecond")

    def test_injection_and_disconnect_are_explicit(self) -> None:
        sender, receiver = InMemoryEndpoint.pair()
        receiver.inject(b"corrupt")
        self.assertEqual(receiver.read(), b"corrupt")
        receiver.close()
        with self.assertRaises(TransportError):
            sender.write(b"data")

    def test_invalid_limits_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            InMemoryEndpoint(read_limit=0)
        endpoint, _ = InMemoryEndpoint.pair()
        with self.assertRaises(ValueError):
            endpoint.read(0)


class SerialConfigurationTests(unittest.TestCase):
    def test_supported_configuration_does_not_open_device(self) -> None:
        config = SerialConfig("/dev/never-opened-by-this-test", 115200)
        self.assertEqual(config.baud, 115200)

    def test_invalid_configuration_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            SerialConfig("")
        with self.assertRaises(ValueError):
            SerialConfig("/dev/example", 12345)


class FaultTransportTests(unittest.TestCase):
    def setUp(self) -> None:
        inner, self.receiver = InMemoryEndpoint.pair()
        self.transport = DeterministicFaultTransport(inner)

    def test_loss_corruption_delay_stale_and_disconnect_are_counted(self) -> None:
        self.transport.configure(0, LinkFaultState())
        self.transport.write(b"\x02first\x00")
        self.assertEqual(self.receiver.read(), b"\x02first\x00")

        self.transport.configure(1, LinkFaultState(packet_loss=True))
        self.transport.write(b"\x02lost\x00")
        self.assertEqual(self.receiver.read(), b"")

        self.transport.configure(2, LinkFaultState(packet_corruption=True))
        original = b"\x02corrupt\x00"
        self.transport.write(original)
        self.assertNotEqual(self.receiver.read(), original)

        self.transport.configure(3, LinkFaultState(delay_steps=2))
        self.transport.write(b"\x02delayed\x00")
        self.assertEqual(self.receiver.read(), b"")
        self.transport.configure(4, LinkFaultState())
        self.assertEqual(self.receiver.read(), b"")
        self.transport.configure(5, LinkFaultState())
        self.assertEqual(self.receiver.read(), b"\x02delayed\x00")

        self.transport.write(b"\x02fresh\x00")
        self.receiver.read()
        self.transport.configure(6, LinkFaultState(stale_frame=True))
        self.transport.write(b"\x02new\x00")
        self.assertEqual(self.receiver.read(), b"\x02fresh\x00")

        self.transport.configure(7, LinkFaultState(disconnected=True))
        self.transport.write(b"\x02disconnect\x00")
        self.assertEqual(self.receiver.read(), b"")
        self.assertEqual(self.transport.stats.frames_dropped, 1)
        self.assertEqual(self.transport.stats.frames_corrupted, 1)
        self.assertEqual(self.transport.stats.frames_delayed, 1)
        self.assertEqual(self.transport.stats.stale_frames_replayed, 1)
        self.assertEqual(self.transport.stats.disconnect_drops, 1)

    def test_write_faults_are_applied_per_complete_cobs_frame(self) -> None:
        first = b"\x03a1\x00"
        second = b"\x03b2\x00"
        self.transport.configure(0, LinkFaultState(packet_loss=True))
        self.transport.write(first[:2])
        self.assertEqual(self.receiver.read(), b"")
        self.transport.write(first[2:] + second)
        self.assertEqual(self.receiver.read(), b"")
        self.assertEqual(self.transport.stats.frames_dropped, 2)

        self.transport.configure(1, LinkFaultState())
        self.transport.write(first + second)
        self.assertEqual(self.receiver.read(), first + second)
        self.assertEqual(self.transport.stats.frames_forwarded, 2)
        self.assertEqual(self.transport.stats.tx_frames_forwarded, 2)

    def test_read_direction_faults_are_frame_aware_and_counted(self) -> None:
        inner, controller = InMemoryEndpoint.pair(first_read_limit=2)
        transport = DeterministicFaultTransport(inner)
        healthy = LinkFaultState()

        transport.configure(0, healthy, healthy)
        first = b"\x03a1\x00"
        second = b"\x03b2\x00"
        controller.write(first + second)
        received = bytearray()
        while True:
            chunk = transport.read(3)
            if not chunk:
                break
            received.extend(chunk)
        self.assertEqual(bytes(received), first + second)
        self.assertEqual(transport.stats.rx_frames_forwarded, 2)

        transport.configure(1, healthy, LinkFaultState(packet_loss=True))
        controller.write(b"\x03l1\x00\x03l2\x00")
        self.assertEqual(transport.read(), b"")
        self.assertEqual(transport.stats.rx_frames_dropped, 2)

        original = b"\x06corpt\x00"
        transport.configure(
            2,
            healthy,
            LinkFaultState(packet_corruption=True),
        )
        controller.write(original)
        corrupted = transport.read()
        self.assertNotEqual(corrupted, original)
        self.assertTrue(corrupted.endswith(b"\x00"))

        delayed = b"\x06delay\x00"
        transport.configure(3, healthy, LinkFaultState(delay_steps=2))
        controller.write(delayed)
        self.assertEqual(transport.read(), b"")
        transport.configure(4, healthy, healthy)
        self.assertEqual(transport.read(), b"")
        transport.configure(5, healthy, healthy)
        self.assertEqual(transport.read(), delayed)

        fresh = b"\x06fresh\x00"
        controller.write(fresh)
        self.assertEqual(transport.read(), fresh)
        transport.configure(6, healthy, LinkFaultState(stale_frame=True))
        controller.write(b"\x04new\x00")
        self.assertEqual(transport.read(), fresh)

        transport.configure(7, healthy, LinkFaultState(disconnected=True))
        controller.write(b"\x05drop\x00")
        self.assertEqual(transport.read(), b"")
        self.assertEqual(transport.stats.rx_frames_corrupted, 1)
        self.assertEqual(transport.stats.rx_frames_delayed, 1)
        self.assertEqual(transport.stats.rx_stale_frames_replayed, 1)
        self.assertEqual(transport.stats.rx_disconnect_drops, 1)


if __name__ == "__main__":
    unittest.main()
