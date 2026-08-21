from __future__ import annotations

import contextlib
import io
import sys
import unittest
from unittest.mock import patch

from navbench.__main__ import main
from navbench.hardware import (
    HardwareConfig,
    HardwareExitCode,
    run_physical_validation,
    run_serial_diagnostic,
)
from navbench.protocol import (
    ApplicationErrorCode,
    ControlCommandPayload,
    ControlMode,
    EndpointRole,
    ErrorPayload,
    GnssSample,
    HealthStatusPayload,
    HelloAckPayload,
    HelloStatus,
    MessageType,
    NavigationMode,
    RouteChunkPayload,
    RuntimeState,
    SensorFramePayload,
    StateEstimatePayload,
    StreamParser,
    decode_packet_payload,
    encode_packet,
    make_packet,
)
from navbench.session import DEFAULT_REQUIRED_CAPABILITIES
from navbench.transport import SerialConfig, TransportError


class FakeClock:
    def __init__(self) -> None:
        self.value = 0.0

    def __call__(self) -> float:
        return self.value

    def sleep(self, duration_s: float) -> None:
        if duration_s < 0.0:
            raise ValueError("sleep cannot move backwards")
        self.value += duration_s


class ScriptedControllerTransport:
    def __init__(
        self,
        clock: FakeClock,
        *,
        acknowledge: bool = True,
        normal_responses: bool = True,
        watchdog_response: bool = False,
        ack_delay_s: float = 0.0,
        normal_response_at: int = 1,
        normal_response_step: int | None = None,
    ) -> None:
        self.clock = clock
        self.acknowledge = acknowledge
        self.normal_responses = normal_responses
        self.watchdog_response = watchdog_response
        self.ack_delay_s = ack_delay_s
        self.normal_response_at = normal_response_at
        self.normal_response_step = normal_response_step
        self._open = True
        self._parser = StreamParser()
        self._received = bytearray()
        self._tx_sequence = 0
        self._normal_sent = False
        self._watchdog_sent = False
        self._pending_ack: tuple[float, int] | None = None
        self._ack_sent = False
        self.last_sensor_at: float | None = None
        self.first_write_at: float | None = None
        self.input_reset_count = 0
        self.sensor_payloads: list[SensorFramePayload] = []
        self.route_chunks: list[RouteChunkPayload] = []

    @property
    def is_open(self) -> bool:
        return self._open

    def write(self, data: bytes) -> int:
        if self.first_write_at is None:
            self.first_write_at = self.clock()
        for packet in self._parser.feed(data).packets:
            payload = decode_packet_payload(packet)
            if (
                packet.message_type is MessageType.HELLO
                and self.acknowledge
                and not self._ack_sent
                and self._pending_ack is None
            ):
                due_s = self.clock() + self.ack_delay_s
                if due_s <= self.clock():
                    self._queue_ack(packet.step_id)
                else:
                    self._pending_ack = (due_s, packet.step_id)
            elif isinstance(payload, RouteChunkPayload):
                self.route_chunks.append(payload)
            elif isinstance(payload, SensorFramePayload):
                self.sensor_payloads.append(payload)
                self.last_sensor_at = self.clock()
                if (
                    self.normal_responses
                    and not self._normal_sent
                    and len(self.sensor_payloads) == self.normal_response_at
                ):
                    self._normal_sent = True
                    self._queue_normal(
                        packet.step_id
                        if self.normal_response_step is None
                        else self.normal_response_step
                    )
        return len(data)

    def read(self, max_bytes: int = 512) -> bytes:
        if self._pending_ack is not None and self.clock() >= self._pending_ack[0]:
            _, step_id = self._pending_ack
            self._pending_ack = None
            self._queue_ack(step_id)
        if (
            self.watchdog_response
            and not self._watchdog_sent
            and self.last_sensor_at is not None
            and self.clock() - self.last_sensor_at >= 0.501
        ):
            self._watchdog_sent = True
            step_id = len(self.sensor_payloads)
            self._queue(
                MessageType.CONTROL_COMMAND,
                step_id,
                ControlCommandPayload(0.0, -1.0, 0.0, ControlMode.SAFE_STOP),
            )
            self._queue(
                MessageType.HEALTH_STATUS,
                step_id,
                _health(RuntimeState.SAFE_STOP),
            )
        output = bytes(self._received[:max_bytes])
        del self._received[:max_bytes]
        return output

    def close(self) -> None:
        self._open = False

    def reset_input_buffer(self) -> None:
        self.input_reset_count += 1
        self._received.clear()

    def _queue_ack(self, step_id: int) -> None:
        self._ack_sent = True
        self._tx_sequence = 0
        self._queue(
            MessageType.HELLO_ACK,
            step_id,
            HelloAckPayload(
                accepted_version=1,
                status=HelloStatus.OK,
                role=EndpointRole.CONTROLLER,
                capabilities=DEFAULT_REQUIRED_CAPABILITIES,
                max_payload=128,
                heartbeat_timeout_ms=500,
            ),
        )

    def _queue(self, message_type: MessageType, step_id: int, payload: object) -> None:
        packet = make_packet(message_type, self._tx_sequence, step_id, payload)
        self._tx_sequence = (self._tx_sequence + 1) & 0xFFFFFFFF
        self._received.extend(encode_packet(packet))

    def _queue_normal(self, step_id: int) -> None:
        self._queue(
            MessageType.CONTROL_COMMAND,
            step_id,
            ControlCommandPayload(0.0, 0.0, 0.0, ControlMode.TRACKING),
        )
        self._queue(
            MessageType.STATE_ESTIMATE,
            step_id,
            StateEstimatePayload(
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                (1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
                NavigationMode.GNSS_AIDED,
            ),
        )
        self._queue(
            MessageType.HEALTH_STATUS,
            step_id,
            _health(RuntimeState.RUNNING),
        )


class DiagnosticControllerTransport:
    def __init__(self, clock: FakeClock, *, emit_beacon: bool = True) -> None:
        self.clock = clock
        self.emit_beacon = emit_beacon
        self._open = True
        self._parser = StreamParser()
        self._received = bytearray()
        self._diagnostic_sequence = 0x80000000
        self.serial_rx_bytes = 0
        self.parser_frames = 0
        self.parser_errors = 0
        self.hello_result = 0
        self.hello_packets = 0
        self.response_frames = 0

    @property
    def is_open(self) -> bool:
        return self._open

    def write(self, data: bytes) -> int:
        self.serial_rx_bytes += len(data)
        if data == b"\x01\x00":
            self.parser_errors += 1
            return len(data)
        batch = self._parser.feed(data)
        self.parser_errors += len(batch.errors)
        self.parser_frames += len(batch.packets)
        for packet in batch.packets:
            if packet.message_type is MessageType.HELLO:
                self.hello_packets += 1
                self.hello_result = 1
                self.response_frames += 1
                self._queue_packet(
                    MessageType.HELLO_ACK,
                    0,
                    HelloAckPayload(
                        accepted_version=1,
                        status=HelloStatus.OK,
                        role=EndpointRole.CONTROLLER,
                        capabilities=DEFAULT_REQUIRED_CAPABILITIES,
                        max_payload=128,
                        heartbeat_timeout_ms=500,
                    ),
                )
        return len(data)

    def read(self, max_bytes: int = 512) -> bytes:
        if self.emit_beacon and not self._received:
            self._emit_diagnostics()
        output = bytes(self._received[:max_bytes])
        del self._received[:max_bytes]
        return output

    def reset_input_buffer(self) -> None:
        self._received.clear()

    def close(self) -> None:
        self._open = False

    def _emit_diagnostics(self) -> None:
        parser_word = (self.hello_result << 24) | self.hello_packets
        events = (
            ErrorPayload(
                ApplicationErrorCode.DIAGNOSTIC,
                1,
                0x4E424447,
                int(self.clock() * 1000),
            ),
            ErrorPayload(
                ApplicationErrorCode.DIAGNOSTIC,
                2,
                self.serial_rx_bytes,
                self.parser_frames,
            ),
            ErrorPayload(
                ApplicationErrorCode.DIAGNOSTIC,
                3,
                parser_word,
                self.parser_errors,
            ),
            ErrorPayload(
                ApplicationErrorCode.DIAGNOSTIC,
                4,
                (16 << 16) | 16,
                64,
            ),
            ErrorPayload(
                ApplicationErrorCode.DIAGNOSTIC,
                5,
                self.response_frames,
                0,
            ),
        )
        for payload in events:
            self._queue_packet(MessageType.ERROR, 0, payload)

    def _queue_packet(
        self,
        message_type: MessageType,
        step_id: int,
        payload: object,
    ) -> None:
        sequence = (
            0
            if message_type is MessageType.HELLO_ACK
            else self._diagnostic_sequence
        )
        if message_type is not MessageType.HELLO_ACK:
            self._diagnostic_sequence = (self._diagnostic_sequence + 1) & 0xFFFFFFFF
        self._received.extend(
            encode_packet(make_packet(message_type, sequence, step_id, payload))
        )


def _health(state: RuntimeState) -> HealthStatusPayload:
    return HealthStatusPayload(
        runtime_state=state,
        navigation_mode=NavigationMode.GNSS_AIDED,
        flags=1,
        uptime_ms=100,
        last_sensor_age_ms=0,
        rx_frames=1,
        rx_crc_errors=0,
        rx_decode_errors=0,
        rx_missing=0,
        rx_duplicates=0,
        rx_out_of_order=0,
        rx_stale=0,
        queue_overflows=0,
        scheduler_overruns=0,
        max_loop_us=100,
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
        gnss_nis_sum=0.0,
        gnss_nis_max=0.0,
        landmark_nis_evaluated_count=0,
        landmark_nis_gate_rejected_count=0,
        landmark_nis_sum=0.0,
        landmark_nis_max=0.0,
    )


class HardwareValidationTests(unittest.TestCase):
    def test_usb_diagnostic_separates_beacon_and_raw_rx_probe(self) -> None:
        clock = FakeClock()
        controller = DiagnosticControllerTransport(clock)
        result = run_serial_diagnostic(
            HardwareConfig("/dev/cu.usbmodem-test"),
            "usb",
            transport_factory=lambda config: controller,
            clock=clock,
            sleep=clock.sleep,
        )

        self.assertEqual(result.exit_code, HardwareExitCode.SUCCESS)
        self.assertTrue(result.summary["diagnostic"]["tx_proven"])
        self.assertGreaterEqual(result.summary["diagnostic"]["serial_rx_bytes"], 2)
        self.assertGreaterEqual(result.summary["diagnostic"]["parser_errors"], 1)
        self.assertFalse(controller.is_open)

    def test_protocol_diagnostic_requires_accepted_hello_and_ack(self) -> None:
        clock = FakeClock()
        controller = DiagnosticControllerTransport(clock)
        result = run_serial_diagnostic(
            HardwareConfig("/dev/cu.usbmodem-test"),
            "protocol",
            transport_factory=lambda config: controller,
            clock=clock,
            sleep=clock.sleep,
        )

        self.assertEqual(result.exit_code, HardwareExitCode.SUCCESS)
        diagnostic = result.summary["diagnostic"]
        self.assertEqual(diagnostic["hello_result"], "ACCEPTED")
        self.assertEqual(diagnostic["parser_status"], "OK")
        self.assertEqual(diagnostic["hello_ack"]["status"], "OK")
        self.assertGreaterEqual(diagnostic["response_frames_created"], 1)

    def test_diagnostic_without_beacon_fails_explicitly(self) -> None:
        clock = FakeClock()
        controller = DiagnosticControllerTransport(clock, emit_beacon=False)
        result = run_serial_diagnostic(
            HardwareConfig("/dev/cu.usbmodem-test"),
            "usb",
            transport_factory=lambda config: controller,
            clock=clock,
            sleep=clock.sleep,
        )

        self.assertEqual(result.exit_code, HardwareExitCode.DIAGNOSTIC_FAILED)
        self.assertFalse(result.succeeded)

    def test_normal_and_watchdog_exchange_use_typed_protocol(self) -> None:
        clock = FakeClock()
        controller = ScriptedControllerTransport(
            clock,
            watchdog_response=True,
        )
        result = run_physical_validation(
            HardwareConfig("/dev/cu.usbmodem-test", watchdog_check=True),
            transport_factory=lambda config: controller,
            clock=clock,
            sleep=clock.sleep,
        )

        self.assertEqual(result.exit_code, HardwareExitCode.SUCCESS)
        self.assertTrue(result.succeeded)
        self.assertEqual(len(controller.route_chunks), 1)
        self.assertEqual(len(controller.sensor_payloads), 10)
        self.assertEqual(controller.first_write_at, 3.0)
        self.assertEqual(controller.input_reset_count, 1)
        for index, payload in enumerate(controller.sensor_payloads, start=1):
            self.assertEqual(payload.imu.sample_step_id, index)
            self.assertEqual(payload.wheel_speed.speed_mps, 0.0)
            self.assertEqual(payload.gnss, GnssSample(index, index * 20_000, 0.0, 0.0))
        self.assertEqual(
            result.summary["watchdog"]["evidence"],
            [
                "CONTROL_COMMAND.mode=SAFE_STOP",
                "HEALTH_STATUS.runtime_state=SAFE_STOP",
            ],
        )
        self.assertGreaterEqual(result.summary["watchdog"]["observed_after_ms"], 500)
        self.assertFalse(controller.is_open)

    def test_delayed_hello_ack_survives_initial_no_byte_window(self) -> None:
        clock = FakeClock()
        controller = ScriptedControllerTransport(clock, ack_delay_s=0.8)
        result = run_physical_validation(
            HardwareConfig("/dev/cu.usbmodem-test"),
            transport_factory=lambda config: controller,
            clock=clock,
            sleep=clock.sleep,
        )

        self.assertEqual(result.exit_code, HardwareExitCode.SUCCESS)
        self.assertGreaterEqual(result.summary["session_statistics"]["reconnects"], 1)
        self.assertGreaterEqual(controller.first_write_at, 3.0)

    def test_no_byte_response_is_a_handshake_failure_after_retries(self) -> None:
        clock = FakeClock()
        controller = ScriptedControllerTransport(clock, acknowledge=False)
        result = run_physical_validation(
            HardwareConfig("/dev/cu.usbmodem-test"),
            transport_factory=lambda config: controller,
            clock=clock,
            sleep=clock.sleep,
        )

        self.assertEqual(result.exit_code, HardwareExitCode.HANDSHAKE_FAILED)
        self.assertEqual(result.summary["session_statistics"]["rx_bytes"], 0)
        self.assertGreaterEqual(result.summary["session_statistics"]["reconnects"], 1)

    def test_phase_specific_failure_codes(self) -> None:
        def cannot_open(_config: SerialConfig) -> ScriptedControllerTransport:
            raise TransportError("cannot open test port")

        port = run_physical_validation(
            HardwareConfig("/dev/cu.usbmodem-test"),
            transport_factory=cannot_open,
        )
        self.assertEqual(port.exit_code, HardwareExitCode.PORT_OPEN_FAILED)

        cases = (
            (
                HardwareExitCode.HANDSHAKE_FAILED,
                dict(acknowledge=False),
                False,
            ),
            (
                HardwareExitCode.EXCHANGE_FAILED,
                dict(normal_responses=False),
                False,
            ),
            (
                HardwareExitCode.EXCHANGE_FAILED,
                dict(normal_response_at=7, normal_response_step=1),
                False,
            ),
            (
                HardwareExitCode.WATCHDOG_FAILED,
                dict(watchdog_response=False),
                True,
            ),
        )
        for expected, options, watchdog_check in cases:
            with self.subTest(expected=expected):
                clock = FakeClock()
                controller = ScriptedControllerTransport(clock, **options)
                result = run_physical_validation(
                    HardwareConfig(
                        "/dev/cu.usbmodem-test",
                        watchdog_check=watchdog_check,
                    ),
                    transport_factory=lambda config, item=controller: item,
                    clock=clock,
                    sleep=clock.sleep,
                )
                self.assertEqual(result.exit_code, expected)
                self.assertFalse(result.succeeded)

    def test_hardware_help_documents_required_options(self) -> None:
        output = io.StringIO()
        with patch.object(sys, "argv", ["navbench", "hardware", "--help"]):
            with contextlib.redirect_stdout(output):
                with self.assertRaises(SystemExit) as captured:
                    main()
        self.assertEqual(captured.exception.code, 0)
        self.assertIn("--port", output.getvalue())
        self.assertIn("--baud", output.getvalue())
        self.assertIn("--startup-delay", output.getvalue())
        self.assertIn("--diagnostic", output.getvalue())
        self.assertIn("--watchdog-check", output.getvalue())


if __name__ == "__main__":
    unittest.main()
