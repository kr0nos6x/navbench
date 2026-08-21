from __future__ import annotations

import os
import unittest
from pathlib import Path

from navbench.native import NativeFirmwareTransport
from navbench.protocol import (
    ControlCommandPayload,
    ControlMode,
    GnssSample,
    ImuSample,
    MessageType,
    RuntimeState,
    SensorFramePayload,
    SensorMask,
    WheelSpeedSample,
)
from navbench.session import HostSession, SessionState


NATIVE_EXECUTABLE = os.environ.get("NAVBENCH_NATIVE_FIRMWARE")


@unittest.skipUnless(NATIVE_EXECUTABLE, "native firmware executable not configured")
class NativeFirmwareIntegrationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.transport = NativeFirmwareTransport(Path(NATIVE_EXECUTABLE or ""))
        self.session = HostSession(self.transport)
        self.transport.set_time(0)
        self.session.start(0)
        events = self.session.poll(0)
        self.assertEqual([item.packet.message_type for item in events], [MessageType.HELLO_ACK])
        self.assertEqual(self.session.state, SessionState.ACTIVE)

    def tearDown(self) -> None:
        self.session.close()

    def test_typed_sensor_frame_runs_real_cpp_estimator_and_control(self) -> None:
        from navbench.protocol import RouteWaypoint

        self.session.send_route(
            route_id=1,
            points=(
                RouteWaypoint(0.0, 0.0, 1.0, 1.0),
                RouteWaypoint(5.0, 0.0, 0.0, 1.0),
            ),
        )
        payload = SensorFramePayload(
            present_mask=SensorMask.IMU | SensorMask.WHEEL_SPEED | SensorMask.GNSS,
            imu=ImuSample(0, 0, 0.0, 0.0),
            wheel_speed=WheelSpeedSample(0, 0, 0.0),
            gnss=GnssSample(0, 0, 0.0, 0.0),
        )
        self.session.send_sensor(0, payload)
        events = self.session.poll(0)
        self.assertEqual(events, ())
        self.transport.tick(20, 1)
        events = self.session.poll(20)
        self.assertEqual(
            [item.packet.message_type for item in events],
            [
                MessageType.CONTROL_COMMAND,
                MessageType.STATE_ESTIMATE,
                MessageType.HEALTH_STATUS,
            ],
        )
        command = events[0].payload
        self.assertIsInstance(command, ControlCommandPayload)
        self.assertEqual(command.mode, ControlMode.TRACKING)
        health = events[2].payload
        self.assertEqual(health.runtime_state, RuntimeState.RUNNING)

    def test_host_timeout_forces_cpp_safe_stop(self) -> None:
        self.transport.tick(501, 25)
        events = self.session.poll(501)
        command = next(
            event.payload
            for event in events
            if event.packet.message_type is MessageType.CONTROL_COMMAND
        )
        health = next(
            event.payload
            for event in events
            if event.packet.message_type is MessageType.HEALTH_STATUS
        )
        self.assertEqual(command.mode, ControlMode.SAFE_STOP)
        self.assertEqual(health.runtime_state, RuntimeState.SAFE_STOP)


if __name__ == "__main__":
    unittest.main()
