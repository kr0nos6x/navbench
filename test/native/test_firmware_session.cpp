#include <cmath>
#include <cstdio>
#include <cstring>

#include "navbench/firmware_session.hpp"

namespace {

namespace wire = navbench::protocol;

int checks = 0;

#define CHECK(condition)                                                      \
  do {                                                                        \
    ++checks;                                                                 \
    if (!(condition)) {                                                       \
      std::fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__, __LINE__, \
                   #condition);                                               \
      return false;                                                           \
    }                                                                         \
  } while (false)

template <typename Payload>
bool make_frame(wire::MessageType type, uint32_t sequence, uint32_t step_id,
                const Payload& payload, uint8_t* frame,
                std::size_t frame_capacity, std::size_t& frame_size) {
  uint8_t bytes[wire::kMaxPayloadSize]{};
  uint16_t size = 0U;
  return wire::encodePayload(payload, bytes, sizeof(bytes), &size) ==
             wire::Status::Ok &&
         wire::encodePacket(type, sequence, step_id, bytes, size, frame,
                            frame_capacity, &frame_size) == wire::Status::Ok;
}

bool pop_packet(navbench::FirmwareSession& session, wire::Packet& packet) {
  uint8_t frame[wire::kMaxWireFrameSize]{};
  std::size_t frame_size = 0U;
  if (!session.pop_frame(frame, sizeof(frame), &frame_size)) {
    return false;
  }
  return wire::decodePacket(frame, frame_size, &packet) == wire::Status::Ok;
}

wire::HelloPayload host_hello() {
  wire::HelloPayload hello{};
  hello.role = wire::EndpointRole::Host;
  hello.min_version = wire::kProtocolVersion;
  hello.max_version = wire::kProtocolVersion;
  hello.capabilities =
      wire::CapabilitySensorFrame | wire::CapabilityRouteChunk |
      wire::CapabilityStateEstimate | wire::CapabilityHealthStatus |
      wire::CapabilitySafeStop;
  hello.max_payload = static_cast<uint16_t>(wire::kMaxPayloadSize);
  hello.heartbeat_timeout_ms = 500U;
  return hello;
}

bool feed_hello(navbench::FirmwareSession& session, uint32_t sequence = 0U) {
  uint8_t frame[wire::kMaxWireFrameSize]{};
  std::size_t frame_size = 0U;
  const wire::HelloPayload hello = host_hello();
  CHECK(make_frame(wire::MessageType::Hello, sequence, 0U, hello, frame,
                   sizeof(frame), frame_size));
  session.feed(0U, frame, frame_size);
  wire::Packet response{};
  CHECK(pop_packet(session, response));
  CHECK(response.message_type == wire::MessageType::HelloAck);
  wire::HelloAckPayload ack{};
  CHECK(wire::decodePayload(response.payload, response.payload_size, &ack) ==
        wire::Status::Ok);
  CHECK(ack.status == wire::HelloStatus::Ok);
  return true;
}

bool test_fragmented_handshake_and_version_rejection() {
  navbench::FirmwareSession session;
  uint8_t frame[wire::kMaxWireFrameSize]{};
  std::size_t frame_size = 0U;
  wire::HelloPayload hello = host_hello();
  CHECK(make_frame(wire::MessageType::Hello, 0U, 0U, hello, frame,
                   sizeof(frame), frame_size));
  session.feed(0U, frame, frame_size / 2U);
  CHECK(session.pending_frames() == 0U);
  session.feed(0U, frame + frame_size / 2U, frame_size - frame_size / 2U);

  wire::Packet packet{};
  CHECK(pop_packet(session, packet));
  CHECK(packet.message_type == wire::MessageType::HelloAck);
  wire::HelloAckPayload ack{};
  CHECK(wire::decodePayload(packet.payload, packet.payload_size, &ack) ==
        wire::Status::Ok);
  CHECK(ack.status == wire::HelloStatus::Ok);
  CHECK(ack.accepted_version == wire::kProtocolVersion);
  CHECK(session.core().runtime().state() == navbench::SafetyState::Ready);

  session.reset(0U);
  hello.min_version = 2U;
  hello.max_version = 2U;
  CHECK(make_frame(wire::MessageType::Hello, 0U, 0U, hello, frame,
                   sizeof(frame), frame_size));
  session.feed(0U, frame, frame_size);
  CHECK(pop_packet(session, packet));
  CHECK(wire::decodePayload(packet.payload, packet.payload_size, &ack) ==
        wire::Status::Ok);
  CHECK(ack.status == wire::HelloStatus::VersionMismatch);
  CHECK(ack.accepted_version == 0U);
  return true;
}

bool test_hello_restarts_wire_session_without_clearing_safety_latch() {
  navbench::FirmwareSession session;
  CHECK(feed_hello(session));

  uint8_t frame[wire::kMaxWireFrameSize]{};
  std::size_t frame_size = 0U;
  const wire::HelloPayload hello = host_hello();
  CHECK(make_frame(wire::MessageType::Hello, 0U, 0U, hello, frame,
                   sizeof(frame), frame_size));
  session.feed(10U, frame, frame_size);
  wire::Packet packet{};
  CHECK(pop_packet(session, packet));
  CHECK(packet.message_type == wire::MessageType::HelloAck);
  CHECK(packet.sequence == 0U);
  CHECK(session.sequence_stats().accepted == 1U);

  wire::HeartbeatPayload heartbeat{};
  heartbeat.uptime_ms = 20U;
  heartbeat.monotonic_ms = 20U;
  heartbeat.runtime_state = wire::RuntimeState::Ready;
  CHECK(make_frame(wire::MessageType::Heartbeat, 1U, 1U, heartbeat, frame,
                   sizeof(frame), frame_size));
  session.feed(20U, frame, frame_size);
  CHECK(session.sequence_stats().accepted == 2U);
  CHECK(session.sequence_stats().out_of_order == 0U);

  while (pop_packet(session, packet)) {
  }
  wire::SafeStopPayload stop{};
  stop.reason = wire::SafeStopReason::Manual;
  stop.latch = true;
  stop.detail = 0U;
  CHECK(make_frame(wire::MessageType::SafeStop, 2U, 2U, stop, frame,
                   sizeof(frame), frame_size));
  session.feed(21U, frame, frame_size);
  CHECK(session.core().runtime().state() == navbench::SafetyState::SafeStop);
  while (pop_packet(session, packet)) {
  }

  CHECK(make_frame(wire::MessageType::Hello, 0U, 0U, hello, frame,
                   sizeof(frame), frame_size));
  session.feed(22U, frame, frame_size);
  CHECK(pop_packet(session, packet));
  CHECK(packet.message_type == wire::MessageType::HelloAck);
  CHECK(packet.sequence == 0U);
  CHECK(session.core().runtime().state() == navbench::SafetyState::SafeStop);
  return true;
}

bool feed_route(navbench::FirmwareSession& session, uint32_t first_sequence) {
  wire::RouteChunkPayload first{};
  first.route_id = 7U;
  first.start_index = 0U;
  first.total_count = 6U;
  first.point_count = 5U;
  first.flags = wire::RouteClearExisting;
  for (uint8_t index = 0U; index < first.point_count; ++index) {
    first.points[index].x_m = static_cast<float>(index);
    first.points[index].y_m = 0.0F;
    first.points[index].target_speed_mps = 1.0F;
    first.points[index].acceptance_radius_m = 1.0F;
  }
  wire::RouteChunkPayload final{};
  final.route_id = 7U;
  final.start_index = 5U;
  final.total_count = 6U;
  final.point_count = 1U;
  final.flags = wire::RouteFinalChunk;
  final.points[0].x_m = 5.0F;
  final.points[0].y_m = 0.0F;
  final.points[0].target_speed_mps = 0.0F;
  final.points[0].acceptance_radius_m = 1.0F;

  uint8_t frames[2U * wire::kMaxWireFrameSize]{};
  std::size_t first_size = 0U;
  std::size_t second_size = 0U;
  CHECK(make_frame(wire::MessageType::RouteChunk, first_sequence, 0U, first,
                   frames, sizeof(frames), first_size));
  CHECK(make_frame(wire::MessageType::RouteChunk, first_sequence + 1U, 0U,
                   final, frames + first_size, sizeof(frames) - first_size,
                   second_size));
  session.feed(0U, frames, first_size + second_size);
  CHECK(session.pending_frames() == 0U);
  CHECK(session.core().controller().route().waypoint_count() == 6U);
  return true;
}

wire::SensorFramePayload initial_sensor(uint32_t step_id) {
  wire::SensorFramePayload sensor{};
  sensor.present_mask =
      wire::SensorImu | wire::SensorWheelSpeed | wire::SensorGnss |
      wire::SensorLandmark;
  sensor.imu.sample_step_id = step_id;
  sensor.imu.timestamp_us = step_id * 20000U;
  sensor.imu.longitudinal_acceleration_mps2 = 0.0F;
  sensor.imu.yaw_rate_rps = 0.0F;
  sensor.wheel_speed.sample_step_id = step_id;
  sensor.wheel_speed.timestamp_us = step_id * 20000U;
  sensor.wheel_speed.speed_mps = 0.0F;
  sensor.gnss.sample_step_id = step_id;
  sensor.gnss.timestamp_us = step_id * 20000U;
  sensor.gnss.x_m = 0.0F;
  sensor.gnss.y_m = 0.0F;
  sensor.landmark.sample_step_id = step_id;
  sensor.landmark.timestamp_us = step_id * 20000U;
  sensor.landmark.landmark_id = 9U;
  sensor.landmark.landmark_x_m = 5.0F;
  sensor.landmark.landmark_y_m = 0.0F;
  sensor.landmark.range_m = 5.0F;
  sensor.landmark.bearing_rad = 0.0F;
  return sensor;
}

bool test_route_sensor_sequence_and_timeout() {
  navbench::FirmwareSession session;
  CHECK(feed_hello(session));
  CHECK(feed_route(session, 1U));

  const wire::SensorFramePayload sensor = initial_sensor(1U);
  uint8_t frame[wire::kMaxWireFrameSize]{};
  std::size_t frame_size = 0U;
  CHECK(make_frame(wire::MessageType::SensorFrame, 3U, 1U, sensor, frame,
                   sizeof(frame), frame_size));
  session.feed(20U, frame, frame_size);
  CHECK(session.pending_sensor_frames() == 1U);
  CHECK(session.pending_frames() == 0U);
  session.tick(20U, 1U);
  CHECK(session.pending_frames() == 3U);

  wire::Packet packet{};
  CHECK(pop_packet(session, packet));
  CHECK(packet.message_type == wire::MessageType::ControlCommand);
  wire::ControlCommandPayload command{};
  CHECK(wire::decodePayload(packet.payload, packet.payload_size, &command) ==
        wire::Status::Ok);
  CHECK(command.mode == wire::ControlMode::Tracking);
  CHECK(pop_packet(session, packet));
  CHECK(packet.message_type == wire::MessageType::StateEstimate);
  wire::StateEstimatePayload estimate{};
  CHECK(wire::decodePayload(packet.payload, packet.payload_size, &estimate) ==
        wire::Status::Ok);
  CHECK(estimate.navigation_mode == wire::NavigationMode::GnssAided);
  CHECK(pop_packet(session, packet));
  CHECK(packet.message_type == wire::MessageType::HealthStatus);
  wire::HealthStatusPayload health{};
  CHECK(wire::decodePayload(packet.payload, packet.payload_size, &health) ==
        wire::Status::Ok);
  CHECK(health.runtime_state == wire::RuntimeState::Running);
  CHECK(health.imu_yaw_nis_evaluated_count == 1U);
  CHECK(health.imu_yaw_nis_gate_rejected_count == 0U);
  CHECK(health.wheel_nis_evaluated_count == 1U);
  CHECK(health.wheel_nis_gate_rejected_count == 0U);
  CHECK(health.gnss_nis_evaluated_count == 1U);
  CHECK(health.gnss_nis_gate_rejected_count == 0U);
  CHECK(health.landmark_nis_evaluated_count == 1U);
  CHECK(health.landmark_nis_gate_rejected_count == 0U);
  CHECK(std::isfinite(health.imu_yaw_nis_sum));
  CHECK(std::isfinite(health.wheel_nis_sum));
  CHECK(std::isfinite(health.gnss_nis_sum));
  CHECK(std::isfinite(health.landmark_nis_sum));
  const float initial_imu_nis_sum = health.imu_yaw_nis_sum;
  const float initial_wheel_nis_sum = health.wheel_nis_sum;
  const float initial_gnss_nis_sum = health.gnss_nis_sum;
  const float initial_landmark_nis_sum = health.landmark_nis_sum;

  session.feed(21U, frame, frame_size);
  CHECK(session.pending_frames() == 0U);
  CHECK(session.sequence_stats().duplicates == 1U);
  wire::SensorFramePayload stale = initial_sensor(0U);
  CHECK(make_frame(wire::MessageType::SensorFrame, 4U, 0U, stale, frame,
                   sizeof(frame), frame_size));
  session.feed(22U, frame, frame_size);
  CHECK(session.pending_frames() == 0U);
  CHECK(session.sequence_stats().stale == 1U);

  session.tick(521U, 25U);
  CHECK(pop_packet(session, packet));
  CHECK(packet.message_type == wire::MessageType::ControlCommand);
  CHECK(wire::decodePayload(packet.payload, packet.payload_size, &command) ==
        wire::Status::Ok);
  CHECK(command.mode == wire::ControlMode::SafeStop);
  CHECK(pop_packet(session, packet));
  CHECK(packet.message_type == wire::MessageType::StateEstimate);
  CHECK(pop_packet(session, packet));
  CHECK(packet.message_type == wire::MessageType::HealthStatus);
  CHECK(wire::decodePayload(packet.payload, packet.payload_size, &health) ==
        wire::Status::Ok);
  CHECK(health.runtime_state == wire::RuntimeState::SafeStop);
  CHECK(health.imu_yaw_nis_evaluated_count == 1U);
  CHECK(health.wheel_nis_evaluated_count == 1U);
  CHECK(health.gnss_nis_evaluated_count == 1U);
  CHECK(health.landmark_nis_evaluated_count == 1U);
  CHECK(health.imu_yaw_nis_sum == initial_imu_nis_sum);
  CHECK(health.wheel_nis_sum == initial_wheel_nis_sum);
  CHECK(health.gnss_nis_sum == initial_gnss_nis_sum);
  CHECK(health.landmark_nis_sum == initial_landmark_nis_sum);
  CHECK(session.core().runtime().stats().watchdog_timeouts == 1U);
  return true;
}

bool test_ready_without_route_is_neutral_and_old_samples_do_not_refresh() {
  navbench::FirmwareSession session;
  CHECK(feed_hello(session));

  uint8_t frame[wire::kMaxWireFrameSize]{};
  std::size_t frame_size = 0U;
  const wire::SensorFramePayload first = initial_sensor(1U);
  CHECK(make_frame(wire::MessageType::SensorFrame, 1U, 1U, first, frame,
                   sizeof(frame), frame_size));
  session.feed(20U, frame, frame_size);
  session.tick(20U, 1U);

  wire::Packet packet{};
  CHECK(pop_packet(session, packet));
  CHECK(packet.message_type == wire::MessageType::ControlCommand);
  wire::ControlCommandPayload command{};
  CHECK(wire::decodePayload(packet.payload, packet.payload_size, &command) ==
        wire::Status::Ok);
  CHECK(command.mode == wire::ControlMode::Neutral);
  CHECK(pop_packet(session, packet));
  CHECK(packet.message_type == wire::MessageType::StateEstimate);
  CHECK(pop_packet(session, packet));
  CHECK(packet.message_type == wire::MessageType::HealthStatus);
  wire::HealthStatusPayload health{};
  CHECK(wire::decodePayload(packet.payload, packet.payload_size, &health) ==
        wire::Status::Ok);
  CHECK(health.runtime_state == wire::RuntimeState::Ready);
  CHECK(session.core().runtime().stats().accepted_inputs == 1U);

  const wire::SensorFramePayload old = initial_sensor(0U);
  CHECK(make_frame(wire::MessageType::SensorFrame, 2U, 25U, old, frame,
                   sizeof(frame), frame_size));
  session.feed(500U, frame, frame_size);
  CHECK(pop_packet(session, packet));
  CHECK(packet.message_type == wire::MessageType::Error);
  CHECK(session.core().runtime().stats().stale_inputs == 1U);
  CHECK(session.core().runtime().stats().accepted_inputs == 1U);

  session.tick(521U, 26U);
  CHECK(pop_packet(session, packet));
  CHECK(packet.message_type == wire::MessageType::ControlCommand);
  CHECK(wire::decodePayload(packet.payload, packet.payload_size, &command) ==
        wire::Status::Ok);
  CHECK(command.mode == wire::ControlMode::SafeStop);
  CHECK(session.core().runtime().stats().watchdog_timeouts == 1U);
  return true;
}

bool test_corrupt_frame_and_manual_safe_stop() {
  navbench::FirmwareSession session;
  CHECK(feed_hello(session));

  wire::SafeStopPayload stop{};
  stop.reason = wire::SafeStopReason::Manual;
  stop.latch = true;
  stop.detail = 12U;
  uint8_t frame[wire::kMaxWireFrameSize]{};
  std::size_t frame_size = 0U;
  CHECK(make_frame(wire::MessageType::SafeStop, 1U, 4U, stop, frame,
                   sizeof(frame), frame_size));
  frame[frame_size / 2U] ^= 0x55U;
  session.feed(20U, frame, frame_size);
  wire::Packet packet{};
  CHECK(pop_packet(session, packet));
  CHECK(packet.message_type == wire::MessageType::Error);
  CHECK(session.core().runtime().stats().corrupt_inputs == 1U);
  CHECK(session.core().runtime().state() == navbench::SafetyState::Ready);

  CHECK(make_frame(wire::MessageType::SafeStop, 1U, 4U, stop, frame,
                   sizeof(frame), frame_size));
  session.feed(21U, frame, frame_size);
  CHECK(pop_packet(session, packet));
  CHECK(packet.message_type == wire::MessageType::ControlCommand);
  wire::ControlCommandPayload command{};
  CHECK(wire::decodePayload(packet.payload, packet.payload_size, &command) ==
        wire::Status::Ok);
  CHECK(command.mode == wire::ControlMode::SafeStop);
  CHECK(session.core().runtime().state() == navbench::SafetyState::SafeStop);
  return true;
}

bool test_fixed_tx_overflow_enters_safe_stop_and_timing_counters() {
  navbench::FirmwareSession session;
  CHECK(feed_hello(session));
  uint8_t frame[wire::kMaxWireFrameSize]{};
  std::size_t frame_size = 0U;
  wire::ControlCommandPayload unsupported{};
  unsupported.mode = wire::ControlMode::Neutral;
  for (uint32_t sequence = 1U;
       sequence <= navbench::FirmwareSession::kTxQueueCapacity + 1U;
       ++sequence) {
    CHECK(make_frame(wire::MessageType::ControlCommand, sequence, 0U,
                     unsupported, frame, sizeof(frame), frame_size));
    session.feed(sequence, frame, frame_size);
  }
  CHECK(session.core().runtime().stats().queue_overflows == 1U);
  CHECK(session.core().runtime().state() == navbench::SafetyState::SafeStop);
  CHECK(session.pending_frames() == 1U);
  CHECK(session.stats().tx_dropped ==
        navbench::FirmwareSession::kTxQueueCapacity);

  session.tick(100U, 0U);
  session.record_loop_duration(4001U);
  CHECK(session.stats().loop_runs == 1U);
  CHECK(session.stats().max_loop_us == 4001U);
  CHECK(session.core().runtime().scheduler().timing(
            navbench::RuntimeTask::Control).overruns == 1U);
  return true;
}

}  // namespace

int main() {
  if (!test_fragmented_handshake_and_version_rejection() ||
      !test_hello_restarts_wire_session_without_clearing_safety_latch() ||
      !test_route_sensor_sequence_and_timeout() ||
      !test_ready_without_route_is_neutral_and_old_samples_do_not_refresh() ||
      !test_corrupt_frame_and_manual_safe_stop() ||
      !test_fixed_tx_overflow_enters_safe_stop_and_timing_counters()) {
    return 1;
  }
  std::printf("test_firmware_session: PASS (%d checks)\n", checks);
  return 0;
}
