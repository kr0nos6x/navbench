#include "navbench/protocol.hpp"

#include <stdint.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace protocol = navbench::protocol;

namespace {

int failures = 0;
int checks = 0;

#define CHECK(condition)                                                        \
  do {                                                                          \
    ++checks;                                                                    \
    if (!(condition)) {                                                          \
      std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: "           \
                << #condition << '\n';                                           \
      ++failures;                                                                \
    }                                                                            \
  } while (false)

struct GoldenVector {
  std::string name;
  protocol::MessageType message_type;
  uint32_t sequence;
  uint32_t step_id;
  std::vector<uint8_t> payload;
  std::vector<uint8_t> frame;
};

struct RejectionVector {
  std::string name;
  protocol::Status expected;
  std::vector<uint8_t> frame;
};

std::vector<std::string> splitTabs(const std::string &line) {
  std::vector<std::string> fields;
  size_t start = 0U;
  while (true) {
    const size_t tab = line.find('\t', start);
    if (tab == std::string::npos) {
      fields.push_back(line.substr(start));
      return fields;
    }
    fields.push_back(line.substr(start, tab - start));
    start = tab + 1U;
  }
}

uint8_t hexNibble(char value) {
  if (value >= '0' && value <= '9') {
    return static_cast<uint8_t>(value - '0');
  }
  if (value >= 'a' && value <= 'f') {
    return static_cast<uint8_t>(10 + value - 'a');
  }
  if (value >= 'A' && value <= 'F') {
    return static_cast<uint8_t>(10 + value - 'A');
  }
  CHECK(false);
  return 0U;
}

std::vector<uint8_t> parseHex(const std::string &text) {
  if (text == "-") {
    return std::vector<uint8_t>();
  }
  CHECK(text.size() % 2U == 0U);
  std::vector<uint8_t> result;
  result.reserve(text.size() / 2U);
  for (size_t index = 0U; index + 1U < text.size(); index += 2U) {
    result.push_back(static_cast<uint8_t>((hexNibble(text[index]) << 4U) |
                                          hexNibble(text[index + 1U])));
  }
  return result;
}

uint32_t parseU32(const std::string &text) {
  char *end = 0;
  const unsigned long long value = std::strtoull(text.c_str(), &end, 10);
  CHECK(end != 0 && *end == '\0');
  CHECK(value <= 0xffffffffULL);
  return static_cast<uint32_t>(value);
}

std::vector<GoldenVector> readGoldenVectors(const char *path) {
  std::ifstream input(path);
  CHECK(input.good());
  std::vector<GoldenVector> vectors;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    const std::vector<std::string> fields = splitTabs(line);
    CHECK(fields.size() == 6U);
    if (fields.size() != 6U) {
      continue;
    }
    GoldenVector vector;
    vector.name = fields[0];
    const uint32_t type = parseU32(fields[1]);
    CHECK(type <= 0xffU);
    vector.message_type = static_cast<protocol::MessageType>(type);
    vector.sequence = parseU32(fields[2]);
    vector.step_id = parseU32(fields[3]);
    vector.payload = parseHex(fields[4]);
    vector.frame = parseHex(fields[5]);
    vectors.push_back(vector);
  }
  return vectors;
}

protocol::Status parseStatus(const std::string &name) {
  if (name == "EMPTY_FRAME") return protocol::Status::EmptyFrame;
  if (name == "UNEXPECTED_DELIMITER") return protocol::Status::UnexpectedDelimiter;
  if (name == "COBS_MALFORMED") return protocol::Status::CobsMalformed;
  if (name == "PACKET_TOO_SHORT") return protocol::Status::PacketTooShort;
  if (name == "CRC_MISMATCH") return protocol::Status::CrcMismatch;
  if (name == "UNSUPPORTED_VERSION") return protocol::Status::UnsupportedVersion;
  if (name == "UNKNOWN_MESSAGE_TYPE") return protocol::Status::UnknownMessageType;
  if (name == "PAYLOAD_LENGTH_MISMATCH") {
    return protocol::Status::PayloadLengthMismatch;
  }
  if (name == "PAYLOAD_TOO_LARGE") return protocol::Status::PayloadTooLarge;
  if (name == "OVERSIZED_FRAME") return protocol::Status::OversizedFrame;
  CHECK(false);
  return protocol::Status::InvalidValue;
}

std::vector<RejectionVector> readRejectionVectors(const char *path) {
  std::ifstream input(path);
  CHECK(input.good());
  std::vector<RejectionVector> vectors;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    const std::vector<std::string> fields = splitTabs(line);
    CHECK(fields.size() == 3U);
    if (fields.size() != 3U) {
      continue;
    }
    RejectionVector vector;
    vector.name = fields[0];
    vector.expected = parseStatus(fields[1]);
    vector.frame = parseHex(fields[2]);
    vectors.push_back(vector);
  }
  return vectors;
}

template <typename Payload>
void typedRoundTrip(const GoldenVector &vector) {
  Payload value;
  CHECK(protocol::decodePayload(vector.payload.data(),
                                static_cast<uint16_t>(vector.payload.size()),
                                &value) == protocol::Status::Ok);
  uint8_t encoded[protocol::kMaxPayloadSize];
  uint16_t encoded_size = 0U;
  CHECK(protocol::encodePayload(value, encoded, sizeof(encoded), &encoded_size) ==
        protocol::Status::Ok);
  CHECK(encoded_size == vector.payload.size());
  CHECK(std::vector<uint8_t>(encoded, encoded + encoded_size) == vector.payload);
}

void testGoldenVectors(const std::vector<GoldenVector> &vectors) {
  CHECK(vectors.size() == 13U);
  bool saw_maximum = false;
  for (size_t index = 0U; index < vectors.size(); ++index) {
    const GoldenVector &vector = vectors[index];
    uint8_t frame[protocol::kMaxWireFrameSize];
    size_t frame_size = 0U;
    const uint8_t *payload = vector.payload.empty() ? 0 : vector.payload.data();
    CHECK(protocol::encodePacket(
              vector.message_type, vector.sequence, vector.step_id, payload,
              static_cast<uint16_t>(vector.payload.size()), frame, sizeof(frame),
              &frame_size) == protocol::Status::Ok);
    CHECK(frame_size == vector.frame.size());
    CHECK(std::vector<uint8_t>(frame, frame + frame_size) == vector.frame);

    protocol::Packet decoded;
    CHECK(protocol::decodePacket(vector.frame.data(), vector.frame.size(),
                                 &decoded) == protocol::Status::Ok);
    CHECK(decoded.message_type == vector.message_type);
    CHECK(decoded.sequence == vector.sequence);
    CHECK(decoded.step_id == vector.step_id);
    CHECK(decoded.payload_size == vector.payload.size());
    CHECK(std::vector<uint8_t>(decoded.payload,
                               decoded.payload + decoded.payload_size) ==
          vector.payload);

    if (vector.name == "hello_host") {
      typedRoundTrip<protocol::HelloPayload>(vector);
    } else if (vector.name == "hello_ack_controller") {
      typedRoundTrip<protocol::HelloAckPayload>(vector);
    } else if (vector.name == "sensor_all") {
      typedRoundTrip<protocol::SensorFramePayload>(vector);
    } else if (vector.name == "route_two_points") {
      typedRoundTrip<protocol::RouteChunkPayload>(vector);
    } else if (vector.name == "control_zero_bytes") {
      typedRoundTrip<protocol::ControlCommandPayload>(vector);
    } else if (vector.name == "state_estimate") {
      typedRoundTrip<protocol::StateEstimatePayload>(vector);
    } else if (vector.name == "health_status") {
      typedRoundTrip<protocol::HealthStatusPayload>(vector);
      protocol::HealthStatusPayload health;
      CHECK(protocol::decodePayload(
                vector.payload.data(),
                static_cast<uint16_t>(vector.payload.size()), &health) ==
            protocol::Status::Ok);
      CHECK(health.imu_yaw_nis_evaluated_count == 11U);
      CHECK(health.imu_yaw_nis_gate_rejected_count == 2U);
      CHECK(health.imu_yaw_nis_sum == 12.5F);
      CHECK(health.imu_yaw_nis_max == 3.0F);
      CHECK(health.wheel_nis_evaluated_count == 12U);
      CHECK(health.wheel_nis_gate_rejected_count == 1U);
      CHECK(health.wheel_nis_sum == 15.0F);
      CHECK(health.wheel_nis_max == 4.0F);
      CHECK(health.gnss_nis_evaluated_count == 5U);
      CHECK(health.gnss_nis_gate_rejected_count == 2U);
      CHECK(health.gnss_nis_sum == 20.0F);
      CHECK(health.gnss_nis_max == 9.0F);
      CHECK(health.landmark_nis_evaluated_count == 3U);
      CHECK(health.landmark_nis_gate_rejected_count == 0U);
      CHECK(health.landmark_nis_sum == 2.25F);
      CHECK(health.landmark_nis_max == 1.25F);
    } else if (vector.name == "heartbeat") {
      typedRoundTrip<protocol::HeartbeatPayload>(vector);
    } else if (vector.name == "error") {
      typedRoundTrip<protocol::ErrorPayload>(vector);
    } else if (vector.name == "safe_stop") {
      typedRoundTrip<protocol::SafeStopPayload>(vector);
    }
    if (vector.name == "maximum_payload") {
      saw_maximum = true;
      CHECK(vector.payload.size() == protocol::kMaxPayloadSize);
      CHECK(vector.frame.size() == protocol::kMaxWireFrameSize);
    }
  }
  CHECK(saw_maximum);
}

void testRejectionVectors(const std::vector<RejectionVector> &vectors) {
  CHECK(vectors.size() == 10U);
  static const uint8_t empty_storage = 0U;
  for (size_t index = 0U; index < vectors.size(); ++index) {
    const RejectionVector &vector = vectors[index];
    const uint8_t *frame =
        vector.frame.empty() ? &empty_storage : vector.frame.data();
    protocol::Packet packet;
    CHECK(protocol::decodePacket(frame, vector.frame.size(), &packet) ==
          vector.expected);
  }
}

void testPrimitives() {
  static const uint8_t check[] = {'1', '2', '3', '4', '5',
                                  '6', '7', '8', '9'};
  CHECK(protocol::crc16Ccitt(check, sizeof(check)) == 0x29b1U);

  static const uint8_t source[] = {0U, 1U, 0U, 2U, 3U, 0U};
  uint8_t encoded[16];
  size_t encoded_size = 0U;
  CHECK(protocol::cobsEncode(source, sizeof(source), encoded, sizeof(encoded),
                             &encoded_size) == protocol::Status::Ok);
  uint8_t decoded[16];
  size_t decoded_size = 0U;
  CHECK(protocol::cobsDecode(encoded, 0U, decoded, sizeof(decoded),
                             &decoded_size) == protocol::Status::CobsMalformed);
  CHECK(protocol::cobsDecode(encoded, encoded_size, decoded, sizeof(decoded),
                             &decoded_size) == protocol::Status::Ok);
  CHECK(decoded_size == sizeof(source));
  CHECK(std::vector<uint8_t>(decoded, decoded + decoded_size) ==
        std::vector<uint8_t>(source, source + sizeof(source)));
}

struct Capture {
  std::vector<protocol::Packet> packets;
  std::vector<protocol::Status> errors;
};

void capturePacket(const protocol::Packet &packet, void *context) {
  static_cast<Capture *>(context)->packets.push_back(packet);
}

void captureError(protocol::Status status, void *context) {
  static_cast<Capture *>(context)->errors.push_back(status);
}

void testStreamParser(const std::vector<GoldenVector> &vectors) {
  std::vector<uint8_t> combined;
  for (size_t index = 0U; index < vectors.size(); ++index) {
    combined.insert(combined.end(), vectors[index].frame.begin(),
                    vectors[index].frame.end());
  }
  protocol::StreamParser parser;
  Capture capture;
  static const size_t chunks[] = {1U, 2U, 7U, 3U, 19U, 5U};
  size_t offset = 0U;
  size_t chunk_index = 0U;
  while (offset < combined.size()) {
    size_t amount = chunks[chunk_index % (sizeof(chunks) / sizeof(chunks[0]))];
    if (amount > combined.size() - offset) {
      amount = combined.size() - offset;
    }
    parser.feed(combined.data() + offset, amount, capturePacket, captureError,
                &capture);
    offset += amount;
    ++chunk_index;
  }
  CHECK(capture.errors.empty());
  CHECK(capture.packets.size() == vectors.size());
  CHECK(parser.stats().packets_accepted == vectors.size());
  CHECK(parser.stats().frames_received == vectors.size());
  CHECK(parser.finish(captureError, &capture) == protocol::Status::Ok);

  std::vector<uint8_t> corrupt = vectors[1].frame;
  corrupt[5] ^= 0x80U;
  parser.reset(true);
  capture = Capture();
  parser.feed(corrupt.data(), corrupt.size(), capturePacket, captureError,
              &capture);
  CHECK(capture.packets.empty());
  CHECK(capture.errors.size() == 1U);
  CHECK(capture.errors[0] == protocol::Status::CrcMismatch);
  CHECK(parser.stats().crc_errors == 1U);

  std::vector<uint8_t> oversized(protocol::kMaxEncodedFrameSize + 1U, 1U);
  oversized.push_back(0U);
  parser.reset(true);
  capture = Capture();
  parser.feed(oversized.data(), oversized.size(), capturePacket, captureError,
              &capture);
  CHECK(capture.errors.size() == 1U);
  CHECK(capture.errors[0] == protocol::Status::OversizedFrame);
  CHECK(parser.stats().oversized_frames == 1U);

  parser.reset(true);
  capture = Capture();
  parser.feed(vectors[0].frame.data(), vectors[0].frame.size() - 1U,
              capturePacket, captureError, &capture);
  CHECK(parser.finish(captureError, &capture) ==
        protocol::Status::TruncatedFrame);
  CHECK(capture.errors.size() == 1U);
  CHECK(capture.errors[0] == protocol::Status::TruncatedFrame);
}

void testSequenceTracker() {
  protocol::SequenceTracker tracker(32U);
  CHECK(tracker.observe(0xfffffffeUL).disposition ==
        protocol::SequenceDisposition::First);
  CHECK(tracker.observe(0xffffffffUL).disposition ==
        protocol::SequenceDisposition::InOrder);
  CHECK(tracker.observe(0U).disposition ==
        protocol::SequenceDisposition::InOrder);
  CHECK(tracker.observe(0U).disposition ==
        protocol::SequenceDisposition::Duplicate);
  CHECK(tracker.observe(0xffffffffUL).disposition ==
        protocol::SequenceDisposition::OutOfOrder);
  CHECK(tracker.observe(0xffffff00UL).disposition ==
        protocol::SequenceDisposition::Stale);
  const protocol::SequenceResult gap = tracker.observe(3U);
  CHECK(gap.disposition == protocol::SequenceDisposition::Gap);
  CHECK(gap.missing == 2U);
  CHECK(tracker.observe(4U, 100U).accepted);
  CHECK(tracker.observe(5U).accepted);
  CHECK(tracker.observe(6U, 99U).disposition ==
        protocol::SequenceDisposition::Stale);
  CHECK(tracker.stats().duplicates == 1U);
  CHECK(tracker.stats().out_of_order == 1U);
  CHECK(tracker.stats().stale == 2U);
  CHECK(tracker.stats().missing == 2U);
}

void testInvalidTypedPayloads() {
  uint8_t buffer[protocol::kMaxPayloadSize];
  uint16_t size = 0U;

  protocol::StateEstimatePayload estimate = {};
  estimate.navigation_mode = protocol::NavigationMode::GnssAided;
  estimate.covariance_diagonal[2] = -1.0F;
  CHECK(protocol::encodePayload(estimate, buffer, sizeof(buffer), &size) ==
        protocol::Status::InvalidValue);

  protocol::RouteChunkPayload route = {};
  route.point_count = protocol::kMaxRoutePointsPerChunk + 1U;
  CHECK(protocol::encodePayload(route, buffer, sizeof(buffer), &size) ==
        protocol::Status::InvalidValue);

  protocol::SensorFramePayload sensor = {};
  sensor.present_mask = 0x8000U;
  CHECK(protocol::encodePayload(sensor, buffer, sizeof(buffer), &size) ==
        protocol::Status::InvalidValue);

  protocol::HealthStatusPayload health = {};
  health.runtime_state = protocol::RuntimeState::Running;
  health.navigation_mode = protocol::NavigationMode::GnssAided;
  health.gnss_nis_evaluated_count = 1U;
  health.gnss_nis_sum = std::numeric_limits<float>::quiet_NaN();
  CHECK(protocol::encodePayload(health, buffer, sizeof(buffer), &size) ==
        protocol::Status::InvalidValue);
  health.gnss_nis_sum = 1.0F;
  health.gnss_nis_max = 1.0F;
  health.gnss_nis_gate_rejected_count = 2U;
  CHECK(protocol::encodePayload(health, buffer, sizeof(buffer), &size) ==
        protocol::Status::InvalidValue);
}

void testPacketBoundaries() {
  uint8_t payload[protocol::kMaxPayloadSize + 1U] = {};
  uint8_t frame[protocol::kMaxWireFrameSize];
  size_t frame_size = 99U;
  CHECK(protocol::encodePacket(protocol::MessageType::Error, 0U, 0U, payload,
                               protocol::kMaxPayloadSize + 1U, frame,
                               sizeof(frame), &frame_size) ==
        protocol::Status::PayloadTooLarge);
  CHECK(frame_size == 0U);
  CHECK(protocol::encodePacket(static_cast<protocol::MessageType>(0x55U), 0U,
                               0U, 0, 0U, frame, sizeof(frame), &frame_size) ==
        protocol::Status::UnknownMessageType);
  CHECK(protocol::encodePacket(protocol::MessageType::Error, 0U, 0U, payload,
                               1U, frame, 2U, &frame_size) ==
        protocol::Status::BufferTooSmall);
}

void testThousandFrameSoak() {
  protocol::StreamParser parser;
  Capture capture;
  for (uint32_t index = 0U; index < 1000U; ++index) {
    protocol::SensorFramePayload sensor = {};
    sensor.present_mask = protocol::SensorImu | protocol::SensorWheelSpeed |
                          protocol::SensorGnss | protocol::SensorLandmark;
    sensor.imu.sample_step_id = index;
    sensor.imu.timestamp_us = index * 20000U;
    sensor.imu.longitudinal_acceleration_mps2 =
        static_cast<float>(index % 13U) * 0.125F - 0.5F;
    sensor.imu.yaw_rate_rps = static_cast<float>(index % 7U) * 0.01F;
    sensor.wheel_speed.sample_step_id = index;
    sensor.wheel_speed.timestamp_us = index * 20000U;
    sensor.wheel_speed.speed_mps = static_cast<float>(index % 100U) * 0.05F;
    sensor.gnss.sample_step_id = index;
    sensor.gnss.timestamp_us = index * 20000U;
    sensor.gnss.x_m = static_cast<float>(index) * 0.1F;
    sensor.gnss.y_m = static_cast<float>(index % 17U) - 8.0F;
    sensor.landmark.sample_step_id = index;
    sensor.landmark.timestamp_us = index * 20000U;
    sensor.landmark.landmark_id = static_cast<uint16_t>(index % 16U);
    sensor.landmark.landmark_x_m = 25.0F;
    sensor.landmark.landmark_y_m = -5.0F;
    sensor.landmark.range_m = 10.0F + static_cast<float>(index % 5U);
    sensor.landmark.bearing_rad = -0.5F + static_cast<float>(index % 11U) * 0.1F;

    uint8_t payload[protocol::kMaxPayloadSize];
    uint16_t payload_size = 0U;
    CHECK(protocol::encodePayload(sensor, payload, sizeof(payload),
                                  &payload_size) == protocol::Status::Ok);
    CHECK(payload_size == 76U);

    uint8_t frame[protocol::kMaxWireFrameSize];
    size_t frame_size = 0U;
    CHECK(protocol::encodePacket(protocol::MessageType::SensorFrame, index,
                                 index, payload, payload_size, frame,
                                 sizeof(frame), &frame_size) ==
          protocol::Status::Ok);
    const size_t split = (index * 17U) % frame_size;
    parser.feed(frame, split, capturePacket, captureError, &capture);
    parser.feed(frame + split, frame_size - split, capturePacket, captureError,
                &capture);
  }
  CHECK(capture.errors.empty());
  CHECK(capture.packets.size() == 1000U);
  CHECK(parser.stats().frames_received == 1000U);
  CHECK(parser.stats().packets_accepted == 1000U);
  CHECK(parser.finish(captureError, &capture) == protocol::Status::Ok);
}

}  // namespace

int main(int argc, char **argv) {
  const char *fixture = argc > 1 ? argv[1] : "test/fixtures/protocol_v1_golden.tsv";
  const char *invalid_fixture =
      argc > 2 ? argv[2] : "test/fixtures/protocol_v1_invalid.tsv";
  const std::vector<GoldenVector> vectors = readGoldenVectors(fixture);
  const std::vector<RejectionVector> rejection_vectors =
      readRejectionVectors(invalid_fixture);
  testPrimitives();
  testGoldenVectors(vectors);
  testRejectionVectors(rejection_vectors);
  testStreamParser(vectors);
  testSequenceTracker();
  testInvalidTypedPayloads();
  testPacketBoundaries();
  testThousandFrameSoak();
  if (failures != 0) {
    std::cerr << failures << " protocol test(s) failed\n";
    return 1;
  }
  std::cout << "protocol native tests: " << checks
            << " checks, 13 golden vectors, 10 rejection vectors, typed "
               "payloads, stream errors, sequence policy, 1000-frame soak "
               "passed\n";
  return 0;
}
