#include <cstdio>
#include <cstring>

#include "navbench/firmware_session.hpp"
#include "navbench/serial_io.hpp"

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

struct DecodeContext {
  uint32_t packets{0U};
  uint32_t errors{0U};
  uint32_t expected_sequence{0U};
  bool ordered{true};
};

void packet_callback(const wire::Packet& packet, void* context) {
  DecodeContext& decoded = *static_cast<DecodeContext*>(context);
  if (packet.sequence != decoded.expected_sequence) decoded.ordered = false;
  ++decoded.expected_sequence;
  ++decoded.packets;
}
void error_callback(wire::Status, void* context) {
  ++static_cast<DecodeContext*>(context)->errors;
}

bool test_transport_counter_survives_session_reset() {
  navbench::SerialRxCounter counter;
  navbench::FirmwareSession session;
  counter.add(17U);
  session.reset(100U);
  CHECK(counter.bytes() == 17U);
  counter.add(13U);
  CHECK(counter.bytes() == 30U);
  counter.add(static_cast<std::size_t>(0xffffffffUL));
  CHECK(counter.bytes() == 0xffffffffUL);
  counter.add(1U);
  CHECK(counter.bytes() == 0xffffffffUL);
  return true;
}

bool test_partial_writes_never_interleave_frames() {
  navbench::SerialTxStager stager;
  uint8_t* buffer = stager.writable_buffer();
  CHECK(buffer != nullptr);
  const uint8_t first[] = {1U, 2U, 3U, 0U};
  std::memcpy(buffer, first, sizeof(first));
  CHECK(stager.commit_frame(sizeof(first)));
  CHECK(stager.writable_buffer() == nullptr);
  CHECK(!stager.commit_frame(2U));
  CHECK(stager.next_write_size(2U) == 2U);
  CHECK(stager.acknowledge_write(2U, 1U));
  CHECK(stager.remaining() == 3U);
  CHECK(stager.pending_data()[0] == 2U);
  CHECK(stager.acknowledge_write(2U, 0U));
  CHECK(stager.remaining() == 3U);
  CHECK(stager.acknowledge_write(3U, 3U));
  CHECK(stager.idle());
  CHECK(stager.completed_frames() == 1U);
  CHECK(stager.total_bytes_written() == sizeof(first));

  buffer = stager.writable_buffer();
  CHECK(buffer != nullptr);
  const uint8_t second[] = {9U, 8U, 0U};
  std::memcpy(buffer, second, sizeof(second));
  CHECK(stager.commit_frame(sizeof(second)));
  CHECK(stager.acknowledge_write(sizeof(second), sizeof(second)));
  CHECK(stager.completed_frames() == 2U);
  return true;
}

bool test_diagnostic_rate_and_change_suppression() {
  navbench::DiagnosticScheduler scheduler;
  navbench::DiagnosticSnapshot snapshot{};
  navbench::DiagnosticEvent event = navbench::DiagnosticEvent::Queue;
  scheduler.reset(0U);
  CHECK(scheduler.next(0U, snapshot, &event));
  CHECK(event == navbench::DiagnosticEvent::Beacon);
  CHECK(!scheduler.next(249U, snapshot, &event));
  CHECK(scheduler.next(250U, snapshot, &event));
  CHECK(event == navbench::DiagnosticEvent::Receive);
  CHECK(scheduler.next(500U, snapshot, &event));
  CHECK(event == navbench::DiagnosticEvent::Parser);
  CHECK(scheduler.next(750U, snapshot, &event));
  CHECK(event == navbench::DiagnosticEvent::Transmit);
  CHECK(scheduler.next(1000U, snapshot, &event));
  CHECK(event == navbench::DiagnosticEvent::Queue);
  CHECK(scheduler.next(1250U, snapshot, &event));
  CHECK(event == navbench::DiagnosticEvent::Beacon);
  CHECK(!scheduler.next(1500U, snapshot, &event));
  snapshot.serial_rx_bytes = 31U;
  snapshot.parser_frames_received = 1U;
  CHECK(scheduler.next(1500U, snapshot, &event));
  CHECK(event == navbench::DiagnosticEvent::Receive);
  CHECK(!scheduler.next(1750U, snapshot, &event));
  return true;
}

bool test_stable_diagnostic_stream_is_bounded_without_feedback() {
  navbench::DiagnosticScheduler scheduler;
  navbench::DiagnosticSnapshot snapshot{};
  navbench::DiagnosticEvent event = navbench::DiagnosticEvent::Queue;
  scheduler.reset(0U);
  uint32_t emitted = 0U;
  uint32_t beacons = 0U;
  for (uint32_t now_ms = 0U; now_ms <= 75000U; now_ms += 10U) {
    if (!scheduler.next(now_ms, snapshot, &event)) continue;
    ++emitted;
    if (event == navbench::DiagnosticEvent::Beacon) ++beacons;
  }
  CHECK(emitted == 79U);
  CHECK(beacons == 75U);
  return true;
}

bool test_long_partial_diagnostic_stream_has_zero_crc_errors() {
  navbench::SerialTxStager stager;
  wire::StreamParser parser;
  DecodeContext decoded{};
  for (uint32_t sequence = 0U; sequence < 1000U; ++sequence) {
    wire::ErrorPayload payload{};
    payload.code = wire::ApplicationErrorCode::Diagnostic;
    payload.detail = static_cast<uint16_t>(navbench::DiagnosticEvent::Beacon);
    payload.related_sequence = 0x4e424447UL;
    payload.context = sequence;
    uint8_t encoded_payload[wire::kMaxPayloadSize]{};
    uint16_t payload_size = 0U;
    CHECK(wire::encodePayload(payload, encoded_payload,
                              sizeof(encoded_payload), &payload_size) ==
          wire::Status::Ok);
    std::size_t frame_size = 0U;
    CHECK(wire::encodePacket(wire::MessageType::Error, sequence, 0U,
                             encoded_payload, payload_size,
                             stager.writable_buffer(), stager.capacity(),
                             &frame_size) == wire::Status::Ok);
    CHECK(stager.commit_frame(frame_size));
    std::size_t chunk_index = 0U;
    while (!stager.idle()) {
      const std::size_t maximum = (chunk_index++ % 32U) + 1U;
      const std::size_t count = stager.next_write_size(maximum);
      parser.feed(stager.pending_data(), count, &packet_callback,
                  &error_callback, &decoded);
      CHECK(stager.acknowledge_write(count, count));
    }
  }
  CHECK(decoded.packets == 1000U);
  CHECK(decoded.errors == 0U);
  CHECK(decoded.ordered);
  CHECK(parser.stats().crc_errors == 0U);
  CHECK(parser.stats().cobs_errors == 0U);
  CHECK(stager.completed_frames() == 1000U);
  return true;
}

}  // namespace

int main() {
  if (!test_transport_counter_survives_session_reset() ||
      !test_partial_writes_never_interleave_frames() ||
      !test_diagnostic_rate_and_change_suppression() ||
      !test_stable_diagnostic_stream_is_bounded_without_feedback() ||
      !test_long_partial_diagnostic_stream_has_zero_crc_errors()) return 1;
  std::printf("test_serial_io: PASS (%d checks)\n", checks);
  return 0;
}
