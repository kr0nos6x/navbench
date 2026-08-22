#include "navbench/serial_io.hpp"

#include <limits>

namespace navbench {
namespace {

uint32_t saturating_add(uint32_t value, std::size_t increment) {
  const uint32_t maximum = std::numeric_limits<uint32_t>::max();
  if (increment >= static_cast<std::size_t>(maximum - value)) {
    return maximum;
  }
  return value + static_cast<uint32_t>(increment);
}

constexpr uint8_t event_bit(DiagnosticEvent event) {
  return static_cast<uint8_t>(1U << (static_cast<uint8_t>(event) - 1U));
}

}  // namespace

void SerialRxCounter::add(std::size_t count) {
  bytes_ = saturating_add(bytes_, count);
}

bool SerialTxStager::commit_frame(std::size_t size) {
  if (!idle() || size == 0U || size > capacity()) {
    return false;
  }
  size_ = size;
  offset_ = 0U;
  return true;
}

std::size_t SerialTxStager::next_write_size(std::size_t maximum) const {
  if (maximum == 0U || idle()) {
    return 0U;
  }
  return remaining() < maximum ? remaining() : maximum;
}

bool SerialTxStager::acknowledge_write(std::size_t requested,
                                       std::size_t written) {
  if (idle() || requested == 0U || requested > remaining() ||
      written > requested) {
    invalid_write_results_ = saturating_add(invalid_write_results_, 1U);
    return false;
  }
  offset_ += written;
  total_bytes_written_ = saturating_add(total_bytes_written_, written);
  if (offset_ == size_) {
    completed_frames_ = saturating_add(completed_frames_, 1U);
    size_ = 0U;
    offset_ = 0U;
  }
  return true;
}

bool DiagnosticScheduler::elapsed(uint32_t now_ms, uint32_t then_ms,
                                  uint32_t period_ms) {
  return static_cast<uint32_t>(now_ms - then_ms) >= period_ms;
}

void DiagnosticScheduler::reset(uint32_t now_ms) {
  reported_ = DiagnosticSnapshot{};
  last_event_ms_ = now_ms;
  last_beacon_ms_ = now_ms;
  reported_mask_ = 0U;
  event_emitted_ = false;
  initial_beacon_pending_ = true;
}

bool DiagnosticScheduler::receive_changed(
    const DiagnosticSnapshot& snapshot) const {
  return snapshot.serial_rx_bytes != reported_.serial_rx_bytes ||
         snapshot.parser_frames_received != reported_.parser_frames_received;
}

bool DiagnosticScheduler::parser_changed(
    const DiagnosticSnapshot& snapshot) const {
  return snapshot.parser_errors != reported_.parser_errors ||
         snapshot.hello_packets != reported_.hello_packets ||
         snapshot.parser_status != reported_.parser_status ||
         snapshot.hello_result != reported_.hello_result;
}

bool DiagnosticScheduler::transmit_changed(
    const DiagnosticSnapshot& snapshot) const {
  return snapshot.last_write_requested != reported_.last_write_requested ||
         snapshot.last_write_result != reported_.last_write_result;
}

bool DiagnosticScheduler::queue_changed(
    const DiagnosticSnapshot& snapshot) const {
  return snapshot.response_frames_created !=
             reported_.response_frames_created ||
         snapshot.response_frames_dropped !=
             reported_.response_frames_dropped ||
         snapshot.response_frames_pending != reported_.response_frames_pending;
}

bool DiagnosticScheduler::next(uint32_t now_ms,
                               const DiagnosticSnapshot& snapshot,
                               DiagnosticEvent* event) {
  if (event == nullptr ||
      (event_emitted_ &&
       !elapsed(now_ms, last_event_ms_, kMinimumPeriodMs))) {
    return false;
  }

  DiagnosticEvent selected = DiagnosticEvent::Beacon;
  if (initial_beacon_pending_) {
    initial_beacon_pending_ = false;
  } else if ((reported_mask_ & event_bit(DiagnosticEvent::Receive)) == 0U ||
             receive_changed(snapshot)) {
    selected = DiagnosticEvent::Receive;
    reported_.serial_rx_bytes = snapshot.serial_rx_bytes;
    reported_.parser_frames_received = snapshot.parser_frames_received;
  } else if ((reported_mask_ & event_bit(DiagnosticEvent::Parser)) == 0U ||
             parser_changed(snapshot)) {
    selected = DiagnosticEvent::Parser;
    reported_.parser_errors = snapshot.parser_errors;
    reported_.hello_packets = snapshot.hello_packets;
    reported_.parser_status = snapshot.parser_status;
    reported_.hello_result = snapshot.hello_result;
  } else if ((reported_mask_ & event_bit(DiagnosticEvent::Transmit)) == 0U ||
             transmit_changed(snapshot)) {
    selected = DiagnosticEvent::Transmit;
    reported_.last_write_requested = snapshot.last_write_requested;
    reported_.last_write_result = snapshot.last_write_result;
  } else if ((reported_mask_ & event_bit(DiagnosticEvent::Queue)) == 0U ||
             queue_changed(snapshot)) {
    selected = DiagnosticEvent::Queue;
    reported_.response_frames_created = snapshot.response_frames_created;
    reported_.response_frames_dropped = snapshot.response_frames_dropped;
    reported_.response_frames_pending = snapshot.response_frames_pending;
  } else if (!elapsed(now_ms, last_beacon_ms_, kBeaconPeriodMs)) {
    return false;
  }

  *event = selected;
  reported_mask_ |= event_bit(selected);
  last_event_ms_ = now_ms;
  event_emitted_ = true;
  if (selected == DiagnosticEvent::Beacon) {
    last_beacon_ms_ = now_ms;
  }
  return true;
}

}  // namespace navbench
