#pragma once

#include <cstddef>
#include <cstdint>

#include "navbench/protocol.hpp"

namespace navbench {

class SerialRxCounter {
 public:
  void add(std::size_t count);
  uint32_t bytes() const { return bytes_; }

 private:
  uint32_t bytes_{0U};
};

// Owns exactly one wire frame until every byte has been acknowledged by the
// board Serial implementation. A second producer cannot overwrite or append
// to a partially written frame.
class SerialTxStager {
 public:
  bool idle() const { return size_ == 0U; }
  uint8_t* writable_buffer() { return idle() ? bytes_ : nullptr; }
  std::size_t capacity() const { return sizeof(bytes_); }
  bool commit_frame(std::size_t size);
  uint8_t* pending_data() { return bytes_ + offset_; }
  const uint8_t* pending_data() const { return bytes_ + offset_; }
  std::size_t remaining() const { return size_ - offset_; }
  std::size_t next_write_size(std::size_t maximum) const;
  bool acknowledge_write(std::size_t requested, std::size_t written);

  uint32_t total_bytes_written() const { return total_bytes_written_; }
  uint32_t completed_frames() const { return completed_frames_; }
  uint32_t invalid_write_results() const { return invalid_write_results_; }

 private:
  uint8_t bytes_[protocol::kMaxWireFrameSize]{};
  std::size_t size_{0U};
  std::size_t offset_{0U};
  uint32_t total_bytes_written_{0U};
  uint32_t completed_frames_{0U};
  uint32_t invalid_write_results_{0U};
};

enum class DiagnosticEvent : uint16_t {
  Beacon = 1U,
  Receive = 2U,
  Parser = 3U,
  Transmit = 4U,
  Queue = 5U,
};

struct DiagnosticSnapshot {
  uint32_t serial_rx_bytes{0U};
  uint32_t parser_frames_received{0U};
  uint32_t parser_errors{0U};
  uint32_t hello_packets{0U};
  uint32_t response_frames_created{0U};
  uint32_t response_frames_dropped{0U};
  uint16_t response_frames_pending{0U};
  uint16_t last_write_requested{0U};
  uint16_t last_write_result{0U};
  uint8_t parser_status{0U};
  uint8_t hello_result{0U};
};

// Emits one initial beacon, changed status categories, then only a bounded
// periodic beacon while state remains unchanged.
class DiagnosticScheduler {
 public:
  static constexpr uint32_t kMinimumPeriodMs = 250U;
  static constexpr uint32_t kBeaconPeriodMs = 1000U;

  void reset(uint32_t now_ms);
  bool next(uint32_t now_ms, const DiagnosticSnapshot& snapshot,
            DiagnosticEvent* event);

 private:
  static bool elapsed(uint32_t now_ms, uint32_t then_ms, uint32_t period_ms);
  bool receive_changed(const DiagnosticSnapshot& snapshot) const;
  bool parser_changed(const DiagnosticSnapshot& snapshot) const;
  bool transmit_changed(const DiagnosticSnapshot& snapshot) const;
  bool queue_changed(const DiagnosticSnapshot& snapshot) const;

  DiagnosticSnapshot reported_{};
  uint32_t last_event_ms_{0U};
  uint32_t last_beacon_ms_{0U};
  uint8_t reported_mask_{0U};
  bool event_emitted_{false};
  bool initial_beacon_pending_{true};
};

}  // namespace navbench
