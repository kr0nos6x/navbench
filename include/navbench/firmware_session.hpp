#pragma once

#include <cstddef>
#include <cstdint>

#include "navbench/math.hpp"
#include "navbench/protocol.hpp"
#include "navbench/runtime.hpp"

namespace navbench {

// The application/session boundary shared by the Arduino image and native CIL
// executable. It owns all protocol and controller state and never allocates.
struct FirmwareSessionConfig {
  uint32_t heartbeat_period_ms;
  uint32_t health_period_ms;
  uint32_t maximum_sample_lag_steps;
  float nominal_sensor_dt_s;

  static FirmwareSessionConfig defaults();
};

struct FirmwareSessionStats {
  uint32_t packets_handled{0U};
  uint32_t application_errors{0U};
  uint32_t sensor_frames_enqueued{0U};
  uint32_t sensor_frames_processed{0U};
  uint32_t sensor_queue_overflows{0U};
  uint32_t tx_frames{0U};
  uint32_t tx_dropped{0U};
  uint32_t loop_runs{0U};
  uint32_t max_loop_us{0U};
#if defined(NAVBENCH_SERIAL_DIAGNOSTIC)
  uint32_t diagnostic_hello_packets{0U};
  uint8_t diagnostic_last_parser_status{0U};
  uint8_t diagnostic_last_hello_result{0U};
#endif
};

class FirmwareSession {
 public:
  static constexpr std::size_t kTxQueueCapacity = 8U;
  static constexpr std::size_t kSensorQueueCapacity = 4U;

  FirmwareSession();
  explicit FirmwareSession(const FirmwareSessionConfig& config);

  bool set_config(const FirmwareSessionConfig& config);
  void reset(uint32_t now_ms);

  // Input may contain partial, single, or concatenated COBS frames.
  void feed(uint32_t now_ms, const uint8_t* data, std::size_t size);

  // Polls the cooperative scheduler and advances timeout/control/telemetry
  // processing when the configured tasks become due.
  void tick(uint32_t now_ms, uint32_t step_id);

  bool pop_frame(uint8_t* output, std::size_t capacity,
                 std::size_t* output_size);
  std::size_t pending_frames() const { return tx_queue_.size(); }
  std::size_t pending_sensor_frames() const { return sensor_queue_.size(); }
  uint32_t last_step_id() const { return last_step_id_; }
  bool session_active() const { return session_active_; }

  // Board main records actual loop duration without coupling the core to a
  // clock/HAL. Native tests may leave it at zero.
  void record_loop_duration(uint32_t duration_us);

  const FirmwareSessionStats& stats() const { return stats_; }
  const protocol::ParserStats& parser_stats() const { return parser_.stats(); }
  const protocol::SequenceStats& sequence_stats() const {
    return sequence_.stats();
  }
  const EmbeddedControllerCore& core() const { return core_; }

 private:
  struct TxFrame {
    uint16_t size{0U};
    uint8_t bytes[protocol::kMaxWireFrameSize]{};
  };

  struct PendingSensorFrame {
    SensorFrameInput frame{};
    uint32_t packet_step_id{0U};
  };

  struct SampleStepTracker {
    uint32_t last_step_id{0U};
    bool initialized{false};
  };

  static bool valid_config(const FirmwareSessionConfig& config);
  static void packet_callback(const protocol::Packet& packet, void* context);
  static void parser_error_callback(protocol::Status status, void* context);

  void handle_packet(const protocol::Packet& packet);
  void handle_hello(const protocol::Packet& packet);
  void handle_route(const protocol::Packet& packet);
  void handle_sensor(const protocol::Packet& packet);
  void handle_safe_stop(const protocol::Packet& packet);

  bool validate_sample_steps(const protocol::SensorFramePayload& sensor,
                             uint32_t frame_step,
                             InputDisposition* disposition) const;
  void commit_sample_steps(const protocol::SensorFramePayload& sensor);

  float sensor_timestep(uint32_t now_ms);
  void emit_controller_output(uint32_t step_id,
                              const ControllerStepOutput& output,
                              bool force_health);
  void emit_error(uint32_t step_id,
                  protocol::ApplicationErrorCode code, uint16_t detail,
                  uint32_t related_sequence);

  template <typename Payload>
  bool emit_payload(protocol::MessageType type, uint32_t step_id,
                    const Payload& payload);
  bool enqueue(const TxFrame& frame);
  bool reserve_controller_bundle(std::size_t required_slots);

  FirmwareSessionConfig config_{};
  FirmwareSessionStats stats_{};
  EmbeddedControllerCore core_{};
  protocol::StreamParser parser_{};
  protocol::SequenceTracker sequence_{};
  FixedQueue<TxFrame, kTxQueueCapacity> tx_queue_{};
  FixedQueue<PendingSensorFrame, kSensorQueueCapacity> sensor_queue_{};
  Waypoint route_[kMaximumWaypoints]{};
  uint32_t tx_sequence_{0U};
  uint32_t current_now_ms_{0U};
  uint32_t session_started_ms_{0U};
  uint32_t last_step_id_{0U};
  uint32_t last_sensor_frame_ms_{0U};
  uint32_t last_valid_sensor_ms_{0U};
  uint32_t last_core_step_ms_{0U};
  uint32_t last_health_ms_{0U};
  uint32_t last_heartbeat_ms_{0U};
  uint16_t route_id_{0U};
  uint16_t route_expected_{0U};
  uint16_t route_received_{0U};
  SampleStepTracker imu_step_{};
  SampleStepTracker wheel_step_{};
  SampleStepTracker gnss_step_{};
  SampleStepTracker landmark_step_{};
  SafetyState last_emitted_state_{SafetyState::Startup};
  bool session_active_{false};
  bool have_sensor_time_{false};
  bool have_sensor_{false};
  uint8_t scheduled_task_mask_{0U};
};

}  // namespace navbench
