#pragma once

#include <cstddef>
#include <cstdint>

#include "navbench/control.hpp"
#include "navbench/ekf.hpp"
#include "navbench/math.hpp"

namespace navbench {

enum class SafetyState : uint8_t {
  Startup = 0U,
  SelfTest = 1U,
  Ready = 2U,
  Running = 3U,
  Degraded = 4U,
  SafeStop = 5U,
  Fault = 6U,
};

enum class InputDisposition : uint8_t {
  Accepted = 0U,
  Invalid = 1U,
  Corrupt = 2U,
  Stale = 3U,
  Duplicate = 4U,
  OutOfOrder = 5U,
  Oversized = 6U,
};

enum class RuntimeTask : uint8_t {
  Estimator = 0U,
  Control = 1U,
  Health = 2U,
  Telemetry = 3U,
  Count = 4U,
};

constexpr std::size_t kRuntimeTaskCount =
    static_cast<std::size_t>(RuntimeTask::Count);
constexpr std::size_t kMaximumLandmarksPerFrame = 4U;

struct SchedulerConfig {
  uint32_t period_ms[kRuntimeTaskCount];
  uint32_t execution_budget_us[kRuntimeTaskCount];

  static SchedulerConfig defaults();
};

struct ScheduleDecision {
  bool due[kRuntimeTaskCount]{};
};

struct TaskTiming {
  uint32_t runs{0U};
  uint32_t overruns{0U};
  uint32_t missed_releases{0U};
  uint32_t last_duration_us{0U};
  uint32_t maximum_duration_us{0U};
  uint64_t total_duration_us{0U};
};

class CooperativeScheduler {
 public:
  CooperativeScheduler();
  explicit CooperativeScheduler(const SchedulerConfig& config);

  bool set_config(const SchedulerConfig& config);
  void reset(uint32_t now_ms);
  ScheduleDecision poll(uint32_t now_ms);
  void record_execution(RuntimeTask task, uint32_t duration_us);
  const TaskTiming& timing(RuntimeTask task) const;

 private:
  bool validate_config(const SchedulerConfig& config) const;

  SchedulerConfig config_{};
  uint32_t next_release_ms_[kRuntimeTaskCount]{};
  TaskTiming timing_[kRuntimeTaskCount]{};
  bool started_{false};
};

struct RuntimeConfig {
  uint32_t host_timeout_ms;
  uint32_t unavailable_grace_ms;
  uint16_t queue_overflow_safe_stop_threshold;
  SchedulerConfig scheduler;

  static RuntimeConfig defaults();
};

struct RuntimeStats {
  uint32_t accepted_inputs{0U};
  uint32_t invalid_inputs{0U};
  uint32_t corrupt_inputs{0U};
  uint32_t stale_inputs{0U};
  uint32_t duplicate_inputs{0U};
  uint32_t out_of_order_inputs{0U};
  uint32_t oversized_inputs{0U};
  uint32_t queue_overflows{0U};
  uint32_t watchdog_timeouts{0U};
  uint32_t manual_safe_stops{0U};
  uint32_t numerical_faults{0U};
  uint32_t state_transitions{0U};
};

struct RuntimeDecision {
  SafetyState state{SafetyState::Startup};
  bool permit_control{false};
  bool force_safe_stop{true};
  bool output_neutral{true};
};

class RuntimeCore {
 public:
  RuntimeCore();
  explicit RuntimeCore(const RuntimeConfig& config);

  bool set_config(const RuntimeConfig& config);
  void begin(uint32_t now_ms);
  bool start_self_test();
  bool complete_self_test(bool passed, uint32_t now_ms);
  void notify_handshake(uint32_t now_ms);
  void accept_input(uint32_t now_ms);
  void record_rejected_input(InputDisposition disposition);
  void record_queue_overflow();
  void request_safe_stop();
  void report_numerical_fault();
  RuntimeDecision tick(uint32_t now_ms, NavigationMode navigation_mode,
                       bool estimator_healthy, bool control_ready = true);

  SafetyState state() const { return state_; }
  const RuntimeStats& stats() const { return stats_; }
  CooperativeScheduler& scheduler() { return scheduler_; }
  const CooperativeScheduler& scheduler() const { return scheduler_; }

 private:
  bool validate_config(const RuntimeConfig& config) const;
  void transition(SafetyState next_state, uint32_t now_ms);

  RuntimeConfig config_{};
  CooperativeScheduler scheduler_{};
  RuntimeStats stats_{};
  SafetyState state_{SafetyState::Startup};
  uint32_t state_entered_ms_{0U};
  uint32_t last_valid_input_ms_{0U};
  uint32_t unavailable_since_ms_{0U};
  bool host_seen_{false};
  bool unavailable_timer_active_{false};
};

struct SensorFrameInput {
  uint32_t timestamp_ms{0U};
  uint32_t step_id{0U};
  InputDisposition disposition{InputDisposition::Accepted};
  bool has_imu{false};
  bool has_wheel_speed{false};
  bool has_gnss{false};
  ImuMeasurement imu{};
  WheelSpeedMeasurement wheel_speed{};
  GnssMeasurement gnss{};
  LandmarkMeasurement landmarks[kMaximumLandmarksPerFrame]{};
  uint8_t landmark_count{0U};
};

struct ControllerStepInput {
  uint32_t now_ms{0U};
  float dt_s{0.0F};
  bool has_sensor_frame{false};
  bool manual_safe_stop{false};
  SensorFrameInput sensor_frame{};
};

// Logical status only. A board-specific HAL maps these fields to pins or HMI.
struct HalStatus {
  bool ready{false};
  bool running{false};
  bool degraded{false};
  bool safe_stop{true};
  bool fault{false};
};

struct ControllerStepOutput {
  EkfState estimate{};
  ControlCommand command{};
  NavigationMode navigation_mode{NavigationMode::Unavailable};
  SafetyState safety_state{SafetyState::Startup};
  HalStatus hal_status{};
  bool estimator_initialized{false};
  bool estimator_healthy{false};
};

class EmbeddedControllerCore {
 public:
  EmbeddedControllerCore();
  EmbeddedControllerCore(const EkfConfig& ekf_config,
                         const ControlConfig& control_config,
                         const RuntimeConfig& runtime_config);

  bool configure(const EkfConfig& ekf_config,
                 const ControlConfig& control_config,
                 const RuntimeConfig& runtime_config);
  void begin(uint32_t now_ms);
  bool start_self_test();
  bool complete_self_test(bool passed, uint32_t now_ms);
  void notify_handshake(uint32_t now_ms);
  bool set_route(const Waypoint* waypoints, std::size_t count);
  bool initialize_estimator(const EkfState& initial_state,
                            uint32_t timestamp_ms);
  ControllerStepOutput step(const ControllerStepInput& input);

  const Ekf6& estimator() const { return estimator_; }
  Ekf6& estimator() { return estimator_; }
  const ControlSystem& controller() const { return controller_; }
  const RuntimeCore& runtime() const { return runtime_; }
  RuntimeCore& runtime() { return runtime_; }

 private:
  bool process_sensor_frame(const SensorFrameInput& frame, float dt_s,
                            uint32_t received_at_ms);
  ControllerStepOutput make_output(uint32_t now_ms,
                                   const RuntimeDecision& runtime_decision,
                                   float dt_s);

  Ekf6 estimator_{};
  ControlSystem controller_{};
  RuntimeCore runtime_{};
};

}  // namespace navbench
