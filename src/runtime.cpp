#include "navbench/runtime.hpp"

#include <cmath>
#include <cstring>

namespace navbench {
namespace {

constexpr std::size_t task_index(RuntimeTask task) {
  return static_cast<std::size_t>(task);
}

bool reached(uint32_t now_ms, uint32_t deadline_ms) {
  return static_cast<int32_t>(now_ms - deadline_ms) >= 0;
}

bool aided(NavigationMode mode) {
  return mode == NavigationMode::GnssAided ||
         mode == NavigationMode::LandmarkAided;
}

bool update_failed(UpdateInfo update) {
  return update.result == UpdateResult::InvalidMeasurement ||
         update.result == UpdateResult::NumericalFailure;
}

}  // namespace

SchedulerConfig SchedulerConfig::defaults() {
  SchedulerConfig config{};
  // The application core currently advances estimation and control atomically.
  // Keep both releases aligned so a sensor arrival cannot advance the PI state
  // more than once per 20 ms controller cycle.
  config.period_ms[task_index(RuntimeTask::Estimator)] = 20U;
  config.period_ms[task_index(RuntimeTask::Control)] = 20U;
  config.period_ms[task_index(RuntimeTask::Health)] = 100U;
  config.period_ms[task_index(RuntimeTask::Telemetry)] = 100U;
  config.execution_budget_us[task_index(RuntimeTask::Estimator)] = 4000U;
  config.execution_budget_us[task_index(RuntimeTask::Control)] = 3000U;
  config.execution_budget_us[task_index(RuntimeTask::Health)] = 2000U;
  config.execution_budget_us[task_index(RuntimeTask::Telemetry)] = 5000U;
  return config;
}

CooperativeScheduler::CooperativeScheduler()
    : config_(SchedulerConfig::defaults()) {}

CooperativeScheduler::CooperativeScheduler(const SchedulerConfig& config)
    : config_(SchedulerConfig::defaults()) {
  if (validate_config(config)) {
    config_ = config;
  }
}

bool CooperativeScheduler::validate_config(const SchedulerConfig& config) const {
  for (std::size_t i = 0U; i < kRuntimeTaskCount; ++i) {
    if (config.period_ms[i] == 0U || config.period_ms[i] > 60000U ||
        config.execution_budget_us[i] == 0U) {
      return false;
    }
  }
  return true;
}

bool CooperativeScheduler::set_config(const SchedulerConfig& config) {
  if (!validate_config(config)) {
    return false;
  }
  config_ = config;
  started_ = false;
  std::memset(next_release_ms_, 0, sizeof(next_release_ms_));
  for (std::size_t i = 0U; i < kRuntimeTaskCount; ++i) {
    timing_[i] = TaskTiming{};
  }
  return true;
}

void CooperativeScheduler::reset(uint32_t now_ms) {
  for (std::size_t i = 0U; i < kRuntimeTaskCount; ++i) {
    next_release_ms_[i] = now_ms + config_.period_ms[i];
    timing_[i] = TaskTiming{};
  }
  started_ = true;
}

ScheduleDecision CooperativeScheduler::poll(uint32_t now_ms) {
  ScheduleDecision decision{};
  if (!started_) {
    reset(now_ms);
    return decision;
  }

  for (std::size_t i = 0U; i < kRuntimeTaskCount; ++i) {
    if (!reached(now_ms, next_release_ms_[i])) {
      continue;
    }
    decision.due[i] = true;
    const uint32_t lateness = elapsed_ms(now_ms, next_release_ms_[i]);
    const uint32_t skipped = lateness / config_.period_ms[i];
    timing_[i].missed_releases += skipped;
    next_release_ms_[i] += (skipped + 1U) * config_.period_ms[i];
  }
  return decision;
}

void CooperativeScheduler::record_execution(RuntimeTask task,
                                              uint32_t duration_us) {
  const std::size_t index_value = task_index(task);
  if (index_value >= kRuntimeTaskCount) {
    return;
  }
  TaskTiming& timing_value = timing_[index_value];
  ++timing_value.runs;
  timing_value.last_duration_us = duration_us;
  timing_value.total_duration_us += duration_us;
  if (duration_us > timing_value.maximum_duration_us) {
    timing_value.maximum_duration_us = duration_us;
  }
  if (duration_us > config_.execution_budget_us[index_value]) {
    ++timing_value.overruns;
  }
}

const TaskTiming& CooperativeScheduler::timing(RuntimeTask task) const {
  static const TaskTiming kEmptyTiming{};
  const std::size_t index_value = task_index(task);
  return index_value < kRuntimeTaskCount ? timing_[index_value] : kEmptyTiming;
}

RuntimeConfig RuntimeConfig::defaults() {
  RuntimeConfig config{};
  config.host_timeout_ms = 500U;
  config.unavailable_grace_ms = 250U;
  config.queue_overflow_safe_stop_threshold = 1U;
  config.scheduler = SchedulerConfig::defaults();
  return config;
}

RuntimeCore::RuntimeCore()
    : config_(RuntimeConfig::defaults()), scheduler_(config_.scheduler) {}

RuntimeCore::RuntimeCore(const RuntimeConfig& config)
    : config_(RuntimeConfig::defaults()), scheduler_(config_.scheduler) {
  if (validate_config(config)) {
    config_ = config;
    (void)scheduler_.set_config(config.scheduler);
  }
}

bool RuntimeCore::validate_config(const RuntimeConfig& config) const {
  if (config.host_timeout_ms == 0U || config.unavailable_grace_ms == 0U ||
      config.queue_overflow_safe_stop_threshold == 0U) {
    return false;
  }
  CooperativeScheduler scheduler;
  return scheduler.set_config(config.scheduler);
}

bool RuntimeCore::set_config(const RuntimeConfig& config) {
  if (!validate_config(config)) {
    return false;
  }
  config_ = config;
  (void)scheduler_.set_config(config.scheduler);
  begin(0U);
  return true;
}

void RuntimeCore::begin(uint32_t now_ms) {
  stats_ = RuntimeStats{};
  state_ = SafetyState::Startup;
  state_entered_ms_ = now_ms;
  last_valid_input_ms_ = now_ms;
  unavailable_since_ms_ = now_ms;
  host_seen_ = false;
  unavailable_timer_active_ = false;
  scheduler_.reset(now_ms);
}

void RuntimeCore::transition(SafetyState next_state, uint32_t now_ms) {
  if (next_state == state_) {
    return;
  }
  state_ = next_state;
  state_entered_ms_ = now_ms;
  ++stats_.state_transitions;
}

bool RuntimeCore::start_self_test() {
  if (state_ != SafetyState::Startup) {
    return false;
  }
  transition(SafetyState::SelfTest, state_entered_ms_);
  return true;
}

bool RuntimeCore::complete_self_test(bool passed, uint32_t now_ms) {
  if (state_ != SafetyState::SelfTest) {
    return false;
  }
  transition(passed ? SafetyState::Ready : SafetyState::Fault, now_ms);
  return true;
}

void RuntimeCore::notify_handshake(uint32_t now_ms) {
  if (state_ == SafetyState::Ready || state_ == SafetyState::Running ||
      state_ == SafetyState::Degraded) {
    host_seen_ = true;
    last_valid_input_ms_ = now_ms;
  }
}

void RuntimeCore::accept_input(uint32_t now_ms) {
  if (state_ == SafetyState::Ready || state_ == SafetyState::Running ||
      state_ == SafetyState::Degraded) {
    host_seen_ = true;
    last_valid_input_ms_ = now_ms;
    ++stats_.accepted_inputs;
  }
}

void RuntimeCore::record_rejected_input(InputDisposition disposition) {
  switch (disposition) {
    case InputDisposition::Accepted:
      return;
    case InputDisposition::Invalid:
      ++stats_.invalid_inputs;
      return;
    case InputDisposition::Corrupt:
      ++stats_.corrupt_inputs;
      return;
    case InputDisposition::Stale:
      ++stats_.stale_inputs;
      return;
    case InputDisposition::Duplicate:
      ++stats_.duplicate_inputs;
      return;
    case InputDisposition::OutOfOrder:
      ++stats_.out_of_order_inputs;
      return;
    case InputDisposition::Oversized:
      ++stats_.oversized_inputs;
      return;
  }
}

void RuntimeCore::record_queue_overflow() {
  ++stats_.queue_overflows;
  if (stats_.queue_overflows >= config_.queue_overflow_safe_stop_threshold &&
      state_ != SafetyState::Fault) {
    transition(SafetyState::SafeStop, last_valid_input_ms_);
  }
}

void RuntimeCore::request_safe_stop() {
  ++stats_.manual_safe_stops;
  if (state_ != SafetyState::Fault) {
    transition(SafetyState::SafeStop, last_valid_input_ms_);
  }
}

void RuntimeCore::report_numerical_fault() {
  ++stats_.numerical_faults;
  transition(SafetyState::Fault, last_valid_input_ms_);
}

RuntimeDecision RuntimeCore::tick(uint32_t now_ms,
                                  NavigationMode navigation_mode,
                                  bool estimator_healthy,
                                  bool control_ready) {
  if ((state_ == SafetyState::Ready || state_ == SafetyState::Running ||
       state_ == SafetyState::Degraded) &&
      host_seen_ &&
      elapsed_ms(now_ms, last_valid_input_ms_) > config_.host_timeout_ms) {
    ++stats_.watchdog_timeouts;
    transition(SafetyState::SafeStop, now_ms);
  }

  if ((state_ == SafetyState::Running || state_ == SafetyState::Degraded) &&
      !estimator_healthy) {
    report_numerical_fault();
  }

  if (state_ == SafetyState::Ready && host_seen_ && estimator_healthy &&
      control_ready) {
    if (aided(navigation_mode)) {
      transition(SafetyState::Running, now_ms);
    } else if (navigation_mode == NavigationMode::DeadReckoning ||
               navigation_mode == NavigationMode::Degraded) {
      transition(SafetyState::Degraded, now_ms);
    }
  } else if (state_ == SafetyState::Running) {
    if (navigation_mode == NavigationMode::DeadReckoning ||
        navigation_mode == NavigationMode::Degraded) {
      unavailable_timer_active_ = false;
      transition(SafetyState::Degraded, now_ms);
    } else if (navigation_mode == NavigationMode::Unavailable) {
      unavailable_timer_active_ = true;
      unavailable_since_ms_ = now_ms;
      transition(SafetyState::Degraded, now_ms);
    } else {
      unavailable_timer_active_ = false;
    }
  } else if (state_ == SafetyState::Degraded) {
    if (aided(navigation_mode)) {
      unavailable_timer_active_ = false;
      transition(SafetyState::Running, now_ms);
    } else if (navigation_mode == NavigationMode::DeadReckoning ||
               navigation_mode == NavigationMode::Degraded) {
      unavailable_timer_active_ = false;
    } else if (navigation_mode == NavigationMode::Unavailable) {
      if (!unavailable_timer_active_) {
        unavailable_timer_active_ = true;
        unavailable_since_ms_ = now_ms;
      } else if (elapsed_ms(now_ms, unavailable_since_ms_) >=
                 config_.unavailable_grace_ms) {
        transition(SafetyState::SafeStop, now_ms);
      }
    }
  }

  RuntimeDecision decision{};
  decision.state = state_;
  if ((state_ == SafetyState::Running || state_ == SafetyState::Degraded) &&
      estimator_healthy && navigation_mode != NavigationMode::Unavailable &&
      control_ready) {
    decision.permit_control = true;
    decision.force_safe_stop = false;
    decision.output_neutral = false;
  } else if (state_ == SafetyState::SafeStop ||
             state_ == SafetyState::Fault) {
    decision.permit_control = false;
    decision.force_safe_stop = true;
    decision.output_neutral = false;
  } else {
    decision.permit_control = false;
    decision.force_safe_stop = false;
    decision.output_neutral = true;
  }
  return decision;
}

EmbeddedControllerCore::EmbeddedControllerCore() = default;

EmbeddedControllerCore::EmbeddedControllerCore(
    const EkfConfig& ekf_config, const ControlConfig& control_config,
    const RuntimeConfig& runtime_config)
    : estimator_(ekf_config),
      controller_(control_config),
      runtime_(runtime_config) {}

bool EmbeddedControllerCore::configure(const EkfConfig& ekf_config,
                                       const ControlConfig& control_config,
                                       const RuntimeConfig& runtime_config) {
  Ekf6 estimator_candidate;
  ControlSystem controller_candidate;
  RuntimeCore runtime_candidate;
  if (!estimator_candidate.set_config(ekf_config) ||
      !controller_candidate.set_config(control_config) ||
      !runtime_candidate.set_config(runtime_config)) {
    return false;
  }
  estimator_ = estimator_candidate;
  controller_ = controller_candidate;
  runtime_ = runtime_candidate;
  return true;
}

void EmbeddedControllerCore::begin(uint32_t now_ms) {
  estimator_.reset();
  controller_.reset();
  runtime_.begin(now_ms);
}

bool EmbeddedControllerCore::start_self_test() {
  return runtime_.start_self_test();
}

bool EmbeddedControllerCore::complete_self_test(bool passed,
                                                uint32_t now_ms) {
  return runtime_.complete_self_test(passed, now_ms);
}

void EmbeddedControllerCore::notify_handshake(uint32_t now_ms) {
  runtime_.notify_handshake(now_ms);
}

bool EmbeddedControllerCore::set_route(const Waypoint* waypoints,
                                       std::size_t count) {
  return controller_.set_route(waypoints, count);
}

bool EmbeddedControllerCore::initialize_estimator(const EkfState& initial_state,
                                                  uint32_t timestamp_ms) {
  return estimator_.initialize(initial_state, timestamp_ms);
}

bool EmbeddedControllerCore::process_sensor_frame(const SensorFrameInput& frame,
                                                  float dt_s,
                                                  uint32_t received_at_ms) {
  if (frame.disposition != InputDisposition::Accepted) {
    runtime_.record_rejected_input(frame.disposition);
    return false;
  }
  if (frame.landmark_count > kMaximumLandmarksPerFrame) {
    runtime_.record_rejected_input(InputDisposition::Invalid);
    return false;
  }

  Ekf6 estimator_candidate = estimator_;
  if (!estimator_candidate.initialized() && frame.has_gnss) {
    EkfState initial{};
    initial.x_m = frame.gnss.x_m;
    initial.y_m = frame.gnss.y_m;
    initial.speed_mps = frame.has_wheel_speed ? frame.wheel_speed.speed_mps : 0.0F;
    initial.yaw_rate_rad_s = frame.has_imu ? frame.imu.yaw_rate_rad_s : 0.0F;
    if (!estimator_candidate.initialize(initial, frame.timestamp_ms)) {
      runtime_.record_rejected_input(InputDisposition::Invalid);
      return false;
    }
  }

  bool failure = false;
  bool numerical_failure = false;
  if (frame.has_imu) {
    ImuMeasurement measurement = frame.imu;
    // Wire timestamps remain source-clock metadata. Estimator freshness uses
    // the controller receive clock so host/session epochs and uint32-us wrap do
    // not get mixed with board millis(). Sample age is bounded at the session
    // boundary before this point.
    measurement.timestamp_ms = received_at_ms;
    const UpdateInfo result = estimator_candidate.predict(measurement, dt_s);
    failure = failure || update_failed(result);
    numerical_failure =
        numerical_failure || result.result == UpdateResult::NumericalFailure;
  }
  if (frame.has_wheel_speed) {
    WheelSpeedMeasurement measurement = frame.wheel_speed;
    measurement.timestamp_ms = received_at_ms;
    const UpdateInfo result =
        estimator_candidate.update_wheel(measurement);
    failure = failure || update_failed(result);
    numerical_failure =
        numerical_failure || result.result == UpdateResult::NumericalFailure;
  }
  if (frame.has_gnss) {
    GnssMeasurement measurement = frame.gnss;
    measurement.timestamp_ms = received_at_ms;
    const UpdateInfo result = estimator_candidate.update_gnss(measurement);
    failure = failure || update_failed(result);
    numerical_failure =
        numerical_failure || result.result == UpdateResult::NumericalFailure;
  }
  for (uint8_t i = 0U; i < frame.landmark_count; ++i) {
    LandmarkMeasurement measurement = frame.landmarks[i];
    measurement.timestamp_ms = received_at_ms;
    const UpdateInfo result =
        estimator_candidate.update_landmark(measurement);
    failure = failure || update_failed(result);
    numerical_failure =
        numerical_failure || result.result == UpdateResult::NumericalFailure;
  }

  if (numerical_failure) {
    runtime_.report_numerical_fault();
  } else if (failure) {
    runtime_.record_rejected_input(InputDisposition::Invalid);
  } else {
    estimator_ = estimator_candidate;
    runtime_.accept_input(received_at_ms);
  }
  return !failure;
}

ControllerStepOutput EmbeddedControllerCore::make_output(
    uint32_t now_ms, const RuntimeDecision& runtime_decision, float dt_s) {
  ControllerStepOutput output{};
  output.estimate = estimator_.state();
  output.navigation_mode = estimator_.navigation_mode(now_ms);
  output.safety_state = runtime_decision.state;
  output.estimator_initialized = estimator_.initialized();
  output.estimator_healthy = estimator_.healthy();

  if (runtime_decision.force_safe_stop) {
    output.command = controller_.safe_stop_command(output.estimate.speed_mps);
  } else if (runtime_decision.permit_control) {
    output.command = controller_.step(output.estimate, dt_s, false);
  } else {
    output.command = controller_.neutral_command();
  }

  output.hal_status.ready = output.safety_state == SafetyState::Ready;
  output.hal_status.running = output.safety_state == SafetyState::Running;
  output.hal_status.degraded = output.safety_state == SafetyState::Degraded;
  output.hal_status.safe_stop =
      output.safety_state == SafetyState::SafeStop;
  output.hal_status.fault = output.safety_state == SafetyState::Fault;
  return output;
}

ControllerStepOutput EmbeddedControllerCore::step(
    const ControllerStepInput& input) {
  if (input.manual_safe_stop) {
    runtime_.request_safe_stop();
  }
  if (input.has_sensor_frame) {
    (void)process_sensor_frame(input.sensor_frame, input.dt_s, input.now_ms);
  }

  const NavigationMode mode = estimator_.navigation_mode(input.now_ms);
  const RuntimeDecision decision =
      runtime_.tick(input.now_ms, mode, estimator_.healthy(),
                    controller_.route().route_valid());
  return make_output(input.now_ms, decision, input.dt_s);
}

}  // namespace navbench
