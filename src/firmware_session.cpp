#include "navbench/firmware_session.hpp"

#include <cmath>
#include <cstring>

namespace navbench {
namespace {

namespace wire = protocol;

wire::RuntimeState wire_runtime(SafetyState state) {
  return static_cast<wire::RuntimeState>(static_cast<uint8_t>(state));
}

wire::NavigationMode wire_navigation(NavigationMode mode) {
  return static_cast<wire::NavigationMode>(static_cast<uint8_t>(mode));
}

InputDisposition input_disposition(wire::SequenceDisposition disposition) {
  switch (disposition) {
    case wire::SequenceDisposition::Duplicate:
      return InputDisposition::Duplicate;
    case wire::SequenceDisposition::OutOfOrder:
      return InputDisposition::OutOfOrder;
    case wire::SequenceDisposition::Stale:
      return InputDisposition::Stale;
    default:
      return InputDisposition::Accepted;
  }
}

InputDisposition parser_disposition(wire::Status status) {
  return status == wire::Status::OversizedFrame
             ? InputDisposition::Oversized
             : InputDisposition::Corrupt;
}

bool sample_step_is_fresh(uint32_t sample_step, uint32_t frame_step,
                          uint32_t maximum_lag_steps) {
  const uint32_t lag = frame_step - sample_step;
  return lag < 0x80000000UL && lag <= maximum_lag_steps;
}

constexpr uint8_t task_mask(RuntimeTask task) {
  return static_cast<uint8_t>(1U << static_cast<uint8_t>(task));
}

constexpr uint32_t kRequiredHostCapabilities =
    wire::CapabilitySensorFrame | wire::CapabilityRouteChunk |
    wire::CapabilityStateEstimate | wire::CapabilityHealthStatus |
    wire::CapabilitySafeStop;

// HEALTH_STATUS is the largest controller-to-host Protocol-v1 payload.
constexpr uint16_t kControllerMaximumOutboundPayloadSize = 116U;

bool step_is_newer(uint32_t candidate, uint32_t previous) {
  const uint32_t delta = candidate - previous;
  return delta != 0U && delta < 0x80000000UL;
}

}  // namespace

FirmwareSessionConfig FirmwareSessionConfig::defaults() {
  FirmwareSessionConfig config{};
  config.heartbeat_period_ms = 1000U;
  config.health_period_ms = 100U;
  config.maximum_sample_lag_steps = 12U;
  config.nominal_sensor_dt_s = 0.02F;
  return config;
}

bool FirmwareSession::valid_config(const FirmwareSessionConfig& config) {
  return config.heartbeat_period_ms != 0U &&
         config.health_period_ms != 0U &&
         config.health_period_ms <= config.heartbeat_period_ms &&
         config.maximum_sample_lag_steps != 0U &&
         config.maximum_sample_lag_steps <= 10000U &&
         finite(config.nominal_sensor_dt_s) &&
         config.nominal_sensor_dt_s > 0.0F &&
         config.nominal_sensor_dt_s <= 0.25F;
}

FirmwareSession::FirmwareSession()
    : config_(FirmwareSessionConfig::defaults()) {
  reset(0U);
}

FirmwareSession::FirmwareSession(const FirmwareSessionConfig& config)
    : config_(FirmwareSessionConfig::defaults()) {
  if (valid_config(config)) {
    config_ = config;
  }
  reset(0U);
}

bool FirmwareSession::set_config(const FirmwareSessionConfig& config) {
  if (!valid_config(config)) {
    return false;
  }
  config_ = config;
  return true;
}

void FirmwareSession::reset(uint32_t now_ms) {
  core_ = EmbeddedControllerCore{};
  core_.begin(now_ms);
  (void)core_.start_self_test();
  (void)core_.complete_self_test(true, now_ms);
  parser_.reset(true);
  sequence_.reset(true);
  tx_queue_ = FixedQueue<TxFrame, kTxQueueCapacity>{};
  sensor_queue_ = FixedQueue<PendingSensorFrame, kSensorQueueCapacity>{};
  stats_ = FirmwareSessionStats{};
  tx_sequence_ = 0U;
  current_now_ms_ = now_ms;
  session_started_ms_ = now_ms;
  last_step_id_ = 0U;
  last_sensor_frame_ms_ = now_ms;
  last_valid_sensor_ms_ = now_ms;
  last_core_step_ms_ = now_ms;
  last_health_ms_ = now_ms;
  last_heartbeat_ms_ = now_ms;
  route_id_ = 0U;
  route_expected_ = 0U;
  route_received_ = 0U;
  imu_step_ = SampleStepTracker{};
  wheel_step_ = SampleStepTracker{};
  gnss_step_ = SampleStepTracker{};
  landmark_step_ = SampleStepTracker{};
  last_emitted_state_ = SafetyState::Startup;
  session_active_ = false;
  have_sensor_time_ = false;
  have_sensor_ = false;
  scheduled_task_mask_ = 0U;
}

void FirmwareSession::packet_callback(const wire::Packet& packet,
                                      void* context) {
  static_cast<FirmwareSession*>(context)->handle_packet(packet);
}

void FirmwareSession::parser_error_callback(wire::Status status,
                                            void* context) {
  FirmwareSession* session = static_cast<FirmwareSession*>(context);
  session->core_.runtime().record_rejected_input(parser_disposition(status));
  session->emit_error(session->last_step_id_,
                      wire::ApplicationErrorCode::BadPayload,
                      static_cast<uint16_t>(status), 0U);
}

void FirmwareSession::feed(uint32_t now_ms, const uint8_t* data,
                           std::size_t size) {
  current_now_ms_ = now_ms;
  parser_.feed(data, size, &FirmwareSession::packet_callback,
               &FirmwareSession::parser_error_callback, this);
}

void FirmwareSession::tick(uint32_t now_ms, uint32_t step_id) {
  current_now_ms_ = now_ms;
  last_step_id_ = step_id;
  const ScheduleDecision schedule = core_.runtime().scheduler().poll(now_ms);
  const bool control_due =
      schedule.due[static_cast<std::size_t>(RuntimeTask::Control)];
  const bool health_due =
      schedule.due[static_cast<std::size_t>(RuntimeTask::Health)];
  const bool telemetry_due =
      schedule.due[static_cast<std::size_t>(RuntimeTask::Telemetry)];
  if (!control_due && !health_due && !telemetry_due) {
    return;
  }
  if (control_due) {
    scheduled_task_mask_ |= task_mask(RuntimeTask::Control);
  }
  if (health_due) {
    scheduled_task_mask_ |= task_mask(RuntimeTask::Health);
  }
  if (telemetry_due) {
    scheduled_task_mask_ |= task_mask(RuntimeTask::Telemetry);
  }
  // EmbeddedControllerCore intentionally advances estimation and control as a
  // single atomic task. Only a control release may call step(), so duplicate
  // loop polls at the same time cannot integrate PI/control twice.
  if (!control_due || elapsed_ms(now_ms, last_core_step_ms_) == 0U) {
    return;
  }
  ControllerStepInput input{};
  input.now_ms = now_ms;
  const uint32_t elapsed = elapsed_ms(now_ms, last_core_step_ms_);
  input.dt_s = elapsed <= 250U
                   ? static_cast<float>(elapsed) * 0.001F
                   : config_.nominal_sensor_dt_s;
  PendingSensorFrame pending{};
  uint32_t output_step_id = step_id;
  if (sensor_queue_.pop(pending)) {
    input.has_sensor_frame = true;
    input.sensor_frame = pending.frame;
    input.sensor_frame.timestamp_ms = now_ms;
    input.dt_s = sensor_timestep(now_ms);
    output_step_id = pending.packet_step_id;
  }
  const uint32_t accepted_before = core_.runtime().stats().accepted_inputs;
  const ControllerStepOutput output = core_.step(input);
  last_core_step_ms_ = now_ms;
  if (input.has_sensor_frame) {
    ++stats_.sensor_frames_processed;
    if (core_.runtime().stats().accepted_inputs != accepted_before) {
      have_sensor_ = true;
      last_valid_sensor_ms_ = now_ms;
    }
  }
  if (input.has_sensor_frame || health_due || telemetry_due ||
      output.safety_state != last_emitted_state_ ||
      elapsed_ms(now_ms, last_heartbeat_ms_) >=
          config_.heartbeat_period_ms) {
    emit_controller_output(output_step_id, output, health_due);
  }
}

void FirmwareSession::handle_packet(const wire::Packet& packet) {
  // HELLO defines a new protocol session and must be processable even when the
  // peer deliberately restarts its sequence space at zero.
  if (packet.message_type == wire::MessageType::Hello) {
    ++stats_.packets_handled;
    last_step_id_ = packet.step_id;
    handle_hello(packet);
    return;
  }

  if (!session_active_ && packet.message_type != wire::MessageType::SafeStop) {
    ++stats_.packets_handled;
    last_step_id_ = packet.step_id;
    core_.runtime().record_rejected_input(InputDisposition::Invalid);
    emit_error(packet.step_id, wire::ApplicationErrorCode::NotReady,
               static_cast<uint16_t>(packet.message_type), packet.sequence);
    return;
  }

  const bool is_step_bound = packet.message_type == wire::MessageType::SensorFrame;
  const wire::SequenceResult sequence_result =
      is_step_bound ? sequence_.observe(packet.sequence, packet.step_id)
                    : sequence_.observe(packet.sequence);
  if (!sequence_result.accepted) {
    core_.runtime().record_rejected_input(
        input_disposition(sequence_result.disposition));
    return;
  }

  ++stats_.packets_handled;
  last_step_id_ = packet.step_id;
  switch (packet.message_type) {
    case wire::MessageType::RouteChunk:
      handle_route(packet);
      return;
    case wire::MessageType::SensorFrame:
      handle_sensor(packet);
      return;
    case wire::MessageType::SafeStop:
      handle_safe_stop(packet);
      return;
    case wire::MessageType::Heartbeat: {
      wire::HeartbeatPayload heartbeat{};
      if (wire::decodePayload(packet.payload, packet.payload_size,
                              &heartbeat) != wire::Status::Ok) {
        core_.runtime().record_rejected_input(InputDisposition::Invalid);
        emit_error(packet.step_id, wire::ApplicationErrorCode::BadPayload, 0U,
                   packet.sequence);
        return;
      }
      core_.notify_handshake(current_now_ms_);
      return;
    }
    default:
      emit_error(packet.step_id,
                 wire::ApplicationErrorCode::UnsupportedMessage, 0U,
                 packet.sequence);
      return;
  }
}

void FirmwareSession::handle_hello(const wire::Packet& packet) {
  wire::HelloPayload hello{};
  const wire::Status decode_status =
      wire::decodePayload(packet.payload, packet.payload_size, &hello);

  wire::HelloAckPayload response{};
  response.accepted_version = wire::kProtocolVersion;
  response.status = wire::HelloStatus::Ok;
  response.role = wire::EndpointRole::Controller;
  response.capabilities = kRequiredHostCapabilities;
  response.max_payload = static_cast<uint16_t>(wire::kMaxPayloadSize);
  response.heartbeat_timeout_ms =
      static_cast<uint16_t>(RuntimeConfig::defaults().host_timeout_ms);

  if (decode_status != wire::Status::Ok ||
      hello.role != wire::EndpointRole::Host) {
    response.accepted_version = 0U;
    response.status = wire::HelloStatus::Rejected;
  } else if (hello.min_version > wire::kProtocolVersion ||
             hello.max_version < wire::kProtocolVersion) {
    response.accepted_version = 0U;
    response.status = wire::HelloStatus::VersionMismatch;
  } else if ((hello.capabilities & kRequiredHostCapabilities) !=
                 kRequiredHostCapabilities ||
             hello.max_payload < kControllerMaximumOutboundPayloadSize ||
             hello.heartbeat_timeout_ms !=
                 RuntimeConfig::defaults().host_timeout_ms) {
    response.accepted_version = 0U;
    response.status = wire::HelloStatus::Rejected;
  } else {
    stats_.tx_dropped += static_cast<uint32_t>(tx_queue_.size());
    tx_queue_ = FixedQueue<TxFrame, kTxQueueCapacity>{};
    sensor_queue_ =
        FixedQueue<PendingSensorFrame, kSensorQueueCapacity>{};
    tx_sequence_ = 0U;
    sequence_.reset(true);
    (void)sequence_.observe(packet.sequence);
    route_id_ = 0U;
    route_expected_ = 0U;
    route_received_ = 0U;
    session_active_ = true;
    core_.notify_handshake(current_now_ms_);
  }
  (void)emit_payload(wire::MessageType::HelloAck, packet.step_id, response);
}

void FirmwareSession::handle_route(const wire::Packet& packet) {
  wire::RouteChunkPayload route{};
  const wire::Status status =
      wire::decodePayload(packet.payload, packet.payload_size, &route);
  if (status != wire::Status::Ok || route.total_count == 0U ||
      route.total_count > kMaximumWaypoints || route.point_count == 0U ||
      (route.flags & wire::RouteLoop) != 0U ||
      static_cast<uint32_t>(route.start_index) + route.point_count >
          route.total_count) {
    emit_error(packet.step_id, wire::ApplicationErrorCode::RouteRejected,
               static_cast<uint16_t>(status), packet.sequence);
    return;
  }

  if ((route.flags & wire::RouteClearExisting) != 0U) {
    route_id_ = route.route_id;
    route_expected_ = route.total_count;
    route_received_ = 0U;
  }
  if (route.route_id != route_id_ || route.total_count != route_expected_ ||
      route.start_index != route_received_) {
    emit_error(packet.step_id, wire::ApplicationErrorCode::RouteRejected, 0U,
               packet.sequence);
    return;
  }

  for (uint8_t index = 0U; index < route.point_count; ++index) {
    const wire::RouteWaypoint& source = route.points[index];
    Waypoint& target = route_[route_received_ + index];
    target.x_m = source.x_m;
    target.y_m = source.y_m;
    target.target_speed_mps = source.target_speed_mps;
    target.acceptance_radius_m = source.acceptance_radius_m;
  }
  route_received_ =
      static_cast<uint16_t>(route_received_ + route.point_count);

  if ((route.flags & wire::RouteFinalChunk) != 0U &&
      (route_received_ != route_expected_ ||
       !core_.set_route(route_, route_received_))) {
    emit_error(packet.step_id, wire::ApplicationErrorCode::RouteRejected, 0U,
               packet.sequence);
  }
}

float FirmwareSession::sensor_timestep(uint32_t now_ms) {
  float dt_s = config_.nominal_sensor_dt_s;
  if (have_sensor_time_) {
    const uint32_t elapsed = elapsed_ms(now_ms, last_sensor_frame_ms_);
    if (elapsed > 0U && elapsed <= 250U) {
      dt_s = static_cast<float>(elapsed) * 0.001F;
    }
  }
  last_sensor_frame_ms_ = now_ms;
  have_sensor_time_ = true;
  return dt_s;
}

bool FirmwareSession::validate_sample_steps(
    const wire::SensorFramePayload& sensor, uint32_t frame_step,
    InputDisposition* disposition) const {
  if (disposition == nullptr) {
    return false;
  }
  *disposition = InputDisposition::Stale;
  const auto valid = [&](uint16_t mask, uint32_t sample_step,
                         const SampleStepTracker& tracker) -> bool {
    if ((sensor.present_mask & mask) == 0U) {
      return true;
    }
    if (!sample_step_is_fresh(sample_step, frame_step,
                              config_.maximum_sample_lag_steps)) {
      return false;
    }
    if (!tracker.initialized) {
      return true;
    }
    if (sample_step == tracker.last_step_id) {
      *disposition = InputDisposition::Duplicate;
      return false;
    }
    return step_is_newer(sample_step, tracker.last_step_id);
  };

  return valid(wire::SensorImu, sensor.imu.sample_step_id, imu_step_) &&
         valid(wire::SensorWheelSpeed, sensor.wheel_speed.sample_step_id,
               wheel_step_) &&
         valid(wire::SensorGnss, sensor.gnss.sample_step_id, gnss_step_) &&
         valid(wire::SensorLandmark, sensor.landmark.sample_step_id,
               landmark_step_);
}

void FirmwareSession::commit_sample_steps(
    const wire::SensorFramePayload& sensor) {
  const auto commit = [](SampleStepTracker& tracker, uint32_t step_id) {
    tracker.last_step_id = step_id;
    tracker.initialized = true;
  };
  if ((sensor.present_mask & wire::SensorImu) != 0U) {
    commit(imu_step_, sensor.imu.sample_step_id);
  }
  if ((sensor.present_mask & wire::SensorWheelSpeed) != 0U) {
    commit(wheel_step_, sensor.wheel_speed.sample_step_id);
  }
  if ((sensor.present_mask & wire::SensorGnss) != 0U) {
    commit(gnss_step_, sensor.gnss.sample_step_id);
  }
  if ((sensor.present_mask & wire::SensorLandmark) != 0U) {
    commit(landmark_step_, sensor.landmark.sample_step_id);
  }
}

void FirmwareSession::handle_sensor(const wire::Packet& packet) {
  wire::SensorFramePayload sensor{};
  if (wire::decodePayload(packet.payload, packet.payload_size, &sensor) !=
          wire::Status::Ok ||
      sensor.present_mask == 0U) {
    core_.runtime().record_rejected_input(InputDisposition::Invalid);
    emit_error(packet.step_id, wire::ApplicationErrorCode::BadPayload, 0U,
               packet.sequence);
    return;
  }

  InputDisposition disposition = InputDisposition::Stale;
  if (!validate_sample_steps(sensor, packet.step_id, &disposition)) {
    core_.runtime().record_rejected_input(disposition);
    emit_error(packet.step_id, wire::ApplicationErrorCode::BadPayload, 0U,
               packet.sequence);
    return;
  }

  PendingSensorFrame pending{};
  pending.packet_step_id = packet.step_id;
  pending.frame.timestamp_ms = current_now_ms_;
  pending.frame.step_id = packet.step_id;
  pending.frame.disposition = InputDisposition::Accepted;

  if ((sensor.present_mask & wire::SensorImu) != 0U) {
    pending.frame.has_imu = true;
    pending.frame.imu.longitudinal_accel_mps2 =
        sensor.imu.longitudinal_acceleration_mps2;
    pending.frame.imu.yaw_rate_rad_s = sensor.imu.yaw_rate_rps;
    pending.frame.imu.timestamp_ms = sensor.imu.timestamp_us / 1000U;
    pending.frame.imu.step_id = sensor.imu.sample_step_id;
  }
  if ((sensor.present_mask & wire::SensorWheelSpeed) != 0U) {
    pending.frame.has_wheel_speed = true;
    pending.frame.wheel_speed.speed_mps = sensor.wheel_speed.speed_mps;
    pending.frame.wheel_speed.timestamp_ms =
        sensor.wheel_speed.timestamp_us / 1000U;
    pending.frame.wheel_speed.step_id =
        sensor.wheel_speed.sample_step_id;
  }
  if ((sensor.present_mask & wire::SensorGnss) != 0U) {
    pending.frame.has_gnss = true;
    pending.frame.gnss.x_m = sensor.gnss.x_m;
    pending.frame.gnss.y_m = sensor.gnss.y_m;
    pending.frame.gnss.timestamp_ms = sensor.gnss.timestamp_us / 1000U;
    pending.frame.gnss.step_id = sensor.gnss.sample_step_id;
  }
  if ((sensor.present_mask & wire::SensorLandmark) != 0U) {
    pending.frame.landmark_count = 1U;
    LandmarkMeasurement& landmark = pending.frame.landmarks[0];
    landmark.landmark_id = sensor.landmark.landmark_id;
    landmark.landmark_x_m = sensor.landmark.landmark_x_m;
    landmark.landmark_y_m = sensor.landmark.landmark_y_m;
    landmark.range_m = sensor.landmark.range_m;
    landmark.bearing_rad = sensor.landmark.bearing_rad;
    landmark.timestamp_ms = sensor.landmark.timestamp_us / 1000U;
    landmark.step_id = sensor.landmark.sample_step_id;
  }

  if (!sensor_queue_.push(pending)) {
    ++stats_.sensor_queue_overflows;
    core_.runtime().record_queue_overflow();
    emit_error(packet.step_id, wire::ApplicationErrorCode::InternalFault,
               static_cast<uint16_t>(kSensorQueueCapacity), packet.sequence);
    return;
  }
  commit_sample_steps(sensor);
  ++stats_.sensor_frames_enqueued;
}

void FirmwareSession::handle_safe_stop(const wire::Packet& packet) {
  wire::SafeStopPayload payload{};
  if (wire::decodePayload(packet.payload, packet.payload_size, &payload) !=
          wire::Status::Ok ||
      !payload.latch) {
    core_.runtime().record_rejected_input(InputDisposition::Invalid);
    emit_error(packet.step_id, wire::ApplicationErrorCode::BadPayload, 0U,
               packet.sequence);
    return;
  }

  ControllerStepInput input{};
  input.now_ms = current_now_ms_;
  input.dt_s = config_.nominal_sensor_dt_s;
  input.manual_safe_stop = true;
  last_core_step_ms_ = current_now_ms_;
  scheduled_task_mask_ |= task_mask(RuntimeTask::Control);
  const ControllerStepOutput output = core_.step(input);
  emit_controller_output(packet.step_id, output, true);
}

bool FirmwareSession::reserve_controller_bundle(std::size_t required_slots) {
  if (required_slots <= tx_queue_.capacity() - tx_queue_.size()) {
    return true;
  }
  stats_.tx_dropped += static_cast<uint32_t>(tx_queue_.size());
  core_.runtime().record_queue_overflow();
  tx_queue_.clear();
  return false;
}

void FirmwareSession::emit_controller_output(
    uint32_t step_id, const ControllerStepOutput& output, bool force_health) {
  bool health_due = force_health || output.safety_state != last_emitted_state_ ||
                    elapsed_ms(current_now_ms_, last_health_ms_) >=
                        config_.health_period_ms;
  const bool heartbeat_due =
      elapsed_ms(current_now_ms_, last_heartbeat_ms_) >=
      config_.heartbeat_period_ms;
  const std::size_t required = 2U + (health_due ? 1U : 0U) +
                               (heartbeat_due ? 1U : 0U);

  ControllerStepOutput effective = output;
  if (!reserve_controller_bundle(required)) {
    ControllerStepInput safe_input{};
    safe_input.now_ms = current_now_ms_;
    safe_input.dt_s = config_.nominal_sensor_dt_s;
    effective = core_.step(safe_input);
    health_due = true;
  }

  wire::ControlCommandPayload command{};
  command.steering_rad = effective.command.steering_rad;
  command.acceleration_mps2 = effective.command.acceleration_mps2;
  command.target_speed_mps = effective.command.target_speed_mps;
  command.mode = effective.command.safe_stop
                     ? wire::ControlMode::SafeStop
                     : (effective.command.valid ? wire::ControlMode::Tracking
                                                : wire::ControlMode::Neutral);
  command.flags = effective.command.route_complete ? 1U : 0U;
  (void)emit_payload(wire::MessageType::ControlCommand, step_id, command);

  wire::StateEstimatePayload estimate{};
  estimate.x_m = effective.estimate.x_m;
  estimate.y_m = effective.estimate.y_m;
  estimate.heading_rad = effective.estimate.heading_rad;
  estimate.speed_mps = effective.estimate.speed_mps;
  estimate.yaw_rate_rps = effective.estimate.yaw_rate_rad_s;
  estimate.acceleration_bias_mps2 = effective.estimate.accel_bias_mps2;
  float covariance[kEkfStateSize * kEkfStateSize]{};
  core_.estimator().covariance(covariance);
  for (std::size_t index = 0U; index < kEkfStateSize; ++index) {
    estimate.covariance_diagonal[index] =
        covariance[index * kEkfStateSize + index];
  }
  estimate.navigation_mode = wire_navigation(effective.navigation_mode);
  estimate.flags = effective.estimator_healthy ? 1U : 0U;
  (void)emit_payload(wire::MessageType::StateEstimate, step_id, estimate);

  if (health_due) {
    wire::HealthStatusPayload health{};
    health.runtime_state = wire_runtime(effective.safety_state);
    health.navigation_mode = wire_navigation(effective.navigation_mode);
    health.flags = effective.estimator_healthy ? 1U : 0U;
    health.uptime_ms = elapsed_ms(current_now_ms_, session_started_ms_);
    health.last_sensor_age_ms =
        have_sensor_ ? elapsed_ms(current_now_ms_, last_valid_sensor_ms_)
                     : 0xffffffffUL;
    const wire::ParserStats& parser_stats_value = parser_.stats();
    const wire::SequenceStats& sequence_stats_value = sequence_.stats();
    health.rx_frames = parser_stats_value.frames_received;
    health.rx_crc_errors = parser_stats_value.crc_errors;
    health.rx_decode_errors =
        parser_stats_value.cobs_errors + parser_stats_value.version_errors +
        parser_stats_value.type_errors + parser_stats_value.length_errors +
        parser_stats_value.other_errors;
    health.rx_missing = sequence_stats_value.missing;
    health.rx_duplicates = sequence_stats_value.duplicates;
    health.rx_out_of_order = sequence_stats_value.out_of_order;
    health.rx_stale = sequence_stats_value.stale;
    health.queue_overflows = core_.runtime().stats().queue_overflows;
    health.scheduler_overruns = 0U;
    for (std::size_t index = 0U; index < kRuntimeTaskCount; ++index) {
      health.scheduler_overruns +=
          core_.runtime().scheduler().timing(
              static_cast<RuntimeTask>(index)).overruns;
    }
    health.max_loop_us = stats_.max_loop_us;
    const EkfStats& ekf_stats = core_.estimator().stats();
    health.imu_yaw_nis_evaluated_count =
        ekf_stats.imu_yaw_nis.evaluated_count;
    health.imu_yaw_nis_gate_rejected_count =
        ekf_stats.imu_yaw_nis.gate_rejected_count;
    health.imu_yaw_nis_sum = ekf_stats.imu_yaw_nis.nis_sum;
    health.imu_yaw_nis_max = ekf_stats.imu_yaw_nis.nis_max;
    health.wheel_nis_evaluated_count =
        ekf_stats.wheel_nis.evaluated_count;
    health.wheel_nis_gate_rejected_count =
        ekf_stats.wheel_nis.gate_rejected_count;
    health.wheel_nis_sum = ekf_stats.wheel_nis.nis_sum;
    health.wheel_nis_max = ekf_stats.wheel_nis.nis_max;
    health.gnss_nis_evaluated_count = ekf_stats.gnss_nis.evaluated_count;
    health.gnss_nis_gate_rejected_count =
        ekf_stats.gnss_nis.gate_rejected_count;
    health.gnss_nis_sum = ekf_stats.gnss_nis.nis_sum;
    health.gnss_nis_max = ekf_stats.gnss_nis.nis_max;
    health.landmark_nis_evaluated_count =
        ekf_stats.landmark_nis.evaluated_count;
    health.landmark_nis_gate_rejected_count =
        ekf_stats.landmark_nis.gate_rejected_count;
    health.landmark_nis_sum = ekf_stats.landmark_nis.nis_sum;
    health.landmark_nis_max = ekf_stats.landmark_nis.nis_max;
    (void)emit_payload(wire::MessageType::HealthStatus, step_id, health);
    last_health_ms_ = current_now_ms_;
    last_emitted_state_ = effective.safety_state;
  }

  if (heartbeat_due) {
    wire::HeartbeatPayload heartbeat{};
    heartbeat.uptime_ms = elapsed_ms(current_now_ms_, session_started_ms_);
    heartbeat.monotonic_ms = current_now_ms_;
    heartbeat.runtime_state = wire_runtime(effective.safety_state);
    (void)emit_payload(wire::MessageType::Heartbeat, step_id, heartbeat);
    last_heartbeat_ms_ = current_now_ms_;
  }
}

void FirmwareSession::emit_error(uint32_t step_id,
                                 wire::ApplicationErrorCode code,
                                 uint16_t detail,
                                 uint32_t related_sequence) {
  ++stats_.application_errors;
  wire::ErrorPayload error{};
  error.code = code;
  error.detail = detail;
  error.related_sequence = related_sequence;
  error.context = 0U;
  (void)emit_payload(wire::MessageType::Error, step_id, error);
}

template <typename Payload>
bool FirmwareSession::emit_payload(wire::MessageType type, uint32_t step_id,
                                   const Payload& payload) {
  uint8_t encoded_payload[wire::kMaxPayloadSize]{};
  uint16_t payload_size = 0U;
  if (wire::encodePayload(payload, encoded_payload, sizeof(encoded_payload),
                          &payload_size) != wire::Status::Ok) {
    return false;
  }

  TxFrame frame{};
  std::size_t frame_size = 0U;
  if (wire::encodePacket(type, tx_sequence_, step_id, encoded_payload,
                         payload_size, frame.bytes, sizeof(frame.bytes),
                         &frame_size) != wire::Status::Ok) {
    return false;
  }
  frame.size = static_cast<uint16_t>(frame_size);
  if (!enqueue(frame)) {
    return false;
  }
  ++tx_sequence_;
  return true;
}

bool FirmwareSession::enqueue(const TxFrame& frame) {
  if (!tx_queue_.push(frame)) {
    stats_.tx_dropped += static_cast<uint32_t>(tx_queue_.size());
    core_.runtime().record_queue_overflow();
    tx_queue_.clear();
    if (!tx_queue_.push(frame)) {
      ++stats_.tx_dropped;
      return false;
    }
  }
  ++stats_.tx_frames;
  return true;
}

bool FirmwareSession::pop_frame(uint8_t* output, std::size_t capacity,
                                std::size_t* output_size) {
  if (output_size == nullptr) {
    return false;
  }
  *output_size = 0U;
  TxFrame frame{};
  if (!tx_queue_.peek(frame)) {
    return false;
  }
  if (output == nullptr || capacity < frame.size) {
    return false;
  }
  (void)tx_queue_.pop(frame);
  std::memcpy(output, frame.bytes, frame.size);
  *output_size = frame.size;
  return true;
}

void FirmwareSession::record_loop_duration(uint32_t duration_us) {
  ++stats_.loop_runs;
  if (duration_us > stats_.max_loop_us) {
    stats_.max_loop_us = duration_us;
  }
  for (std::size_t index = 0U; index < kRuntimeTaskCount; ++index) {
    const RuntimeTask task = static_cast<RuntimeTask>(index);
    if ((scheduled_task_mask_ & task_mask(task)) != 0U) {
      core_.runtime().scheduler().record_execution(task, duration_us);
    }
  }
  scheduled_task_mask_ = 0U;
}

}  // namespace navbench
