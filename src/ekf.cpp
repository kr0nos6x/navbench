#include "navbench/ekf.hpp"

#include <cmath>
#include <cstring>

#include "navbench/math.hpp"

namespace navbench {
namespace {

constexpr std::size_t kN = kEkfStateSize;

constexpr std::size_t index(std::size_t row, std::size_t column) {
  return row * kN + column;
}

bool positive_finite(float value) {
  return finite(value) && value > 0.0F;
}

bool nonnegative_finite(float value) {
  return finite(value) && value >= 0.0F;
}

}  // namespace

void NisAccumulator::reset() {
  evaluated_count = 0U;
  gate_rejected_count = 0U;
  nis_sum = 0.0F;
  nis_max = 0.0F;
}

bool NisAccumulator::record(bool gate_rejected, float nis) {
  if (!finite(nis) || nis < 0.0F) {
    return false;
  }

  const bool empty_is_consistent =
      evaluated_count != 0U ||
      (gate_rejected_count == 0U && nis_sum == 0.0F && nis_max == 0.0F);
  const bool state_is_valid =
      gate_rejected_count <= evaluated_count && finite(nis_sum) &&
      finite(nis_max) && nis_sum >= 0.0F && nis_max >= 0.0F &&
      empty_is_consistent &&
      (evaluated_count == 0U || nis_max <= nis_sum);

  bool rollover = !state_is_valid || evaluated_count == 0xffffffffUL;
  float next_sum = nis;
  if (!rollover) {
    next_sum = nis_sum + nis;
    rollover = !finite(next_sum);
  }
  if (rollover) {
    reset();
    next_sum = nis;
  }

  ++evaluated_count;
  if (gate_rejected) {
    ++gate_rejected_count;
  }
  nis_sum = next_sum;
  if (evaluated_count == 1U || nis > nis_max) {
    nis_max = nis;
  }
  return true;
}

EkfConfig EkfConfig::defaults() {
  EkfConfig config{};
  config.initial_variance[0] = 4.0F;
  config.initial_variance[1] = 4.0F;
  config.initial_variance[2] = 0.25F;
  config.initial_variance[3] = 1.0F;
  config.initial_variance[4] = 0.09F;
  config.initial_variance[5] = 0.25F;

  config.process_noise_per_second[0] = 0.0025F;
  config.process_noise_per_second[1] = 0.0025F;
  config.process_noise_per_second[2] = 0.001F;
  config.process_noise_per_second[3] = 0.04F;
  // Yaw rate is driven by an unmodelled steering actuator.  With the vehicle
  // limits used by NavBench its derivative can reach several rad/s^2; treating
  // it as nearly constant makes a healthy, fast turn look like an IMU outlier.
  config.process_noise_per_second[4] = 1.0F;
  config.process_noise_per_second[5] = 0.0001F;

  config.imu_yaw_rate_variance = 0.0025F;
  config.wheel_speed_variance = 0.04F;
  config.gnss_x_variance = 1.0F;
  config.gnss_y_variance = 1.0F;
  config.landmark_range_variance = 0.09F;
  config.landmark_bearing_variance = 0.0025F;

  // Chi-square thresholds near 99%: one degree of freedom and two degrees.
  config.imu_yaw_nis_gate = 6.635F;
  config.wheel_nis_gate = 6.635F;
  config.gnss_nis_gate = 9.210F;
  config.landmark_nis_gate = 9.210F;

  config.minimum_landmark_range_m = 0.05F;
  config.maximum_landmark_range_m = 10000.0F;
  config.maximum_abs_position_m = 1000000.0F;
  config.maximum_abs_speed_mps = 50.0F;
  config.maximum_abs_acceleration_mps2 = 100.0F;
  config.maximum_abs_yaw_rate_rad_s = 20.0F;
  config.maximum_prediction_dt_s = 0.25F;
  config.covariance_floor = 1.0e-8F;
  config.maximum_covariance = 1.0e6F;
  config.imu_timeout_ms = 250U;
  config.wheel_timeout_ms = 500U;
  config.gnss_timeout_ms = 1500U;
  config.landmark_timeout_ms = 1000U;
  return config;
}

Ekf6::Ekf6() : config_(EkfConfig::defaults()) {
  reset();
}

Ekf6::Ekf6(const EkfConfig& config) : config_(EkfConfig::defaults()) {
  if (validate_config(config)) {
    config_ = config;
  }
  reset();
}

bool Ekf6::validate_config(const EkfConfig& config) const {
  for (std::size_t i = 0U; i < kN; ++i) {
    if (!positive_finite(config.initial_variance[i]) ||
        !nonnegative_finite(config.process_noise_per_second[i])) {
      return false;
    }
  }

  return positive_finite(config.imu_yaw_rate_variance) &&
         positive_finite(config.wheel_speed_variance) &&
         positive_finite(config.gnss_x_variance) &&
         positive_finite(config.gnss_y_variance) &&
         positive_finite(config.landmark_range_variance) &&
         positive_finite(config.landmark_bearing_variance) &&
         positive_finite(config.imu_yaw_nis_gate) &&
         positive_finite(config.wheel_nis_gate) &&
         positive_finite(config.gnss_nis_gate) &&
         positive_finite(config.landmark_nis_gate) &&
         positive_finite(config.minimum_landmark_range_m) &&
         positive_finite(config.maximum_landmark_range_m) &&
         config.maximum_landmark_range_m > config.minimum_landmark_range_m &&
         positive_finite(config.maximum_abs_position_m) &&
         positive_finite(config.maximum_abs_speed_mps) &&
         positive_finite(config.maximum_abs_acceleration_mps2) &&
         positive_finite(config.maximum_abs_yaw_rate_rad_s) &&
         positive_finite(config.maximum_prediction_dt_s) &&
         positive_finite(config.covariance_floor) &&
         positive_finite(config.maximum_covariance) &&
         config.maximum_covariance > config.covariance_floor &&
         config.imu_timeout_ms > 0U && config.wheel_timeout_ms > 0U &&
         config.gnss_timeout_ms > 0U && config.landmark_timeout_ms > 0U;
}

bool Ekf6::set_config(const EkfConfig& config) {
  if (!validate_config(config)) {
    return false;
  }
  config_ = config;
  reset();
  return true;
}

void Ekf6::reset() {
  std::memset(state_, 0, sizeof(state_));
  std::memset(covariance_, 0, sizeof(covariance_));
  stats_ = EkfStats{};
  initialized_ = false;
  healthy_ = false;
  seen_imu_ = false;
  seen_wheel_ = false;
  seen_gnss_ = false;
  seen_landmark_ = false;
  last_imu_ms_ = 0U;
  last_wheel_ms_ = 0U;
  last_gnss_ms_ = 0U;
  last_landmark_ms_ = 0U;
}

bool Ekf6::initialize(const EkfState& initial_state, uint32_t timestamp_ms) {
  return initialize(initial_state, config_.initial_variance, timestamp_ms);
}

bool Ekf6::initialize(const EkfState& initial_state,
                      const float covariance_diagonal[kEkfStateSize],
                      uint32_t timestamp_ms) {
  (void)timestamp_ms;
  const float values[kN] = {
      initial_state.x_m,          initial_state.y_m,
      initial_state.heading_rad, initial_state.speed_mps,
      initial_state.yaw_rate_rad_s, initial_state.accel_bias_mps2,
  };
  for (std::size_t i = 0U; i < kN; ++i) {
    if (!finite(values[i]) || !positive_finite(covariance_diagonal[i]) ||
        covariance_diagonal[i] > config_.maximum_covariance) {
      return false;
    }
  }
  if (std::fabs(initial_state.x_m) > config_.maximum_abs_position_m ||
      std::fabs(initial_state.y_m) > config_.maximum_abs_position_m ||
      std::fabs(initial_state.heading_rad) > kPi ||
      std::fabs(initial_state.speed_mps) > config_.maximum_abs_speed_mps ||
      std::fabs(initial_state.yaw_rate_rad_s) >
          config_.maximum_abs_yaw_rate_rad_s ||
      std::fabs(initial_state.accel_bias_mps2) >
          config_.maximum_abs_acceleration_mps2) {
    return false;
  }

  set_state(initial_state);
  state_[static_cast<std::size_t>(StateIndex::Heading)] =
      normalize_angle(state_[static_cast<std::size_t>(StateIndex::Heading)]);
  std::memset(covariance_, 0, sizeof(covariance_));
  for (std::size_t i = 0U; i < kN; ++i) {
    covariance_[index(i, i)] = covariance_diagonal[i];
  }
  stats_ = EkfStats{};
  initialized_ = true;
  healthy_ = true;
  seen_imu_ = false;
  seen_wheel_ = false;
  seen_gnss_ = false;
  seen_landmark_ = false;
  return enforce_covariance_health();
}

void Ekf6::set_state(const EkfState& state) {
  state_[0] = state.x_m;
  state_[1] = state.y_m;
  state_[2] = state.heading_rad;
  state_[3] = state.speed_mps;
  state_[4] = state.yaw_rate_rad_s;
  state_[5] = state.accel_bias_mps2;
}

EkfState Ekf6::state() const {
  EkfState result{};
  result.x_m = state_[0];
  result.y_m = state_[1];
  result.heading_rad = state_[2];
  result.speed_mps = state_[3];
  result.yaw_rate_rad_s = state_[4];
  result.accel_bias_mps2 = state_[5];
  return result;
}

void Ekf6::covariance(float output[kEkfStateSize * kEkfStateSize]) const {
  std::memcpy(output, covariance_, sizeof(covariance_));
}

bool Ekf6::prediction_model(
    const EkfState& state, const ImuMeasurement& measurement, float dt_s,
    EkfState& predicted, float jacobian[kEkfStateSize * kEkfStateSize]) {
  if (!finite(state.x_m) || !finite(state.y_m) ||
      !finite(state.heading_rad) || !finite(state.speed_mps) ||
      !finite(state.yaw_rate_rad_s) || !finite(state.accel_bias_mps2) ||
      !finite(measurement.longitudinal_accel_mps2) ||
      !finite(measurement.yaw_rate_rad_s) || !positive_finite(dt_s)) {
    return false;
  }

  std::memset(jacobian, 0, sizeof(float) * kN * kN);
  for (std::size_t i = 0U; i < kN; ++i) {
    jacobian[index(i, i)] = 1.0F;
  }

  const float heading = state.heading_rad;
  const float cosine = std::cos(heading);
  const float sine = std::sin(heading);
  const float dt_squared = dt_s * dt_s;
  const float acceleration =
      measurement.longitudinal_accel_mps2 - state.accel_bias_mps2;
  const float distance = state.speed_mps * dt_s +
                         0.5F * acceleration * dt_squared;

  predicted = state;
  predicted.x_m += distance * cosine;
  predicted.y_m += distance * sine;
  predicted.heading_rad =
      normalize_angle(state.heading_rad + state.yaw_rate_rad_s * dt_s);
  predicted.speed_mps += acceleration * dt_s;

  jacobian[index(0U, 2U)] = -distance * sine;
  jacobian[index(0U, 3U)] = dt_s * cosine;
  jacobian[index(0U, 5U)] = -0.5F * dt_squared * cosine;
  jacobian[index(1U, 2U)] = distance * cosine;
  jacobian[index(1U, 3U)] = dt_s * sine;
  jacobian[index(1U, 5U)] = -0.5F * dt_squared * sine;
  jacobian[index(2U, 4U)] = dt_s;
  jacobian[index(3U, 5U)] = -dt_s;

  return finite(predicted.x_m) && finite(predicted.y_m) &&
         finite(predicted.heading_rad) && finite(predicted.speed_mps);
}

UpdateInfo Ekf6::predict(const ImuMeasurement& measurement, float dt_s) {
  if (!initialized_) {
    return {UpdateResult::NotInitialized, 0.0F};
  }
  if (!healthy_) {
    return {UpdateResult::NumericalFailure, 0.0F};
  }
  if (!positive_finite(dt_s) || dt_s > config_.maximum_prediction_dt_s ||
      !finite(measurement.longitudinal_accel_mps2) ||
      !finite(measurement.yaw_rate_rad_s) ||
      std::fabs(measurement.longitudinal_accel_mps2) >
          config_.maximum_abs_acceleration_mps2 ||
      std::fabs(measurement.yaw_rate_rad_s) >
          config_.maximum_abs_yaw_rate_rad_s) {
    ++stats_.invalid_measurements;
    return {UpdateResult::InvalidMeasurement, 0.0F};
  }

  float jacobian[kN * kN]{};
  EkfState predicted{};
  if (!prediction_model(state(), measurement, dt_s, predicted, jacobian)) {
    ++stats_.invalid_measurements;
    return {UpdateResult::InvalidMeasurement, 0.0F};
  }

  float intermediate[kN * kN]{};
  float predicted_covariance[kN * kN]{};
  for (std::size_t row = 0U; row < kN; ++row) {
    for (std::size_t column = 0U; column < kN; ++column) {
      float sum = 0.0F;
      for (std::size_t inner = 0U; inner < kN; ++inner) {
        sum += jacobian[index(row, inner)] *
               covariance_[index(inner, column)];
      }
      intermediate[index(row, column)] = sum;
    }
  }
  for (std::size_t row = 0U; row < kN; ++row) {
    for (std::size_t column = 0U; column < kN; ++column) {
      float sum = 0.0F;
      for (std::size_t inner = 0U; inner < kN; ++inner) {
        sum += intermediate[index(row, inner)] *
               jacobian[index(column, inner)];
      }
      predicted_covariance[index(row, column)] = sum;
    }
    predicted_covariance[index(row, row)] +=
        config_.process_noise_per_second[row] * dt_s;
  }

  float previous_state[kN]{};
  float previous_covariance[kN * kN]{};
  std::memcpy(previous_state, state_, sizeof(state_));
  std::memcpy(previous_covariance, covariance_, sizeof(covariance_));
  set_state(predicted);
  std::memcpy(covariance_, predicted_covariance, sizeof(covariance_));
  if (!enforce_covariance_health()) {
    std::memcpy(state_, previous_state, sizeof(state_));
    std::memcpy(covariance_, previous_covariance, sizeof(covariance_));
    healthy_ = false;
    ++stats_.numerical_failures;
    return {UpdateResult::NumericalFailure, 0.0F};
  }

  ++stats_.predictions;
  seen_imu_ = true;
  last_imu_ms_ = measurement.timestamp_ms;

  float h[kN]{};
  h[static_cast<std::size_t>(StateIndex::YawRate)] = 1.0F;
  const float innovation =
      measurement.yaw_rate_rad_s -
      state_[static_cast<std::size_t>(StateIndex::YawRate)];
  const UpdateInfo yaw_update =
      scalar_update(innovation, h, config_.imu_yaw_rate_variance,
                    config_.imu_yaw_nis_gate);
  if (yaw_update.result == UpdateResult::Accepted) {
    (void)stats_.imu_yaw_nis.record(false, yaw_update.nis);
    ++stats_.imu_yaw_accepted;
  } else if (yaw_update.result == UpdateResult::RejectedGate) {
    (void)stats_.imu_yaw_nis.record(true, yaw_update.nis);
    ++stats_.imu_yaw_rejected;
  } else if (yaw_update.result == UpdateResult::NumericalFailure) {
    ++stats_.numerical_failures;
  }
  return yaw_update;
}

UpdateInfo Ekf6::scalar_update(float innovation, const float h[kN],
                               float variance, float gate) {
  if (!finite(innovation) || !positive_finite(variance) ||
      !positive_finite(gate)) {
    return {UpdateResult::InvalidMeasurement, 0.0F};
  }

  float covariance_h[kN]{};
  for (std::size_t row = 0U; row < kN; ++row) {
    for (std::size_t column = 0U; column < kN; ++column) {
      covariance_h[row] += covariance_[index(row, column)] * h[column];
    }
  }
  float innovation_variance = variance;
  for (std::size_t i = 0U; i < kN; ++i) {
    innovation_variance += h[i] * covariance_h[i];
  }
  if (!positive_finite(innovation_variance)) {
    return {UpdateResult::NumericalFailure, 0.0F};
  }

  const float nis = innovation * innovation / innovation_variance;
  if (!finite(nis)) {
    return {UpdateResult::NumericalFailure, 0.0F};
  }
  if (nis > gate) {
    return {UpdateResult::RejectedGate, nis};
  }

  float gain[kN]{};
  for (std::size_t i = 0U; i < kN; ++i) {
    gain[i] = covariance_h[i] / innovation_variance;
  }

  float previous_state[kN]{};
  float previous_covariance[kN * kN]{};
  std::memcpy(previous_state, state_, sizeof(state_));
  std::memcpy(previous_covariance, covariance_, sizeof(covariance_));

  for (std::size_t i = 0U; i < kN; ++i) {
    state_[i] += gain[i] * innovation;
  }
  state_[static_cast<std::size_t>(StateIndex::Heading)] = normalize_angle(
      state_[static_cast<std::size_t>(StateIndex::Heading)]);

  float residual_transform[kN * kN]{};
  for (std::size_t row = 0U; row < kN; ++row) {
    for (std::size_t column = 0U; column < kN; ++column) {
      residual_transform[index(row, column)] =
          (row == column ? 1.0F : 0.0F) - gain[row] * h[column];
    }
  }

  float intermediate[kN * kN]{};
  float updated_covariance[kN * kN]{};
  for (std::size_t row = 0U; row < kN; ++row) {
    for (std::size_t column = 0U; column < kN; ++column) {
      for (std::size_t inner = 0U; inner < kN; ++inner) {
        intermediate[index(row, column)] +=
            residual_transform[index(row, inner)] *
            covariance_[index(inner, column)];
      }
    }
  }
  for (std::size_t row = 0U; row < kN; ++row) {
    for (std::size_t column = 0U; column < kN; ++column) {
      for (std::size_t inner = 0U; inner < kN; ++inner) {
        updated_covariance[index(row, column)] +=
            intermediate[index(row, inner)] *
            residual_transform[index(column, inner)];
      }
      updated_covariance[index(row, column)] +=
          gain[row] * variance * gain[column];
    }
  }
  std::memcpy(covariance_, updated_covariance, sizeof(covariance_));

  if (!enforce_covariance_health()) {
    std::memcpy(state_, previous_state, sizeof(state_));
    std::memcpy(covariance_, previous_covariance, sizeof(covariance_));
    healthy_ = false;
    return {UpdateResult::NumericalFailure, nis};
  }
  return {UpdateResult::Accepted, nis};
}

UpdateInfo Ekf6::vector2_update(const float innovation_input[2],
                                const float h[2 * kN],
                                const float variance[4], float gate,
                                bool normalize_second_innovation) {
  float innovation[2] = {innovation_input[0], innovation_input[1]};
  if (normalize_second_innovation) {
    innovation[1] = normalize_angle(innovation[1]);
  }
  if (!finite(innovation[0]) || !finite(innovation[1]) ||
      !positive_finite(variance[0]) || !positive_finite(variance[3]) ||
      !finite(variance[1]) || !finite(variance[2]) ||
      !positive_finite(gate)) {
    return {UpdateResult::InvalidMeasurement, 0.0F};
  }

  float covariance_h_transpose[kN * 2U]{};
  for (std::size_t row = 0U; row < kN; ++row) {
    for (std::size_t measurement = 0U; measurement < 2U; ++measurement) {
      for (std::size_t inner = 0U; inner < kN; ++inner) {
        covariance_h_transpose[row * 2U + measurement] +=
            covariance_[index(row, inner)] * h[measurement * kN + inner];
      }
    }
  }

  float innovation_covariance[4] = {variance[0], variance[1], variance[2],
                                    variance[3]};
  for (std::size_t row = 0U; row < 2U; ++row) {
    for (std::size_t column = 0U; column < 2U; ++column) {
      for (std::size_t inner = 0U; inner < kN; ++inner) {
        innovation_covariance[row * 2U + column] +=
            h[row * kN + inner] *
            covariance_h_transpose[inner * 2U + column];
      }
    }
  }

  float innovation_covariance_inverse[4]{};
  if (!invert_symmetric_2x2(innovation_covariance,
                            innovation_covariance_inverse)) {
    return {UpdateResult::NumericalFailure, 0.0F};
  }
  const float weighted0 =
      innovation_covariance_inverse[0] * innovation[0] +
      innovation_covariance_inverse[1] * innovation[1];
  const float weighted1 =
      innovation_covariance_inverse[2] * innovation[0] +
      innovation_covariance_inverse[3] * innovation[1];
  const float nis = innovation[0] * weighted0 + innovation[1] * weighted1;
  if (!finite(nis) || nis < 0.0F) {
    return {UpdateResult::NumericalFailure, 0.0F};
  }
  if (nis > gate) {
    return {UpdateResult::RejectedGate, nis};
  }

  float gain[kN * 2U]{};
  for (std::size_t row = 0U; row < kN; ++row) {
    for (std::size_t column = 0U; column < 2U; ++column) {
      gain[row * 2U + column] =
          covariance_h_transpose[row * 2U] *
              innovation_covariance_inverse[column] +
          covariance_h_transpose[row * 2U + 1U] *
              innovation_covariance_inverse[2U + column];
    }
  }

  float previous_state[kN]{};
  float previous_covariance[kN * kN]{};
  std::memcpy(previous_state, state_, sizeof(state_));
  std::memcpy(previous_covariance, covariance_, sizeof(covariance_));

  for (std::size_t row = 0U; row < kN; ++row) {
    state_[row] += gain[row * 2U] * innovation[0] +
                   gain[row * 2U + 1U] * innovation[1];
  }
  state_[static_cast<std::size_t>(StateIndex::Heading)] = normalize_angle(
      state_[static_cast<std::size_t>(StateIndex::Heading)]);

  float residual_transform[kN * kN]{};
  for (std::size_t row = 0U; row < kN; ++row) {
    for (std::size_t column = 0U; column < kN; ++column) {
      float gain_h = 0.0F;
      for (std::size_t measurement = 0U; measurement < 2U; ++measurement) {
        gain_h += gain[row * 2U + measurement] *
                  h[measurement * kN + column];
      }
      residual_transform[index(row, column)] =
          (row == column ? 1.0F : 0.0F) - gain_h;
    }
  }

  float intermediate[kN * kN]{};
  float updated_covariance[kN * kN]{};
  for (std::size_t row = 0U; row < kN; ++row) {
    for (std::size_t column = 0U; column < kN; ++column) {
      for (std::size_t inner = 0U; inner < kN; ++inner) {
        intermediate[index(row, column)] +=
            residual_transform[index(row, inner)] *
            covariance_[index(inner, column)];
      }
    }
  }
  for (std::size_t row = 0U; row < kN; ++row) {
    for (std::size_t column = 0U; column < kN; ++column) {
      for (std::size_t inner = 0U; inner < kN; ++inner) {
        updated_covariance[index(row, column)] +=
            intermediate[index(row, inner)] *
            residual_transform[index(column, inner)];
      }
      for (std::size_t left = 0U; left < 2U; ++left) {
        for (std::size_t right = 0U; right < 2U; ++right) {
          updated_covariance[index(row, column)] +=
              gain[row * 2U + left] * variance[left * 2U + right] *
              gain[column * 2U + right];
        }
      }
    }
  }
  std::memcpy(covariance_, updated_covariance, sizeof(covariance_));

  if (!enforce_covariance_health()) {
    std::memcpy(state_, previous_state, sizeof(state_));
    std::memcpy(covariance_, previous_covariance, sizeof(covariance_));
    healthy_ = false;
    return {UpdateResult::NumericalFailure, nis};
  }
  return {UpdateResult::Accepted, nis};
}

UpdateInfo Ekf6::update_wheel(const WheelSpeedMeasurement& measurement) {
  if (!initialized_) {
    return {UpdateResult::NotInitialized, 0.0F};
  }
  if (!healthy_) {
    return {UpdateResult::NumericalFailure, 0.0F};
  }
  if (!finite(measurement.speed_mps) ||
      std::fabs(measurement.speed_mps) > config_.maximum_abs_speed_mps) {
    ++stats_.invalid_measurements;
    return {UpdateResult::InvalidMeasurement, 0.0F};
  }
  float h[kN]{};
  h[static_cast<std::size_t>(StateIndex::Speed)] = 1.0F;
  const UpdateInfo result =
      scalar_update(measurement.speed_mps - state_[3], h,
                    config_.wheel_speed_variance, config_.wheel_nis_gate);
  if (result.result == UpdateResult::Accepted) {
    (void)stats_.wheel_nis.record(false, result.nis);
    ++stats_.wheel_accepted;
    seen_wheel_ = true;
    last_wheel_ms_ = measurement.timestamp_ms;
  } else if (result.result == UpdateResult::RejectedGate) {
    (void)stats_.wheel_nis.record(true, result.nis);
    ++stats_.wheel_rejected;
  } else if (result.result == UpdateResult::InvalidMeasurement) {
    ++stats_.invalid_measurements;
  } else if (result.result == UpdateResult::NumericalFailure) {
    ++stats_.numerical_failures;
  }
  return result;
}

UpdateInfo Ekf6::update_gnss(const GnssMeasurement& measurement) {
  if (!initialized_) {
    return {UpdateResult::NotInitialized, 0.0F};
  }
  if (!healthy_) {
    return {UpdateResult::NumericalFailure, 0.0F};
  }
  if (!finite(measurement.x_m) || !finite(measurement.y_m) ||
      std::fabs(measurement.x_m) > config_.maximum_abs_position_m ||
      std::fabs(measurement.y_m) > config_.maximum_abs_position_m) {
    ++stats_.invalid_measurements;
    return {UpdateResult::InvalidMeasurement, 0.0F};
  }
  float h[2U * kN]{};
  h[0U * kN + 0U] = 1.0F;
  h[1U * kN + 1U] = 1.0F;
  const float innovation[2] = {measurement.x_m - state_[0],
                               measurement.y_m - state_[1]};
  const float variance[4] = {config_.gnss_x_variance, 0.0F, 0.0F,
                             config_.gnss_y_variance};
  const UpdateInfo result =
      vector2_update(innovation, h, variance, config_.gnss_nis_gate, false);
  if (result.result == UpdateResult::Accepted) {
    (void)stats_.gnss_nis.record(false, result.nis);
    ++stats_.gnss_accepted;
    seen_gnss_ = true;
    last_gnss_ms_ = measurement.timestamp_ms;
  } else if (result.result == UpdateResult::RejectedGate) {
    (void)stats_.gnss_nis.record(true, result.nis);
    ++stats_.gnss_rejected;
  } else if (result.result == UpdateResult::InvalidMeasurement) {
    ++stats_.invalid_measurements;
  } else if (result.result == UpdateResult::NumericalFailure) {
    ++stats_.numerical_failures;
  }
  return result;
}

bool Ekf6::landmark_model_and_jacobian(
    const EkfState& state, float landmark_x_m, float landmark_y_m,
    float minimum_range_m, float predicted_measurement[2],
    float jacobian[2 * kEkfStateSize]) {
  if (!finite(state.x_m) || !finite(state.y_m) ||
      !finite(state.heading_rad) || !finite(landmark_x_m) ||
      !finite(landmark_y_m) || !positive_finite(minimum_range_m)) {
    return false;
  }

  const float dx = landmark_x_m - state.x_m;
  const float dy = landmark_y_m - state.y_m;
  const float range_squared = dx * dx + dy * dy;
  if (!finite(range_squared) ||
      range_squared < minimum_range_m * minimum_range_m) {
    return false;
  }
  const float range = std::sqrt(range_squared);
  predicted_measurement[0] = range;
  predicted_measurement[1] =
      normalize_angle(std::atan2(dy, dx) - state.heading_rad);

  std::memset(jacobian, 0, sizeof(float) * 2U * kN);
  jacobian[0U * kN + 0U] = -dx / range;
  jacobian[0U * kN + 1U] = -dy / range;
  jacobian[1U * kN + 0U] = dy / range_squared;
  jacobian[1U * kN + 1U] = -dx / range_squared;
  jacobian[1U * kN + 2U] = -1.0F;
  return finite(predicted_measurement[0]) && finite(predicted_measurement[1]);
}

UpdateInfo Ekf6::update_landmark(const LandmarkMeasurement& measurement) {
  if (!initialized_) {
    return {UpdateResult::NotInitialized, 0.0F};
  }
  if (!healthy_) {
    return {UpdateResult::NumericalFailure, 0.0F};
  }
  if (!finite(measurement.landmark_x_m) ||
      !finite(measurement.landmark_y_m) || !finite(measurement.range_m) ||
      !finite(measurement.bearing_rad) ||
      std::fabs(measurement.landmark_x_m) > config_.maximum_abs_position_m ||
      std::fabs(measurement.landmark_y_m) > config_.maximum_abs_position_m ||
      measurement.range_m < config_.minimum_landmark_range_m ||
      measurement.range_m > config_.maximum_landmark_range_m ||
      std::fabs(measurement.bearing_rad) > kPi) {
    ++stats_.invalid_measurements;
    return {UpdateResult::InvalidMeasurement, 0.0F};
  }

  float predicted[2]{};
  float h[2U * kN]{};
  if (!landmark_model_and_jacobian(
          state(), measurement.landmark_x_m, measurement.landmark_y_m,
          config_.minimum_landmark_range_m, predicted, h)) {
    ++stats_.invalid_measurements;
    return {UpdateResult::InvalidMeasurement, 0.0F};
  }
  const float innovation[2] = {
      measurement.range_m - predicted[0],
      normalize_angle(measurement.bearing_rad - predicted[1]),
  };
  const float variance[4] = {config_.landmark_range_variance, 0.0F, 0.0F,
                             config_.landmark_bearing_variance};
  const UpdateInfo result = vector2_update(
      innovation, h, variance, config_.landmark_nis_gate, true);
  if (result.result == UpdateResult::Accepted) {
    (void)stats_.landmark_nis.record(false, result.nis);
    ++stats_.landmark_accepted;
    seen_landmark_ = true;
    last_landmark_ms_ = measurement.timestamp_ms;
  } else if (result.result == UpdateResult::RejectedGate) {
    (void)stats_.landmark_nis.record(true, result.nis);
    ++stats_.landmark_rejected;
  } else if (result.result == UpdateResult::InvalidMeasurement) {
    ++stats_.invalid_measurements;
  } else if (result.result == UpdateResult::NumericalFailure) {
    ++stats_.numerical_failures;
  }
  return result;
}

bool Ekf6::state_is_finite() const {
  for (std::size_t i = 0U; i < kN; ++i) {
    if (!finite(state_[i])) {
      return false;
    }
  }
  return std::fabs(state_[0]) <= config_.maximum_abs_position_m &&
         std::fabs(state_[1]) <= config_.maximum_abs_position_m &&
         std::fabs(state_[2]) <= kPi &&
         std::fabs(state_[3]) <= config_.maximum_abs_speed_mps &&
         std::fabs(state_[4]) <= config_.maximum_abs_yaw_rate_rad_s &&
         std::fabs(state_[5]) <= config_.maximum_abs_acceleration_mps2;
}

bool Ekf6::enforce_covariance_health() {
  if (!state_is_finite()) {
    healthy_ = false;
    return false;
  }

  for (std::size_t row = 0U; row < kN; ++row) {
    for (std::size_t column = row; column < kN; ++column) {
      const float a = covariance_[index(row, column)];
      const float b = covariance_[index(column, row)];
      if (!finite(a) || !finite(b)) {
        healthy_ = false;
        return false;
      }
      const float symmetric = 0.5F * (a + b);
      covariance_[index(row, column)] = symmetric;
      covariance_[index(column, row)] = symmetric;
    }
    float& diagonal = covariance_[index(row, row)];
    if (diagonal < -config_.covariance_floor ||
        diagonal > config_.maximum_covariance) {
      healthy_ = false;
      return false;
    }
    if (diagonal < config_.covariance_floor) {
      diagonal = config_.covariance_floor;
    }
  }
  healthy_ = true;
  return true;
}

NavigationMode Ekf6::navigation_mode(uint32_t now_ms) const {
  if (!initialized_ || !healthy_) {
    return NavigationMode::Unavailable;
  }
  if (seen_gnss_ && elapsed_ms(now_ms, last_gnss_ms_) <= config_.gnss_timeout_ms) {
    return NavigationMode::GnssAided;
  }
  if (seen_landmark_ &&
      elapsed_ms(now_ms, last_landmark_ms_) <= config_.landmark_timeout_ms) {
    return NavigationMode::LandmarkAided;
  }
  if (seen_imu_ && seen_wheel_ &&
      elapsed_ms(now_ms, last_imu_ms_) <= config_.imu_timeout_ms &&
      elapsed_ms(now_ms, last_wheel_ms_) <= config_.wheel_timeout_ms) {
    return NavigationMode::DeadReckoning;
  }
  if ((seen_imu_ &&
       elapsed_ms(now_ms, last_imu_ms_) <= config_.imu_timeout_ms) ||
      (seen_wheel_ &&
       elapsed_ms(now_ms, last_wheel_ms_) <= config_.wheel_timeout_ms)) {
    return NavigationMode::Degraded;
  }
  return NavigationMode::Unavailable;
}

}  // namespace navbench
