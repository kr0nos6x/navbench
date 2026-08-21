#include <cmath>
#include <cstdio>
#include <limits>

#include "navbench/ekf.hpp"
#include "navbench/math.hpp"

namespace {

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

bool close(float lhs, float rhs, float tolerance) {
  return std::fabs(lhs - rhs) <= tolerance;
}

float component(const navbench::EkfState& state, std::size_t index) {
  switch (index) {
    case 0U:
      return state.x_m;
    case 1U:
      return state.y_m;
    case 2U:
      return state.heading_rad;
    case 3U:
      return state.speed_mps;
    case 4U:
      return state.yaw_rate_rad_s;
    default:
      return state.accel_bias_mps2;
  }
}

void set_component(navbench::EkfState& state, std::size_t index, float value) {
  switch (index) {
    case 0U:
      state.x_m = value;
      return;
    case 1U:
      state.y_m = value;
      return;
    case 2U:
      state.heading_rad = value;
      return;
    case 3U:
      state.speed_mps = value;
      return;
    case 4U:
      state.yaw_rate_rad_s = value;
      return;
    default:
      state.accel_bias_mps2 = value;
      return;
  }
}

bool test_angle_normalization() {
  CHECK(close(navbench::normalize_angle(0.0F), 0.0F, 1.0e-6F));
  CHECK(close(navbench::normalize_angle(3.0F * navbench::kPi),
              -navbench::kPi, 1.0e-5F));
  CHECK(close(navbench::normalize_angle(-1.5F * navbench::kPi),
              0.5F * navbench::kPi, 1.0e-5F));
  return true;
}

bool test_prediction_jacobian() {
  navbench::EkfState state{};
  state.x_m = 2.0F;
  state.y_m = -1.0F;
  state.heading_rad = 0.4F;
  state.speed_mps = 3.0F;
  state.yaw_rate_rad_s = -0.2F;
  state.accel_bias_mps2 = 0.08F;
  navbench::ImuMeasurement imu{};
  imu.longitudinal_accel_mps2 = 0.7F;
  imu.yaw_rate_rad_s = -0.18F;

  navbench::EkfState predicted{};
  float analytic[navbench::kEkfStateSize * navbench::kEkfStateSize]{};
  CHECK(navbench::Ekf6::prediction_model(state, imu, 0.05F, predicted,
                                         analytic));

  constexpr float epsilon = 1.0e-3F;
  for (std::size_t column = 0U; column < navbench::kEkfStateSize; ++column) {
    navbench::EkfState plus = state;
    navbench::EkfState minus = state;
    set_component(plus, column, component(plus, column) + epsilon);
    set_component(minus, column, component(minus, column) - epsilon);
    navbench::EkfState predicted_plus{};
    navbench::EkfState predicted_minus{};
    float unused[navbench::kEkfStateSize * navbench::kEkfStateSize]{};
    CHECK(navbench::Ekf6::prediction_model(plus, imu, 0.05F, predicted_plus,
                                           unused));
    CHECK(navbench::Ekf6::prediction_model(minus, imu, 0.05F,
                                           predicted_minus, unused));
    for (std::size_t row = 0U; row < navbench::kEkfStateSize; ++row) {
      float difference = component(predicted_plus, row) -
                         component(predicted_minus, row);
      if (row == 2U) {
        difference = navbench::normalize_angle(difference);
      }
      const float numeric = difference / (2.0F * epsilon);
      CHECK(close(numeric,
                  analytic[row * navbench::kEkfStateSize + column],
                  8.0e-3F));
    }
  }
  return true;
}

bool test_landmark_jacobian() {
  navbench::EkfState state{};
  state.x_m = 1.2F;
  state.y_m = -0.7F;
  state.heading_rad = 0.3F;
  state.speed_mps = 2.0F;

  float predicted[2]{};
  float analytic[2U * navbench::kEkfStateSize]{};
  CHECK(navbench::Ekf6::landmark_model_and_jacobian(
      state, 4.0F, 3.0F, 0.05F, predicted, analytic));

  constexpr float epsilon = 1.0e-3F;
  for (std::size_t column = 0U; column < navbench::kEkfStateSize; ++column) {
    navbench::EkfState plus = state;
    navbench::EkfState minus = state;
    set_component(plus, column, component(plus, column) + epsilon);
    set_component(minus, column, component(minus, column) - epsilon);
    float plus_measurement[2]{};
    float minus_measurement[2]{};
    float unused[2U * navbench::kEkfStateSize]{};
    CHECK(navbench::Ekf6::landmark_model_and_jacobian(
        plus, 4.0F, 3.0F, 0.05F, plus_measurement, unused));
    CHECK(navbench::Ekf6::landmark_model_and_jacobian(
        minus, 4.0F, 3.0F, 0.05F, minus_measurement, unused));
    const float range_numeric =
        (plus_measurement[0] - minus_measurement[0]) / (2.0F * epsilon);
    const float bearing_numeric =
        navbench::normalize_angle(plus_measurement[1] - minus_measurement[1]) /
        (2.0F * epsilon);
    CHECK(close(range_numeric, analytic[column], 2.0e-3F));
    CHECK(close(bearing_numeric,
                analytic[navbench::kEkfStateSize + column], 2.0e-3F));
  }

  float invalid_prediction[2]{};
  float invalid_jacobian[2U * navbench::kEkfStateSize]{};
  state.x_m = 4.0F;
  state.y_m = 3.0F;
  CHECK(!navbench::Ekf6::landmark_model_and_jacobian(
      state, 4.0F, 3.0F, 0.05F, invalid_prediction, invalid_jacobian));
  return true;
}

bool test_updates_gating_modes_and_health() {
  navbench::EkfConfig config = navbench::EkfConfig::defaults();
  config.imu_timeout_ms = 500U;
  config.wheel_timeout_ms = 300U;
  config.gnss_timeout_ms = 50U;
  config.landmark_timeout_ms = 100U;
  navbench::Ekf6 ekf(config);

  navbench::EkfState initial{};
  initial.speed_mps = 1.0F;
  CHECK(ekf.initialize(initial, 100U));
  CHECK(ekf.healthy());

  navbench::ImuMeasurement imu{};
  imu.longitudinal_accel_mps2 = 0.1F;
  imu.yaw_rate_rad_s = 0.02F;
  imu.timestamp_ms = 100U;
  const navbench::UpdateInfo initial_imu = ekf.predict(imu, 0.02F);
  CHECK(initial_imu.result == navbench::UpdateResult::Accepted);

  navbench::WheelSpeedMeasurement wheel{};
  wheel.speed_mps = 1.0F;
  wheel.timestamp_ms = 100U;
  const navbench::UpdateInfo initial_wheel = ekf.update_wheel(wheel);
  CHECK(initial_wheel.result == navbench::UpdateResult::Accepted);

  navbench::GnssMeasurement gnss{};
  gnss.x_m = 0.02F;
  gnss.y_m = 0.01F;
  gnss.timestamp_ms = 100U;
  const navbench::UpdateInfo initial_gnss = ekf.update_gnss(gnss);
  CHECK(initial_gnss.result == navbench::UpdateResult::Accepted);

  float predicted_landmark[2]{};
  float unused_jacobian[2U * navbench::kEkfStateSize]{};
  CHECK(navbench::Ekf6::landmark_model_and_jacobian(
      ekf.state(), 5.0F, 2.0F, config.minimum_landmark_range_m,
      predicted_landmark, unused_jacobian));
  navbench::LandmarkMeasurement landmark{};
  landmark.landmark_x_m = 5.0F;
  landmark.landmark_y_m = 2.0F;
  landmark.range_m = predicted_landmark[0];
  landmark.bearing_rad = predicted_landmark[1];
  landmark.timestamp_ms = 100U;
  const navbench::UpdateInfo initial_landmark =
      ekf.update_landmark(landmark);
  CHECK(initial_landmark.result == navbench::UpdateResult::Accepted);

  const navbench::EkfStats& initial_stats = ekf.stats();
  CHECK(initial_stats.imu_yaw_nis.evaluated_count == 1U);
  CHECK(initial_stats.imu_yaw_nis.gate_rejected_count == 0U);
  CHECK(close(initial_stats.imu_yaw_nis.nis_sum, initial_imu.nis, 1.0e-7F));
  CHECK(close(initial_stats.imu_yaw_nis.nis_max, initial_imu.nis, 1.0e-7F));
  CHECK(initial_stats.wheel_nis.evaluated_count == 1U);
  CHECK(initial_stats.wheel_nis.gate_rejected_count == 0U);
  CHECK(close(initial_stats.wheel_nis.nis_sum, initial_wheel.nis, 1.0e-7F));
  CHECK(initial_stats.gnss_nis.evaluated_count == 1U);
  CHECK(initial_stats.gnss_nis.gate_rejected_count == 0U);
  CHECK(close(initial_stats.gnss_nis.nis_sum, initial_gnss.nis, 1.0e-7F));
  CHECK(initial_stats.landmark_nis.evaluated_count == 1U);
  CHECK(initial_stats.landmark_nis.gate_rejected_count == 0U);
  CHECK(close(initial_stats.landmark_nis.nis_sum, initial_landmark.nis,
              1.0e-7F));

  CHECK(ekf.navigation_mode(120U) == navbench::NavigationMode::GnssAided);
  CHECK(ekf.navigation_mode(160U) ==
        navbench::NavigationMode::LandmarkAided);
  CHECK(ekf.navigation_mode(220U) ==
        navbench::NavigationMode::DeadReckoning);
  CHECK(ekf.navigation_mode(450U) == navbench::NavigationMode::Degraded);
  CHECK(ekf.navigation_mode(700U) == navbench::NavigationMode::Unavailable);

  const navbench::EkfState before_outlier = ekf.state();
  gnss.x_m = 1000.0F;
  gnss.y_m = -1000.0F;
  gnss.timestamp_ms = 130U;
  const navbench::UpdateInfo rejected = ekf.update_gnss(gnss);
  CHECK(rejected.result == navbench::UpdateResult::RejectedGate);
  CHECK(rejected.nis > config.gnss_nis_gate);
  CHECK(ekf.stats().gnss_nis.evaluated_count == 2U);
  CHECK(ekf.stats().gnss_nis.gate_rejected_count == 1U);
  CHECK(close(ekf.stats().gnss_nis.nis_sum,
              initial_gnss.nis + rejected.nis, 1.0e-3F));
  CHECK(close(ekf.stats().gnss_nis.nis_max, rejected.nis, 1.0e-3F));
  CHECK(close(ekf.state().x_m, before_outlier.x_m, 1.0e-7F));
  CHECK(close(ekf.state().y_m, before_outlier.y_m, 1.0e-7F));

  wheel.speed_mps = std::numeric_limits<float>::quiet_NaN();
  CHECK(ekf.update_wheel(wheel).result ==
        navbench::UpdateResult::InvalidMeasurement);
  CHECK(ekf.stats().wheel_nis.evaluated_count == 1U);
  CHECK(ekf.healthy());

  for (uint32_t i = 1U; i <= 300U; ++i) {
    const uint32_t timestamp = 200U + i * 20U;
    imu.timestamp_ms = timestamp;
    imu.longitudinal_accel_mps2 = 0.0F;
    imu.yaw_rate_rad_s = 0.02F;
    const navbench::UpdateInfo prediction = ekf.predict(imu, 0.02F);
    CHECK(prediction.result == navbench::UpdateResult::Accepted);
    wheel.speed_mps = ekf.state().speed_mps;
    wheel.timestamp_ms = timestamp;
    CHECK(ekf.update_wheel(wheel).result == navbench::UpdateResult::Accepted);
    if (i % 10U == 0U) {
      gnss.x_m = ekf.state().x_m;
      gnss.y_m = ekf.state().y_m;
      gnss.timestamp_ms = timestamp;
      CHECK(ekf.update_gnss(gnss).result == navbench::UpdateResult::Accepted);
    }
  }

  float covariance[navbench::kEkfStateSize * navbench::kEkfStateSize]{};
  ekf.covariance(covariance);
  for (std::size_t row = 0U; row < navbench::kEkfStateSize; ++row) {
    CHECK(std::isfinite(covariance[row * navbench::kEkfStateSize + row]));
    CHECK(covariance[row * navbench::kEkfStateSize + row] > 0.0F);
    for (std::size_t column = 0U; column < navbench::kEkfStateSize; ++column) {
      CHECK(close(covariance[row * navbench::kEkfStateSize + column],
                  covariance[column * navbench::kEkfStateSize + row],
                  1.0e-5F));
    }
  }
  CHECK(ekf.healthy());
  CHECK(ekf.stats().predictions == 301U);
  CHECK(ekf.stats().gnss_rejected == 1U);
  CHECK(ekf.stats().invalid_measurements == 1U);
  CHECK(ekf.stats().imu_yaw_nis.evaluated_count == 301U);
  CHECK(ekf.stats().wheel_nis.evaluated_count == 301U);
  CHECK(ekf.stats().gnss_nis.evaluated_count == 32U);
  CHECK(ekf.stats().gnss_nis.gate_rejected_count == 1U);
  CHECK(ekf.stats().landmark_nis.evaluated_count == 1U);
  CHECK(std::isfinite(ekf.stats().imu_yaw_nis.nis_sum));
  CHECK(std::isfinite(ekf.stats().wheel_nis.nis_sum));
  CHECK(std::isfinite(ekf.stats().gnss_nis.nis_sum));
  CHECK(std::isfinite(ekf.stats().landmark_nis.nis_sum));
  return true;
}

bool test_nis_accumulator_reset_and_overflow() {
  navbench::NisAccumulator accumulator{};
  CHECK(accumulator.record(false, 1.5F));
  CHECK(accumulator.record(true, 2.5F));
  CHECK(accumulator.evaluated_count == 2U);
  CHECK(accumulator.gate_rejected_count == 1U);
  CHECK(accumulator.nis_sum == 4.0F);
  CHECK(accumulator.nis_max == 2.5F);

  CHECK(!accumulator.record(false,
                            std::numeric_limits<float>::quiet_NaN()));
  CHECK(!accumulator.record(false, -1.0F));
  CHECK(accumulator.evaluated_count == 2U);
  CHECK(accumulator.nis_sum == 4.0F);

  accumulator.evaluated_count = 0xffffffffUL;
  accumulator.gate_rejected_count = 10U;
  accumulator.nis_sum = 100.0F;
  accumulator.nis_max = 5.0F;
  CHECK(accumulator.record(false, 3.0F));
  CHECK(accumulator.evaluated_count == 1U);
  CHECK(accumulator.gate_rejected_count == 0U);
  CHECK(accumulator.nis_sum == 3.0F);
  CHECK(accumulator.nis_max == 3.0F);

  accumulator.evaluated_count = 2U;
  accumulator.gate_rejected_count = 1U;
  accumulator.nis_sum = std::numeric_limits<float>::max();
  accumulator.nis_max = std::numeric_limits<float>::max();
  CHECK(accumulator.record(true, std::numeric_limits<float>::max()));
  CHECK(accumulator.evaluated_count == 1U);
  CHECK(accumulator.gate_rejected_count == 1U);
  CHECK(std::isfinite(accumulator.nis_sum));
  CHECK(accumulator.nis_sum == std::numeric_limits<float>::max());
  CHECK(accumulator.nis_max == std::numeric_limits<float>::max());

  accumulator.reset();
  CHECK(accumulator.evaluated_count == 0U);
  CHECK(accumulator.gate_rejected_count == 0U);
  CHECK(accumulator.nis_sum == 0.0F);
  CHECK(accumulator.nis_max == 0.0F);
  return true;
}

bool test_configuration_rejection_preserves_defaults() {
  navbench::Ekf6 ekf;
  navbench::EkfConfig invalid = navbench::EkfConfig::defaults();
  invalid.process_noise_per_second[2] = -1.0F;
  CHECK(!ekf.set_config(invalid));
  CHECK(!ekf.initialized());
  return true;
}

bool test_yaw_rate_process_noise_tracks_steering_transient() {
  navbench::Ekf6 ekf;
  navbench::EkfState initial{};
  initial.speed_mps = 1.5F;
  CHECK(ekf.initialize(initial, 0U));
  navbench::ImuMeasurement imu{};
  imu.longitudinal_accel_mps2 = 0.0F;
  for (uint32_t step = 1U; step <= 100U; ++step) {
    imu.timestamp_ms = step * 20U;
    imu.step_id = step;
    imu.yaw_rate_rad_s = 1.5F * static_cast<float>(step) / 100.0F;
    CHECK(ekf.predict(imu, 0.02F).result == navbench::UpdateResult::Accepted);
  }
  CHECK(std::fabs(ekf.state().yaw_rate_rad_s - 1.5F) < 0.10F);
  CHECK(ekf.stats().imu_yaw_rejected == 0U);
  return true;
}

}  // namespace

int main() {
  if (!test_angle_normalization() || !test_prediction_jacobian() ||
      !test_landmark_jacobian() ||
      !test_updates_gating_modes_and_health() ||
      !test_nis_accumulator_reset_and_overflow() ||
      !test_configuration_rejection_preserves_defaults() ||
      !test_yaw_rate_process_noise_tracks_steering_transient()) {
    return 1;
  }
  std::printf("test_ekf: PASS (%d checks)\n", checks);
  return 0;
}
