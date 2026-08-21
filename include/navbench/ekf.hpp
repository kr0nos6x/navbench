#pragma once

#include <cstddef>
#include <cstdint>

namespace navbench {

constexpr std::size_t kEkfStateSize = 6U;

enum class StateIndex : uint8_t {
  X = 0U,
  Y = 1U,
  Heading = 2U,
  Speed = 3U,
  YawRate = 4U,
  AccelBias = 5U,
};

enum class NavigationMode : uint8_t {
  Unavailable = 0U,
  DeadReckoning = 1U,
  LandmarkAided = 2U,
  GnssAided = 3U,
  Degraded = 4U,
};

enum class UpdateResult : uint8_t {
  Accepted = 0U,
  RejectedGate = 1U,
  InvalidMeasurement = 2U,
  NotInitialized = 3U,
  NumericalFailure = 4U,
};

struct EkfState {
  float x_m{0.0F};
  float y_m{0.0F};
  float heading_rad{0.0F};
  float speed_mps{0.0F};
  float yaw_rate_rad_s{0.0F};
  float accel_bias_mps2{0.0F};
};

struct ImuMeasurement {
  float longitudinal_accel_mps2{0.0F};
  float yaw_rate_rad_s{0.0F};
  uint32_t timestamp_ms{0U};
  uint32_t step_id{0U};
};

struct WheelSpeedMeasurement {
  float speed_mps{0.0F};
  uint32_t timestamp_ms{0U};
  uint32_t step_id{0U};
};

struct GnssMeasurement {
  float x_m{0.0F};
  float y_m{0.0F};
  uint32_t timestamp_ms{0U};
  uint32_t step_id{0U};
};

struct LandmarkMeasurement {
  uint16_t landmark_id{0U};
  float landmark_x_m{0.0F};
  float landmark_y_m{0.0F};
  float range_m{0.0F};
  float bearing_rad{0.0F};
  uint32_t timestamp_ms{0U};
  uint32_t step_id{0U};
};

struct UpdateInfo {
  constexpr UpdateInfo(UpdateResult result_value = UpdateResult::NotInitialized,
                       float nis_value = 0.0F)
      : result(result_value), nis(nis_value) {}

  UpdateResult result;
  float nis;
};

struct EkfConfig {
  float initial_variance[kEkfStateSize];
  float process_noise_per_second[kEkfStateSize];
  float imu_yaw_rate_variance;
  float wheel_speed_variance;
  float gnss_x_variance;
  float gnss_y_variance;
  float landmark_range_variance;
  float landmark_bearing_variance;
  float imu_yaw_nis_gate;
  float wheel_nis_gate;
  float gnss_nis_gate;
  float landmark_nis_gate;
  float minimum_landmark_range_m;
  float maximum_landmark_range_m;
  float maximum_abs_position_m;
  float maximum_abs_speed_mps;
  float maximum_abs_acceleration_mps2;
  float maximum_abs_yaw_rate_rad_s;
  float maximum_prediction_dt_s;
  float covariance_floor;
  float maximum_covariance;
  uint32_t imu_timeout_ms;
  uint32_t wheel_timeout_ms;
  uint32_t gnss_timeout_ms;
  uint32_t landmark_timeout_ms;

  static EkfConfig defaults();
};

struct NisAccumulator {
  uint32_t evaluated_count{0U};
  uint32_t gate_rejected_count{0U};
  float nis_sum{0.0F};
  float nis_max{0.0F};

  void reset();
  // Records one finite, non-negative accepted or gate-rejected NIS. On count
  // or finite-sum overflow the accumulator starts a new epoch with this sample.
  bool record(bool gate_rejected, float nis);
};

struct EkfStats {
  uint32_t predictions{0U};
  uint32_t imu_yaw_accepted{0U};
  uint32_t imu_yaw_rejected{0U};
  uint32_t wheel_accepted{0U};
  uint32_t wheel_rejected{0U};
  uint32_t gnss_accepted{0U};
  uint32_t gnss_rejected{0U};
  uint32_t landmark_accepted{0U};
  uint32_t landmark_rejected{0U};
  uint32_t invalid_measurements{0U};
  uint32_t numerical_failures{0U};
  NisAccumulator imu_yaw_nis{};
  NisAccumulator wheel_nis{};
  NisAccumulator gnss_nis{};
  NisAccumulator landmark_nis{};
};

class Ekf6 {
 public:
  Ekf6();
  explicit Ekf6(const EkfConfig& config);

  bool set_config(const EkfConfig& config);
  void reset();
  bool initialize(const EkfState& initial_state, uint32_t timestamp_ms);
  bool initialize(const EkfState& initial_state,
                  const float covariance_diagonal[kEkfStateSize],
                  uint32_t timestamp_ms);

  UpdateInfo predict(const ImuMeasurement& measurement, float dt_s);
  UpdateInfo update_wheel(const WheelSpeedMeasurement& measurement);
  UpdateInfo update_gnss(const GnssMeasurement& measurement);
  UpdateInfo update_landmark(const LandmarkMeasurement& measurement);

  EkfState state() const;
  void covariance(float output[kEkfStateSize * kEkfStateSize]) const;
  const EkfStats& stats() const { return stats_; }
  bool initialized() const { return initialized_; }
  bool healthy() const { return healthy_; }
  NavigationMode navigation_mode(uint32_t now_ms) const;

  static bool prediction_model(const EkfState& state,
                               const ImuMeasurement& measurement, float dt_s,
                               EkfState& predicted,
                               float jacobian[kEkfStateSize * kEkfStateSize]);
  static bool landmark_model_and_jacobian(
      const EkfState& state, float landmark_x_m, float landmark_y_m,
      float minimum_range_m, float predicted_measurement[2],
      float jacobian[2 * kEkfStateSize]);

 private:
  bool validate_config(const EkfConfig& config) const;
  bool enforce_covariance_health();
  bool state_is_finite() const;
  void set_state(const EkfState& state);
  UpdateInfo scalar_update(float innovation, const float h[kEkfStateSize],
                           float variance, float gate);
  UpdateInfo vector2_update(const float innovation[2],
                            const float h[2 * kEkfStateSize],
                            const float variance[4], float gate,
                            bool normalize_second_innovation);

  EkfConfig config_{};
  float state_[kEkfStateSize]{};
  float covariance_[kEkfStateSize * kEkfStateSize]{};
  EkfStats stats_{};
  bool initialized_{false};
  bool healthy_{false};
  bool seen_imu_{false};
  bool seen_wheel_{false};
  bool seen_gnss_{false};
  bool seen_landmark_{false};
  uint32_t last_imu_ms_{0U};
  uint32_t last_wheel_ms_{0U};
  uint32_t last_gnss_ms_{0U};
  uint32_t last_landmark_ms_{0U};
};

}  // namespace navbench
