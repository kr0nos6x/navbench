#pragma once

#include <cstddef>
#include <cstdint>

#include "navbench/ekf.hpp"

namespace navbench {

constexpr std::size_t kMaximumWaypoints = 32U;

struct Waypoint {
  constexpr Waypoint(float x = 0.0F, float y = 0.0F,
                     float target_speed = 0.0F,
                     float acceptance_radius = 0.0F)
      : x_m(x),
        y_m(y),
        target_speed_mps(target_speed),
        acceptance_radius_m(acceptance_radius) {}

  float x_m;
  float y_m;
  float target_speed_mps;
  // Zero selects ControlConfig::waypoint_acceptance_radius_m.
  float acceptance_radius_m;
};

struct ControlConfig {
  float wheelbase_m;
  float lookahead_m;
  float waypoint_acceptance_radius_m;
  float final_position_tolerance_m;
  float final_speed_tolerance_mps;
  float final_slowdown_distance_m;
  float maximum_speed_mps;
  float maximum_steering_rad;
  float maximum_steering_rate_rad_s;
  float maximum_acceleration_mps2;
  float maximum_deceleration_mps2;
  float speed_kp;
  float speed_ki;
  float speed_integral_limit;

  static ControlConfig defaults();
};

struct RouteProgress {
  bool valid{false};
  bool complete{false};
  uint8_t active_waypoint{0U};
  float target_x_m{0.0F};
  float target_y_m{0.0F};
  float target_speed_mps{0.0F};
  float distance_to_final_m{0.0F};
};

struct ControlCommand {
  float steering_rad{0.0F};
  float acceleration_mps2{0.0F};
  float target_speed_mps{0.0F};
  bool valid{false};
  bool safe_stop{true};
  bool route_complete{false};
};

class RouteManager {
 public:
  explicit RouteManager(const ControlConfig& config = ControlConfig::defaults());

  bool set_config(const ControlConfig& config);
  bool set_route(const Waypoint* waypoints, std::size_t count);
  void clear();
  void reset_progress();
  RouteProgress update(const EkfState& estimate);

  std::size_t waypoint_count() const { return count_; }
  uint8_t active_waypoint() const { return active_waypoint_; }
  bool route_valid() const { return route_valid_; }

 private:
  bool validate_config(const ControlConfig& config) const;
  bool estimate_is_finite(const EkfState& estimate) const;
  void compute_lookahead_target(const EkfState& estimate, float& target_x_m,
                                float& target_y_m) const;

  ControlConfig config_{};
  Waypoint waypoints_[kMaximumWaypoints]{};
  std::size_t count_{0U};
  uint8_t active_waypoint_{0U};
  bool route_valid_{false};
  bool final_stop_latched_{false};
};

class SpeedPiController {
 public:
  explicit SpeedPiController(
      const ControlConfig& config = ControlConfig::defaults());

  bool set_config(const ControlConfig& config);
  void reset();
  float step(float target_speed_mps, float measured_speed_mps, float dt_s);
  float integral() const { return integral_; }

 private:
  ControlConfig config_{};
  float integral_{0.0F};
};

class ControlSystem {
 public:
  explicit ControlSystem(
      const ControlConfig& config = ControlConfig::defaults());

  bool set_config(const ControlConfig& config);
  bool set_route(const Waypoint* waypoints, std::size_t count);
  void reset();
  ControlCommand step(const EkfState& estimate, float dt_s,
                      bool force_safe_stop);
  ControlCommand neutral_command() const;
  ControlCommand safe_stop_command(float measured_speed_mps) const;
  const RouteManager& route() const { return route_; }
  const ControlConfig& config() const { return config_; }
  float speed_integral() const { return speed_controller_.integral(); }

 private:
  bool estimate_is_finite(const EkfState& estimate) const;

  ControlConfig config_{};
  RouteManager route_{};
  SpeedPiController speed_controller_{};
  float previous_steering_rad_{0.0F};
};

}  // namespace navbench
