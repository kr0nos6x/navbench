#include "navbench/control.hpp"

#include <cmath>

#include "navbench/math.hpp"

namespace navbench {
namespace {

bool positive_finite(float value) {
  return finite(value) && value > 0.0F;
}

bool valid_control_config(const ControlConfig& config) {
  return positive_finite(config.wheelbase_m) &&
         positive_finite(config.lookahead_m) &&
         positive_finite(config.waypoint_acceptance_radius_m) &&
         positive_finite(config.final_position_tolerance_m) &&
         positive_finite(config.final_speed_tolerance_mps) &&
         positive_finite(config.final_slowdown_distance_m) &&
         positive_finite(config.maximum_speed_mps) &&
         positive_finite(config.maximum_steering_rad) &&
         positive_finite(config.maximum_steering_rate_rad_s) &&
         positive_finite(config.maximum_acceleration_mps2) &&
         positive_finite(config.maximum_deceleration_mps2) &&
         finite(config.speed_kp) && config.speed_kp >= 0.0F &&
         finite(config.speed_ki) && config.speed_ki >= 0.0F &&
         (config.speed_kp > 0.0F || config.speed_ki > 0.0F) &&
         positive_finite(config.speed_integral_limit);
}

float distance(float x0, float y0, float x1, float y1) {
  return std::hypot(x1 - x0, y1 - y0);
}

float acceptance_radius(const Waypoint& waypoint,
                        const ControlConfig& config) {
  return waypoint.acceptance_radius_m > 0.0F
             ? waypoint.acceptance_radius_m
             : config.waypoint_acceptance_radius_m;
}

}  // namespace

ControlConfig ControlConfig::defaults() {
  ControlConfig config{};
  config.wheelbase_m = 0.32F;
  config.lookahead_m = 1.25F;
  config.waypoint_acceptance_radius_m = 0.45F;
  config.final_position_tolerance_m = 0.30F;
  config.final_speed_tolerance_mps = 0.08F;
  config.final_slowdown_distance_m = 2.5F;
  config.maximum_speed_mps = 4.0F;
  config.maximum_steering_rad = 0.55F;
  config.maximum_steering_rate_rad_s = 1.5F;
  config.maximum_acceleration_mps2 = 2.0F;
  config.maximum_deceleration_mps2 = 3.0F;
  config.speed_kp = 1.2F;
  config.speed_ki = 0.35F;
  config.speed_integral_limit = 4.0F;
  return config;
}

RouteManager::RouteManager(const ControlConfig& config)
    : config_(ControlConfig::defaults()) {
  if (validate_config(config)) {
    config_ = config;
  }
}

bool RouteManager::validate_config(const ControlConfig& config) const {
  return valid_control_config(config);
}

bool RouteManager::set_config(const ControlConfig& config) {
  if (!validate_config(config)) {
    return false;
  }
  config_ = config;
  return true;
}

bool RouteManager::set_route(const Waypoint* waypoints, std::size_t count) {
  if (waypoints == nullptr || count == 0U || count > kMaximumWaypoints) {
    return false;
  }
  for (std::size_t i = 0U; i < count; ++i) {
    if (!finite(waypoints[i].x_m) || !finite(waypoints[i].y_m) ||
        !finite(waypoints[i].target_speed_mps) ||
        !finite(waypoints[i].acceptance_radius_m) ||
        waypoints[i].target_speed_mps < 0.0F ||
        waypoints[i].target_speed_mps > config_.maximum_speed_mps ||
        waypoints[i].acceptance_radius_m < 0.0F) {
      return false;
    }
  }
  for (std::size_t i = 0U; i < count; ++i) {
    waypoints_[i] = waypoints[i];
  }
  count_ = count;
  active_waypoint_ = 0U;
  route_valid_ = true;
  final_stop_latched_ = false;
  return true;
}

void RouteManager::clear() {
  count_ = 0U;
  active_waypoint_ = 0U;
  route_valid_ = false;
  final_stop_latched_ = false;
}

void RouteManager::reset_progress() {
  active_waypoint_ = 0U;
  final_stop_latched_ = false;
}

bool RouteManager::estimate_is_finite(const EkfState& estimate) const {
  return finite(estimate.x_m) && finite(estimate.y_m) &&
         finite(estimate.heading_rad) && finite(estimate.speed_mps);
}

void RouteManager::compute_lookahead_target(const EkfState& estimate,
                                            float& target_x_m,
                                            float& target_y_m) const {
  // Before the first waypoint is accepted there is no preceding route
  // segment to project onto.  Target it directly instead of accidentally
  // starting on the segment from waypoint 0 to waypoint 1.
  if (count_ == 1U || active_waypoint_ == 0U) {
    target_x_m = waypoints_[0].x_m;
    target_y_m = waypoints_[0].y_m;
    return;
  }

  std::size_t start_segment =
      static_cast<std::size_t>(active_waypoint_ - 1U);
  if (start_segment >= count_ - 1U) {
    start_segment = count_ - 2U;
  }

  const std::size_t best_segment = start_segment;
  const float x0 = waypoints_[best_segment].x_m;
  const float y0 = waypoints_[best_segment].y_m;
  const float segment_dx = waypoints_[best_segment + 1U].x_m - x0;
  const float segment_dy = waypoints_[best_segment + 1U].y_m - y0;
  const float segment_length_squared =
      segment_dx * segment_dx + segment_dy * segment_dy;
  float best_fraction = 0.0F;
  if (segment_length_squared > 1.0e-10F) {
    best_fraction = clamp(
        ((estimate.x_m - x0) * segment_dx +
         (estimate.y_m - y0) * segment_dy) /
            segment_length_squared,
        0.0F, 1.0F);
  }

  float remaining = config_.lookahead_m;
  for (std::size_t segment = best_segment; segment + 1U < count_; ++segment) {
    const float x0 = waypoints_[segment].x_m;
    const float y0 = waypoints_[segment].y_m;
    const float dx = waypoints_[segment + 1U].x_m - x0;
    const float dy = waypoints_[segment + 1U].y_m - y0;
    const float length = std::hypot(dx, dy);
    const float start_fraction = segment == best_segment ? best_fraction : 0.0F;
    const float available = length * (1.0F - start_fraction);
    if (length > 1.0e-6F && remaining <= available) {
      const float fraction = start_fraction + remaining / length;
      target_x_m = x0 + fraction * dx;
      target_y_m = y0 + fraction * dy;
      return;
    }
    remaining -= available;
  }

  target_x_m = waypoints_[count_ - 1U].x_m;
  target_y_m = waypoints_[count_ - 1U].y_m;
}

RouteProgress RouteManager::update(const EkfState& estimate) {
  RouteProgress progress{};
  if (!route_valid_ || !estimate_is_finite(estimate)) {
    return progress;
  }

  while (static_cast<std::size_t>(active_waypoint_) + 1U < count_ &&
         distance(estimate.x_m, estimate.y_m,
                  waypoints_[active_waypoint_].x_m,
                  waypoints_[active_waypoint_].y_m) <=
             acceptance_radius(waypoints_[active_waypoint_], config_)) {
    ++active_waypoint_;
  }

  const Waypoint& final_waypoint = waypoints_[count_ - 1U];
  progress.valid = true;
  progress.active_waypoint = active_waypoint_;
  progress.distance_to_final_m = distance(
      estimate.x_m, estimate.y_m, final_waypoint.x_m, final_waypoint.y_m);
  const float final_acceptance_radius =
      final_waypoint.acceptance_radius_m > 0.0F
          ? final_waypoint.acceptance_radius_m
          : config_.final_position_tolerance_m;
  if (active_waypoint_ == count_ - 1U &&
      progress.distance_to_final_m <= final_acceptance_radius) {
    final_stop_latched_ = true;
  }
  progress.complete =
      progress.distance_to_final_m <= final_acceptance_radius &&
      std::fabs(estimate.speed_mps) <= config_.final_speed_tolerance_mps;

  compute_lookahead_target(estimate, progress.target_x_m, progress.target_y_m);
  progress.target_speed_mps = waypoints_[active_waypoint_].target_speed_mps;

  if (active_waypoint_ == count_ - 1U) {
    float approach_speed = progress.target_speed_mps;
    if (approach_speed <= config_.final_speed_tolerance_mps && count_ > 1U) {
      approach_speed = waypoints_[count_ - 2U].target_speed_mps;
    }
    const float braking_speed =
        std::sqrt(2.0F * config_.maximum_deceleration_mps2 *
                  progress.distance_to_final_m);
    const float linear_speed =
        config_.maximum_speed_mps *
        clamp((progress.distance_to_final_m -
               final_acceptance_radius) /
                  config_.final_slowdown_distance_m,
              0.0F, 1.0F);
    progress.target_speed_mps = clamp(
        approach_speed, 0.0F,
        braking_speed < linear_speed ? braking_speed : linear_speed);
    if (final_stop_latched_) {
      progress.target_speed_mps = 0.0F;
      progress.target_x_m = final_waypoint.x_m;
      progress.target_y_m = final_waypoint.y_m;
    }
  }
  if (progress.complete) {
    progress.target_speed_mps = 0.0F;
    progress.target_x_m = final_waypoint.x_m;
    progress.target_y_m = final_waypoint.y_m;
  }
  return progress;
}

SpeedPiController::SpeedPiController(const ControlConfig& config)
    : config_(ControlConfig::defaults()) {
  if (valid_control_config(config)) {
    config_ = config;
  }
}

bool SpeedPiController::set_config(const ControlConfig& config) {
  if (!valid_control_config(config)) {
    return false;
  }
  config_ = config;
  reset();
  return true;
}

void SpeedPiController::reset() {
  integral_ = 0.0F;
}

float SpeedPiController::step(float target_speed_mps, float measured_speed_mps,
                              float dt_s) {
  if (!finite(target_speed_mps) || !finite(measured_speed_mps) ||
      !positive_finite(dt_s)) {
    return 0.0F;
  }
  target_speed_mps =
      clamp(target_speed_mps, 0.0F, config_.maximum_speed_mps);
  const float error = target_speed_mps - measured_speed_mps;
  const float candidate_integral =
      clamp(integral_ + error * dt_s, -config_.speed_integral_limit,
            config_.speed_integral_limit);
  const float candidate_output =
      config_.speed_kp * error + config_.speed_ki * candidate_integral;

  const bool saturated_high =
      candidate_output > config_.maximum_acceleration_mps2;
  const bool saturated_low =
      candidate_output < -config_.maximum_deceleration_mps2;
  if ((!saturated_high || error < 0.0F) && (!saturated_low || error > 0.0F)) {
    integral_ = candidate_integral;
  }

  const float output = config_.speed_kp * error + config_.speed_ki * integral_;
  return clamp(output, -config_.maximum_deceleration_mps2,
               config_.maximum_acceleration_mps2);
}

ControlSystem::ControlSystem(const ControlConfig& config)
    : config_(valid_control_config(config) ? config : ControlConfig::defaults()),
      route_(config_),
      speed_controller_(config_) {}

bool ControlSystem::set_config(const ControlConfig& config) {
  if (!valid_control_config(config)) {
    return false;
  }
  config_ = config;
  (void)route_.set_config(config);
  (void)speed_controller_.set_config(config);
  previous_steering_rad_ = 0.0F;
  return true;
}

bool ControlSystem::set_route(const Waypoint* waypoints, std::size_t count) {
  speed_controller_.reset();
  previous_steering_rad_ = 0.0F;
  return route_.set_route(waypoints, count);
}

void ControlSystem::reset() {
  route_.reset_progress();
  speed_controller_.reset();
  previous_steering_rad_ = 0.0F;
}

bool ControlSystem::estimate_is_finite(const EkfState& estimate) const {
  return finite(estimate.x_m) && finite(estimate.y_m) &&
         finite(estimate.heading_rad) && finite(estimate.speed_mps);
}

ControlCommand ControlSystem::neutral_command() const {
  ControlCommand command{};
  command.steering_rad = 0.0F;
  command.acceleration_mps2 = 0.0F;
  command.target_speed_mps = 0.0F;
  command.valid = false;
  command.safe_stop = false;
  command.route_complete = false;
  return command;
}

ControlCommand ControlSystem::safe_stop_command(float measured_speed_mps) const {
  ControlCommand command{};
  command.steering_rad = 0.0F;
  command.acceleration_mps2 =
      finite(measured_speed_mps) &&
              std::fabs(measured_speed_mps) > config_.final_speed_tolerance_mps
          ? (measured_speed_mps > 0.0F ? -config_.maximum_deceleration_mps2
                                      : config_.maximum_deceleration_mps2)
          : 0.0F;
  command.target_speed_mps = 0.0F;
  command.valid = true;
  command.safe_stop = true;
  command.route_complete = false;
  return command;
}

ControlCommand ControlSystem::step(const EkfState& estimate, float dt_s,
                                   bool force_safe_stop) {
  if (force_safe_stop) {
    speed_controller_.reset();
    previous_steering_rad_ = 0.0F;
    return safe_stop_command(estimate.speed_mps);
  }
  if (!estimate_is_finite(estimate) || !positive_finite(dt_s)) {
    speed_controller_.reset();
    previous_steering_rad_ = 0.0F;
    return neutral_command();
  }

  const RouteProgress progress = route_.update(estimate);
  if (!progress.valid) {
    speed_controller_.reset();
    previous_steering_rad_ = 0.0F;
    return neutral_command();
  }

  ControlCommand command{};
  command.valid = true;
  command.safe_stop = false;
  command.route_complete = progress.complete;
  command.target_speed_mps = progress.target_speed_mps;

  if (progress.complete) {
    speed_controller_.reset();
    previous_steering_rad_ = 0.0F;
    return command;
  }

  const float target_dx = progress.target_x_m - estimate.x_m;
  const float target_dy = progress.target_y_m - estimate.y_m;
  const float target_distance = std::hypot(target_dx, target_dy);
  float desired_steering = 0.0F;
  if (target_distance > 1.0e-5F) {
    const float target_heading = std::atan2(target_dy, target_dx);
    const float heading_error =
        normalize_angle(target_heading - estimate.heading_rad);
    desired_steering =
        std::atan2(2.0F * config_.wheelbase_m * std::sin(heading_error),
                   target_distance);
  }
  desired_steering = clamp(desired_steering, -config_.maximum_steering_rad,
                           config_.maximum_steering_rad);
  const float maximum_change = config_.maximum_steering_rate_rad_s * dt_s;
  command.steering_rad =
      clamp(desired_steering, previous_steering_rad_ - maximum_change,
            previous_steering_rad_ + maximum_change);
  command.steering_rad =
      clamp(command.steering_rad, -config_.maximum_steering_rad,
            config_.maximum_steering_rad);
  previous_steering_rad_ = command.steering_rad;

  command.acceleration_mps2 = speed_controller_.step(
      progress.target_speed_mps, estimate.speed_mps, dt_s);
  return command;
}

}  // namespace navbench
