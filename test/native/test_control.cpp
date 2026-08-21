#include <cmath>
#include <cstdio>

#include "navbench/control.hpp"
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

bool test_route_validation_and_progress() {
  navbench::ControlConfig config = navbench::ControlConfig::defaults();
  navbench::RouteManager route(config);
  CHECK(!route.set_route(nullptr, 1U));

  navbench::Waypoint invalid[1] = {{0.0F, 0.0F, -1.0F}};
  CHECK(!route.set_route(invalid, 1U));

  const navbench::Waypoint waypoints[3] = {
      {0.0F, 0.0F, 1.5F},
      {5.0F, 0.0F, 1.5F},
      {10.0F, 0.0F, 0.0F},
  };
  CHECK(route.set_route(waypoints, 3U));
  navbench::EkfState estimate{};
  navbench::RouteProgress progress = route.update(estimate);
  CHECK(progress.valid);
  CHECK(progress.active_waypoint == 1U);
  CHECK(progress.target_x_m > 1.0F);
  CHECK(close(progress.target_y_m, 0.0F, 1.0e-6F));
  CHECK(close(progress.target_speed_mps, 1.5F, 1.0e-6F));

  estimate.x_m = 5.0F;
  progress = route.update(estimate);
  CHECK(progress.active_waypoint == 2U);
  estimate.x_m = 10.0F;
  estimate.speed_mps = 1.0F;
  progress = route.update(estimate);
  CHECK(!progress.complete);
  CHECK(close(progress.target_speed_mps, 0.0F, 1.0e-6F));

  estimate.speed_mps = 0.0F;
  progress = route.update(estimate);
  CHECK(progress.complete);
  return true;
}

bool test_pi_anti_windup_and_saturation() {
  const navbench::ControlConfig config = navbench::ControlConfig::defaults();
  navbench::SpeedPiController speed(config);
  for (int i = 0; i < 200; ++i) {
    CHECK(close(speed.step(config.maximum_speed_mps, 0.0F, 0.02F),
                config.maximum_acceleration_mps2, 1.0e-6F));
  }
  CHECK(close(speed.integral(), 0.0F, 1.0e-6F));
  const float braking = speed.step(0.0F, 4.0F, 0.02F);
  CHECK(braking < 0.0F);
  CHECK(braking >= -config.maximum_deceleration_mps2);
  return true;
}

bool test_pure_pursuit_rate_limit_and_safe_stop() {
  const navbench::ControlConfig config = navbench::ControlConfig::defaults();
  navbench::ControlSystem controller(config);
  const navbench::Waypoint turn[2] = {
      {0.0F, 0.0F, 1.0F},
      {0.0F, 5.0F, 0.0F},
  };
  CHECK(controller.set_route(turn, 2U));
  navbench::EkfState estimate{};
  const navbench::ControlCommand command =
      controller.step(estimate, 0.02F, false);
  CHECK(command.valid);
  CHECK(!command.safe_stop);
  CHECK(command.steering_rad > 0.0F);
  CHECK(command.steering_rad <=
        config.maximum_steering_rate_rad_s * 0.02F + 1.0e-6F);
  CHECK(command.acceleration_mps2 > 0.0F);

  estimate.speed_mps = 2.0F;
  const navbench::ControlCommand stopped =
      controller.step(estimate, 0.02F, true);
  CHECK(stopped.valid);
  CHECK(stopped.safe_stop);
  CHECK(close(stopped.steering_rad, 0.0F, 1.0e-6F));
  CHECK(close(stopped.acceleration_mps2,
              -config.maximum_deceleration_mps2, 1.0e-6F));
  return true;
}

bool test_neutral_is_distinct_from_latched_safe_stop() {
  const navbench::ControlSystem controller;
  const navbench::ControlCommand neutral = controller.neutral_command();
  CHECK(!neutral.valid);
  CHECK(!neutral.safe_stop);
  CHECK(close(neutral.steering_rad, 0.0F, 1.0e-6F));
  CHECK(close(neutral.acceleration_mps2, 0.0F, 1.0e-6F));
  CHECK(close(neutral.target_speed_mps, 0.0F, 1.0e-6F));
  return true;
}

bool test_closed_loop_s_curve_and_final_stop() {
  navbench::ControlConfig config = navbench::ControlConfig::defaults();
  config.lookahead_m = 1.0F;
  config.waypoint_acceptance_radius_m = 0.55F;
  config.final_position_tolerance_m = 0.35F;
  navbench::ControlSystem controller(config);
  const navbench::Waypoint route[5] = {
      {0.0F, 0.0F, 1.5F},
      {5.0F, 0.0F, 1.5F},
      {10.0F, 2.0F, 1.5F},
      {15.0F, -2.0F, 1.5F},
      {20.0F, 0.0F, 0.0F},
  };
  CHECK(controller.set_route(route, 5U));

  navbench::EkfState estimate{};
  constexpr float dt_s = 0.02F;
  bool completed = false;
  float minimum_final_distance = 1.0e9F;
  for (int step = 0; step < 3500; ++step) {
    const navbench::ControlCommand command =
        controller.step(estimate, dt_s, false);
    CHECK(command.valid);
    CHECK(std::fabs(command.steering_rad) <=
          config.maximum_steering_rad + 1.0e-6F);
    CHECK(command.acceleration_mps2 <=
          config.maximum_acceleration_mps2 + 1.0e-6F);
    CHECK(command.acceleration_mps2 >=
          -config.maximum_deceleration_mps2 - 1.0e-6F);
    if (command.route_complete) {
      completed = true;
      break;
    }
    const float final_distance =
        std::hypot(estimate.x_m - route[4].x_m,
                   estimate.y_m - route[4].y_m);
    if (final_distance < minimum_final_distance) {
      minimum_final_distance = final_distance;
    }

    estimate.speed_mps =
        navbench::clamp(estimate.speed_mps + command.acceleration_mps2 * dt_s,
                        0.0F, config.maximum_speed_mps);
    estimate.heading_rad = navbench::normalize_angle(
        estimate.heading_rad +
        estimate.speed_mps / config.wheelbase_m *
            std::tan(command.steering_rad) * dt_s);
    estimate.x_m += estimate.speed_mps * std::cos(estimate.heading_rad) * dt_s;
    estimate.y_m += estimate.speed_mps * std::sin(estimate.heading_rad) * dt_s;
  }

  if (!completed) {
    std::fprintf(stderr,
                 "closed-loop did not finish: x=%.3f y=%.3f heading=%.3f "
                 "speed=%.3f waypoint=%u min_final=%.3f\n",
                 estimate.x_m, estimate.y_m, estimate.heading_rad,
                 estimate.speed_mps,
                 static_cast<unsigned>(controller.route().active_waypoint()),
                 minimum_final_distance);
  }
  CHECK(completed);
  CHECK(std::hypot(estimate.x_m - route[4].x_m,
                   estimate.y_m - route[4].y_m) <=
        config.final_position_tolerance_m);
  CHECK(std::fabs(estimate.speed_mps) <= config.final_speed_tolerance_mps);
  CHECK(controller.route().active_waypoint() == 4U);
  return true;
}

bool test_invalid_config_is_rejected() {
  navbench::ControlSystem controller;
  navbench::ControlConfig invalid = navbench::ControlConfig::defaults();
  invalid.lookahead_m = 0.0F;
  CHECK(!controller.set_config(invalid));
  return true;
}

}  // namespace

int main() {
  if (!test_route_validation_and_progress() ||
      !test_pi_anti_windup_and_saturation() ||
      !test_pure_pursuit_rate_limit_and_safe_stop() ||
      !test_neutral_is_distinct_from_latched_safe_stop() ||
      !test_closed_loop_s_curve_and_final_stop() ||
      !test_invalid_config_is_rejected()) {
    return 1;
  }
  std::printf("test_control: PASS (%d checks)\n", checks);
  return 0;
}
