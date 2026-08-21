#include "navbench/hmi.hpp"

#include <cmath>

namespace navbench {

HmiConfig HmiConfig::defaults() {
  HmiConfig config{};
  config.oled_period_ms = 500U;
  config.blink_period_ms = 250U;
  config.buzzer_enabled = false;
  config.servo_enabled = false;
  return config;
}

bool HmiController::valid_config(const HmiConfig& config) {
  return config.oled_period_ms >= 200U && config.oled_period_ms <= 5000U &&
         config.blink_period_ms >= 50U && config.blink_period_ms <= 2000U;
}

HmiController::HmiController(const HmiConfig& config)
    : config_(HmiConfig::defaults()) {
  if (valid_config(config)) {
    config_ = config;
  }
}

bool HmiController::set_config(const HmiConfig& config) {
  if (!valid_config(config)) {
    return false;
  }
  config_ = config;
  return true;
}

bool HmiController::elapsed(uint32_t now_ms, uint32_t then_ms,
                            uint32_t period_ms) {
  return static_cast<uint32_t>(now_ms - then_ms) >= period_ms;
}

void HmiController::begin(uint32_t now_ms, const HmiHal& hal) {
  started_ms_ = now_ms;
  last_oled_ms_ = now_ms - config_.oled_period_ms;
  frame_ = HmiFrame{};
  initialized_ = true;
  write_outputs(now_ms, hal);
  if (hal.render_oled != nullptr) {
    (void)hal.render_oled(hal.context, frame_);
    last_oled_ms_ = now_ms;
  }
}

void HmiController::update(uint32_t now_ms, const HmiStatus& status,
                           const HmiHal& hal) {
  if (!initialized_) {
    begin(now_ms, hal);
  }
  frame_.status = status;
  if (!std::isfinite(frame_.status.steering_normalized)) {
    frame_.status.steering_normalized = 0.0F;
  } else if (frame_.status.steering_normalized > 1.0F) {
    frame_.status.steering_normalized = 1.0F;
  } else if (frame_.status.steering_normalized < -1.0F) {
    frame_.status.steering_normalized = -1.0F;
  }
  frame_.button_mask =
      hal.read_buttons != nullptr ? hal.read_buttons(hal.context) : 0U;
  write_outputs(now_ms, hal);
  if (elapsed(now_ms, last_oled_ms_, config_.oled_period_ms) &&
      hal.render_oled != nullptr) {
    (void)hal.render_oled(hal.context, frame_);
    last_oled_ms_ = now_ms;
  }
}

void HmiController::write_outputs(uint32_t now_ms, const HmiHal& hal) {
  const uint32_t phase =
      static_cast<uint32_t>(now_ms - started_ms_) / config_.blink_period_ms;
  bool builtin = false;
  bool user = false;
  bool buzzer = false;
  switch (frame_.status.state) {
    case HmiState::Boot:
      builtin = (phase & 1U) == 0U;
      break;
    case HmiState::Ready:
      builtin = true;
      break;
    case HmiState::Running:
      builtin = (phase % 4U) == 0U;
      user = true;
      break;
    case HmiState::Degraded:
      builtin = (phase % 4U) < 2U;
      user = (phase & 1U) == 0U;
      break;
    case HmiState::SafeStop:
      builtin = (phase & 1U) == 0U;
      user = builtin;
      buzzer = (phase % 8U) == 0U;
      break;
    case HmiState::Fault:
      builtin = (phase & 1U) == 0U;
      user = !builtin;
      buzzer = (phase % 4U) < 2U;
      break;
  }
  if (hal.write_builtin_led != nullptr) {
    hal.write_builtin_led(hal.context, builtin);
  }
  if (hal.write_user_led != nullptr) {
    hal.write_user_led(hal.context, user);
  }
  if (hal.write_buzzer != nullptr) {
    hal.write_buzzer(hal.context, config_.buzzer_enabled && buzzer);
  }
  if (hal.write_servo != nullptr) {
    const bool permitted = config_.servo_enabled &&
                           frame_.status.state == HmiState::Running;
    hal.write_servo(hal.context, permitted,
                    permitted ? frame_.status.steering_normalized : 0.0F);
  }
}

}  // namespace navbench
