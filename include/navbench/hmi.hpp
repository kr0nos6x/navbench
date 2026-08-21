#pragma once

#include <cstdint>

namespace navbench {

enum class HmiState : uint8_t {
  Boot = 0U,
  Ready = 1U,
  Running = 2U,
  Degraded = 3U,
  SafeStop = 4U,
  Fault = 5U,
};

struct HmiConfig {
  uint32_t oled_period_ms;
  uint32_t blink_period_ms;
  bool buzzer_enabled;
  bool servo_enabled;

  static HmiConfig defaults();
};

struct HmiStatus {
  HmiState state{HmiState::Boot};
  uint8_t navigation_mode{0U};
  bool host_connected{false};
  bool estimator_healthy{false};
  float steering_normalized{0.0F};
};

struct HmiFrame {
  HmiStatus status{};
  uint8_t button_mask{0U};
};

// Fixed callback table: no allocation, Arduino String, or dependency on the
// navigation/control types. Missing optional callbacks are safe no-ops.
struct HmiHal {
  void* context{nullptr};
  void (*write_builtin_led)(void*, bool){nullptr};
  void (*write_user_led)(void*, bool){nullptr};
  void (*write_buzzer)(void*, bool){nullptr};
  void (*write_servo)(void*, bool, float){nullptr};
  uint8_t (*read_buttons)(void*){nullptr};
  bool (*render_oled)(void*, const HmiFrame&){nullptr};
};

class HmiController {
 public:
  explicit HmiController(const HmiConfig& config = HmiConfig::defaults());

  bool set_config(const HmiConfig& config);
  void begin(uint32_t now_ms, const HmiHal& hal);
  void update(uint32_t now_ms, const HmiStatus& status, const HmiHal& hal);

  uint8_t button_mask() const { return frame_.button_mask; }
  const HmiFrame& frame() const { return frame_; }

 private:
  static bool valid_config(const HmiConfig& config);
  static bool elapsed(uint32_t now_ms, uint32_t then_ms,
                      uint32_t period_ms);
  void write_outputs(uint32_t now_ms, const HmiHal& hal);

  HmiConfig config_{};
  HmiFrame frame_{};
  uint32_t started_ms_{0U};
  uint32_t last_oled_ms_{0U};
  bool initialized_{false};
};

}  // namespace navbench
