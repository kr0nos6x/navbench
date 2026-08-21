#include <cmath>
#include <cstdio>
#include <limits>

#include "navbench/hmi.hpp"

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

struct MockHal {
  bool builtin{false};
  bool user{false};
  bool buzzer{false};
  bool servo_enabled{false};
  float servo_position{0.0F};
  uint8_t buttons{0U};
  uint32_t renders{0U};
  navbench::HmiFrame rendered{};
};

void builtin(void* context, bool value) { static_cast<MockHal*>(context)->builtin = value; }
void user(void* context, bool value) { static_cast<MockHal*>(context)->user = value; }
void buzzer(void* context, bool value) { static_cast<MockHal*>(context)->buzzer = value; }
void servo(void* context, bool enabled, float position) {
  MockHal& mock = *static_cast<MockHal*>(context);
  mock.servo_enabled = enabled; mock.servo_position = position;
}
uint8_t buttons(void* context) { return static_cast<MockHal*>(context)->buttons; }
bool render(void* context, const navbench::HmiFrame& frame) {
  MockHal& mock = *static_cast<MockHal*>(context);
  ++mock.renders; mock.rendered = frame; return true;
}
navbench::HmiHal callbacks(MockHal& mock) {
  navbench::HmiHal hal{}; hal.context=&mock; hal.write_builtin_led=builtin;
  hal.write_user_led=user; hal.write_buzzer=buzzer; hal.write_servo=servo;
  hal.read_buttons=buttons; hal.render_oled=render; return hal;
}

bool test_defaults_are_safe_and_oled_is_rate_limited() {
  MockHal mock{}; navbench::HmiController hmi;
  const navbench::HmiHal hal = callbacks(mock);
  hmi.begin(100U, hal);
  CHECK(mock.renders == 1U); CHECK(!mock.buzzer); CHECK(!mock.servo_enabled);
  navbench::HmiStatus status{}; status.state=navbench::HmiState::Running;
  status.host_connected=true; status.estimator_healthy=true;
  status.steering_normalized=0.5F; mock.buttons=3U;
  hmi.update(200U, status, hal);
  CHECK(mock.renders == 1U); CHECK(hmi.button_mask() == 3U);
  CHECK(!mock.servo_enabled); CHECK(!mock.buzzer);
  hmi.update(600U, status, hal);
  CHECK(mock.renders == 2U); CHECK(mock.rendered.status.host_connected);
  return true;
}

bool test_all_states_and_optional_outputs() {
  navbench::HmiConfig config = navbench::HmiConfig::defaults();
  config.buzzer_enabled=true; config.servo_enabled=true;
  MockHal mock{}; navbench::HmiController hmi(config);
  const navbench::HmiHal hal=callbacks(mock); hmi.begin(0U, hal);
  navbench::HmiStatus status{};
  status.state=navbench::HmiState::Ready; hmi.update(0U,status,hal);
  CHECK(mock.builtin); CHECK(!mock.user); CHECK(!mock.servo_enabled);
  status.state=navbench::HmiState::Running; status.steering_normalized=2.0F;
  hmi.update(250U,status,hal); CHECK(mock.servo_enabled);
  CHECK(std::fabs(mock.servo_position-1.0F)<1.0e-6F);
  status.state=navbench::HmiState::Degraded; hmi.update(500U,status,hal);
  CHECK(!mock.servo_enabled);
  status.state=navbench::HmiState::SafeStop; hmi.update(0U,status,hal);
  CHECK(mock.buzzer); CHECK(!mock.servo_enabled);
  status.state=navbench::HmiState::Fault; hmi.update(250U,status,hal);
  CHECK(mock.buzzer); CHECK(!mock.servo_enabled);
  status.steering_normalized=std::numeric_limits<float>::quiet_NaN();
  hmi.update(500U,status,hal); CHECK(hmi.frame().status.steering_normalized==0.0F);
  return true;
}

bool test_invalid_configuration_is_rejected() {
  navbench::HmiController hmi;
  navbench::HmiConfig config=navbench::HmiConfig::defaults();
  config.oled_period_ms=10U; CHECK(!hmi.set_config(config));
  config=navbench::HmiConfig::defaults(); config.blink_period_ms=0U;
  CHECK(!hmi.set_config(config)); return true;
}
}  // namespace

int main() {
  if (!test_defaults_are_safe_and_oled_is_rate_limited() ||
      !test_all_states_and_optional_outputs() ||
      !test_invalid_configuration_is_rejected()) return 1;
  std::printf("test_hmi: PASS (%d checks)\n", checks); return 0;
}
