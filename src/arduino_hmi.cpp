#include "navbench/arduino_hmi.hpp"

#include <Wire.h>

namespace navbench {
namespace {

constexpr uint8_t kOledAddress = 0x3CU;
constexpr uint8_t kOledColumns = 128U;
constexpr uint8_t kRenderedPages = 4U;

const char* state_text(HmiState state) {
  switch (state) {
    case HmiState::Boot:
      return "BOOT";
    case HmiState::Ready:
      return "READY";
    case HmiState::Running:
      return "RUNNING";
    case HmiState::Degraded:
      return "DEGRADED";
    case HmiState::SafeStop:
      return "SAFE STOP";
    case HmiState::Fault:
      return "FAULT";
  }
  return "FAULT";
}

void glyph(char character, uint8_t output[5]) {
  uint8_t a = 0U, b = 0U, c = 0U, d = 0U, e = 0U;
  switch (character) {
    case 'A': a=0x7e;b=0x11;c=0x11;d=0x11;e=0x7e; break;
    case 'B': a=0x7f;b=0x49;c=0x49;d=0x49;e=0x36; break;
    case 'C': a=0x3e;b=0x41;c=0x41;d=0x41;e=0x22; break;
    case 'D': a=0x7f;b=0x41;c=0x41;d=0x22;e=0x1c; break;
    case 'E': a=0x7f;b=0x49;c=0x49;d=0x49;e=0x41; break;
    case 'F': a=0x7f;b=0x09;c=0x09;d=0x09;e=0x01; break;
    case 'G': a=0x3e;b=0x41;c=0x49;d=0x49;e=0x7a; break;
    case 'H': a=0x7f;b=0x08;c=0x08;d=0x08;e=0x7f; break;
    case 'I': a=0x00;b=0x41;c=0x7f;d=0x41;e=0x00; break;
    case 'K': a=0x7f;b=0x08;c=0x14;d=0x22;e=0x41; break;
    case 'L': a=0x7f;b=0x40;c=0x40;d=0x40;e=0x40; break;
    case 'N': a=0x7f;b=0x02;c=0x04;d=0x08;e=0x7f; break;
    case 'O': a=0x3e;b=0x41;c=0x41;d=0x41;e=0x3e; break;
    case 'P': a=0x7f;b=0x09;c=0x09;d=0x09;e=0x06; break;
    case 'R': a=0x7f;b=0x09;c=0x19;d=0x29;e=0x46; break;
    case 'S': a=0x46;b=0x49;c=0x49;d=0x49;e=0x31; break;
    case 'T': a=0x01;b=0x01;c=0x7f;d=0x01;e=0x01; break;
    case 'U': a=0x3f;b=0x40;c=0x40;d=0x40;e=0x3f; break;
    case 'V': a=0x1f;b=0x20;c=0x40;d=0x20;e=0x1f; break;
    case 'Y': a=0x03;b=0x04;c=0x78;d=0x04;e=0x03; break;
    case '0': a=0x3e;b=0x51;c=0x49;d=0x45;e=0x3e; break;
    case '1': a=0x00;b=0x42;c=0x7f;d=0x40;e=0x00; break;
    case '2': a=0x42;b=0x61;c=0x51;d=0x49;e=0x46; break;
    case '3': a=0x21;b=0x41;c=0x45;d=0x4b;e=0x31; break;
    case '4': a=0x18;b=0x14;c=0x12;d=0x7f;e=0x10; break;
    case '5': a=0x27;b=0x45;c=0x45;d=0x45;e=0x39; break;
    case '-': a=0x08;b=0x08;c=0x08;d=0x08;e=0x08; break;
    case ':': a=0x00;b=0x36;c=0x36;d=0x00;e=0x00; break;
    default: break;
  }
  output[0]=a; output[1]=b; output[2]=c; output[3]=d; output[4]=e;
}

}  // namespace

ArduinoHmiHal::ArduinoHmiHal() = default;

void ArduinoHmiHal::begin() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
#if NAVBENCH_HMI_USER_LED_ENABLED
  pinMode(NAVBENCH_HMI_USER_LED_PIN, OUTPUT);
  digitalWrite(NAVBENCH_HMI_USER_LED_PIN,
               NAVBENCH_HMI_USER_LED_ACTIVE_LEVEL ? LOW : HIGH);
#endif
#if NAVBENCH_HMI_BUZZER_ENABLED
  pinMode(NAVBENCH_HMI_BUZZER_PIN, OUTPUT);
  digitalWrite(NAVBENCH_HMI_BUZZER_PIN,
               NAVBENCH_HMI_BUZZER_ACTIVE_LEVEL ? LOW : HIGH);
#endif
#if NAVBENCH_HMI_BUTTON1_ENABLED
  pinMode(NAVBENCH_HMI_BUTTON1_PIN, INPUT);
#endif
#if NAVBENCH_HMI_BUTTON2_ENABLED
  pinMode(NAVBENCH_HMI_BUTTON2_PIN, INPUT);
#endif
#if NAVBENCH_HMI_SERVO_ENABLED
  servo_.attach(NAVBENCH_HMI_SERVO_PIN);
  servo_attached_ = true;
  servo_.write(90);
#endif
#if NAVBENCH_HMI_OLED_ENABLED
  Wire.begin();
  Wire.setClock(400000UL);
  const uint8_t initialization[] = {
      0xaeU, 0xd5U, 0x80U, 0xa8U, 0x3fU, 0xd3U, 0x00U, 0x40U,
      0x8dU, 0x14U, 0x20U, 0x02U, 0xa1U, 0xc8U, 0xdaU, 0x12U,
      0x81U, 0x7fU, 0xd9U, 0xf1U, 0xdbU, 0x40U, 0xa4U, 0xa6U,
      0xafU,
  };
  oled_detected_ = true;
  for (uint8_t command : initialization) {
    if (!oled_command(command)) {
      oled_detected_ = false;
      break;
    }
  }
  if (oled_detected_) {
    oled_clear();
  }
#endif
}

HmiHal ArduinoHmiHal::callbacks() {
  HmiHal hal{};
  hal.context = this;
  hal.write_builtin_led = &ArduinoHmiHal::builtin_callback;
  hal.write_user_led = &ArduinoHmiHal::user_callback;
  hal.write_buzzer = &ArduinoHmiHal::buzzer_callback;
  hal.write_servo = &ArduinoHmiHal::servo_callback;
  hal.read_buttons = &ArduinoHmiHal::buttons_callback;
  hal.render_oled = &ArduinoHmiHal::oled_callback;
  return hal;
}

void ArduinoHmiHal::builtin_callback(void* context, bool on) {
  static_cast<ArduinoHmiHal*>(context)->write_builtin(on);
}
void ArduinoHmiHal::user_callback(void* context, bool on) {
  static_cast<ArduinoHmiHal*>(context)->write_user(on);
}
void ArduinoHmiHal::buzzer_callback(void* context, bool on) {
  static_cast<ArduinoHmiHal*>(context)->write_buzzer(on);
}
void ArduinoHmiHal::servo_callback(void* context, bool enabled,
                                   float normalized) {
  static_cast<ArduinoHmiHal*>(context)->write_servo(enabled, normalized);
}
uint8_t ArduinoHmiHal::buttons_callback(void* context) {
  return static_cast<ArduinoHmiHal*>(context)->read_buttons();
}
bool ArduinoHmiHal::oled_callback(void* context, const HmiFrame& frame) {
  return static_cast<ArduinoHmiHal*>(context)->render(frame);
}

void ArduinoHmiHal::write_builtin(bool on) {
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
}
void ArduinoHmiHal::write_user(bool on) {
#if NAVBENCH_HMI_USER_LED_ENABLED
  digitalWrite(NAVBENCH_HMI_USER_LED_PIN,
               on == static_cast<bool>(NAVBENCH_HMI_USER_LED_ACTIVE_LEVEL)
                   ? HIGH : LOW);
#else
  (void)on;
#endif
}
void ArduinoHmiHal::write_buzzer(bool on) {
#if NAVBENCH_HMI_BUZZER_ENABLED
  digitalWrite(NAVBENCH_HMI_BUZZER_PIN,
               on == static_cast<bool>(NAVBENCH_HMI_BUZZER_ACTIVE_LEVEL)
                   ? HIGH : LOW);
#else
  (void)on;
#endif
}
void ArduinoHmiHal::write_servo(bool enabled, float normalized) {
#if NAVBENCH_HMI_SERVO_ENABLED
  if (servo_attached_) {
    const float bounded = normalized < -1.0F ? -1.0F :
                          (normalized > 1.0F ? 1.0F : normalized);
    servo_.write(enabled ? static_cast<int>(90.0F + 45.0F * bounded) : 90);
  }
#else
  (void)enabled;
  (void)normalized;
#endif
}
uint8_t ArduinoHmiHal::read_buttons() const {
  uint8_t mask = 0U;
#if NAVBENCH_HMI_BUTTON1_ENABLED
  if (digitalRead(NAVBENCH_HMI_BUTTON1_PIN) ==
      NAVBENCH_HMI_BUTTON1_ACTIVE_LEVEL) mask |= 1U;
#endif
#if NAVBENCH_HMI_BUTTON2_ENABLED
  if (digitalRead(NAVBENCH_HMI_BUTTON2_PIN) ==
      NAVBENCH_HMI_BUTTON2_ACTIVE_LEVEL) mask |= 2U;
#endif
  return mask;
}

bool ArduinoHmiHal::oled_bytes(uint8_t control, const uint8_t* bytes,
                               uint8_t size) {
#if NAVBENCH_HMI_OLED_ENABLED
  Wire.beginTransmission(kOledAddress);
  Wire.write(control);
  Wire.write(bytes, size);
  return Wire.endTransmission() == 0U;
#else
  (void)control; (void)bytes; (void)size; return false;
#endif
}
bool ArduinoHmiHal::oled_command(uint8_t command) {
  return oled_bytes(0x00U, &command, 1U);
}
void ArduinoHmiHal::oled_clear() {
  uint8_t zeros[16]{};
  for (uint8_t page = 0U; page < 8U; ++page) {
    (void)oled_command(static_cast<uint8_t>(0xb0U + page));
    (void)oled_command(0x00U); (void)oled_command(0x10U);
    for (uint8_t column = 0U; column < kOledColumns; column += sizeof(zeros)) {
      (void)oled_bytes(0x40U, zeros, sizeof(zeros));
    }
  }
}
void ArduinoHmiHal::oled_text(uint8_t page, uint8_t column, const char* text) {
  (void)oled_command(static_cast<uint8_t>(0xb0U + page));
  (void)oled_command(static_cast<uint8_t>(column & 0x0fU));
  (void)oled_command(static_cast<uint8_t>(0x10U | (column >> 4U)));
  while (*text != '\0') {
    uint8_t pixels[6]{};
    glyph(*text++, pixels);
    (void)oled_bytes(0x40U, pixels, sizeof(pixels));
  }
}
bool ArduinoHmiHal::render(const HmiFrame& frame) {
  if (!oled_detected_) return false;
  uint8_t zeros[16]{};
  for (uint8_t page = 0U; page < kRenderedPages; ++page) {
    (void)oled_command(static_cast<uint8_t>(0xb0U + page));
    (void)oled_command(0x00U); (void)oled_command(0x10U);
    for (uint8_t column = 0U; column < kOledColumns; column += sizeof(zeros)) {
      if (!oled_bytes(0x40U, zeros, sizeof(zeros))) return false;
    }
  }
  oled_text(0U, 0U, "NAVBENCH");
  oled_text(1U, 0U, state_text(frame.status.state));
  oled_text(2U, 0U, frame.status.host_connected ? "LINK OK" : "LINK LOST");
  char navigation[] = "NAV:0";
  navigation[4] = static_cast<char>('0' +
      (frame.status.navigation_mode <= 5U ? frame.status.navigation_mode : 0U));
  oled_text(3U, 0U, navigation);
  return true;
}

}  // namespace navbench
