#pragma once

#include <Arduino.h>

#if NAVBENCH_HMI_SERVO_ENABLED
#include <Servo.h>
#endif

#include "navbench/hmi.hpp"

#ifndef NAVBENCH_HMI_OLED_ENABLED
#define NAVBENCH_HMI_OLED_ENABLED 1
#endif
#ifndef NAVBENCH_HMI_USER_LED_ENABLED
#define NAVBENCH_HMI_USER_LED_ENABLED 0
#endif
#ifndef NAVBENCH_HMI_BUZZER_ENABLED
#define NAVBENCH_HMI_BUZZER_ENABLED 0
#endif
#ifndef NAVBENCH_HMI_BUTTON1_ENABLED
#define NAVBENCH_HMI_BUTTON1_ENABLED 0
#endif
#ifndef NAVBENCH_HMI_BUTTON2_ENABLED
#define NAVBENCH_HMI_BUTTON2_ENABLED 0
#endif
#ifndef NAVBENCH_HMI_SERVO_ENABLED
#define NAVBENCH_HMI_SERVO_ENABLED 0
#endif

#if NAVBENCH_HMI_USER_LED_ENABLED && \
    (!defined(NAVBENCH_HMI_USER_LED_PIN) || \
     !defined(NAVBENCH_HMI_USER_LED_ACTIVE_LEVEL))
#error "Enabled user LED requires explicit PIN and ACTIVE_LEVEL"
#endif
#if NAVBENCH_HMI_BUZZER_ENABLED && \
    (!defined(NAVBENCH_HMI_BUZZER_PIN) || \
     !defined(NAVBENCH_HMI_BUZZER_ACTIVE_LEVEL))
#error "Enabled buzzer requires explicit PIN and ACTIVE_LEVEL"
#endif
#if NAVBENCH_HMI_BUTTON1_ENABLED && \
    (!defined(NAVBENCH_HMI_BUTTON1_PIN) || \
     !defined(NAVBENCH_HMI_BUTTON1_ACTIVE_LEVEL))
#error "Enabled button 1 requires explicit PIN and ACTIVE_LEVEL"
#endif
#if NAVBENCH_HMI_BUTTON2_ENABLED && \
    (!defined(NAVBENCH_HMI_BUTTON2_PIN) || \
     !defined(NAVBENCH_HMI_BUTTON2_ACTIVE_LEVEL))
#error "Enabled button 2 requires explicit PIN and ACTIVE_LEVEL"
#endif
#if NAVBENCH_HMI_SERVO_ENABLED && \
    (!defined(NAVBENCH_HMI_SERVO_PIN) || \
     !defined(NAVBENCH_SERVO_PHYSICALLY_QUALIFIED) || \
     NAVBENCH_SERVO_PHYSICALLY_QUALIFIED != 1)
#error "Servo requires an explicit pin and physical power/common-ground qualification"
#endif

namespace navbench {

class ArduinoHmiHal {
 public:
  ArduinoHmiHal();
  void begin();
  HmiHal callbacks();
  bool oled_detected() const { return oled_detected_; }

 private:
  static void builtin_callback(void* context, bool on);
  static void user_callback(void* context, bool on);
  static void buzzer_callback(void* context, bool on);
  static void servo_callback(void* context, bool enabled, float normalized);
  static uint8_t buttons_callback(void* context);
  static bool oled_callback(void* context, const HmiFrame& frame);

  void write_builtin(bool on);
  void write_user(bool on);
  void write_buzzer(bool on);
  void write_servo(bool enabled, float normalized);
  uint8_t read_buttons() const;
  bool render(const HmiFrame& frame);
  bool oled_command(uint8_t command);
  bool oled_bytes(uint8_t control, const uint8_t* bytes, uint8_t size);
  void oled_clear();
  void oled_text(uint8_t page, uint8_t column, const char* text);

  bool oled_detected_{false};
#if NAVBENCH_HMI_SERVO_ENABLED
  Servo servo_{};
  bool servo_attached_{false};
#endif
};

}  // namespace navbench
