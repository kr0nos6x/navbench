#include <Arduino.h>

namespace {

constexpr unsigned long kSerialBaud = 115200;
constexpr unsigned long kHeartbeatPeriodMs = 1000;
constexpr char kBuildId[] = "navbench-m1-bringup-v1";

unsigned long lastHeartbeatMs = 0;
bool ledOn = false;

}  // namespace

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(kSerialBaud);

  const unsigned long serialWaitStartMs = millis();
  while (!Serial && (millis() - serialWaitStartMs < 2000)) {
  }

  Serial.println();
  Serial.println("NAVBENCH_BOOT");
  Serial.print("BUILD_ID=");
  Serial.println(kBuildId);
  Serial.println("BOARD=ARDUINO_UNO_R4_WIFI");
  Serial.println("STATUS=READY");
}

void loop() {
  const unsigned long nowMs = millis();

  if (nowMs - lastHeartbeatMs >= kHeartbeatPeriodMs) {
    lastHeartbeatMs = nowMs;
    ledOn = !ledOn;
    digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);

    Serial.print("HEARTBEAT_MS=");
    Serial.println(nowMs);
  }
}
