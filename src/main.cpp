#include <Arduino.h>

#include "navbench/firmware_session.hpp"

namespace {

constexpr unsigned long kSerialBaud = 115200UL;
constexpr uint32_t kLedHeartbeatPeriodMs = 1000U;
constexpr size_t kMaximumRxBytesPerLoop = 64U;

navbench::FirmwareSession firmwareSession;
uint32_t lastLedHeartbeatMs = 0U;
bool ledOn = false;
uint8_t pendingTx[navbench::protocol::kMaxWireFrameSize]{};
size_t pendingTxSize = 0U;
size_t pendingTxOffset = 0U;

void receiveSerial(uint32_t nowMs) {
  uint8_t input[kMaximumRxBytesPerLoop]{};
  size_t count = 0U;
  while (count < sizeof(input) && Serial.available() > 0) {
    const int value = Serial.read();
    if (value < 0) {
      break;
    }
    input[count++] = static_cast<uint8_t>(value);
  }
  if (count != 0U) {
    firmwareSession.feed(nowMs, input, count);
  }
}

void transmitSerial() {
  if (pendingTxOffset == pendingTxSize) {
    pendingTxOffset = 0U;
    pendingTxSize = 0U;
    (void)firmwareSession.pop_frame(pendingTx, sizeof(pendingTx),
                                    &pendingTxSize);
  }
  if (pendingTxSize == 0U) {
    return;
  }

  const int available = Serial.availableForWrite();
  if (available <= 0) {
    return;
  }
  const size_t remaining = pendingTxSize - pendingTxOffset;
  const size_t writable = static_cast<size_t>(available);
  const size_t count = remaining < writable ? remaining : writable;
  pendingTxOffset += Serial.write(pendingTx + pendingTxOffset, count);
}

}  // namespace

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(kSerialBaud);
  const uint32_t nowMs = millis();
  firmwareSession.reset(nowMs);
  lastLedHeartbeatMs = nowMs;
}

void loop() {
  const uint32_t loopStartUs = micros();
  const uint32_t nowMs = millis();

  receiveSerial(nowMs);
  firmwareSession.tick(nowMs, firmwareSession.last_step_id());
  transmitSerial();

  if (nowMs - lastLedHeartbeatMs >= kLedHeartbeatPeriodMs) {
    lastLedHeartbeatMs = nowMs;
    ledOn = !ledOn;
    digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
  }

  firmwareSession.record_loop_duration(micros() - loopStartUs);
}
