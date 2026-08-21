#include <Arduino.h>

#include "navbench/firmware_session.hpp"

namespace {

constexpr unsigned long kSerialBaud = 115200UL;
constexpr uint32_t kLedHeartbeatPeriodMs = 1000U;
constexpr size_t kMaximumRxBytesPerLoop = 64U;
constexpr size_t kMaximumTxBytesPerLoop = 32U;

#if defined(NAVBENCH_SERIAL_DIAGNOSTIC)
constexpr uint32_t kDiagnosticPeriodMs = 100U;
constexpr uint32_t kDiagnosticMagic = 0x4e424447UL;  // "NBDG"
enum class DiagnosticEvent : uint16_t {
  Beacon = 1U,
  Receive = 2U,
  Parser = 3U,
  Transmit = 4U,
  Queue = 5U,
};
#endif

navbench::FirmwareSession firmwareSession;
uint32_t lastLedHeartbeatMs = 0U;
bool ledOn = false;
uint8_t pendingTx[navbench::protocol::kMaxWireFrameSize]{};
size_t pendingTxSize = 0U;
size_t pendingTxOffset = 0U;

#if defined(NAVBENCH_SERIAL_DIAGNOSTIC)
uint32_t diagnosticSequence = 0x80000000UL;
uint32_t lastDiagnosticMs = 0U;
uint32_t serialTxBytes = 0U;
uint16_t lastWriteRequested = 0U;
uint16_t lastWriteResult = 0U;
uint8_t diagnosticEventIndex = 0U;
#endif

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

bool stageDiagnosticFrame(uint32_t nowMs) {
#if defined(NAVBENCH_SERIAL_DIAGNOSTIC)
  if (nowMs - lastDiagnosticMs < kDiagnosticPeriodMs) {
    return false;
  }
  lastDiagnosticMs = nowMs;
  diagnosticEventIndex = static_cast<uint8_t>((diagnosticEventIndex % 5U) + 1U);

  const navbench::protocol::ParserStats& parser =
      firmwareSession.parser_stats();
  const navbench::FirmwareSessionStats& session = firmwareSession.stats();
  navbench::protocol::ErrorPayload diagnostic{};
  diagnostic.code = navbench::protocol::ApplicationErrorCode::Diagnostic;
  diagnostic.detail = diagnosticEventIndex;
  switch (static_cast<DiagnosticEvent>(diagnosticEventIndex)) {
    case DiagnosticEvent::Beacon:
      diagnostic.related_sequence = kDiagnosticMagic;
      diagnostic.context = nowMs;
      break;
    case DiagnosticEvent::Receive:
      diagnostic.related_sequence = parser.bytes_received;
      diagnostic.context = parser.frames_received;
      break;
    case DiagnosticEvent::Parser:
      diagnostic.related_sequence =
          (static_cast<uint32_t>(session.diagnostic_last_hello_result) << 24U) |
          (static_cast<uint32_t>(session.diagnostic_last_parser_status) << 16U) |
          (session.diagnostic_hello_packets & 0xffffU);
      diagnostic.context = parser.cobs_errors + parser.crc_errors +
                           parser.version_errors + parser.type_errors +
                           parser.length_errors + parser.other_errors;
      break;
    case DiagnosticEvent::Transmit:
      diagnostic.related_sequence =
          (static_cast<uint32_t>(lastWriteRequested) << 16U) |
          static_cast<uint32_t>(lastWriteResult);
      diagnostic.context = serialTxBytes;
      break;
    case DiagnosticEvent::Queue:
      diagnostic.related_sequence = session.tx_frames;
      diagnostic.context =
          (session.tx_dropped << 16U) |
          static_cast<uint32_t>(firmwareSession.pending_frames() & 0xffffU);
      break;
  }

  uint8_t payload[navbench::protocol::kMaxPayloadSize]{};
  uint16_t payloadSize = 0U;
  if (navbench::protocol::encodePayload(
          diagnostic, payload, sizeof(payload), &payloadSize) !=
      navbench::protocol::Status::Ok) {
    return false;
  }
  size_t frameSize = 0U;
  if (navbench::protocol::encodePacket(
          navbench::protocol::MessageType::Error, diagnosticSequence++,
          firmwareSession.last_step_id(), payload, payloadSize, pendingTx,
          sizeof(pendingTx), &frameSize) != navbench::protocol::Status::Ok) {
    return false;
  }
  pendingTxSize = frameSize;
  return true;
#else
  (void)nowMs;
  return false;
#endif
}

void transmitSerial(uint32_t nowMs) {
  if (pendingTxOffset == pendingTxSize) {
    pendingTxOffset = 0U;
    pendingTxSize = 0U;
    if (!firmwareSession.pop_frame(pendingTx, sizeof(pendingTx),
                                   &pendingTxSize)) {
      (void)stageDiagnosticFrame(nowMs);
    }
  }
  if (pendingTxSize == 0U) {
    return;
  }

  const size_t remaining = pendingTxSize - pendingTxOffset;
  size_t count =
      remaining < kMaximumTxBytesPerLoop ? remaining : kMaximumTxBytesPerLoop;
#if !defined(NO_USB)
  // UNO R4 WiFi's default ESP32-S3 bridge is _UART1_. Its UART class inherits
  // Print::availableForWrite(), which always returns zero, so that method is
  // meaningful only for the native USB Serial implementation.
  const int available = Serial.availableForWrite();
  if (available <= 0) {
    return;
  }
  const size_t writable = static_cast<size_t>(available);
  if (count > writable) {
    count = writable;
  }
#endif
  const size_t written = Serial.write(pendingTx + pendingTxOffset, count);
  if (written <= count) {
    pendingTxOffset += written;
  }
#if defined(NAVBENCH_SERIAL_DIAGNOSTIC)
  lastWriteRequested = static_cast<uint16_t>(count);
  lastWriteResult =
      static_cast<uint16_t>(written <= 0xffffU ? written : 0xffffU);
  serialTxBytes += static_cast<uint32_t>(written);
#endif
}

}  // namespace

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(kSerialBaud);
  const uint32_t nowMs = millis();
  firmwareSession.reset(nowMs);
  lastLedHeartbeatMs = nowMs;
#if defined(NAVBENCH_SERIAL_DIAGNOSTIC)
  lastDiagnosticMs = nowMs - kDiagnosticPeriodMs;
#endif
}

void loop() {
  const uint32_t loopStartUs = micros();
  const uint32_t nowMs = millis();

  receiveSerial(nowMs);
  firmwareSession.tick(nowMs, firmwareSession.last_step_id());
  transmitSerial(nowMs);

  if (nowMs - lastLedHeartbeatMs >= kLedHeartbeatPeriodMs) {
    lastLedHeartbeatMs = nowMs;
    ledOn = !ledOn;
    digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
  }

  firmwareSession.record_loop_duration(micros() - loopStartUs);
}
