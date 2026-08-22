#include <Arduino.h>

#include "navbench/arduino_hmi.hpp"
#include "navbench/firmware_session.hpp"
#include "navbench/serial_io.hpp"

namespace {

constexpr unsigned long kSerialBaud = 115200UL;
constexpr size_t kMaximumRxBytesPerLoop = 64U;
constexpr size_t kMaximumTxBytesPerLoop = 32U;

#if defined(NAVBENCH_SERIAL_DIAGNOSTIC)
constexpr uint32_t kDiagnosticMagic = 0x4e424447UL;  // "NBDG"
#endif

navbench::FirmwareSession firmwareSession;
navbench::ArduinoHmiHal boardHmi;
navbench::HmiController hmi;
navbench::HmiHal hmiHal;
navbench::SerialRxCounter serialRxCounter;
navbench::SerialTxStager serialTxStager;

#if defined(NAVBENCH_SERIAL_DIAGNOSTIC)
uint32_t diagnosticSequence = 0x80000000UL;
uint16_t lastWriteRequested = 0U;
uint16_t lastWriteResult = 0U;
navbench::DiagnosticScheduler diagnosticScheduler;
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
    serialRxCounter.add(count);
    firmwareSession.feed(nowMs, input, count);
  }
}

bool stageDiagnosticFrame(uint32_t nowMs) {
#if defined(NAVBENCH_SERIAL_DIAGNOSTIC)
  navbench::DiagnosticSnapshot snapshot{};
  const navbench::protocol::ParserStats& parser =
      firmwareSession.parser_stats();
  const navbench::FirmwareSessionStats& session = firmwareSession.stats();
  snapshot.serial_rx_bytes = serialRxCounter.bytes();
  snapshot.parser_frames_received = parser.frames_received;
  snapshot.parser_errors = parser.cobs_errors + parser.crc_errors +
                           parser.version_errors + parser.type_errors +
                           parser.length_errors + parser.other_errors;
  snapshot.hello_packets = session.diagnostic_hello_packets;
  snapshot.response_frames_created = session.tx_frames;
  snapshot.response_frames_dropped = session.tx_dropped;
  snapshot.response_frames_pending = static_cast<uint16_t>(
      firmwareSession.pending_frames() & 0xffffU);
  snapshot.last_write_requested = lastWriteRequested;
  snapshot.last_write_result = lastWriteResult;
  snapshot.parser_status = session.diagnostic_last_parser_status;
  snapshot.hello_result = session.diagnostic_last_hello_result;

  navbench::DiagnosticEvent event = navbench::DiagnosticEvent::Beacon;
  if (!diagnosticScheduler.next(nowMs, snapshot, &event)) {
    return false;
  }

  navbench::protocol::ErrorPayload diagnostic{};
  diagnostic.code = navbench::protocol::ApplicationErrorCode::Diagnostic;
  diagnostic.detail = static_cast<uint16_t>(event);
  switch (event) {
    case navbench::DiagnosticEvent::Beacon:
      diagnostic.related_sequence = kDiagnosticMagic;
      diagnostic.context = nowMs;
      break;
    case navbench::DiagnosticEvent::Receive:
      diagnostic.related_sequence = snapshot.serial_rx_bytes;
      diagnostic.context = snapshot.parser_frames_received;
      break;
    case navbench::DiagnosticEvent::Parser:
      diagnostic.related_sequence =
          (static_cast<uint32_t>(snapshot.hello_result) << 24U) |
          (static_cast<uint32_t>(snapshot.parser_status) << 16U) |
          (snapshot.hello_packets & 0xffffU);
      diagnostic.context = snapshot.parser_errors;
      break;
    case navbench::DiagnosticEvent::Transmit:
      diagnostic.related_sequence =
          (static_cast<uint32_t>(lastWriteRequested) << 16U) |
          static_cast<uint32_t>(lastWriteResult);
      diagnostic.context = serialTxStager.total_bytes_written();
      break;
    case navbench::DiagnosticEvent::Queue:
      diagnostic.related_sequence = snapshot.response_frames_created;
      diagnostic.context =
          (snapshot.response_frames_dropped << 16U) |
          static_cast<uint32_t>(snapshot.response_frames_pending);
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
  uint8_t* frame = serialTxStager.writable_buffer();
  if (frame == nullptr) {
    return false;
  }
  if (navbench::protocol::encodePacket(
          navbench::protocol::MessageType::Error, diagnosticSequence++,
          firmwareSession.last_step_id(), payload, payloadSize, frame,
          serialTxStager.capacity(), &frameSize) !=
      navbench::protocol::Status::Ok) {
    return false;
  }
  return serialTxStager.commit_frame(frameSize);
#else
  (void)nowMs;
  return false;
#endif
}

void transmitSerial(uint32_t nowMs) {
  if (serialTxStager.idle()) {
    size_t frameSize = 0U;
    uint8_t* frame = serialTxStager.writable_buffer();
    if (frame != nullptr &&
        firmwareSession.pop_frame(frame, serialTxStager.capacity(),
                                  &frameSize)) {
      (void)serialTxStager.commit_frame(frameSize);
    } else {
      (void)stageDiagnosticFrame(nowMs);
    }
  }
  if (serialTxStager.idle()) {
    return;
  }

  size_t count = serialTxStager.next_write_size(kMaximumTxBytesPerLoop);
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
  const size_t written = Serial.write(serialTxStager.pending_data(), count);
#if defined(NO_USB)
  // Renesas UART::write() returns on TX_DATA_EMPTY while the final byte is
  // still shifting. Starting the next chunk before TX_COMPLETE can reuse the
  // driver state at a frame boundary. Bound the wait to the final byte so a
  // staged frame remains contiguous on the UNO R4 WiFi UART bridge.
  if (written != 0U) {
    Serial.flush();
  }
#endif
#if defined(NAVBENCH_SERIAL_DIAGNOSTIC)
  lastWriteRequested = static_cast<uint16_t>(count);
  lastWriteResult =
      static_cast<uint16_t>(written <= 0xffffU ? written : 0xffffU);
#endif
  (void)serialTxStager.acknowledge_write(count, written);
}

navbench::HmiState hmiState(navbench::SafetyState state) {
  switch (state) {
    case navbench::SafetyState::Startup:
    case navbench::SafetyState::SelfTest:
      return navbench::HmiState::Boot;
    case navbench::SafetyState::Ready:
      return navbench::HmiState::Ready;
    case navbench::SafetyState::Running:
      return navbench::HmiState::Running;
    case navbench::SafetyState::Degraded:
      return navbench::HmiState::Degraded;
    case navbench::SafetyState::SafeStop:
      return navbench::HmiState::SafeStop;
    case navbench::SafetyState::Fault:
      return navbench::HmiState::Fault;
  }
  return navbench::HmiState::Fault;
}

void updateHmi(uint32_t nowMs) {
  navbench::HmiStatus status{};
  status.state = hmiState(firmwareSession.core().runtime().state());
  status.navigation_mode = static_cast<uint8_t>(
      firmwareSession.core().estimator().navigation_mode(nowMs));
  status.host_connected = firmwareSession.session_active();
  status.estimator_healthy = firmwareSession.core().estimator().healthy();
  const float maximumSteering =
      firmwareSession.core().controller().config().maximum_steering_rad;
  status.steering_normalized = maximumSteering > 0.0F
      ? firmwareSession.last_steering_command_rad() / maximumSteering : 0.0F;
  hmi.update(nowMs, status, hmiHal);
}

}  // namespace

void setup() {
  boardHmi.begin();
  hmiHal = boardHmi.callbacks();
  navbench::HmiConfig hmiConfig = navbench::HmiConfig::defaults();
  hmiConfig.buzzer_enabled = NAVBENCH_HMI_BUZZER_ENABLED != 0;
  hmiConfig.servo_enabled = NAVBENCH_HMI_SERVO_ENABLED != 0;
  (void)hmi.set_config(hmiConfig);
  hmi.begin(millis(), hmiHal);
  Serial.begin(kSerialBaud);
  const uint32_t nowMs = millis();
  firmwareSession.reset(nowMs);
#if defined(NAVBENCH_SERIAL_DIAGNOSTIC)
  diagnosticScheduler.reset(nowMs);
#endif
}

void loop() {
  const uint32_t loopStartUs = micros();
  const uint32_t nowMs = millis();

  receiveSerial(nowMs);
#if !defined(NAVBENCH_SERIAL_DIAGNOSTIC)
  firmwareSession.tick(nowMs, firmwareSession.last_step_id());
#endif
  transmitSerial(nowMs);
  updateHmi(nowMs);

  firmwareSession.record_loop_duration(micros() - loopStartUs);
}
