#ifndef NAVBENCH_PROTOCOL_HPP
#define NAVBENCH_PROTOCOL_HPP

#include <stddef.h>
#include <stdint.h>

namespace navbench {
namespace protocol {

static const uint8_t kProtocolVersion = 1U;
static const size_t kHeaderSize = 12U;
static const size_t kCrcSize = 2U;
static const size_t kMaxPayloadSize = 128U;
static const size_t kMaxRawPacketSize = kHeaderSize + kMaxPayloadSize + kCrcSize;
static const size_t kMaxEncodedFrameSize = kMaxRawPacketSize + 1U;
static const size_t kMaxWireFrameSize = kMaxEncodedFrameSize + 1U;
static const uint8_t kMaxRoutePointsPerChunk = 5U;

enum class MessageType : uint8_t {
  Hello = 0x01,
  HelloAck = 0x02,
  SensorFrame = 0x10,
  RouteChunk = 0x11,
  RouteReference = 0x11,
  ControlCommand = 0x20,
  StateEstimate = 0x21,
  HealthStatus = 0x22,
  Heartbeat = 0x23,
  Error = 0x7e,
  SafeStop = 0x7f,
};

enum class EndpointRole : uint8_t { Host = 1, Controller = 2 };

enum class HelloStatus : uint8_t {
  Ok = 0,
  VersionMismatch = 1,
  Busy = 2,
  Rejected = 3,
};

enum Capability : uint32_t {
  CapabilitySensorFrame = 1UL << 0,
  CapabilityRouteChunk = 1UL << 1,
  CapabilityStateEstimate = 1UL << 2,
  CapabilityHealthStatus = 1UL << 3,
  CapabilitySafeStop = 1UL << 4,
};

enum SensorMask : uint16_t {
  SensorImu = 1U << 0,
  SensorWheelSpeed = 1U << 1,
  SensorGnss = 1U << 2,
  SensorLandmark = 1U << 3,
};

enum RouteFlag : uint8_t {
  RouteClearExisting = 1U << 0,
  RouteFinalChunk = 1U << 1,
  RouteLoop = 1U << 2,
};

enum class ControlMode : uint8_t { Neutral = 0, Tracking = 1, SafeStop = 2 };

enum class RuntimeState : uint8_t {
  Startup = 0,
  SelfTest = 1,
  Ready = 2,
  Running = 3,
  Degraded = 4,
  SafeStop = 5,
  Fault = 6,
};

enum class NavigationMode : uint8_t {
  Unavailable = 0,
  DeadReckoning = 1,
  LandmarkAided = 2,
  GnssAided = 3,
  Degraded = 4,
};

enum class SafeStopReason : uint8_t {
  None = 0,
  Manual = 1,
  HostTimeout = 2,
  StaleSensor = 3,
  ProtocolError = 4,
  InternalFault = 5,
};

enum class ApplicationErrorCode : uint16_t {
  None = 0,
  BadPayload = 1,
  UnsupportedMessage = 2,
  RouteRejected = 3,
  NotReady = 4,
  InternalFault = 5,
};

enum class Status : uint8_t {
  Ok = 0,
  NullArgument,
  BufferTooSmall,
  EmptyFrame,
  UnexpectedDelimiter,
  CobsMalformed,
  PacketTooShort,
  CrcMismatch,
  UnsupportedVersion,
  UnknownMessageType,
  PayloadLengthMismatch,
  PayloadTooLarge,
  InvalidValue,
  OversizedFrame,
  TruncatedFrame,
};

struct Packet {
  MessageType message_type;
  uint32_t sequence;
  uint32_t step_id;
  uint16_t payload_size;
  uint8_t payload[kMaxPayloadSize];
};

struct HelloPayload {
  EndpointRole role;
  uint8_t min_version;
  uint8_t max_version;
  uint32_t capabilities;
  uint16_t max_payload;
  uint16_t heartbeat_timeout_ms;
};

struct HelloAckPayload {
  uint8_t accepted_version;
  HelloStatus status;
  EndpointRole role;
  uint32_t capabilities;
  uint16_t max_payload;
  uint16_t heartbeat_timeout_ms;
};

struct ImuSample {
  uint32_t sample_step_id;
  uint32_t timestamp_us;
  float longitudinal_acceleration_mps2;
  float yaw_rate_rps;
};

struct WheelSpeedSample {
  uint32_t sample_step_id;
  uint32_t timestamp_us;
  float speed_mps;
};

struct GnssSample {
  uint32_t sample_step_id;
  uint32_t timestamp_us;
  float x_m;
  float y_m;
};

struct LandmarkSample {
  uint32_t sample_step_id;
  uint32_t timestamp_us;
  uint16_t landmark_id;
  float landmark_x_m;
  float landmark_y_m;
  float range_m;
  float bearing_rad;
};

struct SensorFramePayload {
  uint16_t present_mask;
  uint16_t fault_mask;
  ImuSample imu;
  WheelSpeedSample wheel_speed;
  GnssSample gnss;
  LandmarkSample landmark;
};

struct RouteWaypoint {
  float x_m;
  float y_m;
  float target_speed_mps;
  float acceptance_radius_m;
};

struct RouteChunkPayload {
  uint16_t route_id;
  uint16_t start_index;
  uint16_t total_count;
  uint8_t point_count;
  uint8_t flags;
  RouteWaypoint points[kMaxRoutePointsPerChunk];
};

struct ControlCommandPayload {
  float steering_rad;
  float acceleration_mps2;
  float target_speed_mps;
  ControlMode mode;
  uint8_t flags;
};

struct StateEstimatePayload {
  float x_m;
  float y_m;
  float heading_rad;
  float speed_mps;
  float yaw_rate_rps;
  float acceleration_bias_mps2;
  float covariance_diagonal[6];
  NavigationMode navigation_mode;
  uint8_t flags;
};

struct HealthStatusPayload {
  RuntimeState runtime_state;
  NavigationMode navigation_mode;
  uint16_t flags;
  uint32_t uptime_ms;
  uint32_t last_sensor_age_ms;
  uint32_t rx_frames;
  uint32_t rx_crc_errors;
  uint32_t rx_decode_errors;
  uint32_t rx_missing;
  uint32_t rx_duplicates;
  uint32_t rx_out_of_order;
  uint32_t rx_stale;
  uint32_t queue_overflows;
  uint32_t scheduler_overruns;
  uint32_t max_loop_us;
  uint32_t imu_yaw_nis_evaluated_count;
  uint32_t imu_yaw_nis_gate_rejected_count;
  float imu_yaw_nis_sum;
  float imu_yaw_nis_max;
  uint32_t wheel_nis_evaluated_count;
  uint32_t wheel_nis_gate_rejected_count;
  float wheel_nis_sum;
  float wheel_nis_max;
  uint32_t gnss_nis_evaluated_count;
  uint32_t gnss_nis_gate_rejected_count;
  float gnss_nis_sum;
  float gnss_nis_max;
  uint32_t landmark_nis_evaluated_count;
  uint32_t landmark_nis_gate_rejected_count;
  float landmark_nis_sum;
  float landmark_nis_max;
};

struct HeartbeatPayload {
  uint32_t uptime_ms;
  uint32_t monotonic_ms;
  RuntimeState runtime_state;
};

struct ErrorPayload {
  ApplicationErrorCode code;
  uint16_t detail;
  uint32_t related_sequence;
  uint32_t context;
};

struct SafeStopPayload {
  SafeStopReason reason;
  bool latch;
  uint16_t detail;
};

bool isKnownMessageType(uint8_t value);
uint16_t crc16Ccitt(const uint8_t *data, size_t size);

Status cobsEncode(const uint8_t *data, size_t size, uint8_t *output,
                  size_t output_capacity, size_t *output_size);
Status cobsDecode(const uint8_t *data, size_t size, uint8_t *output,
                  size_t output_capacity, size_t *output_size);

Status encodePacket(MessageType message_type, uint32_t sequence,
                    uint32_t step_id, const uint8_t *payload,
                    uint16_t payload_size, uint8_t *output,
                    size_t output_capacity, size_t *output_size);
Status decodePacket(const uint8_t *frame, size_t frame_size, Packet *packet);

Status encodePayload(const HelloPayload &value, uint8_t *output,
                     size_t capacity, uint16_t *size);
Status decodePayload(const uint8_t *data, uint16_t size, HelloPayload *value);
Status encodePayload(const HelloAckPayload &value, uint8_t *output,
                     size_t capacity, uint16_t *size);
Status decodePayload(const uint8_t *data, uint16_t size,
                     HelloAckPayload *value);
Status encodePayload(const SensorFramePayload &value, uint8_t *output,
                     size_t capacity, uint16_t *size);
Status decodePayload(const uint8_t *data, uint16_t size,
                     SensorFramePayload *value);
Status encodePayload(const RouteChunkPayload &value, uint8_t *output,
                     size_t capacity, uint16_t *size);
Status decodePayload(const uint8_t *data, uint16_t size,
                     RouteChunkPayload *value);
Status encodePayload(const ControlCommandPayload &value, uint8_t *output,
                     size_t capacity, uint16_t *size);
Status decodePayload(const uint8_t *data, uint16_t size,
                     ControlCommandPayload *value);
Status encodePayload(const StateEstimatePayload &value, uint8_t *output,
                     size_t capacity, uint16_t *size);
Status decodePayload(const uint8_t *data, uint16_t size,
                     StateEstimatePayload *value);
Status encodePayload(const HealthStatusPayload &value, uint8_t *output,
                     size_t capacity, uint16_t *size);
Status decodePayload(const uint8_t *data, uint16_t size,
                     HealthStatusPayload *value);
Status encodePayload(const HeartbeatPayload &value, uint8_t *output,
                     size_t capacity, uint16_t *size);
Status decodePayload(const uint8_t *data, uint16_t size,
                     HeartbeatPayload *value);
Status encodePayload(const ErrorPayload &value, uint8_t *output,
                     size_t capacity, uint16_t *size);
Status decodePayload(const uint8_t *data, uint16_t size, ErrorPayload *value);
Status encodePayload(const SafeStopPayload &value, uint8_t *output,
                     size_t capacity, uint16_t *size);
Status decodePayload(const uint8_t *data, uint16_t size,
                     SafeStopPayload *value);

struct ParserStats {
  uint32_t bytes_received;
  uint32_t frames_received;
  uint32_t packets_accepted;
  uint32_t empty_delimiters;
  uint32_t oversized_frames;
  uint32_t truncated_frames;
  uint32_t cobs_errors;
  uint32_t crc_errors;
  uint32_t version_errors;
  uint32_t type_errors;
  uint32_t length_errors;
  uint32_t other_errors;
};

typedef void (*PacketCallback)(const Packet &packet, void *context);
typedef void (*ParserErrorCallback)(Status status, void *context);

class StreamParser {
 public:
  StreamParser();
  void reset(bool clear_stats = false);
  void feed(const uint8_t *data, size_t size, PacketCallback packet_callback,
            ParserErrorCallback error_callback, void *context);
  Status finish(ParserErrorCallback error_callback = 0, void *context = 0);
  const ParserStats &stats() const { return stats_; }

 private:
  void countError(Status status);
  uint8_t encoded_[kMaxEncodedFrameSize];
  size_t encoded_size_;
  bool discard_until_delimiter_;
  ParserStats stats_;
};

enum class SequenceDisposition : uint8_t {
  First = 0,
  InOrder = 1,
  Gap = 2,
  Duplicate = 3,
  OutOfOrder = 4,
  Stale = 5,
};

struct SequenceResult {
  SequenceDisposition disposition;
  bool accepted;
  uint32_t missing;
};

struct SequenceStats {
  uint32_t accepted;
  uint32_t missing;
  uint32_t duplicates;
  uint32_t out_of_order;
  uint32_t stale;
};

class SequenceTracker {
 public:
  explicit SequenceTracker(uint32_t reorder_window = 32U);
  void reset(bool clear_stats = false);
  SequenceResult observe(uint32_t sequence);
  SequenceResult observe(uint32_t sequence, uint32_t step_id);
  const SequenceStats &stats() const { return stats_; }

 private:
  SequenceResult observeInternal(uint32_t sequence, bool has_step,
                                 uint32_t step_id);
  uint32_t reorder_window_;
  uint32_t last_sequence_;
  uint32_t last_step_id_;
  bool initialized_;
  bool has_last_step_;
  SequenceStats stats_;
};

}  // namespace protocol
}  // namespace navbench

#endif  // NAVBENCH_PROTOCOL_HPP
