#include "navbench/protocol.hpp"

#include <math.h>
#include <string.h>

namespace navbench {
namespace protocol {
namespace {

static const uint16_t kKnownSensorMask =
    SensorImu | SensorWheelSpeed | SensorGnss | SensorLandmark;
static const uint8_t kKnownRouteFlags =
    RouteClearExisting | RouteFinalChunk | RouteLoop;

static const uint16_t kHelloPayloadSize = 12U;
static const uint16_t kHelloAckPayloadSize = 12U;
static const uint16_t kSensorFramePayloadSize = 76U;
static const uint16_t kRoutePrefixSize = 8U;
static const uint16_t kRoutePointSize = 16U;
static const uint16_t kControlCommandPayloadSize = 16U;
static const uint16_t kStateEstimatePayloadSize = 52U;
static const uint16_t kHealthStatusPayloadSize = 116U;
static const uint16_t kHeartbeatPayloadSize = 12U;
static const uint16_t kErrorPayloadSize = 12U;
static const uint16_t kSafeStopPayloadSize = 4U;

uint32_t saturatingAdd(uint32_t value, uint32_t increment) {
  const uint32_t maximum = 0xffffffffUL;
  return increment > maximum - value ? maximum : value + increment;
}

uint32_t saturatingAddSize(uint32_t value, size_t increment) {
  if (increment > static_cast<size_t>(0xffffffffUL)) {
    return 0xffffffffUL;
  }
  return saturatingAdd(value, static_cast<uint32_t>(increment));
}

class Writer {
 public:
  Writer(uint8_t *data, size_t capacity)
      : data_(data), capacity_(capacity), position_(0U), status_(Status::Ok) {
    if (data_ == 0) {
      status_ = Status::NullArgument;
    }
  }

  void putU8(uint8_t value) {
    if (!reserve(1U)) {
      return;
    }
    data_[position_++] = value;
  }

  void putU16(uint16_t value) {
    if (!reserve(2U)) {
      return;
    }
    data_[position_++] = static_cast<uint8_t>(value & 0xffU);
    data_[position_++] = static_cast<uint8_t>((value >> 8U) & 0xffU);
  }

  void putU32(uint32_t value) {
    if (!reserve(4U)) {
      return;
    }
    data_[position_++] = static_cast<uint8_t>(value & 0xffU);
    data_[position_++] = static_cast<uint8_t>((value >> 8U) & 0xffU);
    data_[position_++] = static_cast<uint8_t>((value >> 16U) & 0xffU);
    data_[position_++] = static_cast<uint8_t>((value >> 24U) & 0xffU);
  }

  void putF32(float value) {
    if (!isfinite(value)) {
      setError(Status::InvalidValue);
      return;
    }
    uint32_t bits = 0U;
    static_assert(sizeof(bits) == sizeof(value), "Protocol v1 requires binary32");
    memcpy(&bits, &value, sizeof(bits));
    putU32(bits);
  }

  void putBytes(const uint8_t *value, size_t size) {
    if (size != 0U && value == 0) {
      setError(Status::NullArgument);
      return;
    }
    if (!reserve(size)) {
      return;
    }
    if (size != 0U) {
      memcpy(data_ + position_, value, size);
      position_ += size;
    }
  }

  Status status() const { return status_; }
  size_t size() const { return position_; }

 private:
  bool reserve(size_t size) {
    if (status_ != Status::Ok) {
      return false;
    }
    if (size > capacity_ - position_) {
      status_ = Status::BufferTooSmall;
      return false;
    }
    return true;
  }

  void setError(Status status) {
    if (status_ == Status::Ok) {
      status_ = status;
    }
  }

  uint8_t *data_;
  size_t capacity_;
  size_t position_;
  Status status_;
};

class Reader {
 public:
  Reader(const uint8_t *data, size_t size)
      : data_(data), size_(size), position_(0U), status_(Status::Ok) {
    if (size_ != 0U && data_ == 0) {
      status_ = Status::NullArgument;
    }
  }

  uint8_t getU8() {
    if (!reserve(1U)) {
      return 0U;
    }
    return data_[position_++];
  }

  uint16_t getU16() {
    if (!reserve(2U)) {
      return 0U;
    }
    const uint16_t value = static_cast<uint16_t>(data_[position_]) |
                           (static_cast<uint16_t>(data_[position_ + 1U]) << 8U);
    position_ += 2U;
    return value;
  }

  uint32_t getU32() {
    if (!reserve(4U)) {
      return 0U;
    }
    const uint32_t value = static_cast<uint32_t>(data_[position_]) |
                           (static_cast<uint32_t>(data_[position_ + 1U]) << 8U) |
                           (static_cast<uint32_t>(data_[position_ + 2U]) << 16U) |
                           (static_cast<uint32_t>(data_[position_ + 3U]) << 24U);
    position_ += 4U;
    return value;
  }

  float getF32() {
    const uint32_t bits = getU32();
    float value = 0.0F;
    static_assert(sizeof(bits) == sizeof(value), "Protocol v1 requires binary32");
    memcpy(&value, &bits, sizeof(value));
    if (status_ == Status::Ok && !isfinite(value)) {
      status_ = Status::InvalidValue;
    }
    return value;
  }

  Status status() const { return status_; }
  size_t remaining() const { return size_ - position_; }

 private:
  bool reserve(size_t size) {
    if (status_ != Status::Ok) {
      return false;
    }
    if (size > size_ - position_) {
      status_ = Status::PayloadLengthMismatch;
      return false;
    }
    return true;
  }

  const uint8_t *data_;
  size_t size_;
  size_t position_;
  Status status_;
};

Status finishWrite(const Writer &writer, uint16_t *size) {
  if (size == 0) {
    return Status::NullArgument;
  }
  if (writer.status() != Status::Ok) {
    *size = 0U;
    return writer.status();
  }
  *size = static_cast<uint16_t>(writer.size());
  return Status::Ok;
}

Status finishRead(const Reader &reader) {
  if (reader.status() != Status::Ok) {
    return reader.status();
  }
  return reader.remaining() == 0U ? Status::Ok : Status::PayloadLengthMismatch;
}

bool validEndpointRole(EndpointRole value) {
  return value == EndpointRole::Host || value == EndpointRole::Controller;
}

bool validHelloStatus(HelloStatus value) {
  return value == HelloStatus::Ok || value == HelloStatus::VersionMismatch ||
         value == HelloStatus::Busy || value == HelloStatus::Rejected;
}

bool validHelloPayload(const HelloPayload &value) {
  return validEndpointRole(value.role) && value.min_version != 0U &&
         value.min_version <= value.max_version && value.max_payload != 0U &&
         value.max_payload <= kMaxPayloadSize &&
         value.heartbeat_timeout_ms != 0U;
}

bool validHelloAckPayload(const HelloAckPayload &value) {
  return validHelloStatus(value.status) && validEndpointRole(value.role) &&
         value.accepted_version ==
             (value.status == HelloStatus::Ok ? kProtocolVersion : 0U) &&
         value.max_payload != 0U &&
         value.max_payload <= kMaxPayloadSize &&
         value.heartbeat_timeout_ms != 0U;
}

bool validControlMode(ControlMode value) {
  return value == ControlMode::Neutral || value == ControlMode::Tracking ||
         value == ControlMode::SafeStop;
}

bool validRuntimeState(RuntimeState value) {
  return static_cast<uint8_t>(value) <= static_cast<uint8_t>(RuntimeState::Fault);
}

bool validNavigationMode(NavigationMode value) {
  return static_cast<uint8_t>(value) <=
         static_cast<uint8_t>(NavigationMode::Degraded);
}

bool validSafeStopReason(SafeStopReason value) {
  return static_cast<uint8_t>(value) <=
         static_cast<uint8_t>(SafeStopReason::InternalFault);
}

bool validApplicationError(ApplicationErrorCode value) {
  return static_cast<uint16_t>(value) <=
         static_cast<uint16_t>(ApplicationErrorCode::InternalFault);
}

bool validNisSummary(uint32_t evaluated_count,
                     uint32_t gate_rejected_count, float nis_sum,
                     float nis_maximum) {
  if (gate_rejected_count > evaluated_count || !isfinite(nis_sum) ||
      !isfinite(nis_maximum) || nis_sum < 0.0F || nis_maximum < 0.0F) {
    return false;
  }
  if (evaluated_count == 0U) {
    return gate_rejected_count == 0U && nis_sum == 0.0F &&
           nis_maximum == 0.0F;
  }
  return nis_maximum <= nis_sum;
}

Status requireExactSize(const uint8_t *data, uint16_t size, uint16_t expected) {
  if (data == 0) {
    return Status::NullArgument;
  }
  return size == expected ? Status::Ok : Status::PayloadLengthMismatch;
}

}  // namespace

bool isKnownMessageType(uint8_t value) {
  switch (static_cast<MessageType>(value)) {
    case MessageType::Hello:
    case MessageType::HelloAck:
    case MessageType::SensorFrame:
    case MessageType::RouteChunk:
    case MessageType::ControlCommand:
    case MessageType::StateEstimate:
    case MessageType::HealthStatus:
    case MessageType::Heartbeat:
    case MessageType::Error:
    case MessageType::SafeStop:
      return true;
  }
  return false;
}

uint16_t crc16Ccitt(const uint8_t *data, size_t size) {
  if (size != 0U && data == 0) {
    return 0U;
  }
  uint16_t crc = 0xffffU;
  for (size_t index = 0U; index < size; ++index) {
    crc ^= static_cast<uint16_t>(data[index]) << 8U;
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
      crc = (crc & 0x8000U) != 0U
                ? static_cast<uint16_t>((crc << 1U) ^ 0x1021U)
                : static_cast<uint16_t>(crc << 1U);
    }
  }
  return crc;
}

Status cobsEncode(const uint8_t *data, size_t size, uint8_t *output,
                  size_t output_capacity, size_t *output_size) {
  if ((size != 0U && data == 0) || output == 0 || output_size == 0) {
    return Status::NullArgument;
  }
  *output_size = 0U;
  if (output_capacity == 0U) {
    return Status::BufferTooSmall;
  }

  size_t read_index = 0U;
  size_t write_index = 1U;
  size_t code_index = 0U;
  uint8_t code = 1U;
  while (read_index < size) {
    const uint8_t value = data[read_index++];
    if (value == 0U) {
      if (code_index >= output_capacity || write_index >= output_capacity) {
        return Status::BufferTooSmall;
      }
      output[code_index] = code;
      code_index = write_index++;
      code = 1U;
      continue;
    }
    if (write_index >= output_capacity) {
      return Status::BufferTooSmall;
    }
    output[write_index++] = value;
    ++code;
    if (code == 0xffU) {
      if (code_index >= output_capacity || write_index >= output_capacity) {
        return Status::BufferTooSmall;
      }
      output[code_index] = code;
      code_index = write_index++;
      code = 1U;
    }
  }
  if (code_index >= output_capacity) {
    return Status::BufferTooSmall;
  }
  output[code_index] = code;
  *output_size = write_index;
  return Status::Ok;
}

Status cobsDecode(const uint8_t *data, size_t size, uint8_t *output,
                  size_t output_capacity, size_t *output_size) {
  if (data == 0 || output == 0 || output_size == 0) {
    return Status::NullArgument;
  }
  *output_size = 0U;
  if (size == 0U) {
    return Status::CobsMalformed;
  }

  size_t read_index = 0U;
  size_t write_index = 0U;
  while (read_index < size) {
    const uint8_t code = data[read_index++];
    if (code == 0U) {
      return Status::CobsMalformed;
    }
    const size_t block_size = static_cast<size_t>(code - 1U);
    if (block_size > size - read_index) {
      return Status::CobsMalformed;
    }
    if (block_size > output_capacity - write_index) {
      return Status::BufferTooSmall;
    }
    if (block_size != 0U) {
      memcpy(output + write_index, data + read_index, block_size);
      write_index += block_size;
      read_index += block_size;
    }
    if (code != 0xffU && read_index < size) {
      if (write_index >= output_capacity) {
        return Status::BufferTooSmall;
      }
      output[write_index++] = 0U;
    }
  }
  *output_size = write_index;
  return Status::Ok;
}

Status encodePacket(MessageType message_type, uint32_t sequence,
                    uint32_t step_id, const uint8_t *payload,
                    uint16_t payload_size, uint8_t *output,
                    size_t output_capacity, size_t *output_size) {
  if (output == 0 || output_size == 0 ||
      (payload_size != 0U && payload == 0)) {
    return Status::NullArgument;
  }
  *output_size = 0U;
  if (!isKnownMessageType(static_cast<uint8_t>(message_type))) {
    return Status::UnknownMessageType;
  }
  if (payload_size > kMaxPayloadSize) {
    return Status::PayloadTooLarge;
  }
  if (output_capacity < 2U) {
    return Status::BufferTooSmall;
  }

  uint8_t raw[kMaxRawPacketSize];
  Writer writer(raw, sizeof(raw));
  writer.putU8(kProtocolVersion);
  writer.putU8(static_cast<uint8_t>(message_type));
  writer.putU16(payload_size);
  writer.putU32(sequence);
  writer.putU32(step_id);
  writer.putBytes(payload, payload_size);
  if (writer.status() != Status::Ok) {
    return writer.status();
  }
  const uint16_t checksum = crc16Ccitt(raw, writer.size());
  writer.putU16(checksum);
  if (writer.status() != Status::Ok) {
    return writer.status();
  }

  size_t encoded_size = 0U;
  const Status status = cobsEncode(raw, writer.size(), output,
                                   output_capacity - 1U, &encoded_size);
  if (status != Status::Ok) {
    return status;
  }
  output[encoded_size] = 0U;
  *output_size = encoded_size + 1U;
  return Status::Ok;
}

Status decodePacket(const uint8_t *frame, size_t frame_size, Packet *packet) {
  if (frame == 0 || packet == 0) {
    return Status::NullArgument;
  }
  if (frame_size != 0U && frame[frame_size - 1U] == 0U) {
    --frame_size;
  }
  if (frame_size == 0U) {
    return Status::EmptyFrame;
  }
  if (frame_size > kMaxEncodedFrameSize) {
    return Status::OversizedFrame;
  }
  for (size_t index = 0U; index < frame_size; ++index) {
    if (frame[index] == 0U) {
      return Status::UnexpectedDelimiter;
    }
  }

  uint8_t raw[kMaxRawPacketSize];
  size_t raw_size = 0U;
  Status status = cobsDecode(frame, frame_size, raw, sizeof(raw), &raw_size);
  if (status != Status::Ok) {
    return status;
  }
  if (raw_size < kHeaderSize + kCrcSize) {
    return Status::PacketTooShort;
  }
  if (raw_size > kMaxRawPacketSize) {
    return Status::PayloadTooLarge;
  }
  const uint16_t received_crc =
      static_cast<uint16_t>(raw[raw_size - 2U]) |
      (static_cast<uint16_t>(raw[raw_size - 1U]) << 8U);
  const uint16_t calculated_crc = crc16Ccitt(raw, raw_size - kCrcSize);
  if (received_crc != calculated_crc) {
    return Status::CrcMismatch;
  }

  Reader reader(raw, raw_size - kCrcSize);
  const uint8_t version = reader.getU8();
  const uint8_t type_value = reader.getU8();
  const uint16_t payload_size = reader.getU16();
  const uint32_t sequence = reader.getU32();
  const uint32_t step_id = reader.getU32();
  if (reader.status() != Status::Ok) {
    return reader.status();
  }
  if (version != kProtocolVersion) {
    return Status::UnsupportedVersion;
  }
  if (!isKnownMessageType(type_value)) {
    return Status::UnknownMessageType;
  }
  if (payload_size > kMaxPayloadSize) {
    return Status::PayloadTooLarge;
  }
  if (reader.remaining() != payload_size) {
    return Status::PayloadLengthMismatch;
  }

  packet->message_type = static_cast<MessageType>(type_value);
  packet->sequence = sequence;
  packet->step_id = step_id;
  packet->payload_size = payload_size;
  if (payload_size != 0U) {
    memcpy(packet->payload, raw + kHeaderSize, payload_size);
  }
  return Status::Ok;
}

Status encodePayload(const HelloPayload &value, uint8_t *output,
                     size_t capacity, uint16_t *size) {
  if (!validHelloPayload(value)) {
    return Status::InvalidValue;
  }
  Writer writer(output, capacity);
  writer.putU8(static_cast<uint8_t>(value.role));
  writer.putU8(value.min_version);
  writer.putU8(value.max_version);
  writer.putU8(0U);
  writer.putU32(value.capabilities);
  writer.putU16(value.max_payload);
  writer.putU16(value.heartbeat_timeout_ms);
  return finishWrite(writer, size);
}

Status decodePayload(const uint8_t *data, uint16_t size, HelloPayload *value) {
  if (value == 0) {
    return Status::NullArgument;
  }
  Status status = requireExactSize(data, size, kHelloPayloadSize);
  if (status != Status::Ok) {
    return status;
  }
  Reader reader(data, size);
  value->role = static_cast<EndpointRole>(reader.getU8());
  value->min_version = reader.getU8();
  value->max_version = reader.getU8();
  const uint8_t reserved = reader.getU8();
  value->capabilities = reader.getU32();
  value->max_payload = reader.getU16();
  value->heartbeat_timeout_ms = reader.getU16();
  status = finishRead(reader);
  if (status != Status::Ok) {
    return status;
  }
  return reserved == 0U && validHelloPayload(*value) ? Status::Ok
                                                      : Status::InvalidValue;
}

Status encodePayload(const HelloAckPayload &value, uint8_t *output,
                     size_t capacity, uint16_t *size) {
  if (!validHelloAckPayload(value)) {
    return Status::InvalidValue;
  }
  Writer writer(output, capacity);
  writer.putU8(value.accepted_version);
  writer.putU8(static_cast<uint8_t>(value.status));
  writer.putU8(static_cast<uint8_t>(value.role));
  writer.putU8(0U);
  writer.putU32(value.capabilities);
  writer.putU16(value.max_payload);
  writer.putU16(value.heartbeat_timeout_ms);
  return finishWrite(writer, size);
}

Status decodePayload(const uint8_t *data, uint16_t size,
                     HelloAckPayload *value) {
  if (value == 0) {
    return Status::NullArgument;
  }
  Status status = requireExactSize(data, size, kHelloAckPayloadSize);
  if (status != Status::Ok) {
    return status;
  }
  Reader reader(data, size);
  value->accepted_version = reader.getU8();
  value->status = static_cast<HelloStatus>(reader.getU8());
  value->role = static_cast<EndpointRole>(reader.getU8());
  const uint8_t reserved = reader.getU8();
  value->capabilities = reader.getU32();
  value->max_payload = reader.getU16();
  value->heartbeat_timeout_ms = reader.getU16();
  status = finishRead(reader);
  if (status != Status::Ok) {
    return status;
  }
  return reserved == 0U && validHelloAckPayload(*value)
             ? Status::Ok
             : Status::InvalidValue;
}

Status encodePayload(const SensorFramePayload &value, uint8_t *output,
                     size_t capacity, uint16_t *size) {
  if ((value.present_mask & ~kKnownSensorMask) != 0U ||
      (value.fault_mask & ~kKnownSensorMask) != 0U) {
    return Status::InvalidValue;
  }
  Writer writer(output, capacity);
  writer.putU16(value.present_mask);
  writer.putU16(value.fault_mask);
  writer.putU32(value.imu.sample_step_id);
  writer.putU32(value.imu.timestamp_us);
  writer.putF32(value.imu.longitudinal_acceleration_mps2);
  writer.putF32(value.imu.yaw_rate_rps);
  writer.putU32(value.wheel_speed.sample_step_id);
  writer.putU32(value.wheel_speed.timestamp_us);
  writer.putF32(value.wheel_speed.speed_mps);
  writer.putU32(value.gnss.sample_step_id);
  writer.putU32(value.gnss.timestamp_us);
  writer.putF32(value.gnss.x_m);
  writer.putF32(value.gnss.y_m);
  writer.putU32(value.landmark.sample_step_id);
  writer.putU32(value.landmark.timestamp_us);
  writer.putU16(value.landmark.landmark_id);
  writer.putU16(0U);
  writer.putF32(value.landmark.landmark_x_m);
  writer.putF32(value.landmark.landmark_y_m);
  writer.putF32(value.landmark.range_m);
  writer.putF32(value.landmark.bearing_rad);
  return finishWrite(writer, size);
}

Status decodePayload(const uint8_t *data, uint16_t size,
                     SensorFramePayload *value) {
  if (value == 0) {
    return Status::NullArgument;
  }
  Status status = requireExactSize(data, size, kSensorFramePayloadSize);
  if (status != Status::Ok) {
    return status;
  }
  Reader reader(data, size);
  value->present_mask = reader.getU16();
  value->fault_mask = reader.getU16();
  value->imu.sample_step_id = reader.getU32();
  value->imu.timestamp_us = reader.getU32();
  value->imu.longitudinal_acceleration_mps2 = reader.getF32();
  value->imu.yaw_rate_rps = reader.getF32();
  value->wheel_speed.sample_step_id = reader.getU32();
  value->wheel_speed.timestamp_us = reader.getU32();
  value->wheel_speed.speed_mps = reader.getF32();
  value->gnss.sample_step_id = reader.getU32();
  value->gnss.timestamp_us = reader.getU32();
  value->gnss.x_m = reader.getF32();
  value->gnss.y_m = reader.getF32();
  value->landmark.sample_step_id = reader.getU32();
  value->landmark.timestamp_us = reader.getU32();
  value->landmark.landmark_id = reader.getU16();
  const uint16_t reserved = reader.getU16();
  value->landmark.landmark_x_m = reader.getF32();
  value->landmark.landmark_y_m = reader.getF32();
  value->landmark.range_m = reader.getF32();
  value->landmark.bearing_rad = reader.getF32();
  status = finishRead(reader);
  if (status != Status::Ok) {
    return status;
  }
  return reserved == 0U && (value->present_mask & ~kKnownSensorMask) == 0U &&
                 (value->fault_mask & ~kKnownSensorMask) == 0U
             ? Status::Ok
             : Status::InvalidValue;
}

Status encodePayload(const RouteChunkPayload &value, uint8_t *output,
                     size_t capacity, uint16_t *size) {
  if (value.point_count > kMaxRoutePointsPerChunk ||
      (value.flags & ~kKnownRouteFlags) != 0U ||
      (value.point_count != 0U &&
       (value.total_count == 0U ||
        static_cast<uint32_t>(value.start_index) + value.point_count >
            value.total_count)) ||
      (value.point_count == 0U &&
       (value.flags & RouteClearExisting) == 0U)) {
    return Status::InvalidValue;
  }
  Writer writer(output, capacity);
  writer.putU16(value.route_id);
  writer.putU16(value.start_index);
  writer.putU16(value.total_count);
  writer.putU8(value.point_count);
  writer.putU8(value.flags);
  for (uint8_t index = 0U; index < value.point_count; ++index) {
    writer.putF32(value.points[index].x_m);
    writer.putF32(value.points[index].y_m);
    writer.putF32(value.points[index].target_speed_mps);
    writer.putF32(value.points[index].acceptance_radius_m);
  }
  return finishWrite(writer, size);
}

Status decodePayload(const uint8_t *data, uint16_t size,
                     RouteChunkPayload *value) {
  if (value == 0 || data == 0) {
    return Status::NullArgument;
  }
  if (size < kRoutePrefixSize) {
    return Status::PayloadLengthMismatch;
  }
  Reader reader(data, size);
  value->route_id = reader.getU16();
  value->start_index = reader.getU16();
  value->total_count = reader.getU16();
  value->point_count = reader.getU8();
  value->flags = reader.getU8();
  if (value->point_count > kMaxRoutePointsPerChunk ||
      size != kRoutePrefixSize + value->point_count * kRoutePointSize) {
    return Status::PayloadLengthMismatch;
  }
  for (uint8_t index = 0U; index < value->point_count; ++index) {
    value->points[index].x_m = reader.getF32();
    value->points[index].y_m = reader.getF32();
    value->points[index].target_speed_mps = reader.getF32();
    value->points[index].acceptance_radius_m = reader.getF32();
  }
  Status status = finishRead(reader);
  if (status != Status::Ok) {
    return status;
  }
  if ((value->flags & ~kKnownRouteFlags) != 0U ||
      (value->point_count != 0U &&
       (value->total_count == 0U ||
        static_cast<uint32_t>(value->start_index) + value->point_count >
            value->total_count)) ||
      (value->point_count == 0U &&
       (value->flags & RouteClearExisting) == 0U)) {
    return Status::InvalidValue;
  }
  return Status::Ok;
}

Status encodePayload(const ControlCommandPayload &value, uint8_t *output,
                     size_t capacity, uint16_t *size) {
  if (!validControlMode(value.mode)) {
    return Status::InvalidValue;
  }
  Writer writer(output, capacity);
  writer.putF32(value.steering_rad);
  writer.putF32(value.acceleration_mps2);
  writer.putF32(value.target_speed_mps);
  writer.putU8(static_cast<uint8_t>(value.mode));
  writer.putU8(value.flags);
  writer.putU16(0U);
  return finishWrite(writer, size);
}

Status decodePayload(const uint8_t *data, uint16_t size,
                     ControlCommandPayload *value) {
  if (value == 0) {
    return Status::NullArgument;
  }
  Status status = requireExactSize(data, size, kControlCommandPayloadSize);
  if (status != Status::Ok) {
    return status;
  }
  Reader reader(data, size);
  value->steering_rad = reader.getF32();
  value->acceleration_mps2 = reader.getF32();
  value->target_speed_mps = reader.getF32();
  value->mode = static_cast<ControlMode>(reader.getU8());
  value->flags = reader.getU8();
  const uint16_t reserved = reader.getU16();
  status = finishRead(reader);
  if (status != Status::Ok) {
    return status;
  }
  return reserved == 0U && validControlMode(value->mode) ? Status::Ok
                                                         : Status::InvalidValue;
}

Status encodePayload(const StateEstimatePayload &value, uint8_t *output,
                     size_t capacity, uint16_t *size) {
  if (!validNavigationMode(value.navigation_mode)) {
    return Status::InvalidValue;
  }
  Writer writer(output, capacity);
  writer.putF32(value.x_m);
  writer.putF32(value.y_m);
  writer.putF32(value.heading_rad);
  writer.putF32(value.speed_mps);
  writer.putF32(value.yaw_rate_rps);
  writer.putF32(value.acceleration_bias_mps2);
  for (uint8_t index = 0U; index < 6U; ++index) {
    if (value.covariance_diagonal[index] < 0.0F) {
      return Status::InvalidValue;
    }
    writer.putF32(value.covariance_diagonal[index]);
  }
  writer.putU8(static_cast<uint8_t>(value.navigation_mode));
  writer.putU8(value.flags);
  writer.putU16(0U);
  return finishWrite(writer, size);
}

Status decodePayload(const uint8_t *data, uint16_t size,
                     StateEstimatePayload *value) {
  if (value == 0) {
    return Status::NullArgument;
  }
  Status status = requireExactSize(data, size, kStateEstimatePayloadSize);
  if (status != Status::Ok) {
    return status;
  }
  Reader reader(data, size);
  value->x_m = reader.getF32();
  value->y_m = reader.getF32();
  value->heading_rad = reader.getF32();
  value->speed_mps = reader.getF32();
  value->yaw_rate_rps = reader.getF32();
  value->acceleration_bias_mps2 = reader.getF32();
  for (uint8_t index = 0U; index < 6U; ++index) {
    value->covariance_diagonal[index] = reader.getF32();
  }
  value->navigation_mode = static_cast<NavigationMode>(reader.getU8());
  value->flags = reader.getU8();
  const uint16_t reserved = reader.getU16();
  status = finishRead(reader);
  if (status != Status::Ok) {
    return status;
  }
  if (reserved != 0U || !validNavigationMode(value->navigation_mode)) {
    return Status::InvalidValue;
  }
  for (uint8_t index = 0U; index < 6U; ++index) {
    if (value->covariance_diagonal[index] < 0.0F) {
      return Status::InvalidValue;
    }
  }
  return Status::Ok;
}

Status encodePayload(const HealthStatusPayload &value, uint8_t *output,
                     size_t capacity, uint16_t *size) {
  if (!validRuntimeState(value.runtime_state) ||
      !validNavigationMode(value.navigation_mode) ||
      !validNisSummary(value.imu_yaw_nis_evaluated_count,
                       value.imu_yaw_nis_gate_rejected_count,
                       value.imu_yaw_nis_sum, value.imu_yaw_nis_max) ||
      !validNisSummary(value.wheel_nis_evaluated_count,
                       value.wheel_nis_gate_rejected_count,
                       value.wheel_nis_sum, value.wheel_nis_max) ||
      !validNisSummary(value.gnss_nis_evaluated_count,
                       value.gnss_nis_gate_rejected_count,
                       value.gnss_nis_sum, value.gnss_nis_max) ||
      !validNisSummary(value.landmark_nis_evaluated_count,
                       value.landmark_nis_gate_rejected_count,
                       value.landmark_nis_sum, value.landmark_nis_max)) {
    return Status::InvalidValue;
  }
  Writer writer(output, capacity);
  writer.putU8(static_cast<uint8_t>(value.runtime_state));
  writer.putU8(static_cast<uint8_t>(value.navigation_mode));
  writer.putU16(value.flags);
  writer.putU32(value.uptime_ms);
  writer.putU32(value.last_sensor_age_ms);
  writer.putU32(value.rx_frames);
  writer.putU32(value.rx_crc_errors);
  writer.putU32(value.rx_decode_errors);
  writer.putU32(value.rx_missing);
  writer.putU32(value.rx_duplicates);
  writer.putU32(value.rx_out_of_order);
  writer.putU32(value.rx_stale);
  writer.putU32(value.queue_overflows);
  writer.putU32(value.scheduler_overruns);
  writer.putU32(value.max_loop_us);
  writer.putU32(value.imu_yaw_nis_evaluated_count);
  writer.putU32(value.imu_yaw_nis_gate_rejected_count);
  writer.putF32(value.imu_yaw_nis_sum);
  writer.putF32(value.imu_yaw_nis_max);
  writer.putU32(value.wheel_nis_evaluated_count);
  writer.putU32(value.wheel_nis_gate_rejected_count);
  writer.putF32(value.wheel_nis_sum);
  writer.putF32(value.wheel_nis_max);
  writer.putU32(value.gnss_nis_evaluated_count);
  writer.putU32(value.gnss_nis_gate_rejected_count);
  writer.putF32(value.gnss_nis_sum);
  writer.putF32(value.gnss_nis_max);
  writer.putU32(value.landmark_nis_evaluated_count);
  writer.putU32(value.landmark_nis_gate_rejected_count);
  writer.putF32(value.landmark_nis_sum);
  writer.putF32(value.landmark_nis_max);
  return finishWrite(writer, size);
}

Status decodePayload(const uint8_t *data, uint16_t size,
                     HealthStatusPayload *value) {
  if (value == 0) {
    return Status::NullArgument;
  }
  Status status = requireExactSize(data, size, kHealthStatusPayloadSize);
  if (status != Status::Ok) {
    return status;
  }
  Reader reader(data, size);
  value->runtime_state = static_cast<RuntimeState>(reader.getU8());
  value->navigation_mode = static_cast<NavigationMode>(reader.getU8());
  value->flags = reader.getU16();
  value->uptime_ms = reader.getU32();
  value->last_sensor_age_ms = reader.getU32();
  value->rx_frames = reader.getU32();
  value->rx_crc_errors = reader.getU32();
  value->rx_decode_errors = reader.getU32();
  value->rx_missing = reader.getU32();
  value->rx_duplicates = reader.getU32();
  value->rx_out_of_order = reader.getU32();
  value->rx_stale = reader.getU32();
  value->queue_overflows = reader.getU32();
  value->scheduler_overruns = reader.getU32();
  value->max_loop_us = reader.getU32();
  value->imu_yaw_nis_evaluated_count = reader.getU32();
  value->imu_yaw_nis_gate_rejected_count = reader.getU32();
  value->imu_yaw_nis_sum = reader.getF32();
  value->imu_yaw_nis_max = reader.getF32();
  value->wheel_nis_evaluated_count = reader.getU32();
  value->wheel_nis_gate_rejected_count = reader.getU32();
  value->wheel_nis_sum = reader.getF32();
  value->wheel_nis_max = reader.getF32();
  value->gnss_nis_evaluated_count = reader.getU32();
  value->gnss_nis_gate_rejected_count = reader.getU32();
  value->gnss_nis_sum = reader.getF32();
  value->gnss_nis_max = reader.getF32();
  value->landmark_nis_evaluated_count = reader.getU32();
  value->landmark_nis_gate_rejected_count = reader.getU32();
  value->landmark_nis_sum = reader.getF32();
  value->landmark_nis_max = reader.getF32();
  status = finishRead(reader);
  if (status != Status::Ok) {
    return status;
  }
  return validRuntimeState(value->runtime_state) &&
                 validNavigationMode(value->navigation_mode) &&
                 validNisSummary(value->imu_yaw_nis_evaluated_count,
                                 value->imu_yaw_nis_gate_rejected_count,
                                 value->imu_yaw_nis_sum,
                                 value->imu_yaw_nis_max) &&
                 validNisSummary(value->wheel_nis_evaluated_count,
                                 value->wheel_nis_gate_rejected_count,
                                 value->wheel_nis_sum,
                                 value->wheel_nis_max) &&
                 validNisSummary(value->gnss_nis_evaluated_count,
                                 value->gnss_nis_gate_rejected_count,
                                 value->gnss_nis_sum,
                                 value->gnss_nis_max) &&
                 validNisSummary(value->landmark_nis_evaluated_count,
                                 value->landmark_nis_gate_rejected_count,
                                 value->landmark_nis_sum,
                                 value->landmark_nis_max)
             ? Status::Ok
             : Status::InvalidValue;
}

Status encodePayload(const HeartbeatPayload &value, uint8_t *output,
                     size_t capacity, uint16_t *size) {
  if (!validRuntimeState(value.runtime_state)) {
    return Status::InvalidValue;
  }
  Writer writer(output, capacity);
  writer.putU32(value.uptime_ms);
  writer.putU32(value.monotonic_ms);
  writer.putU8(static_cast<uint8_t>(value.runtime_state));
  writer.putU8(0U);
  writer.putU8(0U);
  writer.putU8(0U);
  return finishWrite(writer, size);
}

Status decodePayload(const uint8_t *data, uint16_t size,
                     HeartbeatPayload *value) {
  if (value == 0) {
    return Status::NullArgument;
  }
  Status status = requireExactSize(data, size, kHeartbeatPayloadSize);
  if (status != Status::Ok) {
    return status;
  }
  Reader reader(data, size);
  value->uptime_ms = reader.getU32();
  value->monotonic_ms = reader.getU32();
  value->runtime_state = static_cast<RuntimeState>(reader.getU8());
  const uint8_t reserved0 = reader.getU8();
  const uint8_t reserved1 = reader.getU8();
  const uint8_t reserved2 = reader.getU8();
  status = finishRead(reader);
  if (status != Status::Ok) {
    return status;
  }
  return reserved0 == 0U && reserved1 == 0U && reserved2 == 0U &&
                 validRuntimeState(value->runtime_state)
             ? Status::Ok
             : Status::InvalidValue;
}

Status encodePayload(const ErrorPayload &value, uint8_t *output,
                     size_t capacity, uint16_t *size) {
  if (!validApplicationError(value.code)) {
    return Status::InvalidValue;
  }
  Writer writer(output, capacity);
  writer.putU16(static_cast<uint16_t>(value.code));
  writer.putU16(value.detail);
  writer.putU32(value.related_sequence);
  writer.putU32(value.context);
  return finishWrite(writer, size);
}

Status decodePayload(const uint8_t *data, uint16_t size, ErrorPayload *value) {
  if (value == 0) {
    return Status::NullArgument;
  }
  Status status = requireExactSize(data, size, kErrorPayloadSize);
  if (status != Status::Ok) {
    return status;
  }
  Reader reader(data, size);
  value->code = static_cast<ApplicationErrorCode>(reader.getU16());
  value->detail = reader.getU16();
  value->related_sequence = reader.getU32();
  value->context = reader.getU32();
  status = finishRead(reader);
  if (status != Status::Ok) {
    return status;
  }
  return validApplicationError(value->code) ? Status::Ok : Status::InvalidValue;
}

Status encodePayload(const SafeStopPayload &value, uint8_t *output,
                     size_t capacity, uint16_t *size) {
  if (!validSafeStopReason(value.reason)) {
    return Status::InvalidValue;
  }
  Writer writer(output, capacity);
  writer.putU8(static_cast<uint8_t>(value.reason));
  writer.putU8(value.latch ? 1U : 0U);
  writer.putU16(value.detail);
  return finishWrite(writer, size);
}

Status decodePayload(const uint8_t *data, uint16_t size,
                     SafeStopPayload *value) {
  if (value == 0) {
    return Status::NullArgument;
  }
  Status status = requireExactSize(data, size, kSafeStopPayloadSize);
  if (status != Status::Ok) {
    return status;
  }
  Reader reader(data, size);
  value->reason = static_cast<SafeStopReason>(reader.getU8());
  const uint8_t latch = reader.getU8();
  value->detail = reader.getU16();
  status = finishRead(reader);
  if (status != Status::Ok) {
    return status;
  }
  if (latch > 1U || !validSafeStopReason(value->reason)) {
    return Status::InvalidValue;
  }
  value->latch = latch != 0U;
  return Status::Ok;
}

StreamParser::StreamParser()
    : encoded_size_(0U), discard_until_delimiter_(false) {
  memset(&stats_, 0, sizeof(stats_));
}

void StreamParser::reset(bool clear_stats) {
  encoded_size_ = 0U;
  discard_until_delimiter_ = false;
  if (clear_stats) {
    memset(&stats_, 0, sizeof(stats_));
  }
}

void StreamParser::countError(Status status) {
  switch (status) {
    case Status::CobsMalformed:
      stats_.cobs_errors = saturatingAdd(stats_.cobs_errors, 1U);
      break;
    case Status::CrcMismatch:
      stats_.crc_errors = saturatingAdd(stats_.crc_errors, 1U);
      break;
    case Status::UnsupportedVersion:
      stats_.version_errors = saturatingAdd(stats_.version_errors, 1U);
      break;
    case Status::UnknownMessageType:
      stats_.type_errors = saturatingAdd(stats_.type_errors, 1U);
      break;
    case Status::PacketTooShort:
    case Status::PayloadLengthMismatch:
    case Status::PayloadTooLarge:
      stats_.length_errors = saturatingAdd(stats_.length_errors, 1U);
      break;
    default:
      stats_.other_errors = saturatingAdd(stats_.other_errors, 1U);
      break;
  }
}

void StreamParser::feed(const uint8_t *data, size_t size,
                        PacketCallback packet_callback,
                        ParserErrorCallback error_callback, void *context) {
  if (size != 0U && data == 0) {
    countError(Status::NullArgument);
    if (error_callback != 0) {
      error_callback(Status::NullArgument, context);
    }
    return;
  }
  stats_.bytes_received = saturatingAddSize(stats_.bytes_received, size);
  for (size_t index = 0U; index < size; ++index) {
    const uint8_t value = data[index];
    if (value == 0U) {
      if (discard_until_delimiter_) {
        discard_until_delimiter_ = false;
        encoded_size_ = 0U;
        continue;
      }
      if (encoded_size_ == 0U) {
        stats_.empty_delimiters =
            saturatingAdd(stats_.empty_delimiters, 1U);
        continue;
      }
      stats_.frames_received = saturatingAdd(stats_.frames_received, 1U);
      Packet packet;
      const Status status = decodePacket(encoded_, encoded_size_, &packet);
      encoded_size_ = 0U;
      if (status == Status::Ok) {
        stats_.packets_accepted =
            saturatingAdd(stats_.packets_accepted, 1U);
        if (packet_callback != 0) {
          packet_callback(packet, context);
        }
      } else {
        countError(status);
        if (error_callback != 0) {
          error_callback(status, context);
        }
      }
      continue;
    }
    if (discard_until_delimiter_) {
      continue;
    }
    if (encoded_size_ >= kMaxEncodedFrameSize) {
      encoded_size_ = 0U;
      discard_until_delimiter_ = true;
      stats_.oversized_frames = saturatingAdd(stats_.oversized_frames, 1U);
      if (error_callback != 0) {
        error_callback(Status::OversizedFrame, context);
      }
      continue;
    }
    encoded_[encoded_size_++] = value;
  }
}

Status StreamParser::finish(ParserErrorCallback error_callback, void *context) {
  if (encoded_size_ == 0U && !discard_until_delimiter_) {
    return Status::Ok;
  }
  encoded_size_ = 0U;
  discard_until_delimiter_ = false;
  stats_.truncated_frames = saturatingAdd(stats_.truncated_frames, 1U);
  if (error_callback != 0) {
    error_callback(Status::TruncatedFrame, context);
  }
  return Status::TruncatedFrame;
}

SequenceTracker::SequenceTracker(uint32_t reorder_window)
    : reorder_window_(reorder_window == 0U || reorder_window >= 0x80000000UL
                          ? 32U
                          : reorder_window),
      last_sequence_(0U),
      last_step_id_(0U),
      initialized_(false),
      has_last_step_(false) {
  memset(&stats_, 0, sizeof(stats_));
}

void SequenceTracker::reset(bool clear_stats) {
  last_sequence_ = 0U;
  last_step_id_ = 0U;
  initialized_ = false;
  has_last_step_ = false;
  if (clear_stats) {
    memset(&stats_, 0, sizeof(stats_));
  }
}

SequenceResult SequenceTracker::observe(uint32_t sequence) {
  return observeInternal(sequence, false, 0U);
}

SequenceResult SequenceTracker::observe(uint32_t sequence, uint32_t step_id) {
  return observeInternal(sequence, true, step_id);
}

SequenceResult SequenceTracker::observeInternal(uint32_t sequence, bool has_step,
                                                uint32_t step_id) {
  if (!initialized_) {
    initialized_ = true;
    last_sequence_ = sequence;
    last_step_id_ = step_id;
    has_last_step_ = has_step;
    stats_.accepted = saturatingAdd(stats_.accepted, 1U);
    const SequenceResult result = {SequenceDisposition::First, true, 0U};
    return result;
  }

  const uint32_t delta = sequence - last_sequence_;
  if (delta == 0U) {
    stats_.duplicates = saturatingAdd(stats_.duplicates, 1U);
    const SequenceResult result = {SequenceDisposition::Duplicate, false, 0U};
    return result;
  }
  if (delta < 0x80000000UL) {
    if (has_step && has_last_step_) {
      const uint32_t step_delta = step_id - last_step_id_;
      if (step_delta >= 0x80000000UL && step_id != last_step_id_) {
        stats_.stale = saturatingAdd(stats_.stale, 1U);
        const SequenceResult result = {SequenceDisposition::Stale, false, 0U};
        return result;
      }
    }
    const uint32_t missing = delta - 1U;
    last_sequence_ = sequence;
    if (has_step) {
      last_step_id_ = step_id;
      has_last_step_ = true;
    }
    stats_.accepted = saturatingAdd(stats_.accepted, 1U);
    stats_.missing = saturatingAdd(stats_.missing, missing);
    const SequenceResult result = {
        missing == 0U ? SequenceDisposition::InOrder : SequenceDisposition::Gap,
        true, missing};
    return result;
  }

  const uint32_t backwards = last_sequence_ - sequence;
  if (backwards <= reorder_window_) {
    stats_.out_of_order = saturatingAdd(stats_.out_of_order, 1U);
    const SequenceResult result = {SequenceDisposition::OutOfOrder, false, 0U};
    return result;
  }
  stats_.stale = saturatingAdd(stats_.stale, 1U);
  const SequenceResult result = {SequenceDisposition::Stale, false, 0U};
  return result;
}

}  // namespace protocol
}  // namespace navbench
