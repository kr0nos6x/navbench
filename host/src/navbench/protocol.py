from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum


PROTOCOL_VERSION = 1
MAX_PAYLOAD_SIZE = 128

_HEADER = struct.Struct("<BBHII")
_CRC = struct.Struct("<H")


class MessageType(IntEnum):
    HELLO = 1
    HELLO_ACK = 2
    SENSOR_FRAME = 3
    CONTROL_COMMAND = 4
    HEARTBEAT = 5
    ERROR = 255


@dataclass(frozen=True, slots=True)
class Packet:
    message_type: MessageType
    sequence: int
    step_id: int
    payload: bytes = b""


class ProtocolError(ValueError):
    pass


def encode_packet(packet: Packet) -> bytes:
    payload_size = len(packet.payload)

    if payload_size > MAX_PAYLOAD_SIZE:
        raise ProtocolError(
            f"payload exceeds {MAX_PAYLOAD_SIZE} bytes"
        )

    if not 0 <= packet.sequence <= 0xFFFFFFFF:
        raise ProtocolError("sequence must fit in uint32")

    if not 0 <= packet.step_id <= 0xFFFFFFFF:
        raise ProtocolError("step_id must fit in uint32")

    header = _HEADER.pack(
        PROTOCOL_VERSION,
        int(packet.message_type),
        payload_size,
        packet.sequence,
        packet.step_id,
    )

    packet_without_crc = header + packet.payload
    checksum = crc16_ccitt(packet_without_crc)
    raw_packet = packet_without_crc + _CRC.pack(checksum)

    return cobs_encode(raw_packet) + b"\x00"


def decode_packet(frame: bytes) -> Packet:
    if frame.endswith(b"\x00"):
        frame = frame[:-1]

    if not frame:
        raise ProtocolError("frame is empty")

    if b"\x00" in frame:
        raise ProtocolError("frame contains an unexpected delimiter")

    raw_packet = cobs_decode(frame)

    minimum_size = _HEADER.size + _CRC.size
    if len(raw_packet) < minimum_size:
        raise ProtocolError("packet is shorter than the header and CRC")

    packet_without_crc = raw_packet[:-_CRC.size]
    received_crc = _CRC.unpack(raw_packet[-_CRC.size:])[0]
    calculated_crc = crc16_ccitt(packet_without_crc)

    if received_crc != calculated_crc:
        raise ProtocolError("CRC mismatch")

    (
        version,
        message_type_value,
        payload_size,
        sequence,
        step_id,
    ) = _HEADER.unpack(raw_packet[:_HEADER.size])

    if version != PROTOCOL_VERSION:
        raise ProtocolError(
            f"unsupported protocol version: {version}"
        )

    payload = raw_packet[_HEADER.size:-_CRC.size]

    if len(payload) != payload_size:
        raise ProtocolError("payload length mismatch")

    if payload_size > MAX_PAYLOAD_SIZE:
        raise ProtocolError("payload exceeds protocol limit")

    try:
        message_type = MessageType(message_type_value)
    except ValueError as error:
        raise ProtocolError(
            f"unknown message type: {message_type_value}"
        ) from error

    return Packet(
        message_type=message_type,
        sequence=sequence,
        step_id=step_id,
        payload=payload,
    )


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF

    for value in data:
        crc ^= value << 8

        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF

    return crc


def cobs_encode(data: bytes) -> bytes:
    encoded = bytearray([0])
    code_index = 0
    code = 1

    for value in data:
        if value == 0:
            encoded[code_index] = code
            code_index = len(encoded)
            encoded.append(0)
            code = 1
            continue

        encoded.append(value)
        code += 1

        if code == 0xFF:
            encoded[code_index] = code
            code_index = len(encoded)
            encoded.append(0)
            code = 1

    encoded[code_index] = code
    return bytes(encoded)


def cobs_decode(data: bytes) -> bytes:
    if not data:
        raise ProtocolError("COBS frame is empty")

    decoded = bytearray()
    index = 0

    while index < len(data):
        code = data[index]

        if code == 0:
            raise ProtocolError("invalid zero byte in COBS frame")

        index += 1
        block_end = index + code - 1

        if block_end > len(data):
            raise ProtocolError("COBS block exceeds frame length")

        decoded.extend(data[index:block_end])
        index = block_end

        if code != 0xFF and index < len(data):
            decoded.append(0)

    return bytes(decoded)
