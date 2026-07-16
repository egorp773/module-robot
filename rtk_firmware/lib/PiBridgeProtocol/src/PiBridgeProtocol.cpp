#include "PiBridgeProtocol.h"

#include <string.h>

namespace pibridge {
namespace {

uint16_t readU16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0]) |
           (static_cast<uint16_t>(p[1]) << 8u);
}

uint32_t readU32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8u) |
           (static_cast<uint32_t>(p[2]) << 16u) |
           (static_cast<uint32_t>(p[3]) << 24u);
}

uint64_t readU64(const uint8_t* p) {
    return static_cast<uint64_t>(readU32(p)) |
           (static_cast<uint64_t>(readU32(p + 4u)) << 32u);
}

void writeU16(uint8_t* p, uint16_t value) {
    p[0] = static_cast<uint8_t>(value);
    p[1] = static_cast<uint8_t>(value >> 8u);
}

void writeU32(uint8_t* p, uint32_t value) {
    p[0] = static_cast<uint8_t>(value);
    p[1] = static_cast<uint8_t>(value >> 8u);
    p[2] = static_cast<uint8_t>(value >> 16u);
    p[3] = static_cast<uint8_t>(value >> 24u);
}

void writeU64(uint8_t* p, uint64_t value) {
    writeU32(p, static_cast<uint32_t>(value));
    writeU32(p + 4u, static_cast<uint32_t>(value >> 32u));
}

}  // namespace

uint16_t crc16CcittFalse(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFFu;
    if (data == nullptr && length != 0u) return crc;
    for (size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8u;
        for (uint8_t bit = 0; bit < 8u; ++bit) {
            crc = (crc & 0x8000u) != 0u
                ? static_cast<uint16_t>((crc << 1u) ^ 0x1021u)
                : static_cast<uint16_t>(crc << 1u);
        }
    }
    return crc;
}

size_t cobsEncode(const uint8_t* input, size_t input_length,
                  uint8_t* output, size_t output_capacity) {
    if (output == nullptr || output_capacity == 0u ||
        (input == nullptr && input_length != 0u)) {
        return 0u;
    }

    size_t read_index = 0u;
    size_t write_index = 1u;
    size_t code_index = 0u;
    uint8_t code = 1u;

    while (read_index < input_length) {
        if (input[read_index] == 0u) {
            if (code_index >= output_capacity) return 0u;
            output[code_index] = code;
            code = 1u;
            code_index = write_index++;
            if (write_index > output_capacity) return 0u;
            ++read_index;
        } else {
            if (write_index >= output_capacity) return 0u;
            output[write_index++] = input[read_index++];
            ++code;
            if (code == 0xFFu) {
                if (code_index >= output_capacity) return 0u;
                output[code_index] = code;
                code = 1u;
                code_index = write_index++;
                if (write_index > output_capacity) return 0u;
            }
        }
    }

    if (code_index >= output_capacity) return 0u;
    output[code_index] = code;
    return write_index;
}

size_t cobsDecode(const uint8_t* input, size_t input_length,
                  uint8_t* output, size_t output_capacity) {
    if (input == nullptr || output == nullptr || input_length == 0u) return 0u;

    size_t read_index = 0u;
    size_t write_index = 0u;
    while (read_index < input_length) {
        const uint8_t code = input[read_index++];
        if (code == 0u) return 0u;
        const size_t copy_length = static_cast<size_t>(code - 1u);
        if (copy_length > input_length - read_index ||
            copy_length > output_capacity - write_index) {
            return 0u;
        }
        for (size_t i = 0u; i < copy_length; ++i) {
            output[write_index++] = input[read_index++];
        }
        if (code != 0xFFu && read_index < input_length) {
            if (write_index >= output_capacity) return 0u;
            output[write_index++] = 0u;
        }
    }
    return write_index;
}

size_t encodeFrame(const FrameHeader& header, const void* payload,
                   uint8_t* output, size_t output_capacity) {
    if (header.payload_length > kMaxPayloadSize || output == nullptr ||
        output_capacity < 2u ||
        (header.payload_length != 0u && payload == nullptr)) {
        return 0u;
    }

    uint8_t raw[kMaxRawFrameSize]{};
    raw[0] = header.protocol_version;
    raw[1] = header.message_type;
    writeU16(raw + 2u, header.payload_length);
    writeU32(raw + 4u, header.sequence);
    writeU64(raw + 8u, header.sender_monotonic_us);
    if (header.payload_length != 0u) {
        memcpy(raw + kHeaderSize, payload, header.payload_length);
    }
    const size_t crc_offset = kHeaderSize + header.payload_length;
    writeU16(raw + crc_offset, crc16CcittFalse(raw, crc_offset));

    const size_t encoded_length = cobsEncode(
        raw, crc_offset + kCrcSize, output, output_capacity - 1u);
    if (encoded_length == 0u || encoded_length >= output_capacity) return 0u;
    output[encoded_length] = 0u;
    return encoded_length + 1u;
}

void FrameStreamDecoder::reset() {
    _encoded_length = 0u;
    _discard_until_delimiter = false;
}

DecodeResult FrameStreamDecoder::consume(uint8_t byte, Frame& output) {
    if (byte != 0u) {
        if (_discard_until_delimiter) return DecodeResult::NONE;
        // One byte in kMaxEncodedFrameSize is reserved for the delimiter.
        if (_encoded_length >= sizeof(_encoded) - 1u) {
            _encoded_length = 0u;
            _discard_until_delimiter = true;
            ++_counters.oversized_frames;
            ++_counters.rx_overflows;
            return DecodeResult::OVERSIZED;
        }
        _encoded[_encoded_length++] = byte;
        return DecodeResult::NONE;
    }

    if (_discard_until_delimiter) {
        _discard_until_delimiter = false;
        _encoded_length = 0u;
        return DecodeResult::NONE;
    }
    if (_encoded_length == 0u) return DecodeResult::EMPTY_DELIMITER;

    uint8_t raw[kMaxRawFrameSize]{};
    const size_t raw_length = cobsDecode(
        _encoded, _encoded_length, raw, sizeof(raw));
    _encoded_length = 0u;
    if (raw_length == 0u) {
        ++_counters.cobs_errors;
        return DecodeResult::COBS_ERROR;
    }
    if (raw_length < kHeaderSize + kCrcSize) {
        ++_counters.length_errors;
        return DecodeResult::LENGTH_ERROR;
    }

    const uint16_t payload_length = readU16(raw + 2u);
    if (payload_length > kMaxPayloadSize ||
        raw_length != kHeaderSize + payload_length + kCrcSize) {
        ++_counters.length_errors;
        if (payload_length > kMaxPayloadSize) ++_counters.oversized_frames;
        return payload_length > kMaxPayloadSize
            ? DecodeResult::OVERSIZED : DecodeResult::LENGTH_ERROR;
    }

    const uint16_t expected_crc = readU16(raw + raw_length - kCrcSize);
    const uint16_t actual_crc = crc16CcittFalse(raw, raw_length - kCrcSize);
    if (actual_crc != expected_crc) {
        ++_counters.crc_errors;
        return DecodeResult::CRC_ERROR;
    }

    output.header.protocol_version = raw[0];
    output.header.message_type = raw[1];
    output.header.payload_length = payload_length;
    output.header.sequence = readU32(raw + 4u);
    output.header.sender_monotonic_us = readU64(raw + 8u);
    if (payload_length != 0u) {
        memcpy(output.payload, raw + kHeaderSize, payload_length);
    }
    ++_counters.rx_frames;
    return DecodeResult::FRAME_READY;
}

bool isKnownInboundMessageType(uint8_t message_type) {
    // Keep this explicit. A numeric range would silently treat a future hole
    // in the enum as an implemented command instead of dropping and counting
    // it as required by the protocol.
    switch (static_cast<MessageType>(message_type)) {
        case MessageType::HELLO:
        case MessageType::TIME_SYNC_REQUEST:
        case MessageType::HEARTBEAT:
        case MessageType::ARM:
        case MessageType::DISARM:
        case MessageType::CMD_VEL:
        case MessageType::STOP:
        case MessageType::ESTOP:
        case MessageType::RESET_ESTOP:
        case MessageType::RESET_FAULT:
        case MessageType::RELAY_COMMAND:
        case MessageType::SET_LIMITS:
        case MessageType::REQUEST_STATUS:
            return true;
        default:
            return false;
    }
}

bool sequenceIsNewer(uint32_t candidate, uint32_t previous) {
    // RFC-1982 serial-number arithmetic. Expressing the comparison entirely
    // in the unsigned domain avoids implementation-defined uint32_t ->
    // int32_t conversion at and above the half-range boundary.
    const uint32_t delta = candidate - previous;
    return delta != 0u && delta < 0x80000000u;
}

}  // namespace pibridge
