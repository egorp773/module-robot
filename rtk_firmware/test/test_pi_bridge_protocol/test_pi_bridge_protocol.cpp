#include <cassert>
#include <cstdint>
#include <cstring>

#include "PiBridgeProtocol.h"

using namespace pibridge;

namespace {

uint8_t nibble(char value) {
    if (value >= '0' && value <= '9') return static_cast<uint8_t>(value - '0');
    if (value >= 'a' && value <= 'f') {
        return static_cast<uint8_t>(value - 'a' + 10);
    }
    if (value >= 'A' && value <= 'F') {
        return static_cast<uint8_t>(value - 'A' + 10);
    }
    assert(false);
    return 0u;
}

size_t decodeHex(const char* hex, uint8_t* output, size_t capacity) {
    const size_t length = std::strlen(hex);
    assert((length & 1u) == 0u);
    assert(length / 2u <= capacity);
    for (size_t i = 0u; i < length / 2u; ++i) {
        output[i] = static_cast<uint8_t>(
            (nibble(hex[i * 2u]) << 4u) | nibble(hex[i * 2u + 1u]));
    }
    return length / 2u;
}

void expectBytes(const uint8_t* actual, size_t actual_length,
                 const char* expected_hex) {
    uint8_t expected[kMaxEncodedFrameSize]{};
    const size_t expected_length = decodeHex(
        expected_hex, expected, sizeof(expected));
    assert(actual_length == expected_length);
    assert(std::memcmp(actual, expected, expected_length) == 0);
}

}  // namespace

int main() {
    static const uint8_t crc_input[] = {'1', '2', '3', '4', '5',
                                        '6', '7', '8', '9'};
    assert(crc16CcittFalse(crc_input, sizeof(crc_input)) == 0x29B1u);

    const uint8_t cobs_input[] = {0x00u, 0x11u, 0x00u, 0x22u, 0x33u, 0x00u};
    uint8_t encoded_cobs[16]{};
    const size_t encoded_cobs_length = cobsEncode(
        cobs_input, sizeof(cobs_input), encoded_cobs, sizeof(encoded_cobs));
    assert(encoded_cobs_length != 0u);
    uint8_t decoded_cobs[16]{};
    const size_t decoded_cobs_length = cobsDecode(
        encoded_cobs, encoded_cobs_length, decoded_cobs,
        sizeof(decoded_cobs));
    assert(decoded_cobs_length == sizeof(cobs_input));
    assert(std::memcmp(decoded_cobs, cobs_input, sizeof(cobs_input)) == 0);
    const uint8_t truncated_cobs[] = {0x03u, 0x11u};
    assert(cobsDecode(truncated_cobs, sizeof(truncated_cobs), decoded_cobs,
                      sizeof(decoded_cobs)) == 0u);

    HelloPayload hello{};
    hello.protocol_version = 1u;
    hello.requested_capabilities = 0x3Fu;
    std::strncpy(hello.client_name, "module_robot_bridge",
                 sizeof(hello.client_name) - 1u);
    std::strncpy(hello.client_build_id, "test-build",
                 sizeof(hello.client_build_id) - 1u);
    FrameHeader hello_header{};
    hello_header.protocol_version = 1u;
    hello_header.message_type = static_cast<uint8_t>(MessageType::HELLO);
    hello_header.payload_length = sizeof(hello);
    hello_header.sequence = 0x01020304u;
    hello_header.sender_monotonic_us = 0x0102030405060708ULL;
    uint8_t hello_wire[kMaxEncodedFrameSize]{};
    const size_t hello_wire_length = encodeFrame(
        hello_header, &hello, hello_wire, sizeof(hello_wire));
    expectBytes(
        hello_wire, hello_wire_length,
        "040101480e040302010807060504030201010101023f0101146d6f64756c655f"
        "726f626f745f6272696467650101010101010101010101010b746573742d6275"
        "696c6401010101010101010101010101010101010101010103b5f800");

    CmdVelPayload command{};
    command.linear_mm_s = 100;
    command.angular_mrad_s = -250;
    command.command_timeout_ms = 300u;
    command.control_mode = static_cast<uint8_t>(ControlMode::MANUAL);
    FrameHeader command_header{};
    command_header.protocol_version = 1u;
    command_header.message_type = static_cast<uint8_t>(MessageType::CMD_VEL);
    command_header.payload_length = sizeof(command);
    command_header.sequence = 0xFFFFFFFEu;
    command_header.sender_monotonic_us = 0x1122334455667788ULL;
    uint8_t command_wire[kMaxEncodedFrameSize]{};
    const size_t command_wire_length = encodeFrame(
        command_header, &command, command_wire, sizeof(command_wire));
    expectBytes(command_wire, command_wire_length,
                "0401060d0efeffffff88776655443322116401010706ffffff2c01010401a46000");

    StatusPayload status{};
    status.state = static_cast<uint8_t>(EspState::ARMED);
    status.armed = 1u;
    status.reset_reason = 1u;
    status.last_cmd_vel_age_ms = 20u;
    status.last_pi_heartbeat_age_ms = 40u;
    status.last_gnss_age_ms = 60u;
    status.last_imu_age_ms = 80u;
    status.last_motor_feedback_age_ms = 100u;
    status.applied_left_command = 5;
    status.applied_right_command = 6;
    status.uart_speed = 7;
    status.uart_steer = 8;
    status.watchdog_trips = 2u;
    status.boot_counter = 9u;
    status.uptime_ms = 1234u;
    FrameHeader status_header{};
    status_header.protocol_version = 1u;
    status_header.message_type = static_cast<uint8_t>(MessageType::STATUS);
    status_header.payload_length = sizeof(status);
    status_header.sequence = 42u;
    status_header.sender_monotonic_us = 1u;
    uint8_t status_wire[kMaxEncodedFrameSize]{};
    const size_t status_wire_length = encodeFrame(
        status_header, &status, status_wire, sizeof(status_wire));
    expectBytes(
        status_wire, status_wire_length,
        "04018330022a010102010101010101010303010201010101021401010228010102"
        "3c010102500101026401010205020602070208020201010209010103d204010382"
        "d300");

    // Fragmentation: a frame is published only when the delimiter arrives.
    FrameStreamDecoder decoder;
    Frame decoded;
    for (size_t i = 0u; i + 1u < command_wire_length; ++i) {
        assert(decoder.consume(command_wire[i], decoded) == DecodeResult::NONE);
    }
    assert(decoder.consume(command_wire[command_wire_length - 1u], decoded) ==
           DecodeResult::FRAME_READY);
    assert(decoded.header.sequence == command_header.sequence);
    assert(decoded.header.payload_length == sizeof(command));
    CmdVelPayload decoded_command{};
    std::memcpy(&decoded_command, decoded.payload, sizeof(decoded_command));
    assert(decoded_command.angular_mrad_s == -250);

    // The configured maximum payload must fit without heap allocation or an
    // off-by-one at the COBS delimiter boundary.
    uint8_t maximum_payload[kMaxPayloadSize]{};
    for (size_t i = 0u; i < sizeof(maximum_payload); ++i) {
        maximum_payload[i] = static_cast<uint8_t>(i);
    }
    FrameHeader maximum_header{};
    maximum_header.protocol_version = kProtocolVersion;
    maximum_header.message_type = static_cast<uint8_t>(MessageType::STATUS);
    maximum_header.payload_length = sizeof(maximum_payload);
    maximum_header.sequence = 77u;
    uint8_t maximum_wire[kMaxEncodedFrameSize]{};
    const size_t maximum_wire_length = encodeFrame(
        maximum_header, maximum_payload, maximum_wire,
        sizeof(maximum_wire));
    assert(maximum_wire_length > 0u);
    assert(maximum_wire_length <= sizeof(maximum_wire));
    FrameStreamDecoder maximum_decoder;
    for (size_t i = 0u; i < maximum_wire_length; ++i) {
        const DecodeResult result = maximum_decoder.consume(
            maximum_wire[i], decoded);
        if (i + 1u == maximum_wire_length) {
            assert(result == DecodeResult::FRAME_READY);
        } else {
            assert(result == DecodeResult::NONE);
        }
    }
    assert(decoded.header.payload_length == sizeof(maximum_payload));
    assert(std::memcmp(decoded.payload, maximum_payload,
                       sizeof(maximum_payload)) == 0);

    // Multiple frames in one read are handled by feeding through both
    // delimiters without resetting the decoder.
    unsigned ready_count = 0u;
    for (size_t i = 0u; i < command_wire_length; ++i) {
        if (decoder.consume(command_wire[i], decoded) ==
            DecodeResult::FRAME_READY) ++ready_count;
    }
    for (size_t i = 0u; i < hello_wire_length; ++i) {
        if (decoder.consume(hello_wire[i], decoded) ==
            DecodeResult::FRAME_READY) ++ready_count;
    }
    assert(ready_count == 2u);

    uint8_t corrupted[kMaxEncodedFrameSize]{};
    std::memcpy(corrupted, command_wire, command_wire_length);
    corrupted[10] ^= 0x01u;
    DecodeResult corrupted_result = DecodeResult::NONE;
    for (size_t i = 0u; i < command_wire_length; ++i) {
        const DecodeResult result = decoder.consume(corrupted[i], decoded);
        if (result != DecodeResult::NONE) corrupted_result = result;
    }
    assert(corrupted_result == DecodeResult::CRC_ERROR);

    // A valid-CRC frame with a lying payload length is rejected before copy.
    uint8_t raw[32]{};
    raw[0] = 1u;
    raw[1] = static_cast<uint8_t>(MessageType::STOP);
    raw[2] = 1u;  // claims one payload byte, but raw contains none
    raw[3] = 0u;
    const uint16_t bad_length_crc = crc16CcittFalse(raw, 16u);
    raw[16] = static_cast<uint8_t>(bad_length_crc);
    raw[17] = static_cast<uint8_t>(bad_length_crc >> 8u);
    uint8_t malformed[40]{};
    const size_t malformed_length = cobsEncode(
        raw, 18u, malformed, sizeof(malformed) - 1u);
    malformed[malformed_length] = 0u;
    DecodeResult malformed_result = DecodeResult::NONE;
    for (size_t i = 0u; i <= malformed_length; ++i) {
        const DecodeResult result = decoder.consume(malformed[i], decoded);
        if (result != DecodeResult::NONE) malformed_result = result;
    }
    assert(malformed_result == DecodeResult::LENGTH_ERROR);

    FrameStreamDecoder oversized_decoder;
    DecodeResult oversized_result = DecodeResult::NONE;
    for (size_t i = 0u; i < kMaxEncodedFrameSize; ++i) {
        const DecodeResult result = oversized_decoder.consume(0x01u, decoded);
        if (result != DecodeResult::NONE) oversized_result = result;
    }
    oversized_decoder.consume(0u, decoded);
    assert(oversized_result == DecodeResult::OVERSIZED);

    assert(sequenceIsNewer(0xFFFFFFFFu, 0xFFFFFFFEu));
    assert(sequenceIsNewer(0u, 0xFFFFFFFFu));
    assert(!sequenceIsNewer(0xFFFFFFFFu, 0u));
    assert(!sequenceIsNewer(5u, 5u));
    assert(!sequenceIsNewer(0x80000000u, 0u));
    assert(!sequenceIsNewer(0u, 0x80000000u));
    assert(isKnownInboundMessageType(
        static_cast<uint8_t>(MessageType::HELLO)));
    assert(isKnownInboundMessageType(
        static_cast<uint8_t>(MessageType::REQUEST_STATUS)));
    assert(!isKnownInboundMessageType(
        static_cast<uint8_t>(MessageType::HELLO_ACK)));
    assert(!isKnownInboundMessageType(0u));
    assert(!isKnownInboundMessageType(0x7Fu));
    return 0;
}
