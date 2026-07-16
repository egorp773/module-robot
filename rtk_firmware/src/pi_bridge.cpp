#include <Arduino.h>
#include <Preferences.h>

#include <esp_idf_version.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>

#include "PiBridgeGnss.h"
#include "PiBridgeImu.h"
#include "PiBridgeMotor.h"
#include "PiBridgeProtocol.h"
#include "PiBridgeSafety.h"
#include "RtkConfig.h"

#include <string.h>

#ifndef PI_BRIDGE_SERIAL_BAUD
#define PI_BRIDGE_SERIAL_BAUD 460800
#endif

#if !defined(ROLE_PI_BRIDGE) || !defined(NO_GLOBAL_SERIAL) || !defined(Serial)
#error "pi_bridge must be built by the isolated binary-UART PlatformIO environment"
#endif

PiBridgeNullPrint PiBridgeLibraryLogSink;

namespace {

using namespace pibridge;

constexpr char kFirmwareBuildId[] = "pi_bridge-v1";
constexpr uint32_t kStatusPeriodMs = 200u;
constexpr uint32_t kMotorTelemetryPeriodMs = 100u;
constexpr uint32_t kPowerPeriodMs = 500u;
constexpr uint32_t kRelayPeriodMs = 1000u;
constexpr uint32_t kDiagnosticsPeriodMs = 1000u;
constexpr uint32_t kSensorRetryPeriodMs = 5000u;
constexpr uint32_t kMotorTaskWatchdogTimeoutMs = 1000u;
constexpr uint8_t kRelayAttachmentBit = 1u << 0;
constexpr uint8_t kRelayMountBit = 1u << 1;

HardwareSerial g_pi_serial(0);
HardwareSerial g_motor_serial(2);
HardwareSerial g_gnss_serial(1);
PiBridgeMotor g_motor;
PiBridgeImu g_imu;
PiBridgeGnss g_gnss;
SafetyMachine g_safety;
FrameStreamDecoder g_decoder;

portMUX_TYPE g_safety_mux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t g_motor_task = nullptr;
volatile uint32_t g_motor_zero_ack_generation = 0u;

uint32_t g_tx_sequence = 0u;
bool g_have_control_sequence = false;
uint32_t g_last_control_sequence = 0u;
uint32_t g_boot_counter = 0u;
uint8_t g_reset_reason = 0u;
bool g_attachment_relay = false;
bool g_mount_relay = false;
uint32_t g_relay_sequence = 0u;

bool g_imu_started = false;
bool g_gnss_started = false;
uint32_t g_last_imu_retry_ms = 0u;
uint32_t g_last_gnss_retry_ms = 0u;
uint32_t g_last_status_ms = 0u;
uint32_t g_last_motor_telemetry_ms = 0u;
uint32_t g_last_power_ms = 0u;
uint32_t g_last_relay_ms = 0u;
uint32_t g_last_diagnostics_ms = 0u;

uint64_t monotonicUs() {
    return static_cast<uint64_t>(esp_timer_get_time());
}

uint32_t nextTxSequence() {
    ++g_tx_sequence;
    if (g_tx_sequence == 0u) ++g_tx_sequence;
    return g_tx_sequence;
}

void sendCommandAck(const Frame& request, const SafetyReply& reply);
void sendHelloAck(bool accepted);
void relaysOff();

enum class SequenceCheck : uint8_t {
    NEWER,
    DUPLICATE,
    OUT_OF_ORDER,
};

SequenceCheck checkControlSequence(uint32_t sequence) {
    if (!g_have_control_sequence ||
        sequenceIsNewer(sequence, g_last_control_sequence)) {
        return SequenceCheck::NEWER;
    }
    return sequence == g_last_control_sequence
        ? SequenceCheck::DUPLICATE : SequenceCheck::OUT_OF_ORDER;
}

void recordControlSequence(uint32_t sequence) {
    g_have_control_sequence = true;
    g_last_control_sequence = sequence;
}

SafetyReply staleSequenceReply(SequenceCheck check) {
    SafetyReply reply;
    reply.result = AckResult::STALE_SEQUENCE;
    if (check == SequenceCheck::DUPLICATE) {
        reply.detail = AckDetail::DUPLICATE_SEQUENCE;
        ++g_decoder.mutableCounters().duplicate_sequences;
    } else {
        reply.detail = AckDetail::OUT_OF_ORDER_SEQUENCE;
        ++g_decoder.mutableCounters().out_of_order_sequences;
    }
    return reply;
}

bool acceptStateChangingSequence(const Frame& frame) {
    const SequenceCheck check = checkControlSequence(frame.header.sequence);
    if (check != SequenceCheck::NEWER) {
        sendCommandAck(frame, staleSequenceReply(check));
        return false;
    }
    recordControlSequence(frame.header.sequence);
    return true;
}

bool acceptInformationalSequence(const Frame& frame) {
    const SequenceCheck check = checkControlSequence(frame.header.sequence);
    if (check != SequenceCheck::NEWER) {
        (void)staleSequenceReply(check);
        return false;
    }
    recordControlSequence(frame.header.sequence);
    return true;
}

void observeZeroOnlySequence(const Frame& frame) {
    // STOP, DISARM and ESTOP are fail-safe overrides, not motion-authority
    // commands. A duplicate/out-of-order instance is counted but still
    // allowed to remove motion; it can never recreate or refresh motion.
    const SequenceCheck check = checkControlSequence(frame.header.sequence);
    if (check == SequenceCheck::NEWER) {
        recordControlSequence(frame.header.sequence);
    } else {
        (void)staleSequenceReply(check);
    }
}

MotorHealth motorHealth(const MotorTelemetry& telemetry) {
    MotorHealth out;
    out.feedback_available = telemetry.available;
    out.feedback_fresh = telemetry.fresh;
    out.feedback_at_zero = telemetry.at_zero;
    out.controller_fault = telemetry.controller_fault != 0u;
    return out;
}

void wakeMotorTask() {
    if (g_motor_task != nullptr) xTaskNotifyGive(g_motor_task);
}

void waitForHardZero(uint32_t generation) {
    if (generation == 0u) return;
    wakeMotorTask();
    const uint32_t started_ms = millis();
    while (g_motor_zero_ack_generation != generation &&
           millis() - started_ms < 25u) {
        delay(1u);
    }
    if (g_motor_zero_ack_generation == generation) return;

    // Emergency zero-only fallback. The safety lock serializes this with the
    // dedicated motor owner; it can never create non-zero motion.
    taskENTER_CRITICAL(&g_safety_mux);
    g_motor.hardZero(millis());
    g_motor_zero_ack_generation = generation;
    taskEXIT_CRITICAL(&g_safety_mux);
}

void rejectHelloSession(uint32_t sequence, uint32_t now_ms) {
    // Any CRC-valid HELLO denotes a new transport session. If its version,
    // layout or requested capabilities cannot be accepted, fail closed: the
    // previous session loses motion authority and relays before HELLO_ACK is
    // returned. A rejected HELLO can never preserve an old ARMED session.
    recordControlSequence(sequence);
    relaysOff();
    taskENTER_CRITICAL(&g_safety_mux);
    g_safety.connectionLost();
    const uint32_t hard_zero_generation =
        g_safety.snapshot(now_ms).hard_zero_generation;
    taskEXIT_CRITICAL(&g_safety_mux);
    waitForHardZero(hard_zero_generation);
    sendHelloAck(false);
}

SafetySnapshot safetySnapshot(uint32_t now_ms) {
    taskENTER_CRITICAL(&g_safety_mux);
    const SafetySnapshot output = g_safety.snapshot(now_ms);
    taskEXIT_CRITICAL(&g_safety_mux);
    return output;
}

void relaysOffLocked() {
    const bool changed = g_attachment_relay || g_mount_relay;
    g_attachment_relay = false;
    g_mount_relay = false;
    digitalWrite(PIN_RELAY_ATTACH, LOW);
    digitalWrite(PIN_RELAY_MOUNT, LOW);
    if (changed) {
        ++g_relay_sequence;
        if (g_relay_sequence == 0u) ++g_relay_sequence;
    }
}

void relaysOff() {
    taskENTER_CRITICAL(&g_safety_mux);
    relaysOffLocked();
    taskEXIT_CRITICAL(&g_safety_mux);
}

void enforceRelaySafety(uint32_t now_ms) {
    (void)now_ms;
    taskENTER_CRITICAL(&g_safety_mux);
    if (g_safety.relayAllowedMask() == 0u) relaysOffLocked();
    taskEXIT_CRITICAL(&g_safety_mux);
}

bool sendPacket(MessageType type, const void* payload, uint16_t length) {
    if (length > kMaxPayloadSize) return false;
    FrameHeader header{};
    header.protocol_version = kProtocolVersion;
    header.message_type = static_cast<uint8_t>(type);
    header.payload_length = length;
    header.sequence = nextTxSequence();
    header.sender_monotonic_us = monotonicUs();

    uint8_t encoded[kMaxEncodedFrameSize]{};
    const size_t encoded_length = encodeFrame(
        header, payload, encoded, sizeof(encoded));
    if (encoded_length == 0u) {
        ++g_decoder.mutableCounters().tx_errors;
        return false;
    }
    const size_t written = g_pi_serial.write(encoded, encoded_length);
    if (written != encoded_length) {
        ++g_decoder.mutableCounters().tx_errors;
        return false;
    }
    ++g_decoder.mutableCounters().tx_frames;
    return true;
}

template <typename T>
bool copyPayload(const Frame& frame, T& output) {
    if (frame.header.payload_length != sizeof(T)) return false;
    memcpy(&output, frame.payload, sizeof(T));
    return true;
}

template <size_t N>
bool allZero(const uint8_t (&bytes)[N]) {
    for (size_t i = 0u; i < N; ++i) {
        if (bytes[i] != 0u) return false;
    }
    return true;
}

bool requireEmptyPayload(const Frame& frame) {
    return frame.header.payload_length == 0u;
}

void sendCommandAck(const Frame& request, const SafetyReply& reply) {
    CommandAckPayload payload{};
    payload.command_sequence = request.header.sequence;
    payload.command_type = request.header.message_type;
    payload.result = static_cast<uint8_t>(reply.result);
    payload.state = static_cast<uint8_t>(
        safetySnapshot(millis()).state);
    payload.detail_code = static_cast<uint16_t>(reply.detail);
    sendPacket(MessageType::COMMAND_ACK, &payload, sizeof(payload));
}

void sendInvalidLengthAck(const Frame& request) {
    ++g_decoder.mutableCounters().length_errors;
    SafetyReply reply;
    reply.result = AckResult::INVALID_PAYLOAD;
    reply.detail = AckDetail::PAYLOAD_LENGTH;
    sendCommandAck(request, reply);
}

void sendHelloAck(bool accepted) {
    HelloAckPayload payload{};
    const SafetySnapshot safety = safetySnapshot(millis());
    payload.protocol_version = kProtocolVersion;
    payload.accepted = accepted ? 1u : 0u;
    payload.state = static_cast<uint8_t>(safety.state);
    payload.supported_capabilities = kSupportedCapabilities;
    payload.max_payload = static_cast<uint16_t>(kMaxPayloadSize);
    strncpy(payload.firmware_build_id, kFirmwareBuildId,
            sizeof(payload.firmware_build_id) - 1u);
    sendPacket(MessageType::HELLO_ACK, &payload, sizeof(payload));
}

void sendStatus() {
    const uint32_t now_ms = millis();
    const SafetySnapshot safety = safetySnapshot(now_ms);
    const MotorTelemetry motor = g_motor.telemetry(now_ms);
    StatusPayload payload{};
    payload.state = static_cast<uint8_t>(safety.state);
    payload.armed = safety.state == EspState::ARMED ? 1u : 0u;
    payload.estop = safety.estop_latched ? 1u : 0u;
    payload.reset_reason = g_reset_reason;
    payload.fault_code = static_cast<uint16_t>(safety.fault);
    payload.last_cmd_vel_age_ms = safety.last_cmd_age_ms;
    payload.last_pi_heartbeat_age_ms = safety.last_heartbeat_age_ms;
    payload.last_gnss_age_ms = g_gnss.ageMs(now_ms);
    payload.last_imu_age_ms = g_imu.ageMs(now_ms);
    payload.last_motor_feedback_age_ms = motor.age_ms;
    payload.applied_left_command = motor.applied_left_percent;
    payload.applied_right_command = motor.applied_right_percent;
    payload.uart_speed = motor.uart_speed;
    payload.uart_steer = motor.uart_steer;
    payload.watchdog_trips = safety.watchdog_trips;
    payload.boot_counter = g_boot_counter;
    payload.uptime_ms = now_ms;
    sendPacket(MessageType::STATUS, &payload, sizeof(payload));
}

void sendMotorTelemetry() {
    const MotorTelemetry motor = g_motor.telemetry(millis());
    if (!motor.available) return;
    MotorFeedbackPayload payload{};
    payload.sensor_monotonic_us = motor.timestamp_us;
    payload.left_feedback = motor.left_feedback;
    payload.right_feedback = motor.right_feedback;
    payload.battery_centivolts = motor.battery_centivolts;
    payload.board_temperature_decic = motor.board_temperature_decic;
    payload.motor_controller_fault = motor.controller_fault;
    payload.uart_valid_frame_count = motor.valid_frames;
    payload.uart_invalid_frame_count = motor.invalid_frames;
    sendPacket(MessageType::MOTOR_FEEDBACK, &payload, sizeof(payload));
}

uint8_t batteryPercent(int16_t centivolts) {
    (void)centivolts;
    // TODO_CALIBRATE: battery chemistry, series-cell count, usable voltage
    // range and loaded-voltage curve are not yet proven. Keep voltage
    // telemetry, but explicitly mark state-of-charge unavailable.
    return 255u;
}

void sendPowerStatus() {
    const MotorTelemetry motor = g_motor.telemetry(millis());
    PowerStatusPayload payload{};
    payload.sensor_monotonic_us = motor.timestamp_us;
    payload.battery_voltage = motor.battery_centivolts * 0.01f;
    payload.battery_percent = batteryPercent(motor.battery_centivolts);
    // POWER_STATUS carries the compact hardware health bitmask; detailed Pi
    // transport corruption counters remain in DIAGNOSTICS and hoverboard
    // frame counters remain in MOTOR_FEEDBACK.
    if (motor.available) payload.flags |= POWER_FLAG_VALID;
    if (!g_imu_started) payload.flags |= POWER_FLAG_IMU_UNAVAILABLE;
    if (!g_gnss_started) payload.flags |= POWER_FLAG_GNSS_UNAVAILABLE;
    if (!motor.fresh) payload.flags |= POWER_FLAG_MOTOR_FEEDBACK_STALE;
    if (motor.controller_fault != 0u) {
        payload.flags |= POWER_FLAG_MOTOR_CONTROLLER_FAULT;
    }
    if (g_gnss.checksumFailures() != 0u) {
        payload.flags |= POWER_FLAG_GNSS_CHECKSUM_ERROR_SEEN;
    }
    if (g_gnss.oversizedPackets() != 0u) {
        payload.flags |= POWER_FLAG_GNSS_OVERSIZED_PACKET_SEEN;
    }
    if (motor.invalid_frames != 0u) {
        payload.flags |= POWER_FLAG_MOTOR_INVALID_FRAME_SEEN;
    }
    sendPacket(MessageType::POWER_STATUS, &payload, sizeof(payload));
}

void sendRelayStatus() {
    RelayStatusPayload payload{};
    payload.sensor_monotonic_us = monotonicUs();
    taskENTER_CRITICAL(&g_safety_mux);
    payload.attachment_enabled = g_attachment_relay ? 1u : 0u;
    payload.mount_enabled = g_mount_relay ? 1u : 0u;
    payload.allowed_mask = g_safety.relayAllowedMask();
    payload.state = static_cast<uint8_t>(g_safety.snapshot(millis()).state);
    payload.sequence = g_relay_sequence;
    taskEXIT_CRITICAL(&g_safety_mux);
    sendPacket(MessageType::RELAY_STATUS, &payload, sizeof(payload));
}

void sendDiagnostics() {
    const ProtocolCounters counters = g_decoder.counters();
    DiagnosticsPayload payload{};
    payload.sensor_monotonic_us = monotonicUs();
    payload.rx_frames = counters.rx_frames;
    payload.tx_frames = counters.tx_frames;
    payload.crc_errors = counters.crc_errors;
    payload.cobs_errors = counters.cobs_errors;
    payload.length_errors = counters.length_errors;
    payload.oversized_frames = counters.oversized_frames;
    payload.unknown_message_types = counters.unknown_message_types;
    payload.duplicate_sequences = counters.duplicate_sequences;
    payload.out_of_order_sequences = counters.out_of_order_sequences;
    payload.rx_overflows = counters.rx_overflows;
    payload.tx_errors = counters.tx_errors;
    sendPacket(MessageType::DIAGNOSTICS, &payload, sizeof(payload));
}

void sendPendingTransitions() {
    if (!safetySnapshot(millis()).handshake) return;

    FaultTransition fault;
    bool have_fault = false;
    EstopTransition estop;
    bool have_estop = false;
    taskENTER_CRITICAL(&g_safety_mux);
    have_fault = g_safety.consumeFaultTransition(fault);
    have_estop = g_safety.consumeEstopTransition(estop);
    taskEXIT_CRITICAL(&g_safety_mux);

    if (have_fault) {
        FaultEventPayload payload{};
        payload.sensor_monotonic_us = monotonicUs();
        payload.fault_code = static_cast<uint16_t>(fault.fault);
        payload.from_state = static_cast<uint8_t>(fault.from);
        payload.to_state = static_cast<uint8_t>(fault.to);
        payload.occurrence_count = fault.occurrence_count;
        sendPacket(MessageType::FAULT_EVENT, &payload, sizeof(payload));
    }
    if (have_estop) {
        EstopEventPayload payload{};
        payload.sensor_monotonic_us = monotonicUs();
        payload.latched = estop.latched ? 1u : 0u;
        payload.source = estop.source;
        payload.occurrence_count = estop.occurrence_count;
        sendPacket(MessageType::ESTOP_EVENT, &payload, sizeof(payload));
    }
}

void sendSensorTelemetry() {
    ImuSample imu;
    if (g_imu.consume(imu)) {
        ImuPayload payload{};
        payload.sensor_monotonic_us = imu.timestamp_us;
        payload.quaternion_x = imu.qx;
        payload.quaternion_y = imu.qy;
        payload.quaternion_z = imu.qz;
        payload.quaternion_w = imu.qw;
        payload.angular_velocity_x = imu.gx;
        payload.angular_velocity_y = imu.gy;
        payload.angular_velocity_z = imu.gz;
        payload.linear_acceleration_x = imu.ax;
        payload.linear_acceleration_y = imu.ay;
        payload.linear_acceleration_z = imu.az;
        payload.calibration_status = imu.calibration_status;
        payload.accuracy_rad = imu.accuracy_rad;
        payload.sequence = imu.sequence;
        sendPacket(MessageType::IMU, &payload, sizeof(payload));
    }

    GnssSample gnss;
    if (g_gnss.consume(gnss, millis())) {
        GnssPayload payload{};
        payload.sensor_monotonic_us = gnss.timestamp_us;
        payload.latitude_e7 = gnss.latitude_e7;
        payload.longitude_e7 = gnss.longitude_e7;
        payload.altitude_mm = gnss.altitude_mm;
        payload.h_acc_mm = gnss.h_acc_mm;
        payload.v_acc_mm = gnss.v_acc_mm;
        payload.ground_speed_mm_s = gnss.ground_speed_mm_s;
        payload.motion_heading_deg_e5 = gnss.motion_heading_deg_e5;
        payload.fix_type = gnss.fix_type;
        payload.carrier_solution = gnss.carrier_solution;
        payload.satellite_count = gnss.satellite_count;
        payload.rtcm_age_ms = gnss.rtcm_age_ms;
        payload.sequence = gnss.sequence;
        sendPacket(MessageType::GNSS, &payload, sizeof(payload));
    }
}

SafetyReply relayCommand(const RelayCommandPayload& command) {
    SafetyReply reply;
    if ((command.relay_mask & ~0x03u) != 0u ||
        (command.relay_values & ~0x03u) != 0u) {
        reply.result = AckResult::INVALID_PAYLOAD;
        reply.detail = AckDetail::BAD_LIMITS;
        return reply;
    }
    taskENTER_CRITICAL(&g_safety_mux);
    const uint8_t allowed = g_safety.relayAllowedMask();
    if ((command.relay_mask & allowed) != command.relay_mask) {
        taskEXIT_CRITICAL(&g_safety_mux);
        reply.result = AckResult::REJECTED_STATE;
        reply.detail = AckDetail::RELAY_STATE_FORBIDDEN;
        return reply;
    }

    if ((command.relay_mask & kRelayAttachmentBit) != 0u) {
        g_attachment_relay =
            (command.relay_values & kRelayAttachmentBit) != 0u;
        digitalWrite(PIN_RELAY_ATTACH, g_attachment_relay ? HIGH : LOW);
    }
    if ((command.relay_mask & kRelayMountBit) != 0u) {
        g_mount_relay = (command.relay_values & kRelayMountBit) != 0u;
        digitalWrite(PIN_RELAY_MOUNT, g_mount_relay ? HIGH : LOW);
    }
    ++g_relay_sequence;
    if (g_relay_sequence == 0u) ++g_relay_sequence;
    taskEXIT_CRITICAL(&g_safety_mux);
    return reply;
}

void handleFrame(const Frame& frame, uint64_t receive_us) {
    const MessageType type = static_cast<MessageType>(
        frame.header.message_type);
    const bool known_type =
        isKnownInboundMessageType(frame.header.message_type);
    if (frame.header.protocol_version != kProtocolVersion) {
        if (type == MessageType::HELLO) {
            rejectHelloSession(frame.header.sequence, millis());
        }
        if (!known_type) {
            ++g_decoder.mutableCounters().unknown_message_types;
        }
        return;
    }
    if (!known_type) {
        ++g_decoder.mutableCounters().unknown_message_types;
        // Unknown messages are dropped completely. In particular, a
        // CRC-valid but unsupported type must not advance the accepted Pi
        // sequence and poison the following valid control command.
        return;
    }

    const uint32_t now_ms = millis();
    switch (type) {
        case MessageType::HELLO: {
            HelloPayload command{};
            if (!copyPayload(frame, command)) {
                ++g_decoder.mutableCounters().length_errors;
                rejectHelloSession(frame.header.sequence, now_ms);
                return;
            }
            if (command.protocol_version != kProtocolVersion) {
                rejectHelloSession(frame.header.sequence, now_ms);
                return;
            }
            if (!allZero(command.reserved) ||
                (command.requested_capabilities &
                 ~kSupportedCapabilities) != 0u) {
                rejectHelloSession(frame.header.sequence, now_ms);
                return;
            }
            // HELLO starts a new sequence session unconditionally. A restarted
            // Pi deliberately chooses a random initial uint32 sequence, which
            // cannot be compared to the previous process' sequence space.
            // Replaying HELLO is safe: it can only hard-zero, turn relays off
            // and return to DISARMED; it can never create motion.
            recordControlSequence(frame.header.sequence);
            relaysOff();
            taskENTER_CRITICAL(&g_safety_mux);
            g_safety.acceptHello(now_ms);
            const uint32_t hard_zero_generation =
                g_safety.snapshot(now_ms).hard_zero_generation;
            taskEXIT_CRITICAL(&g_safety_mux);
            waitForHardZero(hard_zero_generation);
            sendHelloAck(true);
            break;
        }
        case MessageType::TIME_SYNC_REQUEST: {
            TimeSyncRequestPayload command{};
            if (!copyPayload(frame, command)) {
                sendInvalidLengthAck(frame);
                return;
            }
            if (!acceptInformationalSequence(frame)) return;
            TimeSyncResponsePayload response{};
            response.pi_send_monotonic_us = command.pi_send_monotonic_us;
            response.esp_receive_monotonic_us = receive_us;
            response.esp_send_monotonic_us = monotonicUs();
            sendPacket(MessageType::TIME_SYNC_RESPONSE,
                       &response, sizeof(response));
            break;
        }
        case MessageType::HEARTBEAT: {
            HeartbeatPayload command{};
            if (!copyPayload(frame, command)) {
                sendInvalidLengthAck(frame);
                return;
            }
            if (!allZero(command.reserved) || command.pi_state > 5u) {
                SafetyReply reply;
                reply.result = AckResult::INVALID_PAYLOAD;
                reply.detail = AckDetail::BAD_CONTROL_MODE;
                sendCommandAck(frame, reply);
                return;
            }
            if (!acceptInformationalSequence(frame)) return;
            taskENTER_CRITICAL(&g_safety_mux);
            g_safety.noteHeartbeat(now_ms);
            taskEXIT_CRITICAL(&g_safety_mux);
            break;
        }
        case MessageType::ARM: {
            ArmPayload command{};
            if (!copyPayload(frame, command)) {
                sendInvalidLengthAck(frame);
                return;
            }
            if (!allZero(command.reserved)) {
                SafetyReply reply;
                reply.result = AckResult::INVALID_PAYLOAD;
                sendCommandAck(frame, reply);
                return;
            }
            if (!acceptStateChangingSequence(frame)) {
                return;
            }
            const MotorTelemetry telemetry = g_motor.telemetry(now_ms);
            SafetyReply reply;
            taskENTER_CRITICAL(&g_safety_mux);
            reply = g_safety.arm(command, motorHealth(telemetry), now_ms);
            const uint32_t hard_zero_generation =
                g_safety.snapshot(now_ms).hard_zero_generation;
            taskEXIT_CRITICAL(&g_safety_mux);
            waitForHardZero(hard_zero_generation);
            sendCommandAck(frame, reply);
            break;
        }
        case MessageType::DISARM: {
            if (!requireEmptyPayload(frame)) {
                sendInvalidLengthAck(frame);
                return;
            }
            observeZeroOnlySequence(frame);
            SafetyReply reply;
            taskENTER_CRITICAL(&g_safety_mux);
            reply = g_safety.disarm();
            const uint32_t hard_zero_generation =
                g_safety.snapshot(now_ms).hard_zero_generation;
            taskEXIT_CRITICAL(&g_safety_mux);
            waitForHardZero(hard_zero_generation);
            sendCommandAck(frame, reply);
            break;
        }
        case MessageType::CMD_VEL: {
            CmdVelPayload command{};
            if (!copyPayload(frame, command)) {
                sendInvalidLengthAck(frame);
                return;
            }
            if (!acceptStateChangingSequence(frame)) return;
            SafetyReply reply;
            taskENTER_CRITICAL(&g_safety_mux);
            reply = g_safety.commandVelocity(
                frame.header.sequence, command, now_ms);
            taskEXIT_CRITICAL(&g_safety_mux);
            if (reply.result == AckResult::STALE_SEQUENCE) {
                if (reply.detail == AckDetail::DUPLICATE_SEQUENCE) {
                    ++g_decoder.mutableCounters().duplicate_sequences;
                } else {
                    ++g_decoder.mutableCounters().out_of_order_sequences;
                }
            }
            wakeMotorTask();
            sendCommandAck(frame, reply);
            break;
        }
        case MessageType::STOP: {
            if (!requireEmptyPayload(frame)) {
                sendInvalidLengthAck(frame);
                return;
            }
            observeZeroOnlySequence(frame);
            SafetyReply reply;
            taskENTER_CRITICAL(&g_safety_mux);
            reply = g_safety.stop();
            const uint32_t hard_zero_generation =
                g_safety.snapshot(now_ms).hard_zero_generation;
            taskEXIT_CRITICAL(&g_safety_mux);
            waitForHardZero(hard_zero_generation);
            sendCommandAck(frame, reply);
            break;
        }
        case MessageType::ESTOP: {
            if (!requireEmptyPayload(frame)) {
                sendInvalidLengthAck(frame);
                return;
            }
            observeZeroOnlySequence(frame);
            SafetyReply reply;
            taskENTER_CRITICAL(&g_safety_mux);
            reply = g_safety.estop(1u);
            const uint32_t hard_zero_generation =
                g_safety.snapshot(now_ms).hard_zero_generation;
            taskEXIT_CRITICAL(&g_safety_mux);
            relaysOff();
            waitForHardZero(hard_zero_generation);
            sendCommandAck(frame, reply);
            break;
        }
        case MessageType::RESET_ESTOP: {
            if (!requireEmptyPayload(frame)) {
                sendInvalidLengthAck(frame);
                return;
            }
            if (!acceptStateChangingSequence(frame)) return;
            const MotorTelemetry telemetry = g_motor.telemetry(now_ms);
            SafetyReply reply;
            taskENTER_CRITICAL(&g_safety_mux);
            reply = g_safety.resetEstop(motorHealth(telemetry));
            const uint32_t hard_zero_generation =
                g_safety.snapshot(now_ms).hard_zero_generation;
            taskEXIT_CRITICAL(&g_safety_mux);
            waitForHardZero(hard_zero_generation);
            sendCommandAck(frame, reply);
            break;
        }
        case MessageType::RESET_FAULT: {
            if (!requireEmptyPayload(frame)) {
                sendInvalidLengthAck(frame);
                return;
            }
            if (!acceptStateChangingSequence(frame)) return;
            const MotorTelemetry telemetry = g_motor.telemetry(now_ms);
            SafetyReply reply;
            taskENTER_CRITICAL(&g_safety_mux);
            reply = g_safety.resetFault(motorHealth(telemetry));
            const uint32_t hard_zero_generation =
                g_safety.snapshot(now_ms).hard_zero_generation;
            taskEXIT_CRITICAL(&g_safety_mux);
            waitForHardZero(hard_zero_generation);
            sendCommandAck(frame, reply);
            break;
        }
        case MessageType::RELAY_COMMAND: {
            RelayCommandPayload command{};
            if (!copyPayload(frame, command)) {
                sendInvalidLengthAck(frame);
                return;
            }
            if (!acceptStateChangingSequence(frame)) return;
            const SafetyReply reply = relayCommand(command);
            sendCommandAck(frame, reply);
            sendRelayStatus();
            break;
        }
        case MessageType::SET_LIMITS: {
            SetLimitsPayload command{};
            if (!copyPayload(frame, command)) {
                sendInvalidLengthAck(frame);
                return;
            }
            if (!acceptStateChangingSequence(frame)) return;
            SafetyReply reply;
            taskENTER_CRITICAL(&g_safety_mux);
            reply = g_safety.setLimits(command);
            const uint32_t hard_zero_generation =
                g_safety.snapshot(now_ms).hard_zero_generation;
            taskEXIT_CRITICAL(&g_safety_mux);
            waitForHardZero(hard_zero_generation);
            sendCommandAck(frame, reply);
            break;
        }
        case MessageType::REQUEST_STATUS:
            if (!requireEmptyPayload(frame)) {
                sendInvalidLengthAck(frame);
                return;
            }
            if (!acceptInformationalSequence(frame)) return;
            sendStatus();
            break;
        default:
            ++g_decoder.mutableCounters().unknown_message_types;
            break;
    }
}

void processSerialInput() {
    Frame frame;
    size_t budget = 1024u;
    while (budget-- > 0u && g_pi_serial.available() > 0) {
        const int value = g_pi_serial.read();
        if (value < 0) break;
        const DecodeResult result = g_decoder.consume(
            static_cast<uint8_t>(value), frame);
        if (result == DecodeResult::FRAME_READY) {
            const uint64_t receive_us = monotonicUs();
            handleFrame(frame, receive_us);
        }
    }
}

bool configureTaskWatchdog() {
    esp_err_t configured = ESP_OK;
#if ESP_IDF_VERSION_MAJOR >= 5
    esp_task_wdt_config_t config{};
    config.timeout_ms = kMotorTaskWatchdogTimeoutMs;
    config.idle_core_mask = 0u;
    config.trigger_panic = true;
    configured = esp_task_wdt_init(&config);
    if (configured == ESP_ERR_INVALID_STATE) {
        configured = esp_task_wdt_reconfigure(&config);
    }
#else
    configured = esp_task_wdt_init(
        (kMotorTaskWatchdogTimeoutMs + 999u) / 1000u, true);
#endif
    if (configured != ESP_OK) return false;
    return esp_task_wdt_add(nullptr) == ESP_OK;
}

void motorTaskEntry(void*) {
    if (!configureTaskWatchdog()) {
        // Motion without an independent task watchdog is not an allowed
        // degraded mode. Emit zero, then reboot into the same zero-first BOOT
        // sequence instead of continuing with an unprotected motor owner.
        g_motor.hardZero(millis());
        delay(20u);
        esp_restart();
        vTaskDelete(nullptr);
    }
    uint32_t observed_hard_zero_generation = 0u;
    for (;;) {
        const uint32_t now_ms = millis();
        g_motor.pollFeedback();
        const MotorTelemetry telemetry = g_motor.telemetry(now_ms);

        SafetySnapshot safety;
        taskENTER_CRITICAL(&g_safety_mux);
        g_safety.tick(now_ms, motorHealth(telemetry));
        safety = g_safety.snapshot(now_ms);
        if (g_safety.relayAllowedMask() == 0u) relaysOffLocked();
        if (safety.hard_zero_generation !=
            observed_hard_zero_generation) {
            g_motor.hardZero(now_ms);
            observed_hard_zero_generation = safety.hard_zero_generation;
            g_motor_zero_ack_generation = observed_hard_zero_generation;
        } else {
            // Final owner-side clamp. SafetySnapshot already exposes zero in
            // every non-ARMED state, but the UART owner independently repeats
            // the authority check before its sole non-zero write path.
            const bool motion_allowed =
                safety.handshake && safety.state == EspState::ARMED &&
                safety.fault == FaultCode::NONE && !safety.estop_latched;
            g_motor.setSafeOutput(
                motion_allowed ? safety.applied_left_percent : 0,
                motion_allowed ? safety.applied_right_percent : 0,
                now_ms);
        }
        taskEXIT_CRITICAL(&g_safety_mux);
        esp_task_wdt_reset();
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2u));
    }
}

uint32_t loadAndIncrementBootCounter() {
    Preferences preferences;
    if (!preferences.begin("pi_bridge", false)) return 0u;
    uint32_t count = preferences.getUInt("boot_count", 0u);
    ++count;
    if (count == 0u) count = 1u;
    preferences.putUInt("boot_count", count);
    preferences.end();
    return count;
}

void pollSensorsAndRetry(uint32_t now_ms) {
    const bool may_retry_blocking_init =
        !safetySnapshot(now_ms).handshake;
    if (g_imu_started) {
        g_imu.poll();
        if (!g_imu.available()) g_imu_started = false;
    } else if (may_retry_blocking_init &&
               now_ms - g_last_imu_retry_ms >= kSensorRetryPeriodMs) {
        g_last_imu_retry_ms = now_ms;
        g_imu_started = g_imu.begin(PIN_IMU_SDA, PIN_IMU_SCL);
    }

    if (!g_gnss_started && may_retry_blocking_init &&
        now_ms - g_last_gnss_retry_ms >= kSensorRetryPeriodMs) {
        g_last_gnss_retry_ms = now_ms;
        g_gnss_started = g_gnss.begin(g_gnss_serial);
    }
}

void sendPeriodicTelemetry(uint32_t now_ms) {
    const SafetySnapshot safety = safetySnapshot(now_ms);
    if (!safety.handshake) return;

    sendSensorTelemetry();
    sendPendingTransitions();
    if (now_ms - g_last_status_ms >= kStatusPeriodMs) {
        g_last_status_ms = now_ms;
        sendStatus();
    }
    if (now_ms - g_last_motor_telemetry_ms >=
        kMotorTelemetryPeriodMs) {
        g_last_motor_telemetry_ms = now_ms;
        sendMotorTelemetry();
    }
    if (now_ms - g_last_power_ms >= kPowerPeriodMs) {
        g_last_power_ms = now_ms;
        sendPowerStatus();
    }
    if (now_ms - g_last_relay_ms >= kRelayPeriodMs) {
        g_last_relay_ms = now_ms;
        sendRelayStatus();
    }
    if (now_ms - g_last_diagnostics_ms >= kDiagnosticsPeriodMs) {
        g_last_diagnostics_ms = now_ms;
        sendDiagnostics();
    }
}

}  // namespace

void setup() {
    // BOOT physical invariants are established before any potentially
    // blocking sensor initialization.
    pinMode(PIN_RELAY_ATTACH, OUTPUT);
    pinMode(PIN_RELAY_MOUNT, OUTPUT);
    digitalWrite(PIN_RELAY_ATTACH, LOW);
    digitalWrite(PIN_RELAY_MOUNT, LOW);
    g_motor.begin(g_motor_serial);

    g_pi_serial.setRxBufferSize(4096u);
    g_pi_serial.begin(PI_BRIDGE_SERIAL_BAUD);
    // ROM boot output and any host bytes sent during reset are outside the
    // protocol. A delimiter gives both peers an explicit clean COBS boundary.
    g_pi_serial.write(static_cast<uint8_t>(0u));
    while (g_pi_serial.available() > 0) g_pi_serial.read();

    g_reset_reason = static_cast<uint8_t>(esp_reset_reason());
    g_boot_counter = loadAndIncrementBootCounter();

    // Keep emitting physical zero and run the independent task watchdog even
    // while BNO085/F9P initialization is blocking in BOOT.
    const BaseType_t motor_task_created = xTaskCreatePinnedToCore(
        motorTaskEntry, "piMotorOwner", 4096u, nullptr, 4u,
        &g_motor_task, 0u);
    if (motor_task_created != pdPASS) {
        g_motor.hardZero(millis());
        delay(100u);
        esp_restart();
    }

    g_imu_started = g_imu.begin(PIN_IMU_SDA, PIN_IMU_SCL);
    g_gnss_started = g_gnss.begin(g_gnss_serial);
    const uint32_t now_ms = millis();
    g_last_imu_retry_ms = now_ms;
    g_last_gnss_retry_ms = now_ms;

    taskENTER_CRITICAL(&g_safety_mux);
    g_safety.completeBoot(now_ms);
    taskEXIT_CRITICAL(&g_safety_mux);
    wakeMotorTask();
}

void loop() {
    const uint32_t now_ms = millis();
    processSerialInput();
    pollSensorsAndRetry(now_ms);
    enforceRelaySafety(now_ms);
    sendPeriodicTelemetry(now_ms);
    delay(1u);
}
