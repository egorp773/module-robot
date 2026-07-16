#pragma once

#include <float.h>
#include <stddef.h>
#include <stdint.h>

#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__) && \
    __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "PiBridge packed payload ABI requires a little-endian target"
#endif

namespace pibridge {

constexpr uint8_t kProtocolVersion = 1u;
constexpr size_t kHeaderSize = 16u;
constexpr size_t kCrcSize = 2u;
constexpr size_t kMaxPayloadSize = 192u;
constexpr size_t kMaxRawFrameSize = kHeaderSize + kMaxPayloadSize + kCrcSize;
constexpr size_t kMaxEncodedFrameSize =
    kMaxRawFrameSize + (kMaxRawFrameSize / 254u) + 2u;

enum class MessageType : uint8_t {
    HELLO = 0x01,
    TIME_SYNC_REQUEST = 0x02,
    HEARTBEAT = 0x03,
    ARM = 0x04,
    DISARM = 0x05,
    CMD_VEL = 0x06,
    STOP = 0x07,
    ESTOP = 0x08,
    RESET_ESTOP = 0x09,
    RESET_FAULT = 0x0A,
    RELAY_COMMAND = 0x0B,
    SET_LIMITS = 0x0C,
    REQUEST_STATUS = 0x0D,

    HELLO_ACK = 0x81,
    TIME_SYNC_RESPONSE = 0x82,
    STATUS = 0x83,
    IMU = 0x84,
    GNSS = 0x85,
    MOTOR_FEEDBACK = 0x86,
    POWER_STATUS = 0x87,
    RELAY_STATUS = 0x88,
    FAULT_EVENT = 0x89,
    ESTOP_EVENT = 0x8A,
    COMMAND_ACK = 0x8B,
    DIAGNOSTICS = 0x8C,
};

enum class EspState : uint8_t {
    BOOT = 0,
    DISCONNECTED = 1,
    DISARMED = 2,
    ARMED = 3,
    FAULT = 4,
    ESTOP = 5,
};

enum class FaultCode : uint16_t {
    NONE = 0,
    CMD_VEL_TIMEOUT = 1,
    MOTOR_FEEDBACK_UNAVAILABLE = 2,
    MOTOR_FEEDBACK_LOST = 3,
    MOTOR_CONTROLLER_FAULT = 4,
    INVALID_MOTOR_COMMAND = 5,
    INTERNAL_WATCHDOG = 6,
};

enum class AckResult : uint8_t {
    OK = 0,
    REJECTED_STATE = 1,
    INVALID_PAYLOAD = 2,
    UNSUPPORTED = 3,
    STALE_SEQUENCE = 4,
    PRECONDITION_FAILED = 5,
    ESTOP_LATCHED = 6,
    FAULT_LATCHED = 7,
    PROTOCOL_MISMATCH = 8,
};

enum class AckDetail : uint16_t {
    NONE = 0,
    HANDSHAKE_REQUIRED = 1,
    MOTOR_FEEDBACK_REQUIRED = 2,
    MOTOR_NOT_ZERO = 3,
    TARGET_NOT_ZERO = 4,
    BAD_NONCE = 5,
    BAD_CONTROL_MODE = 6,
    BAD_LIMITS = 7,
    RELAY_STATE_FORBIDDEN = 8,
    PAYLOAD_LENGTH = 9,
    DUPLICATE_SEQUENCE = 10,
    OUT_OF_ORDER_SEQUENCE = 11,
};

enum class ControlMode : uint8_t {
    // Arduino's ESP32 GPIO headers define DISABLED as a macro. Keep the wire
    // value zero while using a collision-free C++ identifier.
    INACTIVE = 0,
    MANUAL = 1,
    AUTO = 2,
};

enum Capability : uint32_t {
    CAP_IMU = 1u << 0,
    CAP_GNSS = 1u << 1,
    CAP_MOTOR_FEEDBACK = 1u << 2,
    CAP_RELAYS = 1u << 3,
    CAP_TIME_SYNC = 1u << 4,
    CAP_SET_LIMITS = 1u << 5,
};

constexpr uint32_t kSupportedCapabilities =
    CAP_IMU | CAP_GNSS | CAP_MOTOR_FEEDBACK | CAP_RELAYS |
    CAP_TIME_SYNC | CAP_SET_LIMITS;

enum PowerStatusFlag : uint32_t {
    POWER_FLAG_VALID = 1u << 0,
    POWER_FLAG_IMU_UNAVAILABLE = 1u << 8,
    POWER_FLAG_GNSS_UNAVAILABLE = 1u << 9,
    POWER_FLAG_MOTOR_FEEDBACK_STALE = 1u << 10,
    POWER_FLAG_MOTOR_CONTROLLER_FAULT = 1u << 11,
    POWER_FLAG_GNSS_CHECKSUM_ERROR_SEEN = 1u << 12,
    POWER_FLAG_GNSS_OVERSIZED_PACKET_SEEN = 1u << 13,
    POWER_FLAG_MOTOR_INVALID_FRAME_SEEN = 1u << 14,
};

#pragma pack(push, 1)

struct FrameHeader {
    uint8_t protocol_version;
    uint8_t message_type;
    uint16_t payload_length;
    uint32_t sequence;
    uint64_t sender_monotonic_us;
};

struct HelloPayload {
    uint8_t protocol_version;
    uint8_t reserved[3];
    uint32_t requested_capabilities;
    char client_name[32];
    char client_build_id[32];
};

struct TimeSyncRequestPayload {
    uint64_t pi_send_monotonic_us;
};

struct HeartbeatPayload {
    uint64_t pi_monotonic_us;
    uint8_t pi_state;
    uint8_t reserved[7];
};

struct ArmPayload {
    uint32_t arm_nonce;
    uint8_t requested_mode;
    uint8_t reserved[3];
};

struct CmdVelPayload {
    int32_t linear_mm_s;
    int32_t angular_mrad_s;
    uint32_t command_timeout_ms;
    uint8_t control_mode;
};

struct RelayCommandPayload {
    uint8_t relay_mask;
    uint8_t relay_values;
};

struct SetLimitsPayload {
    int32_t max_forward_mm_s;
    int32_t max_reverse_mm_s;
    int32_t max_angular_mrad_s;
    uint32_t linear_accel_mm_s2;
    uint32_t angular_accel_mrad_s2;
    int16_t left_scale_milli;
    int16_t right_scale_milli;
    int16_t linear_scale_milli;
    int16_t angular_scale_milli;
    int8_t left_sign;
    int8_t right_sign;
    uint8_t swap_left_right;
    uint8_t motor_deadband_percent;
    uint8_t max_motor_percent;
    uint16_t track_width_mm;
    uint16_t command_timeout_ms;
};

struct HelloAckPayload {
    uint8_t protocol_version;
    uint8_t accepted;
    uint8_t state;
    uint8_t reserved;
    uint32_t supported_capabilities;
    uint16_t max_payload;
    uint8_t reserved2[2];
    char firmware_build_id[32];
};

struct TimeSyncResponsePayload {
    uint64_t pi_send_monotonic_us;
    uint64_t esp_receive_monotonic_us;
    uint64_t esp_send_monotonic_us;
};

struct StatusPayload {
    uint8_t state;
    uint8_t armed;
    uint8_t estop;
    uint8_t reset_reason;
    uint16_t fault_code;
    uint8_t reserved[2];
    uint32_t last_cmd_vel_age_ms;
    uint32_t last_pi_heartbeat_age_ms;
    uint32_t last_gnss_age_ms;
    uint32_t last_imu_age_ms;
    uint32_t last_motor_feedback_age_ms;
    int16_t applied_left_command;
    int16_t applied_right_command;
    int16_t uart_speed;
    int16_t uart_steer;
    uint32_t watchdog_trips;
    uint32_t boot_counter;
    uint32_t uptime_ms;
};

struct ImuPayload {
    uint64_t sensor_monotonic_us;
    float quaternion_x;
    float quaternion_y;
    float quaternion_z;
    float quaternion_w;
    float angular_velocity_x;
    float angular_velocity_y;
    float angular_velocity_z;
    float linear_acceleration_x;
    float linear_acceleration_y;
    float linear_acceleration_z;
    uint8_t calibration_status;
    uint8_t reserved[3];
    float accuracy_rad;
    uint32_t sequence;
};

struct GnssPayload {
    uint64_t sensor_monotonic_us;
    int32_t latitude_e7;
    int32_t longitude_e7;
    int32_t altitude_mm;
    uint32_t h_acc_mm;
    uint32_t v_acc_mm;
    int32_t ground_speed_mm_s;
    int32_t motion_heading_deg_e5;
    uint8_t fix_type;
    uint8_t carrier_solution;
    uint8_t satellite_count;
    uint8_t reserved;
    uint32_t rtcm_age_ms;
    uint32_t sequence;
};

struct MotorFeedbackPayload {
    uint64_t sensor_monotonic_us;
    int16_t left_feedback;
    int16_t right_feedback;
    int16_t battery_centivolts;
    int16_t board_temperature_decic;
    uint16_t motor_controller_fault;
    uint16_t reserved;
    uint32_t uart_valid_frame_count;
    uint32_t uart_invalid_frame_count;
};

struct PowerStatusPayload {
    uint64_t sensor_monotonic_us;
    float battery_voltage;
    uint8_t battery_percent;
    uint8_t reserved[3];
    uint32_t flags;  // Bitmask of PowerStatusFlag.
};

struct RelayStatusPayload {
    uint64_t sensor_monotonic_us;
    uint8_t attachment_enabled;
    uint8_t mount_enabled;
    uint8_t allowed_mask;
    uint8_t state;
    uint32_t sequence;
};

struct FaultEventPayload {
    uint64_t sensor_monotonic_us;
    uint16_t fault_code;
    uint8_t from_state;
    uint8_t to_state;
    uint32_t occurrence_count;
};

struct EstopEventPayload {
    uint64_t sensor_monotonic_us;
    uint8_t latched;
    uint8_t source;
    uint16_t reserved;
    uint32_t occurrence_count;
};

struct CommandAckPayload {
    uint32_t command_sequence;
    uint8_t command_type;
    uint8_t result;
    uint8_t state;
    uint8_t reserved;
    uint16_t detail_code;
    uint16_t reserved2;
};

struct DiagnosticsPayload {
    uint64_t sensor_monotonic_us;
    uint32_t rx_frames;
    uint32_t tx_frames;
    uint32_t crc_errors;
    uint32_t cobs_errors;
    uint32_t length_errors;
    uint32_t oversized_frames;
    uint32_t unknown_message_types;
    uint32_t duplicate_sequences;
    uint32_t out_of_order_sequences;
    uint32_t rx_overflows;
    uint32_t tx_errors;
};

#pragma pack(pop)

static_assert(sizeof(FrameHeader) == 16u, "wire header size changed");
static_assert(sizeof(float) == 4u && FLT_RADIX == 2 &&
                  FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128,
              "wire protocol requires IEEE-754 float32");
static_assert(sizeof(HelloPayload) == 72u, "HELLO wire size changed");
static_assert(sizeof(TimeSyncRequestPayload) == 8u,
              "TIME_SYNC_REQUEST wire size changed");
static_assert(sizeof(HeartbeatPayload) == 16u,
              "HEARTBEAT wire size changed");
static_assert(sizeof(ArmPayload) == 8u, "ARM wire size changed");
static_assert(sizeof(CmdVelPayload) == 13u, "CMD_VEL wire size changed");
static_assert(sizeof(RelayCommandPayload) == 2u,
              "RELAY_COMMAND wire size changed");
static_assert(sizeof(SetLimitsPayload) == 37u, "SET_LIMITS wire size changed");
static_assert(sizeof(HelloAckPayload) == 44u, "HELLO_ACK wire size changed");
static_assert(sizeof(TimeSyncResponsePayload) == 24u,
              "TIME_SYNC_RESPONSE wire size changed");
static_assert(sizeof(StatusPayload) == 48u, "STATUS wire size changed");
static_assert(sizeof(ImuPayload) == 60u, "IMU wire size changed");
static_assert(sizeof(GnssPayload) == 48u, "GNSS wire size changed");
static_assert(sizeof(MotorFeedbackPayload) == 28u, "MOTOR wire size changed");
static_assert(sizeof(PowerStatusPayload) == 20u, "POWER wire size changed");
static_assert(sizeof(RelayStatusPayload) == 16u, "RELAY wire size changed");
static_assert(sizeof(FaultEventPayload) == 16u, "FAULT wire size changed");
static_assert(sizeof(EstopEventPayload) == 16u, "ESTOP wire size changed");
static_assert(sizeof(CommandAckPayload) == 12u, "ACK wire size changed");
static_assert(sizeof(DiagnosticsPayload) == 52u, "DIAGNOSTICS wire size changed");

struct Frame {
    FrameHeader header{};
    uint8_t payload[kMaxPayloadSize]{};
};

struct ProtocolCounters {
    uint32_t rx_frames = 0u;
    uint32_t tx_frames = 0u;
    uint32_t crc_errors = 0u;
    uint32_t cobs_errors = 0u;
    uint32_t length_errors = 0u;
    uint32_t oversized_frames = 0u;
    uint32_t unknown_message_types = 0u;
    uint32_t duplicate_sequences = 0u;
    uint32_t out_of_order_sequences = 0u;
    uint32_t rx_overflows = 0u;
    uint32_t tx_errors = 0u;
};

uint16_t crc16CcittFalse(const uint8_t* data, size_t length);
size_t cobsEncode(const uint8_t* input, size_t input_length,
                  uint8_t* output, size_t output_capacity);
size_t cobsDecode(const uint8_t* input, size_t input_length,
                  uint8_t* output, size_t output_capacity);

size_t encodeFrame(const FrameHeader& header, const void* payload,
                   uint8_t* output, size_t output_capacity);

enum class DecodeResult : uint8_t {
    NONE = 0,
    FRAME_READY,
    EMPTY_DELIMITER,
    COBS_ERROR,
    LENGTH_ERROR,
    CRC_ERROR,
    OVERSIZED,
};

class FrameStreamDecoder {
public:
    DecodeResult consume(uint8_t byte, Frame& output);
    void reset();
    const ProtocolCounters& counters() const { return _counters; }
    ProtocolCounters& mutableCounters() { return _counters; }

private:
    uint8_t _encoded[kMaxEncodedFrameSize]{};
    size_t _encoded_length = 0u;
    bool _discard_until_delimiter = false;
    ProtocolCounters _counters{};
};

bool isKnownInboundMessageType(uint8_t message_type);
bool sequenceIsNewer(uint32_t candidate, uint32_t previous);

}  // namespace pibridge
