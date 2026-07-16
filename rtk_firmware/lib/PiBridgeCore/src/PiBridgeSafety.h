#pragma once

#include <stdint.h>

#include "PiBridgeProtocol.h"

namespace pibridge {

// Conservative boot defaults. Values marked TODO_CALIBRATE are placeholders
// inherited from the field-proven rover configuration, not final geometry or
// sign calibration. SET_LIMITS remains bounded by compiled hard ceilings and
// is accepted only while DISARMED.
struct MotionLimits {
    int32_t max_forward_mm_s = 150;
    int32_t max_reverse_mm_s = 150;
    int32_t max_angular_mrad_s = 600;
    uint32_t linear_accel_mm_s2 = 200;   // TODO_CALIBRATE
    uint32_t angular_accel_mrad_s2 = 800;  // TODO_CALIBRATE
    int16_t left_scale_milli = 1000;     // TODO_CALIBRATE
    int16_t right_scale_milli = 1000;    // TODO_CALIBRATE
    int16_t linear_scale_milli = 1000;   // TODO_CALIBRATE
    int16_t angular_scale_milli = 1000;  // TODO_CALIBRATE
    int8_t left_sign = 1;                // TODO_CALIBRATE
    int8_t right_sign = 1;               // TODO_CALIBRATE
    bool swap_left_right = false;        // TODO_CALIBRATE
    uint8_t motor_deadband_percent = 5;  // TODO_CALIBRATE
    uint8_t max_motor_percent = 12;      // TODO_CALIBRATE
    uint16_t track_width_mm = 380;        // TODO_CALIBRATE
    uint16_t command_timeout_ms = 300;
};

struct MotorHealth {
    bool feedback_available = false;
    bool feedback_fresh = false;
    bool feedback_at_zero = false;
    bool controller_fault = false;
};

struct SafetyReply {
    AckResult result = AckResult::OK;
    AckDetail detail = AckDetail::NONE;
};

struct SafetySnapshot {
    EspState state = EspState::BOOT;
    FaultCode fault = FaultCode::NONE;
    bool handshake = false;
    bool estop_latched = false;
    int16_t applied_left_percent = 0;
    int16_t applied_right_percent = 0;
    uint8_t control_mode = 0;
    uint32_t last_cmd_age_ms = 0xFFFFFFFFu;
    uint32_t last_heartbeat_age_ms = 0xFFFFFFFFu;
    uint32_t watchdog_trips = 0u;
    uint32_t hard_zero_generation = 0u;
};

struct FaultTransition {
    FaultCode fault = FaultCode::NONE;
    EspState from = EspState::BOOT;
    EspState to = EspState::BOOT;
    uint32_t occurrence_count = 0u;
};

struct EstopTransition {
    bool latched = false;
    uint8_t source = 0u;
    uint32_t occurrence_count = 0u;
};

class SafetyMachine {
public:
    static constexpr uint32_t kHeartbeatTimeoutMs = 1500u;
    static constexpr uint32_t kMotorFeedbackTimeoutMs = 500u;

    SafetyMachine();

    void completeBoot(uint32_t now_ms);
    void acceptHello(uint32_t now_ms);
    void noteHeartbeat(uint32_t now_ms);
    void connectionLost();

    SafetyReply arm(const ArmPayload& command, const MotorHealth& motor,
                    uint32_t now_ms);
    SafetyReply disarm();
    SafetyReply stop();
    SafetyReply estop(uint8_t source);
    SafetyReply resetEstop(const MotorHealth& motor);
    SafetyReply resetFault(const MotorHealth& motor);
    SafetyReply commandVelocity(uint32_t sequence,
                                const CmdVelPayload& command,
                                uint32_t now_ms);
    SafetyReply setLimits(const SetLimitsPayload& command);

    void tick(uint32_t now_ms, const MotorHealth& motor);
    SafetySnapshot snapshot(uint32_t now_ms) const;
    const MotionLimits& limits() const { return _limits; }
    uint8_t relayAllowedMask() const;

    bool consumeFaultTransition(FaultTransition& output);
    bool consumeEstopTransition(EstopTransition& output);

private:
    static int32_t clampI32(int32_t value, int32_t low, int32_t high);
    static int32_t absI32(int32_t value);
    static int16_t roundedPercent(int32_t milli_percent);
    static uint32_t age(uint32_t now_ms, uint32_t then_ms);

    void hardZero();
    void enterFault(FaultCode fault);
    void calculateTarget();
    void updateRamp(uint32_t now_ms);
    bool validLimits(const SetLimitsPayload& command) const;

    MotionLimits _limits{};
    EspState _state = EspState::BOOT;
    FaultCode _fault = FaultCode::NONE;
    bool _handshake = false;
    bool _estop_latched = false;
    uint8_t _control_mode = 0u;

    int32_t _target_linear_mm_s = 0;
    int32_t _target_angular_mrad_s = 0;
    int16_t _target_left_percent = 0;
    int16_t _target_right_percent = 0;
    int32_t _applied_left_milli_percent = 0;
    int32_t _applied_right_milli_percent = 0;
    uint32_t _last_ramp_ms = 0u;
    uint32_t _ramp_remainder = 0u;

    uint32_t _last_cmd_ms = 0u;
    uint32_t _armed_ms = 0u;
    uint32_t _active_command_timeout_ms = 300u;
    uint32_t _last_heartbeat_ms = 0u;
    uint32_t _hard_zero_generation = 1u;
    uint32_t _watchdog_trips = 0u;
    uint32_t _fault_occurrences = 0u;
    uint32_t _estop_occurrences = 0u;

    bool _have_cmd_sequence = false;
    bool _have_valid_cmd = false;
    uint32_t _last_cmd_sequence = 0u;
    uint32_t _last_arm_nonce = 0u;

    bool _fault_transition_pending = false;
    FaultTransition _fault_transition{};
    bool _estop_transition_pending = false;
    EstopTransition _estop_transition{};
};

}  // namespace pibridge
