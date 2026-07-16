#include "PiBridgeSafety.h"

#include <limits.h>

namespace pibridge {
namespace {

constexpr int32_t kHardMaxForwardMmS = 150;
constexpr int32_t kHardMaxReverseMmS = 150;
constexpr int32_t kHardMaxAngularMradS = 600;
constexpr uint32_t kHardMaxCommandTimeoutMs = 300u;
constexpr uint8_t kHardMaxMotorPercent = 12u;  // TODO_CALIBRATE
constexpr uint32_t kHardMaxLinearAccelMmS2 = 1000u;
constexpr uint32_t kHardMaxAngularAccelMradS2 = 4000u;
constexpr int16_t kHardMaxScaleMilli = 2000;
constexpr int16_t kHardMinScaleMilli = 1;
constexpr uint16_t kMinTrackWidthMm = 100u;
constexpr uint16_t kMaxTrackWidthMm = 2000u;

SafetyReply reply(AckResult result,
                  AckDetail detail = AckDetail::NONE) {
    SafetyReply out;
    out.result = result;
    out.detail = detail;
    return out;
}

}  // namespace

SafetyMachine::SafetyMachine() = default;

int32_t SafetyMachine::clampI32(int32_t value, int32_t low, int32_t high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

int32_t SafetyMachine::absI32(int32_t value) {
    if (value == INT32_MIN) return INT32_MAX;
    return value < 0 ? -value : value;
}

int16_t SafetyMachine::roundedPercent(int32_t milli_percent) {
    if (milli_percent >= 0) {
        return static_cast<int16_t>((milli_percent + 500) / 1000);
    }
    return static_cast<int16_t>((milli_percent - 500) / 1000);
}

uint32_t SafetyMachine::age(uint32_t now_ms, uint32_t then_ms) {
    // Unsigned subtraction is deliberately wrap-safe. Callers use explicit
    // validity flags instead of reserving timestamp zero, which is a valid
    // millis() value after the 49-day rollover.
    return now_ms - then_ms;
}

void SafetyMachine::hardZero() {
    _target_linear_mm_s = 0;
    _target_angular_mrad_s = 0;
    _target_left_percent = 0;
    _target_right_percent = 0;
    _applied_left_milli_percent = 0;
    _applied_right_milli_percent = 0;
    _last_ramp_ms = 0u;
    _ramp_remainder = 0u;
    ++_hard_zero_generation;
    if (_hard_zero_generation == 0u) ++_hard_zero_generation;
}

void SafetyMachine::completeBoot(uint32_t now_ms) {
    (void)now_ms;
    _handshake = false;
    _estop_latched = false;
    _fault = FaultCode::NONE;
    _state = EspState::DISCONNECTED;
    _control_mode = static_cast<uint8_t>(ControlMode::INACTIVE);
    _have_cmd_sequence = false;
    _have_valid_cmd = false;
    _last_cmd_ms = 0u;
    _armed_ms = 0u;
    _last_heartbeat_ms = 0u;
    hardZero();
}

void SafetyMachine::acceptHello(uint32_t now_ms) {
    _handshake = true;
    _last_heartbeat_ms = now_ms;
    _last_cmd_ms = 0u;
    _armed_ms = 0u;
    _have_cmd_sequence = false;
    _have_valid_cmd = false;
    // Preserve the last successful ARM nonce across transport reconnects so
    // replaying the previous session's ARM cannot regain authority. A full
    // ESP32 reboot still starts from the constructor's zero nonce and remains
    // DISCONNECTED/DISARMED until a new explicit ARM.
    _control_mode = static_cast<uint8_t>(ControlMode::INACTIVE);
    hardZero();
    if (!_estop_latched && _fault == FaultCode::NONE) {
        _state = EspState::DISARMED;
    }
}

void SafetyMachine::noteHeartbeat(uint32_t now_ms) {
    if (_handshake) _last_heartbeat_ms = now_ms;
}

void SafetyMachine::connectionLost() {
    _handshake = false;
    _last_heartbeat_ms = 0u;
    _have_cmd_sequence = false;
    _have_valid_cmd = false;
    _armed_ms = 0u;
    _control_mode = static_cast<uint8_t>(ControlMode::INACTIVE);
    hardZero();
    if (!_estop_latched && _fault == FaultCode::NONE) {
        _state = EspState::DISCONNECTED;
    }
}

SafetyReply SafetyMachine::arm(const ArmPayload& command,
                               const MotorHealth& motor,
                               uint32_t now_ms) {
    const bool target_was_zero = _target_linear_mm_s == 0 &&
                                 _target_angular_mrad_s == 0 &&
                                 _applied_left_milli_percent == 0 &&
                                 _applied_right_milli_percent == 0;
    hardZero();
    if (!_handshake) {
        return reply(AckResult::PRECONDITION_FAILED,
                     AckDetail::HANDSHAKE_REQUIRED);
    }
    if (_estop_latched || _state == EspState::ESTOP) {
        return reply(AckResult::ESTOP_LATCHED);
    }
    if (_fault != FaultCode::NONE || _state == EspState::FAULT) {
        return reply(AckResult::FAULT_LATCHED);
    }
    if (_state != EspState::DISARMED) {
        return reply(AckResult::REJECTED_STATE);
    }
    if (command.arm_nonce == 0u || command.arm_nonce == _last_arm_nonce) {
        return reply(AckResult::PRECONDITION_FAILED, AckDetail::BAD_NONCE);
    }
    if (command.requested_mode < static_cast<uint8_t>(ControlMode::MANUAL) ||
        command.requested_mode > static_cast<uint8_t>(ControlMode::AUTO)) {
        return reply(AckResult::INVALID_PAYLOAD,
                     AckDetail::BAD_CONTROL_MODE);
    }
    if (!motor.feedback_available || !motor.feedback_fresh) {
        return reply(AckResult::PRECONDITION_FAILED,
                     AckDetail::MOTOR_FEEDBACK_REQUIRED);
    }
    if (motor.controller_fault) {
        return reply(AckResult::PRECONDITION_FAILED,
                     AckDetail::MOTOR_FEEDBACK_REQUIRED);
    }
    if (!motor.feedback_at_zero ||
        _applied_left_milli_percent != 0 ||
        _applied_right_milli_percent != 0) {
        return reply(AckResult::PRECONDITION_FAILED,
                     AckDetail::MOTOR_NOT_ZERO);
    }
    if (!target_was_zero) {
        return reply(AckResult::PRECONDITION_FAILED,
                     AckDetail::TARGET_NOT_ZERO);
    }

    _last_arm_nonce = command.arm_nonce;
    _control_mode = command.requested_mode;
    _active_command_timeout_ms = _limits.command_timeout_ms;
    _last_cmd_ms = 0u;
    _armed_ms = now_ms;
    // ARM opens a new motion-authority epoch. The transport-level sequence
    // gate in pi_bridge.cpp still rejects replay from the current HELLO
    // session; resetting this CMD_VEL-local tracker prevents an ancient
    // command from a much older arm epoch poisoning the first fresh command
    // after more than half of the uint32 sequence space has elapsed.
    _have_cmd_sequence = false;
    _last_cmd_sequence = 0u;
    _have_valid_cmd = false;
    _last_ramp_ms = now_ms;
    _state = EspState::ARMED;
    return reply(AckResult::OK);
}

SafetyReply SafetyMachine::disarm() {
    hardZero();
    _have_valid_cmd = false;
    _last_cmd_ms = 0u;
    _armed_ms = 0u;
    _control_mode = static_cast<uint8_t>(ControlMode::INACTIVE);
    if (!_handshake) {
        return reply(AckResult::PRECONDITION_FAILED,
                     AckDetail::HANDSHAKE_REQUIRED);
    }
    if (_state == EspState::ARMED || _state == EspState::DISARMED) {
        _state = EspState::DISARMED;
    }
    return reply(AckResult::OK);
}

SafetyReply SafetyMachine::stop() {
    hardZero();
    return reply(AckResult::OK);
}

SafetyReply SafetyMachine::estop(uint8_t source) {
    if (_estop_latched && _state == EspState::ESTOP) {
        hardZero();
        return reply(AckResult::OK);
    }
    hardZero();
    _estop_latched = true;
    _control_mode = static_cast<uint8_t>(ControlMode::INACTIVE);
    _state = EspState::ESTOP;
    ++_estop_occurrences;
    _estop_transition.latched = true;
    _estop_transition.source = source;
    _estop_transition.occurrence_count = _estop_occurrences;
    _estop_transition_pending = true;
    return reply(AckResult::OK);
}

SafetyReply SafetyMachine::resetEstop(const MotorHealth& motor) {
    hardZero();
    if (!_handshake) {
        return reply(AckResult::PRECONDITION_FAILED,
                     AckDetail::HANDSHAKE_REQUIRED);
    }
    if (!_estop_latched || _state != EspState::ESTOP) {
        return reply(AckResult::REJECTED_STATE);
    }
    if (_fault != FaultCode::NONE) return reply(AckResult::FAULT_LATCHED);
    if (!motor.feedback_available || !motor.feedback_fresh) {
        return reply(AckResult::PRECONDITION_FAILED,
                     AckDetail::MOTOR_FEEDBACK_REQUIRED);
    }
    if (!motor.feedback_at_zero || motor.controller_fault) {
        return reply(AckResult::PRECONDITION_FAILED,
                     AckDetail::MOTOR_NOT_ZERO);
    }
    _estop_latched = false;
    _state = EspState::DISARMED;
    _control_mode = static_cast<uint8_t>(ControlMode::INACTIVE);
    _estop_transition.latched = false;
    _estop_transition.source = 0u;
    _estop_transition.occurrence_count = _estop_occurrences;
    _estop_transition_pending = true;
    return reply(AckResult::OK);
}

SafetyReply SafetyMachine::resetFault(const MotorHealth& motor) {
    hardZero();
    if (!_handshake) {
        return reply(AckResult::PRECONDITION_FAILED,
                     AckDetail::HANDSHAKE_REQUIRED);
    }
    if (_fault == FaultCode::NONE) return reply(AckResult::REJECTED_STATE);
    if (!motor.feedback_available || !motor.feedback_fresh ||
        motor.controller_fault) {
        return reply(AckResult::PRECONDITION_FAILED,
                     AckDetail::MOTOR_FEEDBACK_REQUIRED);
    }
    if (!motor.feedback_at_zero) {
        return reply(AckResult::PRECONDITION_FAILED,
                     AckDetail::MOTOR_NOT_ZERO);
    }
    _fault = FaultCode::NONE;
    if (!_estop_latched) {
        _state = EspState::DISARMED;
        _control_mode = static_cast<uint8_t>(ControlMode::INACTIVE);
    }
    return reply(AckResult::OK);
}

SafetyReply SafetyMachine::commandVelocity(
    uint32_t sequence, const CmdVelPayload& command, uint32_t now_ms) {
    if (!_handshake) {
        return reply(AckResult::PRECONDITION_FAILED,
                     AckDetail::HANDSHAKE_REQUIRED);
    }
    if (_state != EspState::ARMED) {
        return _estop_latched ? reply(AckResult::ESTOP_LATCHED)
             : (_fault != FaultCode::NONE ? reply(AckResult::FAULT_LATCHED)
                                          : reply(AckResult::REJECTED_STATE));
    }
    if (command.control_mode < static_cast<uint8_t>(ControlMode::MANUAL) ||
        command.control_mode > static_cast<uint8_t>(ControlMode::AUTO) ||
        command.control_mode != _control_mode) {
        return reply(AckResult::INVALID_PAYLOAD,
                     AckDetail::BAD_CONTROL_MODE);
    }
    if (command.command_timeout_ms == 0u) {
        return reply(AckResult::INVALID_PAYLOAD,
                     AckDetail::BAD_LIMITS);
    }
    if (_have_cmd_sequence && !sequenceIsNewer(sequence, _last_cmd_sequence)) {
        return reply(AckResult::STALE_SEQUENCE,
                     sequence == _last_cmd_sequence
                         ? AckDetail::DUPLICATE_SEQUENCE
                         : AckDetail::OUT_OF_ORDER_SEQUENCE);
    }

    _have_cmd_sequence = true;
    _last_cmd_sequence = sequence;
    _target_linear_mm_s = clampI32(
        command.linear_mm_s, -_limits.max_reverse_mm_s,
        _limits.max_forward_mm_s);
    _target_angular_mrad_s = clampI32(
        command.angular_mrad_s, -_limits.max_angular_mrad_s,
        _limits.max_angular_mrad_s);
    _active_command_timeout_ms = command.command_timeout_ms;
    if (_active_command_timeout_ms > _limits.command_timeout_ms) {
        _active_command_timeout_ms = _limits.command_timeout_ms;
    }
    if (_active_command_timeout_ms > kHardMaxCommandTimeoutMs) {
        _active_command_timeout_ms = kHardMaxCommandTimeoutMs;
    }
    _last_cmd_ms = now_ms;
    _have_valid_cmd = true;
    calculateTarget();
    return reply(AckResult::OK);
}

bool SafetyMachine::validLimits(const SetLimitsPayload& command) const {
    return command.max_forward_mm_s > 0 &&
           command.max_forward_mm_s <= kHardMaxForwardMmS &&
           command.max_reverse_mm_s > 0 &&
           command.max_reverse_mm_s <= kHardMaxReverseMmS &&
           command.max_angular_mrad_s > 0 &&
           command.max_angular_mrad_s <= kHardMaxAngularMradS &&
           command.linear_accel_mm_s2 > 0u &&
           command.linear_accel_mm_s2 <= kHardMaxLinearAccelMmS2 &&
           command.angular_accel_mrad_s2 > 0u &&
           command.angular_accel_mrad_s2 <= kHardMaxAngularAccelMradS2 &&
           command.left_scale_milli >= kHardMinScaleMilli &&
           command.left_scale_milli <= kHardMaxScaleMilli &&
           command.right_scale_milli >= kHardMinScaleMilli &&
           command.right_scale_milli <= kHardMaxScaleMilli &&
           command.linear_scale_milli >= kHardMinScaleMilli &&
           command.linear_scale_milli <= kHardMaxScaleMilli &&
           command.angular_scale_milli >= kHardMinScaleMilli &&
           command.angular_scale_milli <= kHardMaxScaleMilli &&
           (command.left_sign == -1 || command.left_sign == 1) &&
           (command.right_sign == -1 || command.right_sign == 1) &&
           command.swap_left_right <= 1u &&
           command.max_motor_percent > 0u &&
           command.max_motor_percent <= kHardMaxMotorPercent &&
           command.motor_deadband_percent <= command.max_motor_percent &&
           command.track_width_mm >= kMinTrackWidthMm &&
           command.track_width_mm <= kMaxTrackWidthMm &&
           command.command_timeout_ms > 0u &&
           command.command_timeout_ms <= kHardMaxCommandTimeoutMs;
}

SafetyReply SafetyMachine::setLimits(const SetLimitsPayload& command) {
    hardZero();
    if (!_handshake) {
        return reply(AckResult::PRECONDITION_FAILED,
                     AckDetail::HANDSHAKE_REQUIRED);
    }
    if (_state != EspState::DISARMED) {
        return reply(AckResult::REJECTED_STATE);
    }
    if (!validLimits(command)) {
        return reply(AckResult::INVALID_PAYLOAD, AckDetail::BAD_LIMITS);
    }
    _limits.max_forward_mm_s = command.max_forward_mm_s;
    _limits.max_reverse_mm_s = command.max_reverse_mm_s;
    _limits.max_angular_mrad_s = command.max_angular_mrad_s;
    _limits.linear_accel_mm_s2 = command.linear_accel_mm_s2;
    _limits.angular_accel_mrad_s2 = command.angular_accel_mrad_s2;
    _limits.left_scale_milli = command.left_scale_milli;
    _limits.right_scale_milli = command.right_scale_milli;
    _limits.linear_scale_milli = command.linear_scale_milli;
    _limits.angular_scale_milli = command.angular_scale_milli;
    _limits.left_sign = command.left_sign;
    _limits.right_sign = command.right_sign;
    _limits.swap_left_right = command.swap_left_right != 0u;
    _limits.motor_deadband_percent = command.motor_deadband_percent;
    _limits.max_motor_percent = command.max_motor_percent;
    _limits.track_width_mm = command.track_width_mm;
    _limits.command_timeout_ms = command.command_timeout_ms;
    return reply(AckResult::OK);
}

void SafetyMachine::calculateTarget() {
    int32_t linear = static_cast<int32_t>(
        (static_cast<int64_t>(_target_linear_mm_s) *
         _limits.linear_scale_milli) / 1000);
    int32_t angular = static_cast<int32_t>(
        (static_cast<int64_t>(_target_angular_mrad_s) *
         _limits.angular_scale_milli) / 1000);

    const int32_t differential = static_cast<int32_t>(
        (static_cast<int64_t>(angular) * _limits.track_width_mm) / 2000);
    int32_t left_mm_s = linear - differential;
    int32_t right_mm_s = linear + differential;
    if (_limits.swap_left_right) {
        const int32_t temporary = left_mm_s;
        left_mm_s = right_mm_s;
        right_mm_s = temporary;
    }

    auto speedToPercent = [&](int32_t speed_mm_s) -> int32_t {
        const int32_t denominator = speed_mm_s >= 0
            ? _limits.max_forward_mm_s : _limits.max_reverse_mm_s;
        return denominator == 0 ? 0 : static_cast<int32_t>(
            (static_cast<int64_t>(speed_mm_s) *
             _limits.max_motor_percent) / denominator);
    };

    int32_t left = speedToPercent(left_mm_s) * _limits.left_sign;
    int32_t right = speedToPercent(right_mm_s) * _limits.right_sign;
    left = static_cast<int32_t>(
        (static_cast<int64_t>(left) * _limits.left_scale_milli) / 1000);
    right = static_cast<int32_t>(
        (static_cast<int64_t>(right) * _limits.right_scale_milli) / 1000);

    left = clampI32(left, -_limits.max_motor_percent,
                    _limits.max_motor_percent);
    right = clampI32(right, -_limits.max_motor_percent,
                     _limits.max_motor_percent);

    auto compensateDeadband = [&](int32_t value) -> int32_t {
        if (value == 0) return 0;
        if (absI32(value) < _limits.motor_deadband_percent) {
            return value < 0 ? -_limits.motor_deadband_percent
                             : _limits.motor_deadband_percent;
        }
        return value;
    };
    _target_left_percent = static_cast<int16_t>(compensateDeadband(left));
    _target_right_percent = static_cast<int16_t>(compensateDeadband(right));
}

void SafetyMachine::updateRamp(uint32_t now_ms) {
    if (_state != EspState::ARMED) {
        hardZero();
        return;
    }
    if (_last_ramp_ms == 0u) {
        _last_ramp_ms = now_ms;
        return;
    }
    const uint32_t dt_ms = now_ms - _last_ramp_ms;
    if (dt_ms == 0u) return;
    _last_ramp_ms = now_ms;

    const uint32_t normalization_speed = static_cast<uint32_t>(
        _limits.max_forward_mm_s > _limits.max_reverse_mm_s
            ? _limits.max_forward_mm_s : _limits.max_reverse_mm_s);
    const uint32_t linear_rate = static_cast<uint32_t>(
        (static_cast<uint64_t>(_limits.linear_accel_mm_s2) *
         _limits.max_motor_percent * 1000u) /
        normalization_speed);
    const uint32_t angular_wheel_accel = static_cast<uint32_t>(
        (static_cast<uint64_t>(_limits.angular_accel_mrad_s2) *
         _limits.track_width_mm) / 2000u);
    const uint32_t angular_rate = static_cast<uint32_t>(
        (static_cast<uint64_t>(angular_wheel_accel) *
         _limits.max_motor_percent * 1000u) /
        normalization_speed);
    // One common wheel-domain slew must honor both independent acceleration
    // ceilings. Choosing the slower rate is conservative for pure linear,
    // pure angular and combined commands; choosing the faster one can violate
    // the other configured ceiling.
    uint32_t rate_milli_percent_s = linear_rate < angular_rate
        ? linear_rate : angular_rate;
    if (rate_milli_percent_s == 0u) rate_milli_percent_s = 1u;
    const uint64_t ramp_numerator =
        static_cast<uint64_t>(rate_milli_percent_s) * dt_ms +
        _ramp_remainder;
    const uint64_t raw_step = ramp_numerator / 1000u;
    // A scheduler stall must never turn a uint64 elapsed-time calculation
    // into an overflowing int32 slew step. Two full-scale spans are enough to
    // reach any allowed target, including +max to -max reversal.
    constexpr int32_t kLargestUsefulStep =
        static_cast<int32_t>(kHardMaxMotorPercent) * 2 * 1000;
    const int32_t max_step = raw_step >
        static_cast<uint64_t>(kLargestUsefulStep)
            ? kLargestUsefulStep : static_cast<int32_t>(raw_step);
    _ramp_remainder = static_cast<uint32_t>(ramp_numerator % 1000u);
    if (max_step == 0) return;

    auto slew = [&](int32_t current, int32_t target) -> int32_t {
        int32_t delta = target - current;
        if (delta > max_step) delta = max_step;
        if (delta < -max_step) delta = -max_step;
        return current + delta;
    };
    _applied_left_milli_percent = slew(
        _applied_left_milli_percent,
        static_cast<int32_t>(_target_left_percent) * 1000);
    _applied_right_milli_percent = slew(
        _applied_right_milli_percent,
        static_cast<int32_t>(_target_right_percent) * 1000);
}

void SafetyMachine::enterFault(FaultCode fault) {
    if (_fault != FaultCode::NONE || _estop_latched) return;
    const EspState from = _state;
    hardZero();
    _fault = fault;
    _state = EspState::FAULT;
    _control_mode = static_cast<uint8_t>(ControlMode::INACTIVE);
    ++_fault_occurrences;
    _fault_transition.fault = fault;
    _fault_transition.from = from;
    _fault_transition.to = EspState::FAULT;
    _fault_transition.occurrence_count = _fault_occurrences;
    _fault_transition_pending = true;
}

void SafetyMachine::tick(uint32_t now_ms, const MotorHealth& motor) {
    if (_state == EspState::ARMED) {
        if (motor.controller_fault) {
            enterFault(FaultCode::MOTOR_CONTROLLER_FAULT);
            return;
        }
        if (!motor.feedback_available || !motor.feedback_fresh) {
            enterFault(FaultCode::MOTOR_FEEDBACK_LOST);
            return;
        }
        const uint32_t watchdog_reference_ms = _have_valid_cmd
            ? _last_cmd_ms : _armed_ms;
        if (age(now_ms, watchdog_reference_ms) >=
            _active_command_timeout_ms) {
            ++_watchdog_trips;
            enterFault(FaultCode::CMD_VEL_TIMEOUT);
            return;
        }
        updateRamp(now_ms);
    }

    // Connection liveness is independent, but never masks the mandatory
    // CMD_VEL_TIMEOUT fault when both deadlines are already overdue.
    if (_handshake &&
        age(now_ms, _last_heartbeat_ms) >= kHeartbeatTimeoutMs) {
        connectionLost();
    }
}

SafetySnapshot SafetyMachine::snapshot(uint32_t now_ms) const {
    SafetySnapshot out;
    out.state = _state;
    out.fault = _fault;
    out.handshake = _handshake;
    out.estop_latched = _estop_latched;
    out.control_mode = _control_mode;
    out.last_cmd_age_ms = _have_valid_cmd
        ? age(now_ms, _last_cmd_ms) : 0xFFFFFFFFu;
    out.last_heartbeat_age_ms = _handshake
        ? age(now_ms, _last_heartbeat_ms) : 0xFFFFFFFFu;
    out.watchdog_trips = _watchdog_trips;
    out.hard_zero_generation = _hard_zero_generation;
    if (_state == EspState::ARMED && _fault == FaultCode::NONE &&
        !_estop_latched) {
        out.applied_left_percent = roundedPercent(
            _applied_left_milli_percent);
        out.applied_right_percent = roundedPercent(
            _applied_right_milli_percent);
    }
    return out;
}

uint8_t SafetyMachine::relayAllowedMask() const {
    if (!_handshake || _estop_latched || _fault != FaultCode::NONE) return 0u;
    return (_state == EspState::DISARMED || _state == EspState::ARMED)
        ? 0x03u : 0u;
}

bool SafetyMachine::consumeFaultTransition(FaultTransition& output) {
    if (!_fault_transition_pending) return false;
    output = _fault_transition;
    _fault_transition_pending = false;
    return true;
}

bool SafetyMachine::consumeEstopTransition(EstopTransition& output) {
    if (!_estop_transition_pending) return false;
    output = _estop_transition;
    _estop_transition_pending = false;
    return true;
}

}  // namespace pibridge
