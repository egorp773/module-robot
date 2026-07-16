#include "PiBridgeMotor.h"

#include <esp_timer.h>
#include <string.h>

namespace pibridge {
namespace {

constexpr uint16_t kHoverStartFrame = 0xABCDu;
constexpr uint32_t kHoverBaud = 115200u;
constexpr int32_t kHoverMaxCommand = 300;
constexpr int32_t kHoverPercentDomain = 70;
constexpr int16_t kFeedbackZeroThreshold = 5;  // TODO_CALIBRATE raw units
constexpr int16_t kMinBatteryCentivolts = 1500;
constexpr int16_t kMaxBatteryCentivolts = 6000;
constexpr int16_t kMinTemperatureDecic = -400;
constexpr int16_t kMaxTemperatureDecic = 900;

int32_t absoluteFeedback(int16_t value) {
    const int32_t widened = value;
    return widened < 0 ? -widened : widened;
}

}  // namespace

int16_t PiBridgeMotor::clampPercent(int16_t value) {
    if (value > kFinalMaxPercent) return kFinalMaxPercent;
    if (value < -kFinalMaxPercent) return -kFinalMaxPercent;
    return value;
}

void PiBridgeMotor::begin(HardwareSerial& serial, uint8_t rx_pin,
                          uint8_t tx_pin) {
    _serial = &serial;
    _serial->setRxBufferSize(512u);
    _serial->begin(kHoverBaud, SERIAL_8N1, rx_pin, tx_pin);
    _feedback_index = 0u;
    _candidate_ptr = nullptr;
    _previous_byte = 0u;
    _last_feedback_ms = 0u;
    _last_feedback_us = 0u;
    _last_send_ms = 0u;
    _valid_frames = 0u;
    _invalid_frames = 0u;
    _have_feedback = false;
    _controller_fault = 0u;
    _applied_left = 0;
    _applied_right = 0;
    _uart_speed = 0;
    _uart_steer = 0;
    // BOOT invariant: the first frame emitted by this environment is zero.
    transmit(0, 0);
}

void PiBridgeMotor::transmit(int16_t left_percent, int16_t right_percent) {
    if (_serial == nullptr) return;
    left_percent = clampPercent(left_percent);
    right_percent = clampPercent(right_percent);

    const int16_t speed = static_cast<int16_t>(
        (static_cast<int32_t>(left_percent) + right_percent) *
        kHoverMaxCommand / (2 * kHoverPercentDomain));
    const int16_t steer = static_cast<int16_t>(
        (static_cast<int32_t>(right_percent) - left_percent) *
        kHoverMaxCommand / (2 * kHoverPercentDomain));

    HoverCommand command{};
    command.start = kHoverStartFrame;
    command.steer = steer;
    command.speed = speed;
    command.checksum = static_cast<uint16_t>(
        command.start ^ static_cast<uint16_t>(command.steer) ^
        static_cast<uint16_t>(command.speed));
    _serial->write(reinterpret_cast<const uint8_t*>(&command),
                   sizeof(command));

    taskENTER_CRITICAL(&_mux);
    _applied_left = left_percent;
    _applied_right = right_percent;
    _uart_speed = speed;
    _uart_steer = steer;
    taskEXIT_CRITICAL(&_mux);
}

void PiBridgeMotor::setSafeOutput(int16_t left_percent,
                                  int16_t right_percent,
                                  uint32_t now_ms) {
    left_percent = clampPercent(left_percent);
    right_percent = clampPercent(right_percent);
    if (now_ms - _last_send_ms < kSendPeriodMs) return;
    _last_send_ms = now_ms;
    transmit(left_percent, right_percent);
}

void PiBridgeMotor::hardZero(uint32_t now_ms) {
    (void)now_ms;
    // Hard stops bypass every ramp and the regular 50 Hz send schedule.
    transmit(0, 0);
    _last_send_ms = now_ms;
}

void PiBridgeMotor::pollFeedback() {
    if (_serial == nullptr) return;
    while (_serial->available() > 0) {
        const int value = _serial->read();
        if (value < 0) break;
        const uint8_t byte = static_cast<uint8_t>(value);
        const uint16_t possible_start =
            (static_cast<uint16_t>(byte) << 8u) | _previous_byte;

        if (possible_start == kHoverStartFrame) {
            _candidate_ptr = reinterpret_cast<uint8_t*>(&_candidate);
            *_candidate_ptr++ = _previous_byte;
            *_candidate_ptr++ = byte;
            _feedback_index = 2u;
        } else if (_feedback_index >= 2u &&
                   _feedback_index < sizeof(HoverFeedback)) {
            *_candidate_ptr++ = byte;
            ++_feedback_index;
        }

        if (_feedback_index == sizeof(HoverFeedback)) {
            const uint16_t checksum = static_cast<uint16_t>(
                _candidate.start ^ static_cast<uint16_t>(_candidate.cmd1) ^
                static_cast<uint16_t>(_candidate.cmd2) ^
                static_cast<uint16_t>(_candidate.speed_right) ^
                static_cast<uint16_t>(_candidate.speed_left) ^
                static_cast<uint16_t>(_candidate.battery_centivolts) ^
                static_cast<uint16_t>(_candidate.board_temperature_decic) ^
                _candidate.cmd_led);
            taskENTER_CRITICAL(&_mux);
            if (_candidate.start == kHoverStartFrame &&
                checksum == _candidate.checksum) {
                memcpy(&_feedback, &_candidate, sizeof(_feedback));
                _have_feedback = true;
                _last_feedback_ms = millis();
                _last_feedback_us = static_cast<uint64_t>(esp_timer_get_time());
                ++_valid_frames;
                const bool invalid_power =
                    _feedback.battery_centivolts < kMinBatteryCentivolts ||
                    _feedback.battery_centivolts > kMaxBatteryCentivolts ||
                    _feedback.board_temperature_decic < kMinTemperatureDecic ||
                    _feedback.board_temperature_decic >= kMaxTemperatureDecic;
                _controller_fault = invalid_power ? 1u : 0u;
            } else {
                ++_invalid_frames;
            }
            taskEXIT_CRITICAL(&_mux);
            _feedback_index = 0u;
            _candidate_ptr = nullptr;
        }
        _previous_byte = byte;
    }
}

MotorTelemetry PiBridgeMotor::telemetry(uint32_t now_ms) const {
    MotorTelemetry out;
    taskENTER_CRITICAL(&_mux);
    out.timestamp_us = _last_feedback_us;
    out.left_feedback = _feedback.speed_left;
    out.right_feedback = _feedback.speed_right;
    out.battery_centivolts = _feedback.battery_centivolts;
    out.board_temperature_decic = _feedback.board_temperature_decic;
    out.controller_fault = _controller_fault;
    out.valid_frames = _valid_frames;
    out.invalid_frames = _invalid_frames;
    out.applied_left_percent = _applied_left;
    out.applied_right_percent = _applied_right;
    out.uart_speed = _uart_speed;
    out.uart_steer = _uart_steer;
    out.available = _have_feedback;
    const uint32_t last_feedback_ms = _last_feedback_ms;
    taskEXIT_CRITICAL(&_mux);
    out.age_ms = out.available
        ? now_ms - last_feedback_ms : 0xFFFFFFFFu;
    out.fresh = out.available && out.age_ms <= kFeedbackTimeoutMs;
    out.at_zero = out.fresh &&
        absoluteFeedback(out.left_feedback) <= kFeedbackZeroThreshold &&
        absoluteFeedback(out.right_feedback) <= kFeedbackZeroThreshold &&
        out.applied_left_percent == 0 && out.applied_right_percent == 0;
    return out;
}

}  // namespace pibridge
