#pragma once

#include <Arduino.h>

#include "RtkConfig.h"

namespace pibridge {

struct MotorTelemetry {
    uint64_t timestamp_us = 0u;
    int16_t left_feedback = 0;
    int16_t right_feedback = 0;
    int16_t battery_centivolts = 0;
    int16_t board_temperature_decic = 0;
    uint16_t controller_fault = 0u;
    uint32_t valid_frames = 0u;
    uint32_t invalid_frames = 0u;
    uint32_t age_ms = 0xFFFFFFFFu;
    int16_t applied_left_percent = 0;
    int16_t applied_right_percent = 0;
    int16_t uart_speed = 0;
    int16_t uart_steer = 0;
    bool available = false;
    bool fresh = false;
    bool at_zero = false;
};

// The only class in pi_bridge that can write the hoverboard UART. Its public
// non-zero input is called only by the dedicated motor task in pi_bridge.cpp.
class PiBridgeMotor {
public:
    static constexpr uint32_t kFeedbackTimeoutMs = 500u;
    static constexpr uint32_t kSendPeriodMs = 20u;
    static constexpr int16_t kFinalMaxPercent = 12;  // TODO_CALIBRATE

    void begin(HardwareSerial& serial, uint8_t rx_pin = PIN_MOTOR_RX,
               uint8_t tx_pin = PIN_MOTOR_TX);
    void pollFeedback();
    void setSafeOutput(int16_t left_percent, int16_t right_percent,
                       uint32_t now_ms);
    void hardZero(uint32_t now_ms);
    MotorTelemetry telemetry(uint32_t now_ms) const;

private:
#pragma pack(push, 1)
    struct HoverCommand {
        uint16_t start;
        int16_t steer;
        int16_t speed;
        uint16_t checksum;
    };
    struct HoverFeedback {
        uint16_t start;
        int16_t cmd1;
        int16_t cmd2;
        int16_t speed_right;
        int16_t speed_left;
        int16_t battery_centivolts;
        int16_t board_temperature_decic;
        uint16_t cmd_led;
        uint16_t checksum;
    };
#pragma pack(pop)
    static_assert(sizeof(HoverCommand) == 8u,
                  "hoverboard command layout changed");
    static_assert(sizeof(HoverFeedback) == 18u,
                  "hoverboard feedback layout changed");

    static int16_t clampPercent(int16_t value);
    void transmit(int16_t left_percent, int16_t right_percent);

    HardwareSerial* _serial = nullptr;
    mutable portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
    HoverFeedback _feedback{};
    HoverFeedback _candidate{};
    uint8_t* _candidate_ptr = nullptr;
    uint8_t _feedback_index = 0u;
    uint8_t _previous_byte = 0u;
    uint32_t _last_feedback_ms = 0u;
    uint64_t _last_feedback_us = 0u;
    uint32_t _last_send_ms = 0u;
    uint32_t _valid_frames = 0u;
    uint32_t _invalid_frames = 0u;
    int16_t _applied_left = 0;
    int16_t _applied_right = 0;
    int16_t _uart_speed = 0;
    int16_t _uart_steer = 0;
    uint16_t _controller_fault = 0u;
    bool _have_feedback = false;
};

}  // namespace pibridge
