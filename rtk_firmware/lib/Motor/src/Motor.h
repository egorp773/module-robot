// Motor.h - Hoverboard UART driver (Niklas Fauth hoverboard-firmware-hack protocol).
// Эталон протокола: sound/sound.ino (рабочая прошивка, робот ездил без проблем).
// Serial2 @115200, RX=16 TX=17. START=0xABCD.
// MIT.

#pragma once
#include <Arduino.h>
#include "RtkConfig.h"

// Команда ESP32 -> плата гироскутера
typedef struct __attribute__((packed)) {
    uint16_t start;
    int16_t  steer;
    int16_t  speed;
    uint16_t checksum;
} HoverCommand;

// Фидбэк плата -> ESP32
typedef struct __attribute__((packed)) {
    uint16_t start;
    int16_t  cmd1;
    int16_t  cmd2;
    int16_t  speedR_meas;
    int16_t  speedL_meas;
    int16_t  batVoltage;     // *0.01 = вольты
    int16_t  boardTemp;      // *0.1  = °C
    uint16_t cmdLed;
    uint16_t checksum;
} HoverFeedback;

class Motor {
public:
    // serial — уже сконструированный HardwareSerial(2). wheelBaseM для diff-drive.
    void begin(HardwareSerial& serial, float wheelBaseM,
               uint8_t rxPin = PIN_MOTOR_RX, uint8_t txPin = PIN_MOTOR_TX);

    // Главный вход навигации: линейная (м/с) + угловая (рад/с) скорость.
    void setLinearAngularSpeed(float linearMps, float angularRadps, bool useRamp = true);

    // Ручной режим из app: left/right в процентах (-100..100).
    void setManualPercent(int leftPct, int rightPct);

    void stopImmediately();
    void enable(bool en);
    bool enabled() const { return _enabled; }

    // Вызывать часто из loop(): шлёт команду (каждые HOVER_SEND_MS) + читает фидбэк.
    void loop(uint32_t nowMs);

    // ---- телеметрия (реальная, из фидбэка) ----
    bool  haveFeedback() const { return _haveFeedback; }
    float batteryVolts() const { return _batVoltsFilt; }
    int   batteryPercent() const;                 // по 10S Li-ion кривой
    float boardTempC() const { return _fb.boardTemp * 0.1f; }
    int   speedLeftMeas()  const { return _fb.speedL_meas; }
    int   speedRightMeas() const { return _fb.speedR_meas; }

    // последняя посланная команда (для телеметрии/отладки)
    int   lastSpeedCmd() const { return _cmdSpeed; }
    int   lastSteerCmd() const { return _cmdSteer; }
    // совместимость со старой телеметрией WsServer (left/right "pwm"-эквивалент)
    int   currentLeftPwm()  const { return _curLeftPct; }
    int   currentRightPwm() const { return _curRightPct; }

private:
    void sendHover(int16_t steer, int16_t speed);
    void receiveFeedback();
    static int16_t clamp16(int32_t v, int16_t lo, int16_t hi);
    static int16_t slewToward(int16_t cur, int16_t target, int16_t maxStep);

    HardwareSerial* _serial = nullptr;
    float _wheelBase = ROVER_WHEELBASE_M;
    bool  _enabled = false;

    // целевые/текущие команды в домене платы (steer/speed)
    int16_t _cmdSpeed = 0, _cmdSteer = 0;            // текущие (после slew)
    int16_t _targetSpeed = 0, _targetSteer = 0;      // цель
    // текущие проценты (для телеметрии)
    int   _curLeftPct = 0, _curRightPct = 0;

    uint32_t _lastSendMs = 0;
    uint32_t _lastSetMs  = 0;                        // когда последний раз задавали команду

    // feedback parser
    HoverFeedback _fb{};
    HoverFeedback _fbNew{};
    uint8_t  _fbIdx = 0;
    uint8_t  _fbPrev = 0;
    uint8_t* _fbP = nullptr;
    bool     _haveFeedback = false;
    float    _batVoltsFilt = 0.0f;
};
