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
    // Positive linear means physical forward translation. Positive angular
    // means clockwise heading increase in DRIVE and TURN_IN_PLACE; the
    // field-proven mode-aware hoverboard adapter is in MotorCommandMath.h.
    void setLinearAngularSpeed(float linearMps, float angularRadps, bool useRamp = true);

    // Ручной режим из app: left/right в процентах (-100..100).
    void setManualPercent(int leftPct, int rightPct);

    // Запустить фоновую TX-задачу на ядре 0: она сама крутит loop() ровным 50Гц,
    // не завися от блокировок главного цикла (GNSS/IMU/WiFi init и т.п.) — иначе
    // поток команд на плату рвётся и hoverboard пищит «нет сигнала». Вызывать ОДИН
    // раз сразу после begin(). После запуска главному циклу loop() звать НЕ нужно.
    void startTxTask();
    bool txTaskRunning() const { return _taskRunning; }

    void stopImmediately();
    void enable(bool en);
    // Задать «тишину» на N мс после старта — плата hoverboard FOC не любит пакеты
    // пока не стабилизировалась. По умолчанию 1500мс (вызов не обязателен).
    void setStartupHoldMs(uint32_t ms) { _startupHoldUntilMs = millis() + ms; }
    bool enabled() const { return _enabled; }

    // Вызывать часто из loop(): шлёт команду (каждые HOVER_SEND_MS) + читает фидбэк.
    // ПРИМ.: при работающей TX-задаче (см. begin) вызывать вручную НЕ нужно — задача
    // на ядре 0 сама крутит loop() ровным 50Гц независимо от блокировок главного цикла.
    void loop(uint32_t nowMs);

    // ---- телеметрия (реальная, из фидбэка) ----
    bool  haveFeedback() const { return _haveFeedback; }
    uint32_t feedbackAgeMs(uint32_t nowMs) const;
    bool feedbackAlive(uint32_t nowMs) const {
        return feedbackAgeMs(nowMs) <= MOTOR_FEEDBACK_TIMEOUT_MS;
    }
    bool hardwareFault() const { return _hardwareFault; }
    bool commandFault() const { return _commandFault; }
    void clearSafetyFaults();
    float batteryVolts() const { return _batVoltsFilt; }
    int   batteryPercent() const;                 // по 10S Li-ion кривой
    float boardTempC() const { return _fb.boardTemp * 0.1f; }
    // Raw side-labelled hoverboard protocol feedback.  No sign normalization
    // is performed here; a per-side inversion must not be invented without a
    // forward/backward hardware observation for both tracks.
    int   speedLeftMeas()  const { return _fb.speedL_meas; }
    int   speedRightMeas() const { return _fb.speedR_meas; }

    // последняя посланная команда (для телеметрии/отладки)
    int   lastSpeedCmd() const { return _cmdSpeed; }
    int   lastSteerCmd() const { return _cmdSteer; }
    uint32_t sendCount() const { return _sendCount; }
    // Legacy hoverboard virtual-channel targets used by telemetry.  These are
    // not normalized physical track velocities in every motion mode; see the
    // mode-aware sign contract in MotorCommandMath.h.
    int   currentLeftPwm()  const { return _curLeftPct; }
    int   currentRightPwm() const { return _curRightPct; }

private:
    void sendHover(int16_t steer, int16_t speed);
    void receiveFeedback();
    static int16_t clamp16(int32_t v, int16_t lo, int16_t hi);
    static int16_t slewToward(int16_t cur, int16_t target, int16_t maxStep);
    static void txTaskTrampoline(void* arg);   // FreeRTOS entry -> ((Motor*)arg)->loop()

    // Фоновая TX-задача на ядре 0 (изоляция потока команд от блокировок главного цикла).
    TaskHandle_t   _txTask = nullptr;
    volatile bool  _taskRunning = false;
    // Спинлок для записи цели из главного цикла / чтения в TX-задаче (разные ядра).
    portMUX_TYPE   _mux = portMUX_INITIALIZER_UNLOCKED;

    HardwareSerial* _serial = nullptr;
    float _wheelBase = ROVER_WHEELBASE_M;
    bool  _enabled = false;

    // целевые/текущие команды в домене платы (steer/speed)
    int16_t _cmdSpeed = 0, _cmdSteer = 0;            // текущие (после slew)
    int16_t _targetSpeed = 0, _targetSteer = 0;      // цель
    // текущие проценты (для телеметрии)
    int   _curLeftPct = 0, _curRightPct = 0;

    uint32_t _lastSendMs = 0;
    uint32_t _lastDeadbandLogMs = 0;
    uint32_t _lastSetMs  = 0;                        // когда последний раз задавали команду
    uint32_t _startupHoldUntilMs = 0;                // первые N мс не слать (плата FOC стартует)
    uint32_t _sendCount = 0;                         // счётчик отправленных пакетов (диагностика 50Гц)

    uint32_t _commandGeneration = 0;

    // feedback parser
    HoverFeedback _fb{};
    HoverFeedback _fbNew{};
    uint8_t  _fbIdx = 0;
    uint8_t  _fbPrev = 0;
    uint8_t* _fbP = nullptr;
    bool     _haveFeedback = false;
    volatile uint32_t _lastFeedbackMs = 0;
    volatile bool _feedbackFault = false;
    volatile bool _hardwareFault = false;
    volatile bool _commandFault = false;
    float    _batVoltsFilt = 0.0f;
};
