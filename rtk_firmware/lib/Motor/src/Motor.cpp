// Motor.cpp - Hoverboard UART driver. Протокол 1:1 из sound/sound.ino (рабочий). MIT.

#include "Motor.h"
#include "MotorCommandMath.h"
#include <cstring>

int16_t Motor::clamp16(int32_t v, int16_t lo, int16_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return (int16_t)v;
}

int16_t Motor::slewToward(int16_t cur, int16_t target, int16_t maxStep) {
    int32_t diff = (int32_t)target - (int32_t)cur;
    if (diff >  maxStep) diff =  maxStep;
    if (diff < -maxStep) diff = -maxStep;
    return (int16_t)(cur + diff);
}

void Motor::begin(HardwareSerial& serial, float wheelBaseM, uint8_t rxPin, uint8_t txPin) {
    _serial = &serial;
    _wheelBase = wheelBaseM;
    _serial->begin(HOVER_BAUD, SERIAL_8N1, rxPin, txPin);
    _cmdSpeed = _cmdSteer = 0;
    _targetSpeed = _targetSteer = 0;
    _curLeftPct = _curRightPct = 0;
    _enabled = false;
    _lastSendMs = 0;
    _lastSetMs = 0;
    _commandGeneration = 0;
    _commandGuard = motorcmd::MotorCommandGuard{};
    _commandSource = "none";
    _commandTimeoutMs = HOVER_CMD_TIMEOUT_MS;
    _lastTxHeartbeatMs = 0;
    _lastTxDiagValid = false;
    _fbIdx = 0; _fbPrev = 0; _fbP = nullptr;
    _haveFeedback = false;
    _lastFeedbackMs = 0;
    _feedbackFault = false;
    _hardwareFault = false;
    _commandFault = false;
    _batVoltsFilt = 0.0f;
}

void Motor::txTaskTrampoline(void* arg) {
    Motor* self = (Motor*)arg;
    // Ровный 50Гц поток на плату, независимо от блокировок главного цикла.
    const TickType_t period = pdMS_TO_TICKS(HOVER_SEND_MS);
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        self->loop(millis());
        vTaskDelayUntil(&last, period);
    }
}

void Motor::startTxTask() {
    if (_taskRunning) return;
    _taskRunning = true;
    // Ядро 0 (главный Arduino loop живёт на ядре 1) — поток мотора не страдает от
    // долгих GNSS/IMU/WiFi вызовов в setup()/loop(). Приоритет выше idle.
    xTaskCreatePinnedToCore(txTaskTrampoline, "motorTx", 4096, this, 3, &_txTask, 0);
}

void Motor::sendHover(int16_t steer, int16_t speed) {
    if (!_serial) return;
    HoverCommand cmd;
    cmd.start    = (uint16_t)HOVER_START_FRAME;
    cmd.steer    = steer;
    cmd.speed    = speed;
    cmd.checksum = (uint16_t)(cmd.start ^ cmd.steer ^ cmd.speed);
    _serial->write((uint8_t*)&cmd, sizeof(cmd));
}

// Legacy left/right hoverboard virtual channels -> packet axes.  The sign contract and
// pure/tested mapping live in MotorCommandMath.h; the UART packet itself is
// unchanged from the proven sound/sound.ino drive() implementation.
static void pctToHover(int leftPct, int rightPct, int16_t& speedOut, int16_t& steerOut) {
    const motorcmd::HoverAxes axes = motorcmd::hoverChannelsToAxes(
        leftPct, rightPct, HOVER_MAX_CMD, HOVER_MAX_PERCENT);
    speedOut = (int16_t)axes.speed;
    steerOut = (int16_t)axes.steer;
}

uint32_t Motor::authorizeMotionCommand(const char* source,
                                       uint32_t sourceTimeoutMs) {
    if (source == nullptr || source[0] == '\0') source = "unspecified";
    taskENTER_CRITICAL(&_mux);
    const motorcmd::GuardedMotorCommand before = _commandGuard.state();
    if (!before.zeroLatchActive && std::strcmp(_commandSource, source) != 0) {
        taskEXIT_CRITICAL(&_mux);
        return 0u;
    }
    const uint32_t authorization = _commandGuard.authorize();
    _commandSource = source;
    _commandTimeoutMs = sourceTimeoutMs;
    _lastSetMs = millis();
    taskEXIT_CRITICAL(&_mux);
    return authorization;
}

bool Motor::refreshMotionAuthorization(uint32_t authorization) {
    taskENTER_CRITICAL(&_mux);
    const motorcmd::GuardedMotorCommand state = _commandGuard.state();
    const bool accepted = !state.zeroLatchActive && authorization != 0u &&
                          authorization == state.authorizationEpoch;
    if (accepted) _lastSetMs = millis();
    taskEXIT_CRITICAL(&_mux);
    return accepted;
}

bool Motor::setManualPercent(int leftPct, int rightPct,
                             uint32_t authorization) {
    leftPct  = (int)clamp16(leftPct,  -HOVER_MAX_PERCENT, HOVER_MAX_PERCENT);
    rightPct = (int)clamp16(rightPct, -HOVER_MAX_PERCENT, HOVER_MAX_PERCENT);
    int16_t sp, st;
    pctToHover(leftPct, rightPct, sp, st);
    sp = clamp16(sp, -HOVER_MAX_CMD, HOVER_MAX_CMD);
    st = clamp16(st, -HOVER_MAX_CMD, HOVER_MAX_CMD);
    taskENTER_CRITICAL(&_mux);
    const bool accepted = _commandGuard.apply(
        authorization, leftPct, rightPct, leftPct, rightPct);
    if (accepted) {
        _curLeftPct = leftPct;
        _curRightPct = rightPct;
        _targetSpeed = sp;
        _targetSteer = st;
        _enabled = true;
        _lastSetMs = millis();
        _commandGeneration++;
    }
    taskEXIT_CRITICAL(&_mux);
    return accepted;
}

bool Motor::setLinearAngularSpeed(float linearMps, float angularRadps,
                                  bool useRamp, uint32_t authorization) {
    (void)useRamp;  // slew всегда применяется в loop()
    if (!isfinite(linearMps) || !isfinite(angularRadps)) {
        taskENTER_CRITICAL(&_mux);
        _commandFault = true;
        taskEXIT_CRITICAL(&_mux);
        stopImmediately("invalid_command");
        return false;
    }
    // Compass-frame command contract is mode-aware because these legacy
    // hoverboard left/right values are virtual channels, not normalized
    // physical track velocities.  Positive angular increases heading in both
    // modes, using the separately field-proven DRIVE and TURN adapters.
    const float scale =
        (float)ROVER_AUTO_MAX_PERCENT / ROVER_MAX_SPEED_MPS;
    const bool translationalDrive = fabsf(linearMps) > 0.001f;
    const motorcmd::MotionMixMode mixMode = translationalDrive
        ? motorcmd::MotionMixMode::DRIVE
        : motorcmd::MotionMixMode::TURN_IN_PLACE;
    const motorcmd::CommandMix mixed =
        motorcmd::mixEffectiveCompassCommandPercent(
            linearMps, angularRadps, _wheelBase, scale,
            ROVER_AUTO_MIN_EFFECTIVE_LEFT_PERCENT,
            ROVER_AUTO_MIN_EFFECTIVE_RIGHT_PERCENT,
            ROVER_AUTO_MAX_PERCENT, mixMode);
    int leftPct = mixed.effective.left;
    int rightPct = mixed.effective.right;
    const uint32_t deadbandNow = millis();
    if ((leftPct != mixed.requested.left ||
         rightPct != mixed.requested.right) &&
        (deadbandNow - _lastDeadbandLogMs) >= 750u) {
        Serial.printf("[MOTOR_DEADBAND] requestedL=%d requestedR=%d "
                      "appliedL=%d appliedR=%d measL=%d measR=%d "
                      "feedbackAge=%u\n",
                      mixed.requested.left, mixed.requested.right,
                      leftPct, rightPct,
                      speedLeftMeas(), speedRightMeas(),
                      (unsigned)feedbackAgeMs(deadbandNow));
        _lastDeadbandLogMs = deadbandNow;
    }
    leftPct  = (int)clamp16(leftPct,  -ROVER_AUTO_MAX_PERCENT, ROVER_AUTO_MAX_PERCENT);
    rightPct = (int)clamp16(rightPct, -ROVER_AUTO_MAX_PERCENT, ROVER_AUTO_MAX_PERCENT);
    int16_t sp, st;
    pctToHover(leftPct, rightPct, sp, st);
    sp = clamp16(sp, -HOVER_MAX_CMD, HOVER_MAX_CMD);
    st = clamp16(st, -HOVER_MAX_CMD, HOVER_MAX_CMD);
    taskENTER_CRITICAL(&_mux);
    const bool accepted = _commandGuard.apply(
        authorization, mixed.requested.left, mixed.requested.right,
        leftPct, rightPct);
    if (accepted) {
        _curLeftPct = leftPct;
        _curRightPct = rightPct;
        _targetSpeed = sp;
        _targetSteer = st;
        _enabled = true;
        _lastSetMs = millis();
        _commandGeneration++;
    }
    taskEXIT_CRITICAL(&_mux);
    return accepted;
}

void Motor::stopImmediately(const char* reason) {
    const uint32_t nowMs = millis();
    bool logTransition = false;
    const char* previousSource = "none";
    uint32_t previousSequence = 0u;
    uint32_t commandAge = 0xFFFFFFFFu;
    uint32_t tokenEpoch = 0u;
    taskENTER_CRITICAL(&_mux);
    const motorcmd::GuardedMotorCommand before = _commandGuard.state();
    logTransition = !before.zeroLatchActive;
    previousSource = _commandSource;
    previousSequence = before.motorCommandSequence;
    tokenEpoch = before.authorizationEpoch;
    commandAge = _lastSetMs == 0u ? 0xFFFFFFFFu : nowMs - _lastSetMs;
    _targetSpeed = _targetSteer = 0;
    _cmdSpeed = _cmdSteer = 0;
    _curLeftPct = _curRightPct = 0;
    _lastSetMs = nowMs;
    _commandGuard.hardZero();
    _commandSource = "none";
    _commandTimeoutMs = HOVER_CMD_TIMEOUT_MS;
    _commandGeneration++;
    if (logTransition) {
        // Close the epoch and emit zero atomically. The TX task cannot append
        // a frame calculated from the previous epoch after this frame.
        sendHover(0, 0);
        ++_sendCount;
    }
    taskEXIT_CRITICAL(&_mux);
    if (logTransition) {
        Serial.printf("[MOTOR_ZERO_REASON] reason=%s previousSource=%s "
                      "previousSequence=%u commandAge=%u tokenEpoch=%u\n",
                      reason ? reason : "explicit_stop", previousSource,
                      (unsigned)previousSequence, (unsigned)commandAge,
                      (unsigned)tokenEpoch);
    }
}

void Motor::enable(bool en) {
    if (!en) { stopImmediately(); _enabled = false; return; }
    _enabled = true;
}

void Motor::clearSafetyFaults() {
    taskENTER_CRITICAL(&_mux);
    _feedbackFault = false;
    _hardwareFault = false;
    _commandFault = false;
    taskEXIT_CRITICAL(&_mux);
}

uint32_t Motor::feedbackAgeMs(uint32_t nowMs) const {
    const uint32_t last = _lastFeedbackMs;
    if (last == 0) return 0xFFFFFFFFu;
    if (nowMs < last) return 0;
    return nowMs - last;
}

void Motor::loop(uint32_t nowMs) {
    receiveFeedback();

    // Startup hold: первые N мс после begin() НЕ шлём команды — даём плате hoverboard-FOC
    // время на инициализацию (иначе она пищит «нет сигнала» при первом мусорном пакете).
    if (_startupHoldUntilMs != 0) {
        if (nowMs < _startupHoldUntilMs) return;
        _startupHoldUntilMs = 0;
    }

    if (nowMs - _lastSendMs < HOVER_SEND_MS) return;
    _lastSendMs = nowMs;

    // Атомарный снимок цели (пишется из главного ядра, читается здесь на ядре 0).
    taskENTER_CRITICAL(&_mux);
    int16_t  tgtSpeed = _targetSpeed;
    int16_t  tgtSteer = _targetSteer;
    bool     en       = _enabled;
    uint32_t lastSet  = _lastSetMs;
    int16_t  cmdSpeed = _cmdSpeed;
    int16_t  cmdSteer = _cmdSteer;
    uint32_t generation = _commandGeneration;
    bool zeroLatched = _commandGuard.state().zeroLatchActive;
    uint32_t commandTimeoutMs = _commandTimeoutMs;
    taskEXIT_CRITICAL(&_mux);

    const bool nonZeroTarget = tgtSpeed != 0 || tgtSteer != 0 ||
                               cmdSpeed != 0 || cmdSteer != 0;
    bool forceHardZero = false;
    if (en && nonZeroTarget && feedbackAgeMs(nowMs) > MOTOR_FEEDBACK_TIMEOUT_MS) {
        // Independent motor-thread failsafe: stop before the main loop has a
        // chance to evaluate Safety. A fresh valid frame clears this fault.
        tgtSpeed = 0;
        tgtSteer = 0;
        _feedbackFault = true;
        forceHardZero = true;
    }
    if (_hardwareFault || _commandFault) {
        tgtSpeed = 0;
        tgtSteer = 0;
        forceHardZero = true;
    }

    // failsafe: давно не было новой команды -> цель в ноль (плавно через slew)
    if (en && lastSet != 0 && (nowMs - lastSet) > commandTimeoutMs) {
        tgtSpeed = 0;
        tgtSteer = 0;
        forceHardZero = true;
    }

    bool logZeroTransition = false;
    const char* zeroReason = nullptr;
    const char* previousSource = "none";
    uint32_t previousSequence = 0u;
    uint32_t commandAge = 0xFFFFFFFFu;
    uint32_t tokenEpoch = 0u;
    if (forceHardZero) {
        taskENTER_CRITICAL(&_mux);
        // Re-evaluate under the command lock. A main-loop refresh that raced
        // with the TX snapshot must cancel a stale timeout decision.
        const bool currentNonZero = _targetSpeed != 0 || _targetSteer != 0 ||
                                    _cmdSpeed != 0 || _cmdSteer != 0;
        const bool currentFeedbackStop = _enabled && currentNonZero &&
            feedbackAgeMs(nowMs) > MOTOR_FEEDBACK_TIMEOUT_MS;
        const bool currentHardwareStop = _hardwareFault;
        const bool currentCommandStop = _commandFault;
        const bool currentTimeoutStop = _enabled && _lastSetMs != 0u &&
            (nowMs - _lastSetMs) > _commandTimeoutMs;
        if (!_commandGuard.state().zeroLatchActive &&
            (currentFeedbackStop || currentHardwareStop ||
             currentCommandStop || currentTimeoutStop)) {
            const motorcmd::GuardedMotorCommand before = _commandGuard.state();
            previousSource = _commandSource;
            previousSequence = before.motorCommandSequence;
            commandAge = nowMs - _lastSetMs;
            tokenEpoch = before.authorizationEpoch;
            if (currentFeedbackStop) {
                _feedbackFault = true;
                zeroReason = "feedback_fault";
            } else if (currentHardwareStop) {
                zeroReason = "hardware_fault";
            } else if (currentCommandStop) {
                zeroReason = "invalid_command";
            } else {
                zeroReason = "command_timeout";
            }
            _targetSpeed = _targetSteer = 0;
            _cmdSpeed = _cmdSteer = 0;
            _curLeftPct = _curRightPct = 0;
            _commandGuard.hardZero();
            _commandSource = "none";
            _commandTimeoutMs = HOVER_CMD_TIMEOUT_MS;
            ++_commandGeneration;
            generation = _commandGeneration;
            zeroLatched = true;
            logZeroTransition = true;
        }
        taskEXIT_CRITICAL(&_mux);
        if (logZeroTransition) {
            tgtSpeed = tgtSteer = 0;
            cmdSpeed = cmdSteer = 0;
        }
    }

    // slew к цели в домене команды (сглаживание как в sound.ino)
    int16_t nextSpeed = zeroLatched ? 0 :
        slewToward(cmdSpeed, tgtSpeed, HOVER_SLEW_SPEED);
    int16_t nextSteer = zeroLatched ? 0 :
        slewToward(cmdSteer, tgtSteer, HOVER_SLEW_STEER);
    taskENTER_CRITICAL(&_mux);
    if (_commandGuard.state().zeroLatchActive) {
        nextSpeed = 0;
        nextSteer = 0;
        _cmdSpeed = 0;
        _cmdSteer = 0;
    } else if (generation == _commandGeneration) {
        _cmdSpeed = nextSpeed;
        _cmdSteer = nextSteer;
    } else {
        nextSpeed = _cmdSpeed;
        nextSteer = _cmdSteer;
    }
    nextSpeed = (int16_t)_commandGuard.uartSpeedForFrame(nextSpeed);
    nextSteer = (int16_t)_commandGuard.uartSteerForFrame(nextSteer);
    _commandGuard.recordUartFrame(nextSpeed, nextSteer);
    // UART write is inside the same critical section as the latch check. Once
    // stopImmediately() returns, no pre-latch nonzero frame can still be sent.
    sendHover(nextSteer, nextSpeed);
    const motorcmd::GuardedMotorCommand diag = _commandGuard.state();
    const char* diagSource = _commandSource;
    const uint32_t diagLastSetMs = _lastSetMs;
    taskEXIT_CRITICAL(&_mux);
    _sendCount++;

    if (logZeroTransition) {
        Serial.printf("[MOTOR_ZERO_REASON] reason=%s previousSource=%s "
                      "previousSequence=%u commandAge=%u tokenEpoch=%u\n",
                      zeroReason, previousSource,
                      (unsigned)previousSequence, (unsigned)commandAge,
                      (unsigned)tokenEpoch);
    }

    const bool stateChanged = !_lastTxDiagValid ||
        diag.routeRequestedLeft != _lastTxDiagRequestedLeft ||
        diag.routeRequestedRight != _lastTxDiagRequestedRight ||
        diag.motorAppliedLeft != _lastTxDiagAppliedLeft ||
        diag.motorAppliedRight != _lastTxDiagAppliedRight ||
        diag.uartSpeed != _lastTxDiagUartSpeed ||
        diag.uartSteer != _lastTxDiagUartSteer ||
        diag.zeroLatchActive != _lastTxDiagZeroLatch ||
        std::strcmp(diagSource, _lastTxDiagSource) != 0;
    const bool activeMovement = !diag.zeroLatchActive &&
        (diag.routeRequestedLeft != 0 || diag.routeRequestedRight != 0 ||
         diag.motorAppliedLeft != 0 || diag.motorAppliedRight != 0 ||
         diag.uartSpeed != 0 || diag.uartSteer != 0);
    const bool heartbeat = activeMovement && !stateChanged &&
        nowMs - _lastTxHeartbeatMs >= 5000u;
    const bool fullyStopped = diag.zeroLatchActive &&
        diag.routeRequestedLeft == 0 && diag.routeRequestedRight == 0 &&
        diag.motorAppliedLeft == 0 && diag.motorAppliedRight == 0 &&
        diag.uartSpeed == 0 && diag.uartSteer == 0;
    if ((stateChanged && !fullyStopped) || heartbeat) {
        const uint32_t age = diagLastSetMs == 0u
            ? 0xFFFFFFFFu : nowMs - diagLastSetMs;
        Serial.printf("[MOTOR_TX] motorCommandSequence=%u "
                      "routeRequestedL=%d routeRequestedR=%d "
                      "motorAppliedL=%d motorAppliedR=%d uartSpeed=%d "
                      "uartSteer=%d lastCommandUpdateAge=%u "
                      "zeroLatchActive=%d motorSource=%s heartbeat=%d\n",
                      (unsigned)diag.motorCommandSequence,
                      diag.routeRequestedLeft, diag.routeRequestedRight,
                      diag.motorAppliedLeft, diag.motorAppliedRight,
                      diag.uartSpeed, diag.uartSteer, (unsigned)age,
                      diag.zeroLatchActive ? 1 : 0, diagSource,
                      heartbeat ? 1 : 0);
        if (activeMovement) _lastTxHeartbeatMs = nowMs;
    }
    _lastTxDiagRequestedLeft = diag.routeRequestedLeft;
    _lastTxDiagRequestedRight = diag.routeRequestedRight;
    _lastTxDiagAppliedLeft = diag.motorAppliedLeft;
    _lastTxDiagAppliedRight = diag.motorAppliedRight;
    _lastTxDiagUartSpeed = diag.uartSpeed;
    _lastTxDiagUartSteer = diag.uartSteer;
    _lastTxDiagZeroLatch = diag.zeroLatchActive;
    _lastTxDiagSource = diagSource;
    _lastTxDiagValid = true;
}

Motor::TxDiagnostics Motor::txDiagnostics(uint32_t nowMs) {
    taskENTER_CRITICAL(&_mux);
    const motorcmd::GuardedMotorCommand state = _commandGuard.state();
    const uint32_t lastSet = _lastSetMs;
    const char* source = _commandSource;
    taskEXIT_CRITICAL(&_mux);
    TxDiagnostics out;
    out.motorCommandSequence = state.motorCommandSequence;
    out.routeRequestedLeft = state.routeRequestedLeft;
    out.routeRequestedRight = state.routeRequestedRight;
    out.motorAppliedLeft = state.motorAppliedLeft;
    out.motorAppliedRight = state.motorAppliedRight;
    out.uartSpeed = state.uartSpeed;
    out.uartSteer = state.uartSteer;
    out.lastCommandUpdateAgeMs = lastSet == 0u
        ? 0xFFFFFFFFu : nowMs - lastSet;
    out.zeroLatchActive = state.zeroLatchActive;
    out.motorSource = source;
    return out;
}

void Motor::receiveFeedback() {
    if (!_serial) return;
    while (_serial->available()) {
        uint8_t in = (uint8_t)_serial->read();
        uint16_t startFrame = ((uint16_t)in << 8) | _fbPrev;

        if (startFrame == HOVER_START_FRAME) {
            _fbP = (uint8_t*)&_fbNew;
            *_fbP++ = _fbPrev;
            *_fbP++ = in;
            _fbIdx = 2;
        } else if (_fbIdx >= 2 && _fbIdx < sizeof(HoverFeedback)) {
            *_fbP++ = in;
            _fbIdx++;
        }

        if (_fbIdx == sizeof(HoverFeedback)) {
            uint16_t cs = (uint16_t)(
                _fbNew.start ^ _fbNew.cmd1 ^ _fbNew.cmd2 ^
                _fbNew.speedR_meas ^ _fbNew.speedL_meas ^
                _fbNew.batVoltage ^ _fbNew.boardTemp ^ _fbNew.cmdLed);
            if (_fbNew.start == HOVER_START_FRAME && cs == _fbNew.checksum) {
                memcpy(&_fb, &_fbNew, sizeof(HoverFeedback));
                _haveFeedback = true;
                _lastFeedbackMs = millis();
                _feedbackFault = false;
                float v = _fb.batVoltage * HOVER_BAT_VOLT_SCALE;
                const float tempC = _fb.boardTemp * 0.1f;
                // The stock feedback protocol has no dedicated fault word.
                // Invalid power telemetry and board over-temperature are the
                // hardware-fault signals available on this UART protocol.
                if (!isfinite(v) || !isfinite(tempC) || v < 15.0f ||
                    v > 60.0f || tempC < -40.0f ||
                    tempC >= MOTOR_BOARD_OVERTEMP_C) {
                    _hardwareFault = true;
                }
                if (_batVoltsFilt <= 0.01f) _batVoltsFilt = v;       // первый замер
                else _batVoltsFilt += 0.25f * (v - _batVoltsFilt);   // low-pass
            }
            _fbIdx = 0;
        }
        _fbPrev = in;
    }
}

int Motor::batteryPercent() const {
    if (!_haveFeedback || _batVoltsFilt < 1.0f) return -1;
    // 10S Li-ion: ~3.3 В/ячейку (0%) .. 4.2 В/ячейку (100%)
    float perCell = _batVoltsFilt / (float)HOVER_BAT_CELLS;
    float pct = (perCell - 3.3f) / (4.2f - 3.3f) * 100.0f;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return (int)(pct + 0.5f);
}
