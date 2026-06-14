// Motor.cpp - Hoverboard UART driver. Протокол 1:1 из sound/sound.ino (рабочий). MIT.

#include "Motor.h"

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
    _fbIdx = 0; _fbPrev = 0; _fbP = nullptr;
    _haveFeedback = false;
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

// left/right (-70..70%) -> speed/steer (домен платы) — формула из sound.ino drive()
static void pctToHover(int leftPct, int rightPct, int16_t& speedOut, int16_t& steerOut) {
    int32_t speedT = (int32_t)(leftPct + rightPct) * (int32_t)HOVER_MAX_CMD / (2 * HOVER_MAX_PERCENT);
    int32_t steerT = (int32_t)(rightPct - leftPct) * (int32_t)HOVER_MAX_CMD / (2 * HOVER_MAX_PERCENT);
    speedOut = (int16_t)speedT;
    steerOut = (int16_t)steerT;
}

void Motor::setManualPercent(int leftPct, int rightPct) {
    leftPct  = (int)clamp16(leftPct,  -HOVER_MAX_PERCENT, HOVER_MAX_PERCENT);
    rightPct = (int)clamp16(rightPct, -HOVER_MAX_PERCENT, HOVER_MAX_PERCENT);
    int16_t sp, st;
    pctToHover(leftPct, rightPct, sp, st);
    sp = clamp16(sp, -HOVER_MAX_CMD, HOVER_MAX_CMD);
    st = clamp16(st, -HOVER_MAX_CMD, HOVER_MAX_CMD);
    taskENTER_CRITICAL(&_mux);
    _curLeftPct = leftPct;
    _curRightPct = rightPct;
    _targetSpeed = sp;
    _targetSteer = st;
    _enabled = true;
    _lastSetMs = millis();
    taskEXIT_CRITICAL(&_mux);
}

void Motor::setLinearAngularSpeed(float linearMps, float angularRadps, bool useRamp) {
    (void)useRamp;  // slew всегда применяется в loop()
    // diff-drive: v_left = v - w*b/2 ; v_right = v + w*b/2  (м/с)
    float vL = linearMps - angularRadps * _wheelBase * 0.5f;
    float vR = linearMps + angularRadps * _wheelBase * 0.5f;
    // м/с -> проценты: ROVER_MAX_SPEED_MPS соответствует HOVER_MAX_PERCENT
    float scale = (float)HOVER_MAX_PERCENT / ROVER_MAX_SPEED_MPS;
    int leftPct  = (int)roundf(vL * scale);
    int rightPct = (int)roundf(vR * scale);
    leftPct  = (int)clamp16(leftPct,  -HOVER_MAX_PERCENT, HOVER_MAX_PERCENT);
    rightPct = (int)clamp16(rightPct, -HOVER_MAX_PERCENT, HOVER_MAX_PERCENT);
    int16_t sp, st;
    pctToHover(leftPct, rightPct, sp, st);
    sp = clamp16(sp, -HOVER_MAX_CMD, HOVER_MAX_CMD);
    st = clamp16(st, -HOVER_MAX_CMD, HOVER_MAX_CMD);
    taskENTER_CRITICAL(&_mux);
    _curLeftPct = leftPct;
    _curRightPct = rightPct;
    _targetSpeed = sp;
    _targetSteer = st;
    _enabled = true;
    _lastSetMs = millis();
    taskEXIT_CRITICAL(&_mux);
}

void Motor::stopImmediately() {
    taskENTER_CRITICAL(&_mux);
    _targetSpeed = _targetSteer = 0;
    _cmdSpeed = _cmdSteer = 0;
    _curLeftPct = _curRightPct = 0;
    _lastSetMs = millis();
    taskEXIT_CRITICAL(&_mux);
    if (_serial) sendHover(0, 0);
}

void Motor::enable(bool en) {
    if (!en) { stopImmediately(); _enabled = false; return; }
    _enabled = true;
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
    taskEXIT_CRITICAL(&_mux);

    // failsafe: давно не было новой команды -> цель в ноль (плавно через slew)
    if (en && lastSet != 0 && (nowMs - lastSet) > HOVER_CMD_TIMEOUT_MS) {
        tgtSpeed = 0;
        tgtSteer = 0;
    }

    // slew к цели в домене команды (сглаживание как в sound.ino)
    _cmdSpeed = slewToward(_cmdSpeed, tgtSpeed, HOVER_SLEW_SPEED);
    _cmdSteer = slewToward(_cmdSteer, tgtSteer, HOVER_SLEW_STEER);
    sendHover(_cmdSteer, _cmdSpeed);
    _sendCount++;
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
                float v = _fb.batVoltage * HOVER_BAT_VOLT_SCALE;
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
