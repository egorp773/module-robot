// Gnss.cpp - F9P wrapper. MIT. Поверх SparkFun_u-blox_GNSS_Arduino_Library.

#include "Gnss.h"
#include "RtkConfig.h"

// Глобальная переменная для PVT-callback (SparkFun не хранит user-data)
static Gnss* g_pvtTarget = nullptr;

void Gnss::pvtCallbackStatic(UBX_NAV_PVT_data_t *pvt) {
    if (g_pvtTarget) g_pvtTarget->pvtCallback(pvt);
}

void Gnss::pvtCallback(UBX_NAV_PVT_data_t *pvt) {
    capturePvt(pvt);
}

void Gnss::capturePvt(const UBX_NAV_PVT_data_t *pvt) {
    if (!pvt) return;
    _latE7   = pvt->lat;
    _lonE7   = pvt->lon;
    _h       = pvt->height;
    _hAcc    = pvt->hAcc;
    _vAcc    = pvt->vAcc;
    _gSp     = pvt->gSpeed;
    _headMot = pvt->headMot;
    _fix     = pvt->fixType;
    // РЕАЛЬНЫЙ carrSoln из NAV-PVT flags (SparkFun 2.2.28 ОТДАЁТ его):
    //   carrSoln: 0 = none, 1 = float, 2 = fixed
    //   carrSolnValid гарантирует валидность поля; gnssFixOK — валидный fix по маскам.
    int carr = pvt->flags.bits.carrSoln;
    bool fixOk = pvt->flags.bits.gnssFixOK;
    if (fixOk && carr == 2)      _carSol = 2;   // RTK FIXED
    else if (fixOk && carr == 1) _carSol = 1;   // RTK FLOAT
    else                          _carSol = 0;   // нет RTK
    _diff    = pvt->flags.bits.diffSoln;
    _nSv     = pvt->numSV;
    _pDop    = pvt->pDOP * 0.01f;
    _hasFreshPvt = true;
    _lastPvtMs = millis();
    _lastPvtITow = pvt->iTOW;
}

bool Gnss::begin(HardwareSerial& serial, GnssRole role) {
    _serial = &serial;
    _role = role;
    serial.begin(F9P_BAUD, SERIAL_8N1, PIN_F9P_RX, PIN_F9P_TX);

    // F9P может не ответить с первого раза (cold start, autobaud). Retry.
    // НО! SparkFun _gnss.begin() часто возвращает false даже когда F9P жив —
    // это особенность autobaud. Поэтому НЕ прерываем — пробуем настроить.
    for (int i = 0; i < 5; ++i) {
        if (_gnss.begin(serial)) {
            Serial.println("[GNSS] F9P begin() OK");
            break;
        }
        delay(300);
    }

    g_pvtTarget = this;
    _gnss.setNavigationFrequency(_role == GNSS_ROVER ? 5 : 1);

    if (_role == GNSS_ROVER) {
        _gnss.setAutoPVT(true);
    }

    if (_role == GNSS_BASE) {
        _gnss.setSurveyMode(1, 60, 2.0);
        _svinStartMs = millis();
        _surveyInProgress = true;

        _gnss.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_UART1, 1);
        _gnss.enableRTCMmessage(UBX_RTCM_1074, COM_PORT_UART1, 1);
        _gnss.enableRTCMmessage(UBX_RTCM_1084, COM_PORT_UART1, 1);
        _gnss.enableRTCMmessage(UBX_RTCM_1094, COM_PORT_UART1, 1);
        _gnss.enableRTCMmessage(UBX_RTCM_1124, COM_PORT_UART1, 1);
        _gnss.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_UART1, 1);
        Serial.println("[GNSS] RTCM output configured");
    }
    return true;
}

void Gnss::loop() {
    if (_role == GNSS_ROVER && _serial) {
        while (_serial->available()) {
            parseRoverUbxByte((uint8_t)_serial->read());
        }
        return;
    }

    _gnss.checkUblox();
    _gnss.checkCallbacks();

    uint32_t now = millis();

    if (_role == GNSS_BASE && _surveyInProgress) {
        if (_gnss.getSurveyInValid() == 1) {
            _surveyInProgress = false;
        }
    }
    // RTCM output на base может слететь при переходе Survey-In → FIXED.
    // Каждые 3 сек повторно включаем — это идемпотентно.
    static uint32_t lastRtcm = 0;
    if (_role == GNSS_BASE && now - lastRtcm > 3000) {
        lastRtcm = now;
        _gnss.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_UART1, 1);
        _gnss.enableRTCMmessage(UBX_RTCM_1074, COM_PORT_UART1, 1);
        _gnss.enableRTCMmessage(UBX_RTCM_1084, COM_PORT_UART1, 1);
        _gnss.enableRTCMmessage(UBX_RTCM_1094, COM_PORT_UART1, 1);
        _gnss.enableRTCMmessage(UBX_RTCM_1124, COM_PORT_UART1, 1);
        _gnss.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_UART1, 1);
    }
}

static uint16_t rdU16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static uint32_t rdU32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static int32_t rdI32(const uint8_t *p) { return (int32_t)rdU32(p); }

void Gnss::captureNavPvtPayload(const uint8_t *p, uint16_t len) {
    if (len < 92) return;
    uint32_t iTOW = rdU32(p + 0);
    uint8_t flags = p[21];
    uint8_t numSv = p[23];
    uint32_t hAcc = rdU32(p + 40);
    if (iTOW == 0 || numSv > 64 || hAcc == 0) return;

    _latE7   = rdI32(p + 28);
    _lonE7   = rdI32(p + 24);
    _h       = rdI32(p + 32);
    _hAcc    = (int32_t)hAcc;
    _vAcc    = (int32_t)rdU32(p + 44);
    _gSp     = rdI32(p + 60);
    _headMot = rdI32(p + 64);
    _fix     = p[20];
    bool fixOk = (flags & 0x01) != 0;
    int carr = (flags >> 6) & 0x03;
    if (fixOk && carr == 2)      _carSol = 2;
    else if (fixOk && carr == 1) _carSol = 1;
    else                         _carSol = 0;
    _diff    = (flags & 0x02) != 0;
    _nSv     = numSv;
    _pDop    = rdU16(p + 76) * 0.01f;
    _hasFreshPvt = true;
    _lastPvtMs = millis();
    _lastPvtITow = iTOW;
}

void Gnss::parseRoverUbxByte(uint8_t b) {
    auto ck = [&](uint8_t v) { _ubxCkA += v; _ubxCkB += _ubxCkA; };
    switch (_ubxState) {
        case UBX_SYNC1:
            _ubxState = (b == 0xB5) ? UBX_SYNC2 : UBX_SYNC1;
            break;
        case UBX_SYNC2:
            _ubxState = (b == 0x62) ? UBX_CLASS : UBX_SYNC1;
            _ubxCkA = _ubxCkB = 0;
            break;
        case UBX_CLASS:
            _ubxClass = b; ck(b); _ubxState = UBX_ID; break;
        case UBX_ID:
            _ubxId = b; ck(b); _ubxState = UBX_LEN1; break;
        case UBX_LEN1:
            _ubxLen = b; ck(b); _ubxState = UBX_LEN2; break;
        case UBX_LEN2:
            _ubxLen |= ((uint16_t)b << 8); ck(b); _ubxIdx = 0;
            _ubxState = (_ubxLen <= sizeof(_ubxPayload)) ? UBX_PAYLOAD : UBX_SYNC1;
            break;
        case UBX_PAYLOAD:
            _ubxPayload[_ubxIdx++] = b; ck(b);
            if (_ubxIdx >= _ubxLen) _ubxState = UBX_CKA;
            break;
        case UBX_CKA:
            if (b == _ubxCkA) _ubxState = UBX_CKB;
            else _ubxState = UBX_SYNC1;
            break;
        case UBX_CKB:
            if (b == _ubxCkB && _ubxClass == 0x01 && _ubxId == 0x07) {
                captureNavPvtPayload(_ubxPayload, _ubxLen);
            }
            _ubxState = UBX_SYNC1;
            break;
    }
}

bool Gnss::baseSurveyComplete(uint16_t &accMm, uint32_t &durationS) {
    accMm = 0;
    durationS = 0;
    return _gnss.getSurveyInValid() == 1;
}

size_t Gnss::feedRtcm(const uint8_t* data, size_t n) {
    if (!_serial || _role != GNSS_ROVER) return 0;
    size_t w = _serial->write(data, n);
    _rxBytes += w;
    _rxPackets++;
    _rtcm.lastRxMs = millis();
    _rtcm.active = true;
    return w;
}

void Gnss::setRtcmInput(bool enable) { (void)enable; }

void Gnss::pollRxmRtcm() {}

uint32_t Gnss::pvtAgeMs(uint32_t nowMs) const {
    if (_lastPvtMs == 0) return 0xFFFFFFFFu;
    if (nowMs < _lastPvtMs) return 0;
    return nowMs - _lastPvtMs;
}
