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
    taskENTER_CRITICAL(&_pvtMux);
    _latE7   = pvt->lat;
    _lonE7   = pvt->lon;
    _h       = pvt->height;
    _hAcc    = pvt->hAcc;
    _vAcc    = pvt->vAcc;
    _gSp     = pvt->gSpeed;
    _headMot = pvt->headMot;
    _headAcc = (int32_t)pvt->headAcc;
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
    const uint32_t now = millis();
    _lastPvtIntervalMs = _lastPvtMs == 0 ? 0 : now - _lastPvtMs;
    _lastPvtITowDeltaMs = _lastPvtITow == 0xFFFFFFFFu ? 0
                                                       : pvt->iTOW - _lastPvtITow;
    _lastPvtMs = now;
    _lastPvtITow = pvt->iTOW;
    _pvtCount++;
    taskEXIT_CRITICAL(&_pvtMux);
}

bool Gnss::begin(HardwareSerial& serial, GnssRole role) {
    _serial = &serial;
    _role = role;
    // Большой RX-буфер ДО begin(): на 115200 RTCM/PVT приходят пачками,
    // дефолтных 256 байт не хватает при блокировках loop().
    const bool rxBufferOk = serial.setRxBufferSize(8192);
    serial.begin(F9P_BAUD, SERIAL_8N1, PIN_F9P_RX, PIN_F9P_TX);
    Serial.printf("[GNSS] UART RX buffer=8192 result=%d\n", rxBufferOk ? 1 : 0);

    // === Автодетект бауда + подъём до F9P_RUN_BAUD ===
    // После power-cycle F9P на дефолтных 38400; после soft-reset ESP32 чип
    // уже может быть на F9P_RUN_BAUD (конфиг UART1 переживает reset ESP32).
    // Пробуем оба, затем поднимаем UART1 до рабочего бауда — на 38400 канал
    // забит RTCM-инжекцией и FIXED флапает.
    const uint32_t kBauds[2] = { F9P_RUN_BAUD, F9P_BAUD };
    bool connected = false;
    uint32_t activeBaud = F9P_BAUD;
    for (int attempt = 0; attempt < 3 && !connected; ++attempt) {
        for (uint32_t b : kBauds) {
            serial.updateBaudRate(b);
            delay(50);
            if (_gnss.begin(serial)) {
                connected = true;
                activeBaud = b;
                Serial.printf("[GNSS] F9P begin() OK @%lu\n", (unsigned long)b);
                break;
            }
        }
        if (!connected) delay(250);
    }
    if (!connected) {
        // SparkFun begin() иногда фейлит даже на живом чипе — не прерываемся,
        // пробуем настроить вслепую на дефолтном бауде.
        Serial.println("[GNSS] F9P begin() failed @115200/38400 — continue blind @38400");
        serial.updateBaudRate(F9P_BAUD);
        activeBaud = F9P_BAUD;
    }

    if (activeBaud != F9P_RUN_BAUD) {
        // CFG-PRT: UART1 → F9P_RUN_BAUD. ACK может потеряться при смене бауда,
        // поэтому подтверждаем повторным begin() на новой скорости.
        _gnss.setSerialRate(F9P_RUN_BAUD, COM_PORT_UART1);
        delay(100);
        serial.updateBaudRate(F9P_RUN_BAUD);
        delay(50);
        bool ok = false;
        for (int i = 0; i < 3 && !ok; ++i) {
            ok = _gnss.begin(serial);
            if (!ok) delay(150);
        }
        if (ok) {
            Serial.printf("[GNSS] UART1 baud -> %u OK\n", (unsigned)F9P_RUN_BAUD);
        } else {
            // Не подтвердилось — возможно чип остался на 38400. Откат.
            serial.updateBaudRate(F9P_BAUD);
            delay(50);
            if (_gnss.begin(serial)) {
                Serial.println("[GNSS] baud change not confirmed, staying @38400");
            } else {
                // Ни там ни там не отвечает — остаёмся на RUN (write-only режим
                // всё равно позволит инжектить RTCM, если чип переключился).
                serial.updateBaudRate(F9P_RUN_BAUD);
                Serial.println("[GNSS] baud state unknown, assuming F9P_RUN_BAUD");
            }
        }
    }

    g_pvtTarget = this;
    const bool navRateOk = _gnss.setNavigationFrequency(_role == GNSS_ROVER ? 5 : 1);
    Serial.printf("[GNSS] navigation frequency request=%uHz result=%d\n",
                  (unsigned)(_role == GNSS_ROVER ? 5 : 1), navRateOk ? 1 : 0);

    if (_role == GNSS_ROVER) {
        bool inOk = _gnss.setPortInput(COM_PORT_UART1, COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3);
        bool outOk = _gnss.setUART1Output(COM_TYPE_UBX);
        Serial.printf("[GNSS] rover UART1 input UBX/NMEA/RTCM3=%d output UBX=%d\n",
                      inOk ? 1 : 0, outOk ? 1 : 0);
        const bool autoPvtOk = _gnss.setAutoPVT(true);
        Serial.printf("[GNSS] NAV-PVT UART1 auto rate=1 result=%d\n",
                      autoPvtOk ? 1 : 0);
        // Do not rely on a successful poll-before-set inside the library.
        // These explicit UBX writes make 200ms NAV solutions and one PVT per
        // solution deterministic even if the initial CFG-RATE poll timed out.
        configureRoverPvtRate();
        enableRoverRtcmStatus();
        startRoverRxTask();
    }

    if (_role == GNSS_BASE) {
        bool inOk = _gnss.setPortInput(COM_PORT_UART1, COM_TYPE_UBX | COM_TYPE_NMEA);
        // ВАЖНО: на выходе нужен RTCM3 (для форвардинга) И UBX (иначе getSurveyStatus /
        // CFG-ACK не возвращаются → waitSurveyIn вечно в таймауте, setSurveyMode/
        // enableRTCMmessage не подтверждаются). RTCM-фильтр в base.cpp отсеет UBX по CRC.
        bool outOk = _gnss.setUART1Output(COM_TYPE_RTCM3 | COM_TYPE_UBX);
        Serial.printf("[GNSS] base UART1 input UBX/NMEA=%d output RTCM3+UBX=%d\n",
                      inOk ? 1 : 0, outOk ? 1 : 0);
        _gnss.setSurveyMode(1, BASE_SURVEY_MIN_S, BASE_SURVEY_ACC_M);
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
        // The dedicated reader is the sole UART RX consumer. Keeping RX out
        // of Arduino loop prevents WiFi / Serial logging stalls from turning
        // into multi-second pvtAge gaps.
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

    taskENTER_CRITICAL(&_pvtMux);
    _latE7   = rdI32(p + 28);
    _lonE7   = rdI32(p + 24);
    _h       = rdI32(p + 32);
    _hAcc    = (int32_t)hAcc;
    _vAcc    = (int32_t)rdU32(p + 44);
    _gSp     = rdI32(p + 60);
    _headMot = rdI32(p + 64);
    _headAcc = (int32_t)rdU32(p + 72);   // headAcc, deg * 1e-5 (точность headMot)
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
    const uint32_t now = millis();
    _lastPvtIntervalMs = _lastPvtMs == 0 ? 0 : now - _lastPvtMs;
    _lastPvtITowDeltaMs = _lastPvtITow == 0xFFFFFFFFu ? 0
                                                       : iTOW - _lastPvtITow;
    _lastPvtMs = now;
    _lastPvtITow = iTOW;
    _pvtCount++;
    taskEXIT_CRITICAL(&_pvtMux);
}

void Gnss::captureRxmRtcmPayload(const uint8_t *p, uint16_t len) {
    if (len < 8) return;

    const bool crcFailed = (p[1] & 0x01) != 0;
    _rtcm.lastType = rdU16(p + 6);
    if (crcFailed) {
        _rtcm.crcFail++;
        return;
    }

    _rtcm.msgCount++;
    _rtcm.lastRxMs = millis();
    _rtcm.active = true;
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
            if (_ubxLen == 0) {
                _ubxState = UBX_CKA;
            } else if (_ubxLen <= sizeof(_ubxPayload)) {
                _ubxState = UBX_PAYLOAD;
            } else {
                // Consume the complete payload and checksum. Scanning an
                // oversized payload for sync bytes caused false framing and
                // could hide several following NAV-PVT messages.
                _ubxSkipRemaining = _ubxLen + 2u;
                _ubxOversizePackets++;
                _ubxState = UBX_SKIP;
            }
            break;
        case UBX_PAYLOAD:
            _ubxPayload[_ubxIdx++] = b; ck(b);
            if (_ubxIdx >= _ubxLen) _ubxState = UBX_CKA;
            break;
        case UBX_CKA:
            if (b == _ubxCkA) _ubxState = UBX_CKB;
            else { _ubxChecksumFailures++; _ubxState = UBX_SYNC1; }
            break;
        case UBX_CKB:
            if (b == _ubxCkB) {
                if (_ubxClass == 0x01 && _ubxId == 0x07) {
                    captureNavPvtPayload(_ubxPayload, _ubxLen);
                } else if (_ubxClass == 0x02 && _ubxId == 0x32) {
                    captureRxmRtcmPayload(_ubxPayload, _ubxLen);
                }
            } else _ubxChecksumFailures++;
            _ubxState = UBX_SYNC1;
            break;
        case UBX_SKIP:
            if (_ubxSkipRemaining > 0) _ubxSkipRemaining--;
            if (_ubxSkipRemaining == 0) _ubxState = UBX_SYNC1;
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
    const uint32_t last = _lastPvtMs;
    if (last == 0) return 0xFFFFFFFFu;
    if (nowMs < last) return 0;
    return nowMs - last;
}

bool Gnss::consumeFreshPvt(GnssPvtData& out) {
    bool fresh = false;
    taskENTER_CRITICAL(&_pvtMux);
    fresh = _hasFreshPvt;
    if (fresh) {
        out.captureTimestampMs = _lastPvtMs;
        out.pvtId = _pvtCount;
        out.latE7 = _latE7; out.lonE7 = _lonE7; out.heightMm = _h;
        out.hAccMm = _hAcc; out.vAccMm = _vAcc; out.gSpeedMmps = _gSp;
        out.headMotDegE5 = _headMot; out.headAccDegE5 = _headAcc;
        out.fixType = _fix; out.carrierSol = _carSol; out.diffSoln = _diff;
        out.numSv = _nSv; out.pDop = _pDop;
        _hasFreshPvt = false;
    }
    taskEXIT_CRITICAL(&_pvtMux);
    return fresh;
}

void Gnss::sendUbx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
    if (!_serial) return;

    uint8_t ckA = 0;
    uint8_t ckB = 0;
    auto add = [&](uint8_t v) {
        _serial->write(v);
        ckA += v;
        ckB += ckA;
    };

    _serial->write(0xB5);
    _serial->write(0x62);
    add(cls);
    add(id);
    add((uint8_t)(len & 0xFF));
    add((uint8_t)(len >> 8));
    for (uint16_t i = 0; i < len; ++i) add(payload[i]);
    _serial->write(ckA);
    _serial->write(ckB);
}

void Gnss::enableRoverRtcmStatus() {
    if (!_serial || _role != GNSS_ROVER) return;

    // UBX-CFG-MSG: enable UBX-RXM-RTCM (class 0x02, id 0x32) on UART1.
    const uint8_t payload[] = {
        0x02, 0x32, // msgClass, msgID
        0x00,       // I2C/DDC rate
        0x01,       // UART1 rate
        0x00,       // UART2 rate
        0x00,       // USB rate
        0x00,       // SPI rate
        0x00        // reserved
    };
    sendUbx(0x06, 0x01, payload, sizeof(payload));
    Serial.println("[GNSS] rover UBX-RXM-RTCM status enabled on UART1");
}

void Gnss::configureRoverPvtRate() {
    // UBX-CFG-RATE: measRate=200ms, navRate=1, timeRef=GPS.
    const uint8_t rate[] = { 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00 };
    sendUbx(0x06, 0x08, rate, sizeof(rate));
    // UBX-CFG-MSG: NAV-PVT rate 1 on UART1.
    const uint8_t pvt[] = { 0x01, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00 };
    sendUbx(0x06, 0x01, pvt, sizeof(pvt));
    Serial.println("[GNSS] explicit NAV-PVT configuration: 5Hz, UART1 rate=1");
}

void Gnss::roverRxTaskTrampoline(void* arg) {
    Gnss* self = static_cast<Gnss*>(arg);
    for (;;) {
        bool readAny = false;
        while (self->_serial && self->_serial->available()) {
            const int v = self->_serial->read();
            if (v < 0) break;
            self->_uartRxBytes++;
            self->parseRoverUbxByte((uint8_t)v);
            readAny = true;
        }
        if (!readAny) vTaskDelay(pdMS_TO_TICKS(1));
        else taskYIELD();
    }
}

void Gnss::startRoverRxTask() {
    if (_role != GNSS_ROVER || !_serial || _rxTaskRunning) return;
    _rxTaskRunning = true;
    const BaseType_t ok = xTaskCreatePinnedToCore(
        roverRxTaskTrampoline, "gnssRx", 4096, this, 2, &_rxTask, 0);
    if (ok != pdPASS) {
        _rxTaskRunning = false;
        _rxTask = nullptr;
        Serial.println("[GNSS] ERROR: failed to start dedicated UART RX task");
    } else {
        Serial.println("[GNSS] dedicated UART RX task started on core0");
    }
}
