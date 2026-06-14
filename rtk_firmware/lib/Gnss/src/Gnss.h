// Gnss.h - F9P wrapper. MIT. Поверх SparkFun_u-blox_GNSS_Arduino_Library.

#pragma once
#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

enum GnssRole : uint8_t { GNSS_ROVER = 0, GNSS_BASE = 1 };

struct RtcmStatus {
    uint32_t lastType = 0;
    uint32_t msgCount = 0;
    uint32_t crcFail  = 0;
    bool     active   = false;
    uint32_t lastRxMs = 0;
};

class Gnss {
public:
    bool begin(HardwareSerial& serial, GnssRole role);
    void loop();

    // base: запускает Survey-In; возвращает true когда валидно
    bool baseSurveyInProgress() const { return _surveyInProgress; }
    bool baseSurveyComplete(uint16_t &accMm, uint32_t &durationS);

    // rover: писать RTCM-байты прямо в UART F9P
    size_t feedRtcm(const uint8_t* data, size_t n);

    // включить / выключить приём RTCM на rover UART (защита от случайного F9P в base)
    void setRtcmInput(bool enable);

    // опросить RXM-RTCM для статистики
    void pollRxmRtcm();

    // последний PVT
    bool hasFreshPvt() const { return _hasFreshPvt; }
    bool consumeFreshPvt() {
        bool fresh = _hasFreshPvt;
        _hasFreshPvt = false;
        return fresh;
    }
    uint32_t pvtAgeMs(uint32_t nowMs) const;

    // доступ к внутреннему объекту
    SFE_UBLOX_GNSS& ubx() { return _gnss; }

    // данные для StateEstimator
    int32_t lastLatE7()       const { return _latE7; }
    int32_t lastLonE7()       const { return _lonE7; }
    int32_t lastHeightMm()    const { return _h; }
    int32_t lastHAccMm()      const { return _hAcc; }
    int32_t lastVAccMm()      const { return _vAcc; }
    int32_t lastGSpeedMmps()  const { return _gSp; }
    int32_t lastHeadMotDe5()  const { return _headMot; }
    int     lastFixType()     const { return _fix; }
    int     lastCarrierSol()  const { return _carSol; }
    bool    lastDiffSoln()    const { return _diff; }
    int     lastNumSv()       const { return _nSv; }
    float   lastPDop()        const { return _pDop; }

    const RtcmStatus& rtcm() const { return _rtcm; }
    GnssRole role() const { return _role; }

    // counters
    uint32_t rxBytes()   const { return _rxBytes; }
    uint32_t rxPackets() const { return _rxPackets; }

private:
    static void pvtCallbackStatic(UBX_NAV_PVT_data_t *pvt);
    void pvtCallback(UBX_NAV_PVT_data_t *pvt);
    void capturePvt(const UBX_NAV_PVT_data_t *pvt);
    void parseRoverUbxByte(uint8_t b);
    void captureNavPvtPayload(const uint8_t *p, uint16_t len);
    void captureRxmRtcmPayload(const uint8_t *p, uint16_t len);
    void sendUbx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len);
    void enableRoverRtcmStatus();

    SFE_UBLOX_GNSS _gnss;
    HardwareSerial* _serial = nullptr;
    GnssRole _role = GNSS_ROVER;
    bool _hasFreshPvt = false;
    uint32_t _lastPvtMs = 0;
    uint32_t _lastPvtITow = 0xFFFFFFFFu;

    enum UbxParseState : uint8_t { UBX_SYNC1, UBX_SYNC2, UBX_CLASS, UBX_ID, UBX_LEN1, UBX_LEN2, UBX_PAYLOAD, UBX_CKA, UBX_CKB };
    UbxParseState _ubxState = UBX_SYNC1;
    uint8_t _ubxClass = 0, _ubxId = 0, _ubxCkA = 0, _ubxCkB = 0;
    uint16_t _ubxLen = 0, _ubxIdx = 0;
    uint8_t _ubxPayload[100]{};

    // raw PVT cache
    int32_t _latE7 = 0, _lonE7 = 0;
    int32_t _h = 0, _hAcc = 0, _vAcc = 0;
    int32_t _gSp = 0, _headMot = 0;
    int     _fix = 0, _carSol = 0, _nSv = 0;
    bool    _diff = false;
    float   _pDop = 0;

    // base: Survey-In
    bool _surveyInProgress = true;
    uint32_t _svinStartMs = 0;

    // rx counters
    uint32_t _rxBytes = 0;
    uint32_t _rxPackets = 0;

    // RXM-RTCM status
    RtcmStatus _rtcm{};

    // PVT callback registration flag
    bool _pvtCbInstalled = false;
};
