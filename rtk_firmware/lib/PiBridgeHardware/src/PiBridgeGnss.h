#pragma once

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#include "RtkConfig.h"

namespace pibridge {

struct GnssSample {
    uint64_t timestamp_us = 0u;
    int32_t latitude_e7 = 0;
    int32_t longitude_e7 = 0;
    int32_t altitude_mm = 0;
    uint32_t h_acc_mm = 0u;
    uint32_t v_acc_mm = 0u;
    int32_t ground_speed_mm_s = 0;
    int32_t motion_heading_deg_e5 = 0;
    uint8_t fix_type = 0u;
    uint8_t carrier_solution = 0u;
    uint8_t satellite_count = 0u;
    uint32_t rtcm_age_ms = 0xFFFFFFFFu;
    uint32_t sequence = 0u;
};

// Isolated rover-only use of the proven UART setup and bounded UBX NAV-PVT /
// RXM-RTCM parser. It has no route, Wi-Fi, NTRIP, or motion responsibilities.
class PiBridgeGnss {
public:
    bool begin(HardwareSerial& serial, uint8_t rx_pin = PIN_F9P_RX,
               uint8_t tx_pin = PIN_F9P_TX);
    bool consume(GnssSample& output, uint32_t now_ms);
    uint32_t ageMs(uint32_t now_ms) const;
    bool available() const { return _running; }
    uint32_t uartRxBytes() const { return _uart_rx_bytes; }
    uint32_t checksumFailures() const { return _checksum_failures; }
    uint32_t oversizedPackets() const { return _oversized_packets; }

private:
    enum class ParseState : uint8_t {
        SYNC1,
        SYNC2,
        CLASS,
        ID,
        LEN1,
        LEN2,
        PAYLOAD,
        CHECKSUM_A,
        CHECKSUM_B,
        SKIP,
    };

    static void readerTaskEntry(void* argument);
    void readerTask();
    void parseByte(uint8_t byte);
    void captureNavPvt(const uint8_t* payload, uint16_t length);
    void captureRxmRtcm(const uint8_t* payload, uint16_t length);
    void sendUbx(uint8_t cls, uint8_t id, const uint8_t* payload,
                 uint16_t length);
    void configureMessages();

    SFE_UBLOX_GNSS _gnss;
    HardwareSerial* _serial = nullptr;
    TaskHandle_t _reader_task = nullptr;
    volatile bool _running = false;
    mutable portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;

    ParseState _parse_state = ParseState::SYNC1;
    uint8_t _class = 0u;
    uint8_t _id = 0u;
    uint8_t _checksum_a = 0u;
    uint8_t _checksum_b = 0u;
    uint16_t _payload_length = 0u;
    uint16_t _payload_index = 0u;
    uint32_t _skip_remaining = 0u;
    uint8_t _payload[128]{};

    GnssSample _sample{};
    bool _fresh = false;
    uint32_t _last_pvt_ms = 0u;
    uint32_t _last_rtcm_ms = 0u;
    volatile uint32_t _uart_rx_bytes = 0u;
    volatile uint32_t _checksum_failures = 0u;
    volatile uint32_t _oversized_packets = 0u;
};

}  // namespace pibridge
