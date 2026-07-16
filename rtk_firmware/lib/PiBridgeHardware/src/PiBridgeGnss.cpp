#include "PiBridgeGnss.h"

#include <esp_timer.h>

namespace pibridge {
namespace {

constexpr uint32_t kF9pBootBaud = 38400u;
constexpr uint32_t kF9pRunBaud = 115200u;

uint16_t readU16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0]) |
           (static_cast<uint16_t>(p[1]) << 8u);
}

uint32_t readU32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8u) |
           (static_cast<uint32_t>(p[2]) << 16u) |
           (static_cast<uint32_t>(p[3]) << 24u);
}

int32_t readI32(const uint8_t* p) {
    return static_cast<int32_t>(readU32(p));
}

}  // namespace

bool PiBridgeGnss::begin(HardwareSerial& serial, uint8_t rx_pin,
                         uint8_t tx_pin) {
    if (_running) return true;
    _serial = &serial;
    _serial->setRxBufferSize(8192u);
    _serial->begin(kF9pBootBaud, SERIAL_8N1, rx_pin, tx_pin);

    const uint32_t candidate_bauds[] = {kF9pRunBaud, kF9pBootBaud};
    bool connected = false;
    uint32_t active_baud = kF9pBootBaud;
    for (uint8_t attempt = 0u; attempt < 3u && !connected; ++attempt) {
        for (uint32_t baud : candidate_bauds) {
            _serial->updateBaudRate(baud);
            delay(50u);
            if (_gnss.begin(*_serial)) {
                connected = true;
                active_baud = baud;
                break;
            }
        }
        if (!connected) delay(250u);
    }
    if (!connected) {
        _serial->updateBaudRate(kF9pBootBaud);
        return false;
    }

    if (active_baud != kF9pRunBaud) {
        _gnss.setSerialRate(kF9pRunBaud, COM_PORT_UART1);
        delay(50u);
        _serial->updateBaudRate(kF9pRunBaud);
        delay(50u);
        bool confirmed = false;
        for (uint8_t attempt = 0u; attempt < 3u && !confirmed; ++attempt) {
            confirmed = _gnss.begin(*_serial);
            if (!confirmed) delay(150u);
        }
        if (!confirmed) {
            _serial->updateBaudRate(kF9pBootBaud);
            if (!_gnss.begin(*_serial)) return false;
        }
    }

    _gnss.setNavigationFrequency(5u);
    _gnss.setPortInput(COM_PORT_UART1,
                       COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3);
    _gnss.setUART1Output(COM_TYPE_UBX);
    _gnss.setAutoPVT(true);
    configureMessages();

    _parse_state = ParseState::SYNC1;
    _fresh = false;
    _last_pvt_ms = 0u;
    _last_rtcm_ms = 0u;
    _running = true;
    const BaseType_t created = xTaskCreatePinnedToCore(
        readerTaskEntry, "piGnssRx", 4096u, this, 2u, &_reader_task, 0u);
    if (created != pdPASS) {
        _running = false;
        _reader_task = nullptr;
        return false;
    }
    return true;
}

void PiBridgeGnss::readerTaskEntry(void* argument) {
    static_cast<PiBridgeGnss*>(argument)->readerTask();
}

void PiBridgeGnss::readerTask() {
    for (;;) {
        bool read_any = false;
        while (_serial != nullptr && _serial->available() > 0) {
            const int value = _serial->read();
            if (value < 0) break;
            ++_uart_rx_bytes;
            parseByte(static_cast<uint8_t>(value));
            read_any = true;
        }
        if (read_any) {
            taskYIELD();
        } else {
            vTaskDelay(pdMS_TO_TICKS(1u));
        }
    }
}

void PiBridgeGnss::parseByte(uint8_t byte) {
    auto update_checksum = [&](uint8_t value) {
        _checksum_a = static_cast<uint8_t>(_checksum_a + value);
        _checksum_b = static_cast<uint8_t>(_checksum_b + _checksum_a);
    };

    switch (_parse_state) {
        case ParseState::SYNC1:
            _parse_state = byte == 0xB5u
                ? ParseState::SYNC2 : ParseState::SYNC1;
            break;
        case ParseState::SYNC2:
            _parse_state = byte == 0x62u
                ? ParseState::CLASS : ParseState::SYNC1;
            _checksum_a = 0u;
            _checksum_b = 0u;
            break;
        case ParseState::CLASS:
            _class = byte;
            update_checksum(byte);
            _parse_state = ParseState::ID;
            break;
        case ParseState::ID:
            _id = byte;
            update_checksum(byte);
            _parse_state = ParseState::LEN1;
            break;
        case ParseState::LEN1:
            _payload_length = byte;
            update_checksum(byte);
            _parse_state = ParseState::LEN2;
            break;
        case ParseState::LEN2:
            _payload_length |= static_cast<uint16_t>(byte) << 8u;
            update_checksum(byte);
            _payload_index = 0u;
            if (_payload_length == 0u) {
                _parse_state = ParseState::CHECKSUM_A;
            } else if (_payload_length <= sizeof(_payload)) {
                _parse_state = ParseState::PAYLOAD;
            } else {
                _skip_remaining = static_cast<uint32_t>(_payload_length) + 2u;
                ++_oversized_packets;
                _parse_state = ParseState::SKIP;
            }
            break;
        case ParseState::PAYLOAD:
            _payload[_payload_index++] = byte;
            update_checksum(byte);
            if (_payload_index >= _payload_length) {
                _parse_state = ParseState::CHECKSUM_A;
            }
            break;
        case ParseState::CHECKSUM_A:
            if (byte == _checksum_a) {
                _parse_state = ParseState::CHECKSUM_B;
            } else {
                ++_checksum_failures;
                _parse_state = ParseState::SYNC1;
            }
            break;
        case ParseState::CHECKSUM_B:
            if (byte == _checksum_b) {
                if (_class == 0x01u && _id == 0x07u) {
                    captureNavPvt(_payload, _payload_length);
                } else if (_class == 0x02u && _id == 0x32u) {
                    captureRxmRtcm(_payload, _payload_length);
                }
            } else {
                ++_checksum_failures;
            }
            _parse_state = ParseState::SYNC1;
            break;
        case ParseState::SKIP:
            if (_skip_remaining > 0u) --_skip_remaining;
            if (_skip_remaining == 0u) _parse_state = ParseState::SYNC1;
            break;
    }
}

void PiBridgeGnss::captureNavPvt(const uint8_t* payload,
                                 uint16_t length) {
    if (length < 92u) return;
    const uint32_t itow = readU32(payload);
    const uint8_t flags = payload[21];
    const uint8_t satellites = payload[23];
    const uint32_t h_acc = readU32(payload + 40u);
    if (itow == 0u || satellites > 64u || h_acc == 0u) return;

    taskENTER_CRITICAL(&_mux);
    _sample.timestamp_us = static_cast<uint64_t>(esp_timer_get_time());
    _sample.latitude_e7 = readI32(payload + 28u);
    _sample.longitude_e7 = readI32(payload + 24u);
    _sample.altitude_mm = readI32(payload + 32u);
    _sample.h_acc_mm = h_acc;
    _sample.v_acc_mm = readU32(payload + 44u);
    _sample.ground_speed_mm_s = readI32(payload + 60u);
    _sample.motion_heading_deg_e5 = readI32(payload + 64u);
    const bool fix_ok = (flags & 0x01u) != 0u;
    _sample.fix_type = fix_ok ? payload[20] : 0u;
    const uint8_t carrier = static_cast<uint8_t>((flags >> 6u) & 0x03u);
    _sample.carrier_solution = fix_ok && carrier <= 2u ? carrier : 0u;
    _sample.satellite_count = satellites;
    ++_sample.sequence;
    if (_sample.sequence == 0u) ++_sample.sequence;
    _last_pvt_ms = millis();
    _fresh = true;
    taskEXIT_CRITICAL(&_mux);
}

void PiBridgeGnss::captureRxmRtcm(const uint8_t* payload,
                                  uint16_t length) {
    if (length < 8u || (payload[1] & 0x01u) != 0u) return;
    taskENTER_CRITICAL(&_mux);
    _last_rtcm_ms = millis();
    taskEXIT_CRITICAL(&_mux);
}

bool PiBridgeGnss::consume(GnssSample& output, uint32_t now_ms) {
    bool fresh = false;
    taskENTER_CRITICAL(&_mux);
    fresh = _fresh;
    if (fresh) {
        output = _sample;
        output.rtcm_age_ms = _last_rtcm_ms == 0u
            ? 0xFFFFFFFFu : now_ms - _last_rtcm_ms;
        _fresh = false;
    }
    taskEXIT_CRITICAL(&_mux);
    return fresh;
}

uint32_t PiBridgeGnss::ageMs(uint32_t now_ms) const {
    taskENTER_CRITICAL(&_mux);
    const uint32_t last = _last_pvt_ms;
    const bool have_sample = _sample.sequence != 0u;
    taskEXIT_CRITICAL(&_mux);
    return have_sample ? now_ms - last : 0xFFFFFFFFu;
}

void PiBridgeGnss::sendUbx(uint8_t cls, uint8_t id,
                           const uint8_t* payload, uint16_t length) {
    if (_serial == nullptr) return;
    uint8_t checksum_a = 0u;
    uint8_t checksum_b = 0u;
    auto write_checked = [&](uint8_t value) {
        _serial->write(value);
        checksum_a = static_cast<uint8_t>(checksum_a + value);
        checksum_b = static_cast<uint8_t>(checksum_b + checksum_a);
    };
    _serial->write(0xB5u);
    _serial->write(0x62u);
    write_checked(cls);
    write_checked(id);
    write_checked(static_cast<uint8_t>(length));
    write_checked(static_cast<uint8_t>(length >> 8u));
    for (uint16_t i = 0u; i < length; ++i) write_checked(payload[i]);
    _serial->write(checksum_a);
    _serial->write(checksum_b);
}

void PiBridgeGnss::configureMessages() {
    // NAV rate 5 Hz, one NAV-PVT per solution, and RXM-RTCM status at UART1.
    const uint8_t rate[] = {0xC8u, 0x00u, 0x01u, 0x00u, 0x01u, 0x00u};
    sendUbx(0x06u, 0x08u, rate, sizeof(rate));
    const uint8_t pvt[] = {
        0x01u, 0x07u, 0x00u, 0x01u, 0x00u, 0x00u, 0x00u, 0x00u};
    sendUbx(0x06u, 0x01u, pvt, sizeof(pvt));
    const uint8_t rtcm_status[] = {
        0x02u, 0x32u, 0x00u, 0x01u, 0x00u, 0x00u, 0x00u, 0x00u};
    sendUbx(0x06u, 0x01u, rtcm_status, sizeof(rtcm_status));
}

}  // namespace pibridge
