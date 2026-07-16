#pragma once

#include <Arduino.h>
#include <Adafruit_BNO08x.h>

namespace pibridge {

struct ImuSample {
    uint64_t timestamp_us = 0u;
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;
    float qw = 1.0f;
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    uint8_t calibration_status = 0u;
    float accuracy_rad = 999.0f;
    uint32_t sequence = 0u;
};

class PiBridgeImu {
public:
    bool begin(uint8_t sda_pin, uint8_t scl_pin);
    void poll();
    bool consume(ImuSample& output);
    uint32_t ageMs(uint32_t now_ms) const;
    bool available() const { return _running; }

private:
    bool enableReports();

    Adafruit_BNO08x _bno;
    sh2_SensorValue_t _event{};
    ImuSample _sample{};
    bool _running = false;
    bool _fresh = false;
    bool _have_sample = false;
    bool _have_orientation = false;
    bool _orientation_updated = false;
    bool _have_gyro = false;
    bool _have_acceleration = false;
    uint8_t _orientation_status = 0u;
    uint8_t _gyro_status = 0u;
    uint8_t _acceleration_status = 0u;
    uint32_t _last_sample_ms = 0u;
};

}  // namespace pibridge
