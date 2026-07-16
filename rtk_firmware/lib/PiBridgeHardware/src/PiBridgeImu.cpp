#include "PiBridgeImu.h"

#include <Wire.h>
#include <esp_timer.h>
#include <math.h>

namespace pibridge {

bool PiBridgeImu::enableReports() {
    const bool orientation = _bno.enableReport(SH2_ROTATION_VECTOR, 20000u);
    const bool gyro = _bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 20000u);
    const bool acceleration =
        _bno.enableReport(SH2_LINEAR_ACCELERATION, 20000u);
    return orientation && gyro && acceleration;
}

bool PiBridgeImu::begin(uint8_t sda_pin, uint8_t scl_pin) {
    Wire.begin(sda_pin, scl_pin, 400000u);
    // The proven rover wiring has used both legal BNO085 I2C addresses.
    // Probe explicitly rather than silently assuming only the 0x4A default.
    if (!_bno.begin_I2C(0x4Au) && !_bno.begin_I2C(0x4Bu)) {
        _running = false;
        return false;
    }
    _running = enableReports();
    _fresh = false;
    _have_sample = false;
    _have_orientation = false;
    _orientation_updated = false;
    _have_gyro = false;
    _have_acceleration = false;
    _last_sample_ms = 0u;
    return _running;
}

void PiBridgeImu::poll() {
    if (!_running) return;
    if (_bno.wasReset()) {
        // Never combine a post-reset orientation with cached pre-reset gyro
        // or acceleration. A complete new triplet is required for telemetry.
        _fresh = false;
        _have_sample = false;
        _have_orientation = false;
        _orientation_updated = false;
        _have_gyro = false;
        _have_acceleration = false;
        if (!enableReports()) {
            _running = false;
            return;
        }
    }

    int budget = 24;
    while (budget-- > 0 && _bno.getSensorEvent(&_event)) {
        const uint8_t status = static_cast<uint8_t>(_event.status & 0x03u);
        switch (_event.sensorId) {
            case SH2_ROTATION_VECTOR:
                _sample.qx = _event.un.rotationVector.i;
                _sample.qy = _event.un.rotationVector.j;
                _sample.qz = _event.un.rotationVector.k;
                _sample.qw = _event.un.rotationVector.real;
                _sample.accuracy_rad = _event.un.rotationVector.accuracy;
                _orientation_status = status;
                _have_orientation = true;
                _orientation_updated = true;
                ++_sample.sequence;
                if (_sample.sequence == 0u) ++_sample.sequence;
                break;
            case SH2_GYROSCOPE_CALIBRATED:
                _sample.gx = _event.un.gyroscope.x;
                _sample.gy = _event.un.gyroscope.y;
                _sample.gz = _event.un.gyroscope.z;
                _gyro_status = status;
                _have_gyro = true;
                break;
            case SH2_LINEAR_ACCELERATION:
                _sample.ax = _event.un.linearAcceleration.x;
                _sample.ay = _event.un.linearAcceleration.y;
                _sample.az = _event.un.linearAcceleration.z;
                _acceleration_status = status;
                _have_acceleration = true;
                break;
            default:
                break;
        }
        if (_orientation_updated && _have_orientation &&
            _have_gyro && _have_acceleration) {
            uint8_t calibration = _orientation_status;
            if (_gyro_status < calibration) calibration = _gyro_status;
            if (_acceleration_status < calibration) {
                calibration = _acceleration_status;
            }
            const bool finite_sample =
                isfinite(_sample.qx) && isfinite(_sample.qy) &&
                isfinite(_sample.qz) && isfinite(_sample.qw) &&
                isfinite(_sample.gx) && isfinite(_sample.gy) &&
                isfinite(_sample.gz) && isfinite(_sample.ax) &&
                isfinite(_sample.ay) && isfinite(_sample.az) &&
                isfinite(_sample.accuracy_rad);
            if (finite_sample) {
                _sample.calibration_status = calibration;
                // ESP receive time is monotonic and common with all other
                // wire telemetry; the raw SH-2 timestamp restarts on reset.
                _sample.timestamp_us =
                    static_cast<uint64_t>(esp_timer_get_time());
                _last_sample_ms = millis();
                _fresh = true;
                _have_sample = true;
            }
            _orientation_updated = false;
        }
    }
}

bool PiBridgeImu::consume(ImuSample& output) {
    if (!_fresh) return false;
    output = _sample;
    _fresh = false;
    return true;
}

uint32_t PiBridgeImu::ageMs(uint32_t now_ms) const {
    return !_have_sample
        ? 0xFFFFFFFFu : now_ms - _last_sample_ms;
}

}  // namespace pibridge
