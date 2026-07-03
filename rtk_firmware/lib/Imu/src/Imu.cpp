// Imu.cpp - Adafruit_BNO08x wrapper with explicit absolute/relative heading states.

#include "Imu.h"
#include "RtkConfig.h"
#include <Wire.h>

#ifdef ARDUINO_ARCH_ESP32
#include <Preferences.h>
#endif

namespace {

static float quatYawDeg(float x, float y, float z, float w) {
    const float siny = 2.0f * (w * z + x * y);
    const float cosy = 1.0f - 2.0f * (y * y + z * z);
    return ImuMath::normalizeDeg360(atan2f(siny, cosy) * 180.0f / M_PI);
}

static float headingFromSourceYaw(float rawYawDeg) {
    return ImuMath::imuRawToRobotHeadingDeg(
        rawYawDeg,
        IMU_ROT_YAW_SIGN,
        IMU_ROT_YAW_OFFSET_DEG,
        IMU_COMPASS_YAW_ADJUST_DEG);
}

}  // namespace

void Imu::recomputeFinalYaw() {
    _robotYawDeg = ImuMath::applyHeadingCorrectionDeg(_baseYawDeg, _userHeadingCorrectionDeg);
}

void Imu::loadUserHeadingCorrection() {
#ifdef ARDUINO_ARCH_ESP32
    Preferences prefs;
    if (!prefs.begin(IMU_PREF_NAMESPACE, true)) {
        _userHeadingCorrectionDeg = 0.0f;
        return;
    }
    const float v = prefs.getFloat(IMU_PREF_HEAD_CORR_KEY, 0.0f);
    prefs.end();
    if (!isfinite(v)) {
        _userHeadingCorrectionDeg = 0.0f;
        return;
    }
    // Wrap into [-180, 180] so storage stays compact.
    _userHeadingCorrectionDeg = ImuMath::wrapDeg180(v);
#else
    // Host tests / non-ESP32 build: in-memory default. Tests can poke the
    // value directly via setUserHeadingCorrectionDeg (no-op persistence).
    _userHeadingCorrectionDeg = 0.0f;
#endif
}

bool Imu::setUserHeadingCorrectionDeg(float correctionDeg, String* err) {
    if (!isfinite(correctionDeg)) {
        if (err) *err = "non-finite correctionDeg";
        return false;
    }
    _userHeadingCorrectionDeg = ImuMath::wrapDeg180(correctionDeg);
    recomputeFinalYaw();
    return true;
}

bool Imu::applyAndSaveUserCorrection(float newCorrectionDeg, String* err) {
    if (!setUserHeadingCorrectionDeg(newCorrectionDeg, err)) return false;
#ifdef ARDUINO_ARCH_ESP32
    Preferences prefs;
    if (!prefs.begin(IMU_PREF_NAMESPACE, false)) {
        if (err) *err = "Preferences.begin failed";
        return false;
    }
    const size_t wrote = prefs.putFloat(IMU_PREF_HEAD_CORR_KEY, _userHeadingCorrectionDeg);
    prefs.end();
    if (wrote == 0) {
        if (err) *err = "Preferences.putFloat failed";
        return false;
    }
#endif
    return true;
}

bool Imu::clearUserHeadingCorrection(String* err) {
    _userHeadingCorrectionDeg = 0.0f;
    recomputeFinalYaw();
#ifdef ARDUINO_ARCH_ESP32
    Preferences prefs;
    if (!prefs.begin(IMU_PREF_NAMESPACE, false)) {
        if (err) *err = "Preferences.begin failed";
        return false;
    }
    prefs.remove(IMU_PREF_HEAD_CORR_KEY);
    prefs.end();
#endif
    return true;
}

void Imu::setManualYawTrusted(bool trusted, float headingDeg) {
    _manualYawTrusted = trusted;
    _manualTrustedHeadingDeg = ImuMath::normalizeDeg360(headingDeg);
    _manualTrustedAtMs = trusted ? millis() : 0;
}

bool Imu::begin(uint8_t sdaPin, uint8_t sclPin) {
    Wire.begin(sdaPin, sclPin, 400000);
    loadUserHeadingCorrection();
    Serial.print("[IMU] I2C scan: ");
    bool found = false;
    for (uint8_t addr = 0x08; addr < 0x78; ++addr) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("0x%02X ", addr);
            found = true;
        }
    }
    if (!found) Serial.print("none");
    Serial.println();

    if (_bno.begin_I2C(0x4A)) {
        Serial.println("[IMU] BNO08x at 0x4A");
    } else if (_bno.begin_I2C(0x4B)) {
        Serial.println("[IMU] BNO08x at 0x4B");
    } else {
        Serial.println("[IMU] BNO08x not found");
        return false;
    }

    _bno.enableReport(SH2_ROTATION_VECTOR, 10000);
    _bno.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR, 10000);
    _bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
    _bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 20000);
    _bno.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 20000);

    if (sh2_setCalConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG) != SH2_OK) {
        Serial.println("[IMU] WARNING: sh2_setCalConfig failed");
    }
    if (sh2_setDcdAutoSave(true) != SH2_OK) {
        Serial.println("[IMU] WARNING: sh2_setDcdAutoSave(true) failed");
    }

    _running = true;
    _headingState = ImuHeadingState::IMU_NO_DATA;
    _yawSource = ImuYawSource::NONE;
    _has = false;
    _absoluteReady = false;
    _absCandidateSource = ImuYawSource::NONE;
    _absCandidateSinceMs = 0;
    _startupAbsDeltaDeg = 0.0f;
    _magDisturbed = false;
    return true;
}

bool Imu::updateAbsoluteCandidate(ImuYawSource source, float sourceYawDeg, float robotYawDeg, float accRad, uint32_t nowMs) {
    const float absDelta = ImuMath::wrapDeg180(robotYawDeg - _absCandidateYawDeg);
    const bool sourceChanged = source != _absCandidateSource;
    if (_absCandidateSinceMs == 0 || sourceChanged ||
        fabsf(absDelta) > IMU_ABS_CANDIDATE_RESET_DELTA_DEG) {
        _absCandidateSinceMs = nowMs;
        _absCandidateYawDeg = robotYawDeg;
        _startupAbsDeltaDeg = 0.0f;
        _absCandidateSource = source;
        _absoluteReady = false;
    } else {
        _startupAbsDeltaDeg = fmaxf(_startupAbsDeltaDeg, fabsf(absDelta));
    }

    _yawSource = source;
    _sourceYawDeg = sourceYawDeg;
    _absYawDeg = robotYawDeg;
    _rawYawDeg = sourceYawDeg;
    _baseYawDeg = robotYawDeg;
    recomputeFinalYaw();
    _yawAccRad = accRad;
    _has = true;
    _lastMs = nowMs;
    _lastYawMs = nowMs;

    if (accRad > IMU_ABS_YAW_MAX_ACC_RAD) {
        _headingState = ImuHeadingState::IMU_ABSOLUTE_UNCALIBRATED;
        _absoluteReady = false;
        return false;
    }

    const uint32_t stableMs = nowMs - _absCandidateSinceMs;
    if (stableMs >= IMU_ABS_STARTUP_STABLE_MS && !_magDisturbed) {
        _headingState = ImuHeadingState::IMU_ABSOLUTE_OK;
        _absoluteReady = true;
        if (_magNormBaselineValid) {
            const float blend = 0.05f;
            _magNormBaseline = _magNormBaseline * (1.0f - blend) + _magNorm * blend;
        } else if (_magNorm > 0.0f) {
            _magNormBaseline = _magNorm;
            _magNormBaselineValid = true;
        }
        return true;
    }

    _headingState = ImuHeadingState::IMU_ABSOLUTE_UNCALIBRATED;
    _absoluteReady = false;
    return false;
}

void Imu::rememberMagNorm(float mx, float my, float mz) {
    _magX = mx;
    _magY = my;
    _magZ = mz;
    _magNorm = sqrtf(mx * mx + my * my + mz * mz);

    if (_magNorm <= 0.0f) return;
    if (!_magNormBaselineValid) {
        if (_absoluteReady) {
            _magNormBaseline = _magNorm;
            _magNormBaselineValid = true;
        }
        return;
    }

    const float baseline = fmaxf(_magNormBaseline, 0.001f);
    const float ratio = fabsf(_magNorm - baseline) / baseline;
    _magDisturbed = ratio > IMU_MAG_DISTURBANCE_RATIO;
    if (!_magDisturbed && _headingState == ImuHeadingState::IMU_ABSOLUTE_OK) {
        _magNormBaseline = _magNormBaseline * 0.98f + _magNorm * 0.02f;
    }
}

void Imu::updateHeadingState(uint32_t nowMs) {
    if (!_has) {
        _headingState = ImuHeadingState::IMU_NO_DATA;
        return;
    }
    if ((nowMs - _lastYawMs) > SAFE_IMU_AGE_MS) {
        _headingState = ImuHeadingState::IMU_STALE;
        return;
    }
    if (_yawSource == ImuYawSource::GAME_ROTATION_VECTOR) {
        if (_absoluteReady && _headingState == ImuHeadingState::IMU_ABSOLUTE_OK) return;
        _headingState = ImuHeadingState::IMU_RELATIVE_ONLY;
        return;
    }
    if (_magDisturbed) {
        _headingState = ImuHeadingState::IMU_MAG_DISTURBED;
        return;
    }
    if (_yawAccRad > IMU_ABS_YAW_MAX_ACC_RAD) {
        _headingState = ImuHeadingState::IMU_ABSOLUTE_UNCALIBRATED;
        return;
    }
    if (_absoluteReady) {
        _headingState = ImuHeadingState::IMU_ABSOLUTE_OK;
    } else if (_yawSource == ImuYawSource::GAME_ROTATION_VECTOR) {
        _headingState = ImuHeadingState::IMU_RELATIVE_ONLY;
    } else if (_absCandidateSinceMs != 0 &&
               (nowMs - _absCandidateSinceMs) >= IMU_ABS_STARTUP_STABLE_MS &&
               _startupAbsDeltaDeg <= IMU_ABS_STARTUP_MAX_DELTA_DEG) {
        _absoluteReady = true;
        _headingState = ImuHeadingState::IMU_ABSOLUTE_OK;
    } else if (_yawSource == ImuYawSource::NONE) {
        _headingState = ImuHeadingState::IMU_NO_DATA;
    } else {
        _headingState = ImuHeadingState::IMU_ABSOLUTE_UNCALIBRATED;
    }
}

void Imu::loop() {
    if (!_running) return;

    uint32_t nowMs = millis();
    if (!_bno.getSensorEvent(&_val)) {
        updateHeadingState(nowMs);
        return;
    }

    switch (_val.sensorId) {
        case SH2_MAGNETIC_FIELD_CALIBRATED: {
            float mx = _val.un.magneticField.x;
            float my = _val.un.magneticField.y;
            float mz = _val.un.magneticField.z;
            rememberMagNorm(mx, my, mz);
            _magCount++;
            break;
        }
        case SH2_GEOMAGNETIC_ROTATION_VECTOR: {
            _geoCount++;
            _lastGeoYawMs = nowMs;
            float yaw = quatYawDeg(
                _val.un.geoMagRotationVector.i,
                _val.un.geoMagRotationVector.j,
                _val.un.geoMagRotationVector.k,
                _val.un.geoMagRotationVector.real);
            _geoYawDeg = yaw;
            const float accRad = _val.un.geoMagRotationVector.accuracy;
            const float robotYaw = headingFromSourceYaw(yaw);
            const bool recentGoodRotation =
                _lastGoodRotYawMs != 0 &&
                (nowMs - _lastGoodRotYawMs) <= (SAFE_IMU_AGE_MS * 2u);
            if (accRad <= IMU_ABS_YAW_MAX_ACC_RAD && !recentGoodRotation) {
                _lastGoodGeoYawMs = nowMs;
                updateAbsoluteCandidate(ImuYawSource::GEOMAGNETIC_ROTATION_VECTOR, yaw, robotYaw, accRad, nowMs);
            } else if (!recentGoodRotation && _yawSource == ImuYawSource::GEOMAGNETIC_ROTATION_VECTOR) {
                _yawSource = ImuYawSource::GEOMAGNETIC_ROTATION_VECTOR;
                _sourceYawDeg = yaw;
                _rawYawDeg = yaw;
                _baseYawDeg = robotYaw;
                recomputeFinalYaw();
                _yawAccRad = accRad;
                _has = true;
                _lastMs = nowMs;
                _lastYawMs = nowMs;
                _headingState = ImuHeadingState::IMU_ABSOLUTE_UNCALIBRATED;
            }
            break;
        }
        case SH2_ROTATION_VECTOR: {
            _rotCount++;
            _lastRotYawMs = nowMs;
            float yaw = quatYawDeg(
                _val.un.rotationVector.i,
                _val.un.rotationVector.j,
                _val.un.rotationVector.k,
                _val.un.rotationVector.real);
            _rotYawDeg = yaw;
            const float accRad = _val.un.rotationVector.accuracy;
            const float robotYaw = headingFromSourceYaw(yaw);
            if (accRad <= IMU_ABS_YAW_MAX_ACC_RAD) {
                _lastGoodRotYawMs = nowMs;
                updateAbsoluteCandidate(ImuYawSource::ROTATION_VECTOR, yaw, robotYaw, accRad, nowMs);
            } else if (_yawSource == ImuYawSource::ROTATION_VECTOR ||
                       _lastGoodGeoYawMs == 0 ||
                       (nowMs - _lastGoodGeoYawMs) > (SAFE_IMU_AGE_MS * 2u)) {
                _yawSource = ImuYawSource::ROTATION_VECTOR;
                _sourceYawDeg = yaw;
                _rawYawDeg = yaw;
                _baseYawDeg = robotYaw;
                recomputeFinalYaw();
                _yawAccRad = accRad;
                _has = true;
                _lastMs = nowMs;
                _lastYawMs = nowMs;
                _headingState = ImuHeadingState::IMU_ABSOLUTE_UNCALIBRATED;
            }
            break;
        }
        case SH2_GAME_ROTATION_VECTOR: {
            _gameCount++;
            float yaw = quatYawDeg(
                _val.un.gameRotationVector.i,
                _val.un.gameRotationVector.j,
                _val.un.gameRotationVector.k,
                _val.un.gameRotationVector.real);
            _gameYawDeg = yaw;
            _has = true;
            _lastMs = nowMs;
            if (_yawSource == ImuYawSource::NONE ||
                _headingState == ImuHeadingState::IMU_NO_DATA ||
                _headingState == ImuHeadingState::IMU_RELATIVE_ONLY) {
                _yawSource = ImuYawSource::GAME_ROTATION_VECTOR;
                _sourceYawDeg = yaw;
                _rawYawDeg = yaw;
                _baseYawDeg = headingFromSourceYaw(yaw);
                recomputeFinalYaw();
                _yawAccRad = 999.0f;
                _lastYawMs = nowMs;
                _headingState = ImuHeadingState::IMU_RELATIVE_ONLY;
            }
            break;
        }
        case SH2_GYROSCOPE_CALIBRATED: {
            _gyroCount++;
            _gyroXDps = _val.un.gyroscope.x * 180.0f / M_PI;
            _gyroYDps = _val.un.gyroscope.y * 180.0f / M_PI;
            _gyroZDps = _val.un.gyroscope.z * 180.0f / M_PI;
            if (_lastGyroMs != 0) {
                const float dt = (nowMs - _lastGyroMs) * 0.001f;
                if (dt > 0.0f && dt < 0.2f) {
                    _gyroIntXDeg += _gyroXDps * dt;
                    _gyroIntYDeg += _gyroYDps * dt;
                    _gyroIntZDeg += _gyroZDps * dt;
                }
            }
            _lastGyroMs = nowMs;
            _yawRateDps = _gyroZDps * (float)IMU_YAW_RATE_SIGN;
            _lastMs = nowMs;
            break;
        }
        default:
            break;
    }

    updateHeadingState(nowMs);
}

// Async calibration start. sh2_startCal() inside Adafruit_BNO08x can hang
// the caller (blocking retry loop on I2C). We run it in a dedicated
// FreeRTOS task on core 0 and poll its result with a hard 1500 ms cap
// so the rover main loop never blocks indefinitely.
//
// Lifecycle:
//   IDLE -> RUNNING -> OK / ERROR / TIMEOUT
// We only allow a new run when the previous one finished (state != RUNNING).
namespace {
struct ImuCalStartArgs {
    int* rcOut;
    volatile bool* doneFlag;
};

void imuCalStartTask(void* arg) {
    ImuCalStartArgs* a = static_cast<ImuCalStartArgs*>(arg);
    Serial.println("[IMU-CAL] before sh2_startCal");
    const int rc = sh2_startCal(10000);
    Serial.printf("[IMU-CAL] after sh2_startCal ok=%d rc=%d\n", (rc == SH2_OK) ? 1 : 0, rc);
    *(a->rcOut) = rc;
    *(a->doneFlag) = true;
    vTaskDelete(nullptr);
}
}  // namespace

bool Imu::startCalibration(String* err) {
    // sh2_startCal() inside Adafruit_BNO08x can block the caller for an
    // unbounded time (retry loop on a wedged I2C). Until the underlying
    // library is fixed, refuse rather than hang the rover main loop.
    _calStartState = CalStartState::ERROR;
    _calStartRc = -100;
    _calStartMs = millis();
    if (err) *err = "sh2_startCal_disabled_blocking_api";
    return false;
}

#if 0
// The full non-blocking implementation is preserved below for the day
// when sh2_startCal() is replaced or wrapped safely. It compiles fine
// behind #if 0 and keeps the surrounding async infrastructure (state
// machine + worker task trampoline) ready to use.
bool Imu::startCalibrationAsyncImpl(String* err) {
    if (!_running) {
        if (err) *err = "imu_not_initialized";
        _calStartState = CalStartState::ERROR;
        _calStartRc = -999;
        _calStartMs = millis();
        return false;
    }

    // If a previous async run is still in flight, refuse rather than
    // racing. The caller will see IMU_CAL_START,ERR,busy and may retry.
    if (_calStartState == CalStartState::RUNNING) {
        if (err) *err = "calibration_already_running";
        return false;
    }

    _calStartState = CalStartState::RUNNING;
    _calStartMs = millis();
    _calStartRc = 0;
    _calTaskHandle = nullptr;

    int rcLocal = 0;
    volatile bool done = false;
    ImuCalStartArgs args{&rcLocal, &done};

    BaseType_t ok = xTaskCreatePinnedToCore(
        imuCalStartTask,
        "imuCalStart",
        4096,
        &args,
        3,
        (TaskHandle_t*)&_calTaskHandle,
        0 /* core */);
    if (ok != pdPASS) {
        _calStartState = CalStartState::ERROR;
        _calStartRc = -1;
        if (err) *err = "task_create_failed";
        return false;
    }

    // Hard cap: 1500 ms. We do not wait on the task handle (deleting a
    // task that is stuck in I2C is unsafe), we just let it run and
    // forget. Worst case the next IMU_CAL_START will be rejected as
    // busy until the lib eventually returns.
    constexpr uint32_t CAL_START_TIMEOUT_MS = 1500u;
    const uint32_t t0 = millis();
    while (!done && (millis() - t0) < CAL_START_TIMEOUT_MS) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (!done) {
        _calStartState = CalStartState::TIMEOUT;
        _calStartRc = -2;
        Serial.println("[IMU-CAL] start timeout; SH2 calibration call did not return");
        if (err) *err = "timeout";
        // Intentionally do not block on _calTaskHandle. The orphaned
        // task may eventually return; further IMU_CAL_STARTs will be
        // rejected as `calibration_already_running` until it does.
        return false;
    }

    _calStartRc = rcLocal;
    if (rcLocal == SH2_OK) {
        _calStartState = CalStartState::OK;
        return true;
    }
    _calStartState = CalStartState::ERROR;
    if (err) *err = "sh2_startCal failed rc=" + String(rcLocal);
    return false;
}
#endif  // disabled async impl, see startCalibration() above

bool Imu::saveCalibration(String* err) {
    sh2_CalStatus_t status;
    const int finishRc = sh2_finishCal(&status);
    if (finishRc != SH2_OK) {
        if (err) *err = "sh2_finishCal failed rc=" + String(finishRc);
        return false;
    }
    const int saveRc = sh2_saveDcdNow();
    if (saveRc != SH2_OK) {
        if (err) *err = "sh2_saveDcdNow failed rc=" + String(saveRc);
        return false;
    }
    return true;
}

bool Imu::clearCalibration(String* err) {
    const int rc = sh2_clearDcdAndReset();
    if (rc == SH2_OK) return true;
    if (err) *err = "sh2_clearDcdAndReset failed rc=" + String(rc);
    return false;
}

bool Imu::tareYaw(bool persist, String* err) {
    const sh2_TareBasis_t basis =
        (_yawSource == ImuYawSource::GAME_ROTATION_VECTOR)
            ? SH2_TARE_BASIS_GAMING_ROTATION_VECTOR
            : (_yawSource == ImuYawSource::GEOMAGNETIC_ROTATION_VECTOR)
                ? SH2_TARE_BASIS_GEOMAGNETIC_ROTATION_VECTOR
                : SH2_TARE_BASIS_ROTATION_VECTOR;
    const int rc = sh2_setTareNow(SH2_TARE_Z, basis);
    if (rc != SH2_OK) {
        if (err) *err = "sh2_setTareNow failed rc=" + String(rc);
        return false;
    }
    if (persist) {
        const int prc = sh2_persistTare();
        if (prc != SH2_OK) {
            if (err) *err = "sh2_persistTare failed rc=" + String(prc);
            return false;
        }
    }
    return true;
}

bool Imu::clearTare(String* err) {
    const int rc = sh2_clearTare();
    if (rc == SH2_OK) return true;
    if (err) *err = "sh2_clearTare failed rc=" + String(rc);
    return false;
}

bool Imu::clearDcdAndReset(String* err) {
    const int rc = sh2_clearDcdAndReset();
    if (rc == SH2_OK) return true;
    if (err) *err = "sh2_clearDcdAndReset failed rc=" + String(rc);
    return false;
}

bool Imu::persistTare(String* err) {
    const int rc = sh2_persistTare();
    if (rc == SH2_OK) return true;
    if (err) *err = "sh2_persistTare failed rc=" + String(rc);
    return false;
}

bool Imu::setDynamicCalibration(bool enabled, String* err) {
    const int rc = sh2_setDcdAutoSave(enabled);
    if (rc == SH2_OK) return true;
    if (err) *err = String("sh2_setDcdAutoSave(") + (enabled ? "true" : "false") +
                     ") failed rc=" + String(rc);
    return false;
}

bool Imu::saveDcdNow(String* err) {
    const int rc = sh2_saveDcdNow();
    if (rc == SH2_OK) return true;
    if (err) *err = "sh2_saveDcdNow failed rc=" + String(rc);
    return false;
}
