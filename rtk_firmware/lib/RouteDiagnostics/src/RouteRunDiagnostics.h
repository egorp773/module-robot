#pragma once

#include <cmath>

namespace routediag {

// Classification of the displacement interval ending at a new PVT.  The
// caller deliberately supplies the *previous* executor phase: an interval
// which triggered BRAKE still belongs to powered drive, while subsequent
// motion with zero motor command is brake/coast distance.
enum class PathIntervalKind : unsigned char {
    NONE = 0,
    POWERED_DRIVE,
    BRAKE_COAST,
};

struct PoseSample {
    float x = NAN;
    float y = NAN;
    float headingDeg = NAN;
    bool valid = false;
};

inline float wrapDeg180(float angleDeg) {
    while (angleDeg > 180.0f) angleDeg -= 360.0f;
    while (angleDeg <= -180.0f) angleDeg += 360.0f;
    return angleDeg;
}

inline float distance(float ax, float ay, float bx, float by) {
    const float dx = bx - ax;
    const float dy = by - ay;
    return std::sqrt(dx * dx + dy * dy);
}

inline bool captureOnce(PoseSample& destination,
                        float x, float y, float headingDeg) {
    if (destination.valid) return false;
    if (!std::isfinite(x) || !std::isfinite(y) ||
        !std::isfinite(headingDeg)) return false;
    destination.x = x;
    destination.y = y;
    destination.headingDeg = headingDeg;
    destination.valid = true;
    return true;
}

// One planned corner produces exactly these five immutable observations.
// Repeated BRAKE/WAIT states (for example a post-turn correction) must never
// overwrite the approach or turn-start samples.
struct CornerSnapshots {
    PoseSample cornerApproachBrakeStart;
    PoseSample cornerPhysicalStopBeforeTurn;
    PoseSample turnStart;
    PoseSample turnPhysicalStop;
    PoseSample nextSegmentInterceptStart;

    void reset() { *this = CornerSnapshots{}; }

    float turnActualDeg() const {
        if (!turnStart.valid || !turnPhysicalStop.valid) return NAN;
        return wrapDeg180(turnPhysicalStop.headingDeg - turnStart.headingDeg);
    }

    float turnPositionChordM() const {
        if (!turnStart.valid || !turnPhysicalStop.valid) return NAN;
        return distance(turnStart.x, turnStart.y,
                        turnPhysicalStop.x, turnPhysicalStop.y);
    }

    float turnBodyForwardShiftM() const {
        if (!turnStart.valid || !turnPhysicalStop.valid) return NAN;
        constexpr float kDegToRad = 0.01745329251994329577f;
        const float headingRad = turnStart.headingDeg * kDegToRad;
        const float dx = turnPhysicalStop.x - turnStart.x;
        const float dy = turnPhysicalStop.y - turnStart.y;
        // Project convention: x=East, y=North, 0 deg=North, clockwise +.
        return dx * std::sin(headingRad) + dy * std::cos(headingRad);
    }

    float turnBodyLeftShiftM() const {
        if (!turnStart.valid || !turnPhysicalStop.valid) return NAN;
        constexpr float kDegToRad = 0.01745329251994329577f;
        const float headingRad = turnStart.headingDeg * kDegToRad;
        const float dx = turnPhysicalStop.x - turnStart.x;
        const float dy = turnPhysicalStop.y - turnStart.y;
        return -dx * std::cos(headingRad) + dy * std::sin(headingRad);
    }

    float equivalentRadiusM() const {
        constexpr float kDegToRad = 0.01745329251994329577f;
        const float deltaDeg = turnActualDeg();
        const float chord = turnPositionChordM();
        if (!std::isfinite(deltaDeg) || !std::isfinite(chord)) return NAN;
        const float deltaRad = std::fabs(deltaDeg) * kDegToRad;
        if (deltaRad < 0.01f) return NAN;
        return chord / (2.0f * std::sin(deltaRad * 0.5f));
    }
};

struct SegmentPathMetrics {
    PoseSample segmentStart;
    PoseSample brakeStart;
    PoseSample physicalStop;
    float poweredDrivePathM = 0.0f;
    float brakeCoastPathM = 0.0f;
    float alongAtBrakeM = NAN;
    float crossAtBrakeM = NAN;
    float alongAtPhysicalStopM = NAN;
    float crossAtPhysicalStopM = NAN;
    float previousX = NAN;
    float previousY = NAN;
    bool previousPvtValid = false;
    bool closed = false;

    void reset(float startX, float startY, float headingDeg) {
        *this = SegmentPathMetrics{};
        captureOnce(segmentStart, startX, startY, headingDeg);
        previousX = startX;
        previousY = startY;
        previousPvtValid = std::isfinite(startX) && std::isfinite(startY);
    }

    float observePvt(float x, float y, PathIntervalKind intervalKind) {
        if (!std::isfinite(x) || !std::isfinite(y)) return 0.0f;
        float stepM = 0.0f;
        if (!closed && previousPvtValid) {
            stepM = distance(previousX, previousY, x, y);
            if (intervalKind == PathIntervalKind::POWERED_DRIVE)
                poweredDrivePathM += stepM;
            else if (intervalKind == PathIntervalKind::BRAKE_COAST)
                brakeCoastPathM += stepM;
        }
        previousX = x;
        previousY = y;
        previousPvtValid = true;
        return stepM;
    }

    bool captureBrake(float x, float y, float headingDeg,
                      float alongM, float crossM) {
        if (!captureOnce(brakeStart, x, y, headingDeg)) return false;
        alongAtBrakeM = alongM;
        crossAtBrakeM = crossM;
        return true;
    }

    bool capturePhysicalStop(float x, float y, float headingDeg,
                             float alongM, float crossM) {
        if (!captureOnce(physicalStop, x, y, headingDeg)) return false;
        alongAtPhysicalStopM = alongM;
        crossAtPhysicalStopM = crossM;
        closed = true;
        return true;
    }

    float totalTranslationalPathM() const {
        return poweredDrivePathM + brakeCoastPathM;
    }

    float plannedStartToActualStopChordM() const {
        if (!segmentStart.valid || !physicalStop.valid) return NAN;
        return distance(segmentStart.x, segmentStart.y,
                        physicalStop.x, physicalStop.y);
    }

    bool totalPathCoversActualStopChord(float toleranceM = 0.02f) const {
        const float chordM = plannedStartToActualStopChordM();
        return !std::isfinite(chordM) ||
               totalTranslationalPathM() + toleranceM >= chordM;
    }
};

}  // namespace routediag
