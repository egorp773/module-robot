#pragma once

#include <math.h>
#include <stdint.h>

namespace motorcmd {

// Legacy API name retained for the existing deadband callers.  `left` and
// `right` are hoverboard virtual channels, not universally normalized physical
// wheel velocities; use MotionMixMode when interpreting their differential.
struct WheelPercent {
    int left;
    int right;
};

struct HoverChannelSpeed {
    float leftMps;
    float rightMps;
};

struct HoverAxes {
    int speed;
    int steer;
};

enum class MotionMixMode : uint8_t {
    DRIVE = 0,
    TURN_IN_PLACE,
};

struct CommandMix {
    WheelPercent requested;
    WheelPercent effective;
    MotionMixMode mode;
};

inline int absPercent(int value) {
    return value < 0 ? -value : value;
}

inline int clampPercent(int value, int maxAbs) {
    if (value > maxAbs) return maxAbs;
    if (value < -maxAbs) return -maxAbs;
    return value;
}

// Motor/navigation sign contract (proved by real drive_sync* field runs and
// the latest autonomous logs; the app arrow comments are stale evidence):
//
//   headingDeg / yawRateDps / angularRadps:
//       0 deg = North, 90 deg = East, clockwise is positive.
//   left/right percent:
//       these are legacy hoverboard virtual channels, not normalized physical
//       track velocities.  A same-sign positive pair means physical forward
//       translation.  An individual channel has no mode-independent physical
//       track meaning; its differential sign is empirically mode-dependent.
//   clockwise-positive angularRadps in DRIVE:
//       left virtual channel > right virtual channel (drive_sync3 and the
//       field smoking gun: the former right>left mix changed heading negative).
//   clockwise-positive angularRadps in TURN_IN_PLACE:
//       left virtual channel < 0, right virtual channel > 0 (drive_sync*,
//       drive_phase and the successful real corner-turn traces).
//   hoverboard UART axes (the proven sound/sound.ino protocol mapping):
//       speed=(left+right)/2, steer=(right-left)/2.  Consequently a
//       clockwise DRIVE produces negative packet steer, while clockwise
//       TURN_IN_PLACE produces positive packet steer on this controller.
//
// The previous autonomous mixer used vL=v-w*b/2, vR=v+w*b/2.  That is the
// verified turn-in-place mapping, but applying it to translational DRIVE
// inverted steering.  Keep the two adapters explicit; do not infer physical
// track signs from these virtual channel names.
inline HoverChannelSpeed mixCompassLinearAngular(
    float linearMps,
    float angularRadps,
    float wheelBaseM,
    MotionMixMode mode,
    float movingWheelFloorFraction = 0.15f) {
    const float halfDifferential = angularRadps * wheelBaseM * 0.5f;
    HoverChannelSpeed out;
    if (mode == MotionMixMode::TURN_IN_PLACE) {
        out.leftMps = -halfDifferential;
        out.rightMps = halfDifferential;
        return out;
    }
    out.leftMps = linearMps + halfDifferential;
    out.rightMps = linearMps - halfDifferential;

    // A non-zero linear command means translational DRIVE, not a pivot.  Keep
    // both tracks moving in that direction using the pre-existing 15% floor.
    // Applying the floor unconditionally (instead of only after a sign cross)
    // also prevents a small positive wheel speed rounding to intentional zero.
    if (linearMps > 0.0f) {
        const float floorMps = linearMps * movingWheelFloorFraction;
        if (out.leftMps < floorMps) out.leftMps = floorMps;
        if (out.rightMps < floorMps) out.rightMps = floorMps;
    } else if (linearMps < 0.0f) {
        const float ceilingMps = linearMps * movingWheelFloorFraction;
        if (out.leftMps > ceilingMps) out.leftMps = ceilingMps;
        if (out.rightMps > ceilingMps) out.rightMps = ceilingMps;
    }
    return out;
}

inline WheelPercent hoverChannelSpeedToPercent(
    const HoverChannelSpeed& speed,
    float percentPerMps,
    int maxAbsPercent) {
    return WheelPercent{
        clampPercent((int)roundf(speed.leftMps * percentPerMps),
                     maxAbsPercent),
        clampPercent((int)roundf(speed.rightMps * percentPerMps),
                     maxAbsPercent),
    };
}

inline WheelPercent mixCompassLinearAngularPercent(
    float linearMps,
    float angularRadps,
    float wheelBaseM,
    float percentPerMps,
    int maxAbsPercent,
    MotionMixMode mode,
    float movingWheelFloorFraction = 0.15f) {
    WheelPercent out = hoverChannelSpeedToPercent(
        mixCompassLinearAngular(linearMps, angularRadps, wheelBaseM,
                                mode, movingWheelFloorFraction),
        percentPerMps, maxAbsPercent);
    // Preserve the semantic distinction between translational DRIVE and an
    // intentional pivot after integer quantisation.  The following deadband
    // stage can then lift both non-zero tracks together without inventing
    // motion for an explicitly requested wheel zero.
    if (mode == MotionMixMode::DRIVE && linearMps > 0.0f) {
        if (out.left <= 0) out.left = 1;
        if (out.right <= 0) out.right = 1;
    } else if (mode == MotionMixMode::DRIVE && linearMps < 0.0f) {
        if (out.left >= 0) out.left = -1;
        if (out.right >= 0) out.right = -1;
    }
    return out;
}

inline int expectedClockwiseHeadingChange(const WheelPercent& channels,
                                          MotionMixMode mode) {
    const int differential = mode == MotionMixMode::DRIVE
        ? channels.left - channels.right
        : channels.right - channels.left;
    return differential > 0 ? 1 : (differential < 0 ? -1 : 0);
}

inline HoverAxes hoverChannelsToAxes(int leftPercent,
                                     int rightPercent,
                                     int maxCommand,
                                     int maxPercent) {
    HoverAxes out;
    out.speed = (leftPercent + rightPercent) * maxCommand /
                (2 * maxPercent);
    out.steer = (rightPercent - leftPercent) * maxCommand /
                (2 * maxPercent);
    return out;
}

// Adds only feed-forward needed to cross the physical forward deadband.
// For two moving wheels the offset is common-mode, so (right-left), its
// sign, and therefore the requested curvature are preserved.  A requested
// zero is intentional and is never turned into motion.  Opposite-sign wheel
// pairs are turn-in-place commands and are deliberately left untouched.
inline WheelPercent compensateForwardDeadband(int requestedLeft,
                                               int requestedRight,
                                               int minEffectiveLeft,
                                               int minEffectiveRight,
                                               int maxAbsPercent,
                                               bool forwardMode) {
    WheelPercent out{
        clampPercent(requestedLeft, maxAbsPercent),
        clampPercent(requestedRight, maxAbsPercent),
    };
    if (!forwardMode || (out.left == 0 && out.right == 0)) return out;

    // Intentional pivot/inside-wheel zero.  Compensate only the wheel that
    // was actually requested; 0/4 must become 0/5, never 5/9.
    if (out.left == 0 || out.right == 0) {
        if (out.left != 0 && absPercent(out.left) < minEffectiveLeft) {
            out.left = out.left > 0 ? minEffectiveLeft : -minEffectiveLeft;
        }
        if (out.right != 0 && absPercent(out.right) < minEffectiveRight) {
            out.right = out.right > 0 ? minEffectiveRight : -minEffectiveRight;
        }
        out.left = clampPercent(out.left, maxAbsPercent);
        out.right = clampPercent(out.right, maxAbsPercent);
        return out;
    }

    const bool sameDirection = (out.left > 0) == (out.right > 0);
    if (!sameDirection) return out;

    const int needLeft = minEffectiveLeft - absPercent(out.left);
    const int needRight = minEffectiveRight - absPercent(out.right);
    int wantedOffset = needLeft > needRight ? needLeft : needRight;
    if (wantedOffset <= 0) return out;

    const int maxAbs = absPercent(out.left) > absPercent(out.right)
        ? absPercent(out.left) : absPercent(out.right);
    const int headroom = maxAbsPercent - maxAbs;
    if (wantedOffset > headroom) wantedOffset = headroom;
    if (wantedOffset <= 0) return out;

    const int direction = out.left > 0 ? 1 : -1;
    out.left += direction * wantedOffset;
    out.right += direction * wantedOffset;
    return out;
}

// Turn-in-place has no common/linear component, so forward common-mode
// compensation must not be used.  Raise the two opposite-sign tracks to one
// shared physically-effective magnitude.  This is deliberately symmetric:
// it does not encode an unproven left/right mechanical bias.
inline WheelPercent compensateTurnInPlaceDeadband(int requestedLeft,
                                                  int requestedRight,
                                                  int minEffectiveLeft,
                                                  int minEffectiveRight,
                                                  int maxAbsPercent) {
    WheelPercent out{
        clampPercent(requestedLeft, maxAbsPercent),
        clampPercent(requestedRight, maxAbsPercent),
    };
    if (out.left == 0 && out.right == 0) return out;
    if (out.left == 0 || out.right == 0 ||
        ((out.left > 0) == (out.right > 0))) {
        return out;
    }

    int magnitude = absPercent(out.left);
    if (absPercent(out.right) > magnitude)
        magnitude = absPercent(out.right);
    if (minEffectiveLeft > magnitude) magnitude = minEffectiveLeft;
    if (minEffectiveRight > magnitude) magnitude = minEffectiveRight;
    if (magnitude > maxAbsPercent) magnitude = maxAbsPercent;

    out.left = out.left > 0 ? magnitude : -magnitude;
    out.right = out.right > 0 ? magnitude : -magnitude;
    return out;
}

// Complete pure command path used by Motor::setLinearAngularSpeed(). Keeping
// mixing and the mode-specific deadband stage together prevents host tests
// from validating a sequence different from the firmware's actual sequence.
inline CommandMix mixEffectiveCompassCommandPercent(
    float linearMps,
    float angularRadps,
    float wheelBaseM,
    float percentPerMps,
    int minEffectiveLeft,
    int minEffectiveRight,
    int maxAbsPercent,
    MotionMixMode mode,
    float movingWheelFloorFraction = 0.15f) {
    CommandMix out{
        mixCompassLinearAngularPercent(
            linearMps, angularRadps, wheelBaseM, percentPerMps,
            maxAbsPercent, mode, movingWheelFloorFraction),
        WheelPercent{0, 0},
        mode,
    };
    out.effective = mode == MotionMixMode::DRIVE
        ? compensateForwardDeadband(
              out.requested.left, out.requested.right,
              minEffectiveLeft, minEffectiveRight, maxAbsPercent, true)
        : compensateTurnInPlaceDeadband(
              out.requested.left, out.requested.right,
              minEffectiveLeft, minEffectiveRight, maxAbsPercent);
    return out;
}

}  // namespace motorcmd
