#pragma once

#include "RouteTypes.h"

namespace routeexec {

struct SegmentProjection {
    float alongTrackM;
    float crossTrackM;
    float distanceToEndM;
    bool crossedTransitionPlane;

    SegmentProjection()
        : alongTrackM(0.0f),
          crossTrackM(0.0f),
          distanceToEndM(0.0f),
          crossedTransitionPlane(false) {}
};

struct InterceptSolution {
    LocalPoint target;
    float targetAlongTrackM;
    float bearingDeg;

    InterceptSolution()
        : target(), targetAlongTrackM(0.0f), bearingDeg(0.0f) {}
};

float clampFloat(float value, float lo, float hi);
float distance(const LocalPoint& a, const LocalPoint& b);
float wrapHeadingErrorDeg(float degrees);
float bearingDeg(const LocalPoint& from, const LocalPoint& to);
LocalPoint pointAtHeading(const LocalPoint& from,
                          float headingDeg,
                          float distanceM);
LocalPoint finalPosePreApproach(const LocalPoint& finalPosition,
                                float finalHeadingDeg,
                                float preApproachDistanceM);
SegmentProjection projectToSegment(const RouteSegment& segment,
                                   const LocalPoint& position);
InterceptSolution computeInterceptTarget(const RouteSegment& segment,
                                         const LocalPoint& position,
                                         float lookaheadM);

// Both helpers deliberately use modular arithmetic so rollover is not treated
// as stale time or as a backward sample.
inline uint32_t elapsedMillis(uint32_t nowMs, uint32_t sinceMs) {
    return static_cast<uint32_t>(nowMs - sinceMs);
}

uint32_t elapsedItowMillis(uint32_t newerItowMs,
                           uint32_t olderItowMs,
                           uint32_t gpsWeekMs = 604800000u);

inline bool isNewPvtId(uint32_t candidate, uint32_t previous) {
    return candidate != previous;
}

}  // namespace routeexec
