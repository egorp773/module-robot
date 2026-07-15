#include "RouteGeometry.h"

#include <cmath>

namespace routeexec {

float clampFloat(float value, float lo, float hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

float distance(const LocalPoint& a, const LocalPoint& b) {
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

float wrapHeadingErrorDeg(float degrees) {
    while (degrees > 180.0f) degrees -= 360.0f;
    while (degrees < -180.0f) degrees += 360.0f;
    return degrees;
}

float bearingDeg(const LocalPoint& from, const LocalPoint& to) {
    return headingForVector(to.x - from.x, to.y - from.y);
}

LocalPoint pointAtHeading(const LocalPoint& from,
                          float headingDeg,
                          float distanceM) {
    static const float kDegToRad = 0.01745329251994329577f;
    const float radians = headingDeg * kDegToRad;
    return LocalPoint(from.x + std::sin(radians) * distanceM,
                      from.y + std::cos(radians) * distanceM);
}

LocalPoint finalPosePreApproach(const LocalPoint& finalPosition,
                                float finalHeadingDeg,
                                float preApproachDistanceM) {
    return pointAtHeading(finalPosition, finalHeadingDeg, -preApproachDistanceM);
}

SegmentProjection projectToSegment(const RouteSegment& segment,
                                   const LocalPoint& position) {
    SegmentProjection projection;
    const float dx = segment.plannedEnd.x - segment.plannedStart.x;
    const float dy = segment.plannedEnd.y - segment.plannedStart.y;
    const float rx = position.x - segment.plannedStart.x;
    const float ry = position.y - segment.plannedStart.y;
    const float length = segment.segmentLengthM;
    if (length <= 0.0f) return projection;

    projection.alongTrackM = (rx * dx + ry * dy) / length;
    projection.crossTrackM = (dx * ry - dy * rx) / length;
    projection.distanceToEndM = distance(position, segment.plannedEnd);
    const float planeDx = position.x - segment.transitionPlanePoint.x;
    const float planeDy = position.y - segment.transitionPlanePoint.y;
    projection.crossedTransitionPlane =
        planeDx * segment.transitionNormal.x +
        planeDy * segment.transitionNormal.y >= 0.0f;
    return projection;
}

InterceptSolution computeInterceptTarget(const RouteSegment& segment,
                                         const LocalPoint& position,
                                         float lookaheadM) {
    InterceptSolution solution;
    const SegmentProjection projection = projectToSegment(segment, position);
    solution.targetAlongTrackM = clampFloat(
        projection.alongTrackM + lookaheadM, 0.0f, segment.segmentLengthM);
    const float ratio = segment.segmentLengthM > 0.0f
        ? solution.targetAlongTrackM / segment.segmentLengthM
        : 0.0f;
    solution.target.x = segment.plannedStart.x +
        (segment.plannedEnd.x - segment.plannedStart.x) * ratio;
    solution.target.y = segment.plannedStart.y +
        (segment.plannedEnd.y - segment.plannedStart.y) * ratio;
    solution.bearingDeg = bearingDeg(position, solution.target);
    return solution;
}

uint32_t elapsedItowMillis(uint32_t newerItowMs,
                           uint32_t olderItowMs,
                           uint32_t gpsWeekMs) {
    if (gpsWeekMs == 0u) return 0u;
    newerItowMs %= gpsWeekMs;
    olderItowMs %= gpsWeekMs;
    if (newerItowMs >= olderItowMs) return newerItowMs - olderItowMs;
    return gpsWeekMs - olderItowMs + newerItowMs;
}

}  // namespace routeexec
