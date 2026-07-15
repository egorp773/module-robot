#pragma once

#include "RouteTypes.h"

namespace routeexec {

struct CoveragePlanConfig {
    LocalPoint start;
    float firstLaneHeadingDeg;
    float laneLengthM;
    uint16_t laneCount;
    float toolWorkingWidthM;
    float laneSpacingM;
    bool shiftLeft;
    float laneSpeedMps;
    float transitionSpeedMps;
    float corridorHalfWidthM;
    float cornerToleranceM;
    float finalToleranceM;
    WaypointType finalType;
    float finalHeadingDeg;
    float finalHeadingToleranceDeg;
    float finalPreApproachM;

    CoveragePlanConfig()
        : start(),
          firstLaneHeadingDeg(0.0f),
          laneLengthM(1.0f),
          laneCount(1u),
          toolWorkingWidthM(0.50f),
          laneSpacingM(0.42f),
          shiftLeft(true),
          laneSpeedMps(0.11f),
          transitionSpeedMps(0.07f),
          corridorHalfWidthM(0.15f),
          cornerToleranceM(0.11f),
          finalToleranceM(0.09f),
          finalType(WaypointType::FINAL_POSITION),
          finalHeadingDeg(0.0f),
          finalHeadingToleranceDeg(7.0f),
          finalPreApproachM(0.40f) {}
};

enum class CoveragePlanResult : uint8_t {
    OK = 0,
    INVALID_ARGUMENT,
    TOO_MANY_POINTS,
    ROUTE_FINALIZE_FAILED,
};

// Builds fixed parallel lanes. Between adjacent lanes the polyline contains a
// perpendicular lane-spacing segment, so the executor performs the reliable
// stop/90/shift/stop/90/intercept sequence rather than recovery improvisation.
CoveragePlanResult buildRectangularSerpentine(const CoveragePlanConfig& config,
                                              uint32_t routeId,
                                              RoutePlan& output);

}  // namespace routeexec
