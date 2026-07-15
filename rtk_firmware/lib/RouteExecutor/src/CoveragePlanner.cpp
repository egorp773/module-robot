#include "CoveragePlanner.h"

#include "RouteGeometry.h"

#include <cmath>

namespace routeexec {

CoveragePlanResult buildRectangularSerpentine(const CoveragePlanConfig& config,
                                              uint32_t routeId,
                                              RoutePlan& output) {
    if (!finitePoint(config.start) ||
        !std::isfinite(config.firstLaneHeadingDeg) ||
        !std::isfinite(config.laneLengthM) || config.laneLengthM <= 0.05f ||
        config.laneCount == 0u ||
        !std::isfinite(config.toolWorkingWidthM) || config.toolWorkingWidthM <= 0.0f ||
        !std::isfinite(config.laneSpacingM) || config.laneSpacingM <= 0.0f ||
        !std::isfinite(config.laneSpeedMps) || config.laneSpeedMps <= 0.0f ||
        !std::isfinite(config.transitionSpeedMps) || config.transitionSpeedMps <= 0.0f ||
        !std::isfinite(config.corridorHalfWidthM) || config.corridorHalfWidthM <= 0.0f ||
        !std::isfinite(config.finalPreApproachM) ||
        config.finalPreApproachM <= 0.05f ||
        (config.finalType != WaypointType::FINAL_POSITION &&
         config.finalType != WaypointType::FINAL_POSE)) {
        return CoveragePlanResult::INVALID_ARGUMENT;
    }
    const size_t requiredPoints = static_cast<size_t>(config.laneCount) * 2u +
        (config.finalType == WaypointType::FINAL_POSE ? 1u : 0u);
    if (requiredPoints > RoutePlan::MAX_POINTS)
        return CoveragePlanResult::TOO_MANY_POINTS;

    output.clear();
    output.setRouteId(routeId);
    RoutePoint start;
    start.position = config.start;
    start.type = WaypointType::PASS_THROUGH;
    start.positionToleranceM = config.cornerToleranceM;
    if (!output.appendPoint(start)) return CoveragePlanResult::TOO_MANY_POINTS;

    LocalPoint laneStart = config.start;
    float laneHeading = normalizeHeadingDeg(config.firstLaneHeadingDeg);
    const float shiftHeading = normalizeHeadingDeg(
        config.firstLaneHeadingDeg + (config.shiftLeft ? -90.0f : 90.0f));

    for (uint16_t lane = 0u; lane < config.laneCount; ++lane) {
        const bool lastLane = lane + 1u == config.laneCount;
        const LocalPoint laneEnd = pointAtHeading(
            laneStart, laneHeading, config.laneLengthM);
        if (lastLane && config.finalType == WaypointType::FINAL_POSE) {
            if (config.laneLengthM <= config.finalPreApproachM + 0.05f ||
                std::fabs(wrapHeadingErrorDeg(
                    laneHeading - config.finalHeadingDeg)) >
                    config.finalHeadingToleranceDeg) {
                return CoveragePlanResult::INVALID_ARGUMENT;
            }
            RoutePoint preApproach;
            preApproach.position = finalPosePreApproach(
                laneEnd, config.finalHeadingDeg, config.finalPreApproachM);
            preApproach.type = WaypointType::CORNER;
            preApproach.positionToleranceM = config.cornerToleranceM;
            if (!output.appendPoint(preApproach, SegmentType::COVERAGE_LANE,
                                    config.corridorHalfWidthM,
                                    config.laneSpeedMps)) {
                return CoveragePlanResult::TOO_MANY_POINTS;
            }

            RoutePoint finalPose;
            finalPose.position = laneEnd;
            finalPose.type = WaypointType::FINAL_POSE;
            finalPose.positionToleranceM = config.finalToleranceM;
            finalPose.finalHeadingRequired = true;
            finalPose.finalHeadingDeg = normalizeHeadingDeg(
                config.finalHeadingDeg);
            finalPose.headingToleranceDeg = config.finalHeadingToleranceDeg;
            if (!output.appendPoint(finalPose,
                                    SegmentType::TERMINAL_APPROACH,
                                    config.corridorHalfWidthM,
                                    config.transitionSpeedMps)) {
                return CoveragePlanResult::TOO_MANY_POINTS;
            }
            break;
        }
        RoutePoint laneEndPoint;
        laneEndPoint.position = laneEnd;
        laneEndPoint.type = lastLane ? config.finalType : WaypointType::CORNER;
        laneEndPoint.positionToleranceM = lastLane
            ? config.finalToleranceM : config.cornerToleranceM;
        laneEndPoint.finalHeadingRequired =
            lastLane && config.finalType == WaypointType::FINAL_POSE;
        laneEndPoint.finalHeadingDeg = normalizeHeadingDeg(config.finalHeadingDeg);
        laneEndPoint.headingToleranceDeg = config.finalHeadingToleranceDeg;
        if (!output.appendPoint(laneEndPoint, SegmentType::COVERAGE_LANE,
                                config.corridorHalfWidthM,
                                config.laneSpeedMps)) {
            return CoveragePlanResult::TOO_MANY_POINTS;
        }
        if (lastLane) break;

        const LocalPoint shifted = pointAtHeading(
            laneEnd, shiftHeading, config.laneSpacingM);
        RoutePoint shiftEnd;
        shiftEnd.position = shifted;
        shiftEnd.type = WaypointType::CORNER;
        shiftEnd.positionToleranceM = config.cornerToleranceM;
        if (!output.appendPoint(shiftEnd, SegmentType::COVERAGE_SHIFT,
                                config.corridorHalfWidthM,
                                config.transitionSpeedMps)) {
            return CoveragePlanResult::TOO_MANY_POINTS;
        }
        laneStart = shifted;
        laneHeading = normalizeHeadingDeg(laneHeading + 180.0f);
    }

    return output.finalize() ? CoveragePlanResult::OK
                             : CoveragePlanResult::ROUTE_FINALIZE_FAILED;
}

}  // namespace routeexec
