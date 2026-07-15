#include "FootprintGeometry.h"

#include <cmath>

namespace routeexec {

namespace {

float maxFloat(float a, float b) { return a > b ? a : b; }

LocalPoint offsetPoint(const Pose2D& pose, float forwardM, float leftM) {
    static const float kDegToRad = 0.01745329251994329577f;
    const float radians = pose.headingDeg * kDegToRad;
    const float forwardX = std::sin(radians);
    const float forwardY = std::cos(radians);
    const float leftX = -std::cos(radians);
    const float leftY = std::sin(radians);
    return LocalPoint(pose.position.x + forwardX * forwardM + leftX * leftM,
                      pose.position.y + forwardY * forwardM + leftY * leftM);
}

}  // namespace

RobotFootprint combinedFootprint(const FootprintConfig& config) {
    RobotFootprint result = config.body;
    if (config.toolConfigured) {
        result.frontM = maxFloat(result.frontM, config.tool.frontM);
        result.rearM = maxFloat(result.rearM, config.tool.rearM);
        result.leftM = maxFloat(result.leftM, config.tool.leftM);
        result.rightM = maxFloat(result.rightM, config.tool.rightM);
    }
    return result;
}

size_t footprintPoseSamples(const Pose2D& pose,
                            const RobotFootprint& footprint,
                            LocalPoint* output,
                            size_t outputCapacity) {
    static const size_t kRequired = 9u;
    if (output == 0 || outputCapacity < kRequired ||
        footprint.frontM < 0.0f || footprint.rearM < 0.0f ||
        footprint.leftM < 0.0f || footprint.rightM < 0.0f) return 0u;

    output[0] = pose.position;
    output[1] = offsetPoint(pose, footprint.frontM, footprint.leftM);
    output[2] = offsetPoint(pose, footprint.frontM, -footprint.rightM);
    output[3] = offsetPoint(pose, -footprint.rearM, footprint.leftM);
    output[4] = offsetPoint(pose, -footprint.rearM, -footprint.rightM);
    output[5] = offsetPoint(pose, footprint.frontM, 0.0f);
    output[6] = offsetPoint(pose, -footprint.rearM, 0.0f);
    output[7] = offsetPoint(pose, 0.0f, footprint.leftM);
    output[8] = offsetPoint(pose, 0.0f, -footprint.rightM);
    return kRequired;
}

FootprintCheckResult checkFootprintPose(const Pose2D& pose,
                                        const RobotFootprint& footprint,
                                        PointAllowedFunction pointAllowed,
                                        void* context) {
    if (pointAllowed == 0 || !finitePoint(pose.position) ||
        !std::isfinite(pose.headingDeg)) {
        return FootprintCheckResult::INVALID_ARGUMENT;
    }
    LocalPoint samples[9];
    const size_t count = footprintPoseSamples(pose, footprint, samples, 9u);
    if (count == 0u) return FootprintCheckResult::INVALID_ARGUMENT;
    for (size_t i = 0u; i < count; ++i) {
        if (!pointAllowed(samples[i], context)) return FootprintCheckResult::BLOCKED;
    }
    return FootprintCheckResult::CLEAR;
}

FootprintCheckResult checkReverseSweptFootprint(
    const Pose2D& startPose,
    float reverseDistanceM,
    float poseSampleStepM,
    const FootprintConfig& footprint,
    bool nearRestrictedArea,
    PointAllowedFunction pointAllowed,
    void* context) {
    if (!finitePoint(startPose.position) || !std::isfinite(startPose.headingDeg) ||
        !std::isfinite(reverseDistanceM) || reverseDistanceM < 0.0f ||
        !std::isfinite(poseSampleStepM) || poseSampleStepM <= 0.0f ||
        pointAllowed == 0) return FootprintCheckResult::INVALID_ARGUMENT;
    if (!footprint.toolConfigured && nearRestrictedArea)
        return FootprintCheckResult::TOOL_FOOTPRINT_UNCONFIGURED;

    const RobotFootprint total = combinedFootprint(footprint);
    size_t stepCount = static_cast<size_t>(
        std::ceil(reverseDistanceM / poseSampleStepM));
    if (stepCount < 1u) stepCount = 1u;
    for (size_t i = 0u; i <= stepCount; ++i) {
        const float ratio = static_cast<float>(i) /
                            static_cast<float>(stepCount);
        Pose2D pose = startPose;
        pose.position = offsetPoint(startPose, -reverseDistanceM * ratio, 0.0f);
        const FootprintCheckResult result =
            checkFootprintPose(pose, total, pointAllowed, context);
        if (result != FootprintCheckResult::CLEAR) return result;
    }
    return FootprintCheckResult::CLEAR;
}

}  // namespace routeexec
