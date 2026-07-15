#pragma once

#include "RouteTypes.h"

#include <cstddef>

namespace routeexec {

struct FootprintConfig {
    RobotFootprint body;
    RobotFootprint tool;
    bool toolConfigured;

    FootprintConfig() : body(), tool(), toolConfigured(false) {}
};

enum class FootprintCheckResult : uint8_t {
    CLEAR = 0,
    INVALID_ARGUMENT,
    TOOL_FOOTPRINT_UNCONFIGURED,
    BLOCKED,
};

typedef bool (*PointAllowedFunction)(const LocalPoint& point, void* context);

RobotFootprint combinedFootprint(const FootprintConfig& config);

// Center, four corners and four side midpoints. The fixed output capacity is
// intentionally explicit for stack/static use in the fast loop.
size_t footprintPoseSamples(const Pose2D& pose,
                            const RobotFootprint& footprint,
                            LocalPoint* output,
                            size_t outputCapacity);

FootprintCheckResult checkFootprintPose(const Pose2D& pose,
                                        const RobotFootprint& footprint,
                                        PointAllowedFunction pointAllowed,
                                        void* context);

// Samples every pose along the predicted straight reverse path, including both
// endpoints. If a tool footprint is unknown, reverse near a restricted area is
// rejected instead of guessing its forward overhang.
FootprintCheckResult checkReverseSweptFootprint(
    const Pose2D& startPose,
    float reverseDistanceM,
    float poseSampleStepM,
    const FootprintConfig& footprint,
    bool nearRestrictedArea,
    PointAllowedFunction pointAllowed,
    void* context);

}  // namespace routeexec
