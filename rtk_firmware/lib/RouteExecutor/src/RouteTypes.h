#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace routeexec {

// Local navigation frame: x=East, y=North. Heading is clockwise from North.
struct LocalPoint {
    float x;
    float y;

    LocalPoint() : x(0.0f), y(0.0f) {}
    LocalPoint(float xEast, float yNorth) : x(xEast), y(yNorth) {}
};

struct Pose2D {
    LocalPoint position;
    float headingDeg;

    Pose2D() : position(), headingDeg(0.0f) {}
    Pose2D(const LocalPoint& p, float heading) : position(p), headingDeg(heading) {}
};

// Distances from the robot control point. Tool dimensions are deliberately
// supplied by configuration rather than inferred from antenna motion.
struct RobotFootprint {
    float frontM;
    float rearM;
    float leftM;
    float rightM;

    RobotFootprint()
        : frontM(0.0f), rearM(0.0f), leftM(0.0f), rightM(0.0f) {}
    RobotFootprint(float front, float rear, float left, float right)
        : frontM(front), rearM(rear), leftM(left), rightM(right) {}
};

enum class WaypointType : uint8_t {
    PASS_THROUGH = 0,
    CORNER,
    FINAL_POSITION,
    FINAL_POSE,
    WORK_ACTION,
};

enum class SegmentType : uint8_t {
    LINE = 0,
    COVERAGE_LANE,
    COVERAGE_SHIFT,
    TERMINAL_APPROACH,
};

struct RoutePoint {
    LocalPoint position;
    WaypointType type;
    bool finalHeadingRequired;
    float finalHeadingDeg;
    float positionToleranceM;
    float headingToleranceDeg;

    RoutePoint()
        : position(),
          type(WaypointType::PASS_THROUGH),
          finalHeadingRequired(false),
          finalHeadingDeg(0.0f),
          positionToleranceM(0.10f),
          headingToleranceDeg(7.0f) {}
};

struct RouteSegment {
    LocalPoint plannedStart;
    LocalPoint plannedEnd;
    float plannedHeadingDeg;
    SegmentType type;
    float corridorHalfWidthM;
    float speedLimitMps;
    float segmentLengthM;
    LocalPoint transitionPlanePoint;
    LocalPoint transitionNormal;

    RouteSegment()
        : plannedStart(),
          plannedEnd(),
          plannedHeadingDeg(0.0f),
          type(SegmentType::LINE),
          corridorHalfWidthM(0.15f),
          speedLimitMps(0.11f),
          segmentLengthM(0.0f),
          transitionPlanePoint(),
          transitionNormal() {}
};

inline bool finitePoint(const LocalPoint& p) {
    return std::isfinite(p.x) && std::isfinite(p.y);
}

inline float normalizeHeadingDeg(float degrees) {
    while (degrees < 0.0f) degrees += 360.0f;
    while (degrees >= 360.0f) degrees -= 360.0f;
    return degrees;
}

inline float headingForVector(float dxEast, float dyNorth) {
    static const float kRadToDeg = 57.295779513082320876f;
    return normalizeHeadingDeg(std::atan2(dxEast, dyNorth) * kRadToDeg);
}

// Fixed-capacity, immutable-after-finalize route. No allocation occurs in
// construction or in the navigation loop.
class RoutePlan {
public:
    static const size_t MAX_POINTS = 254u;
    static const size_t MAX_SEGMENTS = MAX_POINTS - 1u;

    RoutePlan() { clear(); }

    void clear() {
        routeId_ = 0u;
        pointCount_ = 0u;
        segmentCount_ = 0u;
        finalized_ = false;
        valid_ = false;
        for (size_t i = 0; i < MAX_SEGMENTS; ++i) {
            incomingType_[i] = SegmentType::LINE;
            incomingCorridor_[i] = 0.15f;
            incomingSpeed_[i] = 0.11f;
        }
    }

    void setRouteId(uint32_t routeId) {
        if (!finalized_) routeId_ = routeId;
    }

    uint32_t routeId() const { return routeId_; }
    size_t pointCount() const { return pointCount_; }
    size_t segmentCount() const { return segmentCount_; }
    bool finalized() const { return finalized_; }
    bool valid() const { return finalized_ && valid_; }

    // The first point is the explicit planned start. For later points the
    // incoming segment metadata describes the segment from the previous point.
    bool appendPoint(const RoutePoint& point,
                     SegmentType incomingType = SegmentType::LINE,
                     float corridorHalfWidthM = 0.15f,
                     float speedLimitMps = 0.11f) {
        if (finalized_ || pointCount_ >= MAX_POINTS) return false;
        if (!finitePoint(point.position) ||
            !std::isfinite(point.finalHeadingDeg) ||
            !std::isfinite(point.positionToleranceM) ||
            !std::isfinite(point.headingToleranceDeg)) return false;
        if (point.positionToleranceM <= 0.0f || point.headingToleranceDeg <= 0.0f)
            return false;
        if (pointCount_ > 0u) {
            if (!std::isfinite(corridorHalfWidthM) || corridorHalfWidthM <= 0.0f ||
                !std::isfinite(speedLimitMps) || speedLimitMps <= 0.0f) return false;
            const size_t incomingIndex = pointCount_ - 1u;
            incomingType_[incomingIndex] = incomingType;
            incomingCorridor_[incomingIndex] = corridorHalfWidthM;
            incomingSpeed_[incomingIndex] = speedLimitMps;
        }
        points_[pointCount_++] = point;
        return true;
    }

    // Compatibility helper for the current app upload format, which contains
    // positions only. The adapter prepends the explicit route start and maps
    // the last uploaded point to FINAL_POSITION.
    bool appendLegacyPosition(const LocalPoint& position,
                              bool isFinal,
                              float positionToleranceM,
                              float corridorHalfWidthM,
                              float speedLimitMps) {
        RoutePoint point;
        point.position = position;
        point.type = isFinal ? WaypointType::FINAL_POSITION
                             : WaypointType::PASS_THROUGH;
        point.finalHeadingRequired = false;
        point.positionToleranceM = positionToleranceM;
        return appendPoint(point, SegmentType::LINE,
                           corridorHalfWidthM, speedLimitMps);
    }

    bool finalize() {
        finalized_ = true;
        valid_ = false;
        segmentCount_ = 0u;
        if (pointCount_ < 2u) return false;

        const RoutePoint& terminal = points_[pointCount_ - 1u];
        if (terminal.type != WaypointType::FINAL_POSITION &&
            terminal.type != WaypointType::FINAL_POSE) return false;
        if (terminal.type == WaypointType::FINAL_POSE &&
            !terminal.finalHeadingRequired) return false;
        if (terminal.type == WaypointType::FINAL_POSITION &&
            terminal.finalHeadingRequired) return false;

        for (size_t i = 0u; i + 1u < pointCount_; ++i) {
            if (points_[i].type == WaypointType::FINAL_POSITION ||
                points_[i].type == WaypointType::FINAL_POSE ||
                points_[i].finalHeadingRequired) return false;
        }

        for (size_t i = 0u; i + 1u < pointCount_; ++i) {
            const LocalPoint& a = points_[i].position;
            const LocalPoint& b = points_[i + 1u].position;
            const float dx = b.x - a.x;
            const float dy = b.y - a.y;
            const float length = std::sqrt(dx * dx + dy * dy);
            if (!std::isfinite(length) || length < 0.01f) return false;

            RouteSegment& segment = segments_[i];
            segment.plannedStart = a;
            segment.plannedEnd = b;
            segment.plannedHeadingDeg = headingForVector(dx, dy);
            segment.type = incomingType_[i];
            segment.corridorHalfWidthM = incomingCorridor_[i];
            segment.speedLimitMps = incomingSpeed_[i];
            segment.segmentLengthM = length;
            segment.transitionPlanePoint = b;
            segment.transitionNormal = LocalPoint(dx / length, dy / length);
            ++segmentCount_;
        }
        valid_ = true;
        return true;
    }

    const RoutePoint& point(size_t index) const { return points_[index]; }
    const RouteSegment& segment(size_t index) const { return segments_[index]; }

private:
    uint32_t routeId_;
    RoutePoint points_[MAX_POINTS];
    RouteSegment segments_[MAX_SEGMENTS];
    SegmentType incomingType_[MAX_SEGMENTS];
    float incomingCorridor_[MAX_SEGMENTS];
    float incomingSpeed_[MAX_SEGMENTS];
    size_t pointCount_;
    size_t segmentCount_;
    bool finalized_;
    bool valid_;
};

}  // namespace routeexec
