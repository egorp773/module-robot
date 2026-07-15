// Route.cpp - MIT.

#include "Route.h"
#include "RtkConfig.h"

static float routeDistanceToSegment(const NavPoint& p, const NavPoint& a, const NavPoint& b) {
    float abx = b.x - a.x;
    float aby = b.y - a.y;
    float len2 = abx * abx + aby * aby;
    if (len2 <= 1e-9f) return navDist(p, a);
    float t = ((p.x - a.x) * abx + (p.y - a.y) * aby) / len2;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    NavPoint q{a.x + abx * t, a.y + aby * t};
    return navDist(p, q);
}

static float routeDistanceToPolygon(const NavPoint& p, const NavPoint* poly, int n) {
    if (n < 3) return 1e9f;
    float best = 1e9f;
    for (int i = 0; i < n; ++i) {
        float d = routeDistanceToSegment(p, poly[i], poly[(i + 1) % n]);
        if (d < best) best = d;
    }
    return best;
}

static void routeReversePolygon(NavPoint* poly, int n) {
    for (int i = 0; i < n / 2; ++i) {
        NavPoint tmp = poly[i];
        poly[i] = poly[n - 1 - i];
        poly[n - 1 - i] = tmp;
    }
}

static void routeNormalizePolygon(NavPoint* poly, int& n) {
    const float eps2 = 0.001f * 0.001f;
    int w = 0;
    for (int i = 0; i < n; ++i) {
        if (w == 0 || navDist2(poly[i], poly[w - 1]) > eps2) {
            poly[w++] = poly[i];
        }
    }
    n = w;
    if (n > 1 && navDist2(poly[0], poly[n - 1]) <= eps2) {
        n--;
    }
    if (n >= 3 && navPolygonArea(poly, n) < 0.0f) {
        routeReversePolygon(poly, n);
    }
}

static bool routePolygonSimple(const NavPoint* poly, int n) {
    if (n < 3) return false;
    for (int i = 0; i < n; ++i) {
        NavPoint a1 = poly[i];
        NavPoint a2 = poly[(i + 1) % n];
        for (int j = i + 1; j < n; ++j) {
            if (j == i) continue;
            if (j == (i + 1) % n) continue;
            if (i == (j + 1) % n) continue;
            NavPoint b1 = poly[j];
            NavPoint b2 = poly[(j + 1) % n];
            if (navSegmentsIntersect(a1, a2, b1, b2)) return false;
        }
    }
    return true;
}

static bool routePointInsideOrOnPolygon(const NavPoint& p,
                                        const NavPoint* poly,
                                        int n) {
    return n >= 3 &&
           (navPointInPolygon(p, poly, n) ||
            routeDistanceToPolygon(p, poly, n) <= 1e-5f);
}

static bool routePolygonsTouchOrIntersect(const NavPoint* a, int aCount,
                                          const NavPoint* b, int bCount) {
    if (aCount < 3 || bCount < 3) return false;
    for (int i = 0; i < aCount; ++i) {
        const NavPoint& a0 = a[i];
        const NavPoint& a1 = a[(i + 1) % aCount];
        for (int j = 0; j < bCount; ++j) {
            if (navSegmentsIntersect(a0, a1,
                                     b[j], b[(j + 1) % bCount])) {
                return true;
            }
        }
    }
    return false;
}

static routeexec::Pose2D routeReversePose(const routeexec::Pose2D& start,
                                          float reverseDistanceM) {
    constexpr float kDegToRad = 0.01745329251994329577f;
    const float headingRad = start.headingDeg * kDegToRad;
    routeexec::Pose2D pose = start;
    pose.position.x -= sinf(headingRad) * reverseDistanceM;
    pose.position.y -= cosf(headingRad) * reverseDistanceM;
    return pose;
}

void Route::begin() {
    _core.clear();
    _state = ROUTE_NONE;
    _expectedCount = 0;
    _uploadedCount = 0;
    _originLat = _originLon = 0;
    _running = _paused = false;
    for (int i = 0; i < NavCore::MAX_WAYPOINTS; ++i)
        _waypointMetadata[i] = RouteWaypointMetadata{};
    _boundaryExpected = _boundaryUploaded = _boundaryCount = 0;
    _boundaryUploading = false;
    _boundaryReady = false;
    _forbidExpectedPolys = 0;
    _forbidActivePolygon = -1;
    _forbidUploading = false;
    _forbidReady = false;
    for (int i = 0; i < NavCore::MAX_OBSTACLES; ++i) {
        _forbidSize[i] = 0;
        _forbidExpectedSize[i] = 0;
    }
}

bool Route::beginUpload(int count, double originLat, double originLon) {
    if (count < 1 || count > NavCore::MAX_WAYPOINTS) {
        _state = ROUTE_INVALID;
        return false;
    }
    if (originLat == 0.0 && originLon == 0.0) {
        _state = ROUTE_INVALID;
        return false;
    }
    _core.clear();
    _expectedCount = count;
    _uploadedCount = 0;
    for (int i = 0; i < NavCore::MAX_WAYPOINTS; ++i)
        _waypointMetadata[i] = RouteWaypointMetadata{};
    _originLat = originLat;
    _originLon = originLon;
    _boundaryExpected = _boundaryUploaded = _boundaryCount = 0;
    _boundaryUploading = false;
    _boundaryReady = false;
    _forbidExpectedPolys = 0;
    _forbidUploading = false;
    _forbidReady = false;
    for (int i = 0; i < NavCore::MAX_OBSTACLES; ++i) {
        _forbidSize[i] = 0;
        _forbidExpectedSize[i] = 0;
    }
    _state = ROUTE_UPLOADING;
    return true;
}

bool Route::addWaypoint(int index, float xMeters, float yMeters) {
    if (_state != ROUTE_UPLOADING) return false;
    if (index < 0 || index >= _expectedCount) return false;
    if (!isfinite(xMeters) || !isfinite(yMeters)) return false;
    if (index < _uploadedCount) return true;
    if (index != _uploadedCount) return false;
    if (!_core.addWaypoint({xMeters, yMeters}, WAY_MOW)) return false;
    _uploadedCount++;
    return true;
}

bool Route::setWaypointMetadata(
    int index, const RouteWaypointMetadata& metadata) {
    if (_state != ROUTE_UPLOADING || index < 0 || index >= _expectedCount)
        return false;
    if (!isfinite(metadata.finalHeadingDeg) ||
        !isfinite(metadata.positionToleranceM) ||
        !isfinite(metadata.headingToleranceDeg) ||
        !isfinite(metadata.corridorHalfWidthM) ||
        !isfinite(metadata.speedLimitMps) ||
        metadata.positionToleranceM <= 0.0f ||
        metadata.headingToleranceDeg <= 0.0f ||
        metadata.corridorHalfWidthM <= 0.0f ||
        metadata.speedLimitMps <= 0.0f) return false;
    if (metadata.type == routeexec::WaypointType::FINAL_POSE &&
        !metadata.finalHeadingRequired) return false;
    if (metadata.type == routeexec::WaypointType::FINAL_POSITION &&
        metadata.finalHeadingRequired) return false;
    _waypointMetadata[index] = metadata;
    _waypointMetadata[index].present = true;
    return true;
}

bool Route::beginBoundary(int count) {
    if (_state != ROUTE_UPLOADING) return false;
    if (count < 3 || count > MAX_BOUNDARY_POINTS) return false;
    _boundaryExpected = count;
    _boundaryUploaded = 0;
    _boundaryCount = 0;
    _boundaryReady = false;
    _boundaryUploading = true;
    return true;
}

bool Route::addBoundaryPoint(int index, float xMeters, float yMeters) {
    if (_state != ROUTE_UPLOADING || !_boundaryUploading) return false;
    if (index < 0 || index >= _boundaryExpected) return false;
    if (!isfinite(xMeters) || !isfinite(yMeters)) return false;
    if (index < _boundaryUploaded) return true;
    if (index != _boundaryUploaded) return false;
    _boundary[index] = {xMeters, yMeters};
    _boundaryUploaded++;
    return true;
}

bool Route::endBoundary() {
    if (_state != ROUTE_UPLOADING || !_boundaryUploading) return false;
    _boundaryUploading = false;
    if (_boundaryUploaded != _boundaryExpected || _boundaryExpected < 3) return false;
    int n = _boundaryUploaded;
    routeNormalizePolygon(_boundary, n);
    if (n < 3 || !routePolygonSimple(_boundary, n)) return false;
    if (navPolygonAreaAbs(_boundary, n) < 0.20f) return false;
    _boundaryCount = n;
    _boundaryReady = true;
    return true;
}

bool Route::beginForbidden(int polygonCount, const int* pointCounts) {
    if (_state != ROUTE_UPLOADING) return false;
    if (polygonCount < 0 || polygonCount > NavCore::MAX_OBSTACLES) return false;
    if (polygonCount > 0 && pointCounts == nullptr) return false;
    int totalPointCount = 0;
    for (int i = 0; i < polygonCount; ++i) {
        if (pointCounts[i] < 3 ||
            pointCounts[i] > NavCore::MAX_OBSTACLE_POINTS) return false;
        totalPointCount += pointCounts[i];
    }
    if (totalPointCount > NavCore::MAX_OBSTACLE_TOTAL_POINTS) return false;

    _core.clearObstacles();
    _forbidExpectedPolys = polygonCount;
    _forbidActivePolygon = -1;
    _forbidUploading = true;
    _forbidReady = false;
    for (int i = 0; i < NavCore::MAX_OBSTACLES; ++i) {
        _forbidSize[i] = 0;
        _forbidExpectedSize[i] = 0;
    }
    for (int i = 0; i < polygonCount; ++i) {
        _forbidExpectedSize[i] = pointCounts[i];
    }
    return true;
}

bool Route::commitForbiddenScratch(int polygonIndex) {
    if (polygonIndex < 0 || polygonIndex >= _forbidExpectedPolys) return false;
    int n = _forbidSize[polygonIndex];
    if (n != _forbidExpectedSize[polygonIndex]) return false;
    routeNormalizePolygon(_forbidScratch, n);
    _forbidSize[polygonIndex] = n;
    return n == _forbidExpectedSize[polygonIndex] &&
           n >= 3 && routePolygonSimple(_forbidScratch, n) &&
           navPolygonAreaAbs(_forbidScratch, n) >= 0.05f &&
           _core.addObstacle(_forbidScratch, n);
}

bool Route::addForbiddenPoint(int polygonIndex, int pointIndex, float xMeters, float yMeters) {
    if (_state != ROUTE_UPLOADING || !_forbidUploading) return false;
    if (polygonIndex < 0 || polygonIndex >= _forbidExpectedPolys) return false;
    if (pointIndex < 0 || pointIndex >= _forbidExpectedSize[polygonIndex]) return false;
    if (!isfinite(xMeters) || !isfinite(yMeters)) return false;
    if (_forbidActivePolygon < 0) {
        if (polygonIndex != 0 || pointIndex != 0) return false;
        _forbidActivePolygon = 0;
    } else if (polygonIndex < _forbidActivePolygon) {
        // Idempotent retry for an already committed polygon.
        return pointIndex < _forbidSize[polygonIndex];
    } else if (polygonIndex > _forbidActivePolygon) {
        if (polygonIndex != _forbidActivePolygon + 1 || pointIndex != 0 ||
            !commitForbiddenScratch(_forbidActivePolygon)) return false;
        _forbidActivePolygon = polygonIndex;
    }
    if (pointIndex < _forbidSize[polygonIndex]) return true;
    if (pointIndex != _forbidSize[polygonIndex]) return false;
    _forbidScratch[pointIndex] = {xMeters, yMeters};
    _forbidSize[polygonIndex]++;
    return true;
}

bool Route::endForbidden() {
    if (_state != ROUTE_UPLOADING || !_forbidUploading) return false;
    _forbidUploading = false;
    const bool complete = _forbidExpectedPolys == 0
        ? _forbidActivePolygon == -1
        : (_forbidActivePolygon == _forbidExpectedPolys - 1 &&
           commitForbiddenScratch(_forbidActivePolygon));
    if (!complete || _core.obstacleCount() != _forbidExpectedPolys) {
        _state = ROUTE_INVALID;
        _forbidReady = false;
        _core.clearObstacles();
        return false;
    }
    _forbidReady = true;
    return true;
}

void Route::endUpload() {
    if (_state != ROUTE_UPLOADING) return;
    bool ok = _core.waypointCount() == _expectedCount &&
              _uploadedCount == _expectedCount &&
              _boundaryReady &&
              !_boundaryUploading &&
              !_forbidUploading &&
              _forbidReady;
    if (ok) {
        for (int i = 0; i < _core.waypointCount(); ++i) {
            if (!positionAllowed(_core.waypoint(i).p, ROVER_BOUNDARY_TOLERANCE_M)) {
                ok = false;
                break;
            }
            if (i > 0 &&
                !segmentAllowed(_core.waypoint(i - 1).p, _core.waypoint(i).p,
                                ROVER_BOUNDARY_TOLERANCE_M, ROVER_BOUNDARY_SAMPLE_M)) {
                ok = false;
                break;
            }
        }
    }
    if (ok) {
        _state = ROUTE_READY;
    } else {
        _state = ROUTE_INVALID;
    }
}

void Route::start() {
    if (_state == ROUTE_READY) {
        _running = true;
        _paused = false;
    }
}
void Route::pause() {
    if (_running) _paused = true;
}
void Route::resume() {
    if (_running) _paused = false;
}
void Route::finish() {
    _running = false;
    _paused = false;
}
void Route::stop() {
    _running = false;
    _paused = false;
    _state = ROUTE_NONE;
    _uploadedCount = 0;
    _core.clear();
    for (int i = 0; i < NavCore::MAX_WAYPOINTS; ++i)
        _waypointMetadata[i] = RouteWaypointMetadata{};
}

bool Route::positionAllowed(const NavPoint& p, float toleranceM) const {
    if (!_boundaryReady || _boundaryCount < 3) return false;
    bool insideBoundary = navPointInPolygon(p, _boundary, _boundaryCount) ||
                          routeDistanceToPolygon(p, _boundary, _boundaryCount) <= toleranceM;
    if (!insideBoundary) return false;
    for (int i = 0; i < _core.obstacleCount(); ++i) {
        const NavPoint* obs = _core.obstacle(i);
        int n = _core.obstacleSize(i);
        if (navPointInPolygon(p, obs, n)) return false;
        if (routeDistanceToPolygon(p, obs, n) <= toleranceM) return false;
    }
    return true;
}

bool Route::segmentAllowed(const NavPoint& a, const NavPoint& b,
                           float toleranceM, float sampleStepM) const {
    if (!_boundaryReady) return false;
    if (!positionAllowed(a, toleranceM) || !positionAllowed(b, toleranceM)) return false;
    if (!_core.pathIsClear(a, b)) return false;
    float len = navDist(a, b);
    int steps = (int)ceilf(len / max(0.02f, sampleStepM));
    if (steps < 1) steps = 1;
    for (int i = 1; i < steps; ++i) {
        float t = (float)i / (float)steps;
        NavPoint p{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
        if (!positionAllowed(p, toleranceM)) return false;
    }
    return true;
}

routeexec::FootprintConfig Route::configuredFootprint() {
    routeexec::FootprintConfig config;
    config.body = routeexec::RobotFootprint(
        ROVER_BODY_FOOTPRINT_FRONT_M,
        ROVER_BODY_FOOTPRINT_REAR_M,
        ROVER_BODY_FOOTPRINT_LEFT_M,
        ROVER_BODY_FOOTPRINT_RIGHT_M);
    config.toolConfigured = ROVER_TOOL_FOOTPRINT_CONFIGURED != 0;
    config.tool = routeexec::RobotFootprint(
        ROVER_TOOL_FOOTPRINT_FRONT_M,
        ROVER_TOOL_FOOTPRINT_REAR_M,
        ROVER_TOOL_FOOTPRINT_LEFT_M,
        ROVER_TOOL_FOOTPRINT_RIGHT_M);
    return config;
}

bool Route::footprintPointAllowed(const routeexec::LocalPoint& point,
                                  void* context) {
    if (context == nullptr) return false;
    const Route* route = static_cast<const Route*>(context);
    return route->positionAllowed({point.x, point.y}, 0.0f);
}

bool Route::footprintPolygonAllowed(
    const routeexec::Pose2D& pose,
    const routeexec::RobotFootprint& footprint) const {
    routeexec::LocalPoint samples[9];
    const size_t sampleCount = routeexec::footprintPoseSamples(
        pose, footprint, samples, 9u);
    if (sampleCount != 9u || !_boundaryReady || !_forbidReady) return false;

    // footprintPoseSamples order is center, FL, FR, RL, RR, then side
    // midpoints.  Reorder the corners into a non-self-intersecting polygon.
    const NavPoint rectangle[4] = {
        {samples[1].x, samples[1].y},
        {samples[2].x, samples[2].y},
        {samples[4].x, samples[4].y},
        {samples[3].x, samples[3].y},
    };

    // All footprint samples must be inside/on the permitted boundary.  The
    // explicit edge test catches a rectangle crossing a concave boundary even
    // when its corners happen to remain inside.
    for (size_t i = 0u; i < sampleCount; ++i) {
        const NavPoint p{samples[i].x, samples[i].y};
        if (!routePointInsideOrOnPolygon(p, _boundary, _boundaryCount)) {
            return false;
        }
    }
    if (routePolygonsTouchOrIntersect(rectangle, 4,
                                      _boundary, _boundaryCount)) {
        return false;
    }

    for (int obstacleIndex = 0;
         obstacleIndex < _core.obstacleCount();
         ++obstacleIndex) {
        const NavPoint* obstacle = _core.obstacle(obstacleIndex);
        const int obstacleCount = _core.obstacleSize(obstacleIndex);
        if (obstacleCount < 3) return false;

        for (size_t i = 0u; i < sampleCount; ++i) {
            const NavPoint p{samples[i].x, samples[i].y};
            if (routePointInsideOrOnPolygon(p, obstacle, obstacleCount)) {
                return false;
            }
        }
        if (routePolygonsTouchOrIntersect(rectangle, 4,
                                          obstacle, obstacleCount)) {
            return false;
        }
        // A small forbidden polygon may lie wholly inside the footprint with
        // no footprint vertex inside it and no intersecting edge.
        for (int i = 0; i < obstacleCount; ++i) {
            if (routePointInsideOrOnPolygon(obstacle[i], rectangle, 4)) {
                return false;
            }
        }
    }
    return true;
}

routeexec::FootprintCheckResult Route::checkFootprintPose(
    const routeexec::Pose2D& pose,
    const routeexec::FootprintConfig& footprint) const {
    const routeexec::RobotFootprint combined =
        routeexec::combinedFootprint(footprint);
    const routeexec::FootprintCheckResult sampled =
        routeexec::checkFootprintPose(
            pose, combined, &Route::footprintPointAllowed,
            const_cast<Route*>(this));
    if (sampled != routeexec::FootprintCheckResult::CLEAR) return sampled;
    return footprintPolygonAllowed(pose, combined)
        ? routeexec::FootprintCheckResult::CLEAR
        : routeexec::FootprintCheckResult::BLOCKED;
}

routeexec::FootprintCheckResult Route::checkReverseSweptFootprint(
    const routeexec::Pose2D& startPose,
    float reverseDistanceM,
    float poseSampleStepM,
    const routeexec::FootprintConfig& footprint) const {
    // A route always carries a boundary/red-zone context, so unknown tool
    // overhang is never treated as safe for an automatic reverse.
    const routeexec::FootprintCheckResult sampled =
        routeexec::checkReverseSweptFootprint(
            startPose, reverseDistanceM, poseSampleStepM,
            footprint, true, &Route::footprintPointAllowed,
            const_cast<Route*>(this));
    if (sampled != routeexec::FootprintCheckResult::CLEAR) return sampled;

    const size_t stepCountRaw = static_cast<size_t>(
        ceilf(reverseDistanceM / poseSampleStepM));
    const size_t stepCount = stepCountRaw > 0u ? stepCountRaw : 1u;
    for (size_t i = 0u; i <= stepCount; ++i) {
        const float distance = reverseDistanceM *
            (static_cast<float>(i) / static_cast<float>(stepCount));
        const routeexec::Pose2D pose = routeReversePose(startPose, distance);
        const routeexec::FootprintCheckResult exact =
            checkFootprintPose(pose, footprint);
        if (exact != routeexec::FootprintCheckResult::CLEAR) return exact;
    }
    return routeexec::FootprintCheckResult::CLEAR;
}

routeexec::FootprintCheckResult Route::checkForwardSweptFootprint(
    const routeexec::Pose2D& startPose,
    const routeexec::LocalPoint& target,
    float poseSampleStepM,
    const routeexec::FootprintConfig& footprint) const {
    if (!routeexec::finitePoint(startPose.position) ||
        !routeexec::finitePoint(target) || !isfinite(startPose.headingDeg) ||
        !isfinite(poseSampleStepM) || poseSampleStepM <= 0.0f) {
        return routeexec::FootprintCheckResult::INVALID_ARGUMENT;
    }
    const float dx = target.x - startPose.position.x;
    const float dy = target.y - startPose.position.y;
    const float distanceM = sqrtf(dx * dx + dy * dy);
    const size_t rawSteps = static_cast<size_t>(
        ceilf(distanceM / poseSampleStepM));
    const size_t steps = rawSteps > 0u ? rawSteps : 1u;
    const float pathHeading = routeexec::headingForVector(dx, dy);
    for (size_t i = 0u; i <= steps; ++i) {
        const float ratio = static_cast<float>(i) /
                            static_cast<float>(steps);
        routeexec::Pose2D pose;
        pose.position.x = startPose.position.x + dx * ratio;
        pose.position.y = startPose.position.y + dy * ratio;
        // Check both the live orientation and the commanded path orientation.
        // This conservatively covers the swept rectangle while steering into
        // the line without allocating a polygon in the fast loop.
        pose.headingDeg = startPose.headingDeg;
        routeexec::FootprintCheckResult result =
            checkFootprintPose(pose, footprint);
        if (result != routeexec::FootprintCheckResult::CLEAR) return result;
        pose.headingDeg = pathHeading;
        result = checkFootprintPose(pose, footprint);
        if (result != routeexec::FootprintCheckResult::CLEAR) return result;
    }
    return routeexec::FootprintCheckResult::CLEAR;
}

routeexec::FootprintCheckResult Route::checkTurnSweptFootprint(
    const routeexec::Pose2D& startPose,
    float targetHeadingDeg,
    float headingSampleStepDeg,
    const routeexec::FootprintConfig& footprint) const {
    if (!routeexec::finitePoint(startPose.position) ||
        !isfinite(startPose.headingDeg) || !isfinite(targetHeadingDeg) ||
        !isfinite(headingSampleStepDeg) || headingSampleStepDeg <= 0.0f) {
        return routeexec::FootprintCheckResult::INVALID_ARGUMENT;
    }
    float deltaDeg = fmodf(targetHeadingDeg - startPose.headingDeg, 360.0f);
    if (deltaDeg > 180.0f) deltaDeg -= 360.0f;
    if (deltaDeg < -180.0f) deltaDeg += 360.0f;
    size_t steps = static_cast<size_t>(
        ceilf(fabsf(deltaDeg) / headingSampleStepDeg));
    if (steps < 1u) steps = 1u;
    for (size_t i = 0u; i <= steps; ++i) {
        routeexec::Pose2D pose = startPose;
        pose.headingDeg = routeexec::normalizeHeadingDeg(
            startPose.headingDeg + deltaDeg *
                (static_cast<float>(i) / static_cast<float>(steps)));
        const routeexec::FootprintCheckResult result =
            checkFootprintPose(pose, footprint);
        if (result != routeexec::FootprintCheckResult::CLEAR) return result;
    }
    return routeexec::FootprintCheckResult::CLEAR;
}
