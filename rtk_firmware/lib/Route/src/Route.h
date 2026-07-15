// Route.h - хранение waypoints + origin. MIT.

#pragma once
#include <Arduino.h>
#include "NavCore.h"
#include "FootprintGeometry.h"

enum RouteState : uint8_t {
    ROUTE_NONE = 0,       // ничего не загружено
    ROUTE_UPLOADING,      // идёт загрузка
    ROUTE_READY,          // все WP получены, можно NAV_START
    ROUTE_INVALID,        // ошибка загрузки
};

// Optional additive metadata for the existing ROUTE_WP x/y upload. Legacy
// clients may omit it; new clients can describe exact transition and terminal
// semantics without changing the coordinate protocol.
struct RouteWaypointMetadata {
    bool present = false;
    routeexec::WaypointType type = routeexec::WaypointType::PASS_THROUGH;
    bool finalHeadingRequired = false;
    float finalHeadingDeg = 0.0f;
    float positionToleranceM = 0.10f;
    float headingToleranceDeg = 7.0f;
    routeexec::SegmentType incomingSegmentType = routeexec::SegmentType::LINE;
    float corridorHalfWidthM = 0.15f;
    float speedLimitMps = 0.11f;
};

class Route {
public:
    static const int MAX_BOUNDARY_POINTS = NavCore::MAX_OBSTACLE_POINTS;

    void begin();

    // ROUTE_BEGIN,count,lat,lon
    bool beginUpload(int count, double originLat, double originLon);
    // ROUTE_WP,index,x_m,y_m
    bool addWaypoint(int index, float xMeters, float yMeters);
    bool setWaypointMetadata(int index,
                             const RouteWaypointMetadata& metadata);
    const RouteWaypointMetadata& waypointMetadata(int index) const {
        return _waypointMetadata[index];
    }
    bool hasWaypointMetadata(int index) const {
        return index >= 0 && index < _expectedCount &&
               _waypointMetadata[index].present;
    }

    bool beginBoundary(int count);
    bool addBoundaryPoint(int index, float xMeters, float yMeters);
    bool endBoundary();

    bool beginForbidden(int polygonCount, const int* pointCounts);
    bool addForbiddenPoint(int polygonIndex, int pointIndex, float xMeters, float yMeters);
    bool endForbidden();

    void endUpload();

    // NAV state
    RouteState state() const { return _state; }
    bool isReady() const { return _state == ROUTE_READY; }
    int  count() const { return _core.waypointCount(); }
    const Waypoint& waypoint(int i) const { return _core.waypoint(i); }
    NavCore& core() { return _core; }
    const NavCore& core() const { return _core; }

    // start / pause / resume / stop
    void start();
    void pause();
    void resume();
    void finish();
    void stop();

    bool isRunning() const { return _running; }
    bool isPaused()  const { return _paused; }

    double originLat() const { return _originLat; }
    double originLon() const { return _originLon; }
    bool hasBoundary() const { return _boundaryReady; }
    bool positionAllowed(const NavPoint& p, float toleranceM) const;
    bool segmentAllowed(const NavPoint& a, const NavPoint& b,
                        float toleranceM, float sampleStepM) const;

    // Oriented body/tool checks use the corrected robot control point.  The
    // legacy center-only methods above remain unchanged for protocol and
    // route-upload compatibility.
    static routeexec::FootprintConfig configuredFootprint();
    routeexec::FootprintCheckResult checkFootprintPose(
        const routeexec::Pose2D& pose,
        const routeexec::FootprintConfig& footprint) const;
    routeexec::FootprintCheckResult checkReverseSweptFootprint(
        const routeexec::Pose2D& startPose,
        float reverseDistanceM,
        float poseSampleStepM,
        const routeexec::FootprintConfig& footprint) const;
    routeexec::FootprintCheckResult checkForwardSweptFootprint(
        const routeexec::Pose2D& startPose,
        const routeexec::LocalPoint& target,
        float poseSampleStepM,
        const routeexec::FootprintConfig& footprint) const;
    routeexec::FootprintCheckResult checkTurnSweptFootprint(
        const routeexec::Pose2D& startPose,
        float targetHeadingDeg,
        float headingSampleStepDeg,
        const routeexec::FootprintConfig& footprint) const;

private:
    NavCore _core;
    RouteState _state = ROUTE_NONE;
    int _expectedCount = 0;
    int _uploadedCount = 0;
    double _originLat = 0, _originLon = 0;
    bool _running = false;
    bool _paused = false;
    RouteWaypointMetadata _waypointMetadata[NavCore::MAX_WAYPOINTS];

    NavPoint _boundary[MAX_BOUNDARY_POINTS];
    int _boundaryExpected = 0;
    int _boundaryUploaded = 0;
    int _boundaryCount = 0;
    bool _boundaryUploading = false;
    bool _boundaryReady = false;

    // Only the polygon currently being uploaded needs a second copy. After
    // validation it is committed to NavCore's compact obstacle pool.
    NavPoint _forbidScratch[NavCore::MAX_OBSTACLE_POINTS];
    int _forbidSize[NavCore::MAX_OBSTACLES] = {0};
    int _forbidExpectedSize[NavCore::MAX_OBSTACLES] = {0};
    int _forbidExpectedPolys = 0;
    int _forbidActivePolygon = -1;
    bool _forbidUploading = false;
    bool _forbidReady = false;

    static bool footprintPointAllowed(const routeexec::LocalPoint& point,
                                      void* context);
    bool commitForbiddenScratch(int polygonIndex);
    bool footprintPolygonAllowed(const routeexec::Pose2D& pose,
                                 const routeexec::RobotFootprint& footprint) const;
};
