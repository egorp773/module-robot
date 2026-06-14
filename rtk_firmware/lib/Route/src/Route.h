// Route.h - хранение waypoints + origin. MIT.

#pragma once
#include <Arduino.h>
#include "NavCore.h"

enum RouteState : uint8_t {
    ROUTE_NONE = 0,       // ничего не загружено
    ROUTE_UPLOADING,      // идёт загрузка
    ROUTE_READY,          // все WP получены, можно NAV_START
    ROUTE_INVALID,        // ошибка загрузки
};

class Route {
public:
    static const int MAX_BOUNDARY_POINTS = NavCore::MAX_OBSTACLE_POINTS;

    void begin();

    // ROUTE_BEGIN,count,lat,lon
    bool beginUpload(int count, double originLat, double originLon);
    // ROUTE_WP,index,x_m,y_m
    bool addWaypoint(int index, float xMeters, float yMeters);

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

private:
    NavCore _core;
    RouteState _state = ROUTE_NONE;
    int _expectedCount = 0;
    int _uploadedCount = 0;
    double _originLat = 0, _originLon = 0;
    bool _running = false;
    bool _paused = false;

    NavPoint _boundary[MAX_BOUNDARY_POINTS];
    int _boundaryExpected = 0;
    int _boundaryUploaded = 0;
    int _boundaryCount = 0;
    bool _boundaryUploading = false;
    bool _boundaryReady = false;

    NavPoint _forbid[NavCore::MAX_OBSTACLES][NavCore::MAX_OBSTACLE_POINTS];
    int _forbidSize[NavCore::MAX_OBSTACLES] = {0};
    int _forbidExpectedSize[NavCore::MAX_OBSTACLES] = {0};
    int _forbidExpectedPolys = 0;
    bool _forbidUploading = false;
    bool _forbidReady = false;
};
