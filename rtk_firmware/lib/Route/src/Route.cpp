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

void Route::begin() {
    _core.clear();
    _state = ROUTE_NONE;
    _expectedCount = 0;
    _uploadedCount = 0;
    _originLat = _originLon = 0;
    _running = _paused = false;
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
    _core.clearObstacles();
    _forbidExpectedPolys = polygonCount;
    _forbidUploading = true;
    _forbidReady = false;
    for (int i = 0; i < NavCore::MAX_OBSTACLES; ++i) {
        _forbidSize[i] = 0;
        _forbidExpectedSize[i] = 0;
    }
    for (int i = 0; i < polygonCount; ++i) {
        if (pointCounts[i] < 3 || pointCounts[i] > NavCore::MAX_OBSTACLE_POINTS) {
            _state = ROUTE_INVALID;
            _forbidUploading = false;
            return false;
        }
        _forbidExpectedSize[i] = pointCounts[i];
    }
    return true;
}

bool Route::addForbiddenPoint(int polygonIndex, int pointIndex, float xMeters, float yMeters) {
    if (_state != ROUTE_UPLOADING || !_forbidUploading) return false;
    if (polygonIndex < 0 || polygonIndex >= _forbidExpectedPolys) return false;
    if (pointIndex < 0 || pointIndex >= _forbidExpectedSize[polygonIndex]) return false;
    if (!isfinite(xMeters) || !isfinite(yMeters)) return false;
    if (pointIndex < _forbidSize[polygonIndex]) return true;
    if (pointIndex != _forbidSize[polygonIndex]) return false;
    _forbid[polygonIndex][pointIndex] = {xMeters, yMeters};
    _forbidSize[polygonIndex]++;
    return true;
}

bool Route::endForbidden() {
    if (_state != ROUTE_UPLOADING || !_forbidUploading) return false;
    _forbidUploading = false;
    _core.clearObstacles();
    for (int i = 0; i < _forbidExpectedPolys; ++i) {
        int n = _forbidSize[i];
        routeNormalizePolygon(_forbid[i], n);
        _forbidSize[i] = n;
        if (n != _forbidExpectedSize[i] ||
            n < 3 ||
            !routePolygonSimple(_forbid[i], n) ||
            navPolygonAreaAbs(_forbid[i], n) < 0.05f ||
            !_core.addObstacle(_forbid[i], n)) {
            _state = ROUTE_INVALID;
            _forbidReady = false;
            _core.clearObstacles();
            return false;
        }
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
