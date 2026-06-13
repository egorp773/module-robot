// Route.cpp - MIT.

#include "Route.h"

void Route::begin() {
    _core.clear();
    _state = ROUTE_NONE;
    _expectedCount = 0;
    _originLat = _originLon = 0;
    _running = _paused = false;
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
    _originLat = originLat;
    _originLon = originLon;
    _state = ROUTE_UPLOADING;
    return true;
}

bool Route::addWaypoint(int index, float xMeters, float yMeters) {
    if (_state != ROUTE_UPLOADING) return false;
    if (index < 0 || index >= _expectedCount) return false;
    if (!isfinite(xMeters) || !isfinite(yMeters)) return false;
    if (!_core.addWaypoint({xMeters, yMeters}, WAY_MOW)) return false;
    return true;
}

void Route::endUpload() {
    if (_state != ROUTE_UPLOADING) return;
    if (_core.waypointCount() == _expectedCount) {
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
void Route::stop() {
    _running = false;
    _paused = false;
    _state = ROUTE_NONE;
    _core.clear();
}
