// NavCore.h - алгоритмы из Sunray Map.cpp, переписаны своими словами.
// MIT. Источник алгоритмов: github.com/Ardumower/Sunray (GPLv3).

#pragma once
#include <Arduino.h>
#include <math.h>

struct NavPoint {
    float x, y;       // meters
};

enum WayType : uint8_t {
    WAY_MOW = 0,
    WAY_FREE = 1,
};

struct Waypoint {
    NavPoint p;
    WayType type;
};

// ----------------- геометрия -----------------
float navDist(const NavPoint& a, const NavPoint& b);
float navDist2(const NavPoint& a, const NavPoint& b);   // squared
NavPoint navSub(const NavPoint& a, const NavPoint& b);
NavPoint navAdd(const NavPoint& a, const NavPoint& b);
NavPoint navScale(const NavPoint& a, float s);
float navCross(const NavPoint& a, const NavPoint& b);
float navDot(const NavPoint& a, const NavPoint& b);
float navLength(const NavPoint& a);

// ----------------- полигон -----------------
// signed area (>0 = counter-clockwise in math convention)
float navPolygonArea(const NavPoint* p, int n);

// signed area >=0
float navPolygonAreaAbs(const NavPoint* p, int n);

bool navPointInPolygon(const NavPoint& p, const NavPoint* poly, int n);

// true если отрезок (a,b) пересекает любой отрезок (c,d)
bool navSegmentsIntersect(const NavPoint& a, const NavPoint& b,
                         const NavPoint& c, const NavPoint& d);

// true если отрезок (a,b) пересекает замкнутый контур (любой его отрезок)
bool navLineHitsPolygon(const NavPoint& a, const NavPoint& b,
                        const NavPoint* poly, int n);

// offset (inset при dist<0, outset при dist>0) простым алгоритмом сдвига нормалей.
// dst должен иметь capacity >= n
void navPolygonOffset(const NavPoint* src, int n, float dist, NavPoint* dst, int* dstN);

// ----------------- routing -----------------
// Single-robot mowing / waypoint list. Без A*. Без SD. Без CRC.
class NavCore {
public:
    static const int MAX_WAYPOINTS = 254;
    static const int MAX_OBSTACLES = 16;
    static const int MAX_OBSTACLE_POINTS = MAX_WAYPOINTS;
    // A polygon may still contain 254 points, but all forbidden polygons use
    // one compact pool instead of reserving 16 worst-case arrays in DRAM.
    static const int MAX_OBSTACLE_TOTAL_POINTS = 512;

    NavCore();

    void clear();
    bool addWaypoint(const NavPoint& p, WayType t = WAY_MOW);
    int  waypointCount() const { return _wpCount; }
    bool isRouteReady() const { return _wpCount > 0; }
    const Waypoint& waypoint(int i) const { return _wps[i]; }

    // obstacle (исключение), полигон
    void clearObstacles();
    bool addObstacle(const NavPoint* p, int n);
    int  obstacleCount() const { return _obsCount; }
    int  obstacleSize(int i) const { return _obsSize[i]; }
    const NavPoint* obstacle(int i) const {
        return &_obstPool[_obsOffset[i]];
    }

    // проверить, что путь (a->b) не пересекает ни одно исключение
    bool pathIsClear(const NavPoint& a, const NavPoint& b) const;

    // найти точку, эквивалентную `find` но безопасную относительно препятствий
    // возвращает false если safe-точку не нашли
    bool findObstacleSafePoint(const NavPoint& from, const NavPoint& find,
                               NavPoint& safeOut) const;

    // контекст
    void setRobotPosition(const NavPoint& p) { _robot = p; }
    void setObstacleMargin(float m) { _obstacleMargin = m; }

private:
    Waypoint _wps[MAX_WAYPOINTS];
    int _wpCount = 0;

    NavPoint _obstPool[MAX_OBSTACLE_TOTAL_POINTS];
    uint16_t _obsOffset[MAX_OBSTACLES] = {0};
    int _obsSize[MAX_OBSTACLES] = {0};
    int _obsCount = 0;
    int _obsPointCount = 0;

    NavPoint _robot{0, 0};
    float _obstacleMargin = 0.10f;
};
