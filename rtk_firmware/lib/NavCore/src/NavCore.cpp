// NavCore.cpp - реализация. Источник алгоритмов: github.com/Ardumower/Sunray (GPLv3).
// Реализация переписана своими словами, MIT.

#include "NavCore.h"

static inline float wrapPi(float a) {
    while (a >  M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
}

// ----------------- геометрия -----------------
float navDist(const NavPoint& a, const NavPoint& b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return sqrtf(dx*dx + dy*dy);
}
float navDist2(const NavPoint& a, const NavPoint& b) {
    float dx = a.x - b.x, dy = a.y - b.y;
    return dx*dx + dy*dy;
}
NavPoint navSub(const NavPoint& a, const NavPoint& b) { return {a.x - b.x, a.y - b.y}; }
NavPoint navAdd(const NavPoint& a, const NavPoint& b) { return {a.x + b.x, a.y + b.y}; }
NavPoint navScale(const NavPoint& a, float s)         { return {a.x * s, a.y * s}; }
float    navCross(const NavPoint& a, const NavPoint& b){ return a.x * b.y - a.y * b.x; }
float    navDot  (const NavPoint& a, const NavPoint& b){ return a.x * b.x + a.y * b.y; }
float    navLength(const NavPoint& a) {
    return sqrtf(a.x*a.x + a.y*a.y);
}

float navPolygonArea(const NavPoint* p, int n) {
    if (n < 3) return 0;
    float s = 0;
    for (int i = 0; i < n; ++i) {
        const NavPoint& a = p[i];
        const NavPoint& b = p[(i+1) % n];
        s += (a.x * b.y - b.x * a.y);
    }
    return 0.5f * s;
}
float navPolygonAreaAbs(const NavPoint* p, int n) { return fabsf(navPolygonArea(p, n)); }

bool navPointInPolygon(const NavPoint& pt, const NavPoint* poly, int n) {
    if (n < 3) return false;
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        const NavPoint& pi = poly[i];
        const NavPoint& pj = poly[j];
        if (((pi.y > pt.y) != (pj.y > pt.y)) &&
            (pt.x < (pj.x - pi.x) * (pt.y - pi.y) / (pj.y - pi.y + 1e-9f) + pi.x)) {
            inside = !inside;
        }
    }
    return inside;
}

bool navSegmentsIntersect(const NavPoint& a, const NavPoint& b,
                          const NavPoint& c, const NavPoint& d) {
    auto onSeg = [](const NavPoint& p, const NavPoint& q, const NavPoint& r) {
        return q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
               q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y);
    };
    NavPoint r = navSub(b, a);
    NavPoint s = navSub(d, c);
    float denom = navCross(r, s);
    if (fabsf(denom) < 1e-9f) return false;       // parallel
    NavPoint ac = navSub(c, a);
    float t = navCross(ac, s) / denom;
    float u = navCross(ac, r) / denom;
    return (t >= 0 && t <= 1 && u >= 0 && u <= 1);
}

bool navLineHitsPolygon(const NavPoint& a, const NavPoint& b,
                        const NavPoint* poly, int n) {
    if (n < 3) return false;
    for (int i = 0; i < n; ++i) {
        if (navSegmentsIntersect(a, b, poly[i], poly[(i+1) % n])) return true;
    }
    return false;
}

void navPolygonOffset(const NavPoint* src, int n, float dist, NavPoint* dst, int* dstN) {
    if (n < 3 || dst == nullptr || dstN == nullptr) { *dstN = 0; return; }
    // предполагаем CCW. dist>0 => outset, dist<0 => inset
    int j = 0;
    for (int i = 0; i < n; ++i) {
        const NavPoint& a = src[(i - 1 + n) % n];
        const NavPoint& b = src[i];
        const NavPoint& c = src[(i + 1) % n];
        NavPoint e1 = navSub(b, a);            // edge in
        NavPoint e2 = navSub(c, b);            // edge out
        // rotate edges by 90 deg to get normals (outward for CCW: e.x,-e.y)
        float n1x =  e1.y,  n1y = -e1.x;
        float n2x =  e2.y,  n2y = -e2.x;
        float l1 = sqrtf(n1x*n1x + n1y*n1y);
        float l2 = sqrtf(n2x*n2x + n2y*n2y);
        if (l1 < 1e-6f || l2 < 1e-6f) { dst[j++] = b; continue; }
        n1x /= l1; n1y /= l1;
        n2x /= l2; n2y /= l2;
        // average normal (miter)
        float mx = n1x + n2x, my = n1y + n2y;
        float ml = sqrtf(mx*mx + my*my);
        if (ml < 1e-6f) { dst[j++] = b; continue; }
        mx /= ml; my /= ml;
        // miter length scales with 1/cos(half angle); cap with small factor
        float cross = n1x * n2y - n1y * n2x;
        float miter = 1.0f / (0.5f + 0.5f * (1.0f - fabsf(cross)));
        if (miter > 4.0f) miter = 4.0f;
        float ox = b.x + mx * dist * miter;
        float oy = b.y + my * dist * miter;
        if (j < NavCore::MAX_OBSTACLE_POINTS) {
            dst[j++] = {ox, oy};
        }
    }
    *dstN = j;
}

// ----------------- NavCore -----------------
NavCore::NavCore() {}

void NavCore::clear() {
    _wpCount = 0;
    clearObstacles();
}

bool NavCore::addWaypoint(const NavPoint& p, WayType t) {
    if (_wpCount >= MAX_WAYPOINTS) return false;
    _wps[_wpCount++] = {p, t};
    return true;
}

void NavCore::clearObstacles() {
    for (int i = 0; i < _obsCount; ++i) {
        _obsSize[i] = 0;
    }
    _obsCount = 0;
}

bool NavCore::addObstacle(const NavPoint* p, int n) {
    if (_obsCount >= MAX_OBSTACLES) return false;
    if (n < 3 || n > MAX_OBSTACLE_POINTS) return false;
    for (int i = 0; i < n; ++i) _obst[_obsCount][i] = p[i];
    _obsSize[_obsCount] = n;
    _obsCount++;
    return true;
}

bool NavCore::pathIsClear(const NavPoint& a, const NavPoint& b) const {
    // inflate obstacles by margin, then line-vs-polygon
    for (int i = 0; i < _obsCount; ++i) {
        NavPoint inflated[MAX_OBSTACLE_POINTS];
        int inflatedN = 0;
        navPolygonOffset(_obst[i], _obsSize[i], _obstacleMargin, inflated, &inflatedN);
        if (navLineHitsPolygon(a, b, inflated, inflatedN)) return false;
    }
    return true;
}

bool NavCore::findObstacleSafePoint(const NavPoint& from, const NavPoint& find,
                                    NavPoint& safeOut) const {
    // Самая простая эвристика: пробуем прямую; если не чисто — последовательно
    // ищем точку-обход смещением перпендикуляра от прямой в 8 направлениях
    // с уменьшающимся шагом.
    if (pathIsClear(from, find)) { safeOut = find; return true; }
    const float offsets[] = {0.30f, 0.60f, 1.00f, 1.50f, 2.00f};
    NavPoint dir = navSub(find, from);
    float L = navLength(dir);
    if (L < 1e-3f) return false;
    float ux = -dir.y / L, uy = dir.x / L;   // перпендикуляр
    for (float off : offsets) {
        for (int s = -1; s <= 1; s += 2) {
            NavPoint probe = {from.x + ux * off * s, from.y + uy * off * s};
            if (pathIsClear(from, probe) && pathIsClear(probe, find)) {
                safeOut = probe;
                return true;
            }
        }
    }
    return false;
}
