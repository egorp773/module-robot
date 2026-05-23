import csv
import json
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / ".pio" / "build_root"
NAME = sys.argv[1] if len(sys.argv) > 1 else "hitl_snake_10x10"
TRACE = OUT / f"{NAME}_trace.csv"
ROUTE_JSON = OUT / "hitl_snake_10x10_route.json"
METRICS_JSON = OUT / f"{NAME}_metrics.json"
SCREENSHOT = OUT / f"{NAME}_forbidden_coverage.png"

ZONE = (0.0, 0.0, 10.0, 10.0)
FORBIDDEN = (4.0, 4.0, 6.0, 6.0)
ROBOT_RADIUS = 0.25
GRID = 0.05


def dist_point_segment(px, py, ax, ay, bx, by):
    dx = bx - ax
    dy = by - ay
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / length_sq
    t = max(0.0, min(1.0, t))
    qx = ax + dx * t
    qy = ay + dy * t
    return math.hypot(px - qx, py - qy)


def dist_to_rect(px, py, rect):
    x1, y1, x2, y2 = rect
    dx = max(x1 - px, 0.0, px - x2)
    dy = max(y1 - py, 0.0, py - y2)
    return math.hypot(dx, dy)


def inside_rect(px, py, rect):
    x1, y1, x2, y2 = rect
    return x1 <= px <= x2 and y1 <= py <= y2


def main():
    rows = []
    with TRACE.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            rows.append(row)
    true_path = [(float(r["true_x"]), float(r["true_y"])) for r in rows]
    route_data = json.loads(ROUTE_JSON.read_text(encoding="utf-8"))
    route = [(float(p["x"]), float(p["y"])) for p in route_data["points"]]

    center_entries = [p for p in true_path if inside_rect(p[0], p[1], FORBIDDEN)]
    min_forbidden_dist = min(dist_to_rect(x, y, FORBIDDEN) for x, y in true_path)
    footprint_touched = min_forbidden_dist <= ROBOT_RADIUS

    nx = int((ZONE[2] - ZONE[0]) / GRID)
    ny = int((ZONE[3] - ZONE[1]) / GRID)
    covered_cells = set()
    radius_cells = int(math.ceil(ROBOT_RADIUS / GRID))
    radius_sq = ROBOT_RADIUS * ROBOT_RADIUS
    for px, py in true_path:
        cx = int((px - ZONE[0]) / GRID)
        cy = int((py - ZONE[1]) / GRID)
        for iy in range(cy - radius_cells, cy + radius_cells + 1):
            if iy < 0 or iy >= ny:
                continue
            gy = ZONE[1] + (iy + 0.5) * GRID
            for ix in range(cx - radius_cells, cx + radius_cells + 1):
                if ix < 0 or ix >= nx:
                    continue
                gx = ZONE[0] + (ix + 0.5) * GRID
                if inside_rect(gx, gy, FORBIDDEN):
                    continue
                if (gx - px) * (gx - px) + (gy - py) * (gy - py) <= radius_sq:
                    covered_cells.add((ix, iy))
    safe_total = 0
    for iy in range(ny):
        gy = ZONE[1] + (iy + 0.5) * GRID
        for ix in range(nx):
            gx = ZONE[0] + (ix + 0.5) * GRID
            if not inside_rect(gx, gy, FORBIDDEN):
                safe_total += 1
    covered = len(covered_cells)
    coverage = 100.0 * covered / safe_total if safe_total else 0.0

    stripe_cte = [
        abs(float(r["stripe_cte_m"]))
        for r in rows
        if r.get("stripe_cte_m") not in ("", None)
    ]
    route_cte = [
        abs(float(r["route_cte_m"]))
        for r in rows
        if r.get("route_cte_m") not in ("", None)
    ]
    metrics = {
        "routePointCount": len(route),
        "routeDistanceM": route_data["totalDistanceM"],
        "durationS": float(rows[-1]["time_s"]) if rows else 0.0,
        "enteredForbiddenCenter": bool(center_entries),
        "forbiddenCenterSamples": len(center_entries),
        "minCenterDistanceToForbiddenM": round(min_forbidden_dist, 3),
        "robotFootprintTouchedForbidden": bool(footprint_touched),
        "coveragePercent": round(coverage, 2),
        "meanStripeCrossTrackM": round(sum(stripe_cte) / len(stripe_cte), 4),
        "meanRouteCrossTrackM": round(sum(route_cte) / len(route_cte), 4),
        "stripeCrossTrackSamples": len(stripe_cte),
        "routeCrossTrackSamples": len(route_cte),
    }
    METRICS_JSON.write_text(json.dumps(metrics, indent=2), encoding="utf-8")

    fig, ax = plt.subplots(figsize=(9, 9))
    ax.add_patch(plt.Rectangle((0, 0), 10, 10, color="#38f6a7", alpha=0.18, label="green zone"))
    ax.add_patch(plt.Rectangle((4, 4), 2, 2, color="#ff4d6d", alpha=0.35, label="forbidden"))
    ax.plot([p[0] for p in route], [p[1] for p in route], "--", color="#222222", linewidth=1.0, label="planned route")
    ax.plot([p[0] for p in true_path], [p[1] for p in true_path], color="#0b5cad", linewidth=1.7, label="HITL true path")
    ax.scatter([true_path[0][0]], [true_path[0][1]], color="#111111", s=45, label="start")
    ax.scatter([true_path[-1][0]], [true_path[-1][1]], color="#f5a623", s=45, label="finish")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-2.6, 10.6)
    ax.set_ylim(-0.5, 10.5)
    ax.grid(True, alpha=0.25)
    ax.set_xlabel("x east, m")
    ax.set_ylabel("y north, m")
    ax.set_title(
        f"HITL snake 10x10, coverage={coverage:.1f}%, "
        f"forbidden touch={'YES' if footprint_touched else 'NO'}"
    )
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(SCREENSHOT, dpi=160)
    print(json.dumps(metrics, indent=2))
    print(f"Screenshot: {SCREENSHOT}")


if __name__ == "__main__":
    main()
