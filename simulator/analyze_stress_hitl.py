import csv
import json
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Polygon


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / ".pio" / "build_root"
STRESS_DIR = OUT / "stress_routes"
ROUTES_JSON = STRESS_DIR / "stress_routes.json"
ROBOT_RADIUS = 0.25
GRID = 0.08


def xy(points):
    return [(float(p["x"]), float(p["y"])) for p in points]


def point_in_poly(p, poly):
    x, y = p
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        ax, ay = poly[i]
        bx, by = poly[j]
        if (ay > y) != (by > y):
            at_x = (bx - ax) * (y - ay) / (by - ay) + ax
            if x < at_x:
                inside = not inside
        j = i
    return inside


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


def dist_to_poly(p, poly):
    return min(
        dist_point_segment(p[0], p[1], poly[i][0], poly[i][1], poly[(i + 1) % len(poly)][0], poly[(i + 1) % len(poly)][1])
        for i in range(len(poly))
    )


def dist_to_path(p, path):
    return min(
        dist_point_segment(p[0], p[1], path[i - 1][0], path[i - 1][1], path[i][0], path[i][1])
        for i in range(1, len(path))
    )


def bounds(*point_sets):
    pts = [p for points in point_sets for p in points]
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    pad = max(max(xs) - min(xs), max(ys) - min(ys), 1.0) * 0.08
    return min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad


def coverage(zone, forbiddens, true_path):
    xs = [p[0] for p in zone]
    ys = [p[1] for p in zone]
    safe = 0
    covered = 0
    y = min(ys) + GRID * 0.5
    while y < max(ys):
        x = min(xs) + GRID * 0.5
        while x < max(xs):
            p = (x, y)
            if point_in_poly(p, zone) and not any(point_in_poly(p, f) for f in forbiddens):
                safe += 1
                if dist_to_path(p, true_path) <= ROBOT_RADIUS + GRID * 0.75:
                    covered += 1
            x += GRID
        y += GRID
    return 100.0 * covered / safe if safe else 0.0


def add_route(ax, path, color, label, dashed=False, width=1.6):
    if len(path) < 2:
        return
    if dashed:
        ax.plot(
            [p[0] for p in path],
            [p[1] for p in path],
            linestyle="--",
            color=color,
            linewidth=width,
            label=label,
            zorder=4,
        )
        return
    segments = [[path[i], path[i + 1]] for i in range(len(path) - 1)]
    lc = LineCollection(segments, colors=color, linewidths=width, alpha=0.95, zorder=6, label=label)
    ax.add_collection(lc)


def analyze_case(case, name):
    trace_path = OUT / f"{name}_trace.csv"
    rows = []
    with trace_path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    true_path = [(float(r["true_x"]), float(r["true_y"])) for r in rows]
    planned = xy(case["path"])
    zone = xy(case["zone"])
    forbiddens = [xy(f) for f in case["forbiddens"]]

    forbidden_center_samples = 0
    min_forbidden_clearance = None
    for p in true_path:
        for f in forbiddens:
            if point_in_poly(p, f):
                forbidden_center_samples += 1
            d = dist_to_poly(p, f)
            min_forbidden_clearance = d if min_forbidden_clearance is None else min(min_forbidden_clearance, d)

    route_cte = [abs(float(r["route_cte_m"])) for r in rows if r.get("route_cte_m") not in ("", None)]
    stripe_cte = [abs(float(r["stripe_cte_m"])) for r in rows if r.get("stripe_cte_m") not in ("", None)]
    final = rows[-1] if rows else {}
    cov = coverage(zone, forbiddens, true_path) if true_path else 0.0
    metrics = {
        "name": case["name"],
        "routePointCount": len(planned),
        "plannedDistanceM": float(case["distanceM"]),
        "durationS": float(final.get("time_s", 0.0)) if final else 0.0,
        "finalMissM": float(final.get("miss_m", 0.0)) if final else 0.0,
        "enteredForbiddenCenter": forbidden_center_samples > 0,
        "forbiddenCenterSamples": forbidden_center_samples,
        "minCenterDistanceToForbiddenM": None if min_forbidden_clearance is None else round(min_forbidden_clearance, 3),
        "robotFootprintTouchedForbidden": False if min_forbidden_clearance is None else min_forbidden_clearance <= ROBOT_RADIUS,
        "coveragePercent": round(cov, 2),
        "meanRouteCrossTrackM": round(sum(route_cte) / len(route_cte), 4) if route_cte else 0.0,
        "meanStripeCrossTrackM": round(sum(stripe_cte) / len(stripe_cte), 4) if stripe_cte else 0.0,
    }

    metrics_path = OUT / f"{name}_metrics.json"
    metrics_path.write_text(json.dumps(metrics, indent=2), encoding="utf-8")

    fig, ax = plt.subplots(figsize=(8.5, 8.0), dpi=150)
    ax.add_patch(Polygon(zone, closed=True, facecolor="#7bd88f", edgecolor="#1f7a3b", linewidth=2.0, alpha=0.22, label="green zone"))
    for f in forbiddens:
        ax.add_patch(Polygon(f, closed=True, facecolor="#ff5c5c", edgecolor="#b00020", linewidth=1.8, alpha=0.42, label="forbidden"))
    add_route(ax, planned, "#222222", "planned", dashed=True, width=1.1)
    add_route(ax, true_path, "#0b5cad", "HITL true path", width=1.8)
    if true_path:
        ax.scatter([true_path[0][0]], [true_path[0][1]], marker="s", s=55, c="#111827", label="start", zorder=9)
        ax.scatter([true_path[-1][0]], [true_path[-1][1]], marker="*", s=120, c="#f5a623", label="finish", zorder=9)
    min_x, max_x, min_y, max_y = bounds(zone, planned, true_path, *forbiddens)
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x, m")
    ax.set_ylabel("y, m")
    ax.set_title(
        f"{case['name']}\ncoverage={metrics['coveragePercent']:.1f}% | "
        f"forbidden touch={'YES' if metrics['robotFootprintTouchedForbidden'] else 'NO'} | "
        f"route CTE={metrics['meanRouteCrossTrackM']:.3f} m"
    )
    handles, labels = ax.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    ax.legend(unique.values(), unique.keys(), loc="best", fontsize=8)
    fig.tight_layout()
    screenshot = OUT / f"{name}_hitl_forbidden_coverage.png"
    fig.savefig(screenshot)
    plt.close(fig)
    return metrics, screenshot


def main():
    cases = json.loads(ROUTES_JSON.read_text(encoding="utf-8"))
    if len(sys.argv) > 1:
        indices = [int(x) for x in sys.argv[1:]]
    else:
        indices = list(range(1, len(cases) + 1))
    all_metrics = []
    for idx in indices:
        name = f"stress_{idx:02d}"
        metrics, screenshot = analyze_case(cases[idx - 1], name)
        all_metrics.append(metrics)
        print(json.dumps(metrics, ensure_ascii=False))
        print(f"screenshot={screenshot}")
    if len(all_metrics) > 1:
        summary = OUT / "stress_hitl_summary.json"
        summary.write_text(json.dumps(all_metrics, indent=2), encoding="utf-8")
        print(f"summary={summary}")


if __name__ == "__main__":
    main()
