import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Polygon


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / ".pio" / "build_root"
ROUTES_JSON = OUT / "stress_routes" / "stress_routes.json"


def xy(points):
    return [(float(p["x"]), float(p["y"])) for p in points]


def read_trace(name):
    path = OUT / f"{name}_trace.csv"
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        return [(float(r["true_x"]), float(r["true_y"])) for r in csv.DictReader(f)]


def read_metrics(name):
    path = OUT / f"{name}_metrics.json"
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def add_line(ax, points, color, width, label=None, dashed=False, zorder=5):
    if len(points) < 2:
        return
    if dashed:
        ax.plot(
            [p[0] for p in points],
            [p[1] for p in points],
            color=color,
            linewidth=width,
            linestyle="--",
            alpha=0.8,
            label=label,
            zorder=zorder,
        )
        return
    segments = [[points[i], points[i + 1]] for i in range(len(points) - 1)]
    ax.add_collection(
        LineCollection(segments, colors=color, linewidths=width, alpha=0.95, label=label, zorder=zorder)
    )


def set_bounds(ax, *point_sets):
    pts = [p for points in point_sets for p in points]
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    if not xs:
        return
    span = max(max(xs) - min(xs), max(ys) - min(ys), 1.0)
    pad = span * 0.10
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad, max(ys) + pad)


def main():
    cases = json.loads(ROUTES_JSON.read_text(encoding="utf-8"))
    fig, axes = plt.subplots(3, 3, figsize=(16, 15), dpi=150)
    axes = axes.flatten()

    for idx, case in enumerate(cases[:9], start=1):
        ax = axes[idx - 1]
        name = f"stress_{idx:02d}"
        zone = xy(case["zone"])
        forbiddens = [xy(f) for f in case["forbiddens"]]
        planned = xy(case["path"])
        actual = read_trace(name)
        metrics = read_metrics(name)

        ax.add_patch(
            Polygon(zone, closed=True, facecolor="#7bd88f", edgecolor="#1f7a3b", linewidth=1.2, alpha=0.22)
        )
        for forbidden in forbiddens:
            ax.add_patch(
                Polygon(forbidden, closed=True, facecolor="#ff5c5c", edgecolor="#b00020", linewidth=1.1, alpha=0.42)
            )
        add_line(ax, planned, "#222222", 0.8, dashed=True, zorder=4)
        add_line(ax, actual, "#0b5cad", 1.25, zorder=6)
        if actual:
            ax.scatter([actual[0][0]], [actual[0][1]], marker="s", s=24, c="#111827", zorder=9)
            ax.scatter([actual[-1][0]], [actual[-1][1]], marker="*", s=52, c="#f5a623", zorder=9)

        all_forbidden_points = [p for forbidden in forbiddens for p in forbidden]
        set_bounds(ax, zone, planned, actual, all_forbidden_points)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.22, linewidth=0.6)
        ax.tick_params(labelsize=7)

        cover = metrics.get("coveragePercent")
        miss = metrics.get("finalMissM")
        cte = metrics.get("meanRouteCrossTrackM")
        touch = metrics.get("robotFootprintTouchedForbidden")
        title = case["name"]
        if len(title) > 42:
            title = title[:39] + "..."
        ax.set_title(
            f"{idx}. {title}\n"
            f"cover={cover:.1f}% miss={miss:.2f}m cte={cte:.3f}m "
            f"red={'TOUCH' if touch else 'OK'}",
            fontsize=9,
        )

    fig.suptitle("HITL actual trajectories: green=cleaning zone, red=forbidden, blue=robot path", fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, 0.975))
    out = OUT / "stress_hitl_overview.png"
    fig.savefig(out)
    plt.close(fig)
    print(out)


if __name__ == "__main__":
    main()
