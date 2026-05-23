import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Polygon


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / ".pio" / "build_root" / "stress_routes"
JSON_PATH = OUT_DIR / "stress_routes.json"


def xy(points):
    return [(p["x"], p["y"]) for p in points]


def bounds(case):
    pts = xy(case["zone"]) + xy(case["path"])
    for forbidden in case["forbiddens"]:
        pts += xy(forbidden)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    pad = 0.8
    return min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad


def add_route(ax, path):
    segments = [[path[i], path[i + 1]] for i in range(len(path) - 1)]
    lc = LineCollection(
        segments,
        cmap="viridis",
        linewidths=2.1,
        alpha=0.95,
        zorder=5,
    )
    lc.set_array(list(range(len(segments))))
    ax.add_collection(lc)

    stride = max(8, len(path) // 16)
    for i in range(0, len(path) - 1, stride):
        x0, y0 = path[i]
        x1, y1 = path[i + 1]
        dx = x1 - x0
        dy = y1 - y0
        length = math.hypot(dx, dy)
        if length < 0.08:
            continue
        ax.annotate(
            "",
            xy=(x0 + dx * 0.72, y0 + dy * 0.72),
            xytext=(x0 + dx * 0.42, y0 + dy * 0.42),
            arrowprops=dict(arrowstyle="->", color="#243b53", lw=1.2),
            zorder=7,
        )


def render_case(case, out_path, compact=False):
    fig, ax = plt.subplots(figsize=(7.2, 7.0) if not compact else (5.4, 5.0), dpi=150)

    zone = xy(case["zone"])
    ax.add_patch(
        Polygon(
            zone,
            closed=True,
            facecolor="#7bd88f",
            edgecolor="#1f7a3b",
            linewidth=2.0,
            alpha=0.24,
            zorder=1,
        )
    )

    for forbidden in case["forbiddens"]:
        ax.add_patch(
            Polygon(
                xy(forbidden),
                closed=True,
                facecolor="#ff5c5c",
                edgecolor="#b00020",
                linewidth=1.8,
                alpha=0.42,
                zorder=3,
            )
        )

    path = xy(case["path"])
    add_route(ax, path)

    sx, sy = path[0]
    ex, ey = path[-1]
    ax.scatter([sx], [sy], marker="s", s=62, c="#111827", zorder=9, label="start")
    ax.scatter([ex], [ey], marker="*", s=130, c="#0b5fff", zorder=9, label="finish")

    title = case["name"]
    ax.set_title(title, fontsize=10 if compact else 12, pad=8)
    ax.text(
        0.01,
        0.99,
        f"{case['pointCount']} pts | {case['distanceM']:.1f} m | cover {case['coveragePercent']:.1f}%",
        transform=ax.transAxes,
        ha="left",
        va="top",
        fontsize=8.5,
        bbox=dict(facecolor="white", edgecolor="#d0d7de", alpha=0.88, pad=3),
        zorder=10,
    )
    if case["minForbiddenClearanceM"] is not None:
        ax.text(
            0.01,
            0.925,
            f"min red clearance {case['minForbiddenClearanceM']:.2f} m",
            transform=ax.transAxes,
            ha="left",
            va="top",
            fontsize=8.5,
            bbox=dict(facecolor="white", edgecolor="#d0d7de", alpha=0.88, pad=3),
            zorder=10,
        )

    min_x, max_x, min_y, max_y = bounds(case)
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="#e5e7eb", linewidth=0.7)
    ax.set_xlabel("x, m")
    ax.set_ylabel("y, m")
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def render_overview(cases):
    fig, axes = plt.subplots(3, 3, figsize=(15, 14.5), dpi=150)
    for ax, case in zip(axes.ravel(), cases):
        zone = xy(case["zone"])
        ax.add_patch(
            Polygon(
                zone,
                closed=True,
                facecolor="#7bd88f",
                edgecolor="#1f7a3b",
                linewidth=1.4,
                alpha=0.24,
                zorder=1,
            )
        )
        for forbidden in case["forbiddens"]:
            ax.add_patch(
                Polygon(
                    xy(forbidden),
                    closed=True,
                    facecolor="#ff5c5c",
                    edgecolor="#b00020",
                    linewidth=1.2,
                    alpha=0.42,
                    zorder=3,
                )
            )
        path = xy(case["path"])
        add_route(ax, path)
        ax.scatter([path[0][0]], [path[0][1]], marker="s", s=38, c="#111827", zorder=9)
        ax.scatter([path[-1][0]], [path[-1][1]], marker="*", s=82, c="#0b5fff", zorder=9)
        ax.set_title(
            f"{case['name']}\n{case['pointCount']} pts | cover {case['coveragePercent']:.1f}%",
            fontsize=9,
        )
        min_x, max_x, min_y, max_y = bounds(case)
        ax.set_xlim(min_x, max_x)
        ax.set_ylim(min_y, max_y)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, color="#e5e7eb", linewidth=0.5)
        ax.tick_params(labelsize=7)
    fig.tight_layout()
    overview = OUT_DIR / "stress_routes_overview.png"
    fig.savefig(overview)
    plt.close(fig)
    return overview


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    cases = json.loads(JSON_PATH.read_text(encoding="utf-8"))
    rendered = []
    for idx, case in enumerate(cases, start=1):
        slug = "".join(ch if ch.isalnum() else "_" for ch in case["name"].lower())
        out_path = OUT_DIR / f"{idx:02d}_{slug}.png"
        render_case(case, out_path)
        rendered.append(out_path)
    overview = render_overview(cases)
    print(f"overview={overview}")
    for path in rendered:
        print(f"case={path}")


if __name__ == "__main__":
    main()
