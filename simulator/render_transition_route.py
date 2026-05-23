import json
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / ".pio" / "build_root" / "transition_route"
JSON_PATH = OUT / "transition_route.json"


def xy(points):
    return [(float(p["x"]), float(p["y"])) for p in points]


def add_polyline(ax, points, color, width, label, dash=None, zorder=5):
    if len(points) < 2:
        return
    ax.plot(
        [p[0] for p in points],
        [p[1] for p in points],
        color=color,
        linewidth=width,
        label=label,
        linestyle=dash or "-",
        zorder=zorder,
    )


def main():
    data = json.loads(JSON_PATH.read_text(encoding="utf-8"))
    zones = [xy(z) for z in data["zones"]]
    forbiddens = [xy(f) for f in data["forbiddens"]]
    transitions = [xy(t) for t in data["transitions"]]
    path = xy(data["path"])

    fig, ax = plt.subplots(figsize=(10, 5), dpi=160)
    for idx, zone in enumerate(zones):
        ax.add_patch(
            Polygon(
                zone,
                closed=True,
                facecolor="#7bd88f",
                edgecolor="#1f7a3b",
                linewidth=2.0,
                alpha=0.22,
                label="cleaning zone" if idx == 0 else None,
            )
        )
    for idx, forbidden in enumerate(forbiddens):
        ax.add_patch(
            Polygon(
                forbidden,
                closed=True,
                facecolor="#ff5c5c",
                edgecolor="#b00020",
                linewidth=1.8,
                alpha=0.42,
                label="forbidden" if idx == 0 else None,
            )
        )
    for idx, transition in enumerate(transitions):
        add_polyline(
            ax,
            transition,
            "#00a6d6",
            3.0,
            "transition line" if idx == 0 else None,
            dash="--",
            zorder=7,
        )
    add_polyline(ax, path, "#222222", 1.2, "planned route", zorder=6)
    if path:
        ax.scatter([path[0][0]], [path[0][1]], c="#111827", marker="s", s=60, label="start", zorder=9)
        ax.scatter([path[-1][0]], [path[-1][1]], c="#0b63ff", marker="*", s=130, label="finish", zorder=9)

    pts = [p for group in [*zones, *forbiddens, *transitions, path] for p in group]
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    pad = 0.7
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad, max(ys) + pad)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.set_title(f"{data['name']} | {data['pointCount']} pts")
    ax.set_xlabel("x, m")
    ax.set_ylabel("y, m")
    ax.legend(loc="best")
    fig.tight_layout()
    out = OUT / "transition_route.png"
    fig.savefig(out)
    plt.close(fig)
    print(out)


if __name__ == "__main__":
    main()
