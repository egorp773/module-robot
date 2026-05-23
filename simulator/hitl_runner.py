#!/usr/bin/env python3
import argparse
import csv
import json
import math
import random
import sys
import time
import webbrowser
from collections import deque
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import serial
from serial.tools import list_ports


MAX_CMD = 70.0
MAX_SPEED_MPS = 0.5
TRACK_WIDTH_M = 0.38


def normalize360(value: float) -> float:
    while value >= 360.0:
        value -= 360.0
    while value < 0.0:
        value += 360.0
    return value


def pick_port(requested: Optional[str]) -> str:
    if requested:
        return requested
    ports = [p.device for p in list_ports.comports()]
    if not ports:
        raise RuntimeError("No serial ports found. Plug in ESP32 and check Device Manager.")
    return ports[0]


class TrackPhysics:
    # CRITICAL FIX: Added motor inertia (actual_speed += (target - actual) * 0.3)
    MOTOR_INERTIA = 0.3  # 30% of difference per tick

    def __init__(
        self,
        x: float,
        y: float,
        heading: float,
        dt: float,
        seed: int,
        motor_delay_s: float = 0.10,
        motor_inertia: float = 0.3,
        speed_scale: float = 1.0,
        track_bias: float = 0.0,
        slip_std: float = 0.015,
    ) -> None:
        self.x = x
        self.y = y
        self.heading = normalize360(heading)
        self.speed = 0.0
        self.dt = dt
        self.rng = random.Random(seed)
        self.motor_inertia = max(0.01, min(1.0, motor_inertia))
        self.speed_scale = speed_scale
        self.track_bias = max(-0.20, min(0.20, track_bias))
        self.slip_std = max(0.0, slip_std)
        # Motor command delay (100ms latency)
        delay_ticks = max(1, round(max(0.0, motor_delay_s) / dt))
        self.delay = deque([(0, 0)] * delay_ticks, maxlen=delay_ticks)
        # Motor inertia state
        self._left_actual = 0.0
        self._right_actual = 0.0

    def update(self, left_cmd: int, right_cmd: int) -> None:
        self.delay.append((left_cmd, right_cmd))
        left_cmd, right_cmd = self.delay[0]

        # Apply motor inertia (real motors don't instantly reach target speed)
        self._left_actual += (left_cmd - self._left_actual) * self.motor_inertia
        self._right_actual += (right_cmd - self._right_actual) * self.motor_inertia

        left = self._motor_output(int(round(self._left_actual)))
        right = self._motor_output(int(round(self._right_actual)))
        left_velocity = (left / MAX_CMD) * MAX_SPEED_MPS * self.speed_scale * (1.0 - self.track_bias)
        right_velocity = (right / MAX_CMD) * MAX_SPEED_MPS * self.speed_scale * (1.0 + self.track_bias)

        slip = 1.0 + self.rng.gauss(0.0, self.slip_std)
        velocity = (left_velocity + right_velocity) * 0.5 * slip
        omega = (left_velocity - right_velocity) / TRACK_WIDTH_M

        heading_rad = math.radians(self.heading)
        self.x += velocity * math.sin(heading_rad) * self.dt
        self.y += velocity * math.cos(heading_rad) * self.dt
        self.heading = normalize360(self.heading + math.degrees(omega * self.dt))
        self.speed = abs(velocity)

    @staticmethod
    def _motor_output(command: int) -> float:
        if abs(command) < 3:
            return 0.0
        return float(command) * 0.98


def read_lines_until_motor(ser: serial.Serial, timeout_s: float) -> Tuple[Optional[int], Optional[int], List[str]]:
    deadline = time.monotonic() + timeout_s
    lines: List[str] = []
    motor_left = None
    motor_right = None

    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            continue
        lines.append(line)
        if line.startswith("MOTORS,"):
            parts = line.split(",")
            if len(parts) >= 3:
                try:
                    motor_left = int(parts[1])
                    motor_right = int(parts[2])
                    break
                except ValueError:
                    pass
    return motor_left, motor_right, lines


def send_line(ser: serial.Serial, line: str) -> None:
    ser.write((line + "\n").encode("ascii"))
    ser.flush()


def is_hitl_log_line(line: str) -> bool:
    return line.startswith("SIM_") or line.startswith("MOTORS,") or line.startswith("NAV:")


def is_final_waypoint_line(line: str, waypoint_count: int) -> bool:
    if not line.startswith("NAV: Waypoint"):
        return False
    parts = line.split()
    if len(parts) < 3:
        return False
    try:
      reached_index = int(parts[2])
    except ValueError:
      return False
    return reached_index >= waypoint_count - 1


def parse_route(value: Optional[str], target_x: float, target_y: float) -> List[Tuple[float, float]]:
    if not value:
        return [(target_x, target_y)]
    route: List[Tuple[float, float]] = []
    for token in value.split(";"):
        token = token.strip()
        if not token:
            continue
        parts = token.split(",")
        if len(parts) != 2:
            raise ValueError("Route format must be x,y;x,y;...")
        route.append((float(parts[0]), float(parts[1])))
    if len(route) < 2:
        raise ValueError("Route must contain at least two points")
    return route


def parse_forbiddens(values: Optional[List[str]]) -> List[List[Tuple[float, float]]]:
    forbiddens: List[List[Tuple[float, float]]] = []
    if not values:
        return forbiddens
    for forbid_spec in values:
        polygon: List[Tuple[float, float]] = []
        for token in forbid_spec.split(";"):
            token = token.strip()
            if not token:
                continue
            parts = token.split(",")
            if len(parts) != 2:
                raise ValueError("Forbidden polygon format must be x,y;x,y;...")
            polygon.append((float(parts[0]), float(parts[1])))
        if len(polygon) >= 3:
            forbiddens.append(polygon)
    return forbiddens


def route_distance(route: List[Tuple[float, float]]) -> float:
    return sum(
        math.hypot(route[i][0] - route[i - 1][0], route[i][1] - route[i - 1][1])
        for i in range(1, len(route))
    )


def point_segment_cross_track(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> Tuple[float, float]:
    dx = bx - ax
    dy = by - ay
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-9:
        return 0.0, 0.0
    t = ((px - ax) * dx + (py - ay) * dy) / length_sq
    t_clamped = min(1.0, max(0.0, t))
    length = math.sqrt(length_sq)
    signed = (dx * (py - ay) - dy * (px - ax)) / length
    return signed, t_clamped


def stripe_cross_track(x: float, y: float, route: List[Tuple[float, float]]) -> Optional[float]:
    best: Optional[Tuple[float, float]] = None
    for idx in range(1, len(route)):
        ax, ay = route[idx - 1]
        bx, by = route[idx]
        dx = bx - ax
        dy = by - ay
        if abs(dx) < 1.0 or abs(dx) <= abs(dy):
            continue
        cte, progress = point_segment_cross_track(x, y, ax, ay, bx, by)
        if progress < 0.02 or progress > 0.98:
            continue
        distance_to_line = abs(cte)
        if best is None or distance_to_line < best[0]:
            best = (distance_to_line, cte)
    return best[1] if best is not None else None


def route_cross_track(x: float, y: float, route: List[Tuple[float, float]]) -> Optional[float]:
    best: Optional[Tuple[float, float]] = None
    for idx in range(1, len(route)):
        ax, ay = route[idx - 1]
        bx, by = route[idx]
        cte, progress = point_segment_cross_track(x, y, ax, ay, bx, by)
        if progress < 0.02 or progress > 0.98:
            continue
        distance_to_line = abs(cte)
        if best is None or distance_to_line < best[0]:
            best = (distance_to_line, cte)
    return best[1] if best is not None else None


def _map_bounds(rows: List[Dict[str, Any]], route: List[Tuple[float, float]],
                forbiddens: List[List[Tuple[float, float]]] = None) -> Tuple[float, float, float, float]:
    xs = [p[0] for p in route]
    ys = [p[1] for p in route]
    xs.extend(float(r["true_x"]) for r in rows)
    ys.extend(float(r["true_y"]) for r in rows)
    if forbiddens:
        for poly in forbiddens:
            xs.extend(p[0] for p in poly)
            ys.extend(p[1] for p in poly)
    if not xs:
        return -1.0, 1.0, -1.0, 1.0
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    span = max(max_x - min_x, max_y - min_y, 1.0)
    pad = max(0.35, span * 0.08)
    return min_x - pad, max_x + pad, min_y - pad, max_y + pad


def save_html_map(rows: List[Dict[str, Any]], route: List[Tuple[float, float]],
                  path: Path, live: bool,
                  forbiddens: List[List[Tuple[float, float]]] = None) -> None:
    min_x, max_x, min_y, max_y = _map_bounds(rows, route, forbiddens)
    payload = {
        "route": [{"x": x, "y": y} for x, y in route],
        "forbiddens": [[{"x": p[0], "y": p[1]} for p in poly] for poly in (forbiddens or [])],
        "rows": rows,
        "bounds": {"minX": min_x, "maxX": max_x, "minY": min_y, "maxY": max_y},
        "live": live,
        "generatedAt": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    data_json = json.dumps(payload, separators=(",", ":"))
    refresh = '<meta http-equiv="refresh" content="1">' if live else ""
    html = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  {refresh}
  <title>ESP32 HITL map</title>
  <style>
    html, body {{ margin: 0; height: 100%; font-family: Arial, sans-serif; background: #f4f6f8; color: #1d242c; }}
    #wrap {{ display: grid; grid-template-columns: 1fr 280px; height: 100%; }}
    #map {{ width: 100%; height: 100%; background: #ffffff; display: block; }}
    aside {{ border-left: 1px solid #cfd6dd; background: #f8fafc; padding: 16px; box-sizing: border-box; }}
    h1 {{ font-size: 18px; margin: 0 0 14px; }}
    .metric {{ display: flex; justify-content: space-between; gap: 12px; padding: 8px 0; border-bottom: 1px solid #dde3e8; font-size: 14px; }}
    .label {{ color: #5f6b76; }}
    .value {{ font-weight: 700; text-align: right; }}
    .legend {{ margin-top: 18px; font-size: 13px; line-height: 1.7; }}
    .swatch {{ display: inline-block; width: 18px; height: 3px; margin-right: 8px; vertical-align: middle; }}
    @media (max-width: 820px) {{ #wrap {{ grid-template-columns: 1fr; grid-template-rows: 1fr auto; }} aside {{ border-left: 0; border-top: 1px solid #cfd6dd; }} }}
  </style>
</head>
<body>
<div id="wrap">
  <canvas id="map"></canvas>
  <aside>
    <h1>ESP32 HITL map</h1>
    <div id="metrics"></div>
    <div class="legend">
      <div><span class="swatch" style="background:#20262e"></span>Route</div>
      <div><span class="swatch" style="background:#0b5cad"></span>True path</div>
      <div><span class="swatch" style="background:#9aa3ad"></span>GPS samples</div>
      <div><span class="swatch" style="background:#d93636"></span>Robot heading</div>
      <div><span class="swatch" style="background:#dc3545"></span>Forbidden zone</div>
    </div>
  </aside>
</div>
<script>
const data = {data_json};
const canvas = document.getElementById("map");
const ctx = canvas.getContext("2d");
const metrics = document.getElementById("metrics");

function resize() {{
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.floor(canvas.clientWidth * ratio));
  canvas.height = Math.max(1, Math.floor(canvas.clientHeight * ratio));
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  draw();
}}

function fmt(value, digits = 3) {{
  return Number.isFinite(value) ? value.toFixed(digits) : "-";
}}

function project(x, y) {{
  const b = data.bounds;
  const pad = 48;
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  const sx = (w - pad * 2) / Math.max(0.001, b.maxX - b.minX);
  const sy = (h - pad * 2) / Math.max(0.001, b.maxY - b.minY);
  const s = Math.min(sx, sy);
  const usedW = (b.maxX - b.minX) * s;
  const usedH = (b.maxY - b.minY) * s;
  const ox = (w - usedW) / 2;
  const oy = (h - usedH) / 2;
  return [ox + (x - b.minX) * s, h - (oy + (y - b.minY) * s), s];
}}

function polyline(points, color, width, dash = []) {{
  if (points.length < 2) return;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.setLineDash(dash);
  ctx.beginPath();
  points.forEach((p, i) => {{
    const [x, y] = project(p.x, p.y);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  }});
  ctx.stroke();
  ctx.restore();
}}

function drawGrid() {{
  const b = data.bounds;
  const span = Math.max(b.maxX - b.minX, b.maxY - b.minY);
  const step = span > 20 ? 5 : span > 8 ? 2 : 1;
  ctx.save();
  ctx.strokeStyle = "#e4e8ed";
  ctx.lineWidth = 1;
  ctx.fillStyle = "#71808d";
  ctx.font = "12px Arial";
  for (let x = Math.ceil(b.minX / step) * step; x <= b.maxX; x += step) {{
    const [px] = project(x, b.minY);
    ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, canvas.clientHeight); ctx.stroke();
    ctx.fillText(`${{fmt(x, 0)}}m`, px + 3, canvas.clientHeight - 10);
  }}
  for (let y = Math.ceil(b.minY / step) * step; y <= b.maxY; y += step) {{
    const [, py] = project(b.minX, y);
    ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(canvas.clientWidth, py); ctx.stroke();
    ctx.fillText(`${{fmt(y, 0)}}m`, 8, py - 3);
  }}
  ctx.restore();
}}

function drawWaypoints() {{
  ctx.save();
  ctx.font = "12px Arial";
  data.route.forEach((p, i) => {{
    const [x, y] = project(p.x, p.y);
    ctx.fillStyle = i === 0 ? "#147a3d" : i === data.route.length - 1 ? "#c21f39" : "#20262e";
    ctx.beginPath(); ctx.arc(x, y, 5, 0, Math.PI * 2); ctx.fill();
    ctx.fillText(String(i), x + 8, y - 8);
  }});
  ctx.restore();
}}

function drawRobot(row) {{
  if (!row) return;
  const [x, y, scale] = project(row.true_x, row.true_y);
  const heading = row.heading_deg * Math.PI / 180;
  const size = Math.max(12, Math.min(22, scale * 0.35));
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(heading);
  ctx.fillStyle = "#d93636";
  ctx.beginPath();
  ctx.moveTo(0, -size);
  ctx.lineTo(size * 0.55, size * 0.75);
  ctx.lineTo(-size * 0.55, size * 0.75);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}}

function drawForbiddens() {{
  if (!data.forbiddens || !data.forbiddens.length) return;
  ctx.save();
  data.forbiddens.forEach(poly => {{
    if (poly.length < 3) return;
    ctx.fillStyle = "rgba(220, 53, 69, 0.15)";
    ctx.strokeStyle = "#dc3545";
    ctx.lineWidth = 2;
    ctx.beginPath();
    const [x0, y0] = project(poly[0].x, poly[0].y);
    ctx.moveTo(x0, y0);
    for (let i = 1; i < poly.length; i++) {{
      const [x, y] = project(poly[i].x, poly[i].y);
      ctx.lineTo(x, y);
    }}
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
  }});
  ctx.restore();
}}

function drawMetrics(row) {{
  const cteRows = data.rows.filter(r => r.stripe_cte_m !== "" && Number.isFinite(Number(r.stripe_cte_m)));
  const avgCte = cteRows.length ? cteRows.reduce((s, r) => s + Math.abs(Number(r.stripe_cte_m)), 0) / cteRows.length : 0;
  const routeCteRows = data.rows.filter(r => r.route_cte_m !== "" && Number.isFinite(Number(r.route_cte_m)));
  const avgRouteCte = routeCteRows.length ? routeCteRows.reduce((s, r) => s + Math.abs(Number(r.route_cte_m)), 0) / routeCteRows.length : 0;
  metrics.innerHTML = [
    ["Mode", data.live ? "LIVE" : "final"],
    ["Time", row ? `${{fmt(row.time_s, 2)}} s` : "-"],
    ["Position", row ? `${{fmt(row.true_x)}}, ${{fmt(row.true_y)}} m` : "-"],
    ["Heading", row ? `${{fmt(row.heading_deg, 1)}} deg` : "-"],
    ["Miss", row ? `${{fmt(row.miss_m)}} m` : "-"],
    ["Mean route CTE", `${{fmt(avgRouteCte)}} m`],
    ["Mean stripe CTE", `${{fmt(avgCte)}} m`],
    ["Motors", row ? `${{Math.round(row.left_cmd)}}, ${{Math.round(row.right_cmd)}}` : "-"],
    ["Updated", data.generatedAt],
  ].map(([k, v]) => `<div class="metric"><span class="label">${{k}}</span><span class="value">${{v}}</span></div>`).join("");
}}

function draw() {{
  ctx.clearRect(0, 0, canvas.clientWidth, canvas.clientHeight);
  drawGrid();
  drawForbiddens();
  polyline(data.route, "#20262e", 2, [7, 5]);
  const path = data.rows.map(r => ({{x: Number(r.true_x), y: Number(r.true_y)}}));
  const gps = data.rows.filter((_, i) => i % 5 === 0).map(r => ({{x: Number(r.gps_x), y: Number(r.gps_y)}}));
  ctx.save();
  ctx.fillStyle = "rgba(125, 137, 150, 0.42)";
  gps.forEach(p => {{ const [x, y] = project(p.x, p.y); ctx.beginPath(); ctx.arc(x, y, 2, 0, Math.PI * 2); ctx.fill(); }});
  ctx.restore();
  polyline(path, "#0b5cad", 3);
  drawWaypoints();
  const row = data.rows.length ? data.rows[data.rows.length - 1] : null;
  drawRobot(row);
  drawMetrics(row);
}}

window.addEventListener("resize", resize);
resize();
</script>
</body>
</html>
"""
    path.write_text(html, encoding="utf-8")


def save_plot(rows: List[Dict[str, Any]], route: List[Tuple[float, float]], path: Path, show: bool,
             forbiddens: List[List[Tuple[float, float]]] = None) -> None:
    if not show:
        import matplotlib
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    xs = [r["true_x"] for r in rows]
    ys = [r["true_y"] for r in rows]
    gps_xs = [r["gps_x"] for r in rows]
    gps_ys = [r["gps_y"] for r in rows]
    route_xs = [p[0] for p in route]
    route_ys = [p[1] for p in route]

    fig, ax = plt.subplots(figsize=(8, 6))

    # Draw forbidden zones
    if forbiddens:
        for poly in forbiddens:
            px = [p[0] for p in poly] + [poly[0][0]]
            py = [p[1] for p in poly] + [poly[0][1]]
            ax.fill(px, py, color="#dc3545", alpha=0.2, label="forbidden")
            ax.plot(px, py, color="#dc3545", linewidth=1.5)

    if len(route) > 1:
        ax.plot(route_xs, route_ys, color="#222222", linewidth=1.2, linestyle="--", label="route")
    ax.plot(xs, ys, color="#0b5cad", linewidth=2.0, label="true path")
    ax.scatter(gps_xs[::5], gps_ys[::5], s=6, color="#8a8f98", alpha=0.45, label="GPS samples")
    ax.scatter([route_xs[0]], [route_ys[0]], color="#147a3d", s=60, marker="o", label="start")
    ax.scatter([route_xs[-1]], [route_ys[-1]], color="#c21f39", s=80, marker="x", label="target")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.set_xlabel("x east, m")
    ax.set_ylabel("y north, m")
    ax.set_title("ESP32 HITL trajectory")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    if show:
        plt.show()
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="ESP32 rover HITL runner over USB Serial")
    parser.add_argument("--port", default=None, help="Serial port, for example COM4")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--target", nargs=2, type=float, default=(5.0, 0.0), metavar=("X", "Y"))
    parser.add_argument("--route", default=None, help="Route points: x,y;x,y;...")
    parser.add_argument("--start", nargs=3, type=float, default=(0.0, 0.0, 0.0), metavar=("X", "Y", "HEADING"))
    parser.add_argument("--duration", type=float, default=None,
                        help="Simulation duration. Defaults to 45s for point targets, or route distance / 0.28 + 60s.")
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--realtime-scale", type=float, default=1.0,
                        help="Wall-clock sleep scale for each simulation tick. Use 0 for fast HITL.")
    parser.add_argument("--read-timeout-s", type=float, default=None,
                        help="Serial wait after each GPS sample. Defaults to max(dt, 0.05).")
    parser.add_argument("--seed", type=int, default=123)
    parser.add_argument("--gps-noise-m", type=float, default=0.015)
    parser.add_argument("--gps-bias-x-m", type=float, default=0.0)
    parser.add_argument("--gps-bias-y-m", type=float, default=0.0)
    parser.add_argument("--gps-dropout-rate", type=float, default=0.0)
    parser.add_argument("--gps-spike-rate", type=float, default=0.0)
    parser.add_argument("--gps-spike-m", type=float, default=0.0)
    parser.add_argument("--imu-noise-deg", type=float, default=0.25)
    parser.add_argument("--imu-drift-deg-s", type=float, default=0.0)
    parser.add_argument("--motor-delay-ms", type=float, default=100.0)
    parser.add_argument("--motor-inertia", type=float, default=0.3)
    parser.add_argument("--speed-scale", type=float, default=1.0)
    parser.add_argument("--track-bias", type=float, default=0.0)
    parser.add_argument("--slip-std", type=float, default=0.015)
    parser.add_argument("--k-heading", type=float, default=None, help="Navigation K_HEADING gain")
    parser.add_argument("--k-crosstrack", type=float, default=None, help="Navigation K_CROSSTRACK gain")
    parser.add_argument("--name", default="hitl", help="Output file prefix")
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--live-map", action="store_true", help="Continuously rewrite an auto-refreshing HTML map during the run")
    parser.add_argument("--open-map", action="store_true", help="Open the live/final HTML map in the default browser")
    parser.add_argument("--map-update-s", type=float, default=0.5, help="Live HTML map update period")
    parser.add_argument("--no-html-map", action="store_true", help="Do not write the final interactive HTML map")
    parser.add_argument("--status-interval-s", type=float, default=2.0)
    parser.add_argument("--forbid", action="append", default=None,
                        help="Forbidden zone polygon: x1,y1;x2,y2;... (can be used multiple times)")
    parser.add_argument("--out-dir", default=str(Path(__file__).resolve().parents[1] / ".pio" / "build_root"))
    args = parser.parse_args()

    port = pick_port(args.port)
    target_x, target_y = args.target
    start_x, start_y, start_heading = args.start
    route = parse_route(args.route, target_x, target_y)
    forbiddens = parse_forbiddens(args.forbid)
    target_x, target_y = route[-1]
    duration = args.duration
    if duration is None:
        duration = 45.0 if len(route) <= 1 else max(45.0, route_distance(route) / 0.22 + 120.0)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / f"{args.name}_trace.csv"
    png_path = out_dir / f"{args.name}_trajectory.png"
    html_path = out_dir / f"{args.name}_map.html"
    serial_log_path = out_dir / f"{args.name}_serial.log"

    rng = random.Random(args.seed)
    robot = TrackPhysics(
        start_x,
        start_y,
        start_heading,
        args.dt,
        args.seed,
        motor_delay_s=args.motor_delay_ms / 1000.0,
        motor_inertia=args.motor_inertia,
        speed_scale=args.speed_scale,
        track_bias=args.track_bias,
        slip_std=args.slip_std,
    )
    imu_drift = 0.0
    left = 0
    right = 0
    rows: List[Dict[str, Any]] = []
    serial_log: List[str] = []
    arrived_from_firmware = False
    next_map_write = time.monotonic()

    print(f"HITL port: {port}")
    print(f"Target: ({target_x:.2f}, {target_y:.2f}) m")
    print(f"Duration: {duration:.1f} s")
    if len(route) > 1:
        print("Route: " + ";".join(f"{x:.3f},{y:.3f}" for x, y in route))
    if args.live_map:
        save_html_map(rows, route, html_path, live=True, forbiddens=forbiddens)
        print(f"Live map: {html_path}")
        if args.open_map:
            webbrowser.open(html_path.resolve().as_uri())

    serial_timeout = 0.02
    if args.read_timeout_s is not None:
        serial_timeout = max(0.001, min(serial_timeout, args.read_timeout_s))

    with serial.Serial(port, args.baud, timeout=serial_timeout, write_timeout=1.0) as ser:
        time.sleep(2.0)
        ser.reset_input_buffer()
        if args.route:
            route_spec = ";".join(f"{x:.3f},{y:.3f}" for x, y in route)
            if len(route_spec) <= 180:
                send_line(ser, f"SIM_ROUTE,{route_spec}")
            else:
                send_line(ser, f"SIM_ROUTE_BEGIN,{len(route)}")
                time.sleep(0.10)
                for idx, (x, y) in enumerate(route):
                    send_line(ser, f"SIM_ROUTE_WP,{idx},{x:.3f},{y:.3f}")
                    time.sleep(0.025)
                send_line(ser, "SIM_ROUTE_END")
        else:
            send_line(ser, f"SIM_START,{target_x:.3f},{target_y:.3f}")

        if forbiddens:
            send_line(ser, f"SIM_FORBID,{len(forbiddens)}")
            time.sleep(0.05)
            for poly_idx, polygon in enumerate(forbiddens):
                for pt_idx, (x, y) in enumerate(polygon):
                    send_line(ser, f"SIM_FORBID_PT,{poly_idx},{pt_idx},{x:.3f},{y:.3f}")
                    time.sleep(0.01)
            send_line(ser, "SIM_FORBID_END")
            time.sleep(0.05)

        start_deadline = time.monotonic() + 5.0
        while time.monotonic() < start_deadline:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if line:
                serial_log.append(line)
                print(f"SERIAL: {line}")
            if line.startswith("SIM_OK") or line.startswith("SIM_ROUTE_OK"):
                break
        else:
            raise RuntimeError("ESP32 did not answer SIM_OK. Is the updated rover firmware uploaded?")

        # Set navigation parameters if specified
        if args.k_heading is not None or args.k_crosstrack is not None:
            kh = args.k_heading if args.k_heading is not None else 0.5
            kx = args.k_crosstrack if args.k_crosstrack is not None else 15.0
            send_line(ser, f"SIM_SETPARAMS,{kh},{kx}")
            time.sleep(0.1)

        next_tick = time.monotonic()
        steps = int(duration / args.dt)
        status_every_steps = max(1, round(max(args.dt, args.status_interval_s) / args.dt))
        for step in range(steps):
            if args.realtime_scale > 0:
                now = time.monotonic()
                if now < next_tick:
                    time.sleep(next_tick - now)
                next_tick += args.dt * args.realtime_scale

            robot.update(left, right)
            imu_drift += args.imu_drift_deg_s * args.dt

            gps_x = robot.x + args.gps_bias_x_m + rng.gauss(0.0, args.gps_noise_m)
            gps_y = robot.y + args.gps_bias_y_m + rng.gauss(0.0, args.gps_noise_m)
            quality = "FIXED"
            if args.gps_spike_rate > 0.0 and rng.random() < args.gps_spike_rate:
                angle = rng.uniform(0.0, math.pi * 2.0)
                radius = args.gps_spike_m * rng.uniform(0.4, 1.0)
                gps_x += math.cos(angle) * radius
                gps_y += math.sin(angle) * radius
            if args.gps_dropout_rate > 0.0 and rng.random() < args.gps_dropout_rate:
                quality = "FLOAT"
            heading = normalize360(robot.heading + imu_drift + rng.uniform(-args.imu_noise_deg, args.imu_noise_deg))
            send_line(ser, f"GPS,{gps_x:.4f},{gps_y:.4f},{heading:.2f},{robot.speed:.4f},{quality}")

            read_timeout = args.read_timeout_s
            if read_timeout is None:
                read_timeout = max(args.dt, 0.05)
            motor_left, motor_right, lines = read_lines_until_motor(ser, read_timeout)
            for line in lines:
                serial_log.append(line)
                if is_final_waypoint_line(line, len(route)):
                    arrived_from_firmware = True
            if motor_left is not None and motor_right is not None:
                left = motor_left
                right = motor_right

            miss = math.hypot(target_x - robot.x, target_y - robot.y)
            cte = stripe_cross_track(robot.x, robot.y, route)
            route_cte = route_cross_track(robot.x, robot.y, route)
            rows.append({
                "time_s": step * args.dt,
                "true_x": robot.x,
                "true_y": robot.y,
                "gps_x": gps_x,
                "gps_y": gps_y,
                "heading_deg": robot.heading,
                "gps_heading_deg": heading,
                "speed_mps": robot.speed,
                "left_cmd": float(left),
                "right_cmd": float(right),
                "miss_m": miss,
                "stripe_cte_m": cte if cte is not None else "",
                "route_cte_m": route_cte if route_cte is not None else "",
            })

            if args.live_map and rows and time.monotonic() >= next_map_write:
                save_html_map(rows, route, html_path, live=True, forbiddens=forbiddens)
                next_map_write = time.monotonic() + max(0.1, args.map_update_s)

            if step % status_every_steps == 0:
                print(f"t={step * args.dt:5.2f}s pos=({robot.x:6.3f},{robot.y:6.3f}) "
                      f"heading={robot.heading:6.1f} miss={miss:5.3f} motors=({left:3d},{right:3d})")

            if arrived_from_firmware and abs(left) < 3 and abs(right) < 3:
                break

        send_line(ser, "SIM_STOP")

    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else ["time_s"])
        writer.writeheader()
        writer.writerows(rows)
    serial_log_path.write_text("\n".join(serial_log) + "\n", encoding="utf-8")

    if rows:
        final = rows[-1]
        cte_values = [abs(float(r["stripe_cte_m"])) for r in rows if r["stripe_cte_m"] != ""]
        avg_cte = sum(cte_values) / len(cte_values) if cte_values else 0.0
        route_cte_values = [abs(float(r["route_cte_m"])) for r in rows if r["route_cte_m"] != ""]
        avg_route_cte = sum(route_cte_values) / len(route_cte_values) if route_cte_values else 0.0
        print(f"Final: x={final['true_x']:.3f} y={final['true_y']:.3f} "
              f"miss={final['miss_m']:.3f} m ({final['miss_m'] * 100:.1f} cm), "
              f"motors=({int(final['left_cmd'])},{int(final['right_cmd'])})")
        print(f"Mean route cross-track: {avg_route_cte:.3f} m ({avg_route_cte * 100:.1f} cm), samples={len(route_cte_values)}")
        print(f"Mean stripe cross-track: {avg_cte:.3f} m ({avg_cte * 100:.1f} cm), samples={len(cte_values)}")
    print(f"CSV log: {csv_path}")
    print(f"Serial log: {serial_log_path}")
    print("HITL serial log tail:")
    hitl_lines = [line for line in serial_log if is_hitl_log_line(line)]
    for line in hitl_lines[-25:]:
        print(f"  {line}")

    if not args.no_plot:
        save_plot(rows, route, png_path, show=True, forbiddens=forbiddens)
        print(f"Trajectory: {png_path}")
    if not args.no_html_map:
        save_html_map(rows, route, html_path, live=False, forbiddens=forbiddens)
        print(f"Interactive map: {html_path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
