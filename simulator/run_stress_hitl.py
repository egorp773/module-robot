import json
import math
import os
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / ".pio" / "build_root"
ROUTES_JSON = OUT / "stress_routes" / "stress_routes.json"


def route_arg(points):
    return ";".join(f"{p['x']:.3f},{p['y']:.3f}" for p in points)


def heading_deg(a, b):
    dx = b["x"] - a["x"]
    dy = b["y"] - a["y"]
    return math.degrees(math.atan2(dx, dy)) % 360.0


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM4"
    cases = json.loads(ROUTES_JSON.read_text(encoding="utf-8"))
    if len(sys.argv) > 2:
        indices = [int(x) for x in sys.argv[2:]]
    else:
        indices = list(range(1, len(cases) + 1))

    for idx in indices:
        case = cases[idx - 1]
        points = case["path"]
        name = f"stress_{idx:02d}"
        start = points[0]
        heading = heading_deg(points[0], points[1]) if len(points) > 1 else 0.0
        duration_scale = float(os.environ.get("STRESS_HITL_DURATION_SCALE", "1.0"))
        read_timeout_s = os.environ.get("STRESS_HITL_READ_TIMEOUT_S", "0.02")
        duration = max(90.0, float(case["distanceM"]) / 0.22 + 180.0) * duration_scale
        cmd = [
            sys.executable,
            str(ROOT / "simulator" / "hitl_runner.py"),
            "--port",
            port,
            f"--route={route_arg(points)}",
            "--start",
            f"{start['x']:.3f}",
            f"{start['y']:.3f}",
            f"{heading:.2f}",
            "--duration",
            f"{duration:.1f}",
            "--dt",
            os.environ.get("STRESS_HITL_DT", "0.05"),
            "--name",
            name,
            "--realtime-scale",
            "0",
            "--read-timeout-s",
            read_timeout_s,
            "--status-interval-s",
            os.environ.get("STRESS_HITL_STATUS_INTERVAL_S", "20"),
            "--gps-noise-m",
            os.environ.get("STRESS_HITL_GPS_NOISE_M", "0.025"),
            "--gps-spike-rate",
            os.environ.get("STRESS_HITL_GPS_SPIKE_RATE", "0.002"),
            "--gps-spike-m",
            os.environ.get("STRESS_HITL_GPS_SPIKE_M", "0.12"),
            "--gps-dropout-rate",
            os.environ.get("STRESS_HITL_GPS_DROPOUT_RATE", "0.004"),
            "--imu-noise-deg",
            os.environ.get("STRESS_HITL_IMU_NOISE_DEG", "0.45"),
            "--imu-drift-deg-s",
            os.environ.get("STRESS_HITL_IMU_DRIFT_DEG_S", "0.003"),
            "--motor-delay-ms",
            os.environ.get("STRESS_HITL_MOTOR_DELAY_MS", "140"),
            "--motor-inertia",
            os.environ.get("STRESS_HITL_MOTOR_INERTIA", "0.22"),
            "--slip-std",
            os.environ.get("STRESS_HITL_SLIP_STD", "0.035"),
            "--track-bias",
            os.environ.get("STRESS_HITL_TRACK_BIAS", "0.015"),
            "--no-plot",
            "--no-html-map",
        ]
        for forbidden in case.get("forbiddens", []):
            cmd.extend(["--forbid", route_arg(forbidden)])
        size = case.get("sizeM", {})
        width = float(size.get("width", 0.0) or 0.0)
        height = float(size.get("height", 0.0) or 0.0)
        print(f"=== HITL {idx}/{len(cases)}: {case['name']} ===", flush=True)
        print(
            f"size={width:.1f}x{height:.1f}m points={len(points)} "
            f"turns={case.get('turnCount', 0)} sharp_turns={case.get('sharpTurnCount', 0)} "
            f"distance={case['distanceM']:.1f}m planned_time={case.get('estimatedRunTimeMin', duration / 60.0):.1f}min "
            f"hitl_duration={duration:.1f}s",
            flush=True,
        )
        subprocess.run(cmd, cwd=ROOT, check=True)
        subprocess.run(
            [sys.executable, str(ROOT / "simulator" / "analyze_stress_hitl.py"), str(idx)],
            cwd=ROOT,
            check=True,
        )


if __name__ == "__main__":
    main()
