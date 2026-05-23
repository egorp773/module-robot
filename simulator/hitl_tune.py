#!/usr/bin/env python3
"""
HITL Parameter Tuning Script
Runs multiple HITL tests with different parameters to find optimal settings.
"""
import argparse
import csv
import json
import math
import os
import random
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple, Optional

# Add simulator directory to path
sys.path.insert(0, str(Path(__file__).parent))


@dataclass
class TestResult:
    params: dict
    miss_m: float
    time_s: float
    stuck_count: int
    success: bool


def run_single_test(
    port: str,
    target_x: float,
    target_y: float,
    start_x: float,
    start_y: float,
    start_heading: float,
    dt: float,
    duration: float,
    seed: int,
    gps_noise: float,
    arrival_time: float,
    k_heading: float,
    k_crosstrack: float,
) -> Optional[TestResult]:
    """Run a single HITL test with given parameters."""
    import serial
    from serial.tools import list_ports

    # Temporarily patch rover.cpp parameters if needed
    # For now, we rely on the pre-patched firmware

    try:
        with serial.Serial(port, 115200, timeout=0.1) as ser:
            time.sleep(2.0)
            ser.reset_input_buffer()

            # Send SIM_START
            cmd = f"SIM_START,{target_x:.3f},{target_y:.3f}\n"
            ser.write(cmd.encode())
            ser.flush()

            # Wait for SIM_OK
            deadline = time.monotonic() + 5.0
            sim_ok = False
            while time.monotonic() < deadline:
                line = ser.readline()
                if line:
                    if b"SIM_OK" in line:
                        sim_ok = True
                        break

            if not sim_ok:
                print("  ESP32 did not respond with SIM_OK")
                return None

            robot_x, robot_y = start_x, start_y
            robot_heading = start_heading
            imu_drift = 0.0
            left_cmd, right_cmd = 0, 0
            arrived = False
            miss_history = []
            stuck_count = 0

            next_tick = time.monotonic()
            steps = int(duration / dt)

            for step in range(steps):
                now = time.monotonic()
                if now < next_tick:
                    time.sleep(max(0, next_tick - now))
                next_tick += dt

                # Simple physics (matching TrackPhysics)
                delay_ticks = max(1, round(0.10 / dt))
                if step == 0:
                    delay_buf = [(0, 0)] * delay_ticks
                else:
                    delay_buf = delay_buf[-delay_ticks+1:] + [(left_cmd, right_cmd)]
                l_cmd, r_cmd = delay_buf[0] if delay_buf else (0, 0)

                # Motor inertia
                inertia = 0.3
                l_actual = l_actual * (1-inertia) + l_cmd * inertia if step > 0 else l_cmd
                r_actual = r_actual * (1-inertia) + r_cmd * inertia if step > 0 else r_cmd

                # Velocity
                l_vel = (l_actual / 70.0) * 0.5
                r_vel = (r_actual / 70.0) * 0.5
                velocity = (l_vel + r_vel) / 2
                omega = (l_vel - r_vel) / 0.38

                # Update position
                heading_rad = math.radians(robot_heading)
                robot_x += velocity * math.sin(heading_rad) * dt
                robot_y += velocity * math.cos(heading_rad) * dt
                robot_heading = (robot_heading + math.degrees(omega * dt)) % 360
                if robot_heading < 0:
                    robot_heading += 360

                # GPS with noise
                gps_x = robot_x + random.uniform(-gps_noise, gps_noise)
                gps_y = robot_y + random.uniform(-gps_noise, gps_noise)
                gps_heading = (robot_heading + random.uniform(-0.25, 0.25)) % 360

                # Send GPS
                gps_cmd = f"GPS,{gps_x:.4f},{gps_y:.4f},{gps_heading:.2f},{abs(velocity):.4f},FIXED\n"
                ser.write(gps_cmd.encode())
                ser.flush()

                # Read MOTORS response
                deadline2 = time.monotonic() + max(dt, 0.05)
                motor_l, motor_r = None, None
                while time.monotonic() < deadline2:
                    line = ser.readline()
                    if line and b"MOTORS," in line:
                        parts = line.decode().strip().split(",")
                        if len(parts) >= 3:
                            try:
                                motor_l = int(parts[1])
                                motor_r = int(parts[2])
                            except ValueError:
                                pass
                        if motor_l is not None:
                            break

                if motor_l is not None:
                    left_cmd = motor_l
                    right_cmd = motor_r

                # Check arrival
                miss = math.hypot(target_x - robot_x, target_y - robot_y)
                miss_history.append(miss)

                if step % 10 == 0:
                    print(f"  t={step*dt:.1f}s pos=({robot_x:.3f},{robot_y:.3f}) miss={miss:.3f}m")

                # Check if stopped
                if abs(left_cmd) < 3 and abs(right_cmd) < 3 and miss < 0.15:
                    arrived = True
                    break

            # Get final miss
            final_miss = miss_history[-1] if miss_history else 1.0

            return TestResult(
                params={
                    "arrival_time": arrival_time,
                    "k_heading": k_heading,
                    "k_crosstrack": k_crosstrack,
                },
                miss_m=final_miss,
                time_s=len(miss_history) * dt,
                stuck_count=stuck_count,
                success=arrived,
            )

    except Exception as e:
        print(f"  Error: {e}")
        return None


def main():
    parser = argparse.ArgumentParser(description="HITL Parameter Tuning")
    parser.add_argument("--port", default="COM4", help="Serial port")
    parser.add_argument("--trials", type=int, default=5, help="Trials per config")
    parser.add_argument("--duration", type=float, default=30.0, help="Max duration per test")
    args = parser.parse_args()

    print("=" * 60)
    print("HITL PARAMETER TUNING")
    print("=" * 60)

    # Test configurations to try
    configs = [
        # ARRIVAL_TIME tests
        {"arrival_time": 0.3, "k_heading": 0.5, "k_crosstrack": 15.0},
        {"arrival_time": 0.5, "k_heading": 0.5, "k_crosstrack": 15.0},
        {"arrival_time": 0.7, "k_heading": 0.5, "k_crosstrack": 15.0},
        {"arrival_time": 1.0, "k_heading": 0.5, "k_crosstrack": 15.0},
        # K_HEADING tests
        {"arrival_time": 0.5, "k_heading": 0.3, "k_crosstrack": 15.0},
        {"arrival_time": 0.5, "k_heading": 0.7, "k_crosstrack": 15.0},
        {"arrival_time": 0.5, "k_heading": 1.0, "k_crosstrack": 15.0},
        # K_CROSSTRACK tests
        {"arrival_time": 0.5, "k_heading": 0.5, "k_crosstrack": 10.0},
        {"arrival_time": 0.5, "k_heading": 0.5, "k_crosstrack": 20.0},
        {"arrival_time": 0.5, "k_heading": 0.5, "k_crosstrack": 25.0},
    ]

    results = []

    for cfg in configs:
        print(f"\nTesting: {cfg}")
        cfg_results = []

        for trial in range(args.trials):
            print(f"  Trial {trial + 1}/{args.trials}...")
            seed = trial + 1

            # Random start position
            start_x = random.uniform(-2.0, 2.0)
            start_y = random.uniform(-2.0, 2.0)
            start_heading = random.uniform(0, 360)

            result = run_single_test(
                port=args.port,
                target_x=5.0,
                target_y=0.0,
                start_x=start_x,
                start_y=start_y,
                start_heading=start_heading,
                dt=0.05,
                duration=args.duration,
                seed=seed,
                gps_noise=0.015,
                arrival_time=cfg["arrival_time"],
                k_heading=cfg["k_heading"],
                k_crosstrack=cfg["k_crosstrack"],
            )

            if result:
                cfg_results.append(result)
                print(f"    Result: miss={result.miss_m*100:.1f}cm, time={result.time_s:.1f}s")

        if cfg_results:
            avg_miss = sum(r.miss_m for r in cfg_results) / len(cfg_results)
            avg_time = sum(r.time_s for r in cfg_results) / len(cfg_results)
            print(f"  Average: miss={avg_miss*100:.1f}cm, time={avg_time:.1f}s")

            results.append({
                "config": cfg,
                "avg_miss_m": avg_miss,
                "avg_time_s": avg_time,
                "success_rate": sum(1 for r in cfg_results if r.success) / len(cfg_results),
            })

    # Find best configuration
    if results:
        best = min(results, key=lambda x: x["avg_miss_m"])
        print("\n" + "=" * 60)
        print("BEST CONFIGURATION:")
        print(f"  ARRIVAL_TIME: {best['config']['arrival_time']}")
        print(f"  K_HEADING: {best['config']['k_heading']}")
        print(f"  K_CROSSTRACK: {best['config']['k_crosstrack']}")
        print(f"  Average Miss: {best['avg_miss_m']*100:.1f}cm")
        print(f"  Average Time: {best['avg_time_s']:.1f}s")
        print("=" * 60)

        # Save results
        output_file = Path(__file__).parent / "tuning_results.json"
        with open(output_file, "w") as f:
            json.dump(results, f, indent=2)
        print(f"\nResults saved to: {output_file}")


if __name__ == "__main__":
    main()
