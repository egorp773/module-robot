#!/usr/bin/env python3
"""
Navigation Debugger - Step-by-step analysis of ESP32 navigation.
Shows all telemetry values and highlights anomalies.
"""

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass, asdict
from typing import List, Optional, Tuple
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent))

from simulator import SitlSimulator, Telemetry, NavState


@dataclass
class DebugFrame:
    """Single frame of debug data"""
    step: int
    time: float
    x: float
    y: float
    heading: float
    speed: float
    motor_left: int
    motor_right: int
    state: str
    heading_error: float
    cross_track: float
    dist_to_end: float
    wp_index: int
    wp_count: int
    fix_quality: int
    anomaly: str = ""


class NavDebugger:
    """
    Debug navigation step-by-step.
    Shows all values and highlights problems.
    """

    def __init__(self, speed: float = 1.0):
        self.speed = speed
        self.frames: List[DebugFrame] = []
        self.current_frame = 0
        self.running = True
        self.paused = True  # Start paused for step-by-step

    def run(self, target: Tuple[float, float], max_steps: int = 1000):
        """Run debug session"""

        sim = SitlSimulator(add_gps_noise=True)
        sim.set_waypoints([target])

        print("\n" + "=" * 80)
        print("NAVIGATION DEBUGGER")
        print("=" * 80)
        print(f"Target: ({target[0]:.1f}, {target[1]:.1f})")
        print(f"Speed multiplier: {self.speed}x")
        print()
        print("Controls:")
        print("  Enter    - Next step")
        print("  Space    - Auto-run / Pause")
        print("  R        - Reset")
        print("  S        - Skip to end")
        print("  Q        - Quit")
        print("=" * 80)

        self.frames = []
        self.current_frame = 0
        auto_run = False

        try:
            for step in range(max_steps):
                # Handle input
                if not auto_run:
                    key = input("\n> ").strip()
                else:
                    key = ""

                if key == 'q':
                    break
                elif key == 'r':
                    # Reset
                    sim = SitlSimulator(add_gps_noise=True)
                    sim.set_waypoints([target])
                    self.frames = []
                    self.current_frame = 0
                    print("\n[RESET]")
                    continue
                elif key == 's':
                    # Skip to end
                    while sim.state.running and sim.state.time < 60:
                        sim.step()
                        if sim.telemetry.state in (NavState.ARRIVED, NavState.ERROR):
                            break
                    auto_run = False
                    key = ""
                elif key == ' ':
                    auto_run = not auto_run
                    print(f"\n[{'AUTO-RUN' if auto_run else 'PAUSED'}]")
                    if not auto_run:
                        continue
                elif key == '':
                    pass  # Next step
                else:
                    continue

                # Execute step
                sim.step()

                # Create debug frame
                frame = DebugFrame(
                    step=step,
                    time=sim.state.time,
                    x=sim.physics.x,
                    y=sim.physics.y,
                    heading=sim.physics.heading,
                    speed=sim.physics.vel_n,
                    motor_left=sim.last_motor_left,
                    motor_right=sim.last_motor_right,
                    state=sim.telemetry.state.value,
                    heading_error=sim.telemetry.heading_error,
                    cross_track=sim.telemetry.cross_track,
                    dist_to_end=sim.telemetry.dist_to_end,
                    wp_index=sim.telemetry.wp_index,
                    wp_count=sim.telemetry.wp_count,
                    fix_quality=4  # Simulated RTK Fixed
                )

                # Detect anomalies
                frame.anomaly = self.detect_anomaly(frame, self.frames[-1] if self.frames else None)
                self.frames.append(frame)

                # Print frame
                self.print_frame(frame)

                # Check completion
                if sim.telemetry.state == NavState.ARRIVED:
                    print("\n✓ ARRIVED!")
                    break
                elif sim.telemetry.state == NavState.ERROR:
                    print("\n✗ ERROR!")
                    break

                # Auto-run delay
                if auto_run:
                    time.sleep(0.1 / self.speed)

        except KeyboardInterrupt:
            print("\n[INTERRUPTED]")

        # Print summary
        self.print_summary()

        return self.frames

    def detect_anomaly(self, frame: DebugFrame, prev: Optional[DebugFrame]) -> str:
        """Detect navigation anomalies"""

        anomalies = []

        # Large heading error
        if abs(frame.heading_error) > 60:
            anomalies.append("LARGE_HEADING_ERR")

        # Oscillation (heading error changing sign rapidly)
        if prev:
            if abs(prev.heading_error) > 20 and abs(frame.heading_error) > 20:
                if prev.heading_error * frame.heading_error < 0:
                    anomalies.append("OSCILLATION")

        # Saturation (motors at max)
        if abs(frame.motor_left) >= 70 or abs(frame.motor_right) >= 70:
            anomalies.append("MOTOR_SATURATED")

        # Cross-track error growing
        if prev:
            if abs(frame.cross_track) > abs(prev.cross_track) + 0.1:
                if abs(frame.cross_track) > 0.3:
                    anomalies.append("CROSSTRACK_GROWING")

        # Speed vs command mismatch
        if prev and abs(prev.speed - frame.speed) > 0.2:
            anomalies.append("SPEED_JUMP")

        # No progress
        if prev:
            dist_change = abs(prev.dist_to_end - frame.dist_to_end)
            if dist_change < 0.01 and frame.speed > 0.1:
                anomalies.append("NO_PROGRESS")

        return " | ".join(anomalies) if anomalies else ""

    def print_frame(self, frame: DebugFrame):
        """Print formatted debug frame"""

        # Anomaly highlight
        if frame.anomaly:
            anomaly_str = f"⚠️  {frame.anomaly}"
        else:
            anomaly_str = ""

        print(f"""
┌──────────────────────────────────────────────────────────────────────────┐
│ STEP {frame.step:4d} │ TIME {frame.time:6.2f}s │ STATE {frame.state:<12} │ {anomaly_str}
├──────────────────────────────────────────────────────────────────────────┤
│ Position: ({frame.x:7.2f}, {frame.y:7.2f}) m                              │
│ Heading: {frame.heading:6.1f}° │ Speed: {frame.speed:5.2f} m/s                       │
├──────────────────────────────────────────────────────────────────────────┤
│ Heading Error: {frame.heading_error:+7.1f}° │ Cross Track: {frame.cross_track:+7.2f} m         │
│ Distance to End: {frame.dist_to_end:6.2f} m │ Waypoint {frame.wp_index}/{frame.wp_count}        │
├──────────────────────────────────────────────────────────────────────────┤
│ Motor: L={frame.motor_left:+3d}% ({frame.motor_left/70*100:+.0f}% max) │ R={frame.motor_right:+3d}% ({frame.motor_right/70*100:+.0f}% max) │
└──────────────────────────────────────────────────────────────────────────┘""")

    def print_summary(self):
        """Print debug summary"""

        if not self.frames:
            return

        print("\n" + "=" * 80)
        print("DEBUG SUMMARY")
        print("=" * 80)

        # Basic stats
        errors = [abs(f.heading_error) for f in self.frames]
        cross_tracks = [abs(f.cross_track) for f in self.frames]

        print(f"Total steps: {len(self.frames)}")
        print(f"Final position: ({self.frames[-1].x:.3f}, {self.frames[-1].y:.3f})")
        print(f"Final heading: {self.frames[-1].heading:.1f}°")

        print(f"\nHeading Error:")
        print(f"  Max: {max(errors):.1f}°")
        print(f"  Mean: {sum(errors)/len(errors):.1f}°")

        print(f"\nCross Track:")
        print(f"  Max: {max(cross_tracks):.3f} m")
        print(f"  Mean: {sum(cross_tracks)/len(cross_tracks):.3f} m")

        # Anomalies
        anomalies = [f for f in self.frames if f.anomaly]
        if anomalies:
            print(f"\n⚠️  ANOMALIES DETECTED ({len(anomalies)}):")
            for f in anomalies[:10]:  # Show first 10
                print(f"  Step {f.step}: {f.anomaly}")
            if len(anomalies) > 10:
                print(f"  ... and {len(anomalies) - 10} more")

        # Direction changes (oscillations)
        direction_changes = 0
        for i in range(1, len(self.frames)):
            curr_err = self.frames[i].heading_error
            prev_err = self.frames[i-1].heading_error
            if prev_err * curr_err < 0 and abs(prev_err) > 10 and abs(curr_err) > 10:
                direction_changes += 1

        print(f"\nOscillations (direction changes >10°): {direction_changes}")

        print("=" * 80)


def replay_log(filename: str):
    """Replay a saved debug log"""

    with open(filename, 'r') as f:
        data = json.load(f)

    frames = [DebugFrame(**f) for f in data['frames']]

    print(f"\nReplaying log: {filename}")
    print(f"Total frames: {len(frames)}")

    for i, frame in enumerate(frames):
        cmd = input(f"\nFrame {i+1}/{len(frames)}: ")
        if cmd == 'q':
            break

        debugger = NavDebugger()
        debugger.print_frame(frame)


def main():
    parser = argparse.ArgumentParser(description='Navigation Debugger')
    parser.add_argument('--target', type=str, default='5,0',
                       help='Target coordinates: x,y')
    parser.add_argument('--speed', type=float, default=1.0,
                       help='Playback speed multiplier')
    parser.add_argument('--steps', type=int, default=500,
                       help='Max steps to run')
    parser.add_argument('--save', type=str,
                       help='Save debug log to file')
    parser.add_argument('--load', type=str,
                       help='Load and replay debug log')

    args = parser.parse_args()

    # Parse target
    try:
        tx, ty = [float(x) for x in args.target.split(',')]
    except:
        print(f"Invalid target: {args.target}")
        return 1

    if args.load:
        replay_log(args.load)
        return 0

    # Run debugger
    debugger = NavDebugger(speed=args.speed)
    frames = debugger.run((tx, ty), args.steps)

    # Save if requested
    if args.save and frames:
        data = {
            'target': (tx, ty),
            'frames': [asdict(f) for f in frames]
        }
        with open(args.save, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"\nSaved debug log to {args.save}")

    return 0


if __name__ == '__main__':
    sys.exit(main())
