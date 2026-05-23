#!/usr/bin/env python3
"""
Automated test suite for RTK robot navigation.
Runs multiple simulations and collects statistics.

Usage:
    python auto_test.py --count 100
    python auto_test.py --count 100 --compare
    python auto_test.py --count 50 --json results.json
    python auto_test.py --scenario square --count 20
"""

import argparse
import json
import math
import random
import sys
import time
from dataclasses import dataclass, asdict
from typing import List, Tuple, Optional
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent))

from simulator import SitlSimulator, run_headless, NavState, MAX_PWM


@dataclass
class TestResult:
    """Result of a single test run"""
    run_id: int
    scenario: str
    start_x: float
    start_y: float
    target_x: float
    target_y: float
    final_x: float
    final_y: float
    error_m: float
    success: bool
    time_s: float
    oscillation_count: int
    max_heading_error: float
    max_cross_track: float
    final_state: str


@dataclass
class TestSummary:
    """Summary statistics across all test runs"""
    count: int
    success_count: int
    success_rate: float
    avg_error_m: float
    median_error_m: float
    max_error_m: float
    std_error_m: float
    avg_time_s: float
    avg_oscillations: float
    within_10cm: float
    within_20cm: float


class AutoTester:
    """
    Automated test runner for navigation testing.
    Tests real ESP32 firmware in HITL mode or simulated navigation in SITL mode.
    """

    def __init__(self, mode: str = 'sitl', add_gps_noise: bool = True):
        self.mode = mode
        self.add_gps_noise = add_gps_noise
        self.results: List[TestResult] = []

    def run_single_test(self, run_id: int, scenario: str,
                       start: Tuple[float, float],
                       target: Tuple[float, float],
                       max_time: float = 60.0) -> TestResult:
        """Run a single navigation test"""

        # Create simulator
        if self.mode == 'sitl':
            sim = SitlSimulator(add_gps_noise=self.add_gps_noise)
        else:
            from simulator import HitlSimulator
            sim = HitlSimulator(add_gps_noise=self.add_gps_noise)

        # Set target
        sim.set_waypoints([target])
        if hasattr(sim, 'set_target'):
            sim.set_target(target[0], target[1])

        # Track state during run
        max_heading_err = 0.0
        max_cross_track = 0.0
        oscillation_count = 0
        prev_heading_error = None
        trajectory = []

        # Run simulation
        start_time = time.time()

        while sim.state.running and sim.state.time < max_time:
            sim.step()

            # Track metrics
            tel = sim.telemetry
            err = abs(tel.heading_error)
            if err > max_heading_err:
                max_heading_err = err

            ct = abs(tel.cross_track)
            if ct > max_cross_track:
                max_cross_track = ct

            # Detect oscillations
            if prev_heading_error is not None and err < prev_heading_error + 30:
                if prev_heading_error > 30 and err < prev_heading_error - 20:
                    oscillation_count += 1
            prev_heading_error = err

            trajectory.append((sim.physics.x, sim.physics.y))

            # Check completion
            if tel.state in (NavState.ARRIVED, NavState.ERROR):
                break

        elapsed = time.time() - start_time

        # Calculate final error
        final_x, final_y = sim.physics.x, sim.physics.y
        error = math.sqrt((final_x - target[0])**2 + (final_y - target[1])**2)

        # Reduce oscillation count (overcounted)
        oscillation_count = max(0, oscillation_count // 3)

        return TestResult(
            run_id=run_id,
            scenario=scenario,
            start_x=start[0],
            start_y=start[1],
            target_x=target[0],
            target_y=target[1],
            final_x=final_x,
            final_y=final_y,
            error_m=error,
            success=error < 0.1,  # 10cm threshold
            time_s=elapsed,
            oscillation_count=oscillation_count,
            max_heading_error=max_heading_err,
            max_cross_track=max_cross_track,
            final_state=sim.telemetry.state.value
        )

    def run_scenario(self, scenario: str, count: int = 10) -> List[TestResult]:
        """Run a specific test scenario multiple times"""

        results = []

        if scenario == 'straight':
            # Go straight ahead 5m
            for i in range(count):
                result = self.run_single_test(
                    i, scenario,
                    start=(0.0, 0.0),
                    target=(5.0, 0.0)
                )
                results.append(result)

        elif scenario == 'angle':
            # Go at 45 degree angle
            for i in range(count):
                result = self.run_single_test(
                    i, scenario,
                    start=(0.0, 0.0),
                    target=(3.0, 3.0)
                )
                results.append(result)

        elif scenario == 'random':
            # Random targets
            random.seed(42 + i)  # Reproducible
            for i in range(count):
                angle = random.uniform(0, 2 * math.pi)
                dist = random.uniform(3, 10)
                target = (dist * math.cos(angle), dist * math.sin(angle))
                result = self.run_single_test(
                    i, scenario,
                    start=(0.0, 0.0),
                    target=target
                )
                results.append(result)

        elif scenario == 'multi_wp':
            # Multiple waypoints (square pattern)
            for i in range(count):
                if self.mode == 'sitl':
                    sim = SitlSimulator(add_gps_noise=self.add_gps_noise)
                else:
                    from simulator import HitlSimulator
                    sim = HitlSimulator(add_gps_noise=self.add_gps_noise)

                # Square path: (0,0) -> (5,0) -> (5,5) -> (0,5) -> (0,0)
                waypoints = [(5.0, 0.0), (5.0, 5.0), (0.0, 5.0), (0.0, 0.0)]
                sim.set_waypoints(waypoints)

                max_time = 120.0
                start_time = time.time()
                max_heading_err = 0.0
                max_cross_track = 0.0

                while sim.state.running and sim.state.time < max_time:
                    sim.step()
                    tel = sim.telemetry
                    if abs(tel.heading_error) > max_heading_err:
                        max_heading_err = abs(tel.heading_error)
                    if abs(tel.cross_track) > max_cross_track:
                        max_cross_track = abs(tel.cross_track)
                    if tel.state == NavState.ARRIVED:
                        break

                elapsed = time.time() - start_time
                error = math.sqrt(sim.physics.x**2 + sim.physics.y**2)

                results.append(TestResult(
                    run_id=i,
                    scenario=scenario,
                    start_x=0.0, start_y=0.0,
                    target_x=0.0, target_y=0.0,
                    final_x=sim.physics.x,
                    final_y=sim.physics.y,
                    error_m=error,
                    success=error < 0.2,  # 20cm for multi-WP
                    time_s=elapsed,
                    oscillation_count=0,
                    max_heading_error=max_heading_err,
                    max_cross_track=max_cross_track,
                    final_state=sim.telemetry.state.value
                ))

        else:
            print(f"Unknown scenario: {scenario}")
            return results

        self.results.extend(results)
        return results

    def analyze(self) -> TestSummary:
        """Calculate summary statistics"""
        if not self.results:
            return TestSummary(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

        errors = sorted([r.error_m for r in self.results])
        success_count = sum(1 for r in self.results if r.success)
        oscillation_counts = [r.oscillation_count for r in self.results]

        n = len(self.results)
        avg_error = sum(errors) / n
        median_error = errors[n // 2]

        variance = sum((e - avg_error) ** 2 for e in errors) / n
        std_error = math.sqrt(variance)

        return TestSummary(
            count=n,
            success_count=success_count,
            success_rate=success_count / n * 100,
            avg_error_m=avg_error,
            median_error_m=median_error,
            max_error_m=max(errors),
            std_error_m=std_error,
            avg_time_s=sum(r.time_s for r in self.results) / n,
            avg_oscillations=sum(oscillation_counts) / n,
            within_10cm=sum(1 for e in errors if e <= 0.10) / n * 100,
            within_20cm=sum(1 for e in errors if e <= 0.20) / n * 100
        )

    def save_json(self, filename: str):
        """Save results to JSON file"""
        data = {
            'summary': asdict(self.analyze()),
            'results': [asdict(r) for r in self.results]
        }
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"Saved to {filename}")

    def load_json(self, filename: str) -> bool:
        """Load results from JSON file"""
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            self.results = [TestResult(**r) for r in data['results']]
            return True
        except Exception as e:
            print(f"Load error: {e}")
            return False

    def print_summary(self, summary: TestSummary):
        """Print formatted summary"""
        print("\n" + "=" * 60)
        print("TEST RESULTS SUMMARY")
        print("=" * 60)
        print(f"Mode: {self.mode.upper()}")
        print(f"GPS Noise: {'ON' if self.add_gps_noise else 'OFF'}")
        print()
        print(f"Total runs: {summary.count}")
        print(f"Success: {summary.success_count} ({summary.success_rate:.1f}%)")
        print()
        print(f"Position Error:")
        print(f"  Mean:   {summary.avg_error_m * 100:.1f} cm")
        print(f"  Median: {summary.median_error_m * 100:.1f} cm")
        print(f"  Max:    {summary.max_error_m * 100:.1f} cm")
        print(f"  StdDev: {summary.std_error_m * 100:.1f} cm")
        print(f"  Within 10cm: {summary.within_10cm:.1f}%")
        print(f"  Within 20cm: {summary.within_20cm:.1f}%")
        print()
        print(f"Navigation:")
        print(f"  Avg time: {summary.avg_time_s:.1f}s")
        print(f"  Avg oscillations: {summary.avg_oscillations:.1f}")
        print("=" * 60)


def compare_runs(file1: str, file2: str):
    """Compare two test result files"""
    tester1 = AutoTester()
    tester2 = AutoTester()

    if not tester1.load_json(file1) or not tester2.load_json(file2):
        print("Failed to load files")
        return

    s1 = tester1.analyze()
    s2 = tester2.analyze()

    print("\n" + "=" * 60)
    print("COMPARISON")
    print("=" * 60)
    print(f"{'Metric':<25} {'Run 1':>12} {'Run 2':>12} {'Diff':>10}")
    print("-" * 60)
    print(f"{'Success Rate (%)':<25} {s1.success_rate:>12.1f} {s2.success_rate:>12.1f} {s1.success_rate-s2.success_rate:>+10.1f}")
    print(f"{'Mean Error (cm)':<25} {s1.avg_error_m*100:>12.1f} {s2.avg_error_m*100:>12.1f} {(s1.avg_error_m-s2.avg_error_m)*100:>+10.1f}")
    print(f"{'P95 Error (cm)':<25} {s1.max_error_m*100:>12.1f} {s2.max_error_m*100:>12.1f} {(s1.max_error_m-s2.max_error_m)*100:>+10.1f}")
    print(f"{'Within 10cm (%)':<25} {s1.within_10cm:>12.1f} {s2.within_10cm:>12.1f} {s1.within_10cm-s2.within_10cm:>+10.1f}")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(description='RTK Robot Auto Test')
    parser.add_argument('--count', type=int, default=20, help='Number of test runs per scenario')
    parser.add_argument('--scenario', type=str, default='straight',
                       choices=['straight', 'angle', 'random', 'multi_wp', 'all'],
                       help='Test scenario')
    parser.add_argument('--mode', type=str, default='sitl', choices=['sitl', 'hitl'],
                       help='Simulation mode')
    parser.add_argument('--json', type=str, help='Output JSON file')
    parser.add_argument('--load', type=str, help='Load results from JSON')
    parser.add_argument('--compare', type=str, help='Compare with another JSON file')
    parser.add_argument('--no-noise', action='store_true', help='Disable GPS noise')

    args = parser.parse_args()

    # Load existing results
    if args.load:
        tester = AutoTester()
        if tester.load_json(args.load):
            summary = tester.analyze()
            tester.print_summary(summary)
            return 0
        return 1

    # Compare mode
    if args.compare:
        compare_runs(args.compare, args.load or 'results.json')
        return 0

    # Run tests
    tester = AutoTester(
        mode=args.mode,
        add_gps_noise=not args.no_noise
    )

    scenarios = [args.scenario] if args.scenario != 'all' else ['straight', 'angle', 'random']

    for scenario in scenarios:
        print(f"\nRunning scenario: {scenario}")
        print("-" * 40)
        results = tester.run_scenario(scenario, args.count)

        # Print per-scenario summary
        errors = [r.error_m for r in results]
        successes = sum(1 for r in results if r.success)
        print(f"  Completed: {len(results)}/{args.count}")
        print(f"  Success rate: {successes/len(results)*100:.1f}%")
        print(f"  Mean error: {sum(errors)/len(errors)*100:.1f}cm")
        print(f"  Max error: {max(errors)*100:.1f}cm")

    # Overall summary
    summary = tester.analyze()
    tester.print_summary(summary)

    # Save if requested
    if args.json:
        tester.save_json(args.json)
    else:
        tester.save_json('test_results.json')

    return 0


if __name__ == '__main__':
    sys.exit(main())
