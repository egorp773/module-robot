#!/usr/bin/env python3
"""
HITL Parameter Optimization Script
Finds optimal K_HEADING and K_CROSSTRACK for <2cm accuracy
"""
import subprocess
import re
import time
import csv
from pathlib import Path
from itertools import product


def run_test(port, target, start, k_heading, k_crosstrack, imu_drift=0.0, seed=1):
    """Run single HITL test with given parameters."""
    cmd = [
        'python', 'hitl_runner.py',
        '--port', port,
        '--target', f'{target[0]:.1f}', f'{target[1]:.1f}',
        '--start', f'{start[0]:.1f}', f'{start[1]:.1f}', f'{start[2]:.0f}',
        '--duration', '50',
        '--dt', '0.05',
        '--imu-drift-deg-s', str(imu_drift),
        '--no-plot'
    ]

    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
        for line in result.stdout.split('\n'):
            if 'Final:' in line:
                m = re.search(r'miss=([0-9.]+)', line)
                if m:
                    return float(m.group(1))
    except Exception as e:
        print(f"Error: {e}")
    return None


def run_param_sweep(port, k_headings, k_crosstracks, trials=10):
    """Sweep K_HEADING x K_CROSSTRACK matrix."""
    print("\n" + "="*70)
    print("PARAMETER SWEEP: K_HEADING x K_CROSSTRACK")
    print("="*70)

    results = []
    total = len(k_headings) * len(k_crosstracks) * trials

    print(f"\nRunning {total} tests...")
    print("-"*70)

    for kh in k_headings:
        for kx in k_crosstracks:
            misses = []
            for trial in range(trials):
                # Random start position
                import random
                random.seed(trial * 1000 + int(kh * 100) + int(kx))
                dist = random.uniform(3.0, 5.0)
                sx = dist * random.uniform(-0.8, 0.8)
                sy = dist * random.uniform(-0.5, 0.5)
                sh = random.uniform(0, 360)

                # Set parameters via SIM_SETPARAMS
                subprocess.run([
                    'python', '-c',
                    f'''
import serial
import time
with serial.Serial("{port}", 115200, timeout=0.5) as s:
    time.sleep(1)
    s.write(b"SIM_START,5.0,0.0\\n")
    time.sleep(0.3)
    s.write(f"SIM_SETPARAMS,{kh},{kx}\\n".encode())
    s.write(b"SIM_STOP\\n")
'''
                ], capture_output=True)

                miss = run_test(port, (5.0, 0.0), (sx, sy, sh), kh, kx, seed=trial)
                if miss is not None:
                    misses.append(miss)

                idx = k_headings.index(kh) * len(k_crosstracks) + k_crosstracks.index(kx) * trials + trial
                print(f"\rProgress: {idx+1}/{total} ({100*(idx+1)//total}%)", end='', flush=True)

            avg_miss = sum(misses) / len(misses) if misses else 999
            results.append({
                'k_heading': kh,
                'k_crosstrack': kx,
                'avg_miss_cm': avg_miss * 100,
                'min_miss_cm': min(misses) * 100 if misses else 999,
                'max_miss_cm': max(misses) * 100 if misses else 999,
                'trials': len(misses)
            })
            print(f"  KH={kh:.1f} KX={kx:.0f} -> avg={avg_miss*100:.1f}cm (n={len(misses)})")

    return results


def test_heading_angles(port, angles):
    """Test how different starting angles affect accuracy."""
    print("\n" + "="*70)
    print("HEADING ANGLE TEST")
    print("="*70)

    results = []
    for angle in angles:
        misses = []
        for trial in range(10):
            import random
            random.seed(trial + angle)
            dist = random.uniform(3.0, 5.0)
            sx = dist * random.uniform(-0.5, 0.5)
            sy = dist * random.uniform(-0.3, 0.3)

            miss = run_test(port, (5.0, 0.0), (sx, sy, angle), 0.5, 15.0, seed=trial)
            if miss is not None:
                misses.append(miss)

        avg_miss = sum(misses) / len(misses) if misses else 999
        results.append({
            'angle': angle,
            'avg_miss_cm': avg_miss * 100,
            'max_miss_cm': max(misses) * 100 if misses else 999,
            'trials': len(misses)
        })
        print(f"Angle {angle:3d}° -> avg={avg_miss*100:.1f}cm max={max(misses)*100:.1f if misses else 999:.1f}cm")

    return results


def test_imu_drift(port, drifts):
    """Test how IMU drift affects accuracy."""
    print("\n" + "="*70)
    print("IMU DRIFT TEST")
    print("="*70)

    results = []
    for drift in drifts:
        misses = []
        for trial in range(10):
            import random
            random.seed(trial)
            dist = random.uniform(3.0, 5.0)
            sx = dist * random.uniform(-0.5, 0.5)
            sy = dist * random.uniform(-0.3, 0.3)
            sh = random.uniform(0, 360)

            miss = run_test(port, (5.0, 0.0), (sx, sy, sh), 0.5, 15.0, imu_drift=drift, seed=trial)
            if miss is not None:
                misses.append(miss)

        avg_miss = sum(misses) / len(misses) if misses else 999
        results.append({
            'imu_drift': drift,
            'avg_miss_cm': avg_miss * 100,
            'max_miss_cm': max(misses) * 100 if misses else 999,
            'trials': len(misses)
        })
        print(f"Drift {drift:.2f}°/s -> avg={avg_miss*100:.1f}cm max={max(misses)*100:.1f if misses else 999:.1f}cm")

    return results


def main():
    port = "COM4"

    print("="*70)
    print("HITL PARAMETER OPTIMIZATION FOR <2cm ACCURACY")
    print("="*70)

    # 1. Parameter sweep
    k_headings = [0.3, 0.5, 0.7, 1.0]
    k_crosstracks = [10, 15, 20, 25]

    results = run_param_sweep(port, k_headings, k_crosstracks, trials=10)

    # Find best
    best = min(results, key=lambda x: x['avg_miss_cm'])
    print("\n" + "="*70)
    print("BEST CONFIGURATION:")
    print(f"  K_HEADING:    {best['k_heading']:.1f}")
    print(f"  K_CROSSTRACK: {best['k_crosstrack']:.0f}")
    print(f"  Average miss: {best['avg_miss_cm']:.1f}cm")
    print(f"  Min miss:     {best['min_miss_cm']:.1f}cm")
    print(f"  Max miss:     {best['max_miss_cm']:.1f}cm")
    print("="*70)

    # 2. Heading angle test with best params
    heading_results = test_heading_angles(port, [0, 45, 90, 135, 180, 225, 270, 315])
    worst_angle = max(heading_results, key=lambda x: x['avg_miss_cm'])
    print(f"\nWorst angle: {worst_angle['angle']}° ({worst_angle['avg_miss_cm']:.1f}cm)")

    # 3. IMU drift test
    drift_results = test_imu_drift(port, [0.0, 0.05, 0.1, 0.2, 0.5])

    # Save results
    output_dir = Path(__file__).parent.parent / ".pio" / "build_root"
    output_dir.mkdir(parents=True, exist_ok=True)

    with open(output_dir / "param_sweep.csv", 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['k_heading', 'k_crosstrack', 'avg_miss_cm', 'min_miss_cm', 'max_miss_cm', 'trials'])
        writer.writeheader()
        writer.writerows(results)

    with open(output_dir / "heading_test.csv", 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['angle', 'avg_miss_cm', 'max_miss_cm', 'trials'])
        writer.writeheader()
        writer.writerows(heading_results)

    with open(output_dir / "imu_drift_test.csv", 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['imu_drift', 'avg_miss_cm', 'max_miss_cm', 'trials'])
        writer.writeheader()
        writer.writerows(drift_results)

    print(f"\nResults saved to {output_dir}")


if __name__ == "__main__":
    main()
