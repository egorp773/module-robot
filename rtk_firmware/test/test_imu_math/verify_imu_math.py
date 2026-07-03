#!/usr/bin/env python3
"""Mirrors rtk_firmware/test/test_imu_math/test_imu_math.cpp in Python so the
heading-correction math can be verified on any host without a native PIO
toolchain. Returns 0 on success, non-zero on first mismatch.
"""

import math


def normalize_deg360(d):
    d = d % 360.0
    if d < 0.0:
        d += 360.0
    return d


def wrap_deg180(d):
    d = normalize_deg360(d)
    if d > 180.0:
        d -= 360.0
    return d


def imu_raw_to_robot_heading(raw, sign, mount_off, compass_adj):
    raw = normalize_deg360(raw)
    return normalize_deg360(sign * raw + mount_off + compass_adj)


def heading_correction(current, true_h):
    return wrap_deg180(true_h - current)


def apply_heading_correction(heading, correction):
    return normalize_deg360(heading + correction)


def rtk_forward_heading(dx_east, dy_north):
    return normalize_deg360(math.atan2(dx_east, dy_north) * 180.0 / math.pi)


def expect_near(label, actual, expected, tol=1e-4):
    diff = abs(actual - expected)
    ok = diff <= tol
    flag = "OK " if ok else "FAIL"
    print(f"[{flag}] {label}: actual={actual:.6f} expected={expected:.6f} diff={diff:.6f}")
    return ok


def main():
    failures = 0

    cases = [
        ("normalize -1",     normalize_deg360(-1.0),   359.0),
        ("normalize 361",    normalize_deg360(361.0),  1.0),
        ("wrap 181",         wrap_deg180(181.0),     -179.0),
        ("wrap -181",        wrap_deg180(-181.0),     179.0),

        ("raw 0 sign +1",    imu_raw_to_robot_heading(0.0,   1.0, 0.0, 0.0), 0.0),
        ("raw 90 sign +1",   imu_raw_to_robot_heading(90.0,  1.0, 0.0, 0.0), 90.0),
        ("raw 90 sign -1",   imu_raw_to_robot_heading(90.0, -1.0, 0.0, 0.0), 270.0),
        ("raw 0 sign -1 mount 90", imu_raw_to_robot_heading(0.0, -1.0, 90.0, 0.0), 90.0),
        ("raw 359 sign +1",  imu_raw_to_robot_heading(359.0, 1.0, 0.0, 0.0), 359.0),
        ("raw 361 sign +1",  imu_raw_to_robot_heading(361.0, 1.0, 0.0, 0.0), 1.0),

        ("corr 90->260",     heading_correction(90.0,  260.0),  170.0),
        ("corr 350->10",     heading_correction(350.0, 10.0),    20.0),
        ("corr 10->350",     heading_correction(10.0,  350.0),  -20.0),

        ("apply 90 +170",    apply_heading_correction(90.0,  170.0), 260.0),
        ("apply 350 +20",    apply_heading_correction(350.0, 20.0),  10.0),
        ("apply 10 -20",     apply_heading_correction(10.0, -20.0),  350.0),

        ("rtk +X",           rtk_forward_heading( 1.0,  0.0),  90.0),
        ("rtk +Y",           rtk_forward_heading( 0.0,  1.0),   0.0),
        ("rtk -X",           rtk_forward_heading(-1.0,  0.0), 270.0),
        ("rtk -Y",           rtk_forward_heading( 0.0, -1.0), 180.0),
    ]

    for label, actual, expected in cases:
        if not expect_near(label, actual, expected):
            failures += 1

    # Realistic walk-through
    current = 90.0
    true_h = 260.0
    delta = heading_correction(current, true_h)
    new_corr = wrap_deg180(0.0 + delta)
    final_h = apply_heading_correction(90.0, new_corr)
    print()
    print("Scenario walk-through:")
    print(f"  current={current}  true={true_h}  delta={delta}")
    print(f"  new_corr={new_corr}  final={final_h:.4f}")
    if abs(final_h - 260.0) > 1e-3:
        print("  FAIL: scenario final not 260")
        failures += 1
    else:
        print("  OK: scenario final is 260")

    if failures:
        print(f"\nFAILED: {failures} assertion(s) failed")
        return 1
    print("\nAll ImuMath heading + correction assertions passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
