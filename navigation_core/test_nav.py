#!/usr/bin/env python3
"""
Navigation Core Test - Python port of C++ tests
Tests the exact same algorithm from rover.cpp
"""

import math
import random

# ============================================================================
# Constants (from rover.cpp)
# ============================================================================

MAX_SPEED = 0.5
FLOAT_SPEED = 0.25
DEGRADED_SPEED = 0.15
HOLD_SPEED = 0.05
MAX_SPEED_PERCENT = 70
ARRIVAL_DIST_M = 0.1
ARRIVAL_CONFIRM_TIME_S = 2.0
ARRIVAL_APPROACH_DIST_M = 0.3
LOOKAHEAD_MIN_M = 0.5
LOOKAHEAD_MAX_M = 1.2
LOOKAHEAD_SPEED_GAIN = 1.5
K_HEADING = 0.5
K_CROSSTRACK = 15.0
TURN_THRESHOLD_1 = 30.0
TURN_THRESHOLD_2 = 60.0
TURN_THRESHOLD_3 = 90.0
TURN_THRESHOLD_4 = 120.0

# Physics
WHEELBASE = 0.38
GPS_NOISE_STD = 0.015  # 15mm for RTK Fixed
IMU_NOISE_STD = 0.5  # 0.5 deg
DT = 0.05


# ============================================================================
# Navigation Core (Python port of NavigationCore.h)
# ============================================================================

class NavState:
    IDLE = 0
    MOVING = 1
    APPROACHING = 2
    ARRIVED = 3
    ERROR = 4


class NavQuality:
    NONE = 0
    DEGRADED = 1
    FLOAT_OK = 2
    FIXED = 3


def normalize_angle(a):
    while a > 180: a -= 360
    while a < -180: a += 360
    return a


class NavigationCore:
    def __init__(self):
        self.state = NavState.IDLE
        self.quality = NavQuality.FIXED
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.heading = 0.0
        self.speed = 0.0

        self.waypoints = []
        self.current_wp_index = 0

        self.forward_scale = 1.0
        self.turn_scale = 1.0
        self.invert_forward = False
        self.invert_steering = False

        # Debug info
        self.debug = {
            'heading_error': 0.0,
            'cross_track_error': 0.0,
            'dist_to_target': 0.0,
            'desired_heading': 0.0,
            'forward_cmd': 0.0,
            'turn_cmd': 0.0,
            'cross_track_correction': 0.0,
        }

    def reset(self):
        self.state = NavState.IDLE
        self.current_wp_index = 0
        self.speed = 0.0

    def clear_route(self):
        self.waypoints = []
        self.current_wp_index = 0

    def add_waypoint(self, x, y):
        self.waypoints.append((x, y))

    def is_route_ready(self):
        return len(self.waypoints) > 0

    def update(self, dt):
        output = {
            'left_cmd': 0,
            'right_cmd': 0,
            'heading_error': 0.0,
            'cross_track_error': 0.0,
            'dist_to_target': 0.0,
            'speed': 0.0,
            'state': self.state,
        }

        if not self.is_route_ready():
            self.state = NavState.IDLE
            output['state'] = self.state
            return output

        if self.current_wp_index >= len(self.waypoints):
            self.state = NavState.ARRIVED
            output['state'] = self.state
            return output

        target = self.waypoints[self.current_wp_index]
        dx = target[0] - self.pos_x
        dy = target[1] - self.pos_y
        dist_to_target = math.sqrt(dx*dx + dy*dy)
        output['dist_to_target'] = dist_to_target
        self.debug['dist_to_target'] = dist_to_target

        # Check arrival
        if dist_to_target < ARRIVAL_DIST_M and self.speed < 0.03:
            self.current_wp_index += 1
            if self.current_wp_index >= len(self.waypoints):
                self.state = NavState.ARRIVED
            else:
                self.state = NavState.MOVING
            output['state'] = self.state
            return output

        # Update state
        if dist_to_target < ARRIVAL_APPROACH_DIST_M:
            self.state = NavState.APPROACHING
        else:
            self.state = NavState.MOVING

        # Pure pursuit lookahead
        lookahead = LOOKAHEAD_MIN_M + self.speed * LOOKAHEAD_SPEED_GAIN
        if lookahead > LOOKAHEAD_MAX_M:
            lookahead = LOOKAHEAD_MAX_M

        lookahead_target = target

        # Look past current waypoint
        if dist_to_target < lookahead and self.current_wp_index + 1 < len(self.waypoints):
            remaining = lookahead - dist_to_target
            a = target
            for seg_idx in range(self.current_wp_index + 1, len(self.waypoints)):
                b = self.waypoints[seg_idx]
                seg_len = math.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2)
                if seg_len <= 0.01:
                    continue
                if remaining <= seg_len:
                    t = remaining / seg_len
                    lookahead_target = (
                        a[0] + (b[0] - a[0]) * t,
                        a[1] + (b[1] - a[1]) * t
                    )
                    break
                remaining -= seg_len
                a = b

        # Desired heading
        dx_look = lookahead_target[0] - self.pos_x
        dy_look = lookahead_target[1] - self.pos_y
        desired_heading = math.degrees(math.atan2(dx_look, dy_look))
        if desired_heading < 0:
            desired_heading += 360
        self.debug['desired_heading'] = desired_heading

        # Heading error
        heading_error = normalize_angle(desired_heading - self.heading)
        self.debug['heading_error'] = heading_error
        output['heading_error'] = heading_error

        # Cross track error
        cross_track_error = 0.0
        if self.current_wp_index > 0:
            a = self.waypoints[self.current_wp_index - 1]
            b = target
            seg_len = math.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2)
            if seg_len > 0.1:
                dx_seg = b[0] - a[0]
                dy_seg = b[1] - a[1]
                cross_track_error = (dx_seg * (self.pos_y - a[1]) - dy_seg * (self.pos_x - a[0])) / seg_len
        self.debug['cross_track_error'] = cross_track_error
        output['cross_track_error'] = cross_track_error

        # Speed selection
        if self.quality == NavQuality.FIXED:
            base_speed = MAX_SPEED
        elif self.quality == NavQuality.FLOAT_OK:
            base_speed = FLOAT_SPEED
        elif self.quality == NavQuality.DEGRADED:
            base_speed = DEGRADED_SPEED
        else:
            base_speed = 0

        # Speed reduction for turns
        if abs(heading_error) > TURN_THRESHOLD_4 and dist_to_target > ARRIVAL_APPROACH_DIST_M:
            base_speed = 0
        elif abs(heading_error) > TURN_THRESHOLD_3:
            base_speed *= 0.3
        elif abs(heading_error) > TURN_THRESHOLD_2:
            base_speed *= 0.5
        elif abs(heading_error) > TURN_THRESHOLD_1:
            base_speed *= 0.7

        # Approach mode
        if self.state == NavState.APPROACHING:
            base_speed *= 0.5
            if dist_to_target < 0.2:
                base_speed *= 0.3

        # Cross track speed reduction
        if abs(cross_track_error) > 0.5:
            base_speed *= 0.8
        if abs(cross_track_error) > 1.0:
            base_speed *= 0.6

        output['speed'] = base_speed

        # Motor commands
        forward_cmd = (base_speed / MAX_SPEED) * MAX_SPEED_PERCENT * self.forward_scale
        turn_cmd = K_HEADING * heading_error * self.turn_scale

        # Cross track correction - THIS IS THE POTENTIAL BUG
        # Original code: turnCmd += K_CROSSTRACK * crossTrackError
        # If crossTrackError > 0 (robot right of line), this makes turnCmd MORE positive
        # But to correct, robot needs to turn LEFT (negative turnCmd)
        cross_track_correction = K_CROSSTRACK * cross_track_error * self.turn_scale
        turn_cmd += cross_track_correction

        self.debug['forward_cmd'] = forward_cmd
        self.debug['turn_cmd'] = turn_cmd
        self.debug['cross_track_correction'] = cross_track_correction

        # Limit turn
        max_turn = MAX_SPEED_PERCENT * 0.6
        turn_cmd = max(-max_turn, min(max_turn, turn_cmd))

        # Inversions
        if self.invert_forward:
            forward_cmd = -forward_cmd
        if self.invert_steering:
            turn_cmd = -turn_cmd

        # Motor output
        left = forward_cmd + turn_cmd
        right = forward_cmd - turn_cmd
        left = max(-MAX_SPEED_PERCENT, min(MAX_SPEED_PERCENT, left))
        right = max(-MAX_SPEED_PERCENT, min(MAX_SPEED_PERCENT, right))

        output['left_cmd'] = int(left)
        output['right_cmd'] = int(right)
        output['state'] = self.state

        return output


# ============================================================================
# Physics Simulation
# ============================================================================

class RobotPhysics:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0  # degrees, 0=North, clockwise positive
        self.speed = 0.0

    def update(self, left_pwm, right_pwm):
        v_left = (left_pwm / 70.0) * MAX_SPEED
        v_right = (right_pwm / 70.0) * MAX_SPEED
        v = (v_left + v_right) / 2.0
        omega = (v_left - v_right) / WHEELBASE

        heading_rad = math.radians(self.heading)
        self.x += v * math.sin(heading_rad) * DT
        self.y += v * math.cos(heading_rad) * DT
        self.heading += math.degrees(omega) * DT

        while self.heading >= 360:
            self.heading -= 360
        while self.heading < 0:
            self.heading += 360

        self.speed = abs(v)

    def get_noisy_position(self):
        noise_x = random.gauss(0, GPS_NOISE_STD)
        noise_y = random.gauss(0, GPS_NOISE_STD)
        noise_heading = random.gauss(0, IMU_NOISE_STD)
        return self.x + noise_x, self.y + noise_y, self.heading + noise_heading


# ============================================================================
# Test Runner
# ============================================================================

def run_test(target_x, target_y, seed=42):
    random.seed(seed)

    nav = NavigationCore()
    robot = RobotPhysics()

    nav.clear_route()
    nav.add_waypoint(target_x, target_y)
    nav.reset()
    nav.quality = NavQuality.FIXED

    result = {
        'steps': 0,
        'max_heading_error': 0.0,
        'max_cross_track': 0.0,
        'success': False,
    }

    for step in range(2000):
        gps_x, gps_y, imu_heading = robot.get_noisy_position()

        nav.pos_x = gps_x
        nav.pos_y = gps_y
        nav.heading = imu_heading
        nav.speed = robot.speed

        output = nav.update(DT)
        robot.update(output['left_cmd'], output['right_cmd'])

        if abs(nav.debug['heading_error']) > result['max_heading_error']:
            result['max_heading_error'] = abs(nav.debug['heading_error'])
        if abs(nav.debug['cross_track_error']) > result['max_cross_track']:
            result['max_cross_track'] = abs(nav.debug['cross_track_error'])

        if nav.state == NavState.ARRIVED:
            result['success'] = True
            break
        result['steps'] = step + 1

    result['final_x'] = robot.x
    result['final_y'] = robot.y
    result['error'] = math.sqrt((target_x - robot.x)**2 + (target_y - robot.y)**2)

    return result


# ============================================================================
# Tests
# ============================================================================

def test_straight_5m():
    print("\n" + "="*50)
    print("TEST 1: Straight line 5m")
    print("="*50)

    r = run_test(5.0, 0.0, seed=42)

    print(f"Final: ({r['final_x']:.3f}, {r['final_y']:.3f})")
    print(f"Target: (5.0, 0.0)")
    print(f"Error: {r['error']*100:.1f} cm")
    print(f"Steps: {r['steps']}")
    print(f"Success: {'YES' if r['success'] else 'NO'}")
    print(f"Max heading error: {r['max_heading_error']:.1f} deg")
    print(f"Max cross-track: {r['max_cross_track']*100:.1f} cm")

    assert r['error'] < 0.20, f"Error {r['error']*100:.1f}cm > 20cm"


def test_50_runs():
    print("\n" + "="*50)
    print("TEST 2: 50 runs straight line")
    print("="*50)

    errors = []
    successes = 0

    for i in range(50):
        r = run_test(5.0, 0.0, seed=i * 1000)
        errors.append(r['error'])
        if r['error'] < 0.10:
            successes += 1

    avg_error = sum(errors) / len(errors)
    max_error = max(errors)

    print(f"Average error: {avg_error*100:.1f} cm")
    print(f"Max error: {max_error*100:.1f} cm")
    print(f"Success rate (<10cm): {successes/50*100:.0f}%")

    assert avg_error < 0.15, f"Avg error {avg_error*100:.1f}cm > 15cm"


def test_cross_track_sign():
    print("\n" + "="*50)
    print("TEST 3: Cross-track correction sign")
    print("="*50)
    print("Testing diagonal route with lateral offset")
    print()

    # For diagonal route (0,0) -> (5,5):
    # If robot starts at (0.5, 0) (to the right of line),
    # cross-track error should be positive
    # turnCmd += K_CROSSTRACK * crossTrackError makes turnCmd MORE positive
    # But to correct RIGHT offset, robot needs to turn LEFT (negative turnCmd)

    corrections = []
    cross_tracks = []

    for i in range(20):
        random.seed(i * 333)

        nav = NavigationCore()
        robot = RobotPhysics()

        # Start to the RIGHT of the line (cross-track > 0)
        robot.x = 0.5
        robot.y = 0.0
        robot.heading = 45.0  # Pointing toward target

        # Diagonal route
        nav.clear_route()
        nav.add_waypoint(5.0, 5.0)
        nav.reset()
        nav.quality = NavQuality.FIXED

        # Measure in first few steps before robot moves much
        for step in range(30):
            gps_x, gps_y, imu_heading = robot.get_noisy_position()
            nav.pos_x = gps_x
            nav.pos_y = gps_y
            nav.heading = imu_heading

            output = nav.update(DT)
            robot.update(output['left_cmd'], output['right_cmd'])

        ct = nav.debug['cross_track_error']
        tc = nav.debug['turn_cmd']
        corrections.append(tc)
        cross_tracks.append(ct)

        # Check final position
        dist_from_line = math.sqrt((robot.y - robot.x)**2) / math.sqrt(2)  # Distance to y=x line

        print(f"  Run {i:2d}: crossTrack={ct*100:+6.1f}cm, turnCmd={tc:+7.1f}%, "
              f"final=({robot.x:.2f},{robot.y:.2f})")

    avg_ct = sum(cross_tracks) / len(cross_tracks)
    avg_tc = sum(corrections) / len(corrections)

    print()
    print(f"Average cross-track error: {avg_ct*100:+.1f} cm")
    print(f"Average turn command: {avg_tc:+.1f}%")
    print()

    # Analysis
    if avg_ct > 0.01:  # Cross-track is positive (robot right of line)
        print(f"Cross-track is positive ({avg_ct*100:.1f}cm) - robot is RIGHT of line")
        if avg_tc > 0:
            print("WARNING: turn command is POSITIVE when robot is RIGHT of line")
            print("This means robot will turn RIGHT to correct RIGHT offset")
            print("This is WRONG - it should turn LEFT")
            print()
            print("BUG CONFIRMED: cross-track sign is inverted!")
            print()
            print("FIX: In rover.cpp line 2205, change:")
            print("  turnCmd += K_CROSSTRACK * g_crossTrackError;")
            print("To:")
            print("  turnCmd -= K_CROSSTRACK * g_crossTrackError;")
        else:
            print("OK: turn command is negative - correcting correctly")
    else:
        print("Cross-track near zero - robot is on the line")

    return avg_tc


def test_45_degrees():
    print("\n" + "="*50)
    print("TEST 4: 45 degree angle")
    print("="*50)

    r = run_test(3.0, 3.0, seed=123)

    print(f"Final: ({r['final_x']:.3f}, {r['final_y']:.3f})")
    print(f"Target: (3.0, 3.0)")
    print(f"Error: {r['error']*100:.1f} cm")

    assert r['error'] < 0.20, f"Error {r['error']*100:.1f}cm > 20cm"


def test_noise_variation():
    print("\n" + "="*50)
    print("TEST 5: GPS noise variation")
    print("="*50)

    noise_levels = [
        (0.005, "5mm (very good)"),
        (0.015, "15mm (RTK Fixed)"),
        (0.030, "30mm (good)"),
        (0.050, "50mm (RTK Float)"),
    ]

    global GPS_NOISE_STD
    original_noise = GPS_NOISE_STD

    for noise, name in noise_levels:
        GPS_NOISE_STD = noise
        errors = []
        for i in range(20):
            r = run_test(5.0, 0.0, seed=i*100 + noise*10000)
            errors.append(r['error'])
        avg = sum(errors) / len(errors)
        print(f"  {name}: avg error = {avg*100:.1f} cm")

    GPS_NOISE_STD = original_noise


# ============================================================================
# Main
# ============================================================================

if __name__ == '__main__':
    print("="*50)
    print("  Navigation Core Tests")
    print("="*50)

    test_straight_5m()
    test_50_runs()
    avg_turn = test_cross_track_sign()
    test_45_degrees()
    test_noise_variation()

    print("\n" + "="*50)
    print("  Summary")
    print("="*50)
    print(f"  All tests completed!")

    if avg_turn > 0:
        print()
        print("  WARNING: CROSS-TRACK SIGN BUG DETECTED!")
        print(f"     Average turn command: {avg_turn:.1f}% (should be negative)")
        print()
        print("  FIX: Change line 2205 in rover.cpp from:")
        print("    turnCmd += K_CROSSTRACK * g_crossTrackError;")
        print("  To:")
        print("    turnCmd -= K_CROSSTRACK * g_crossTrackError;")
