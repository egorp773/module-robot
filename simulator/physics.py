#!/usr/bin/env python3
"""
Physics simulation for tracked robot.
Receives PWM commands from ESP32, simulates kinematics, updates position.
"""

import math
from dataclasses import dataclass
from typing import Tuple


# Robot physical parameters
WHEELBASE = 0.38  # meters between tracks
MAX_SPEED_MPS = 0.5  # m/s at 100% PWM
MAX_PWM = 70  # Maximum PWM percentage
GPS_NOISE_STD = 0.015  # meters, 1-sigma


@dataclass
class RobotPhysics:
    """Tracks robot position and heading based on motor commands"""
    x: float = 0.0  # meters
    y: float = 0.0  # meters
    heading: float = 0.0  # degrees, 0=North, clockwise positive

    # Velocity tracking for GPS heading calculation
    prev_x: float = 0.0
    prev_y: float = 0.0
    vel_e: float = 0.0  # East velocity m/s
    vel_n: float = 0.0  # North velocity m/s

    # Optional: IMU heading (if robot has IMU calibration)
    imu_heading: float = 0.0
    has_imu: bool = False


def pwm_to_velocity(left_pwm: float, right_pwm: float) -> Tuple[float, float]:
    """
    Convert PWM percentages to wheel velocities.
    Positive PWM = forward, negative = backward.
    """
    # Clamp to valid range
    left_pwm = max(-MAX_PWM, min(MAX_PWM, left_pwm))
    right_pwm = max(-MAX_PWM, min(MAX_PWM, right_pwm))

    # Convert to m/s
    v_left = (left_pwm / MAX_PWM) * MAX_SPEED_MPS
    v_right = (right_pwm / MAX_PWM) * MAX_SPEED_MPS

    return v_left, v_right


def compute_kinematics(v_left: float, v_right: float, wheelbase: float = WHEELBASE) -> Tuple[float, float, float]:
    """
    Compute overall velocity and angular velocity from wheel velocities.

    For tracked robot:
    - v = (v_left + v_right) / 2 (average)
    - omega = (v_left - v_right) / wheelbase

    When left > right: robot turns LEFT (omega > 0, heading increases)
    When right > left: robot turns RIGHT (omega < 0, heading decreases)
    """
    v = (v_left + v_right) / 2.0
    omega = (v_left - v_right) / wheelbase
    return v, omega


def update_position(physics: RobotPhysics, left_pwm: float, right_pwm: float,
                   dt: float, add_noise: bool = True) -> Tuple[float, float, float]:
    """
    Update robot position based on PWM commands.

    Returns:
        noisy_x, noisy_y, noisy_heading (for GPS simulation)
    """
    # Convert PWM to velocities
    v_left, v_right = pwm_to_velocity(left_pwm, right_pwm)

    # Compute kinematics
    v, omega = compute_kinematics(v_left, v_right)

    # Update heading (in radians for trig)
    heading_rad = math.radians(physics.heading)

    # Update position
    # x += v * sin(heading) * dt  (East positive)
    # y += v * cos(heading) * dt  (North positive)
    physics.x += v * math.sin(heading_rad) * dt
    physics.y += v * math.cos(heading_rad) * dt

    # Update heading
    delta_heading = math.degrees(omega * dt)
    physics.heading = normalize_angle_360(physics.heading + delta_heading)

    # Track velocity for GPS heading
    if dt > 0:
        physics.vel_e = (physics.x - physics.prev_x) / dt
        physics.vel_n = (physics.y - physics.prev_y) / dt
    physics.prev_x = physics.x
    physics.prev_y = physics.y

    # Add GPS noise
    if add_noise:
        import random
        noise_x = random.gauss(0, GPS_NOISE_STD)
        noise_y = random.gauss(0, GPS_NOISE_STD)
    else:
        noise_x = 0
        noise_y = 0

    noisy_heading = physics.heading
    if physics.has_imu:
        # IMU has its own heading with calibration offset
        noisy_heading = normalize_angle_360(physics.imu_heading + get_imu_noise())

    return physics.x + noise_x, physics.y + noise_y, noisy_heading


def get_gps_heading(physics: RobotPhysics) -> float:
    """
    Calculate heading from GPS velocity.
    Returns heading in degrees, 0=North, clockwise positive.
    """
    speed = math.sqrt(physics.vel_e**2 + physics.vel_n**2)

    if speed < 0.05:  # Too slow for reliable heading
        return physics.heading  # Use last known heading

    # atan2(vel_e, vel_n) because:
    # - East velocity is X component
    # - North velocity is Y component
    # - atan2 gives angle from North, clockwise positive
    heading = math.degrees(math.atan2(physics.vel_e, physics.vel_n))
    return normalize_angle_360(heading)


def get_imu_noise() -> float:
    """Simulate IMU noise (small random offset)"""
    import random
    return random.gauss(0, 0.5)  # ~0.5 degree std dev


def normalize_angle_360(angle: float) -> float:
    """Normalize angle to 0-360 range"""
    while angle >= 360:
        angle -= 360
    while angle < 0:
        angle += 360
    return angle


def normalize_angle_180(angle: float) -> float:
    """Normalize angle to -180 to +180 range"""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def distance_to_point(x: float, y: float, target_x: float, target_y: float) -> float:
    """Calculate distance between two points"""
    return math.sqrt((target_x - x)**2 + (target_y - y)**2)
