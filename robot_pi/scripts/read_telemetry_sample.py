#!/usr/bin/env python3
"""Read one commissioning telemetry sample without the ROS CLI daemon."""

from __future__ import annotations

import argparse
import sys
import time

from module_robot_msgs.msg import MotorStatus, RobotStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


def _format_value(value: object) -> str:
    if isinstance(value, bool):
        return str(value).lower()
    return str(value)


def _print_fields(message: object, fields: tuple[str, ...]) -> None:
    for field in fields:
        print(f"{field}: {_format_value(getattr(message, field))}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("sample", choices=("status", "motor"))
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()
    if not 0.0 < args.timeout <= 30.0:
        parser.error("--timeout must be in (0, 30]")

    state_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=50,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    sensor_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=100,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )
    if args.sample == "status":
        message_type = RobotStatus
        topic = "/esp32/status"
        qos = state_qos
        fields = (
            "state",
            "connected",
            "armed",
            "estop",
            "fault_code",
            "fault_reason",
            "last_cmd_vel_age_ms",
            "last_heartbeat_age_ms",
            "last_motor_feedback_age_ms",
            "applied_left_command",
            "applied_right_command",
            "uart_speed",
            "uart_steer",
            "watchdog_trips",
            "boot_counter",
            "reset_reason",
        )
    else:
        message_type = MotorStatus
        topic = "/motor/status"
        qos = sensor_qos
        fields = (
            "sensor_monotonic_us",
            "left_feedback",
            "right_feedback",
            "battery_voltage",
            "board_temperature_c",
            "board_temperature_available",
            "controller_fault",
            "uart_valid_frames",
            "uart_invalid_frames",
        )

    rclpy.init()
    node = Node(f"commissioning_{args.sample}_sample")
    sample = None

    def receive(message):
        nonlocal sample
        sample = message

    subscription = node.create_subscription(message_type, topic, receive, qos)
    deadline = time.monotonic() + args.timeout
    try:
        while sample is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if sample is None:
            print(f"No {topic} sample within {args.timeout:.1f} s", file=sys.stderr)
            return 1
        _print_fields(sample, fields)
        return 0
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
