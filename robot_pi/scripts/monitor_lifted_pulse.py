#!/usr/bin/env python3
"""Record typed ROS evidence for the supervised first lifted pulse."""

from __future__ import annotations

import argparse
import time

from module_robot_msgs.msg import MotorStatus, RelayStatus, RobotStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class PulseMonitor(Node):
    def __init__(self) -> None:
        super().__init__("lifted_pulse_monitor")
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.status: list[RobotStatus] = []
        self.motor: list[MotorStatus] = []
        self.relay: list[RelayStatus] = []
        self._pulse_subscriptions = (
            self.create_subscription(
                RobotStatus, "/esp32/status", self.status.append, state_qos
            ),
            self.create_subscription(
                MotorStatus, "/motor/status", self.motor.append, sensor_qos
            ),
            self.create_subscription(
                RelayStatus, "/relay/status", self.relay.append, state_qos
            ),
        )


def _max_abs(messages: list[object], field: str) -> int:
    return max((abs(int(getattr(message, field))) for message in messages), default=0)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=70.0)
    args = parser.parse_args()
    if not 5.0 <= args.duration <= 180.0:
        parser.error("--duration must be in [5, 180]")

    rclpy.init()
    node = PulseMonitor()
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    states = sorted({int(message.state) for message in node.status})
    fault_codes = sorted({int(message.fault_code) for message in node.status})
    relay_masks = sorted({int(message.active_mask) for message in node.relay})
    valid_frames = [int(message.uart_valid_frames) for message in node.motor]
    valid_frame_delta = max(valid_frames) - min(valid_frames) if valid_frames else 0
    evidence = {
        "status_samples": len(node.status),
        "motor_samples": len(node.motor),
        "relay_samples": len(node.relay),
        "states": states,
        "fault_codes": fault_codes,
        "estop_seen": any(message.estop for message in node.status),
        "max_abs_applied_left": _max_abs(node.status, "applied_left_command"),
        "max_abs_applied_right": _max_abs(node.status, "applied_right_command"),
        "max_abs_uart_speed": _max_abs(node.status, "uart_speed"),
        "max_abs_uart_steer": _max_abs(node.status, "uart_steer"),
        "max_abs_left_feedback": _max_abs(node.motor, "left_feedback"),
        "max_abs_right_feedback": _max_abs(node.motor, "right_feedback"),
        "valid_frame_delta": valid_frame_delta,
        "relay_masks": relay_masks,
    }
    for key, value in evidence.items():
        print(f"{key}: {value}")

    checks = (
        len(node.status) > 0,
        len(node.motor) > 0,
        len(node.relay) > 0,
        RobotStatus.STATE_ARMED in states,
        evidence["max_abs_applied_left"] > 0,
        evidence["max_abs_applied_right"] > 0,
        evidence["max_abs_uart_speed"] > 0,
        evidence["max_abs_left_feedback"] > 0,
        evidence["max_abs_right_feedback"] > 0,
        valid_frame_delta > 0,
        not evidence["estop_seen"],
        fault_codes == [0],
        relay_masks == [0],
    )
    if not all(checks):
        print("RESULT: FAIL")
        return 1
    print("RESULT: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
