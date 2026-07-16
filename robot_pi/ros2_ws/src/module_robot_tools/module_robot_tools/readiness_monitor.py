"""Read-only candidate readiness diagnostics.

This node is deliberately not part of command authorization. It never calls
ARM/DISARM and never publishes velocity; module_robot_safety owns those gates.
"""

from __future__ import annotations

import math
import time
from typing import Dict, Optional

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from module_robot_msgs.msg import MotorStatus, RobotStatus, RtkStatus, SafetyState
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool


class ReadinessMonitor(Node):
    def __init__(self) -> None:
        super().__init__("readiness_monitor")
        defaults = {
            "publish_rate_hz": 1.0,
            "status_freshness_s": 0.75,
            "motor_freshness_s": 0.30,
            "imu_freshness_s": 0.20,
            "gnss_freshness_s": 0.50,
            "rtk_freshness_s": 0.50,
            "safety_freshness_s": 0.50,
            "auto_max_horizontal_accuracy_m": 0.03,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._limits = {
            "status": float(self.get_parameter("status_freshness_s").value),
            "motor": float(self.get_parameter("motor_freshness_s").value),
            "imu": float(self.get_parameter("imu_freshness_s").value),
            "gnss": float(self.get_parameter("gnss_freshness_s").value),
            "rtk": float(self.get_parameter("rtk_freshness_s").value),
            "safety": float(self.get_parameter("safety_freshness_s").value),
        }
        self._last_rx: Dict[str, float] = {}
        self._status: Optional[RobotStatus] = None
        self._motor: Optional[MotorStatus] = None
        self._rtk: Optional[RtkStatus] = None
        self._safety: Optional[SafetyState] = None

        self.create_subscription(
            RobotStatus, "/esp32/status", lambda msg: self._store("status", msg), 10
        )
        self.create_subscription(
            MotorStatus,
            "/motor/status",
            lambda msg: self._store("motor", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            RtkStatus, "/rtk/status", lambda msg: self._store("rtk", msg), 10
        )
        self.create_subscription(
            SafetyState, "/safety/state", lambda msg: self._store("safety", msg), 10
        )
        self.create_subscription(
            Imu, "/imu/data_raw", lambda _: self._touch("imu"), qos_profile_sensor_data
        )
        self.create_subscription(
            NavSatFix,
            "/gps/fix",
            lambda _: self._touch("gnss"),
            qos_profile_sensor_data,
        )

        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._manual_pub = self.create_publisher(
            Bool, "/module_robot_tools/manual_candidate_ready", 10
        )
        self._auto_pub = self.create_publisher(
            Bool, "/module_robot_tools/auto_candidate_ready", 10
        )
        rate = max(0.2, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._publish)

    def _touch(self, key: str) -> None:
        self._last_rx[key] = time.monotonic()

    def _store(self, key: str, msg) -> None:
        self._touch(key)
        if key == "status":
            self._status = msg
        elif key == "motor":
            self._motor = msg
        elif key == "rtk":
            self._rtk = msg
        elif key == "safety":
            self._safety = msg

    def _fresh(self, key: str) -> bool:
        received = self._last_rx.get(key)
        return received is not None and time.monotonic() - received <= self._limits[key]

    def _publish(self) -> None:
        fresh = {key: self._fresh(key) for key in self._limits}
        esp_ok = bool(
            fresh["status"]
            and self._status is not None
            and self._status.connected
            and not self._status.estop
            and self._status.fault_code == 0
        )
        motor_ok = bool(
            fresh["motor"]
            and self._motor is not None
            and self._motor.controller_fault == 0
        )
        safety_ok = bool(
            fresh["safety"]
            and self._safety is not None
            and self._safety.state not in (
                SafetyState.STATE_DISCONNECTED,
                SafetyState.STATE_FAULT,
                SafetyState.STATE_ESTOP,
            )
            and not self._safety.latched_fault
        )
        manual_ready = esp_ok and motor_ok and safety_ok

        max_hacc = float(self.get_parameter("auto_max_horizontal_accuracy_m").value)
        rtk_ok = bool(
            fresh["rtk"]
            and self._rtk is not None
            and self._rtk.carrier_solution == RtkStatus.CARRIER_FIXED
            and math.isfinite(self._rtk.horizontal_accuracy_m)
            and self._rtk.horizontal_accuracy_m <= max_hacc
        )
        safety_auto_inputs = bool(
            self._safety is not None
            and self._safety.heading_initialized
            and self._safety.localization_valid
            and self._safety.route_valid
            and self._safety.nav2_active
        )
        auto_ready = (
            manual_ready
            and rtk_ok
            and fresh["imu"]
            and fresh["gnss"]
            and safety_auto_inputs
        )

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [
            self._diagnostic(
                "module_robot/readiness/manual_candidate",
                manual_ready,
                {
                    "esp32_ok": esp_ok,
                    "motor_ok": motor_ok,
                    "safety_ok": safety_ok,
                    **{f"{key}_fresh": value for key, value in fresh.items()},
                },
            ),
            self._diagnostic(
                "module_robot/readiness/auto_candidate",
                auto_ready,
                {
                    "manual_candidate_ready": manual_ready,
                    "rtk_fixed_and_accurate": rtk_ok,
                    "heading_initialized": bool(
                        self._safety and self._safety.heading_initialized
                    ),
                    "localization_valid": bool(
                        self._safety and self._safety.localization_valid
                    ),
                    "route_valid": bool(self._safety and self._safety.route_valid),
                    "nav2_active": bool(self._safety and self._safety.nav2_active),
                },
            ),
        ]
        self._diagnostics_pub.publish(array)
        self._manual_pub.publish(Bool(data=manual_ready))
        self._auto_pub.publish(Bool(data=auto_ready))

    @staticmethod
    def _diagnostic(name: str, ready: bool, values: Dict[str, object]) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = name
        status.hardware_id = "module_robot"
        status.level = DiagnosticStatus.OK if ready else DiagnosticStatus.WARN
        status.message = "READY_CANDIDATE" if ready else "NOT_READY_CANDIDATE"
        status.values = [
            KeyValue(key=key, value=str(value).lower()) for key, value in values.items()
        ]
        return status


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReadinessMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
