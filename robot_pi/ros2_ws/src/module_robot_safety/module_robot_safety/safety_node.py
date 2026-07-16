"""Fail-closed ROS 2 command arbiter and operator control boundary.

Only this node publishes ``/cmd_vel_safe``.  It selects one complete command
source at a time and never combines manual and navigation velocities.  All
freshness decisions use local monotonic receipt time; upstream ROS stamps are
informational and are replaced on the safety output.

The node never arms automatically.  ``/safety/arm`` is an explicit operator
request which is forwarded to the ESP32 bridge only after the relevant gates
pass.  Reconnect, fault reset, ESTOP reset and mode selection all remain
DISARMED until another explicit ARM request.
"""

from __future__ import annotations

from dataclasses import replace
import math
import threading
import time
from typing import Optional

from geometry_msgs.msg import TwistStamped
from module_robot_msgs.msg import (
    FaultEvent,
    MotorStatus,
    RobotStatus,
    RtkStatus,
    SafetyState,
)
from module_robot_msgs.srv import (
    Arm,
    Disarm,
    ResetEstop,
    ResetFault,
    SetControlMode,
    SetRelays,
)
import rclpy
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from .policy import (
    CommandSlot,
    Conditions,
    FaultLatch,
    GateRequirements,
    Mode,
    State,
    auto_rejection,
    clamp,
    is_fresh,
    manual_rejection,
    selected_source,
)


LOCAL_FAULT_MOTOR_FEEDBACK_STALE = 0x8001
LOCAL_FAULT_MOTOR_CONTROLLER = 0x8002
LOCAL_FAULT_AUTO_PRECONDITION = 0x8003
LOCAL_FAULT_ESP32 = 0x8004


class SafetyNode(Node):
    """Single ROS velocity owner with explicit operator services."""

    def __init__(self) -> None:
        super().__init__("safety_node")
        self._declare_parameters()
        self._read_parameters()

        self._lock = threading.RLock()
        self._status_condition = threading.Condition(self._lock)
        self._status_generation = 0
        self._state_group = MutuallyExclusiveCallbackGroup()
        self._service_group = MutuallyExclusiveCallbackGroup()
        # STOP/DISARM/ESTOP must not wait behind ARM or a reset proxy call.
        self._zero_service_group = ReentrantCallbackGroup()
        self._client_group = ReentrantCallbackGroup()

        self._status: Optional[RobotStatus] = None
        self._motor: Optional[MotorStatus] = None
        self._rtk: Optional[RtkStatus] = None
        self._status_rx_s = float("-inf")
        self._motor_rx_s = float("-inf")
        self._rtk_rx_s = float("-inf")
        self._imu_rx_s = float("-inf")
        self._gnss_rx_s = float("-inf")
        self._imu_valid = False
        self._gnss_valid = False
        self._heading_initialized = False
        self._localization_valid = False
        self._route_valid = False
        self._nav2_active = False

        self._mode = Mode.DISARMED
        self._authority_epoch = 0
        self._selected_mode = Mode.MANUAL
        self._operator_armed = False
        self._auto_armed = False
        self._arm_in_progress = False
        self._arm_ack_s = float("-inf")
        self._ever_armed_confirmed = False
        self._local_estop_latched = False
        self._stop_requested = True
        self._stop_sent_for_stale = False
        self._shutting_down = False
        self._candidate_status = "BOOTSTRAP_ZERO"
        self._fault = FaultLatch()
        self._manual = CommandSlot()
        self._auto = CommandSlot()
        self._manual_generation = 0
        self._auto_generation = 0
        self._last_fail_safe_request_s = float("-inf")

        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._safe_pub = self.create_publisher(
            TwistStamped, "/cmd_vel_safe", command_qos
        )
        self._state_pub = self.create_publisher(SafetyState, "/safety/state", latched)
        self._fault_pub = self.create_publisher(
            FaultEvent, "/safety/fault_event", reliable
        )

        self.create_subscription(
            TwistStamped,
            "/cmd_vel_manual",
            self._on_manual_command,
            command_qos,
            callback_group=self._state_group,
        )
        self.create_subscription(
            TwistStamped,
            "/cmd_vel_nav",
            self._on_auto_command,
            command_qos,
            callback_group=self._state_group,
        )
        self.create_subscription(
            RobotStatus,
            "/esp32/status",
            self._on_status,
            reliable,
            callback_group=self._state_group,
        )
        # The bridge publishes these three streams BEST_EFFORT. A RELIABLE
        # subscription would be DDS-incompatible and permanently stale.
        self.create_subscription(
            MotorStatus,
            "/motor/status",
            self._on_motor,
            qos_profile_sensor_data,
            callback_group=self._state_group,
        )
        self.create_subscription(
            Imu,
            "/imu/data_raw",
            self._on_imu,
            qos_profile_sensor_data,
            callback_group=self._state_group,
        )
        self.create_subscription(
            NavSatFix,
            "/gps/fix",
            self._on_gnss,
            qos_profile_sensor_data,
            callback_group=self._state_group,
        )
        self.create_subscription(
            RtkStatus,
            "/rtk/status",
            self._on_rtk,
            reliable,
            callback_group=self._state_group,
        )
        self.create_subscription(
            Bool,
            "/heading_initialized",
            self._on_heading_initialized,
            latched,
            callback_group=self._state_group,
        )
        self.create_subscription(
            Bool,
            "/route/valid",
            self._on_route_valid,
            latched,
            callback_group=self._state_group,
        )
        self.create_subscription(
            Bool,
            "/localization/valid",
            self._on_localization_valid,
            reliable,
            callback_group=self._state_group,
        )
        self.create_subscription(
            Bool,
            "/nav2/active",
            self._on_nav2_active,
            reliable,
            callback_group=self._state_group,
        )

        self._bridge_arm = self.create_client(
            Arm, "/esp32/arm", callback_group=self._client_group
        )
        self._bridge_disarm = self.create_client(
            Disarm, "/esp32/disarm", callback_group=self._client_group
        )
        self._bridge_stop = self.create_client(
            Trigger, "/esp32/stop", callback_group=self._client_group
        )
        self._bridge_estop = self.create_client(
            Trigger, "/esp32/estop", callback_group=self._client_group
        )
        self._bridge_reset_fault = self.create_client(
            ResetFault, "/esp32/reset_fault", callback_group=self._client_group
        )
        self._bridge_reset_estop = self.create_client(
            ResetEstop, "/esp32/reset_estop", callback_group=self._client_group
        )
        self._bridge_set_relays = self.create_client(
            SetRelays, "/esp32/set_relays", callback_group=self._client_group
        )

        self.create_service(
            Arm,
            "/safety/arm",
            self._on_arm,
            callback_group=self._service_group,
        )
        self.create_service(
            Disarm,
            "/safety/disarm",
            self._on_disarm,
            callback_group=self._zero_service_group,
        )
        self.create_service(
            Trigger,
            "/safety/stop",
            self._on_stop,
            callback_group=self._zero_service_group,
        )
        self.create_service(
            Trigger,
            "/safety/estop",
            self._on_estop,
            callback_group=self._zero_service_group,
        )
        self.create_service(
            ResetFault,
            "/safety/reset_fault",
            self._on_reset_fault,
            callback_group=self._service_group,
        )
        self.create_service(
            ResetEstop,
            "/safety/reset_estop",
            self._on_reset_estop,
            callback_group=self._service_group,
        )
        self.create_service(
            SetControlMode,
            "/safety/set_control_mode",
            self._on_set_control_mode,
            callback_group=self._service_group,
        )
        self.create_service(
            SetRelays,
            "/safety/set_relays",
            self._on_set_relays,
            callback_group=self._service_group,
        )

        self.create_timer(
            1.0 / self._publish_rate_hz,
            self._tick,
            callback_group=self._state_group,
        )
        self._publish_zero_immediately()
        self.get_logger().info(
            "Safety node started fail-closed; explicit ARM is required and "
            f"autonomous_enabled={self._autonomous_enabled}"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "publish_rate_hz": 50.0,
            "status_timeout_ms": 500,
            "motor_feedback_timeout_ms": 250,
            "manual_cmd_timeout_ms": 200,
            "auto_cmd_timeout_ms": 200,
            "imu_timeout_ms": 250,
            "gnss_timeout_ms": 500,
            "rtk_timeout_ms": 500,
            "rtk_horizontal_accuracy_limit_m": 0.05,
            "max_manual_linear_mps": 0.15,
            "max_manual_reverse_mps": 0.15,
            "max_manual_angular_rad_s": 0.60,
            "max_auto_linear_mps": 0.10,
            "max_auto_angular_rad_s": 0.40,
            "arm_motor_feedback_zero_threshold": 5,
            "bridge_service_timeout_s": 0.75,
            "fault_latching": True,
            "estop_latching": True,
            "latch_motor_faults": True,
            "latch_auto_precondition_loss": True,
            "disarm_on_disconnect": True,
            "disarm_on_motor_fault": True,
            "manual_require_connected": True,
            "manual_require_motor_feedback": True,
            "manual_require_no_estop": True,
            "manual_require_no_motor_fault": True,
            "manual_require_operator_arm": True,
            "auto_require_rtk_fixed": True,
            "auto_require_rtk_accuracy": True,
            "auto_require_imu_fresh": True,
            "auto_require_gnss_fresh": True,
            "auto_require_heading_initialized": True,
            "auto_require_localization_valid": True,
            "auto_require_route_valid": True,
            "auto_require_nav2_active": True,
            "auto_require_explicit_arm": True,
            "autonomous_enabled": False,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

    def _read_parameters(self) -> None:
        value = lambda name: self.get_parameter(name).value
        self._publish_rate_hz = float(value("publish_rate_hz"))
        self._status_timeout_s = int(value("status_timeout_ms")) / 1000.0
        self._motor_timeout_s = int(value("motor_feedback_timeout_ms")) / 1000.0
        self._manual_timeout_s = int(value("manual_cmd_timeout_ms")) / 1000.0
        self._auto_timeout_s = int(value("auto_cmd_timeout_ms")) / 1000.0
        self._imu_timeout_s = int(value("imu_timeout_ms")) / 1000.0
        self._gnss_timeout_s = int(value("gnss_timeout_ms")) / 1000.0
        self._rtk_timeout_s = int(value("rtk_timeout_ms")) / 1000.0
        self._rtk_hacc_limit_m = float(value("rtk_horizontal_accuracy_limit_m"))
        self._max_manual_linear = float(value("max_manual_linear_mps"))
        self._max_manual_reverse = float(value("max_manual_reverse_mps"))
        self._max_manual_angular = float(value("max_manual_angular_rad_s"))
        self._max_auto_linear = float(value("max_auto_linear_mps"))
        self._max_auto_angular = float(value("max_auto_angular_rad_s"))
        self._arm_feedback_zero = int(value("arm_motor_feedback_zero_threshold"))
        self._bridge_service_timeout_s = float(value("bridge_service_timeout_s"))
        self._latch_motor_faults = bool(value("latch_motor_faults"))
        self._latch_auto_loss = bool(value("latch_auto_precondition_loss"))
        self._disarm_on_motor_fault = bool(value("disarm_on_motor_fault"))
        self._autonomous_enabled = bool(value("autonomous_enabled"))

        # These gates are safety invariants, not disable switches. Keep the
        # parameters for explicit configuration/audit and fail closed if a
        # file attempts to weaken one.
        mandatory_true = (
            "fault_latching",
            "estop_latching",
            "latch_motor_faults",
            "latch_auto_precondition_loss",
            "disarm_on_disconnect",
            "disarm_on_motor_fault",
            "manual_require_connected",
            "manual_require_motor_feedback",
            "manual_require_no_estop",
            "manual_require_no_motor_fault",
            "manual_require_operator_arm",
            "auto_require_rtk_fixed",
            "auto_require_rtk_accuracy",
            "auto_require_imu_fresh",
            "auto_require_gnss_fresh",
            "auto_require_heading_initialized",
            "auto_require_localization_valid",
            "auto_require_route_valid",
            "auto_require_nav2_active",
            "auto_require_explicit_arm",
        )
        weakened = [name for name in mandatory_true if not bool(value(name))]
        if weakened:
            raise ValueError(
                "safety requirements cannot be disabled: " + ", ".join(weakened)
            )
        self._requirements = GateRequirements()

        finite_positive = {
            "publish_rate_hz": self._publish_rate_hz,
            "status_timeout": self._status_timeout_s,
            "motor_timeout": self._motor_timeout_s,
            "manual_timeout": self._manual_timeout_s,
            "auto_timeout": self._auto_timeout_s,
            "imu_timeout": self._imu_timeout_s,
            "gnss_timeout": self._gnss_timeout_s,
            "rtk_timeout": self._rtk_timeout_s,
            "rtk_hacc_limit": self._rtk_hacc_limit_m,
            "max_manual_linear": self._max_manual_linear,
            "max_manual_reverse": self._max_manual_reverse,
            "max_manual_angular": self._max_manual_angular,
            "max_auto_linear": self._max_auto_linear,
            "max_auto_angular": self._max_auto_angular,
            "bridge_service_timeout": self._bridge_service_timeout_s,
        }
        for name, parameter in finite_positive.items():
            if not math.isfinite(parameter) or parameter <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if not 20.0 <= self._publish_rate_hz <= 100.0:
            raise ValueError("publish_rate_hz must be in [20, 100]")
        if self._arm_feedback_zero < 0:
            raise ValueError("arm_motor_feedback_zero_threshold must be non-negative")
        if self._max_manual_linear > 0.15 or self._max_manual_reverse > 0.15:
            raise ValueError("manual linear limits must not exceed 0.15 m/s")
        if self._max_manual_angular > 0.60:
            raise ValueError("manual angular limit must not exceed 0.60 rad/s")
        if self._max_auto_linear > 0.15 or self._max_auto_angular > 0.60:
            raise ValueError("AUTO limits exceed ESP32 hard limits")

    @staticmethod
    def _now_s() -> float:
        return time.monotonic()

    def _conditions(self, now_s: Optional[float] = None) -> Conditions:
        now = self._now_s() if now_s is None else now_s
        status_fresh = is_fresh(self._status_rx_s, now, self._status_timeout_s)
        motor_fresh = is_fresh(self._motor_rx_s, now, self._motor_timeout_s)
        rtk_fresh = is_fresh(self._rtk_rx_s, now, self._rtk_timeout_s)
        status = self._status
        motor = self._motor
        rtk = self._rtk
        status_estop = bool(
            status_fresh
            and status is not None
            and (status.estop or status.state == RobotStatus.STATE_ESTOP)
        )
        return Conditions(
            connected=bool(status_fresh and status is not None and status.connected),
            esp32_armed=bool(
                status_fresh
                and status is not None
                and status.connected
                and status.armed
                and status.state == RobotStatus.STATE_ARMED
            ),
            esp32_fault_free=bool(
                status_fresh
                and status is not None
                and status.fault_code == 0
                and status.state != RobotStatus.STATE_FAULT
            ),
            motor_feedback_fresh=motor_fresh,
            motor_fault_free=bool(
                motor_fresh and motor is not None and motor.controller_fault == 0
            ),
            estop_clear=not self._local_estop_latched and not status_estop,
            operator_armed=self._operator_armed,
            explicit_auto_arm=self._auto_armed,
            rtk_fresh=rtk_fresh,
            rtk_fixed=bool(
                rtk_fresh
                and rtk is not None
                and rtk.carrier_solution == RtkStatus.CARRIER_FIXED
            ),
            rtk_accuracy_ok=bool(
                rtk_fresh
                and rtk is not None
                and math.isfinite(rtk.horizontal_accuracy_m)
                and 0.0 <= rtk.horizontal_accuracy_m <= self._rtk_hacc_limit_m
            ),
            imu_fresh=self._imu_valid
            and is_fresh(self._imu_rx_s, now, self._imu_timeout_s),
            gnss_fresh=self._gnss_valid
            and is_fresh(self._gnss_rx_s, now, self._gnss_timeout_s),
            heading_initialized=self._heading_initialized,
            localization_valid=self._localization_valid,
            route_valid=self._route_valid,
            nav2_active=self._nav2_active,
        )

    def _on_status(self, message: RobotStatus) -> None:
        now = self._now_s()
        with self._status_condition:
            previous = self._status
            previous_estop = bool(
                previous is not None
                and (previous.estop or previous.state == RobotStatus.STATE_ESTOP)
            )
            previous_fault = bool(
                previous is not None
                and (
                    previous.fault_code != 0
                    or previous.state == RobotStatus.STATE_FAULT
                )
            )
            new_estop = bool(
                message.estop or message.state == RobotStatus.STATE_ESTOP
            )
            new_fault = bool(
                message.fault_code != 0
                or message.state == RobotStatus.STATE_FAULT
            )
            hazard_transition = bool(
                (new_estop and not previous_estop)
                or (new_fault and not previous_fault)
                or (
                    new_fault
                    and previous is not None
                    and message.fault_code != previous.fault_code
                )
                or (
                    previous is not None
                    and previous.connected
                    and not message.connected
                )
            )
            self._status = message
            self._status_rx_s = now
            cancel_pending = (
                self._operator_armed
                or self._arm_in_progress
                or hazard_transition
            )
            if message.estop or message.state == RobotStatus.STATE_ESTOP:
                self._local_estop_latched = True
                self._clear_motion_authority_locked(
                    cancel_pending=cancel_pending
                )
            if message.fault_code != 0 or message.state == RobotStatus.STATE_FAULT:
                reason = message.fault_reason or f"ESP32_FAULT_{message.fault_code}"
                code = message.fault_code or LOCAL_FAULT_ESP32
                self._latch_fault_locked(code, reason)
                self._clear_motion_authority_locked(
                    cancel_pending=cancel_pending
                )
            if not message.connected or message.state in (
                RobotStatus.STATE_BOOT,
                RobotStatus.STATE_DISCONNECTED,
            ):
                # A later reconnect must never inherit operator authority.
                self._clear_motion_authority_locked(
                    cancel_pending=cancel_pending
                )
            elif message.armed and message.state == RobotStatus.STATE_ARMED:
                self._ever_armed_confirmed = True
            elif self._operator_armed and self._ever_armed_confirmed:
                # A confirmed hardware transition away from ARMED removes Pi
                # authority too. It can never be recovered by later cmd_vel.
                self._clear_motion_authority_locked()
            self._status_generation += 1
            self._status_condition.notify_all()

    def _on_motor(self, message: MotorStatus) -> None:
        with self._lock:
            self._motor = message
            self._motor_rx_s = self._now_s()

    def _on_rtk(self, message: RtkStatus) -> None:
        with self._lock:
            self._rtk = message
            self._rtk_rx_s = self._now_s()

    def _on_imu(self, message: Imu) -> None:
        values = (
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
        )
        with self._lock:
            self._imu_valid = all(math.isfinite(value) for value in values)
            self._imu_rx_s = self._now_s()

    def _on_gnss(self, message: NavSatFix) -> None:
        valid = bool(
            message.status.status >= NavSatStatus.STATUS_FIX
            and math.isfinite(message.latitude)
            and math.isfinite(message.longitude)
            and -90.0 <= message.latitude <= 90.0
            and -180.0 <= message.longitude <= 180.0
        )
        with self._lock:
            self._gnss_valid = valid
            self._gnss_rx_s = self._now_s()

    def _on_heading_initialized(self, message: Bool) -> None:
        with self._lock:
            self._heading_initialized = bool(message.data)

    def _on_localization_valid(self, message: Bool) -> None:
        with self._lock:
            self._localization_valid = bool(message.data)

    def _on_route_valid(self, message: Bool) -> None:
        with self._lock:
            self._route_valid = bool(message.data)

    def _on_nav2_active(self, message: Bool) -> None:
        with self._lock:
            self._nav2_active = bool(message.data)

    def _validated_planar_command(
        self,
        message: TwistStamped,
        *,
        max_forward: float,
        max_reverse: float,
        max_angular: float,
    ) -> Optional[tuple[float, float]]:
        twist = message.twist
        components = (
            float(twist.linear.x),
            float(twist.linear.y),
            float(twist.linear.z),
            float(twist.angular.x),
            float(twist.angular.y),
            float(twist.angular.z),
        )
        if not all(math.isfinite(value) for value in components):
            return None
        linear, lateral, vertical, roll, pitch, angular = components
        # The drivetrain accepts exactly one planar differential command. Do
        # not silently reinterpret a holonomic or malformed Twist as a valid
        # tracked-robot command.
        if any(abs(value) > 1.0e-9 for value in (lateral, vertical, roll, pitch)):
            return None
        return (
            clamp(linear, -max_reverse, max_forward),
            clamp(angular, -max_angular, max_angular),
        )

    def _on_manual_command(self, message: TwistStamped) -> None:
        command = self._validated_planar_command(
            message,
            max_forward=self._max_manual_linear,
            max_reverse=self._max_manual_reverse,
            max_angular=self._max_manual_angular,
        )
        invalid = False
        with self._lock:
            if command is None or self._shutting_down:
                self._manual.invalidate()
                self._stop_requested = True
                self._stop_sent_for_stale = False
                self._candidate_status = "INVALID_MANUAL_COMMAND"
                invalid = True
            else:
                self._manual_generation += 1
                self._manual.update(*command, self._now_s(), self._manual_generation)
                self._stop_requested = False
                self._stop_sent_for_stale = False
        if invalid:
            self._publish_zero_immediately("INVALID_MANUAL_COMMAND")
            self._request_bridge_zero(disarm=False)

    def _on_auto_command(self, message: TwistStamped) -> None:
        command = self._validated_planar_command(
            message,
            max_forward=self._max_auto_linear,
            max_reverse=0.0,
            max_angular=self._max_auto_angular,
        )
        invalid = False
        with self._lock:
            if command is None or self._shutting_down:
                self._auto.invalidate()
                self._stop_requested = True
                self._stop_sent_for_stale = False
                self._candidate_status = "INVALID_AUTO_COMMAND"
                invalid = True
            else:
                self._auto_generation += 1
                self._auto.update(*command, self._now_s(), self._auto_generation)
                self._stop_requested = False
                self._stop_sent_for_stale = False
        if invalid:
            self._publish_zero_immediately("INVALID_AUTO_COMMAND")
            self._request_bridge_zero(disarm=False)

    def _latch_fault_locked(self, code: int, reason: str) -> None:
        if self._fault.active:
            return
        if not self._fault.latch(code, reason, enabled=True):
            return
        event = FaultEvent()
        event.header.stamp = self.get_clock().now().to_msg()
        event.event_monotonic_us = time.monotonic_ns() // 1000
        event.fault_code = int(code) & 0xFFFF
        event.fault_name = reason
        event.detail = self._candidate_status
        event.occurrence_count = self._fault.occurrence_count
        event.latched = True
        self._fault_pub.publish(event)
        self.get_logger().error(f"Safety fault latched: {reason} ({code:#06x})")

    def _clear_motion_authority_locked(self, *, cancel_pending: bool = True) -> None:
        if cancel_pending:
            self._authority_epoch += 1
        self._mode = Mode.DISARMED
        self._operator_armed = False
        self._auto_armed = False
        self._arm_in_progress = False
        self._arm_ack_s = float("-inf")
        self._ever_armed_confirmed = False
        self._stop_requested = True
        self._manual.invalidate()
        self._auto.invalidate()

    def _apply_live_faults_locked(self, conditions: Conditions) -> None:
        if not conditions.estop_clear:
            self._local_estop_latched = True
            self._clear_motion_authority_locked()
            return
        if self._operator_armed and not conditions.connected:
            # Covers silent STATUS expiry, not only an explicit connected=false
            # callback. Reconnect always starts with no operator authority.
            self._clear_motion_authority_locked()
            return
        if (
            self._operator_armed
            and not conditions.esp32_armed
            and not self._ever_armed_confirmed
            and self._now_s() - self._arm_ack_s > self._status_timeout_s
        ):
            self._clear_motion_authority_locked()
            return
        if not conditions.esp32_fault_free and conditions.connected:
            code = (
                self._status.fault_code
                if self._status is not None and self._status.fault_code
                else LOCAL_FAULT_ESP32
            )
            reason = (
                self._status.fault_reason
                if self._status is not None and self._status.fault_reason
                else "ESP32_FAULT"
            )
            self._latch_fault_locked(code, reason)
            self._clear_motion_authority_locked()
            return
        if self._operator_armed and not conditions.motor_feedback_fresh:
            self._latch_fault_locked(
                LOCAL_FAULT_MOTOR_FEEDBACK_STALE, "MOTOR_FEEDBACK_STALE"
            )
            self._clear_motion_authority_locked()
            return
        if self._operator_armed and not conditions.motor_fault_free:
            if self._latch_motor_faults:
                self._latch_fault_locked(
                    LOCAL_FAULT_MOTOR_CONTROLLER, "MOTOR_CONTROLLER_FAULT"
                )
            self._clear_motion_authority_locked()
            return
        if self._mode == Mode.AUTO and self._operator_armed:
            rejection = auto_rejection(conditions, self._requirements)
            if rejection is not None and rejection != "ESP32_NOT_ARMED":
                if self._latch_auto_loss:
                    self._latch_fault_locked(
                        LOCAL_FAULT_AUTO_PRECONDITION,
                        f"AUTO_PRECONDITION_LOST:{rejection}",
                    )
                self._clear_motion_authority_locked()

    def _derive_state_locked(self, conditions: Conditions) -> State:
        if self._local_estop_latched or not conditions.estop_clear:
            self._candidate_status = "ESTOP_ACTIVE"
            return State.ESTOP
        if self._fault.active:
            self._candidate_status = f"LATCHED_FAULT:{self._fault.reason}"
            return State.FAULT
        if not conditions.connected:
            self._candidate_status = "ESP32_DISCONNECTED"
            return State.DISCONNECTED
        if self._mode == Mode.DISARMED or not self._operator_armed:
            rejection = self._prearm_rejection_locked(Mode.MANUAL)
            self._candidate_status = (
                "MANUAL_READY_TO_ARM"
                if rejection is None
                else f"MANUAL_BLOCKED:{rejection}"
            )
            return State.DISARMED
        if self._mode == Mode.MANUAL:
            rejection = manual_rejection(conditions, self._requirements)
            if rejection is not None:
                self._candidate_status = rejection
                return State.DISARMED
            self._candidate_status = "READY_MANUAL"
            return State.MANUAL
        rejection = auto_rejection(conditions, self._requirements)
        if rejection is not None:
            self._candidate_status = rejection
            # ARM ACK can precede the next STATUS sample. Publishing FAULT in
            # that bounded interval would make the bridge immediately undo a
            # valid explicit ARM. Keep output zero and report DISARMED until
            # ESP32 ARMED is actually observed; every other AUTO gate loss is
            # fail-closed and is latched by _apply_live_faults_locked().
            if rejection == "ESP32_NOT_ARMED":
                return State.DISARMED
            return State.FAULT
        self._candidate_status = "READY_AUTO"
        return State.AUTO

    def _tick(self) -> None:
        request_fail_safe = False
        disarm_fail_safe = False
        with self._lock:
            now = self._now_s()
            conditions = self._conditions(now)
            was_armed = self._operator_armed
            self._apply_live_faults_locked(conditions)
            conditions = self._conditions(now)
            state = self._derive_state_locked(conditions)
            unexpected_hardware_arm = bool(
                conditions.esp32_armed
                and not self._operator_armed
                and not self._arm_in_progress
            )

            manual_fresh = bool(
                manual_rejection(conditions, self._requirements) is None
                and self._manual.fresh(now, self._manual_timeout_s)
            )
            auto_fresh = bool(
                self._autonomous_enabled
                and auto_rejection(conditions, self._requirements) is None
                and self._auto.fresh(now, self._auto_timeout_s)
            )
            source = selected_source(
                state, self._stop_requested, manual_fresh, auto_fresh
            )
            if self._stop_requested and state in (State.MANUAL, State.AUTO):
                self._candidate_status = "STOP_REQUESTED"
            linear = 0.0
            angular = 0.0
            if source == "MANUAL":
                linear = self._manual.linear_m_s
                angular = self._manual.angular_rad_s
            elif source == "AUTO":
                linear = self._auto.linear_m_s
                angular = self._auto.angular_rad_s

            active_state = state in (State.MANUAL, State.AUTO)
            commands_stale = active_state and not self._stop_requested and source == "ZERO"
            if commands_stale and not self._stop_sent_for_stale:
                self._stop_sent_for_stale = True
                request_fail_safe = True
                self._candidate_status = (
                    "MANUAL_CMD_STALE" if state == State.MANUAL else "AUTO_CMD_STALE"
                )
            if was_armed and not self._operator_armed:
                request_fail_safe = True
                # Loss of any previously granted authority always removes the
                # ESP32 ARM state in this commissioning architecture.
                disarm_fail_safe = True
            if unexpected_hardware_arm:
                # Covers a direct call to /esp32/arm or any inconsistent
                # bridge state. Safety never adopts externally-created motion
                # authority; it removes it.
                linear = 0.0
                angular = 0.0
                self._candidate_status = "UNEXPECTED_ESP32_ARMED"
                request_fail_safe = True
                disarm_fail_safe = True

            command = TwistStamped()
            command.header.stamp = self.get_clock().now().to_msg()
            command.header.frame_id = "base_link"
            command.twist.linear.x = linear
            command.twist.angular.z = angular
            safety = self._make_state_message_locked(state)
            # Publish while holding the same lock used by ESTOP/DISARM. This
            # prevents a command computed before a concurrent safety transition
            # from being emitted after that transition's hard zero.
            self._safe_pub.publish(command)
            self._state_pub.publish(safety)
        if request_fail_safe:
            self._request_bridge_zero(disarm=disarm_fail_safe)

    def _make_state_message_locked(self, state: State) -> SafetyState:
        message = SafetyState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.state = int(state)
        message.current_candidate_status = self._candidate_status
        message.latched_fault = self._fault.active
        message.latched_fault_code = self._fault.code
        message.latched_fault_reason = self._fault.reason
        message.operator_armed = self._operator_armed
        message.auto_armed = self._auto_armed
        message.heading_initialized = self._heading_initialized
        message.localization_valid = self._localization_valid
        message.route_valid = self._route_valid
        message.nav2_active = self._nav2_active
        return message

    def _publish_zero_immediately(
        self, candidate_status: Optional[str] = None
    ) -> None:
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "base_link"
        self._safe_pub.publish(command)
        with self._lock:
            state = self._derive_state_locked(self._conditions())
            if candidate_status is not None:
                self._candidate_status = candidate_status
            safety = self._make_state_message_locked(state)
        self._state_pub.publish(safety)

    def _request_bridge_zero(self, *, disarm: bool) -> None:
        now = self._now_s()
        with self._lock:
            if now - self._last_fail_safe_request_s < 0.5:
                return
            self._last_fail_safe_request_s = now
        if self._bridge_stop.service_is_ready():
            self._bridge_stop.call_async(Trigger.Request())
        if disarm and self._bridge_disarm.service_is_ready():
            self._bridge_disarm.call_async(Disarm.Request())

    def _call_bridge(self, client, request):
        if not client.service_is_ready():
            return None
        try:
            future = client.call_async(request)
        except Exception as exc:
            self.get_logger().error(f"bridge service send failed: {exc}")
            return None
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(self._bridge_service_timeout_s):
            return None
        try:
            return future.result()
        except Exception as exc:
            self.get_logger().error(f"bridge service failed: {exc}")
            return None

    def _hardware_clear_locked(self, *, estop: bool) -> bool:
        status = self._status
        if status is None or not is_fresh(
            self._status_rx_s, self._now_s(), self._status_timeout_s
        ):
            return False
        common = bool(
            status.connected
            and not status.armed
            and status.state == RobotStatus.STATE_DISARMED
        )
        if estop:
            return common and not status.estop
        return common and status.fault_code == 0

    def _wait_for_post_request_clear_status(
        self, *, after_generation: int, estop: bool
    ) -> bool:
        """Wait for ordered STATUS evidence newer than a reset request.

        COMMAND_ACK and STATUS use different ROS endpoints, so an older fault
        sample can be queued when the ACK arrives. Reliable ordering within the
        STATUS topic plus this generation gate prevents that stale sample from
        re-latching immediately after a reset.
        """

        deadline = self._now_s() + max(2.0, 3.0 * self._status_timeout_s)
        with self._status_condition:
            while True:
                if (
                    self._status_generation > after_generation
                    and self._hardware_clear_locked(estop=estop)
                ):
                    return True
                remaining = deadline - self._now_s()
                if remaining <= 0.0:
                    return False
                self._status_condition.wait(timeout=remaining)

    def _prearm_rejection_locked(self, requested_mode: Mode) -> Optional[str]:
        if self._arm_in_progress:
            return "ARM_ALREADY_IN_PROGRESS"
        if self._fault.active:
            return f"FAULT_LATCHED:{self._fault.reason}"
        conditions = self._conditions()
        if not conditions.estop_clear:
            return "ESTOP_ACTIVE"
        if not conditions.connected:
            return "ESP32_DISCONNECTED"
        if not conditions.esp32_fault_free:
            return "ESP32_FAULT"
        if not conditions.motor_feedback_fresh:
            return "MOTOR_FEEDBACK_STALE"
        if not conditions.motor_fault_free:
            return "MOTOR_CONTROLLER_FAULT"
        if self._status is None or self._status.state != RobotStatus.STATE_DISARMED:
            return "ESP32_NOT_DISARMED"
        if self._status.armed:
            return "ESP32_ALREADY_ARMED"
        if self._status.applied_left_command != 0 or self._status.applied_right_command != 0:
            return "APPLIED_COMMAND_NOT_ZERO"
        if self._motor is None or (
            abs(self._motor.left_feedback) > self._arm_feedback_zero
            or abs(self._motor.right_feedback) > self._arm_feedback_zero
        ):
            return "MOTOR_FEEDBACK_NOT_ZERO"
        if requested_mode == Mode.AUTO:
            if not self._autonomous_enabled:
                return "AUTO_NOT_READY:AUTONOMOUS_DISABLED"
            proposed = replace(
                conditions,
                operator_armed=True,
                explicit_auto_arm=True,
                esp32_armed=True,
            )
            rejection = auto_rejection(proposed, self._requirements)
            if rejection is not None:
                return f"AUTO_NOT_READY:{rejection}"
        return None

    def _on_arm(self, request: Arm.Request, response: Arm.Response) -> Arm.Response:
        if request.arm_nonce == 0 or request.requested_mode not in (
            int(Mode.MANUAL),
            int(Mode.AUTO),
        ):
            response.success = False
            response.resulting_state = SafetyState.STATE_DISARMED
            response.message = "ARM requires non-zero nonce and MANUAL(1) or AUTO(2) mode"
            return response
        requested_mode = Mode(request.requested_mode)
        with self._lock:
            rejection = self._prearm_rejection_locked(requested_mode)
            if rejection is not None:
                response.success = False
                response.resulting_state = int(self._derive_state_locked(self._conditions()))
                response.message = rejection
                return response
            # Commands received before the ARM acknowledgement are never
            # eligible. A new post-ARM source sample is mandatory.
            self._manual.invalidate()
            self._auto.invalidate()
            self._stop_requested = True
            self._stop_sent_for_stale = False
            self._arm_in_progress = True
            arm_epoch = self._authority_epoch
            self._publish_zero_immediately()

        bridge_response = self._call_bridge(self._bridge_arm, request)
        with self._lock:
            cancelled = arm_epoch != self._authority_epoch
            self._arm_in_progress = False
            if (
                not cancelled
                and bridge_response is not None
                and bridge_response.success
                and not self._local_estop_latched
                and not self._fault.active
            ):
                # Drop any source sample received while the bridge ARM service
                # was in flight. Motion requires a distinct post-ACK sample.
                self._manual.invalidate()
                self._auto.invalidate()
                self._mode = requested_mode
                self._selected_mode = requested_mode
                self._operator_armed = True
                self._auto_armed = requested_mode == Mode.AUTO
                self._ever_armed_confirmed = False
                self._arm_ack_s = self._now_s()
                self._stop_requested = True
                response.success = True
                response.resulting_state = int(
                    State.AUTO if requested_mode == Mode.AUTO else State.MANUAL
                )
                response.message = (
                    "ARM acknowledged; waiting for ESP32 ARMED status and a new "
                    "post-ARM command"
                )
            else:
                self._clear_motion_authority_locked()
                response.success = False
                response.resulting_state = int(
                    self._derive_state_locked(self._conditions())
                )
                response.message = (
                    "ARM cancelled by a newer STOP/DISARM/ESTOP or safety transition"
                    if cancelled
                    else bridge_response.message
                    if bridge_response is not None
                    else "ESP32 ARM service unavailable or timed out; DISARM requested"
                )
        if not response.success:
            self._request_bridge_zero(disarm=True)
        return response

    def _on_disarm(
        self, request: Disarm.Request, response: Disarm.Response
    ) -> Disarm.Response:
        del request
        with self._lock:
            self._clear_motion_authority_locked()
        self._publish_zero_immediately()
        self._request_bridge_zero(disarm=False)
        bridge_response = self._call_bridge(self._bridge_disarm, Disarm.Request())
        response.success = bool(bridge_response is not None and bridge_response.success)
        response.resulting_state = SafetyState.STATE_DISARMED
        response.message = (
            "DISARM acknowledged"
            if response.success
            else "local authority cleared and zero published; ESP32 DISARM ACK missing"
        )
        return response

    def _on_stop(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        with self._lock:
            # STOP is a newer operator ordering point even though it normally
            # leaves an already-established ARM in place.
            self._authority_epoch += 1
            if self._arm_in_progress:
                self._arm_in_progress = False
            self._stop_requested = True
            self._stop_sent_for_stale = True
            self._manual.invalidate()
            self._auto.invalidate()
            self._candidate_status = "STOP_REQUESTED"
        self._publish_zero_immediately("STOP_REQUESTED")
        bridge_response = self._call_bridge(self._bridge_stop, Trigger.Request())
        response.success = bool(bridge_response is not None and bridge_response.success)
        response.message = (
            "STOP acknowledged; a new source command is required"
            if response.success
            else "zero published; ESP32 STOP ACK missing and watchdog remains active"
        )
        return response

    def _on_estop(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        with self._lock:
            self._local_estop_latched = True
            self._clear_motion_authority_locked()
            self._candidate_status = "ESTOP_ACTIVE"
        self._publish_zero_immediately()
        bridge_response = self._call_bridge(self._bridge_estop, Trigger.Request())
        response.success = bool(bridge_response is not None and bridge_response.success)
        response.message = (
            "ESTOP acknowledged and latched"
            if response.success
            else "local ESTOP latched and zero published; ESP32 ACK missing"
        )
        return response

    def _on_reset_fault(
        self, request: ResetFault.Request, response: ResetFault.Response
    ) -> ResetFault.Response:
        del request
        with self._lock:
            self._clear_motion_authority_locked()
            reset_epoch = self._authority_epoch
            status_generation = self._status_generation
            now = self._now_s()
            status_fresh = is_fresh(
                self._status_rx_s, now, self._status_timeout_s
            )
            hardware_fault = bool(
                status_fresh
                and self._status is not None
                and (
                    self._status.fault_code != 0
                    or self._status.state == RobotStatus.STATE_FAULT
                )
            )
            hardware_confirmed_clear = self._hardware_clear_locked(estop=False)
        self._publish_zero_immediately()
        bridge_response = None
        if hardware_fault:
            bridge_response = self._call_bridge(
                self._bridge_reset_fault, ResetFault.Request()
            )
            if (
                bridge_response is not None
                and bridge_response.success
                and bridge_response.resulting_state == RobotStatus.STATE_DISARMED
            ):
                hardware_confirmed_clear = self._wait_for_post_request_clear_status(
                    after_generation=status_generation, estop=False
                )
        with self._lock:
            cancelled = reset_epoch != self._authority_epoch
            if not cancelled and (
                hardware_confirmed_clear
                or (
                    bridge_response is not None
                    and bridge_response.success
                    and bridge_response.resulting_state == RobotStatus.STATE_DISARMED
                    and hardware_confirmed_clear
                )
            ):
                self._fault.reset()
                response.success = True
                response.resulting_state = SafetyState.STATE_DISARMED
                response.message = (
                    "local fault reset against fresh hardware-clear status; "
                    "explicit ARM remains required"
                    if hardware_confirmed_clear and not hardware_fault
                    else "ESP32 and local fault reset; explicit ARM remains required"
                )
            else:
                response.success = False
                response.resulting_state = SafetyState.STATE_FAULT
                response.message = (
                    "RESET_FAULT cancelled by a newer safety transition"
                    if cancelled
                    else bridge_response.message
                    if bridge_response is not None
                    else (
                        "fresh ESP32 DISARMED/fault-clear confirmation is required"
                        if not hardware_fault
                        else "ESP32 RESET_FAULT service unavailable or timed out"
                    )
                )
        return response

    def _on_reset_estop(
        self, request: ResetEstop.Request, response: ResetEstop.Response
    ) -> ResetEstop.Response:
        del request
        with self._lock:
            self._clear_motion_authority_locked()
            reset_epoch = self._authority_epoch
            status_generation = self._status_generation
            now = self._now_s()
            status_fresh = is_fresh(
                self._status_rx_s, now, self._status_timeout_s
            )
            hardware_estop = bool(
                status_fresh
                and self._status is not None
                and (
                    self._status.estop
                    or self._status.state == RobotStatus.STATE_ESTOP
                )
            )
            hardware_confirmed_clear = self._hardware_clear_locked(estop=True)
        self._publish_zero_immediately()
        bridge_response = None
        if hardware_estop:
            bridge_response = self._call_bridge(
                self._bridge_reset_estop, ResetEstop.Request()
            )
            if (
                bridge_response is not None
                and bridge_response.success
                and bridge_response.resulting_state == RobotStatus.STATE_DISARMED
            ):
                hardware_confirmed_clear = self._wait_for_post_request_clear_status(
                    after_generation=status_generation, estop=True
                )
        with self._lock:
            cancelled = reset_epoch != self._authority_epoch
            if not cancelled and (
                hardware_confirmed_clear
                or (
                    bridge_response is not None
                    and bridge_response.success
                    and bridge_response.resulting_state == RobotStatus.STATE_DISARMED
                    and hardware_confirmed_clear
                )
            ):
                self._local_estop_latched = False
                response.success = True
                response.resulting_state = SafetyState.STATE_DISARMED
                response.message = (
                    "local ESTOP reset against fresh hardware-clear status; "
                    "explicit ARM remains required"
                    if hardware_confirmed_clear and not hardware_estop
                    else "ESP32 and local ESTOP reset; explicit ARM remains required"
                )
            else:
                response.success = False
                response.resulting_state = SafetyState.STATE_ESTOP
                response.message = (
                    "RESET_ESTOP cancelled by a newer safety transition"
                    if cancelled
                    else bridge_response.message
                    if bridge_response is not None
                    else (
                        "fresh ESP32 DISARMED/ESTOP-clear confirmation is required"
                        if not hardware_estop
                        else "ESP32 RESET_ESTOP service unavailable or timed out"
                    )
                )
        return response

    def _on_set_control_mode(
        self,
        request: SetControlMode.Request,
        response: SetControlMode.Response,
    ) -> SetControlMode.Response:
        if request.mode not in (
            SetControlMode.Request.MODE_DISARMED,
            SetControlMode.Request.MODE_MANUAL,
            SetControlMode.Request.MODE_AUTO,
        ):
            response.success = False
            response.resulting_mode = int(self._mode)
            response.message = "unknown control mode"
            return response
        if request.mode == SetControlMode.Request.MODE_DISARMED:
            with self._lock:
                self._clear_motion_authority_locked()
                self._selected_mode = Mode.MANUAL
            self._publish_zero_immediately()
            self._request_bridge_zero(disarm=True)
            response.success = True
            response.resulting_mode = int(Mode.DISARMED)
            response.message = "DISARMED selected; hardware DISARM requested"
            return response
        selected = Mode(request.mode)
        with self._lock:
            if self._operator_armed or self._arm_in_progress:
                response.success = False
                response.resulting_mode = int(self._mode)
                response.message = "DISARM before changing control mode"
                return response
            if selected == Mode.AUTO and not self._autonomous_enabled:
                response.success = False
                response.resulting_mode = int(Mode.DISARMED)
                response.message = "AUTO_NOT_READY: autonomous_enabled is false"
                return response
            self._selected_mode = selected
        response.success = True
        response.resulting_mode = int(selected)
        response.message = "mode selected while DISARMED; call /safety/arm explicitly"
        return response

    def _on_set_relays(
        self, request: SetRelays.Request, response: SetRelays.Response
    ) -> SetRelays.Response:
        with self._lock:
            conditions = self._conditions()
            prohibited = (
                not conditions.connected
                or not conditions.estop_clear
                or self._fault.active
                or not conditions.esp32_fault_free
            )
        if prohibited:
            response.success = False
            response.active_mask = 0
            response.message = "relay command blocked by Safety state"
            return response
        bridge_response = self._call_bridge(self._bridge_set_relays, request)
        if bridge_response is None:
            response.success = False
            response.active_mask = 0
            response.message = "ESP32 relay service unavailable or timed out"
        else:
            response.success = bridge_response.success
            response.active_mask = bridge_response.active_mask
            response.message = bridge_response.message
        return response

    def safe_shutdown(self) -> None:
        with self._lock:
            if self._shutting_down:
                return
            self._shutting_down = True
            self._clear_motion_authority_locked()
        self._publish_zero_immediately()
        self._request_bridge_zero(disarm=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyNode()
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.safe_shutdown()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
