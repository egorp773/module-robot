"""Guarded RTK displacement heading initializer.

The node has no ESP32, motor, ARM, or DISARM client. Its only motion output is
``/cmd_vel_manual``; module_robot_safety remains the command arbiter. Motion
requires two false-by-default parameters, an explicit ``~/start`` Trigger, and
an already operator-armed MANUAL safety state.
"""

from __future__ import annotations

from collections import deque
from enum import Enum, auto
import math
import os
import time
from typing import Deque, Optional, Sequence, Tuple

from geometry_msgs.msg import TwistStamped
from module_robot_msgs.msg import MotorStatus, RtkStatus, SafetyState
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
import yaml


EARTH_RADIUS_M = 6_378_137.0
Position = Tuple[float, float]


class HeadingState(Enum):
    DISABLED = auto()
    IDLE = auto()
    VALIDATING = auto()
    COLLECTING_START = auto()
    DRIVING = auto()
    SETTLING = auto()
    COLLECTING_END = auto()
    COMPUTING = auto()
    COMPLETE = auto()
    FAILED = auto()


class HeadingInitializer(Node):
    """Explicitly triggered, fail-closed heading initialization state machine."""

    ACTIVE_STATES = {
        HeadingState.VALIDATING,
        HeadingState.COLLECTING_START,
        HeadingState.DRIVING,
        HeadingState.SETTLING,
        HeadingState.COLLECTING_END,
        HeadingState.COMPUTING,
    }

    def __init__(self) -> None:
        super().__init__("heading_initializer")
        self._declare_parameters()

        self.enabled = bool(self.get_parameter("enabled").value)
        self.motion_enabled = bool(self.get_parameter("motion_enabled").value)
        self.target_distance_m = float(self.get_parameter("target_distance_m").value)
        self.hard_max_distance_m = float(
            self.get_parameter("hard_max_distance_m").value
        )
        self.minimum_displacement_m = float(
            self.get_parameter("minimum_displacement_m").value
        )
        self.command_speed_mps = float(self.get_parameter("command_speed_mps").value)
        self.hard_max_command_speed_mps = float(
            self.get_parameter("hard_max_command_speed_mps").value
        )
        self.max_hacc_m = float(self.get_parameter("max_horizontal_accuracy_m").value)
        self.freshness_s = float(self.get_parameter("sensor_freshness_s").value)
        self.validation_hold_s = float(self.get_parameter("validation_hold_s").value)
        self.sample_count = max(3, int(self.get_parameter("samples_per_endpoint").value))
        self.max_endpoint_jitter_m = float(
            self.get_parameter("max_endpoint_jitter_m").value
        )
        self.max_cross_track_jitter_m = float(
            self.get_parameter("max_cross_track_jitter_m").value
        )
        self.max_initial_yaw_std_rad = float(
            self.get_parameter("max_initial_yaw_std_rad").value
        )
        self.endpoint_timeout_s = float(self.get_parameter("endpoint_timeout_s").value)
        self.drive_timeout_s = float(self.get_parameter("drive_timeout_s").value)
        self.settle_time_s = float(self.get_parameter("settle_time_s").value)
        self.settle_timeout_s = float(self.get_parameter("settle_timeout_s").value)
        self.stopped_feedback_threshold = int(
            self.get_parameter("stopped_feedback_threshold").value
        )
        self.result_file = os.path.expanduser(str(self.get_parameter("result_file").value))
        self.required_imu_frame_id = str(
            self.get_parameter("required_imu_frame_id").value
        )
        self._configuration_errors = self._validate_configuration()

        self.state = HeadingState.IDLE if self.enabled else HeadingState.DISABLED
        self.failure_reason = ""
        self._state_started_s = self._now_s()
        self._validation_started_s = 0.0
        self._drive_started_s = 0.0
        self._settle_started_s = 0.0
        self._stopped_since_s: Optional[float] = None

        self._safety: Optional[SafetyState] = None
        self._rtk: Optional[RtkStatus] = None
        self._motor: Optional[MotorStatus] = None
        self._latest_fix: Optional[Position] = None
        self._latest_imu_yaw: Optional[float] = None
        self._last_safety_s = float("-inf")
        self._last_rtk_s = float("-inf")
        self._last_motor_s = float("-inf")
        self._last_fix_s = float("-inf")
        self._last_imu_s = float("-inf")
        self._fix_generation = 0
        self._consumed_fix_generation = -1

        self._start_samples: Deque[Position] = deque(maxlen=self.sample_count)
        self._end_samples: Deque[Position] = deque(maxlen=self.sample_count)
        self._drive_samples: Deque[Position] = deque(
            maxlen=max(self.sample_count * 20, 200)
        )
        self._start_yaws: Deque[float] = deque(maxlen=max(self.sample_count * 3, 20))
        self._start_mean: Optional[Position] = None

        transient = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        command_topic = str(self.get_parameter("command_topic").value)
        self._command_pub = self.create_publisher(TwistStamped, command_topic, 10)
        self._initialized_pub = self.create_publisher(
            Bool, "/heading_initialized", transient
        )
        self._state_pub = self.create_publisher(String, "~/state", transient)

        self.create_subscription(
            NavSatFix,
            str(self.get_parameter("gps_topic").value),
            self._on_fix,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            RtkStatus,
            str(self.get_parameter("rtk_topic").value),
            self._on_rtk,
            10,
        )
        self.create_subscription(
            MotorStatus,
            str(self.get_parameter("motor_topic").value),
            self._on_motor,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            SafetyState,
            str(self.get_parameter("safety_topic").value),
            self._on_safety,
            10,
        )

        self.create_service(Trigger, "~/start", self._on_start)
        self.create_service(Trigger, "~/cancel", self._on_cancel)
        command_rate_hz = max(1.0, float(self.get_parameter("command_rate_hz").value))
        self.create_timer(1.0 / command_rate_hz, self._tick)

        self._publish_initialized(False)
        self._publish_state()
        self.get_logger().warn(
            "Heading initializer is %s; enabled=%s motion_enabled=%s. "
            "It never arms the robot and never talks directly to motors."
            % (self.state.name, self.enabled, self.motion_enabled)
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "enabled": False,
            "motion_enabled": False,
            "target_distance_m": 1.0,
            "hard_max_distance_m": 2.0,
            "minimum_displacement_m": 0.80,
            "command_speed_mps": 0.05,
            "hard_max_command_speed_mps": 0.10,
            "max_horizontal_accuracy_m": 0.03,
            "sensor_freshness_s": 0.30,
            "validation_hold_s": 1.0,
            "samples_per_endpoint": 20,
            "max_endpoint_jitter_m": 0.04,
            "max_cross_track_jitter_m": 0.08,
            "max_initial_yaw_std_rad": 0.05,
            "endpoint_timeout_s": 20.0,
            "drive_timeout_s": 40.0,
            "settle_time_s": 1.0,
            "settle_timeout_s": 8.0,
            "stopped_feedback_threshold": 5,
            "command_rate_hz": 20.0,
            "result_file": "~/.ros/module_robot_heading.yaml",
            "gps_topic": "/gps/fix",
            "imu_topic": "/imu/data_raw",
            "required_imu_frame_id": "imu_link",
            "rtk_topic": "/rtk/status",
            "motor_topic": "/motor/status",
            "safety_topic": "/safety/state",
            "command_topic": "/cmd_vel_manual",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _now_s(self) -> float:
        # Sensor and safety freshness must not be extended or made negative by
        # NTP/system-clock corrections during outdoor commissioning.
        return time.monotonic()

    def _on_fix(self, msg: NavSatFix) -> None:
        if (
            msg.status.status < NavSatStatus.STATUS_FIX
            or not math.isfinite(msg.latitude)
            or not math.isfinite(msg.longitude)
            or abs(msg.latitude) > 90.0
            or abs(msg.longitude) > 180.0
        ):
            return
        self._latest_fix = (msg.latitude, msg.longitude)
        self._last_fix_s = self._now_s()
        self._fix_generation += 1

    def _on_imu(self, msg: Imu) -> None:
        if self.required_imu_frame_id and msg.header.frame_id != self.required_imu_frame_id:
            return
        covariance_diagonal = (
            msg.orientation_covariance[0],
            msg.orientation_covariance[4],
            msg.orientation_covariance[8],
        )
        if covariance_diagonal[0] < 0.0 or not all(
            math.isfinite(value) and value >= 0.0 for value in covariance_diagonal
        ):
            return
        if sum(covariance_diagonal) <= 0.0:
            return
        q = msg.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 0.5 or not math.isfinite(norm):
            return
        x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        self._latest_imu_yaw = yaw
        self._last_imu_s = self._now_s()
        if self.state == HeadingState.COLLECTING_START:
            self._start_yaws.append(yaw)

    def _on_rtk(self, msg: RtkStatus) -> None:
        self._rtk = msg
        self._last_rtk_s = self._now_s()

    def _on_motor(self, msg: MotorStatus) -> None:
        self._motor = msg
        self._last_motor_s = self._now_s()

    def _on_safety(self, msg: SafetyState) -> None:
        self._safety = msg
        self._last_safety_s = self._now_s()

    def _on_start(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if not self.enabled:
            response.success = False
            response.message = "DISABLED: set enabled:=true explicitly"
            return response
        if not self.motion_enabled:
            response.success = False
            response.message = "MOTION_DISABLED: set motion_enabled:=true explicitly"
            return response
        if self._configuration_errors:
            response.success = False
            response.message = "CONFIGURATION_INVALID:" + ",".join(
                self._configuration_errors
            )
            return response
        if self.state not in {HeadingState.IDLE, HeadingState.FAILED}:
            response.success = False
            response.message = f"INVALID_STATE:{self.state.name}"
            return response
        failures = self._precondition_failures()
        if failures:
            response.success = False
            response.message = "PRECONDITION_FAILED:" + ",".join(failures)
            return response
        if not self._motor_feedback_is_zero():
            response.success = False
            response.message = "PRECONDITION_FAILED:motor_not_physically_zero"
            return response

        self.failure_reason = ""
        self._reset_samples()
        self._validation_started_s = self._now_s()
        self._publish_initialized(False)
        self._publish_command(0.0)
        self._transition(HeadingState.VALIDATING)
        response.success = True
        response.message = "VALIDATING; robot remains stopped before endpoint sampling"
        return response

    def _on_cancel(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._publish_command(0.0)
        self._publish_initialized(False)
        self.failure_reason = "operator_cancel"
        self._transition(HeadingState.IDLE if self.enabled else HeadingState.DISABLED)
        response.success = True
        response.message = "Cancelled; zero command published"
        return response

    def _tick(self) -> None:
        if self.state not in self.ACTIVE_STATES:
            return

        failures = self._precondition_failures()
        if failures:
            self._fail("precondition_lost:" + ",".join(failures))
            return

        now = self._now_s()
        if self.state == HeadingState.VALIDATING:
            self._publish_command(0.0)
            if not self._motor_feedback_is_zero():
                self._validation_started_s = now
                return
            if now - self._validation_started_s >= self.validation_hold_s:
                self._reset_samples()
                self._transition(HeadingState.COLLECTING_START)
            return

        if self.state == HeadingState.COLLECTING_START:
            self._publish_command(0.0)
            if not self._motor_feedback_is_zero():
                self._fail("motor_moved_during_start_sampling")
                return
            self._consume_latest_fix(self._start_samples)
            stable = self._stable_mean(self._start_samples)
            yaw_stable = (
                len(self._start_yaws) >= 3
                and self._circular_std(self._start_yaws) <= self.max_initial_yaw_std_rad
            )
            if stable is not None and yaw_stable:
                self._start_mean = stable
                self._drive_started_s = now
                self._transition(HeadingState.DRIVING)
            elif now - self._state_started_s > self.endpoint_timeout_s:
                self._fail("start_endpoint_not_stable")
            return

        if self.state == HeadingState.DRIVING:
            assert self._start_mean is not None
            self._consume_latest_fix(self._drive_samples)
            if self._latest_fix is not None:
                east, north = self._enu_delta(self._start_mean, self._latest_fix)
                if math.hypot(east, north) >= self.target_distance_m:
                    self._publish_command(0.0)
                    self._settle_started_s = now
                    self._stopped_since_s = None
                    self._transition(HeadingState.SETTLING)
                    return
            if now - self._drive_started_s > self.drive_timeout_s:
                self._fail("drive_timeout")
                return
            self._publish_command(self.command_speed_mps)
            return

        if self.state == HeadingState.SETTLING:
            self._publish_command(0.0)
            if self._motor_feedback_is_zero():
                if self._stopped_since_s is None:
                    self._stopped_since_s = now
                elif now - self._stopped_since_s >= self.settle_time_s:
                    self._end_samples.clear()
                    self._consumed_fix_generation = self._fix_generation
                    self._transition(HeadingState.COLLECTING_END)
            else:
                self._stopped_since_s = None
            if now - self._settle_started_s > self.settle_timeout_s:
                self._fail("motor_feedback_did_not_settle")
            return

        if self.state == HeadingState.COLLECTING_END:
            self._publish_command(0.0)
            if not self._motor_feedback_is_zero():
                self._fail("motor_moved_during_end_sampling")
                return
            self._consume_latest_fix(self._end_samples)
            stable = self._stable_mean(self._end_samples)
            if stable is not None:
                self._transition(HeadingState.COMPUTING)
                self._compute_and_save(stable)
            elif now - self._state_started_s > self.endpoint_timeout_s:
                self._fail("end_endpoint_not_stable")

    def _precondition_failures(self) -> list[str]:
        now = self._now_s()
        failures = []
        if self._safety is None or now - self._last_safety_s > self.freshness_s:
            failures.append("safety_stale")
        elif (
            self._safety.state != SafetyState.STATE_MANUAL
            or not self._safety.operator_armed
            or self._safety.latched_fault
        ):
            failures.append("manual_not_operator_armed")

        if self._motor is None or now - self._last_motor_s > self.freshness_s:
            failures.append("motor_feedback_stale")
        elif self._motor.controller_fault != 0:
            failures.append("motor_fault")

        if self._rtk is None or now - self._last_rtk_s > self.freshness_s:
            failures.append("rtk_stale")
        elif self._rtk.carrier_solution != RtkStatus.CARRIER_FIXED:
            failures.append("rtk_not_fixed")
        elif (
            not math.isfinite(self._rtk.horizontal_accuracy_m)
            or self._rtk.horizontal_accuracy_m > self.max_hacc_m
        ):
            failures.append("rtk_accuracy")

        if self._latest_fix is None or now - self._last_fix_s > self.freshness_s:
            failures.append("gnss_stale")
        if self._latest_imu_yaw is None or now - self._last_imu_s > self.freshness_s:
            failures.append("imu_stale_or_orientation_invalid")
        return failures

    def _consume_latest_fix(self, target: Deque[Position]) -> None:
        if (
            self._latest_fix is not None
            and self._fix_generation != self._consumed_fix_generation
        ):
            target.append(self._latest_fix)
            self._consumed_fix_generation = self._fix_generation

    def _stable_mean(self, samples: Sequence[Position]) -> Optional[Position]:
        if len(samples) < self.sample_count:
            return None
        mean = self._mean_position(samples)
        radial_errors = [math.hypot(*self._enu_delta(mean, point)) for point in samples]
        rms = math.sqrt(sum(value * value for value in radial_errors) / len(radial_errors))
        return mean if rms <= self.max_endpoint_jitter_m else None

    def _compute_and_save(self, end_mean: Position) -> None:
        assert self._start_mean is not None
        east, north = self._enu_delta(self._start_mean, end_mean)
        displacement = math.hypot(east, north)
        if displacement < self.minimum_displacement_m:
            self._fail(f"displacement_too_short:{displacement:.3f}")
            return
        if displacement > self.hard_max_distance_m:
            self._fail(f"displacement_exceeds_hard_limit:{displacement:.3f}")
            return

        course_enu = math.atan2(north, east)
        cross_track_rms = self._cross_track_rms(course_enu)
        if cross_track_rms > self.max_cross_track_jitter_m:
            self._fail(f"cross_track_jitter:{cross_track_rms:.3f}")
            return

        imu_yaw = self._circular_mean(self._start_yaws)
        yaw_offset = self._normalize_angle(course_enu - imu_yaw)
        result = {
            "version": 1,
            "yaw_convention": "REP-103 ENU: zero east, positive counter-clockwise",
            "correction_equation": "yaw_enu = normalize(imu_yaw + imu_yaw_offset)",
            "course_enu_rad": course_enu,
            "imu_start_yaw_rad": imu_yaw,
            "imu_frame_id": self.required_imu_frame_id,
            "imu_yaw_offset_rad": yaw_offset,
            "quality": {
                "displacement_m": displacement,
                "cross_track_rms_m": cross_track_rms,
                "start_samples": len(self._start_samples),
                "end_samples": len(self._end_samples),
                "drive_samples": len(self._drive_samples),
                "rtk_horizontal_accuracy_m": float(self._rtk.horizontal_accuracy_m),
                "initial_imu_yaw_std_rad": self._circular_std(self._start_yaws),
            },
            "start": {"latitude": self._start_mean[0], "longitude": self._start_mean[1]},
            "end": {"latitude": end_mean[0], "longitude": end_mean[1]},
            "generated_at_ros_time_ns": self.get_clock().now().nanoseconds,
        }
        try:
            directory = os.path.dirname(self.result_file)
            if directory:
                os.makedirs(directory, exist_ok=True)
            with open(self.result_file, "w", encoding="utf-8") as stream:
                yaml.safe_dump(result, stream, sort_keys=False)
        except (OSError, yaml.YAMLError) as exc:
            self._fail(f"result_write_failed:{exc}")
            return

        self._publish_command(0.0)
        self._publish_initialized(True)
        self._transition(HeadingState.COMPLETE)
        self.get_logger().info(
            f"Heading initialized: course={course_enu:.6f} rad, "
            f"offset={yaw_offset:.6f} rad, result={self.result_file}"
        )

    def _motor_feedback_is_zero(self) -> bool:
        return bool(
            self._motor is not None
            and abs(self._motor.left_feedback) <= self.stopped_feedback_threshold
            and abs(self._motor.right_feedback) <= self.stopped_feedback_threshold
        )

    def _cross_track_rms(self, course_enu: float) -> float:
        assert self._start_mean is not None
        values = []
        for point in (
            *self._start_samples,
            *self._drive_samples,
            *self._end_samples,
        ):
            east, north = self._enu_delta(self._start_mean, point)
            values.append(-math.sin(course_enu) * east + math.cos(course_enu) * north)
        return math.sqrt(sum(value * value for value in values) / len(values))

    def _reset_samples(self) -> None:
        self._start_samples.clear()
        self._end_samples.clear()
        self._drive_samples.clear()
        self._start_yaws.clear()
        self._start_mean = None
        self._consumed_fix_generation = self._fix_generation

    def _validate_configuration(self) -> list[str]:
        errors = []
        if not 0.0 < self.command_speed_mps <= self.hard_max_command_speed_mps:
            errors.append("command_speed_outside_hard_limit")
        if not 0.0 < self.minimum_displacement_m <= self.target_distance_m:
            errors.append("minimum_displacement_invalid")
        if not 0.0 < self.target_distance_m <= self.hard_max_distance_m:
            errors.append("target_distance_outside_hard_limit")
        if self.max_hacc_m <= 0.0 or self.freshness_s <= 0.0:
            errors.append("freshness_or_accuracy_invalid")
        return errors

    def _fail(self, reason: str) -> None:
        self.failure_reason = reason
        self._publish_command(0.0)
        self._publish_initialized(False)
        self._transition(HeadingState.FAILED)
        self.get_logger().error(f"Heading initialization failed: {reason}")

    def _transition(self, new_state: HeadingState) -> None:
        if self.state == new_state:
            return
        old_state = self.state
        self.state = new_state
        self._state_started_s = self._now_s()
        self._publish_state()
        self.get_logger().info(f"Heading state {old_state.name} -> {new_state.name}")

    def _publish_command(self, linear_x: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = linear_x
        self._command_pub.publish(msg)

    def _publish_initialized(self, initialized: bool) -> None:
        self._initialized_pub.publish(Bool(data=initialized))

    def _publish_state(self) -> None:
        detail = self.state.name
        if self.failure_reason:
            detail += ":" + self.failure_reason
        self._state_pub.publish(String(data=detail))

    @staticmethod
    def _mean_position(samples: Sequence[Position]) -> Position:
        return (
            sum(point[0] for point in samples) / len(samples),
            sum(point[1] for point in samples) / len(samples),
        )

    @staticmethod
    def _enu_delta(origin: Position, point: Position) -> Tuple[float, float]:
        origin_lat_rad = math.radians(origin[0])
        point_lat_rad = math.radians(point[0])
        delta_lat = point_lat_rad - origin_lat_rad
        delta_lon = math.radians(point[1] - origin[1])
        delta_lon = (delta_lon + math.pi) % (2.0 * math.pi) - math.pi
        east = EARTH_RADIUS_M * delta_lon * math.cos((origin_lat_rad + point_lat_rad) / 2.0)
        north = EARTH_RADIUS_M * delta_lat
        return east, north

    @staticmethod
    def _circular_mean(values: Sequence[float]) -> float:
        return math.atan2(
            sum(math.sin(value) for value in values),
            sum(math.cos(value) for value in values),
        )

    @staticmethod
    def _circular_std(values: Sequence[float]) -> float:
        mean_sin = sum(math.sin(value) for value in values) / len(values)
        mean_cos = sum(math.cos(value) for value in values) / len(values)
        resultant = min(1.0, max(1.0e-12, math.hypot(mean_sin, mean_cos)))
        return math.sqrt(-2.0 * math.log(resultant))

    @staticmethod
    def _normalize_angle(value: float) -> float:
        return math.atan2(math.sin(value), math.cos(value))

    def destroy_node(self) -> bool:
        if self.state in self.ACTIVE_STATES:
            self._publish_command(0.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HeadingInitializer()
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
