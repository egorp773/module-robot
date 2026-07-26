#!/usr/bin/env python3
"""Run one bounded, lifted-track manual commissioning pulse.

The utility is deliberately limited to MANUAL ARM and one of four low-speed
motions.  It never calls relay, fault-reset, or ESTOP-reset services.  Every
exit path attempts a zero command followed by STOP and DISARM, then emits one
JSON summary on stdout; progress messages go to stderr.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
import secrets
import signal
import sys
import time
from typing import Callable, Optional

from geometry_msgs.msg import TwistStamped
from module_robot_msgs.msg import MotorStatus, RelayStatus, RobotStatus
from module_robot_msgs.srv import Arm, Disarm
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger


PUBLISH_RATE_HZ = 50.0
MAX_LINEAR_M_S = 0.05
MAX_ANGULAR_RAD_S = 0.25
MAX_DURATION_S = 3.0
DEFAULT_LINEAR_M_S = 0.03
DEFAULT_ANGULAR_RAD_S = 0.15
TELEMETRY_TIMEOUT_S = 5.0
SERVICE_TIMEOUT_S = 5.0
ARM_CONFIRM_TIMEOUT_S = 3.0
FINAL_EVIDENCE_TIMEOUT_S = 3.0
CLEANUP_MAX_ATTEMPTS = 3
CLEANUP_RETRY_DELAY_S = 0.15
MAX_STATUS_LOCAL_AGE_S = 0.6
MAX_MOTOR_LOCAL_AGE_S = 0.6
MAX_RELAY_LOCAL_AGE_S = 2.0
MAX_REPORTED_MOTOR_AGE_MS = 500
LIFTED_CONFIRMATION = "I_HAVE_LIFTED_THE_ROBOT"


@dataclass(frozen=True)
class PulseCommand:
    mode: str
    linear_x: float
    angular_z: float
    duration_s: float


@dataclass(frozen=True)
class TimedSample:
    received_s: float
    phase: str
    message: object


def command_for(mode: str, speed: Optional[float], duration_s: float) -> PulseCommand:
    """Validate CLI values and turn a mode plus magnitude into a command."""

    if mode not in ("forward", "reverse", "left", "right"):
        raise ValueError(f"unsupported pulse mode: {mode}")
    if not math.isfinite(duration_s) or not 0.0 < duration_s <= MAX_DURATION_S:
        raise ValueError(f"duration must be in (0, {MAX_DURATION_S:g}] seconds")

    linear_mode = mode in ("forward", "reverse")
    magnitude = (
        DEFAULT_LINEAR_M_S if linear_mode and speed is None
        else DEFAULT_ANGULAR_RAD_S if speed is None
        else speed
    )
    if not math.isfinite(magnitude) or magnitude <= 0.0:
        raise ValueError("speed must be a finite positive magnitude")

    if linear_mode:
        if magnitude > MAX_LINEAR_M_S:
            raise ValueError(
                f"linear speed must not exceed {MAX_LINEAR_M_S:g} m/s"
            )
        sign = 1.0 if mode == "forward" else -1.0
        return PulseCommand(mode, sign * magnitude, 0.0, duration_s)

    if magnitude > MAX_ANGULAR_RAD_S:
        raise ValueError(
            f"angular speed must not exceed {MAX_ANGULAR_RAD_S:g} rad/s"
        )
    sign = 1.0 if mode == "left" else -1.0
    return PulseCommand(mode, 0.0, sign * magnitude, duration_s)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run exactly one low-speed pulse with both tracks lifted."
    )
    parser.add_argument("mode", choices=("forward", "reverse", "left", "right"))
    parser.add_argument(
        "--speed",
        type=float,
        default=None,
        help=(
            "positive magnitude; m/s for forward/reverse (default 0.03, max 0.05), "
            "rad/s for left/right (default 0.15, max 0.25)"
        ),
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=1.0,
        help="pulse duration in seconds, in (0, 3] (default: 1)",
    )
    parser.add_argument(
        "--confirm-lifted",
        required=True,
        metavar="TOKEN",
        help=f"must be exactly {LIFTED_CONFIRMATION}",
    )
    return parser


class CommissioningNode(Node):
    """ROS endpoint owner and timestamped typed-evidence collector."""

    def __init__(self) -> None:
        super().__init__("manual_calibration_test")
        self.phase = "startup"
        self.status_samples: list[TimedSample] = []
        self.motor_samples: list[TimedSample] = []
        self.relay_samples: list[TimedSample] = []

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
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._command_publisher = self.create_publisher(
            TwistStamped, "/cmd_vel_manual", command_qos
        )
        # Do not assign to Node._subscriptions; rclpy owns that internal list.
        self._evidence_subscriptions = (
            self.create_subscription(
                RobotStatus, "/esp32/status", self._on_status, state_qos
            ),
            self.create_subscription(
                MotorStatus, "/motor/status", self._on_motor, sensor_qos
            ),
            self.create_subscription(
                RelayStatus, "/relay/status", self._on_relay, state_qos
            ),
        )
        self.arm_client = self.create_client(Arm, "/safety/arm")
        self.stop_client = self.create_client(Trigger, "/safety/stop")
        self.disarm_client = self.create_client(Disarm, "/safety/disarm")

    def _on_status(self, message: RobotStatus) -> None:
        self.status_samples.append(TimedSample(time.monotonic(), self.phase, message))

    def _on_motor(self, message: MotorStatus) -> None:
        self.motor_samples.append(TimedSample(time.monotonic(), self.phase, message))

    def _on_relay(self, message: RelayStatus) -> None:
        self.relay_samples.append(TimedSample(time.monotonic(), self.phase, message))

    def publish_command(self, linear_x: float, angular_z: float) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.twist.linear.x = float(linear_x)
        message.twist.angular.z = float(angular_z)
        self._command_publisher.publish(message)

    def spin_once(self, timeout_s: float) -> None:
        rclpy.spin_once(self, timeout_sec=max(0.0, timeout_s))

    def wait_until(self, predicate: Callable[[], bool], timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while not predicate():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            self.spin_once(min(0.05, remaining))
        return True

    def call_service(self, client, request, service_name: str):
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_S):
            raise RuntimeError(f"service unavailable: {service_name}")
        future = client.call_async(request)
        deadline = time.monotonic() + SERVICE_TIMEOUT_S
        while not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise RuntimeError(f"service timed out: {service_name}")
            self.spin_once(min(0.05, remaining))
        response = future.result()
        if response is None:
            raise RuntimeError(f"service returned no response: {service_name}")
        return response

    def latest_status(self) -> TimedSample:
        if not self.status_samples:
            raise RuntimeError("no /esp32/status telemetry")
        return self.status_samples[-1]

    def latest_motor(self) -> TimedSample:
        if not self.motor_samples:
            raise RuntimeError("no /motor/status telemetry")
        return self.motor_samples[-1]

    def latest_relay(self) -> TimedSample:
        if not self.relay_samples:
            raise RuntimeError("no /relay/status telemetry")
        return self.relay_samples[-1]


def _log(message: str) -> None:
    try:
        print(f"[manual_calibration_test] {message}", file=sys.stderr, flush=True)
    except (BrokenPipeError, OSError):
        # Zero/STOP/DISARM cleanup must survive a lost SSH terminal.
        pass


def _response_dict(response: object) -> dict[str, object]:
    result: dict[str, object] = {"received": True}
    for name in ("success", "resulting_state", "message"):
        if hasattr(response, name):
            result[name] = getattr(response, name)
    return result


def _sample_counts(node: CommissioningNode) -> tuple[int, int, int]:
    return (
        len(node.status_samples),
        len(node.motor_samples),
        len(node.relay_samples),
    )


def _all_counts_advanced(
    node: CommissioningNode, before: tuple[int, int, int]
) -> bool:
    now = _sample_counts(node)
    return all(current > old for current, old in zip(now, before))


def _require_telemetry_set(node: CommissioningNode, timeout_s: float) -> None:
    if not node.wait_until(lambda: all(_sample_counts(node)), timeout_s):
        counts = _sample_counts(node)
        raise RuntimeError(
            "typed telemetry incomplete: "
            f"status={counts[0]}, motor={counts[1]}, relay={counts[2]}"
        )


def _require_zero_outputs(status: RobotStatus, context: str) -> None:
    nonzero = {
        name: int(getattr(status, name))
        for name in (
            "applied_left_command",
            "applied_right_command",
            "uart_speed",
            "uart_steer",
        )
        if int(getattr(status, name)) != 0
    }
    if nonzero:
        raise RuntimeError(f"non-zero motor output {context}: {nonzero}")


def _require_live_health(node: CommissioningNode, *, armed: bool) -> None:
    now = time.monotonic()
    status_sample = node.latest_status()
    motor_sample = node.latest_motor()
    relay_sample = node.latest_relay()
    status = status_sample.message
    motor = motor_sample.message
    relay = relay_sample.message

    if now - status_sample.received_s > MAX_STATUS_LOCAL_AGE_S:
        raise RuntimeError("/esp32/status became locally stale")
    if now - motor_sample.received_s > MAX_MOTOR_LOCAL_AGE_S:
        raise RuntimeError("/motor/status became locally stale")
    if now - relay_sample.received_s > MAX_RELAY_LOCAL_AGE_S:
        raise RuntimeError("/relay/status became locally stale")
    if not status.connected:
        raise RuntimeError("ESP32 disconnected")
    if status.estop or status.state == RobotStatus.STATE_ESTOP:
        raise RuntimeError("ESTOP is active")
    if status.fault_code != 0 or status.state == RobotStatus.STATE_FAULT:
        raise RuntimeError(f"ESP32 fault is active: {int(status.fault_code)}")
    if status.last_motor_feedback_age_ms > MAX_REPORTED_MOTOR_AGE_MS:
        raise RuntimeError(
            "motor feedback is stale: "
            f"{int(status.last_motor_feedback_age_ms)} ms"
        )
    if motor.controller_fault != 0:
        raise RuntimeError(
            f"motor controller fault is active: {int(motor.controller_fault)}"
        )
    if not math.isfinite(float(motor.battery_voltage)):
        raise RuntimeError("motor battery voltage is not finite")
    if not motor.board_temperature_available or not math.isfinite(
        float(motor.board_temperature_c)
    ):
        raise RuntimeError("motor controller temperature is unavailable or not finite")
    if relay.active_mask != 0:
        raise RuntimeError(f"relay active mask is non-zero: {int(relay.active_mask)}")

    if armed:
        if not status.armed or status.state != RobotStatus.STATE_ARMED:
            raise RuntimeError("ESP32 left ARMED state during the pulse")
    elif status.armed or status.state != RobotStatus.STATE_DISARMED:
        raise RuntimeError("ESP32 is not DISARMED before ARM")
    if not armed:
        _require_zero_outputs(status, "while DISARMED")


def _publish_pulse(node: CommissioningNode, command: PulseCommand) -> dict[str, object]:
    interval_s = 1.0 / PUBLISH_RATE_HZ
    started_s = time.monotonic()
    deadline_s = started_s + command.duration_s
    next_publish_s = started_s
    publish_times: list[float] = []

    while True:
        now = time.monotonic()
        if now >= deadline_s:
            break
        if now < next_publish_s:
            node.spin_once(min(next_publish_s - now, deadline_s - now))
            _require_live_health(node, armed=True)
            continue

        _require_live_health(node, armed=True)
        node.publish_command(command.linear_x, command.angular_z)
        publish_times.append(time.monotonic())
        next_publish_s += interval_s
        # Never emit a catch-up burst after scheduler delay.
        if next_publish_s < time.monotonic():
            next_publish_s = time.monotonic() + interval_s

    finished_s = time.monotonic()
    intervals = [
        later - earlier for earlier, later in zip(publish_times, publish_times[1:])
    ]
    return {
        "completed": True,
        "actual_duration_s": round(finished_s - started_s, 6),
        "publish_count": len(publish_times),
        "mean_publish_rate_hz": (
            round((len(publish_times) - 1) / (publish_times[-1] - publish_times[0]), 3)
            if len(publish_times) >= 2 and publish_times[-1] > publish_times[0]
            else None
        ),
        "max_publish_interval_s": (
            round(max(intervals), 6) if intervals else None
        ),
    }


def _zero_burst(node: CommissioningNode) -> None:
    interval_s = 1.0 / PUBLISH_RATE_HZ
    for _ in range(5):
        node.publish_command(0.0, 0.0)
        node.spin_once(interval_s)


def _cleanup_service_with_retries(
    node: CommissioningNode,
    client,
    request_factory: Callable[[], object],
    service_name: str,
) -> tuple[dict[str, object], list[str]]:
    """Call a zero-only cleanup service up to the fixed safety bound."""

    attempts: list[dict[str, object]] = []
    retry_zero_errors: list[str] = []
    result: dict[str, object] = {
        "success": False,
        "received": False,
        "successful_attempt": None,
        "attempts": attempts,
    }
    for attempt_number in range(1, CLEANUP_MAX_ATTEMPTS + 1):
        attempt: dict[str, object] = {"attempt": attempt_number}
        try:
            response = node.call_service(
                client, request_factory(), service_name
            )
            attempt.update(_response_dict(response))
            if response.success:
                result.update(_response_dict(response))
                result["successful_attempt"] = attempt_number
                attempts.append(attempt)
                break
        except BaseException as exc:
            attempt.update({"received": False, "error": str(exc)})
        attempts.append(attempt)

        if attempt_number < CLEANUP_MAX_ATTEMPTS:
            try:
                _zero_burst(node)
                attempt["retry_zero_published"] = True
            except BaseException as exc:
                attempt["retry_zero_published"] = False
                attempt["retry_zero_error"] = str(exc)
                retry_zero_errors.append(str(exc))
            attempt["retry_delay_s"] = CLEANUP_RETRY_DELAY_S
            time.sleep(CLEANUP_RETRY_DELAY_S)

    result["attempt_count"] = len(attempts)
    result["received"] = any(
        attempt.get("received") is True for attempt in attempts
    )
    return result, retry_zero_errors


def _messages_for_phase(samples: list[TimedSample], phase: str) -> list[object]:
    return [sample.message for sample in samples if sample.phase == phase]


def _numeric_range(messages: list[object], field: str) -> Optional[list[object]]:
    values = [getattr(message, field) for message in messages]
    if not values:
        return None
    if any(isinstance(value, float) and not math.isfinite(value) for value in values):
        finite = [float(value) for value in values if math.isfinite(float(value))]
        return [min(finite), max(finite)] if finite else None
    return [min(values), max(values)]


def _numeric_stats(messages: list[object], field: str) -> Optional[dict[str, object]]:
    values = [float(getattr(message, field)) for message in messages]
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return None
    return {
        "min": min(finite),
        "max": max(finite),
        "mean": sum(finite) / len(finite),
        "mean_abs": sum(abs(value) for value in finite) / len(finite),
        "nonzero_count": sum(abs(value) > 0.0 for value in finite),
        "sample_count": len(finite),
    }


def _latest_snapshot(node: CommissioningNode) -> Optional[dict[str, object]]:
    if not node.status_samples or not node.motor_samples or not node.relay_samples:
        return None
    status = node.status_samples[-1].message
    motor = node.motor_samples[-1].message
    relay = node.relay_samples[-1].message
    return {
        "state": int(status.state),
        "connected": bool(status.connected),
        "armed": bool(status.armed),
        "estop": bool(status.estop),
        "fault_code": int(status.fault_code),
        "last_motor_feedback_age_ms": int(status.last_motor_feedback_age_ms),
        "last_heartbeat_age_ms": int(status.last_heartbeat_age_ms),
        "applied_left_command": int(status.applied_left_command),
        "applied_right_command": int(status.applied_right_command),
        "uart_speed": int(status.uart_speed),
        "uart_steer": int(status.uart_steer),
        "watchdog_trips": int(status.watchdog_trips),
        "left_feedback": int(motor.left_feedback),
        "right_feedback": int(motor.right_feedback),
        "battery_voltage": (
            float(motor.battery_voltage)
            if math.isfinite(float(motor.battery_voltage))
            else None
        ),
        "board_temperature_c": (
            float(motor.board_temperature_c)
            if math.isfinite(float(motor.board_temperature_c))
            else None
        ),
        "board_temperature_available": bool(motor.board_temperature_available),
        "controller_fault": int(motor.controller_fault),
        "uart_valid_frames": int(motor.uart_valid_frames),
        "relay_active_mask": int(relay.active_mask),
    }


def _logical_status_matches_mode(message: RobotStatus, mode: str) -> bool:
    left = int(message.applied_left_command)
    right = int(message.applied_right_command)
    speed = int(message.uart_speed)
    steer = int(message.uart_steer)
    if left == 0 and right == 0 and speed == 0 and steer == 0:
        return True
    if mode == "forward":
        return left == right and left > 0 and speed > 0 and steer == 0
    if mode == "reverse":
        return left == right and left < 0 and speed < 0 and steer == 0
    if mode == "left":
        return left == -right and left < 0 < right and speed == 0 and steer > 0
    if mode == "right":
        return left == -right and right < 0 < left and speed == 0 and steer < 0
    return False


def _evidence_summary(
    node: CommissioningNode, command: PulseCommand
) -> dict[str, object]:
    motion_status = _messages_for_phase(node.status_samples, "pulse")
    motion_motor_samples = [
        sample for sample in node.motor_samples if sample.phase == "pulse"
    ]
    motion_motor = [sample.message for sample in motion_motor_samples]
    all_status = [sample.message for sample in node.status_samples]
    all_motor = [sample.message for sample in node.motor_samples]
    all_relay = [sample.message for sample in node.relay_samples]
    valid_frames = [int(message.uart_valid_frames) for message in motion_motor]
    motor_receive_times = [sample.received_s for sample in motion_motor_samples]
    motor_receive_gaps_ms = [
        (later - earlier) * 1000.0
        for earlier, later in zip(motor_receive_times, motor_receive_times[1:])
    ]
    logical_active_status = [
        message
        for message in motion_status
        if any(
            int(getattr(message, field)) != 0
            for field in (
                "applied_left_command",
                "applied_right_command",
                "uart_speed",
                "uart_steer",
            )
        )
    ]
    logical_mismatch_count = sum(
        not _logical_status_matches_mode(message, command.mode)
        for message in motion_status
    )

    phase_counts: dict[str, dict[str, int]] = {}
    for phase in ("preflight", "arm_wait", "pulse", "cleanup"):
        phase_counts[phase] = {
            "status": sum(sample.phase == phase for sample in node.status_samples),
            "motor": sum(sample.phase == phase for sample in node.motor_samples),
            "relay": sum(sample.phase == phase for sample in node.relay_samples),
        }

    return {
        "sample_counts": {
            "status": len(all_status),
            "motor": len(all_motor),
            "relay": len(all_relay),
            "by_phase": phase_counts,
        },
        "observed_states": sorted({int(message.state) for message in all_status}),
        "observed_fault_codes": sorted(
            {int(message.fault_code) for message in all_status}
        ),
        "observed_controller_faults": sorted(
            {int(message.controller_fault) for message in all_motor}
        ),
        "observed_relay_masks": sorted(
            {int(message.active_mask) for message in all_relay}
        ),
        "estop_seen": any(bool(message.estop) for message in all_status),
        "motion": {
            "status_samples": len(motion_status),
            "motor_samples": len(motion_motor),
            "logical_active_status_samples": len(logical_active_status),
            "logical_command_mismatch_count": logical_mismatch_count,
            "logical_command_contract_ok": bool(
                logical_active_status and logical_mismatch_count == 0
            ),
            "motor_local_max_gap_ms": (
                max(motor_receive_gaps_ms) if motor_receive_gaps_ms else None
            ),
            "applied_left_range": _numeric_range(
                motion_status, "applied_left_command"
            ),
            "applied_right_range": _numeric_range(
                motion_status, "applied_right_command"
            ),
            "uart_speed_range": _numeric_range(motion_status, "uart_speed"),
            "uart_steer_range": _numeric_range(motion_status, "uart_steer"),
            "heartbeat_age_ms_range": _numeric_range(
                motion_status, "last_heartbeat_age_ms"
            ),
            "left_feedback_range": _numeric_range(motion_motor, "left_feedback"),
            "right_feedback_range": _numeric_range(motion_motor, "right_feedback"),
            "applied_left_stats": _numeric_stats(
                motion_status, "applied_left_command"
            ),
            "applied_right_stats": _numeric_stats(
                motion_status, "applied_right_command"
            ),
            "left_feedback_stats": _numeric_stats(
                motion_motor, "left_feedback"
            ),
            "right_feedback_stats": _numeric_stats(
                motion_motor, "right_feedback"
            ),
            "battery_voltage_range": _numeric_range(
                motion_motor, "battery_voltage"
            ),
            "board_temperature_c_range": _numeric_range(
                motion_motor, "board_temperature_c"
            ),
            "uart_valid_frame_delta": (
                max(valid_frames) - min(valid_frames) if valid_frames else None
            ),
        },
        "final": _latest_snapshot(node),
    }


def _final_is_safe(summary: dict[str, object]) -> bool:
    final = summary.get("final")
    if not isinstance(final, dict):
        return False
    return bool(
        final.get("state") == RobotStatus.STATE_DISARMED
        and final.get("connected") is True
        and final.get("armed") is False
        and final.get("estop") is False
        and final.get("fault_code") == 0
        and final.get("controller_fault") == 0
        and final.get("applied_left_command") == 0
        and final.get("applied_right_command") == 0
        and final.get("uart_speed") == 0
        and final.get("uart_steer") == 0
        and final.get("relay_active_mask") == 0
        and isinstance(final.get("last_motor_feedback_age_ms"), int)
        and final["last_motor_feedback_age_ms"] <= MAX_REPORTED_MOTOR_AGE_MS
        and final.get("battery_voltage") is not None
        and final.get("board_temperature_available") is True
        and final.get("board_temperature_c") is not None
    )


def _range_has_nonzero(value: object) -> bool:
    return bool(
        isinstance(value, list)
        and len(value) == 2
        and any(abs(float(item)) > 0.0 for item in value)
    )


def _motion_evidence_is_present(
    summary: dict[str, object], command: PulseCommand
) -> bool:
    motion = summary.get("motion")
    if not isinstance(motion, dict):
        return False
    required_uart_range = (
        motion.get("uart_speed_range")
        if command.mode in ("forward", "reverse")
        else motion.get("uart_steer_range")
    )
    return bool(
        int(motion.get("status_samples", 0)) > 0
        and int(motion.get("motor_samples", 0)) > 1
        and _range_has_nonzero(motion.get("applied_left_range"))
        and _range_has_nonzero(motion.get("applied_right_range"))
        and _range_has_nonzero(motion.get("left_feedback_range"))
        and _range_has_nonzero(motion.get("right_feedback_range"))
        and _range_has_nonzero(required_uart_range)
        and motion.get("logical_command_contract_ok") is True
        and isinstance(motion.get("uart_valid_frame_delta"), int)
        and motion["uart_valid_frame_delta"] > 0
    )


def _evidence_stayed_clean(summary: dict[str, object]) -> bool:
    return bool(
        summary.get("observed_fault_codes") == [0]
        and summary.get("observed_controller_faults") == [0]
        and summary.get("observed_relay_masks") == [0]
        and summary.get("estop_seen") is False
    )


def _pulse_timing_is_acceptable(
    pulse: object, command: PulseCommand
) -> bool:
    if not isinstance(pulse, dict) or pulse.get("completed") is not True:
        return False
    count = int(pulse.get("publish_count", 0))
    minimum_count = max(1, math.floor(command.duration_s * PUBLISH_RATE_HZ * 0.8))
    if count < minimum_count:
        return False
    if count < 2:
        return command.duration_s <= 1.0 / PUBLISH_RATE_HZ
    mean_rate = pulse.get("mean_publish_rate_hz")
    maximum_interval = pulse.get("max_publish_interval_s")
    return bool(
        isinstance(mean_rate, (int, float))
        and 40.0 <= float(mean_rate) <= 60.0
        and isinstance(maximum_interval, (int, float))
        and float(maximum_interval) <= 0.1
    )


def run(command: PulseCommand) -> tuple[dict[str, object], int]:
    summary: dict[str, object] = {
        "schema_version": 1,
        "tool": "manual_calibration_test",
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "requested": {
            "mode": command.mode,
            "linear_x_m_s": command.linear_x,
            "angular_z_rad_s": command.angular_z,
            "duration_s": command.duration_s,
            "publish_rate_hz": PUBLISH_RATE_HZ,
        },
        "services": {},
        "pulse": {"completed": False, "publish_count": 0},
        "errors": [],
    }
    errors = summary["errors"]
    services = summary["services"]
    assert isinstance(errors, list)
    assert isinstance(services, dict)

    initialized = False
    node: Optional[CommissioningNode] = None
    pulse_completed = False
    final_evidence_fresh = False
    interrupted_by: Optional[int] = None

    def _interrupt(signum, _frame) -> None:
        nonlocal interrupted_by
        interrupted_by = signum
        raise InterruptedError(f"received signal {signum}")

    try:
        # Keep the ROS context alive on terminal signals so our own handler can
        # unwind into the mandatory zero -> STOP -> DISARM cleanup sequence.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        signal.signal(signal.SIGINT, _interrupt)
        signal.signal(signal.SIGHUP, _interrupt)
        signal.signal(signal.SIGTERM, _interrupt)
        node = CommissioningNode()
        node.phase = "preflight"
        _log("waiting for typed RobotStatus, MotorStatus, and RelayStatus")
        _require_telemetry_set(node, TELEMETRY_TIMEOUT_S)
        _require_live_health(node, armed=False)

        arm_request = Arm.Request()
        arm_request.arm_nonce = secrets.randbits(32) or 1
        arm_request.requested_mode = Arm.Request.MODE_MANUAL
        status_before_arm = len(node.status_samples)
        node.phase = "arm_wait"
        _log("requesting MANUAL ARM")
        arm_response = node.call_service(
            node.arm_client, arm_request, "/safety/arm"
        )
        services["arm"] = _response_dict(arm_response)
        if not arm_response.success:
            raise RuntimeError(f"MANUAL ARM rejected: {arm_response.message}")
        if not node.wait_until(
            lambda: (
                len(node.status_samples) > status_before_arm
                and bool(node.status_samples[-1].message.armed)
                and node.status_samples[-1].message.state == RobotStatus.STATE_ARMED
            ),
            ARM_CONFIRM_TIMEOUT_S,
        ):
            raise RuntimeError("no fresh ESP32 ARMED confirmation")
        _require_live_health(node, armed=True)
        _require_zero_outputs(
            node.latest_status().message, "after ARM and before the first pulse"
        )

        node.phase = "pulse"
        _log(
            f"publishing {command.mode}: linear={command.linear_x:.3f} m/s, "
            f"angular={command.angular_z:.3f} rad/s for {command.duration_s:.3f} s"
        )
        summary["pulse"] = _publish_pulse(node, command)
        pulse_completed = True
    except (KeyboardInterrupt, InterruptedError) as exc:
        errors.append(f"interrupted: {exc}")
    except BaseException as exc:  # cleanup is required even for unexpected failures
        errors.append(f"{type(exc).__name__}: {exc}")
    finally:
        # A second terminal signal must not interrupt zero/STOP/DISARM cleanup.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            node.phase = "cleanup"
            _log("cleanup: publishing zero, then STOP, then DISARM")
            try:
                _zero_burst(node)
                services["zero"] = {"published": True, "count": 5}
            except BaseException as exc:
                services["zero"] = {"published": False, "error": str(exc)}
                errors.append(f"cleanup zero failed: {exc}")

            stop_result, stop_zero_errors = _cleanup_service_with_retries(
                node, node.stop_client, Trigger.Request, "/safety/stop"
            )
            services["stop"] = stop_result
            errors.extend(
                f"cleanup STOP retry zero failed: {message}"
                for message in stop_zero_errors
            )
            if stop_result["success"] is not True:
                errors.append(
                    "cleanup STOP failed after "
                    f"{stop_result['attempt_count']} attempts"
                )

            disarm_result, disarm_zero_errors = _cleanup_service_with_retries(
                node, node.disarm_client, Disarm.Request, "/safety/disarm"
            )
            services["disarm"] = disarm_result
            errors.extend(
                f"cleanup DISARM retry zero failed: {message}"
                for message in disarm_zero_errors
            )
            if disarm_result["success"] is not True:
                errors.append(
                    "cleanup DISARM failed after "
                    f"{disarm_result['attempt_count']} attempts"
                )

            before_final = _sample_counts(node)
            try:
                final_evidence_fresh = node.wait_until(
                    lambda: _all_counts_advanced(node, before_final),
                    FINAL_EVIDENCE_TIMEOUT_S,
                )
                if not final_evidence_fresh:
                    errors.append("fresh post-DISARM typed telemetry set timed out")
            except BaseException as exc:
                errors.append(f"post-DISARM telemetry collection failed: {exc}")

            summary["evidence"] = _evidence_summary(node, command)
            try:
                node.destroy_node()
            except BaseException as exc:
                errors.append(f"ROS node destruction failed: {exc}")
        else:
            errors.append("ROS commissioning node was not created")

        if initialized:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except BaseException as exc:
                errors.append(f"rclpy shutdown failed: {exc}")

    evidence = summary.get("evidence", {})
    final_safe = isinstance(evidence, dict) and _final_is_safe(evidence)
    motion_evidence_present = bool(
        isinstance(evidence, dict)
        and _motion_evidence_is_present(evidence, command)
    )
    evidence_stayed_clean = bool(
        isinstance(evidence, dict) and _evidence_stayed_clean(evidence)
    )
    arm_ok = bool(
        isinstance(services.get("arm"), dict)
        and services["arm"].get("success") is True
    )
    stop_ok = bool(
        isinstance(services.get("stop"), dict)
        and services["stop"].get("success") is True
    )
    disarm_ok = bool(
        isinstance(services.get("disarm"), dict)
        and services["disarm"].get("success") is True
    )
    pulse_timing_ok = _pulse_timing_is_acceptable(summary.get("pulse"), command)
    summary["interrupted_by_signal"] = interrupted_by
    summary["final_evidence_fresh"] = final_evidence_fresh
    summary["final_safe"] = final_safe
    summary["motion_evidence_present"] = motion_evidence_present
    summary["evidence_stayed_clean"] = evidence_stayed_clean
    summary["pulse_timing_ok"] = pulse_timing_ok
    summary["result"] = (
        "PASS"
        if pulse_completed
        and arm_ok
        and stop_ok
        and disarm_ok
        and final_evidence_fresh
        and final_safe
        and motion_evidence_present
        and evidence_stayed_clean
        and pulse_timing_ok
        and not errors
        else "FAIL"
    )
    return summary, 0 if summary["result"] == "PASS" else 1


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.confirm_lifted != LIFTED_CONFIRMATION:
        parser.error(f"--confirm-lifted must be exactly {LIFTED_CONFIRMATION}")
    try:
        command = command_for(args.mode, args.speed, args.duration)
    except ValueError as exc:
        parser.error(str(exc))

    summary, return_code = run(command)
    print(json.dumps(summary, sort_keys=True, separators=(",", ":")), flush=True)
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
