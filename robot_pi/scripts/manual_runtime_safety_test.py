#!/usr/bin/env python3
"""Run one lifted-track STOP or ESP32 command-watchdog test.

This commissioning utility deliberately supports only bounded MANUAL motion.
It never touches relays or ESTOP.  Every exit path resumes a bridge process
that this tool may have stopped, publishes zero, requests STOP and DISARM, and
collects typed telemetry.  Fault reset is attempted only after this run has
observed the expected CMD_VEL_TIMEOUT and a watchdog counter increment.

Progress is written to stderr.  Exactly one machine-readable JSON document is
written to stdout.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import time
from typing import Callable, Optional

from geometry_msgs.msg import TwistStamped
from module_robot_msgs.msg import FaultEvent, MotorStatus, RelayStatus, RobotStatus
from module_robot_msgs.srv import Arm, Disarm, ResetFault
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger


LIFTED_CONFIRMATION = "I_HAVE_LIFTED_THE_ROBOT"
CMD_VEL_TIMEOUT = 1
PUBLISH_RATE_HZ = 50.0
MAX_LINEAR_M_S = 0.05
MAX_ANGULAR_RAD_S = 0.25
MAX_DURATION_S = 3.0
DEFAULT_LINEAR_M_S = 0.03
DEFAULT_DURATION_S = 1.0
DEFAULT_BRIDGE_HOLD_S = 0.35
MIN_BRIDGE_HOLD_S = 0.33
MAX_BRIDGE_HOLD_S = 0.37
ESP32_COMMAND_TIMEOUT_UPPER_MS = 300
WATCHDOG_EVENT_LATENCY_LOWER_MS = 200.0
WATCHDOG_EVENT_LATENCY_UPPER_MS = 400.0
STOP_ZERO_LATENCY_UPPER_MS = 500.0
TELEMETRY_TIMEOUT_S = 5.0
SERVICE_TIMEOUT_S = 5.0
ARM_CONFIRM_TIMEOUT_S = 3.0
TRANSITION_TIMEOUT_S = 5.0
FINAL_EVIDENCE_TIMEOUT_S = 4.0
INJECTION_MAX_DELAY_S = 0.1
MAX_STATUS_LOCAL_AGE_S = 0.6
MAX_MOTOR_LOCAL_AGE_S = 0.6
MAX_RELAY_LOCAL_AGE_S = 2.0
MAX_REPORTED_MOTOR_AGE_MS = 500
CLEANUP_ATTEMPTS = 3
CLEANUP_RETRY_DELAY_S = 0.15


@dataclass(frozen=True)
class TimedSample:
    received_s: float
    phase: str
    message: object


@dataclass(frozen=True)
class PublishPoint:
    monotonic_s: float
    ros_ns: int


@dataclass(frozen=True)
class BridgeIdentity:
    pid: int
    start_ticks: str
    command_line: str


def _log(message: str) -> None:
    try:
        print(f"[manual_runtime_safety_test] {message}", file=sys.stderr, flush=True)
    except (BrokenPipeError, OSError):
        # Cleanup must not depend on an SSH terminal or writable log stream.
        pass


def _stamp_ns(stamp: object) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _round_ms(seconds: float) -> float:
    return round(seconds * 1000.0, 3)


def _response_dict(response: object) -> dict[str, object]:
    result: dict[str, object] = {"received": True}
    for name in ("success", "resulting_state", "message"):
        if hasattr(response, name):
            result[name] = getattr(response, name)
    return result


def _status_is_zero(status: RobotStatus) -> bool:
    return all(
        int(getattr(status, field)) == 0
        for field in (
            "applied_left_command",
            "applied_right_command",
            "uart_speed",
            "uart_steer",
        )
    )


def _status_dict(status: RobotStatus) -> dict[str, object]:
    return {
        "header_stamp_ns": _stamp_ns(status.header.stamp),
        "state": int(status.state),
        "connected": bool(status.connected),
        "armed": bool(status.armed),
        "estop": bool(status.estop),
        "fault_code": int(status.fault_code),
        "fault_reason": str(status.fault_reason),
        "last_cmd_vel_age_ms": int(status.last_cmd_vel_age_ms),
        "last_heartbeat_age_ms": int(status.last_heartbeat_age_ms),
        "last_motor_feedback_age_ms": int(status.last_motor_feedback_age_ms),
        "applied_left_command": int(status.applied_left_command),
        "applied_right_command": int(status.applied_right_command),
        "uart_speed": int(status.uart_speed),
        "uart_steer": int(status.uart_steer),
        "watchdog_trips": int(status.watchdog_trips),
        "boot_counter": int(status.boot_counter),
    }


def _motor_dict(motor: MotorStatus) -> dict[str, object]:
    return {
        "header_stamp_ns": _stamp_ns(motor.header.stamp),
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
        "uart_invalid_frames": int(motor.uart_invalid_frames),
    }


def _relay_dict(relay: RelayStatus) -> dict[str, object]:
    return {
        "header_stamp_ns": _stamp_ns(relay.header.stamp),
        "available_mask": int(relay.available_mask),
        "active_mask": int(relay.active_mask),
    }


def _event_dict(event: FaultEvent) -> dict[str, object]:
    return {
        "header_stamp_ns": _stamp_ns(event.header.stamp),
        "event_monotonic_us": int(event.event_monotonic_us),
        "fault_code": int(event.fault_code),
        "fault_name": str(event.fault_name),
        "detail": str(event.detail),
        "occurrence_count": int(event.occurrence_count),
        "latched": bool(event.latched),
    }


class RuntimeSafetyNode(Node):
    """Own typed ROS endpoints and timestamp all received evidence locally."""

    def __init__(self) -> None:
        super().__init__("manual_runtime_safety_test")
        self.phase = "startup"
        self.status_samples: list[TimedSample] = []
        self.motor_samples: list[TimedSample] = []
        self.relay_samples: list[TimedSample] = []
        self.fault_samples: list[TimedSample] = []

        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        sensor = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        command = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._command_publisher = self.create_publisher(
            TwistStamped, "/cmd_vel_manual", command
        )
        # Keep explicit references without colliding with rclpy Node internals.
        self._evidence_subscriptions = (
            self.create_subscription(
                RobotStatus, "/esp32/status", self._on_status, reliable
            ),
            self.create_subscription(
                MotorStatus, "/motor/status", self._on_motor, sensor
            ),
            self.create_subscription(
                RelayStatus, "/relay/status", self._on_relay, reliable
            ),
            self.create_subscription(
                FaultEvent, "/esp32/fault_event", self._on_fault, reliable
            ),
        )
        self.arm_client = self.create_client(Arm, "/safety/arm")
        self.stop_client = self.create_client(Trigger, "/safety/stop")
        self.disarm_client = self.create_client(Disarm, "/safety/disarm")
        self.reset_fault_client = self.create_client(
            ResetFault, "/safety/reset_fault"
        )

    def _on_status(self, message: RobotStatus) -> None:
        self.status_samples.append(TimedSample(time.monotonic(), self.phase, message))

    def _on_motor(self, message: MotorStatus) -> None:
        self.motor_samples.append(TimedSample(time.monotonic(), self.phase, message))

    def _on_relay(self, message: RelayStatus) -> None:
        self.relay_samples.append(TimedSample(time.monotonic(), self.phase, message))

    def _on_fault(self, message: FaultEvent) -> None:
        self.fault_samples.append(TimedSample(time.monotonic(), self.phase, message))

    def spin_once(self, timeout_s: float) -> None:
        rclpy.spin_once(self, timeout_sec=max(0.0, timeout_s))

    def spin_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + max(0.0, duration_s)
        while time.monotonic() < deadline:
            self.spin_once(min(0.05, deadline - time.monotonic()))

    def wait_until(self, predicate: Callable[[], bool], timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while not predicate():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            self.spin_once(min(0.05, remaining))
        return True

    def call_service(self, client, request: object, name: str) -> object:
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_S):
            raise RuntimeError(f"service unavailable: {name}")
        future = client.call_async(request)
        deadline = time.monotonic() + SERVICE_TIMEOUT_S
        while not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise RuntimeError(f"service timed out: {name}")
            self.spin_once(min(0.05, remaining))
        response = future.result()
        if response is None:
            raise RuntimeError(f"service returned no response: {name}")
        return response

    def publish_command(self, linear_x: float, angular_z: float) -> PublishPoint:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.twist.linear.x = float(linear_x)
        message.twist.angular.z = float(angular_z)
        ros_ns = _stamp_ns(message.header.stamp)
        self._command_publisher.publish(message)
        return PublishPoint(time.monotonic(), ros_ns)


def _sample_counts(node: RuntimeSafetyNode) -> tuple[int, int, int]:
    return (
        len(node.status_samples),
        len(node.motor_samples),
        len(node.relay_samples),
    )


def _require_typed_telemetry(node: RuntimeSafetyNode) -> None:
    if not node.wait_until(lambda: all(_sample_counts(node)), TELEMETRY_TIMEOUT_S):
        status, motor, relay = _sample_counts(node)
        raise RuntimeError(
            "typed telemetry incomplete: "
            f"status={status}, motor={motor}, relay={relay}"
        )


def _require_services(node: RuntimeSafetyNode, *, include_reset: bool) -> None:
    required = [
        (node.arm_client, "/safety/arm"),
        (node.stop_client, "/safety/stop"),
        (node.disarm_client, "/safety/disarm"),
    ]
    if include_reset:
        required.append((node.reset_fault_client, "/safety/reset_fault"))
    unavailable = [
        name
        for client, name in required
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_S)
    ]
    if unavailable:
        raise RuntimeError(f"required services unavailable: {', '.join(unavailable)}")


def _require_live_health(
    node: RuntimeSafetyNode, *, expected_armed: Optional[bool]
) -> None:
    if not node.status_samples or not node.motor_samples or not node.relay_samples:
        raise RuntimeError("typed telemetry is incomplete")
    now = time.monotonic()
    status_sample = node.status_samples[-1]
    motor_sample = node.motor_samples[-1]
    relay_sample = node.relay_samples[-1]
    status = status_sample.message
    motor = motor_sample.message
    relay = relay_sample.message

    if now - status_sample.received_s > MAX_STATUS_LOCAL_AGE_S:
        raise RuntimeError("/esp32/status is locally stale")
    if now - motor_sample.received_s > MAX_MOTOR_LOCAL_AGE_S:
        raise RuntimeError("/motor/status is locally stale")
    if now - relay_sample.received_s > MAX_RELAY_LOCAL_AGE_S:
        raise RuntimeError("/relay/status is locally stale")
    if not status.connected:
        raise RuntimeError("ESP32 is disconnected")
    if status.estop or status.state == RobotStatus.STATE_ESTOP:
        raise RuntimeError("ESTOP is active")
    if status.fault_code != 0 or status.state == RobotStatus.STATE_FAULT:
        raise RuntimeError(
            f"unexpected ESP32 fault before watchdog cut: {int(status.fault_code)}"
        )
    if int(status.last_motor_feedback_age_ms) > MAX_REPORTED_MOTOR_AGE_MS:
        raise RuntimeError(
            "motor feedback is stale: "
            f"{int(status.last_motor_feedback_age_ms)} ms"
        )
    if int(motor.controller_fault) != 0:
        raise RuntimeError(
            f"motor controller fault is active: {int(motor.controller_fault)}"
        )
    if not math.isfinite(float(motor.battery_voltage)):
        raise RuntimeError("motor battery voltage is not finite")
    if not motor.board_temperature_available or not math.isfinite(
        float(motor.board_temperature_c)
    ):
        raise RuntimeError("motor controller temperature is unavailable")
    if int(relay.active_mask) != 0:
        raise RuntimeError(
            f"relay active mask must remain zero, got {int(relay.active_mask)}"
        )

    if expected_armed is True and (
        not status.armed or status.state != RobotStatus.STATE_ARMED
    ):
        raise RuntimeError("ESP32 is not ARMED")
    if expected_armed is False:
        if status.armed or status.state != RobotStatus.STATE_DISARMED:
            raise RuntimeError("ESP32 is not DISARMED")
        if not _status_is_zero(status):
            raise RuntimeError("motor outputs are non-zero while DISARMED")


def _publish_motion(
    node: RuntimeSafetyNode,
    linear_x: float,
    angular_z: float,
    duration_s: float,
) -> tuple[list[PublishPoint], float, float]:
    interval_s = 1.0 / PUBLISH_RATE_HZ
    started_s = time.monotonic()
    deadline_s = started_s + duration_s
    next_publish_s = started_s
    points: list[PublishPoint] = []

    while time.monotonic() < deadline_s:
        now = time.monotonic()
        if now < next_publish_s:
            node.spin_once(min(next_publish_s - now, deadline_s - now))
            _require_live_health(node, expected_armed=True)
            continue
        _require_live_health(node, expected_armed=True)
        points.append(node.publish_command(linear_x, angular_z))
        next_publish_s += interval_s
        if next_publish_s < time.monotonic():
            next_publish_s = time.monotonic() + interval_s

    if not points:
        raise RuntimeError("no non-zero command was published")
    return points, started_s, time.monotonic()


def _zero_burst(node: RuntimeSafetyNode) -> None:
    for _ in range(5):
        node.publish_command(0.0, 0.0)
        node.spin_once(1.0 / PUBLISH_RATE_HZ)


def _call_cleanup_service(
    node: RuntimeSafetyNode,
    client: object,
    request_factory: Callable[[], object],
    name: str,
) -> dict[str, object]:
    attempts: list[dict[str, object]] = []
    result: dict[str, object] = {
        "success": False,
        "received": False,
        "attempts": attempts,
    }
    for attempt_number in range(1, CLEANUP_ATTEMPTS + 1):
        attempt: dict[str, object] = {"attempt": attempt_number}
        try:
            response = node.call_service(client, request_factory(), name)
            attempt.update(_response_dict(response))
            if bool(getattr(response, "success", False)):
                attempts.append(attempt)
                result.update(_response_dict(response))
                result["successful_attempt"] = attempt_number
                break
        except BaseException as exc:
            attempt.update({"received": False, "error": str(exc)})
        attempts.append(attempt)
        if attempt_number < CLEANUP_ATTEMPTS:
            try:
                _zero_burst(node)
                attempt["retry_zero_published"] = True
            except BaseException as exc:
                attempt["retry_zero_published"] = False
                attempt["retry_zero_error"] = str(exc)
            try:
                node.spin_for(CLEANUP_RETRY_DELAY_S)
            except BaseException as exc:
                attempt["retry_spin_error"] = str(exc)
    result["attempt_count"] = len(attempts)
    result["received"] = any(item.get("received") is True for item in attempts)
    return result


def _read_bridge_identity(pid: int) -> BridgeIdentity:
    if pid <= 1 or pid == os.getpid():
        raise ValueError("bridge PID must identify a separate non-init process")
    proc = Path("/proc") / str(pid)
    try:
        stat_text = (proc / "stat").read_text(encoding="utf-8")
        suffix = stat_text.rsplit(")", 1)[1].split()
        # Fields in suffix begin at proc(5) field 3; starttime is field 22.
        start_ticks = suffix[19]
        raw_command = (proc / "cmdline").read_bytes()
        executable = (proc / "exe").resolve(strict=True)
    except (OSError, IndexError, RuntimeError) as exc:
        raise ValueError(f"cannot inspect bridge PID {pid}: {exc}") from exc
    argv = [
        item.decode("utf-8", "replace")
        for item in raw_command.split(b"\0")
        if item
    ]
    command_line = " ".join(argv)
    expected_tail = (
        "install",
        "module_robot_esp32_bridge",
        "lib",
        "module_robot_esp32_bridge",
        "esp32_bridge",
    )
    try:
        script = Path(argv[1]) if len(argv) >= 2 else None
        argv0 = Path(argv[0]).resolve(strict=False) if argv else None
        script_exists = script is not None and script.is_file()
    except (OSError, RuntimeError) as exc:
        raise ValueError(f"cannot resolve bridge PID {pid} argv: {exc}") from exc
    valid_identity = bool(
        argv0 == executable
        and executable.name.startswith("python3")
        and script is not None
        and script_exists
        and tuple(script.parts[-5:]) == expected_tail
        and "--ros-args" in argv[2:]
    )
    if not valid_identity:
        raise ValueError(
            f"PID {pid} is not the installed ROS esp32_bridge process: "
            f"exe={str(executable)!r}, argv={command_line!r}"
        )
    return BridgeIdentity(pid, start_ticks, command_line)


def _same_bridge(identity: BridgeIdentity) -> bool:
    try:
        current = _read_bridge_identity(identity.pid)
    except BaseException:
        return False
    return current.start_ticks == identity.start_ticks


_CONT_HELPER = r"""
import os
from pathlib import Path
import sys
import time

pid = int(sys.argv[1])
expected_start = sys.argv[2]
delay = float(sys.argv[3])
try:
    proc_stat = Path('/proc') / str(pid) / 'stat'
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        stat_text = proc_stat.read_text(encoding='utf-8')
        fields = stat_text.rsplit(')', 1)[1].split()
        if fields[19] != expected_start:
            raise RuntimeError('bridge PID identity changed')
        if fields[0] in ('T', 't'):
            break
        time.sleep(0.005)
    else:
        raise RuntimeError('bridge never entered a stopped state')
    time.sleep(delay)
    stat_text = proc_stat.read_text(encoding='utf-8')
    fields = stat_text.rsplit(')', 1)[1].split()
    if fields[19] == expected_start:
        os.execv('/bin/kill', ['/bin/kill', '-CONT', str(pid)])
except (OSError, IndexError, RuntimeError, ValueError):
    pass
"""


def _schedule_independent_cont(
    identity: BridgeIdentity, delay_s: float
) -> subprocess.Popen[bytes]:
    """Start an independent, identity-checked /bin/kill -CONT fail-safe."""

    return subprocess.Popen(
        [
            sys.executable,
            "-c",
            _CONT_HELPER,
            str(identity.pid),
            identity.start_ticks,
            f"{delay_s:.6f}",
        ],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        close_fds=True,
        start_new_session=True,
    )


def _force_resume(identity: Optional[BridgeIdentity]) -> dict[str, object]:
    result: dict[str, object] = {"attempted": identity is not None, "sent": False}
    if identity is None:
        return result
    try:
        if not _same_bridge(identity):
            result["error"] = "bridge process disappeared or PID identity changed"
            return result
        completed = subprocess.run(
            ["/bin/kill", "-CONT", str(identity.pid)],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            check=False,
            timeout=1.0,
        )
        result["returncode"] = int(completed.returncode)
        result["sent"] = completed.returncode == 0
        if completed.returncode != 0:
            result["stderr"] = completed.stderr.decode("utf-8", "replace").strip()
    except BaseException as exc:
        result["error"] = str(exc)
    return result


def _first_sample(
    samples: list[TimedSample],
    start_index: int,
    predicate: Callable[[object], bool],
) -> Optional[TimedSample]:
    return next(
        (sample for sample in samples[start_index:] if predicate(sample.message)),
        None,
    )


def _wait_for_sample(
    node: RuntimeSafetyNode,
    samples: list[TimedSample],
    start_index: int,
    predicate: Callable[[object], bool],
    timeout_s: float,
) -> Optional[TimedSample]:
    found: Optional[TimedSample] = None

    def locate() -> bool:
        nonlocal found
        found = _first_sample(samples, start_index, predicate)
        return found is not None

    node.wait_until(locate, timeout_s)
    return found


def _motion_evidence(
    node: RuntimeSafetyNode, started_s: float, ended_s: float
) -> dict[str, object]:
    # Include a small receive tail: status and feedback are periodic and may
    # describe a command emitted immediately before the local pulse deadline.
    tail_s = ended_s + 0.25
    statuses = [
        sample.message
        for sample in node.status_samples
        if started_s <= sample.received_s <= tail_s
    ]
    motors = [
        sample.message
        for sample in node.motor_samples
        if started_s <= sample.received_s <= tail_s
    ]
    return {
        "status_samples": len(statuses),
        "motor_samples": len(motors),
        "applied_left_range": (
            [
                min(int(item.applied_left_command) for item in statuses),
                max(int(item.applied_left_command) for item in statuses),
            ]
            if statuses
            else None
        ),
        "applied_right_range": (
            [
                min(int(item.applied_right_command) for item in statuses),
                max(int(item.applied_right_command) for item in statuses),
            ]
            if statuses
            else None
        ),
        "uart_speed_range": (
            [
                min(int(item.uart_speed) for item in statuses),
                max(int(item.uart_speed) for item in statuses),
            ]
            if statuses
            else None
        ),
        "uart_steer_range": (
            [
                min(int(item.uart_steer) for item in statuses),
                max(int(item.uart_steer) for item in statuses),
            ]
            if statuses
            else None
        ),
        "left_feedback_range": (
            [
                min(int(item.left_feedback) for item in motors),
                max(int(item.left_feedback) for item in motors),
            ]
            if motors
            else None
        ),
        "right_feedback_range": (
            [
                min(int(item.right_feedback) for item in motors),
                max(int(item.right_feedback) for item in motors),
            ]
            if motors
            else None
        ),
        "uart_valid_frame_delta": (
            max(int(item.uart_valid_frames) for item in motors)
            - min(int(item.uart_valid_frames) for item in motors)
            if motors
            else None
        ),
    }


def _range_has_nonzero(value: object) -> bool:
    return bool(
        isinstance(value, list)
        and len(value) == 2
        and any(abs(float(item)) > 0.0 for item in value)
    )


def _motion_seen(evidence: dict[str, object]) -> bool:
    return bool(
        int(evidence.get("status_samples", 0)) > 0
        and int(evidence.get("motor_samples", 0)) > 1
        and _range_has_nonzero(evidence.get("applied_left_range"))
        and _range_has_nonzero(evidence.get("applied_right_range"))
        and _range_has_nonzero(evidence.get("left_feedback_range"))
        and _range_has_nonzero(evidence.get("right_feedback_range"))
        and isinstance(evidence.get("uart_valid_frame_delta"), int)
        and int(evidence["uart_valid_frame_delta"]) > 0
    )


def _final_snapshot(node: RuntimeSafetyNode) -> Optional[dict[str, object]]:
    if not node.status_samples or not node.motor_samples or not node.relay_samples:
        return None
    return {
        "status": _status_dict(node.status_samples[-1].message),
        "motor": _motor_dict(node.motor_samples[-1].message),
        "relay": _relay_dict(node.relay_samples[-1].message),
    }


def _final_is_safe(snapshot: object) -> bool:
    if not isinstance(snapshot, dict):
        return False
    status = snapshot.get("status")
    motor = snapshot.get("motor")
    relay = snapshot.get("relay")
    if not all(isinstance(item, dict) for item in (status, motor, relay)):
        return False
    assert isinstance(status, dict)
    assert isinstance(motor, dict)
    assert isinstance(relay, dict)
    return bool(
        status.get("state") == RobotStatus.STATE_DISARMED
        and status.get("connected") is True
        and status.get("armed") is False
        and status.get("estop") is False
        and status.get("fault_code") == 0
        and status.get("applied_left_command") == 0
        and status.get("applied_right_command") == 0
        and status.get("uart_speed") == 0
        and status.get("uart_steer") == 0
        and isinstance(status.get("last_motor_feedback_age_ms"), int)
        and int(status["last_motor_feedback_age_ms"]) <= MAX_REPORTED_MOTOR_AGE_MS
        and motor.get("controller_fault") == 0
        and motor.get("battery_voltage") is not None
        and motor.get("board_temperature_available") is True
        and motor.get("board_temperature_c") is not None
        and relay.get("active_mask") == 0
    )


def _all_evidence_advanced(
    node: RuntimeSafetyNode, before: tuple[int, int, int]
) -> bool:
    after = _sample_counts(node)
    return all(current > previous for current, previous in zip(after, before))


def _wait_for_motor_zero(node: RuntimeSafetyNode, timeout_s: float) -> bool:
    before = len(node.motor_samples)
    return node.wait_until(
        lambda: bool(
            len(node.motor_samples) > before
            and abs(int(node.motor_samples[-1].message.left_feedback)) <= 5
            and abs(int(node.motor_samples[-1].message.right_feedback)) <= 5
            and int(node.motor_samples[-1].message.controller_fault) == 0
        ),
        timeout_s,
    )


def _validate_command(linear_x: float, angular_z: float, duration_s: float) -> None:
    values = (linear_x, angular_z, duration_s)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("command values must be finite")
    if abs(linear_x) > MAX_LINEAR_M_S:
        raise ValueError(f"|linear-x| must not exceed {MAX_LINEAR_M_S:g} m/s")
    if abs(angular_z) > MAX_ANGULAR_RAD_S:
        raise ValueError(
            f"|angular-z| must not exceed {MAX_ANGULAR_RAD_S:g} rad/s"
        )
    if abs(linear_x) <= 1.0e-9 and abs(angular_z) <= 1.0e-9:
        raise ValueError("the test command must be non-zero")
    if not 0.0 < duration_s <= MAX_DURATION_S:
        raise ValueError(f"duration must be in (0, {MAX_DURATION_S:g}] seconds")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run a typed lifted-track STOP or ESP32 watchdog test."
    )
    parser.add_argument("mode", choices=("stop", "watchdog"))
    parser.add_argument(
        "--linear-x",
        type=float,
        default=DEFAULT_LINEAR_M_S,
        help="bounded manual linear command in m/s (default 0.03; |max| 0.05)",
    )
    parser.add_argument(
        "--angular-z",
        type=float,
        default=0.0,
        help="bounded manual angular command in rad/s (default 0; |max| 0.25)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_S,
        help="non-zero publish duration in seconds (default 1; max 3)",
    )
    parser.add_argument(
        "--bridge-pid",
        type=int,
        default=None,
        help="required in watchdog mode: exact esp32_bridge OS PID",
    )
    parser.add_argument(
        "--bridge-hold",
        type=float,
        default=DEFAULT_BRIDGE_HOLD_S,
        help=(
            "watchdog-only SIGSTOP interval in seconds (default 0.35; "
            "kept above the ESP32 300 ms watchdog and below the host 400 ms "
            "motor-feedback deadline)"
        ),
    )
    parser.add_argument(
        "--confirm-lifted",
        required=True,
        metavar="TOKEN",
        help=f"must be exactly {LIFTED_CONFIRMATION}",
    )
    return parser


def run(args: argparse.Namespace) -> tuple[dict[str, object], int]:
    started_s = time.monotonic()
    summary: dict[str, object] = {
        "schema_version": 1,
        "tool": "manual_runtime_safety_test",
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "requested": {
            "mode": args.mode,
            "linear_x_m_s": args.linear_x,
            "angular_z_rad_s": args.angular_z,
            "duration_s": args.duration,
            "publish_rate_hz": PUBLISH_RATE_HZ,
            "bridge_pid": args.bridge_pid,
            "bridge_hold_s": args.bridge_hold if args.mode == "watchdog" else None,
        },
        "services": {},
        "errors": [],
    }
    errors = summary["errors"]
    services = summary["services"]
    assert isinstance(errors, list)
    assert isinstance(services, dict)

    initialized = False
    node: Optional[RuntimeSafetyNode] = None
    bridge_identity: Optional[BridgeIdentity] = None
    cont_helper: Optional[subprocess.Popen[bytes]] = None
    publish_points: list[PublishPoint] = []
    motion_started_s: Optional[float] = None
    motion_ended_s: Optional[float] = None
    baseline_watchdog: Optional[int] = None
    stop_zero_sample: Optional[TimedSample] = None
    watchdog_status_sample: Optional[TimedSample] = None
    watchdog_event_sample: Optional[TimedSample] = None
    watchdog_motor_sample: Optional[TimedSample] = None
    watchdog_relay_sample: Optional[TimedSample] = None
    watchdog_confirmed = False
    bridge_was_stopped = False
    final_evidence_fresh = False
    interrupted_by: Optional[int] = None

    def interrupt(signum, _frame) -> None:
        nonlocal interrupted_by
        interrupted_by = int(signum)
        raise InterruptedError(f"received signal {signum}")

    try:
        if args.mode == "watchdog":
            bridge_identity = _read_bridge_identity(args.bridge_pid)
            summary["bridge_identity"] = {
                "pid": bridge_identity.pid,
                "start_ticks": bridge_identity.start_ticks,
                "command_line": bridge_identity.command_line,
            }
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        signal.signal(signal.SIGINT, interrupt)
        signal.signal(signal.SIGHUP, interrupt)
        signal.signal(signal.SIGTERM, interrupt)
        node = RuntimeSafetyNode()
        node.phase = "preflight"
        _log("waiting for typed status, motor feedback, and relay status")
        _require_typed_telemetry(node)
        _require_services(node, include_reset=args.mode == "watchdog")
        _require_live_health(node, expected_armed=False)
        baseline_watchdog = int(node.status_samples[-1].message.watchdog_trips)
        summary["preflight"] = {
            "status": _status_dict(node.status_samples[-1].message),
            "motor": _motor_dict(node.motor_samples[-1].message),
            "relay": _relay_dict(node.relay_samples[-1].message),
        }

        arm_request = Arm.Request()
        arm_request.arm_nonce = secrets.randbits(32) or 1
        arm_request.requested_mode = Arm.Request.MODE_MANUAL
        arm_status_index = len(node.status_samples)
        node.phase = "arm"
        _log("requesting explicit MANUAL ARM")
        arm_response = node.call_service(node.arm_client, arm_request, "/safety/arm")
        services["arm"] = _response_dict(arm_response)
        if not arm_response.success:
            raise RuntimeError(f"MANUAL ARM rejected: {arm_response.message}")
        armed_sample = _wait_for_sample(
            node,
            node.status_samples,
            arm_status_index,
            lambda message: bool(
                message.armed and message.state == RobotStatus.STATE_ARMED
            ),
            ARM_CONFIRM_TIMEOUT_S,
        )
        if armed_sample is None:
            raise RuntimeError("no fresh ESP32 ARMED confirmation")
        _require_live_health(node, expected_armed=True)
        if not _status_is_zero(armed_sample.message):
            raise RuntimeError("outputs are non-zero immediately after ARM")

        node.phase = "motion"
        _log(
            f"publishing linear={args.linear_x:+.3f} m/s, "
            f"angular={args.angular_z:+.3f} rad/s for {args.duration:.3f} s"
        )
        publish_points, motion_started_s, motion_ended_s = _publish_motion(
            node, args.linear_x, args.angular_z, args.duration
        )
        last_command = publish_points[-1]
        intervals = [
            later.monotonic_s - earlier.monotonic_s
            for earlier, later in zip(publish_points, publish_points[1:])
        ]
        summary["motion_publish"] = {
            "publish_count": len(publish_points),
            "actual_duration_s": round(motion_ended_s - motion_started_s, 6),
            "last_nonzero_command_ros_ns": last_command.ros_ns,
            "mean_rate_hz": (
                round(
                    (len(publish_points) - 1)
                    / (publish_points[-1].monotonic_s - publish_points[0].monotonic_s),
                    3,
                )
                if len(publish_points) > 1
                and publish_points[-1].monotonic_s > publish_points[0].monotonic_s
                else None
            ),
            "maximum_interval_ms": _round_ms(max(intervals)) if intervals else None,
        }

        if args.mode == "stop":
            node.phase = "stop_transition"
            status_index = len(node.status_samples)
            stop_invoked_s = time.monotonic()
            _log("publisher ceased; requesting /safety/stop immediately")
            response = node.call_service(
                node.stop_client, Trigger.Request(), "/safety/stop"
            )
            stop_response_s = time.monotonic()
            services["test_stop"] = _response_dict(response)
            if not response.success:
                raise RuntimeError(f"STOP rejected: {response.message}")
            stop_zero_sample = _wait_for_sample(
                node,
                node.status_samples,
                status_index,
                lambda message: _status_is_zero(message),
                TRANSITION_TIMEOUT_S,
            )
            if stop_zero_sample is None:
                raise RuntimeError("no fresh all-zero RobotStatus after STOP")
            zero_status = stop_zero_sample.message
            summary["stop_evidence"] = {
                "stop_invoked_after_last_command_ms": _round_ms(
                    stop_invoked_s - last_command.monotonic_s
                ),
                "stop_service_response_ms": _round_ms(
                    stop_response_s - stop_invoked_s
                ),
                "last_command_to_first_zero_local_ms": _round_ms(
                    stop_zero_sample.received_s - last_command.monotonic_s
                ),
                "last_command_to_first_zero_ros_ms": round(
                    (_stamp_ns(zero_status.header.stamp) - last_command.ros_ns)
                    / 1_000_000.0,
                    3,
                ),
                "first_zero_status": _status_dict(zero_status),
                "explicit_stop_injected_before_stale_budget": bool(
                    stop_invoked_s - last_command.monotonic_s
                    <= INJECTION_MAX_DELAY_S
                ),
            }
        else:
            assert bridge_identity is not None
            if not _same_bridge(bridge_identity):
                raise RuntimeError("esp32_bridge identity changed before watchdog cut")
            node.phase = "watchdog_cut"
            status_index = len(node.status_samples)
            motor_index = len(node.motor_samples)
            relay_index = len(node.relay_samples)
            event_index = len(node.fault_samples)
            cont_helper = _schedule_independent_cont(
                bridge_identity, args.bridge_hold
            )
            helper_started_s = time.monotonic()
            _log(
                "independent identity-checked /bin/kill -CONT fail-safe scheduled; "
                f"sending SIGSTOP to esp32_bridge PID {bridge_identity.pid}"
            )
            os.kill(bridge_identity.pid, signal.SIGSTOP)
            bridge_was_stopped = True
            sigstop_s = time.monotonic()
            # Do not publish zero or call STOP while the ESP32 command watchdog
            # is under test.  The separate helper resumes the bridge even if
            # this process is interrupted during the hold.
            node.spin_for(args.bridge_hold + 0.15)
            helper_returncode = cont_helper.poll()
            if helper_returncode is None:
                try:
                    helper_returncode = cont_helper.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    helper_returncode = None
            summary["bridge_cut"] = {
                "sigstop_after_last_command_ms": _round_ms(
                    sigstop_s - last_command.monotonic_s
                ),
                "cont_helper_scheduled_before_sigstop_ms": _round_ms(
                    sigstop_s - helper_started_s
                ),
                "requested_hold_ms": round(args.bridge_hold * 1000.0, 3),
                "cont_helper_returncode": helper_returncode,
                "sigstop_injected_before_stale_budget": bool(
                    sigstop_s - last_command.monotonic_s
                    <= INJECTION_MAX_DELAY_S
                ),
            }
            if not _same_bridge(bridge_identity):
                raise RuntimeError("esp32_bridge exited or PID identity changed")

            watchdog_event_sample = _wait_for_sample(
                node,
                node.fault_samples,
                event_index,
                lambda message: int(message.fault_code) == CMD_VEL_TIMEOUT,
                TRANSITION_TIMEOUT_S,
            )
            watchdog_status_sample = _wait_for_sample(
                node,
                node.status_samples,
                status_index,
                lambda message: bool(
                    int(message.fault_code) == CMD_VEL_TIMEOUT
                    and message.state == RobotStatus.STATE_FAULT
                    and not message.armed
                    and _status_is_zero(message)
                    and int(message.watchdog_trips) > int(baseline_watchdog)
                ),
                TRANSITION_TIMEOUT_S,
            )
            watchdog_motor_sample = _wait_for_sample(
                node,
                node.motor_samples,
                motor_index,
                lambda message: int(message.controller_fault) == 0,
                TRANSITION_TIMEOUT_S,
            )
            watchdog_relay_sample = _wait_for_sample(
                node,
                node.relay_samples,
                relay_index,
                lambda message: int(message.active_mask) == 0,
                TRANSITION_TIMEOUT_S,
            )
            if watchdog_status_sample is None:
                raise RuntimeError(
                    "no fresh CMD_VEL_TIMEOUT status with zero outputs and "
                    "an incremented watchdog counter"
                )
            watchdog_confirmed = True
            if watchdog_motor_sample is None:
                raise RuntimeError("no fresh motor feedback after watchdog cut")
            if watchdog_relay_sample is None:
                raise RuntimeError("no fresh relay-zero status after watchdog cut")
            fault_status = watchdog_status_sample.message
            event_data = (
                _event_dict(watchdog_event_sample.message)
                if watchdog_event_sample is not None
                else None
            )
            summary["watchdog_evidence"] = {
                "expected_timeout_upper_ms": ESP32_COMMAND_TIMEOUT_UPPER_MS,
                "baseline_watchdog_trips": baseline_watchdog,
                "watchdog_trips_after": int(fault_status.watchdog_trips),
                "watchdog_counter_delta": int(fault_status.watchdog_trips)
                - int(baseline_watchdog),
                "status_last_cmd_vel_age_ms": int(
                    fault_status.last_cmd_vel_age_ms
                ),
                "last_command_to_fault_status_header_ms": round(
                    (_stamp_ns(fault_status.header.stamp) - last_command.ros_ns)
                    / 1_000_000.0,
                    3,
                ),
                "last_command_to_fault_event_header_ms": (
                    round(
                        (
                            _stamp_ns(watchdog_event_sample.message.header.stamp)
                            - last_command.ros_ns
                        )
                        / 1_000_000.0,
                        3,
                    )
                    if watchdog_event_sample is not None
                    else None
                ),
                "fault_status": _status_dict(fault_status),
                "fault_event": event_data,
                "fresh_motor_after_cut": _motor_dict(
                    watchdog_motor_sample.message
                ),
                "fresh_relay_after_cut": _relay_dict(
                    watchdog_relay_sample.message
                ),
            }
            if watchdog_event_sample is None:
                raise RuntimeError("CMD_VEL_TIMEOUT FaultEvent was not received")
    except (KeyboardInterrupt, InterruptedError) as exc:
        errors.append(f"interrupted: {exc}")
    except BaseException as exc:
        errors.append(f"{type(exc).__name__}: {exc}")
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if bridge_was_stopped or bridge_identity is not None:
            summary["forced_bridge_resume"] = _force_resume(bridge_identity)
        if node is not None:
            node.phase = "cleanup"
            _log("cleanup: zero, STOP, DISARM")
            try:
                node.spin_for(0.15)
                _zero_burst(node)
                services["cleanup_zero"] = {"published": True, "count": 5}
            except BaseException as exc:
                services["cleanup_zero"] = {
                    "published": False,
                    "error": str(exc),
                }
                errors.append(f"cleanup zero failed: {exc}")

            try:
                cleanup_stop = _call_cleanup_service(
                    node, node.stop_client, Trigger.Request, "/safety/stop"
                )
            except BaseException as exc:
                cleanup_stop = {"success": False, "error": str(exc)}
            services["cleanup_stop"] = cleanup_stop
            if cleanup_stop.get("success") is not True:
                errors.append("cleanup STOP failed after all retries")

            try:
                cleanup_disarm = _call_cleanup_service(
                    node, node.disarm_client, Disarm.Request, "/safety/disarm"
                )
            except BaseException as exc:
                cleanup_disarm = {"success": False, "error": str(exc)}
            services["cleanup_disarm"] = cleanup_disarm
            if cleanup_disarm.get("success") is not True:
                errors.append("cleanup DISARM failed after all retries")

            if args.mode == "watchdog" and baseline_watchdog is not None:
                # A fresh status is sufficient proof even if an exception was
                # raised before the main path recorded the same evidence.
                proof = next(
                    (
                        sample
                        for sample in reversed(node.status_samples)
                        if int(sample.message.fault_code) == CMD_VEL_TIMEOUT
                        and int(sample.message.watchdog_trips) > baseline_watchdog
                        and _status_is_zero(sample.message)
                    ),
                    None,
                )
                watchdog_confirmed = watchdog_confirmed or proof is not None
                if watchdog_confirmed:
                    _log(
                        "observed induced CMD_VEL_TIMEOUT; waiting for stopped "
                        "feedback before the permitted fault reset"
                    )
                    motor_zero = _wait_for_motor_zero(node, TRANSITION_TIMEOUT_S)
                    summary["motor_feedback_zero_before_reset"] = motor_zero
                    if not motor_zero:
                        errors.append("motor feedback did not settle to zero before reset")
                    else:
                        reset_result = _call_cleanup_service(
                            node,
                            node.reset_fault_client,
                            ResetFault.Request,
                            "/safety/reset_fault",
                        )
                        services["reset_induced_fault"] = reset_result
                        if reset_result.get("success") is not True:
                            errors.append("induced fault reset failed after all retries")
                        else:
                            try:
                                services["post_reset_stop"] = _call_cleanup_service(
                                    node,
                                    node.stop_client,
                                    Trigger.Request,
                                    "/safety/stop",
                                )
                            except BaseException as exc:
                                services["post_reset_stop"] = {
                                    "success": False,
                                    "error": str(exc),
                                }
                            try:
                                services["post_reset_disarm"] = _call_cleanup_service(
                                    node,
                                    node.disarm_client,
                                    Disarm.Request,
                                    "/safety/disarm",
                                )
                            except BaseException as exc:
                                services["post_reset_disarm"] = {
                                    "success": False,
                                    "error": str(exc),
                                }
                            if services["post_reset_stop"].get("success") is not True:
                                errors.append("post-reset STOP failed after all retries")
                            if services["post_reset_disarm"].get("success") is not True:
                                errors.append("post-reset DISARM failed after all retries")
                else:
                    services["reset_induced_fault"] = {
                        "attempted": False,
                        "reason": (
                            "no typed CMD_VEL_TIMEOUT plus watchdog increment; "
                            "fault reset is intentionally forbidden"
                        ),
                    }

            before_final = _sample_counts(node)
            try:
                final_evidence_fresh = node.wait_until(
                    lambda: _all_evidence_advanced(node, before_final),
                    FINAL_EVIDENCE_TIMEOUT_S,
                )
                if not final_evidence_fresh:
                    errors.append("fresh final typed telemetry set timed out")
            except BaseException as exc:
                errors.append(f"final telemetry collection failed: {exc}")

            if motion_started_s is not None and motion_ended_s is not None:
                summary["motion_evidence"] = _motion_evidence(
                    node, motion_started_s, motion_ended_s
                )
            summary["observed"] = {
                "fault_codes": sorted(
                    {int(item.message.fault_code) for item in node.status_samples}
                ),
                "watchdog_trips": sorted(
                    {int(item.message.watchdog_trips) for item in node.status_samples}
                ),
                "controller_faults": sorted(
                    {int(item.message.controller_fault) for item in node.motor_samples}
                ),
                "relay_active_masks": sorted(
                    {int(item.message.active_mask) for item in node.relay_samples}
                ),
                "estop_seen": any(
                    bool(item.message.estop) for item in node.status_samples
                ),
                "fault_event_codes": [
                    int(item.message.fault_code) for item in node.fault_samples
                ],
            }
            summary["final"] = _final_snapshot(node)
            try:
                node.destroy_node()
            except BaseException as exc:
                errors.append(f"ROS node destruction failed: {exc}")
        else:
            errors.append("ROS runtime-safety node was not created")

        if initialized:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except BaseException as exc:
                errors.append(f"rclpy shutdown failed: {exc}")

    motion_evidence = summary.get("motion_evidence", {})
    motion_ok = bool(
        isinstance(motion_evidence, dict) and _motion_seen(motion_evidence)
    )
    relay_clean = bool(
        isinstance(summary.get("observed"), dict)
        and summary["observed"].get("relay_active_masks") == [0]
    )
    no_estop = bool(
        isinstance(summary.get("observed"), dict)
        and summary["observed"].get("estop_seen") is False
    )
    final_safe = _final_is_safe(summary.get("final"))
    arm_ok = bool(
        isinstance(services.get("arm"), dict)
        and services["arm"].get("success") is True
    )
    cleanup_ok = bool(
        isinstance(services.get("cleanup_stop"), dict)
        and services["cleanup_stop"].get("success") is True
        and isinstance(services.get("cleanup_disarm"), dict)
        and services["cleanup_disarm"].get("success") is True
    )
    motion_publish = summary.get("motion_publish")
    expected_publish_count = max(
        1, math.floor(float(args.duration) * PUBLISH_RATE_HZ * 0.8)
    )
    publish_timing_ok = bool(
        isinstance(motion_publish, dict)
        and int(motion_publish.get("publish_count", 0)) >= expected_publish_count
        and (
            int(motion_publish.get("publish_count", 0)) == 1
            or (
                isinstance(motion_publish.get("mean_rate_hz"), (int, float))
                and 40.0 <= float(motion_publish["mean_rate_hz"]) <= 60.0
                and isinstance(
                    motion_publish.get("maximum_interval_ms"), (int, float)
                )
                and float(motion_publish["maximum_interval_ms"]) <= 100.0
            )
        )
    )
    if args.mode == "stop":
        stop_evidence = summary.get("stop_evidence")
        transition_ok = bool(
            stop_zero_sample is not None
            and isinstance(services.get("test_stop"), dict)
            and services["test_stop"].get("success") is True
            and isinstance(summary.get("observed"), dict)
            and summary["observed"].get("fault_codes") == [0]
            and isinstance(stop_evidence, dict)
            and stop_evidence.get(
                "explicit_stop_injected_before_stale_budget"
            )
            is True
            and isinstance(
                stop_evidence.get("last_command_to_first_zero_local_ms"),
                (int, float),
            )
            and 0.0
            <= float(stop_evidence["last_command_to_first_zero_local_ms"])
            <= STOP_ZERO_LATENCY_UPPER_MS
        )
    else:
        watchdog_evidence = summary.get("watchdog_evidence")
        watchdog_event_latency_ms = (
            watchdog_evidence.get("last_command_to_fault_event_header_ms")
            if isinstance(watchdog_evidence, dict)
            else None
        )
        transition_ok = bool(
            watchdog_confirmed
            and watchdog_status_sample is not None
            and watchdog_event_sample is not None
            and watchdog_motor_sample is not None
            and watchdog_relay_sample is not None
            and isinstance(services.get("reset_induced_fault"), dict)
            and services["reset_induced_fault"].get("success") is True
            and isinstance(watchdog_evidence, dict)
            and int(watchdog_evidence.get("watchdog_counter_delta", 0))
            >= 1
            and isinstance(watchdog_event_latency_ms, (int, float))
            and WATCHDOG_EVENT_LATENCY_LOWER_MS
            <= float(watchdog_event_latency_ms)
            <= WATCHDOG_EVENT_LATENCY_UPPER_MS
            and isinstance(services.get("post_reset_stop"), dict)
            and services["post_reset_stop"].get("success") is True
            and isinstance(services.get("post_reset_disarm"), dict)
            and services["post_reset_disarm"].get("success") is True
            and isinstance(summary.get("bridge_cut"), dict)
            and summary["bridge_cut"].get(
                "sigstop_injected_before_stale_budget"
            )
            is True
        )

    summary["interrupted_by_signal"] = interrupted_by
    summary["final_evidence_fresh"] = final_evidence_fresh
    summary["motion_evidence_present"] = motion_ok
    summary["publish_timing_ok"] = publish_timing_ok
    summary["relay_evidence_clean"] = relay_clean
    summary["final_safe"] = final_safe
    summary["elapsed_s"] = round(time.monotonic() - started_s, 6)
    summary["result"] = (
        "PASS"
        if arm_ok
        and cleanup_ok
        and transition_ok
        and motion_ok
        and publish_timing_ok
        and relay_clean
        and no_estop
        and final_evidence_fresh
        and final_safe
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
        _validate_command(args.linear_x, args.angular_z, args.duration)
        if args.mode == "watchdog":
            if args.bridge_pid is None:
                raise ValueError("--bridge-pid is required in watchdog mode")
            if not math.isfinite(args.bridge_hold) or not (
                MIN_BRIDGE_HOLD_S <= args.bridge_hold <= MAX_BRIDGE_HOLD_S
            ):
                raise ValueError(
                    f"--bridge-hold must be in [{MIN_BRIDGE_HOLD_S:g}, "
                    f"{MAX_BRIDGE_HOLD_S:g}] seconds"
                )
        elif args.bridge_pid is not None:
            raise ValueError("--bridge-pid is valid only in watchdog mode")
    except ValueError as exc:
        parser.error(str(exc))

    summary, return_code = run(args)
    print(json.dumps(summary, sort_keys=True, separators=(",", ":")), flush=True)
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
