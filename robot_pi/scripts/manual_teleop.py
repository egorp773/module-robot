#!/usr/bin/env python3
"""Fail-closed keyboard teleop for the Raspberry Pi manual bringup.

Terminals do not report key-release events.  Motion therefore requires a
stream of repeated W/S/A/D keypresses: if they stop, the local deadman expires
and this process immediately resumes publishing a zero command.
"""

from __future__ import annotations

import argparse
import os
import secrets
import select
import signal
import sys
import termios
import time
import tty
from typing import Optional, Tuple

from geometry_msgs.msg import TwistStamped
from module_robot_msgs.msg import MotorStatus, RelayStatus, RobotStatus, SafetyState
from module_robot_msgs.srv import Arm, Disarm
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger


MAX_LINEAR_MPS = 0.05
MAX_ANGULAR_RAD_S = 0.25
DEFAULT_LINEAR_MPS = 0.03
DEFAULT_ANGULAR_RAD_S = 0.15
DEADMAN_TIMEOUT_S = 0.25
PUBLISH_PERIOD_S = 0.02
DISPLAY_PERIOD_S = 0.20
TELEMETRY_TIMEOUT_S = 0.75
SERVICE_TIMEOUT_S = 3.0
STARTUP_TIMEOUT_S = 5.0
ZERO_SERVICE_ATTEMPTS = 3
ZERO_SERVICE_RETRY_INTERVAL_S = 0.15


ROBOT_STATE_NAMES = {
    RobotStatus.STATE_BOOT: "BOOT",
    RobotStatus.STATE_DISCONNECTED: "DISCONNECTED",
    RobotStatus.STATE_DISARMED: "DISARMED",
    RobotStatus.STATE_ARMED: "ARMED",
    RobotStatus.STATE_FAULT: "FAULT",
    RobotStatus.STATE_ESTOP: "ESTOP",
}

SAFETY_STATE_NAMES = {
    SafetyState.STATE_DISCONNECTED: "DISCONNECTED",
    SafetyState.STATE_DISARMED: "DISARMED",
    SafetyState.STATE_MANUAL: "MANUAL",
    SafetyState.STATE_AUTO: "AUTO",
    SafetyState.STATE_FAULT: "FAULT",
    SafetyState.STATE_ESTOP: "ESTOP",
}


class TerminalClosed(RuntimeError):
    """Raised when the controlling SSH terminal disappears."""


class TerminationRequested(BaseException):
    """Carry a terminating signal through normal fail-safe cleanup."""

    def __init__(self, signum: int) -> None:
        super().__init__(signum)
        self.signum = signum


class TerminalKeys:
    """Put a POSIX TTY in cbreak mode and return available key bytes."""

    def __init__(self, stream) -> None:
        self._stream = stream
        self._fd = stream.fileno()
        self._saved_attributes = None
        self._escape_state = "normal"
        self._osc_escaped = False

    def __enter__(self) -> "TerminalKeys":
        if not self._stream.isatty():
            raise RuntimeError("stdin is not a TTY; run manual_teleop.sh interactively")
        self._saved_attributes = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        del exc_type, exc_value, traceback
        if self._saved_attributes is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved_attributes)

    def read(self, timeout_s: float) -> str:
        ready, _, _ = select.select([self._fd], [], [], max(0.0, timeout_s))
        if not ready:
            return ""
        data = os.read(self._fd, 64)
        if not data:
            raise TerminalClosed("controlling terminal closed")
        return self._without_terminal_escape_sequences(data).decode(
            "ascii", errors="ignore"
        )

    def _without_terminal_escape_sequences(self, data: bytes) -> bytes:
        """Drop complete or fragmented ANSI/VT control sequences.

        Arrow keys and mouse reports contain ordinary letters such as A, D,
        and M.  Passing those bytes to the command mapper could otherwise turn
        an unrelated terminal control sequence into motion or ARM authority.
        """

        output = bytearray()
        for value in data:
            if self._escape_state == "normal":
                if value == 0x1B:
                    self._escape_state = "escape"
                elif value == 0x9B:
                    self._escape_state = "csi"
                elif value == 0x9D:
                    self._escape_state = "osc"
                    self._osc_escaped = False
                elif value in (0x90, 0x98, 0x9E, 0x9F):
                    self._escape_state = "string"
                    self._osc_escaped = False
                else:
                    output.append(value)
            elif self._escape_state == "escape":
                if value == 0x1B:
                    continue
                if value == ord("["):
                    self._escape_state = "csi"
                elif value == ord("O"):
                    self._escape_state = "ss3"
                elif value == ord("]"):
                    self._escape_state = "osc"
                    self._osc_escaped = False
                elif value in (ord("P"), ord("X"), ord("^"), ord("_")):
                    self._escape_state = "string"
                    self._osc_escaped = False
                else:
                    # A two-byte escape sequence ends here.  Both bytes are
                    # deliberately ignored, including a following command key.
                    self._escape_state = "normal"
            elif self._escape_state == "csi":
                if value == 0x1B:
                    self._escape_state = "escape"
                elif 0x40 <= value <= 0x7E:
                    self._escape_state = "normal"
            elif self._escape_state == "ss3":
                self._escape_state = "escape" if value == 0x1B else "normal"
            elif self._escape_state == "osc":
                # OSC accepts BEL or the string terminator ESC backslash.
                if value in (0x07, 0x9C):
                    self._escape_state = "normal"
                    self._osc_escaped = False
                elif self._osc_escaped and value == ord("\\"):
                    self._escape_state = "normal"
                    self._osc_escaped = False
                else:
                    self._osc_escaped = value == 0x1B
            else:  # DCS/SOS/PM/APC: discard through ST (ESC backslash).
                if value == 0x9C or (
                    self._osc_escaped and value == ord("\\")
                ):
                    self._escape_state = "normal"
                    self._osc_escaped = False
                else:
                    self._osc_escaped = value == 0x1B
        return bytes(output)


class ManualTeleop(Node):
    """ROS boundary for conservative, explicitly armed manual control."""

    def __init__(self, linear_speed: float, angular_speed: float) -> None:
        super().__init__("manual_keyboard_teleop")
        self._linear_speed = linear_speed
        self._angular_speed = angular_speed

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._command_pub = self.create_publisher(
            TwistStamped, "/cmd_vel_manual", command_qos
        )
        self._arm_client = self.create_client(Arm, "/safety/arm")
        self._stop_client = self.create_client(Trigger, "/safety/stop")
        self._disarm_client = self.create_client(Disarm, "/safety/disarm")

        self._robot: Optional[RobotStatus] = None
        self._safety: Optional[SafetyState] = None
        self._motor: Optional[MotorStatus] = None
        self._relays: Optional[RelayStatus] = None
        self._robot_rx_s = float("-inf")
        self._safety_rx_s = float("-inf")
        self._motor_rx_s = float("-inf")
        self._relays_rx_s = float("-inf")

        self.create_subscription(
            RobotStatus, "/esp32/status", self._on_robot, state_qos
        )
        self.create_subscription(
            SafetyState, "/safety/state", self._on_safety, latched_qos
        )
        self.create_subscription(
            MotorStatus, "/motor/status", self._on_motor, qos_profile_sensor_data
        )
        self.create_subscription(
            RelayStatus, "/relay/status", self._on_relays, state_qos
        )

        self._requested_linear = 0.0
        self._requested_angular = 0.0
        self._published_linear = 0.0
        self._published_angular = 0.0
        self._deadman_deadline_s = float("-inf")
        self._next_publish_s = time.monotonic()
        self._next_display_s = time.monotonic()
        self._quit = False
        self._shutdown_done = False
        self._line_visible = False
        self._last_arm_key_s = float("-inf")
        self._last_stop_key_s = float("-inf")
        self._last_disarm_key_s = float("-inf")
        self._interlock_reason = ""

    def _on_robot(self, message: RobotStatus) -> None:
        self._robot = message
        self._robot_rx_s = time.monotonic()

    def _on_safety(self, message: SafetyState) -> None:
        self._safety = message
        self._safety_rx_s = time.monotonic()

    def _on_motor(self, message: MotorStatus) -> None:
        self._motor = message
        self._motor_rx_s = time.monotonic()

    def _on_relays(self, message: RelayStatus) -> None:
        self._relays = message
        self._relays_rx_s = time.monotonic()

    @staticmethod
    def _fresh(received_s: float, now_s: float) -> bool:
        return now_s - received_s <= TELEMETRY_TIMEOUT_S

    def _motion_interlock(self, now_s: float) -> Tuple[bool, str]:
        if self._robot is None or not self._fresh(self._robot_rx_s, now_s):
            return False, "ESP32 status missing/stale"
        if self._safety is None or not self._fresh(self._safety_rx_s, now_s):
            return False, "Safety status missing/stale"
        if self._motor is None or not self._fresh(self._motor_rx_s, now_s):
            return False, "motor feedback missing/stale"
        if self._relays is None or not self._fresh(self._relays_rx_s, now_s):
            return False, "relay status missing/stale"
        if not self._robot.connected:
            return False, "ESP32 disconnected"
        if self._robot.estop:
            return False, "ESTOP active"
        if self._robot.fault_code != 0:
            return False, "ESP32 fault={}".format(self._robot.fault_code)
        if self._robot.state != RobotStatus.STATE_ARMED or not self._robot.armed:
            return False, "ESP32 is not ARMED"
        if self._robot.last_motor_feedback_age_ms > 500:
            return False, "motor feedback age >500 ms"
        if self._motor.controller_fault != 0:
            return False, "motor controller fault={}".format(
                self._motor.controller_fault
            )
        if self._relays.active_mask != 0:
            return False, "relay active mask is non-zero"
        if self._safety.state != SafetyState.STATE_MANUAL:
            return False, "Safety is not in MANUAL"
        if not self._safety.operator_armed:
            return False, "MANUAL ARM is not confirmed"
        if self._safety.latched_fault:
            return False, "Safety fault latched"
        return True, "READY"

    def _prearm_interlock(self, now_s: float) -> Tuple[bool, str]:
        if self._robot is None or not self._fresh(self._robot_rx_s, now_s):
            return False, "ESP32 status missing/stale"
        if self._safety is None or not self._fresh(self._safety_rx_s, now_s):
            return False, "Safety status missing/stale"
        if self._motor is None or not self._fresh(self._motor_rx_s, now_s):
            return False, "motor feedback missing/stale"
        if self._relays is None or not self._fresh(self._relays_rx_s, now_s):
            return False, "relay status missing/stale"
        if not self._robot.connected:
            return False, "ESP32 disconnected"
        if self._robot.state != RobotStatus.STATE_DISARMED or self._robot.armed:
            return False, "ESP32 must be DISARMED"
        if self._robot.estop:
            return False, "ESTOP active"
        if self._robot.fault_code != 0:
            return False, "ESP32 fault={}".format(self._robot.fault_code)
        if self._robot.applied_left_command != 0 or self._robot.applied_right_command != 0:
            return False, "applied motor command is non-zero"
        if self._robot.uart_speed != 0 or self._robot.uart_steer != 0:
            return False, "motor UART command is non-zero"
        if self._robot.last_motor_feedback_age_ms > 500:
            return False, "motor feedback age >500 ms"
        if self._motor.controller_fault != 0:
            return False, "motor controller fault={}".format(
                self._motor.controller_fault
            )
        if self._relays.active_mask != 0:
            return False, "relay active mask is non-zero"
        if self._safety.latched_fault:
            return False, "Safety fault latched"
        return True, "READY"

    def _publish(self, linear: float, angular: float) -> None:
        # Clamp here as a second line of defence even though all key mappings
        # are constructed from already validated constants.
        linear = max(-MAX_LINEAR_MPS, min(MAX_LINEAR_MPS, float(linear)))
        angular = max(-MAX_ANGULAR_RAD_S, min(MAX_ANGULAR_RAD_S, float(angular)))
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.twist.linear.x = linear
        message.twist.angular.z = angular
        self._command_pub.publish(message)
        self._published_linear = linear
        self._published_angular = angular

    def _clear_requested_command(self) -> None:
        self._requested_linear = 0.0
        self._requested_angular = 0.0
        self._deadman_deadline_s = float("-inf")

    def _publish_zero_burst(self) -> None:
        self._clear_requested_command()
        for _ in range(3):
            self._publish(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)

    def _call_service(self, client, request, label: str, timeout_s: float):
        self._publish(0.0, 0.0)
        if not client.wait_for_service(timeout_sec=min(0.5, timeout_s)):
            self._print_event("{} unavailable".format(label))
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            self._publish(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done():
            future.cancel()
            self._print_event("{} timed out".format(label))
            return None
        try:
            return future.result()
        except Exception as exc:  # rclpy transports executor exceptions here
            self._print_event("{} failed: {}".format(label, exc))
            return None

    def _zero_service_with_retries(self, client, request_type, label: str):
        """Call a zero-only service with bounded retries while holding zero.

        STOP and DISARM are idempotent safety operations.  A response can be
        rejected transiently while the preceding zero-only operation is still
        being applied, so retry without ever restoring motion authority.
        """
        last_response = None
        for attempt in range(1, ZERO_SERVICE_ATTEMPTS + 1):
            self._publish_zero_burst()
            last_response = self._call_service(
                client, request_type(), label, SERVICE_TIMEOUT_S
            )
            if last_response is not None and last_response.success:
                return last_response
            if attempt == ZERO_SERVICE_ATTEMPTS:
                break

            result = (
                "no acknowledgement"
                if last_response is None
                else "rejected: {}".format(last_response.message)
            )
            self._print_event(
                "{} attempt {}/{} {}; retrying with zero held".format(
                    label, attempt, ZERO_SERVICE_ATTEMPTS, result
                )
            )
            retry_deadline = time.monotonic() + ZERO_SERVICE_RETRY_INTERVAL_S
            while rclpy.ok() and time.monotonic() < retry_deadline:
                self._publish(0.0, 0.0)
                remaining_s = retry_deadline - time.monotonic()
                rclpy.spin_once(
                    self, timeout_sec=max(0.0, min(PUBLISH_PERIOD_S, remaining_s))
                )
        return last_response

    def _stop(self, announce: bool = True):
        response = self._zero_service_with_retries(
            self._stop_client, Trigger.Request, "/safety/stop"
        )
        if announce:
            if response is not None:
                self._print_event(
                    "STOP: {} ({})".format(
                        "ACK" if response.success else "REJECTED", response.message
                    )
                )
            else:
                self._print_event("STOP: no acknowledgement; zero publishing continues")
        return response

    def _disarm(self, announce: bool = True):
        response = self._zero_service_with_retries(
            self._disarm_client,
            Disarm.Request,
            "/safety/disarm",
        )
        if announce:
            if response is not None:
                self._print_event(
                    "DISARM: {} ({})".format(
                        "ACK" if response.success else "REJECTED", response.message
                    )
                )
            else:
                self._print_event("DISARM: no acknowledgement")
        return response

    def _arm_manual(self) -> None:
        now_s = time.monotonic()
        if now_s - self._last_arm_key_s < 1.0:
            return
        self._last_arm_key_s = now_s
        self._publish_zero_burst()
        permitted, reason = self._prearm_interlock(time.monotonic())
        if not permitted:
            self._print_event("MANUAL ARM blocked: {}".format(reason))
            return

        request = Arm.Request()
        request.arm_nonce = secrets.randbits(32) or 1
        request.requested_mode = Arm.Request.MODE_MANUAL
        response = self._call_service(
            self._arm_client, request, "/safety/arm", SERVICE_TIMEOUT_S
        )
        if response is None:
            self._print_event("MANUAL ARM: no acknowledgement; forcing STOP + DISARM")
            self._stop(announce=False)
            self._disarm(announce=False)
            return
        self._print_event(
            "MANUAL ARM: {} ({})".format(
                "ACK" if response.success else "REJECTED", response.message
            )
        )
        if not response.success:
            self._stop(announce=False)
            self._disarm(announce=False)

    def _request_motion(self, linear: float, angular: float) -> None:
        now_s = time.monotonic()
        permitted, reason = self._motion_interlock(now_s)
        if not permitted:
            self._clear_requested_command()
            self._publish(0.0, 0.0)
            if reason != self._interlock_reason:
                self._print_event("Motion blocked: {}".format(reason))
                self._interlock_reason = reason
            return
        self._interlock_reason = ""
        self._requested_linear = linear
        self._requested_angular = angular
        self._deadman_deadline_s = now_s + DEADMAN_TIMEOUT_S

    def handle_keys(self, keys: str) -> None:
        for key in keys:
            if key == "\x03" or key.lower() == "q":
                self._clear_requested_command()
                self._publish(0.0, 0.0)
                self._quit = True
                return
            if key.lower() == "m":
                self._arm_manual()
            elif key.lower() == "w":
                self._request_motion(self._linear_speed, 0.0)
            elif key.lower() == "s":
                self._request_motion(-self._linear_speed, 0.0)
            elif key.lower() == "a":
                self._request_motion(0.0, self._angular_speed)
            elif key.lower() == "d":
                self._request_motion(0.0, -self._angular_speed)
            elif key.lower() == "x":
                self._clear_requested_command()
                self._publish(0.0, 0.0)
            elif key == " ":
                now_s = time.monotonic()
                if now_s - self._last_stop_key_s >= 0.5:
                    self._last_stop_key_s = now_s
                    self._stop()
            elif key.lower() == "k":
                now_s = time.monotonic()
                if now_s - self._last_disarm_key_s >= 0.75:
                    self._last_disarm_key_s = now_s
                    self._stop(announce=False)
                    self._disarm()

    def tick(self) -> None:
        now_s = time.monotonic()
        if now_s >= self._next_publish_s:
            desired_active = (
                now_s < self._deadman_deadline_s
                and (
                    self._requested_linear != 0.0
                    or self._requested_angular != 0.0
                )
            )
            permitted, reason = self._motion_interlock(now_s)
            if desired_active and permitted:
                self._publish(self._requested_linear, self._requested_angular)
                self._interlock_reason = ""
            else:
                if desired_active and not permitted and reason != self._interlock_reason:
                    self._print_event("Motion interlock: {}".format(reason))
                    self._interlock_reason = reason
                if not desired_active:
                    self._clear_requested_command()
                self._publish(0.0, 0.0)
            self._next_publish_s = now_s + PUBLISH_PERIOD_S

        if now_s >= self._next_display_s:
            self._render_status(now_s)
            self._next_display_s = now_s + DISPLAY_PERIOD_S

    def _status_text(self, now_s: float) -> str:
        deadman = (
            "ACTIVE"
            if now_s < self._deadman_deadline_s
            and (self._requested_linear != 0.0 or self._requested_angular != 0.0)
            else "idle"
        )
        if self._robot is None:
            robot_text = "esp32=NO_DATA"
        else:
            robot_text = (
                "esp32={} conn={} arm={} fault={} out=({:+d},{:+d}) "
                "uart=({:+d},{:+d}) motor_age={}ms"
            ).format(
                ROBOT_STATE_NAMES.get(self._robot.state, str(self._robot.state)),
                "Y" if self._robot.connected else "N",
                "Y" if self._robot.armed else "N",
                self._robot.fault_code,
                self._robot.applied_left_command,
                self._robot.applied_right_command,
                self._robot.uart_speed,
                self._robot.uart_steer,
                self._robot.last_motor_feedback_age_ms,
            )
        if self._safety is None:
            safety_text = "safety=NO_DATA"
        else:
            safety_text = "safety={} op_arm={} candidate={}".format(
                SAFETY_STATE_NAMES.get(self._safety.state, str(self._safety.state)),
                "Y" if self._safety.operator_armed else "N",
                self._safety.current_candidate_status,
            )
        if self._motor is None:
            motor_text = "feedback=NO_DATA"
        else:
            motor_text = "feedback=({:+d},{:+d}) ctrl_fault={}".format(
                self._motor.left_feedback,
                self._motor.right_feedback,
                self._motor.controller_fault,
            )
        relay_mask = "?" if self._relays is None else "0x{:X}".format(
            self._relays.active_mask
        )
        return (
            "cmd=({:+.3f} m/s,{:+.3f} rad/s) deadman={} | {} | {} | {} | relays={}"
        ).format(
            self._published_linear,
            self._published_angular,
            deadman,
            safety_text,
            robot_text,
            motor_text,
            relay_mask,
        )

    def _render_status(self, now_s: float) -> None:
        try:
            sys.stdout.write("\r\033[2K" + self._status_text(now_s))
            sys.stdout.flush()
            self._line_visible = True
        except (BrokenPipeError, OSError):
            self._quit = True

    def _print_event(self, text: str) -> None:
        try:
            if self._line_visible:
                sys.stdout.write("\r\033[2K")
            print(text, flush=True)
            self._line_visible = False
            self._next_display_s = time.monotonic()
        except (BrokenPipeError, OSError):
            self._quit = True

    def _spin_until(self, predicate, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return True
            self._publish(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)
        return bool(predicate())

    def prepare(self) -> None:
        for client, label in (
            (self._arm_client, "/safety/arm"),
            (self._stop_client, "/safety/stop"),
            (self._disarm_client, "/safety/disarm"),
        ):
            if not client.wait_for_service(timeout_sec=STARTUP_TIMEOUT_S):
                raise RuntimeError("{} is unavailable; start manual bringup".format(label))

        if not self._spin_until(
            lambda: all(
                item is not None
                for item in (self._robot, self._safety, self._motor, self._relays)
            ),
            STARTUP_TIMEOUT_S,
        ):
            raise RuntimeError("required ESP32/Safety/motor/relay telemetry is missing")

        self._print_event("Establishing zero, STOP, and DISARM before keyboard control")
        stop_response = self._stop(announce=False)
        if stop_response is None or not stop_response.success:
            raise RuntimeError("initial STOP was not acknowledged after retries")
        disarm_response = self._disarm(announce=False)
        if disarm_response is None or not disarm_response.success:
            raise RuntimeError("initial DISARM was not acknowledged after retries")

        def safe_disarmed() -> bool:
            if self._robot is None or self._relays is None:
                return False
            return (
                self._robot.connected
                and self._robot.state == RobotStatus.STATE_DISARMED
                and not self._robot.armed
                and not self._robot.estop
                and self._robot.fault_code == 0
                and self._robot.applied_left_command == 0
                and self._robot.applied_right_command == 0
                and self._robot.uart_speed == 0
                and self._robot.uart_steer == 0
                and self._relays.active_mask == 0
            )

        if not self._spin_until(safe_disarmed, STARTUP_TIMEOUT_S):
            raise RuntimeError(
                "ESP32 did not confirm safe DISARMED/zero state; see live status"
            )

    def print_help(self) -> None:
        print(
            "\nManual keyboard teleop (MANUAL only, relays untouched)\n"
            "  m       explicit MANUAL ARM\n"
            "  w / s   forward / reverse while keypresses repeat\n"
            "  a / d   left / right turn while keypresses repeat\n"
            "  x       zero command (ARM remains)\n"
            "  SPACE   zero + software STOP (ARM normally remains)\n"
            "  k       STOP + DISARM\n"
            "  q       STOP + DISARM + quit\n"
            "  Ctrl+C  STOP + DISARM + quit\n"
            "Deadman: zero {:.0f} ms after the last W/S/A/D keypress. "
            "Limits: {:.3f} m/s, {:.3f} rad/s.\n".format(
                DEADMAN_TIMEOUT_S * 1000.0,
                self._linear_speed,
                self._angular_speed,
            ),
            flush=True,
        )

    def run(self, terminal: TerminalKeys) -> None:
        self.print_help()
        while rclpy.ok() and not self._quit:
            rclpy.spin_once(self, timeout_sec=0.0)
            keys = terminal.read(0.01)
            if keys:
                self.handle_keys(keys)
            if self._quit:
                break
            self.tick()

    def _confirmed_safe_disarmed(self) -> bool:
        if self._robot is None or self._relays is None:
            return False
        return bool(
            self._robot.connected
            and self._robot.state == RobotStatus.STATE_DISARMED
            and not self._robot.armed
            and not self._robot.estop
            and self._robot.fault_code == 0
            and self._robot.applied_left_command == 0
            and self._robot.applied_right_command == 0
            and self._robot.uart_speed == 0
            and self._robot.uart_steer == 0
            and self._relays.active_mask == 0
        )

    def safe_shutdown(self) -> bool:
        if self._shutdown_done:
            return self._confirmed_safe_disarmed()
        self._shutdown_done = True
        self._clear_requested_command()
        warnings = []

        try:
            self._print_event("Fail-safe shutdown: zero + STOP + DISARM")
            self._publish_zero_burst()
        except BaseException as exc:
            warnings.append("initial zero failed: {}".format(exc))

        stop_response = None
        try:
            stop_response = self._stop(announce=False)
            if stop_response is None or not stop_response.success:
                warnings.append("STOP was not acknowledged")
        except BaseException as exc:
            warnings.append("STOP failed: {}".format(exc))

        disarm_response = None
        try:
            disarm_response = self._disarm(announce=False)
            if disarm_response is None or not disarm_response.success:
                warnings.append("DISARM was not acknowledged")
        except BaseException as exc:
            warnings.append("DISARM failed: {}".format(exc))

        try:
            self._publish_zero_burst()
            if not self._spin_until(self._confirmed_safe_disarmed, STARTUP_TIMEOUT_S):
                warnings.append("final DISARMED/zero status was not confirmed")
        except BaseException as exc:
            warnings.append("final zero/status confirmation failed: {}".format(exc))

        for warning in warnings:
            self._print_event("Shutdown warning: {}".format(warning))
        return not warnings and self._confirmed_safe_disarmed()


def _limited_positive(value: str, name: str, maximum: float) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("{} must be a number".format(name)) from exc
    if not 0.0 < parsed <= maximum:
        raise argparse.ArgumentTypeError(
            "{} must be in (0, {}]".format(name, maximum)
        )
    return parsed


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Fail-closed keyboard teleop for manual_bringup"
    )
    parser.add_argument(
        "--linear-speed",
        type=lambda value: _limited_positive(value, "linear speed", MAX_LINEAR_MPS),
        default=DEFAULT_LINEAR_MPS,
        help="absolute forward/reverse speed, maximum 0.05 m/s (default: 0.03)",
    )
    parser.add_argument(
        "--angular-speed",
        type=lambda value: _limited_positive(
            value, "angular speed", MAX_ANGULAR_RAD_S
        ),
        default=DEFAULT_ANGULAR_RAD_S,
        help="absolute turn rate, maximum 0.25 rad/s (default: 0.15)",
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = _parse_args(argv)
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        print("manual_teleop requires an interactive TTY", file=sys.stderr)
        return 2

    node = None
    exit_code = 0
    previous_handlers = {}

    def terminate(signum, frame) -> None:
        del frame
        raise TerminationRequested(signum)

    for signum in (signal.SIGINT, signal.SIGHUP, signal.SIGTERM):
        previous_handlers[signum] = signal.signal(signum, terminate)

    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = ManualTeleop(args.linear_speed, args.angular_speed)
        node.prepare()
        with TerminalKeys(sys.stdin) as terminal:
            node.run(terminal)
    except KeyboardInterrupt:
        exit_code = 130
    except TerminationRequested as exc:
        exit_code = 128 + exc.signum
    except (ExternalShutdownException, TerminalClosed) as exc:
        if node is not None:
            node._print_event(str(exc))
        exit_code = 1
    except Exception as exc:
        print("manual_teleop: ERROR: {}".format(exc), file=sys.stderr, flush=True)
        exit_code = 1
    finally:
        # A second terminal signal must not interrupt STOP/DISARM cleanup.
        for signum in previous_handlers:
            signal.signal(signum, signal.SIG_IGN)
        if node is not None:
            if not node.safe_shutdown() and exit_code == 0:
                exit_code = 1
            try:
                node.destroy_node()
            except BaseException as exc:
                print(
                    "manual_teleop: node cleanup warning: {}".format(exc),
                    file=sys.stderr,
                    flush=True,
                )
                if exit_code == 0:
                    exit_code = 1
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except BaseException as exc:
            print(
                "manual_teleop: ROS shutdown warning: {}".format(exc),
                file=sys.stderr,
                flush=True,
            )
            if exit_code == 0:
                exit_code = 1
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
