"""ROS 2 node owning the only Raspberry Pi serial connection to the ESP32."""

from __future__ import annotations

import math
import secrets
import threading
import time
from typing import Optional

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped
from module_robot_msgs.msg import (
    FaultEvent,
    MotorStatus,
    PowerStatus,
    ProtocolStats,
    RelayStatus,
    RobotStatus,
    RtkStatus,
    SafetyState,
)
from module_robot_msgs.srv import Arm, Disarm, ResetEstop, ResetFault, SetRelays
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
import serial
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from .bridge_state import CommandGate, ReconnectBackoff
from .payloads import (
    PayloadError,
    pack_arm,
    pack_cmd_vel,
    pack_heartbeat,
    pack_hello,
    pack_relay_command,
    pack_time_sync_request,
    unpack_command_ack,
    unpack_diagnostics,
    unpack_estop_event,
    unpack_fault_event,
    unpack_gnss,
    unpack_hello_ack,
    unpack_imu,
    unpack_motor_feedback,
    unpack_power_status,
    unpack_relay_status,
    unpack_status,
    unpack_time_sync_response,
)
from .protocol import Frame, FrameDecoder, MessageType, PROTOCOL_VERSION, encode_frame, sequence_is_newer


FAULT_NAMES = {
    0: "NONE",
    1: "CMD_VEL_TIMEOUT",
    2: "MOTOR_FEEDBACK_UNAVAILABLE",
    3: "MOTOR_FEEDBACK_LOST",
    4: "MOTOR_CONTROLLER_FAULT",
    5: "INVALID_MOTOR_COMMAND",
    6: "INTERNAL_WATCHDOG",
}

POWER_FLAG_VOLTAGE_VALID = 1 << 0
POWER_FLAG_UNDERVOLTAGE = 1 << 1
POWER_FLAG_IMU_UNAVAILABLE = 1 << 8
POWER_FLAG_GNSS_UNAVAILABLE = 1 << 9
POWER_FLAG_MOTOR_FEEDBACK_STALE = 1 << 10
POWER_FLAG_MOTOR_CONTROLLER_FAULT = 1 << 11
POWER_FLAG_UBX_CHECKSUM_ERROR_SEEN = 1 << 12
POWER_FLAG_UBX_OVERSIZED_FRAME_SEEN = 1 << 13
POWER_FLAG_INVALID_HOVERBOARD_FRAME_SEEN = 1 << 14


class Esp32Bridge(Node):
    def __init__(self) -> None:
        super().__init__("esp32_bridge")
        self._declare_parameters()
        self._read_parameters()
        self._io_callback_group = MutuallyExclusiveCallbackGroup()
        self._control_callback_group = MutuallyExclusiveCallbackGroup()
        self._service_callback_group = MutuallyExclusiveCallbackGroup()
        # STOP/DISARM/ESTOP must not queue behind an ARM, reset or relay ACK
        # timeout. Their handlers only remove motion authority and `_send` is
        # serialized independently.
        self._zero_only_callback_group = ReentrantCallbackGroup()

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        state_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._imu_pub = self.create_publisher(Imu, "/imu/data_raw", sensor_qos)
        self._gps_pub = self.create_publisher(NavSatFix, "/gps/fix", sensor_qos)
        self._rtk_pub = self.create_publisher(RtkStatus, "/rtk/status", state_qos)
        self._motor_pub = self.create_publisher(MotorStatus, "/motor/status", sensor_qos)
        self._power_pub = self.create_publisher(PowerStatus, "/power/status", state_qos)
        self._relay_pub = self.create_publisher(RelayStatus, "/relay/status", state_qos)
        self._status_pub = self.create_publisher(RobotStatus, "/esp32/status", state_qos)
        self._stats_pub = self.create_publisher(ProtocolStats, "/esp32/protocol_stats", state_qos)
        self._fault_pub = self.create_publisher(FaultEvent, "/esp32/fault_event", state_qos)
        self._estop_pub = self.create_publisher(Bool, "/esp32/estop_event", state_qos)
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", state_qos)

        self.create_subscription(
            TwistStamped,
            "/cmd_vel_safe",
            self._on_cmd_vel,
            command_qos,
            callback_group=self._control_callback_group,
        )
        self.create_subscription(
            SafetyState,
            "/safety/state",
            self._on_safety_state,
            state_qos,
            callback_group=self._control_callback_group,
        )

        self.create_service(
            Arm, "/esp32/arm", self._on_arm,
            callback_group=self._service_callback_group,
        )
        self.create_service(
            Disarm, "/esp32/disarm", self._on_disarm,
            callback_group=self._zero_only_callback_group,
        )
        self.create_service(
            Trigger, "/esp32/stop", self._on_stop,
            callback_group=self._zero_only_callback_group,
        )
        self.create_service(
            Trigger, "/esp32/estop", self._on_estop,
            callback_group=self._zero_only_callback_group,
        )
        self.create_service(
            ResetFault, "/esp32/reset_fault", self._on_reset_fault,
            callback_group=self._service_callback_group,
        )
        self.create_service(
            ResetEstop, "/esp32/reset_estop", self._on_reset_estop,
            callback_group=self._service_callback_group,
        )
        self.create_service(
            SetRelays, "/esp32/set_relays", self._on_set_relays,
            callback_group=self._service_callback_group,
        )

        self._serial: Optional[serial.Serial] = None
        self._serial_lock = threading.Lock()
        self._tx_lock = threading.Lock()
        self._motion_lock = threading.RLock()
        self._motion_epoch = 0
        self._pending_acks_lock = threading.Lock()
        self._pending_acks = {}
        self._decoder = FrameDecoder(self._protocol_version)
        self._backoff = ReconnectBackoff(self._reconnect_delay_s)
        self._tx_sequence = secrets.randbits(32)
        self._last_rx_sequence: Optional[int] = None
        self._duplicate_sequences = 0
        self._out_of_order_sequences = 0
        self._rx_bytes = 0
        self._tx_bytes = 0
        self._reconnects = 0
        self._handshake_failures = 0
        self._payload_errors = 0
        self._esp_diagnostics = None
        self._crc_baseline = 0
        self._handshake_complete = False
        self._connection_opened_s = 0.0
        self._last_hello_s = 0.0
        self._last_time_sync_s = 0.0
        self._last_status_request_s = 0.0
        self._last_valid_frame_s = 0.0
        self._esp_to_pi_offset_us: Optional[float] = None
        self._last_status = RobotStatus()
        self._last_status.state = RobotStatus.STATE_DISCONNECTED
        self._last_status_received_s = float("-inf")
        self._last_power_flags = 0
        self._last_imu_calibration_status: Optional[int] = None
        self._last_imu_accuracy_rad = math.nan
        self._last_imu_sequence = 0
        self._arming_requested = False
        self._arm_request_in_progress = False
        self._safety_state = SafetyState.STATE_DISCONNECTED
        self._armed_control_mode = 0
        self._arm_ack_confirmed = False
        self._esp_status_armed = False
        self._stop_sent_for_stale = False
        self._last_relay_active_mask = 0
        self._shutting_down = False
        self._gate = CommandGate(self._ros_cmd_stale_ms / 1000.0)

        self._io_timer = self.create_timer(
            0.01, self._io_tick, callback_group=self._io_callback_group
        )
        self._command_timer = self.create_timer(
            1.0 / self._command_rate_hz,
            self._command_tick,
            callback_group=self._control_callback_group,
        )
        self._heartbeat_timer = self.create_timer(
            1.0 / self._heartbeat_rate_hz,
            self._heartbeat_tick,
            callback_group=self._control_callback_group,
        )
        self._diagnostic_timer = self.create_timer(1.0, self._publish_health)
        self.get_logger().info(f"ESP32 bridge ready for {self._device} at {self._baud} baud; automatic ARM is disabled")

    def _declare_parameters(self) -> None:
        defaults = {
            "device": "/dev/module-esp32",
            "baud": 460800,
            "command_rate_hz": 50.0,
            "command_timeout_ms": 300,
            "ros_cmd_stale_ms": 250,
            "reconnect_delay_s": 2.0,
            "protocol_version": PROTOCOL_VERSION,
            "max_crc_errors": 20,
            "heartbeat_rate_hz": 2.0,
            "esp_heartbeat_timeout_ms": 1500,
            "time_sync_period_s": 10.0,
            "handshake_timeout_s": 3.0,
            "serial_liveness_timeout_s": 2.0,
            "command_ack_timeout_s": 0.5,
            "client_name": "module_robot_esp32_bridge",
            "client_build_id": "development",
            "requested_capabilities": 0x1F,
            "frame_id_imu": "imu_link",
            "frame_id_gps": "gps_link",
            "max_linear_mps": 0.15,
            "max_reverse_mps": 0.15,
            "max_angular_rad_s": 0.60,
            "vertical_accuracy_fallback_m": 100.0,
            "imu_orientation_variance": 0.06853892,
            "imu_angular_velocity_variance": 0.00761544,
            "imu_linear_acceleration_variance": 0.25,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self) -> None:
        value = lambda name: self.get_parameter(name).value
        self._device = str(value("device"))
        self._baud = int(value("baud"))
        self._command_rate_hz = float(value("command_rate_hz"))
        self._command_timeout_ms = int(value("command_timeout_ms"))
        self._ros_cmd_stale_ms = int(value("ros_cmd_stale_ms"))
        self._reconnect_delay_s = float(value("reconnect_delay_s"))
        self._protocol_version = int(value("protocol_version"))
        self._max_crc_errors = int(value("max_crc_errors"))
        self._heartbeat_rate_hz = float(value("heartbeat_rate_hz"))
        self._esp_heartbeat_timeout_ms = int(value("esp_heartbeat_timeout_ms"))
        self._time_sync_period_s = float(value("time_sync_period_s"))
        self._handshake_timeout_s = float(value("handshake_timeout_s"))
        self._serial_liveness_timeout_s = float(value("serial_liveness_timeout_s"))
        self._command_ack_timeout_s = float(value("command_ack_timeout_s"))
        self._client_name = str(value("client_name"))
        self._client_build_id = str(value("client_build_id"))
        self._requested_capabilities = int(value("requested_capabilities"))
        self._frame_id_imu = str(value("frame_id_imu"))
        self._frame_id_gps = str(value("frame_id_gps"))
        self._max_linear_mps = float(value("max_linear_mps"))
        self._max_reverse_mps = float(value("max_reverse_mps"))
        self._max_angular_rad_s = float(value("max_angular_rad_s"))
        self._vertical_accuracy_fallback_m = float(value("vertical_accuracy_fallback_m"))
        self._imu_orientation_variance = float(value("imu_orientation_variance"))
        self._imu_angular_velocity_variance = float(value("imu_angular_velocity_variance"))
        self._imu_linear_acceleration_variance = float(value("imu_linear_acceleration_variance"))
        if self._protocol_version != PROTOCOL_VERSION:
            raise ValueError(f"only protocol_version={PROTOCOL_VERSION} is implemented")
        finite_values = {
            "command_rate_hz": self._command_rate_hz,
            "reconnect_delay_s": self._reconnect_delay_s,
            "heartbeat_rate_hz": self._heartbeat_rate_hz,
            "time_sync_period_s": self._time_sync_period_s,
            "handshake_timeout_s": self._handshake_timeout_s,
            "serial_liveness_timeout_s": self._serial_liveness_timeout_s,
            "command_ack_timeout_s": self._command_ack_timeout_s,
            "max_linear_mps": self._max_linear_mps,
            "max_reverse_mps": self._max_reverse_mps,
            "max_angular_rad_s": self._max_angular_rad_s,
            "vertical_accuracy_fallback_m": self._vertical_accuracy_fallback_m,
            "imu_orientation_variance": self._imu_orientation_variance,
            "imu_angular_velocity_variance": self._imu_angular_velocity_variance,
            "imu_linear_acceleration_variance": self._imu_linear_acceleration_variance,
        }
        for name, parameter in finite_values.items():
            if not math.isfinite(parameter):
                raise ValueError(f"{name} must be finite")
        if not self._device:
            raise ValueError("device must not be empty")
        if not 9_600 <= self._baud <= 4_000_000:
            raise ValueError("baud must be in [9600, 4000000]")
        if not 20.0 <= self._command_rate_hz <= 100.0:
            raise ValueError("command_rate_hz must be in [20, 100]; use 50 Hz for commissioning")
        if not 0.5 <= self._heartbeat_rate_hz <= 50.0:
            raise ValueError("heartbeat_rate_hz must be in [0.5, 50]")
        if not 100 <= self._command_timeout_ms <= 300:
            raise ValueError("command_timeout_ms must be in [100, 300]")
        if 3000.0 / self._command_rate_hz >= self._command_timeout_ms:
            raise ValueError("command rate must provide at least three frames inside command_timeout_ms")
        if not 1 <= self._ros_cmd_stale_ms < self._command_timeout_ms:
            raise ValueError("ros_cmd_stale_ms must be positive and below ESP32 watchdog timeout")
        if (
            self._reconnect_delay_s <= 0.0
            or self._time_sync_period_s <= 0.0
            or self._handshake_timeout_s <= 0.0
            or self._serial_liveness_timeout_s <= 0.0
            or self._command_ack_timeout_s <= 0.0
        ):
            raise ValueError("reconnect, sync, handshake, liveness and ACK timeouts must be positive")
        if self._max_crc_errors <= 0:
            raise ValueError("max_crc_errors must be positive")
        if not 500 <= self._esp_heartbeat_timeout_ms <= 10_000:
            raise ValueError("esp_heartbeat_timeout_ms must be in [500, 10000]")
        if 2000.0 / self._heartbeat_rate_hz >= self._esp_heartbeat_timeout_ms:
            raise ValueError(
                "heartbeat_rate_hz must place at least two heartbeats inside "
                "esp_heartbeat_timeout_ms"
            )
        if not 0 <= self._requested_capabilities <= 0xFFFFFFFF:
            raise ValueError("requested_capabilities must fit uint32")
        if not 0.0 < self._max_linear_mps <= 0.15:
            raise ValueError("max_linear_mps must be in (0, 0.15]")
        if not 0.0 < self._max_reverse_mps <= 0.15:
            raise ValueError("max_reverse_mps must be in (0, 0.15]")
        if not 0.0 < self._max_angular_rad_s <= 0.60:
            raise ValueError("max_angular_rad_s must be in (0, 0.60]")
        if self._vertical_accuracy_fallback_m <= 0.0:
            raise ValueError("vertical_accuracy_fallback_m must be positive")
        if any(
            variance < 0.0
            for variance in (
                self._imu_orientation_variance,
                self._imu_angular_velocity_variance,
                self._imu_linear_acceleration_variance,
            )
        ):
            raise ValueError("IMU covariance variances must be non-negative")

    def _now_s(self) -> float:
        return time.monotonic()

    def _now_us(self) -> int:
        return time.monotonic_ns() // 1000

    def _clear_arm_authority(self) -> None:
        with self._motion_lock:
            self._motion_epoch += 1
            self._arming_requested = False
            self._arm_request_in_progress = False
            self._armed_control_mode = 0
            self._arm_ack_confirmed = False
            self._esp_status_armed = False
            self._gate.armed_confirmed = False
            self._gate.invalidate()

    def _refresh_motion_gate(self) -> None:
        with self._motion_lock:
            self._gate.armed_confirmed = bool(
                self._handshake_complete
                and self._arming_requested
                and self._arm_ack_confirmed
                and self._esp_status_armed
                and self._armed_control_mode in (
                    Arm.Request.MODE_MANUAL,
                    Arm.Request.MODE_AUTO,
                )
                and (
                    (
                        self._armed_control_mode == Arm.Request.MODE_MANUAL
                        and self._safety_state == SafetyState.STATE_MANUAL
                    )
                    or (
                        self._armed_control_mode == Arm.Request.MODE_AUTO
                        and self._safety_state == SafetyState.STATE_AUTO
                    )
                )
                and not self._last_status.estop
                and self._last_status.fault_code == 0
            )

    def _open_serial(self, now_s: float) -> None:
        if self._shutting_down or not self._backoff.ready(now_s):
            return
        try:
            port = serial.Serial(
                port=self._device,
                baudrate=self._baud,
                timeout=0,
                write_timeout=0.1,
                rtscts=False,
                dsrdtr=False,
                exclusive=True,
            )
            port.reset_input_buffer()
        except (OSError, serial.SerialException, ValueError) as exc:
            self._backoff.failed(now_s)
            self.get_logger().warning(f"serial connect failed: {exc}", throttle_duration_sec=5.0)
            return
        with self._serial_lock:
            self._serial = port
        self._decoder.reset()
        self._last_rx_sequence = None
        self._handshake_complete = False
        self._safety_state = SafetyState.STATE_DISCONNECTED
        self._clear_arm_authority()
        self._connection_opened_s = now_s
        self._last_valid_frame_s = now_s
        self._crc_baseline = self._decoder.stats.crc_errors
        self._esp_diagnostics = None
        self._last_imu_calibration_status = None
        self._last_imu_accuracy_rad = math.nan
        self._last_imu_sequence = 0
        self._reconnects += 1
        self._send_hello()
        self.get_logger().info(f"serial device opened: {self._device}")

    def _disconnect(self, reason: str) -> None:
        with self._serial_lock:
            port, self._serial = self._serial, None
        if port is not None:
            try:
                port.close()
            except (OSError, serial.SerialException):
                pass
        was_connected = self._handshake_complete
        self._handshake_complete = False
        self._safety_state = SafetyState.STATE_DISCONNECTED
        self._clear_arm_authority()
        self._stop_sent_for_stale = False
        self._esp_to_pi_offset_us = None
        self._backoff.failed(self._now_s())
        with self._pending_acks_lock:
            for event, _, _ in self._pending_acks.values():
                event.set()
        self._publish_disconnected_status()
        if was_connected:
            self.get_logger().error(f"ESP32 disconnected: {reason}")
        else:
            self.get_logger().warning(f"ESP32 connection reset: {reason}")

    def _write(self, wire: bytes) -> bool:
        with self._serial_lock:
            port = self._serial
            if port is None:
                return False
            try:
                written = port.write(wire)
            except (OSError, serial.SerialException, serial.SerialTimeoutException) as exc:
                error = str(exc)
            else:
                if written == len(wire):
                    self._tx_bytes += written
                    return True
                error = f"short serial write {written}/{len(wire)}"
        self._disconnect(error)
        return False

    def _send(self, message_type: MessageType, payload: bytes = b"") -> Optional[int]:
        with self._tx_lock:
            if self._shutting_down and message_type not in (
                MessageType.STOP,
                MessageType.DISARM,
                MessageType.ESTOP,
            ):
                return None
            sequence = self._tx_sequence
            wire = encode_frame(
                message_type,
                sequence,
                self._now_us(),
                payload,
                self._protocol_version,
            )
            if not self._write(wire):
                return None
            self._tx_sequence = (sequence + 1) & 0xFFFFFFFF
            return sequence

    def _send_and_wait_ack(self, message_type: MessageType, payload: bytes = b""):
        """Send a state-changing command and correlate its bounded ACK.

        The I/O timer uses a separate callback group, so it can
        receive COMMAND_ACK while a service callback waits. Periodic CMD_VEL
        ACKs are never registered and therefore cannot grow this dictionary.
        """
        event = threading.Event()
        with self._tx_lock:
            if self._shutting_down:
                return None, None
            sequence = self._tx_sequence
            wire = encode_frame(
                message_type,
                sequence,
                self._now_us(),
                payload,
                self._protocol_version,
            )
            with self._pending_acks_lock:
                self._pending_acks[sequence] = (
                    event,
                    None,
                    int(message_type),
                )
            if not self._write(wire):
                with self._pending_acks_lock:
                    self._pending_acks.pop(sequence, None)
                return None, None
            self._tx_sequence = (sequence + 1) & 0xFFFFFFFF

        event.wait(self._command_ack_timeout_s)
        with self._pending_acks_lock:
            pending = self._pending_acks.pop(sequence, None)
        acknowledgement = pending[1] if pending is not None else None
        return sequence, acknowledgement

    def _send_hello(self) -> None:
        payload = pack_hello(
            self._protocol_version,
            self._requested_capabilities,
            self._client_name,
            self._client_build_id,
        )
        self._last_hello_s = self._now_s()
        self._send(MessageType.HELLO, payload)

    def _io_tick(self) -> None:
        now_s = self._now_s()
        if self._serial is None:
            self._open_serial(now_s)
            return
        try:
            with self._serial_lock:
                port = self._serial
                if port is None:
                    return
                waiting = min(max(port.in_waiting, 1), 4096)
                data = port.read(waiting)
        except (OSError, serial.SerialException) as exc:
            self._disconnect(str(exc))
            return
        if data:
            self._rx_bytes += len(data)
            for frame in self._decoder.feed(data):
                self._handle_frame(frame)
        if self._serial is None:
            return
        if not self._handshake_complete:
            if now_s - self._connection_opened_s > self._handshake_timeout_s:
                self._handshake_failures += 1
                self._disconnect("HELLO timeout")
            elif now_s - self._last_hello_s >= 1.0:
                self._send_hello()
        if self._decoder.stats.crc_errors - self._crc_baseline >= self._max_crc_errors:
            self._disconnect("CRC error threshold reached")
            return
        if (
            self._handshake_complete
            and now_s - self._last_valid_frame_s > self._serial_liveness_timeout_s
        ):
            # Covers an ESP32 reset behind a USB-UART bridge that remains
            # enumerated: its outbound sequence restarts, so close/reopen is
            # the explicit session boundary before HELLO.
            self._disconnect("valid-frame liveness timeout")

    def _handle_frame(self, frame: Frame) -> None:
        if not sequence_is_newer(frame.sequence, self._last_rx_sequence):
            if frame.sequence == self._last_rx_sequence:
                self._duplicate_sequences += 1
            else:
                self._out_of_order_sequences += 1
            return
        self._last_rx_sequence = frame.sequence
        self._last_valid_frame_s = self._now_s()
        try:
            if frame.message_type == MessageType.HELLO_ACK:
                self._handle_hello_ack(frame.payload)
            elif frame.message_type == MessageType.TIME_SYNC_RESPONSE:
                self._handle_time_sync(frame.payload)
            elif frame.message_type == MessageType.STATUS:
                self._publish_status(frame.payload)
            elif frame.message_type == MessageType.IMU:
                self._publish_imu(frame.payload)
            elif frame.message_type == MessageType.GNSS:
                self._publish_gnss(frame.payload)
            elif frame.message_type == MessageType.MOTOR_FEEDBACK:
                self._publish_motor(frame.payload)
            elif frame.message_type == MessageType.POWER_STATUS:
                self._publish_power(frame.payload)
            elif frame.message_type == MessageType.RELAY_STATUS:
                self._publish_relays(frame.payload)
            elif frame.message_type == MessageType.FAULT_EVENT:
                self._publish_fault(frame.payload)
            elif frame.message_type == MessageType.ESTOP_EVENT:
                event = unpack_estop_event(frame.payload)
                if event.latched:
                    self._clear_arm_authority()
                self._estop_pub.publish(Bool(data=event.latched))
            elif frame.message_type == MessageType.COMMAND_ACK:
                self._handle_command_ack(frame.payload)
            elif frame.message_type == MessageType.DIAGNOSTICS:
                self._esp_diagnostics = unpack_diagnostics(frame.payload)
        except PayloadError as exc:
            self._payload_errors += 1
            self.get_logger().warning(f"rejected {frame.message_type.name} payload: {exc}", throttle_duration_sec=5.0)

    def _handle_hello_ack(self, payload: bytes) -> None:
        ack = unpack_hello_ack(payload)
        if (
            ack.version != self._protocol_version
            or not ack.accepted
            or ack.max_payload < 72
            or (ack.capabilities & self._requested_capabilities)
            != self._requested_capabilities
        ):
            self._handshake_failures += 1
            self._disconnect("HELLO rejected or required capabilities are unavailable")
            return
        first_handshake = not self._handshake_complete
        self._handshake_complete = True
        self._clear_arm_authority()
        if first_handshake:
            # Reconnect never inherits a motion-enabled state.
            self._send(MessageType.DISARM)
            self._send(MessageType.REQUEST_STATUS)
            self.get_logger().info(f"HELLO accepted by firmware {ack.firmware_build}; forced DISARM")

    def _handle_time_sync(self, payload: bytes) -> None:
        response = unpack_time_sync_response(payload)
        pi_receive_us = self._now_us()
        if response.echoed_pi_send_us > pi_receive_us or response.esp_receive_us > response.esp_transmit_us:
            raise PayloadError("invalid time-sync chronology")
        self._esp_to_pi_offset_us = (
            (response.esp_receive_us - response.echoed_pi_send_us)
            + (response.esp_transmit_us - pi_receive_us)
        ) / 2.0

    def _sensor_stamp(self, sensor_us: int):
        if self._esp_to_pi_offset_us is None:
            return self.get_clock().now().to_msg()
        local_sensor_us = sensor_us - self._esp_to_pi_offset_us
        age_us = self._now_us() - local_sensor_us
        # Never make stale telemetry look fresh. A far-future mapping means
        # time sync is invalid, so publish an explicitly invalid zero stamp;
        # arbitrarily old but valid samples retain their age.
        if age_us < -100_000:
            return Time(nanoseconds=0).to_msg()
        stamp_ns = self.get_clock().now().nanoseconds - int(age_us * 1000.0)
        return Time(nanoseconds=max(stamp_ns, 0)).to_msg()

    def _publish_status(self, payload: bytes) -> None:
        status = unpack_status(payload)
        firmware_session_lost = (
            status.state in (RobotStatus.STATE_BOOT, RobotStatus.STATE_DISCONNECTED)
            or status.heartbeat_age_ms > self._esp_heartbeat_timeout_ms
        )
        if firmware_session_lost and self._handshake_complete:
            # The tty can stay enumerated while the ESP32 safety handshake is
            # lost (heartbeat timeout or soft reset). STATUS is deliberately
            # allowed before HELLO, so use it as an explicit session recovery
            # signal instead of remaining falsely connected forever.
            self._handshake_complete = False
            self._clear_arm_authority()
            now_s = self._now_s()
            self._connection_opened_s = now_s
            self._last_hello_s = 0.0
            self._send_hello()
        message = RobotStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.state = status.state
        message.connected = self._handshake_complete
        message.armed = status.armed
        message.estop = status.estop
        message.fault_code = status.fault_code
        message.fault_reason = FAULT_NAMES.get(status.fault_code, f"FAULT_{status.fault_code}")
        message.last_cmd_vel_age_ms = status.cmd_age_ms
        message.last_heartbeat_age_ms = status.heartbeat_age_ms
        message.last_gnss_age_ms = status.gnss_age_ms
        message.last_imu_age_ms = status.imu_age_ms
        message.last_motor_feedback_age_ms = status.motor_age_ms
        message.applied_left_command = status.applied_left
        message.applied_right_command = status.applied_right
        message.uart_speed = status.uart_speed
        message.uart_steer = status.uart_steer
        message.watchdog_trips = status.watchdog_trips
        message.boot_counter = status.boot_counter
        message.reset_reason = status.reset_reason
        with self._motion_lock:
            self._last_status = message
            self._last_status_received_s = self._now_s()
            self._esp_status_armed = bool(
                status.armed and not status.estop and status.fault_code == 0
            )
            if not self._esp_status_armed:
                # A normal DISARMED sample received while an ARM request is in
                # flight may predate its ACK. Fault/ESTOP still cancel at once.
                if (
                    not self._arm_request_in_progress
                    or status.estop
                    or status.fault_code != 0
                    or status.state == RobotStatus.STATE_FAULT
                ):
                    self._clear_arm_authority()
            else:
                self._refresh_motion_gate()
        self._status_pub.publish(message)

    def _publish_imu(self, payload: bytes) -> None:
        data = unpack_imu(payload)
        self._last_imu_calibration_status = data.calibration_status
        self._last_imu_accuracy_rad = data.accuracy_rad
        self._last_imu_sequence = data.sequence
        message = Imu()
        message.header.stamp = self._sensor_stamp(data.sensor_monotonic_us)
        message.header.frame_id = self._frame_id_imu
        message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w = data.quaternion_xyzw
        message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = data.angular_velocity_xyz
        message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = data.linear_acceleration_xyz
        orientation_variance = self._imu_orientation_variance
        if math.isfinite(data.accuracy_rad) and data.accuracy_rad > 0.0:
            # BNO085 rotation-vector accuracy is an angular uncertainty in
            # radians. Keep the configured commissioning floor until mounting
            # and absolute ENU yaw have been physically validated.
            orientation_variance = max(
                orientation_variance, data.accuracy_rad * data.accuracy_rad
            )
        message.orientation_covariance = [orientation_variance, 0.0, 0.0, 0.0, orientation_variance, 0.0, 0.0, 0.0, orientation_variance]
        message.angular_velocity_covariance = [self._imu_angular_velocity_variance, 0.0, 0.0, 0.0, self._imu_angular_velocity_variance, 0.0, 0.0, 0.0, self._imu_angular_velocity_variance]
        message.linear_acceleration_covariance = [self._imu_linear_acceleration_variance, 0.0, 0.0, 0.0, self._imu_linear_acceleration_variance, 0.0, 0.0, 0.0, self._imu_linear_acceleration_variance]
        self._imu_pub.publish(message)

    def _publish_gnss(self, payload: bytes) -> None:
        data = unpack_gnss(payload)
        fix = NavSatFix()
        fix.header.stamp = self._sensor_stamp(data.sensor_monotonic_us)
        fix.header.frame_id = self._frame_id_gps
        fix.latitude = data.latitude_deg
        fix.longitude = data.longitude_deg
        fix.altitude = data.altitude_m
        # UBX fix types 2/3/4 are usable position fixes. Carrier solution is
        # exposed only through RtkStatus; it is not NavSatStatus.GBAS_FIX.
        # UBX type 5 is time-only and must remain NO_FIX.
        fix.status.status = (
            NavSatStatus.STATUS_FIX
            if data.fix_type in (2, 3, 4)
            else NavSatStatus.STATUS_NO_FIX
        )
        fix.status.service = NavSatStatus.SERVICE_GPS
        horizontal_variance = max(data.horizontal_accuracy_m, 0.001) ** 2
        vertical_accuracy = data.vertical_accuracy_m if data.vertical_accuracy_m > 0.0 else self._vertical_accuracy_fallback_m
        fix.position_covariance = [horizontal_variance, 0.0, 0.0, 0.0, horizontal_variance, 0.0, 0.0, 0.0, vertical_accuracy ** 2]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._gps_pub.publish(fix)
        rtk = RtkStatus()
        rtk.header = fix.header
        rtk.sensor_monotonic_us = data.sensor_monotonic_us
        rtk.fix_type = data.fix_type
        rtk.carrier_solution = data.carrier_solution
        rtk.satellites = data.satellites
        rtk.horizontal_accuracy_m = data.horizontal_accuracy_m
        rtk.vertical_accuracy_m = data.vertical_accuracy_m
        rtk.ground_speed_mps = data.ground_speed_mps
        rtk.motion_heading_rad = data.motion_heading_rad
        rtk.rtcm_age_ms = data.rtcm_age_ms
        rtk.sensor_sequence = data.sequence
        self._rtk_pub.publish(rtk)

    def _publish_motor(self, payload: bytes) -> None:
        data = unpack_motor_feedback(payload)
        message = MotorStatus()
        message.header.stamp = self._sensor_stamp(data.sensor_monotonic_us)
        message.header.frame_id = "base_link"
        message.sensor_monotonic_us = data.sensor_monotonic_us
        message.left_feedback = data.left
        message.right_feedback = data.right
        message.battery_voltage = data.battery_voltage
        message.board_temperature_c = data.board_temperature_c
        message.board_temperature_available = data.board_temperature_c > -100.0
        message.controller_fault = data.fault
        message.uart_valid_frames = data.valid_frames
        message.uart_invalid_frames = data.invalid_frames
        self._motor_pub.publish(message)

    def _publish_power(self, payload: bytes) -> None:
        data = unpack_power_status(payload)
        message = PowerStatus()
        message.header.stamp = self._sensor_stamp(data.sensor_monotonic_us)
        message.battery_voltage = data.voltage
        message.battery_current_a = math.nan
        message.state_of_charge_percent = math.nan if data.battery_percent == 255 else float(data.battery_percent)
        message.current_available = False
        message.state_of_charge_available = data.battery_percent != 255
        # Bit 0 means that the motor-controller voltage field is valid. Bit 1
        # is reserved for a future measured undervoltage decision; validity is
        # never misreported as undervoltage.
        message.undervoltage = bool(data.flags & POWER_FLAG_UNDERVOLTAGE)
        message.voltage_available = bool(data.flags & POWER_FLAG_VOLTAGE_VALID)
        self._last_power_flags = data.flags
        self._power_pub.publish(message)

    def _publish_relays(self, payload: bytes) -> None:
        data = unpack_relay_status(payload)
        message = RelayStatus()
        message.header.stamp = self._sensor_stamp(data.sensor_monotonic_us)
        message.available_mask = data.allowed_mask
        message.active_mask = (1 if data.attachment_active else 0) | (2 if data.mount_active else 0)
        self._last_relay_active_mask = message.active_mask
        self._relay_pub.publish(message)

    def _publish_fault(self, payload: bytes) -> None:
        data = unpack_fault_event(payload)
        self._clear_arm_authority()
        message = FaultEvent()
        message.header.stamp = self._sensor_stamp(data.event_monotonic_us)
        message.event_monotonic_us = data.event_monotonic_us
        message.fault_code = data.fault_code
        message.fault_name = FAULT_NAMES.get(data.fault_code, f"FAULT_{data.fault_code}")
        message.detail = f"state {data.from_state} -> {data.to_state}"
        message.occurrence_count = data.occurrence_count
        message.latched = True
        self._fault_pub.publish(message)
        self.get_logger().error(f"ESP32 fault: {message.fault_name}")

    def _handle_command_ack(self, payload: bytes) -> None:
        ack = unpack_command_ack(payload)
        with self._pending_acks_lock:
            pending = self._pending_acks.get(ack.request_sequence)
            if pending is not None:
                event, _, expected_type = pending
                if ack.request_type == expected_type:
                    self._pending_acks[ack.request_sequence] = (
                        event,
                        ack,
                        expected_type,
                    )
                    event.set()
                else:
                    self.get_logger().error(
                        "COMMAND_ACK type mismatch for sequence "
                        f"{ack.request_sequence}: expected {expected_type:#04x}, "
                        f"received {ack.request_type:#04x}"
                    )
        if ack.result != 0:
            with self._motion_lock:
                if ack.request_type == int(MessageType.ARM):
                    self._clear_arm_authority()
                elif ack.request_type == int(MessageType.CMD_VEL):
                    self._send(MessageType.STOP)
                    self._clear_arm_authority()
            self.get_logger().warning(
                f"ESP32 rejected {ack.request_type:#04x} seq={ack.request_sequence}: result={ack.result} detail={ack.detail}"
            )

    def _reject_velocity(self, reason: str) -> None:
        with self._motion_lock:
            self._gate.invalidate()
            if self._serial_available() and (
                self._arm_ack_confirmed or self._esp_status_armed or self._last_status.armed
            ):
                self._send(MessageType.STOP)
            self._stop_sent_for_stale = True
        self.get_logger().warning(reason, throttle_duration_sec=2.0)

    def _on_cmd_vel(self, message: TwistStamped) -> None:
        with self._motion_lock:
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
                self._reject_velocity("rejected non-finite /cmd_vel_safe; STOP sent")
                return
            linear, lateral, vertical, roll, pitch, angular = components
            if any(
                abs(value) > 1.0e-9
                for value in (lateral, vertical, roll, pitch)
            ):
                self._reject_velocity(
                    "rejected non-planar /cmd_vel_safe; STOP sent"
                )
                return
            stamp_ns = (
                int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec)
            )
            age_ns = self.get_clock().now().nanoseconds - stamp_ns
            if stamp_ns <= 0 or age_ns < -100_000_000 or age_ns > self._ros_cmd_stale_ms * 1_000_000:
                self._reject_velocity("rejected stale or future /cmd_vel_safe; STOP sent")
                return
            linear = min(max(linear, -self._max_reverse_mps), self._max_linear_mps)
            angular = min(max(angular, -self._max_angular_rad_s), self._max_angular_rad_s)
            self._gate.update(self._now_s(), linear, angular)
            self._stop_sent_for_stale = False

    def _on_safety_state(self, message: SafetyState) -> None:
        with self._motion_lock:
            new_state = int(message.state)
            if not SafetyState.STATE_DISCONNECTED <= new_state <= SafetyState.STATE_ESTOP:
                new_state = SafetyState.STATE_FAULT
            previous_state = self._safety_state
            was_motion_authorized = self._gate.armed_confirmed
            self._safety_state = new_state

            if self._arm_request_in_progress and new_state in (
                SafetyState.STATE_DISCONNECTED,
                SafetyState.STATE_FAULT,
                SafetyState.STATE_ESTOP,
            ):
                if self._serial_available():
                    self._send(MessageType.STOP)
                    self._send(
                        MessageType.ESTOP
                        if new_state == SafetyState.STATE_ESTOP
                        else MessageType.DISARM
                    )
                self._clear_arm_authority()
                return

            if new_state == SafetyState.STATE_ESTOP and previous_state != new_state:
                if self._serial_available():
                    self._send(MessageType.ESTOP)
                self._clear_arm_authority()
                return

            expected_state = (
                SafetyState.STATE_MANUAL
                if self._armed_control_mode == 1
                else SafetyState.STATE_AUTO
                if self._armed_control_mode == 2
                else None
            )
            incompatible_armed_state = (
                self._arm_ack_confirmed
                and new_state != SafetyState.STATE_DISARMED
                and new_state != expected_state
            )
            if (was_motion_authorized and new_state != expected_state) or incompatible_armed_state:
                if self._serial_available():
                    self._send(MessageType.STOP)
                    self._send(MessageType.DISARM)
                self._clear_arm_authority()
                return
            self._refresh_motion_gate()

    def _command_tick(self) -> None:
        with self._motion_lock:
            if self._shutting_down or not self._handshake_complete or not self._gate.armed_confirmed:
                return
            now_s = self._now_s()
            if not self._gate.is_fresh(now_s):
                if not self._stop_sent_for_stale:
                    self._send(MessageType.STOP)
                    self._stop_sent_for_stale = True
                return
            linear, angular = self._gate.output(now_s)
            self._send(
                MessageType.CMD_VEL,
                pack_cmd_vel(linear, angular, self._command_timeout_ms, self._armed_control_mode),
            )

    def _heartbeat_tick(self) -> None:
        if not self._handshake_complete:
            return
        now_s = self._now_s()
        self._send(MessageType.HEARTBEAT, pack_heartbeat(self._now_us(), self._safety_state))
        if now_s - self._last_time_sync_s >= self._time_sync_period_s:
            sent_us = self._now_us()
            self._send(MessageType.TIME_SYNC_REQUEST, pack_time_sync_request(sent_us))
            self._last_time_sync_s = now_s
        if now_s - self._last_status_request_s >= 1.0:
            self._send(MessageType.REQUEST_STATUS)
            self._last_status_request_s = now_s

    def _control_available(self) -> bool:
        return self._handshake_complete and self._serial is not None

    def _serial_available(self) -> bool:
        return self._serial is not None

    def _on_arm(self, request: Arm.Request, response: Arm.Response) -> Arm.Response:
        if not self._control_available():
            response.success = False
            response.resulting_state = RobotStatus.STATE_DISCONNECTED
            response.message = "ESP32 is not connected"
            return response
        if request.arm_nonce == 0 or request.requested_mode not in (
            Arm.Request.MODE_MANUAL,
            Arm.Request.MODE_AUTO,
        ):
            response.success = False
            response.resulting_state = self._last_status.state
            response.message = "ARM requires a non-zero nonce and requested_mode 1 (MANUAL) or 2 (AUTO)"
            return response
        with self._motion_lock:
            self._clear_arm_authority()
            self._arming_requested = True
            self._arm_request_in_progress = True
            arm_epoch = self._motion_epoch
        sequence, ack = self._send_and_wait_ack(
            MessageType.ARM, pack_arm(request.arm_nonce, request.requested_mode)
        )
        with self._motion_lock:
            cancelled = arm_epoch != self._motion_epoch
            response.success = bool(
                not cancelled
                and ack is not None
                and ack.result == 0
                and ack.state == RobotStatus.STATE_ARMED
            )
            response.resulting_state = ack.state if ack is not None else self._last_status.state
            if response.success:
                # Commands received while ARM was in flight are not eligible.
                # A separate post-ACK /cmd_vel_safe sample is required.
                self._gate.invalidate()
                self._arming_requested = True
                self._arm_request_in_progress = False
                self._armed_control_mode = request.requested_mode
                self._arm_ack_confirmed = True
                self._esp_status_armed = True
                zero_sequence = self._send(
                    MessageType.CMD_VEL,
                    pack_cmd_vel(0.0, 0.0, self._command_timeout_ms, self._armed_control_mode),
                )
                if zero_sequence is None:
                    self._send(MessageType.DISARM)
                    self._clear_arm_authority()
                    response.success = False
                    response.message = "ARM was acknowledged but zero keepalive failed; fail-safe DISARM sent"
                    return response
                self._refresh_motion_gate()
                response.message = (
                    "ARM acknowledged and zero CMD_VEL sent; non-zero motion remains blocked "
                    "until matching SafetyState and a fresh /cmd_vel_safe"
                )
            else:
                self._clear_arm_authority()
                if cancelled:
                    self._send(MessageType.DISARM)
                    response.message = "ARM cancelled by a newer STOP/DISARM/ESTOP; fail-safe DISARM sent"
                elif sequence is not None and ack is None:
                    # ARM might have reached ESP32 despite a lost ACK. A new
                    # zero-only sequence removes that ambiguous authority.
                    self._send(MessageType.DISARM)
                    response.message = "ARM ACK timeout; fail-safe DISARM sent"
                elif ack is not None:
                    response.message = (
                        f"ARM rejected or returned non-ARMED state: "
                        f"result={ack.result} state={ack.state} detail={ack.detail}"
                    )
                else:
                    response.message = "ARM write failed"
        return response

    def _on_disarm(self, request: Disarm.Request, response: Disarm.Response) -> Disarm.Response:
        del request
        with self._motion_lock:
            self._clear_arm_authority()
            sequence, ack = self._send_and_wait_ack(MessageType.DISARM) if self._serial_available() else (None, None)
        response.success = ack is not None and ack.result == 0
        response.resulting_state = ack.state if ack is not None else self._last_status.state
        response.message = (
            "DISARM acknowledged"
            if response.success
            else ("DISARM sent but ACK was not received" if sequence is not None else "ESP32 is not connected")
        )
        return response

    def _on_stop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        with self._motion_lock:
            if self._arm_request_in_progress:
                self._motion_epoch += 1
                self._arm_request_in_progress = False
                self._arming_requested = False
            self._gate.invalidate()
            sequence, ack = self._send_and_wait_ack(MessageType.STOP) if self._serial_available() else (None, None)
        response.success = ack is not None and ack.result == 0
        response.message = (
            "STOP acknowledged"
            if response.success
            else ("STOP sent but ACK was not received; watchdog remains active" if sequence is not None else "ESP32 is not connected")
        )
        return response

    def _on_estop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        with self._motion_lock:
            self._clear_arm_authority()
            sequence, ack = self._send_and_wait_ack(MessageType.ESTOP) if self._serial_available() else (None, None)
        response.success = ack is not None and ack.result == 0
        response.message = (
            "ESTOP acknowledged"
            if response.success
            else ("ESTOP sent but ACK was not received" if sequence is not None else "ESP32 is not connected")
        )
        return response

    def _on_reset_fault(self, request: ResetFault.Request, response: ResetFault.Response) -> ResetFault.Response:
        del request
        with self._motion_lock:
            self._clear_arm_authority()
            sequence, ack = self._send_and_wait_ack(MessageType.RESET_FAULT) if self._control_available() else (None, None)
        response.success = ack is not None and ack.result == 0
        response.resulting_state = ack.state if ack is not None else self._last_status.state
        response.message = (
            "RESET_FAULT acknowledged; explicit ARM remains required"
            if response.success
            else (f"RESET_FAULT rejected: result={ack.result} detail={ack.detail}" if ack is not None else ("RESET_FAULT sent but ACK was not received" if sequence is not None else "ESP32 is not connected"))
        )
        return response

    def _on_reset_estop(self, request: ResetEstop.Request, response: ResetEstop.Response) -> ResetEstop.Response:
        del request
        with self._motion_lock:
            self._clear_arm_authority()
            sequence, ack = self._send_and_wait_ack(MessageType.RESET_ESTOP) if self._control_available() else (None, None)
        response.success = ack is not None and ack.result == 0
        response.resulting_state = ack.state if ack is not None else self._last_status.state
        response.message = (
            "RESET_ESTOP acknowledged; explicit ARM remains required"
            if response.success
            else (f"RESET_ESTOP rejected: result={ack.result} detail={ack.detail}" if ack is not None else ("RESET_ESTOP sent but ACK was not received" if sequence is not None else "ESP32 is not connected"))
        )
        return response

    def _on_set_relays(self, request: SetRelays.Request, response: SetRelays.Response) -> SetRelays.Response:
        if request.relay_mask & ~0x03 or request.relay_values & ~0x03:
            response.success = False
            response.active_mask = 0
            response.message = "only relay bits 0 and 1 are defined"
            return response
        sequence, ack = self._send_and_wait_ack(
            MessageType.RELAY_COMMAND,
            pack_relay_command(request.relay_mask, request.relay_values),
        ) if self._control_available() else (None, None)
        response.success = ack is not None and ack.result == 0
        response.active_mask = self._last_relay_active_mask
        if response.success:
            response.active_mask = (
                (self._last_relay_active_mask & ~request.relay_mask)
                | (request.relay_values & request.relay_mask)
            )
            response.message = "RELAY_COMMAND acknowledged; verify /relay/status"
        elif sequence is not None and ack is None:
            self._send(MessageType.RELAY_COMMAND, pack_relay_command(0x03, 0x00))
            response.active_mask = 0
            response.message = "RELAY_COMMAND ACK timeout; relay-off fallback sent"
        elif ack is not None:
            response.message = f"RELAY_COMMAND rejected: result={ack.result} detail={ack.detail}"
        else:
            response.message = "ESP32 is not connected"
        return response

    def _publish_disconnected_status(self) -> None:
        message = RobotStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.state = RobotStatus.STATE_DISCONNECTED
        message.connected = False
        message.armed = False
        message.estop = self._last_status.estop
        message.fault_code = self._last_status.fault_code
        message.fault_reason = self._last_status.fault_reason
        self._last_status = message
        self._status_pub.publish(message)

    def _publish_health(self) -> None:
        now = self.get_clock().now().to_msg()
        stats = ProtocolStats()
        stats.header.stamp = now
        stats.rx_bytes = self._rx_bytes
        stats.tx_bytes = self._tx_bytes
        stats.valid_frames = self._decoder.stats.valid_frames
        stats.cobs_errors = self._decoder.stats.cobs_errors
        stats.crc_errors = self._decoder.stats.crc_errors
        stats.length_errors = self._decoder.stats.length_errors + self._payload_errors
        stats.version_errors = self._decoder.stats.version_errors
        stats.unknown_message_types = self._decoder.stats.unknown_message_types
        stats.duplicate_sequences = self._duplicate_sequences
        stats.out_of_order_sequences = self._out_of_order_sequences
        stats.oversized_frames = self._decoder.stats.oversized_frames
        stats.reconnects = self._reconnects
        stats.handshake_failures = self._handshake_failures
        if self._esp_diagnostics is not None:
            esp = self._esp_diagnostics
            stats.esp_rx_frames = esp.rx_frames
            stats.esp_tx_frames = esp.tx_frames
            stats.esp_crc_errors = esp.crc_errors
            stats.esp_cobs_errors = esp.cobs_errors
            stats.esp_length_errors = esp.length_errors
            stats.esp_oversized_frames = esp.oversized_frames
            stats.esp_unknown_message_types = esp.unknown_types
            stats.esp_duplicate_sequences = esp.duplicate_sequences
            stats.esp_out_of_order_sequences = esp.out_of_order_sequences
            stats.esp_rx_overflows = esp.rx_overflows
            stats.esp_tx_errors = esp.tx_errors
        self._stats_pub.publish(stats)

        diagnostic = DiagnosticStatus()
        diagnostic.name = "module_robot/esp32_bridge/link"
        diagnostic.hardware_id = self._device
        if not self._handshake_complete:
            diagnostic.level = DiagnosticStatus.ERROR
            diagnostic.message = "DISCONNECTED"
        elif (
            stats.crc_errors > 0
            or stats.length_errors > 0
            or stats.esp_crc_errors > 0
            or stats.esp_length_errors > 0
            or stats.esp_rx_overflows > 0
            or stats.esp_tx_errors > 0
        ):
            diagnostic.level = DiagnosticStatus.WARN
            diagnostic.message = "connected with protocol errors"
        else:
            diagnostic.level = DiagnosticStatus.OK
            diagnostic.message = "connected"
        diagnostic.values = [
            KeyValue(key="armed_confirmed", value=str(self._gate.armed_confirmed)),
            KeyValue(key="crc_errors", value=str(stats.crc_errors)),
            KeyValue(key="length_errors", value=str(stats.length_errors)),
            KeyValue(key="reconnects", value=str(stats.reconnects)),
            KeyValue(key="esp_crc_errors", value=str(stats.esp_crc_errors)),
            KeyValue(key="esp_length_errors", value=str(stats.esp_length_errors)),
            KeyValue(key="esp_rx_overflows", value=str(stats.esp_rx_overflows)),
            KeyValue(key="esp_tx_errors", value=str(stats.esp_tx_errors)),
        ]

        status_age_s = self._now_s() - self._last_status_received_s
        status_fresh = math.isfinite(status_age_s) and status_age_s <= 1.5

        safety = DiagnosticStatus()
        safety.name = "module_robot/esp32_bridge/safety"
        safety.hardware_id = self._device
        if not self._handshake_complete:
            safety.level = DiagnosticStatus.ERROR
            safety.message = "DISCONNECTED"
        elif not status_fresh:
            safety.level = DiagnosticStatus.ERROR
            safety.message = "STATUS telemetry stale"
        elif self._last_status.estop:
            safety.level = DiagnosticStatus.ERROR
            safety.message = "ESTOP latched"
        elif self._last_status.fault_code != 0:
            safety.level = DiagnosticStatus.ERROR
            safety.message = self._last_status.fault_reason
        else:
            safety.level = DiagnosticStatus.OK
            safety.message = "ARMED" if self._last_status.armed else "DISARMED"
        safety.values = [
            KeyValue(key="esp_state", value=str(self._last_status.state)),
            KeyValue(key="armed", value=str(self._last_status.armed)),
            KeyValue(key="estop", value=str(self._last_status.estop)),
            KeyValue(key="fault_code", value=str(self._last_status.fault_code)),
            KeyValue(key="status_age_s", value=f"{status_age_s:.3f}"),
            KeyValue(key="watchdog_trips", value=str(self._last_status.watchdog_trips)),
        ]

        motor = DiagnosticStatus()
        motor.name = "module_robot/esp32_bridge/motor_controller"
        motor.hardware_id = self._device
        motor_age = self._last_status.last_motor_feedback_age_ms
        if not self._handshake_complete or not status_fresh:
            motor.level = DiagnosticStatus.ERROR
            motor.message = "motor status unavailable"
        elif self._last_power_flags & POWER_FLAG_MOTOR_CONTROLLER_FAULT:
            motor.level = DiagnosticStatus.ERROR
            motor.message = "motor-controller fault"
        elif self._last_power_flags & POWER_FLAG_MOTOR_FEEDBACK_STALE or motor_age > 500:
            motor.level = DiagnosticStatus.ERROR
            motor.message = "motor feedback stale"
        elif self._last_power_flags & POWER_FLAG_INVALID_HOVERBOARD_FRAME_SEEN:
            motor.level = DiagnosticStatus.WARN
            motor.message = "invalid hoverboard feedback frame observed"
        else:
            motor.level = DiagnosticStatus.OK
            motor.message = "motor feedback fresh"
        motor.values = [
            KeyValue(key="feedback_age_ms", value=str(motor_age)),
            KeyValue(key="applied_left", value=str(self._last_status.applied_left_command)),
            KeyValue(key="applied_right", value=str(self._last_status.applied_right_command)),
            KeyValue(
                key="controller_fault_flag",
                value=str(
                    bool(self._last_power_flags & POWER_FLAG_MOTOR_CONTROLLER_FAULT)
                ),
            ),
            KeyValue(
                key="invalid_hoverboard_frame_seen",
                value=str(
                    bool(
                        self._last_power_flags
                        & POWER_FLAG_INVALID_HOVERBOARD_FRAME_SEEN
                    )
                ),
            ),
        ]

        imu = DiagnosticStatus()
        imu.name = "module_robot/esp32_bridge/imu"
        imu.hardware_id = self._device
        imu_age = self._last_status.last_imu_age_ms
        if not self._handshake_complete or not status_fresh:
            imu.level = DiagnosticStatus.ERROR
            imu.message = "IMU status unavailable"
        elif self._last_power_flags & POWER_FLAG_IMU_UNAVAILABLE:
            imu.level = DiagnosticStatus.ERROR
            imu.message = "IMU unavailable"
        elif imu_age > 200:
            imu.level = DiagnosticStatus.WARN
            imu.message = "IMU telemetry stale"
        else:
            imu.level = DiagnosticStatus.OK
            imu.message = "IMU telemetry fresh"
        imu.values = [
            KeyValue(key="age_ms", value=str(imu_age)),
            KeyValue(
                key="calibration_status",
                value=(
                    "unavailable"
                    if self._last_imu_calibration_status is None
                    else str(self._last_imu_calibration_status)
                ),
            ),
            KeyValue(
                key="rotation_vector_accuracy_rad",
                value=str(self._last_imu_accuracy_rad),
            ),
            KeyValue(key="sensor_sequence", value=str(self._last_imu_sequence)),
            KeyValue(key="frame_convention", value="REP-103 target; mounting/yaw unverified"),
        ]

        gnss = DiagnosticStatus()
        gnss.name = "module_robot/esp32_bridge/gnss"
        gnss.hardware_id = self._device
        gnss_age = self._last_status.last_gnss_age_ms
        if not self._handshake_complete or not status_fresh:
            gnss.level = DiagnosticStatus.ERROR
            gnss.message = "GNSS status unavailable"
        elif self._last_power_flags & POWER_FLAG_GNSS_UNAVAILABLE:
            gnss.level = DiagnosticStatus.ERROR
            gnss.message = "GNSS unavailable"
        elif self._last_power_flags & (
            POWER_FLAG_UBX_CHECKSUM_ERROR_SEEN
            | POWER_FLAG_UBX_OVERSIZED_FRAME_SEEN
        ):
            gnss.level = DiagnosticStatus.WARN
            gnss.message = "GNSS parser errors observed"
        elif gnss_age > 1500:
            gnss.level = DiagnosticStatus.WARN
            gnss.message = "GNSS telemetry stale"
        else:
            gnss.level = DiagnosticStatus.OK
            gnss.message = "GNSS telemetry fresh"
        gnss.values = [
            KeyValue(key="age_ms", value=str(gnss_age)),
            KeyValue(
                key="ubx_checksum_error_seen",
                value=str(
                    bool(self._last_power_flags & POWER_FLAG_UBX_CHECKSUM_ERROR_SEEN)
                ),
            ),
            KeyValue(
                key="ubx_oversized_frame_seen",
                value=str(
                    bool(
                        self._last_power_flags
                        & POWER_FLAG_UBX_OVERSIZED_FRAME_SEEN
                    )
                ),
            ),
        ]

        power = DiagnosticStatus()
        power.name = "module_robot/esp32_bridge/power"
        power.hardware_id = self._device
        if self._last_power_flags & POWER_FLAG_UNDERVOLTAGE:
            power.level = DiagnosticStatus.ERROR
            power.message = "undervoltage"
        elif not self._last_power_flags & POWER_FLAG_VOLTAGE_VALID:
            power.level = DiagnosticStatus.WARN
            power.message = "voltage unavailable"
        else:
            power.level = DiagnosticStatus.OK
            power.message = "voltage telemetry available; SoC uncalibrated"
        power.values = [
            KeyValue(
                key="voltage_available",
                value=str(bool(self._last_power_flags & POWER_FLAG_VOLTAGE_VALID)),
            ),
            KeyValue(key="soc_calibrated", value="False"),
        ]

        array = DiagnosticArray()
        array.header.stamp = now
        array.status = [diagnostic, safety, motor, imu, gnss, power]
        self._diagnostics_pub.publish(array)

    def safe_shutdown(self) -> None:
        if self._shutting_down:
            return
        self._shutting_down = True
        with self._motion_lock:
            self._safety_state = SafetyState.STATE_DISCONNECTED
            self._clear_arm_authority()
            if self._serial is not None:
                # Separate frames and flush where possible. ESP32 watchdog
                # remains the independent final stop if shutdown is abrupt.
                self._send(MessageType.STOP)
                self._send(MessageType.DISARM)
                with self._serial_lock:
                    port = self._serial
                    if port is not None:
                        try:
                            port.flush()
                        except (OSError, serial.SerialException):
                            pass
        self._disconnect("node shutdown")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Esp32Bridge()
    # One service callback may wait for COMMAND_ACK while serial I/O and the
    # command/heartbeat timers must continue independently.
    executor = MultiThreadedExecutor(num_threads=6)
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
