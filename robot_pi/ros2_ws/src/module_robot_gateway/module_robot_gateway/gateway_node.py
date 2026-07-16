"""ROS 2 WebSocket compatibility gateway.

All motion published here goes to ``/cmd_vel_manual``. This node deliberately
has no serial dependency, ESP32 command publisher, ARM client, or Nav2 action
client in stage 1.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from module_robot_msgs.msg import (
    FaultEvent,
    PowerStatus,
    RelayStatus,
    RobotStatus,
    RtkStatus,
    SafetyState,
)
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from .commands import (
    CommandError,
    ManualDrive,
    NavigationCommand,
    Ping,
    RouteBegin,
    RouteEnd,
    RouteWaypoint,
    Stop,
    parse_command,
)
from .routes import RouteError, RouteStore, StoredRoute
from .websocket_server import GatewayWebSocketServer


@dataclass
class _Received:
    message: object
    monotonic_s: float


@dataclass
class _Telemetry:
    gps: Optional[_Received] = None
    imu: Optional[_Received] = None
    rtk: Optional[_Received] = None
    power: Optional[_Received] = None
    relays: Optional[_Received] = None
    robot: Optional[_Received] = None
    safety: Optional[_Received] = None
    fault: Optional[_Received] = None


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

CARRIER_NAMES = {
    RtkStatus.CARRIER_NONE: "NONE",
    RtkStatus.CARRIER_FLOAT: "FLOAT",
    RtkStatus.CARRIER_FIXED: "FIXED",
}


def _csv_text(value: object, limit: int = 96) -> str:
    text = str(value).replace(",", ";").replace("\r", " ").replace("\n", " ")
    return text[:limit]


def _age_ms(received: Optional[_Received], now: float) -> int:
    if received is None:
        return 0xFFFFFFFF
    return min(0xFFFFFFFF, max(0, int((now - received.monotonic_s) * 1000.0)))


def _accuracy_mm(value_m: float) -> int:
    if not math.isfinite(value_m) or value_m < 0.0:
        return 0x7FFFFFFF
    return min(0x7FFFFFFF, int(value_m * 1000.0))


def _quaternion_yaw_deg(message: Imu) -> Tuple[float, bool]:
    q = message.orientation
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if not math.isfinite(norm) or norm < 1e-9:
        return 0.0, False
    x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    covariance_claims_orientation = (
        len(message.orientation_covariance) == 9
        and message.orientation_covariance[0] >= 0.0
        and all(math.isfinite(value) for value in message.orientation_covariance)
    )
    return math.degrees(yaw), covariance_claims_orientation


class GatewayNode(Node):
    def __init__(self) -> None:
        super().__init__("gateway_node")
        self._declare_parameters()
        self._read_and_validate_parameters()

        self._telemetry = _Telemetry()
        self._telemetry_lock = threading.RLock()
        self._motion_lock = threading.RLock()
        self._manual_lock = threading.RLock()
        self._stop_service_lock = threading.Lock()
        self._stop_service_pending = False
        self._manual_owner: Optional[str] = None
        self._last_manual_command_s = 0.0
        self._nav_state = "IDLE"
        self._nav_reason = "STAGE1"
        self._last_stop_service_warning_s = 0.0
        self._destroying = False

        self._route_store = RouteStore(
            min_waypoints=self._route_min_waypoints,
            max_waypoints=self._route_max_waypoints,
            min_segment_m=self._route_min_segment_m,
            max_segment_m=self._route_max_segment_m,
            max_abs_local_m=self._route_max_abs_local_m,
        )

        reliable_state_qos = QoSProfile(
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

        self._manual_publisher = self.create_publisher(
            TwistStamped, "/cmd_vel_manual", 10
        )
        self._route_path_publisher = self.create_publisher(Path, "/route/path", latched_qos)
        self._route_valid_publisher = self.create_publisher(
            Bool, "/route/valid", latched_qos
        )
        self._stop_client = self.create_client(Trigger, "/safety/stop")

        self.create_subscription(
            NavSatFix, "/gps/fix", self._on_gps, qos_profile_sensor_data
        )
        self.create_subscription(Imu, "/imu/data_raw", self._on_imu, qos_profile_sensor_data)
        self.create_subscription(RtkStatus, "/rtk/status", self._on_rtk, reliable_state_qos)
        self.create_subscription(
            PowerStatus, "/power/status", self._on_power, reliable_state_qos
        )
        self.create_subscription(
            RelayStatus, "/relay/status", self._on_relays, reliable_state_qos
        )
        self.create_subscription(
            RobotStatus, "/esp32/status", self._on_robot, reliable_state_qos
        )
        self.create_subscription(
            FaultEvent,
            "/esp32/fault_event",
            self._on_fault,
            reliable_state_qos,
        )
        self.create_subscription(
            SafetyState, "/safety/state", self._on_safety, reliable_state_qos
        )

        self._manual_timer = self.create_timer(0.02, self._check_manual_timeout)
        self._telemetry_timer = self.create_timer(
            1.0 / self._telemetry_rate_hz, self._broadcast_telemetry
        )

        self._websocket = GatewayWebSocketServer(
            bind_address=self._bind_address,
            port=self._websocket_port,
            path=self._websocket_path,
            max_clients=self._max_clients,
            max_message_bytes=self._max_message_bytes,
            queue_depth=self._per_client_queue_depth,
            on_command=self._handle_command,
            on_connect=self._on_client_connect,
            on_disconnect=self._on_client_disconnect,
            log_info=self.get_logger().info,
            log_warning=self.get_logger().warning,
        )

        # Route validity is fail-closed from the first observable state.
        self._publish_route(None)
        self._publish_zero()
        self._websocket.start()
        self.get_logger().info(
            "Gateway stage 1 ready: no ESP32 access, no automatic ARM, "
            "NAV_START disabled"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "config_schema_version": 1,
            "bind_address": "0.0.0.0",
            "websocket_port": 81,
            "websocket_path": "/ws",
            "max_clients": 2,
            "max_message_bytes": 4096,
            "telemetry_rate_hz": 5.0,
            "per_client_queue_depth": 8,
            "manual_input_max_percent": 100.0,
            "manual_deadband_percent": 2.0,
            "manual_max_linear_mps": 0.15,
            "manual_max_angular_rad_s": 0.60,
            "manual_command_timeout_s": 0.25,
            "route_max_waypoints": 2048,
            "route_min_waypoints": 1,
            "route_max_segment_m": 100.0,
            "route_min_segment_m": 0.01,
            "route_max_abs_local_m": 10000.0,
            "route_frame_id": "map",
            "autonomous_start_enabled": False,
            "robot_hostname": "module-robot",
            "wifi_credentials": "NOT_STORED_IN_REPOSITORY",
            "cloud_services_enabled": False,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

    def _read_and_validate_parameters(self) -> None:
        config_schema_version = int(
            self.get_parameter("config_schema_version").value
        )
        self._bind_address = str(self.get_parameter("bind_address").value)
        self._websocket_port = int(self.get_parameter("websocket_port").value)
        self._websocket_path = str(self.get_parameter("websocket_path").value)
        self._max_clients = int(self.get_parameter("max_clients").value)
        self._max_message_bytes = int(self.get_parameter("max_message_bytes").value)
        self._telemetry_rate_hz = float(self.get_parameter("telemetry_rate_hz").value)
        self._per_client_queue_depth = int(
            self.get_parameter("per_client_queue_depth").value
        )
        self._manual_input_max_percent = float(
            self.get_parameter("manual_input_max_percent").value
        )
        self._manual_deadband_percent = float(
            self.get_parameter("manual_deadband_percent").value
        )
        self._manual_max_linear_mps = float(
            self.get_parameter("manual_max_linear_mps").value
        )
        self._manual_max_angular_rad_s = float(
            self.get_parameter("manual_max_angular_rad_s").value
        )
        self._manual_command_timeout_s = float(
            self.get_parameter("manual_command_timeout_s").value
        )
        self._route_max_waypoints = int(
            self.get_parameter("route_max_waypoints").value
        )
        self._route_min_waypoints = int(
            self.get_parameter("route_min_waypoints").value
        )
        self._route_max_segment_m = float(
            self.get_parameter("route_max_segment_m").value
        )
        self._route_min_segment_m = float(
            self.get_parameter("route_min_segment_m").value
        )
        self._route_max_abs_local_m = float(
            self.get_parameter("route_max_abs_local_m").value
        )
        self._route_frame_id = str(self.get_parameter("route_frame_id").value)

        if config_schema_version != 1:
            raise ValueError("unsupported config_schema_version; expected 1")
        if not self._bind_address:
            raise ValueError("bind_address must not be empty")
        if not 1 <= self._websocket_port <= 65535:
            raise ValueError("websocket_port must be in [1, 65535]")
        if not self._websocket_path.startswith("/"):
            raise ValueError("websocket_path must start with '/'")
        if not 1 <= self._max_clients <= 32:
            raise ValueError("max_clients must be in [1, 32]")
        if not 64 <= self._max_message_bytes <= 1_048_576:
            raise ValueError("max_message_bytes must be in [64, 1048576]")
        if not 0.2 <= self._telemetry_rate_hz <= 20.0:
            raise ValueError("telemetry_rate_hz must be in [0.2, 20]")
        if not 1 <= self._per_client_queue_depth <= 100:
            raise ValueError("per_client_queue_depth must be in [1, 100]")
        if self._manual_input_max_percent <= 0.0:
            raise ValueError("manual_input_max_percent must be positive")
        if not 0.0 <= self._manual_deadband_percent < self._manual_input_max_percent:
            raise ValueError("manual_deadband_percent is invalid")
        if not 0.0 < self._manual_max_linear_mps <= 0.15:
            raise ValueError("manual_max_linear_mps must be in (0, 0.15] for stage 1")
        if not 0.0 < self._manual_max_angular_rad_s <= 0.60:
            raise ValueError(
                "manual_max_angular_rad_s must be in (0, 0.60] for stage 1"
            )
        if not 0.05 <= self._manual_command_timeout_s <= 0.30:
            raise ValueError("manual_command_timeout_s must be in [0.05, 0.30]")
        if not self._route_frame_id:
            raise ValueError("route_frame_id must not be empty")
        if bool(self.get_parameter("autonomous_start_enabled").value):
            self.get_logger().warning(
                "autonomous_start_enabled=true ignored: stage 1 always rejects NAV_START"
            )
        if bool(self.get_parameter("cloud_services_enabled").value):
            raise ValueError("cloud_services_enabled must remain false")
        if str(self.get_parameter("wifi_credentials").value) != "NOT_STORED_IN_REPOSITORY":
            raise ValueError("Wi-Fi credentials must not be stored in gateway YAML")

    def _handle_command(self, client_id: str, text: str) -> str:
        if self._destroying:
            return "ERR,COMMAND,SHUTTING_DOWN"
        try:
            command = parse_command(text, max_message_bytes=self._max_message_bytes)
        except CommandError as exc:
            return exc.response()

        if isinstance(command, ManualDrive):
            return self._handle_manual(client_id, command)
        if isinstance(command, Ping):
            return "PONG"
        if isinstance(command, Stop):
            self._release_manual_owner()
            self._nav_state = "IDLE"
            self._nav_reason = "OPERATOR_STOP"
            self._issue_stop("websocket STOP")
            return "OK,STOP"
        if isinstance(command, RouteBegin):
            try:
                origin = None
                if command.legacy_local_mode:
                    origin = (command.origin_latitude, command.origin_longitude)
                self._route_store.begin(client_id, command.expected_count, origin)
            except RouteError as exc:
                return exc.response("ROUTE_BEGIN")
            self._publish_route(None)
            return "OK,ROUTE_BEGIN"
        if isinstance(command, RouteWaypoint):
            try:
                self._route_store.add_waypoint(
                    client_id, command.index, command.first, command.second
                )
            except RouteError as exc:
                return exc.response("ROUTE_WP")
            return f"OK,ROUTE_WP,{command.index}"
        if isinstance(command, RouteEnd):
            try:
                route = self._route_store.finish(client_id)
            except RouteError as exc:
                self._publish_route(None)
                return exc.response("ROUTE_END")
            self._publish_route(route)
            self.get_logger().info(
                f"Validated route version={route.version} mode={route.mode.value} "
                f"points={len(route.points)} length_m={route.total_length_m:.2f}"
            )
            return f"OK,ROUTE,{len(route.points)}"
        if isinstance(command, NavigationCommand):
            return self._handle_navigation(command.action)
        return "ERR,COMMAND,INTERNAL"

    def _handle_manual(self, client_id: str, command: ManualDrive) -> str:
        maximum = self._manual_input_max_percent
        if abs(command.left_percent) > maximum or abs(command.right_percent) > maximum:
            return "ERR,M,RANGE"

        # The shutdown path takes this same lock and publishes zero after any
        # already-running command. A command that enters later observes the
        # destroying flag and cannot overwrite shutdown zero.
        with self._motion_lock:
            if self._destroying:
                return "ERR,M,SHUTTING_DOWN"
            with self._manual_lock:
                if self._manual_owner is not None and self._manual_owner != client_id:
                    return "ERR,M,MANUAL_BUSY"
                if self._manual_owner is None:
                    self._manual_owner = client_id
                self._last_manual_command_s = time.monotonic()

            left = command.left_percent
            right = command.right_percent
            if abs(left) < self._manual_deadband_percent:
                left = 0.0
            if abs(right) < self._manual_deadband_percent:
                right = 0.0
            left_normalized = left / maximum
            right_normalized = right / maximum

            linear = 0.5 * (left_normalized + right_normalized)
            angular = 0.5 * (right_normalized - left_normalized)
            linear = max(-1.0, min(1.0, linear)) * self._manual_max_linear_mps
            angular = max(-1.0, min(1.0, angular)) * self._manual_max_angular_rad_s
            self._publish_manual(linear, angular)
        return "OK M"

    def _handle_navigation(self, action: str) -> str:
        # Every navigation command is fail-closed in stage 1. PAUSE and STOP
        # are accepted as safe state changes; START/RESUME remain explicit
        # errors and cannot produce a Nav2 action goal.
        self._release_manual_owner()
        self._issue_stop(f"websocket {action}")
        if action == "NAV_PAUSE":
            self._nav_state = "PAUSED"
            self._nav_reason = "OPERATOR_PAUSE"
            return "OK,NAV_PAUSE"
        if action == "NAV_STOP":
            self._nav_state = "IDLE"
            self._nav_reason = "OPERATOR_STOP"
            return "OK,NAV_STOP"
        self._nav_state = "AUTO_NOT_READY"
        self._nav_reason = "STAGE1_DISABLED"
        return f"ERR,{action},AUTO_NOT_READY"

    def _on_client_connect(self, client_id: str) -> None:
        del client_id

    def _on_client_disconnect(self, client_id: str) -> None:
        aborted = self._route_store.abort_owner(client_id)
        self._release_manual_owner()
        self._issue_stop("websocket client disconnect")
        if aborted:
            self._publish_route(None)
            self.get_logger().warning("Route upload aborted because its client disconnected")

    def _release_manual_owner(self) -> None:
        with self._manual_lock:
            self._manual_owner = None
            self._last_manual_command_s = 0.0

    def _check_manual_timeout(self) -> None:
        now = time.monotonic()
        with self._manual_lock:
            if self._manual_owner is None:
                return
            if now - self._last_manual_command_s <= self._manual_command_timeout_s:
                return
            self._manual_owner = None
            self._last_manual_command_s = 0.0
        self._issue_stop("gateway manual command timeout")

    def _publish_manual(self, linear_mps: float, angular_rad_s: float) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.twist.linear.x = linear_mps
        message.twist.angular.z = angular_rad_s
        self._manual_publisher.publish(message)

    def _publish_zero(self) -> None:
        self._publish_manual(0.0, 0.0)

    def _issue_stop(self, reason: str) -> None:
        # Zero is intentionally first and is never conditional on service
        # discovery or a successful service response.
        with self._motion_lock:
            self._publish_zero()
        if not self._stop_client.service_is_ready():
            now = time.monotonic()
            if now - self._last_stop_service_warning_s >= 5.0:
                self._last_stop_service_warning_s = now
                self.get_logger().warning(
                    f"/safety/stop unavailable after {reason}; zero Twist was published"
                )
            return
        with self._stop_service_lock:
            if self._stop_service_pending:
                return
            self._stop_service_pending = True
        try:
            future = self._stop_client.call_async(Trigger.Request())
        except Exception as exc:
            with self._stop_service_lock:
                self._stop_service_pending = False
            if not self._destroying:
                self.get_logger().warning(
                    f"could not send /safety/stop after {reason}: {exc}"
                )
            return

        def _log_failure(completed) -> None:
            with self._stop_service_lock:
                self._stop_service_pending = False
            if self._destroying:
                return
            try:
                response = completed.result()
                if response is None or not response.success:
                    detail = response.message if response is not None else "no response"
                    self.get_logger().warning(
                        f"/safety/stop rejected after {reason}: {detail}"
                    )
            except Exception as exc:
                self.get_logger().warning(f"/safety/stop failed after {reason}: {exc}")

        future.add_done_callback(_log_failure)

    def _publish_route(self, route: Optional[StoredRoute]) -> None:
        validity = Bool()
        validity.data = route is not None

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self._route_frame_id
        if route is not None:
            points = route.points
            for index, point in enumerate(points):
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = point.x_m
                pose.pose.position.y = point.y_m
                if len(points) > 1:
                    other = points[index + 1] if index + 1 < len(points) else points[index - 1]
                    if index + 1 < len(points):
                        dx = other.x_m - point.x_m
                        dy = other.y_m - point.y_m
                    else:
                        dx = point.x_m - other.x_m
                        dy = point.y_m - other.y_m
                    yaw = math.atan2(dy, dx)
                    pose.pose.orientation.z = math.sin(0.5 * yaw)
                    pose.pose.orientation.w = math.cos(0.5 * yaw)
                else:
                    pose.pose.orientation.w = 1.0
                path.poses.append(pose)
        if route is None:
            # Remove AUTO eligibility before withdrawing a previously latched
            # path. DDS has no cross-topic transaction, so future consumers
            # must still correlate their own route version before acting.
            self._route_valid_publisher.publish(validity)
            self._route_path_publisher.publish(path)
        else:
            # Publish the complete immutable Path before opening the route
            # validity gate. A future action client must add explicit version
            # correlation rather than assuming cross-topic atomic delivery.
            self._route_path_publisher.publish(path)
            self._route_valid_publisher.publish(validity)

    def _store_telemetry(self, field: str, message: object) -> None:
        with self._telemetry_lock:
            setattr(self._telemetry, field, _Received(message, time.monotonic()))

    def _on_gps(self, message: NavSatFix) -> None:
        self._store_telemetry("gps", message)

    def _on_imu(self, message: Imu) -> None:
        self._store_telemetry("imu", message)

    def _on_rtk(self, message: RtkStatus) -> None:
        self._store_telemetry("rtk", message)

    def _on_power(self, message: PowerStatus) -> None:
        self._store_telemetry("power", message)

    def _on_relays(self, message: RelayStatus) -> None:
        self._store_telemetry("relays", message)

    def _on_robot(self, message: RobotStatus) -> None:
        self._store_telemetry("robot", message)

    def _on_safety(self, message: SafetyState) -> None:
        self._store_telemetry("safety", message)

    def _on_fault(self, message: FaultEvent) -> None:
        self._store_telemetry("fault", message)
        if hasattr(self, "_websocket"):
            self._websocket.broadcast(
                (
                    "FAULT,"
                    f"{message.fault_code},{_csv_text(message.fault_name)},"
                    f"{_csv_text(message.detail)},{int(message.latched)}",
                )
            )

    def _broadcast_telemetry(self) -> None:
        with self._telemetry_lock:
            telemetry = _Telemetry(**vars(self._telemetry))
        now = time.monotonic()
        route = self._route_store.snapshot()
        messages = []

        gps = telemetry.gps.message if telemetry.gps else None
        rtk = telemetry.rtk.message if telemetry.rtk else None
        safety = telemetry.safety.message if telemetry.safety else None
        gps_coordinates_valid = (
            isinstance(gps, NavSatFix)
            and math.isfinite(gps.latitude)
            and math.isfinite(gps.longitude)
            and -90.0 <= gps.latitude <= 90.0
            and -180.0 <= gps.longitude <= 180.0
        )
        if gps_coordinates_valid:
            heading_deg = (
                math.degrees(rtk.motion_heading_rad) if isinstance(rtk, RtkStatus) else 0.0
            )
            if not math.isfinite(heading_deg):
                heading_deg = 0.0
            fix_type = rtk.fix_type if isinstance(rtk, RtkStatus) else gps.status.status
            horizontal_accuracy_m = (
                rtk.horizontal_accuracy_m
                if isinstance(rtk, RtkStatus)
                else math.sqrt(max(0.0, gps.position_covariance[0]))
            )
            h_acc_mm = _accuracy_mm(horizontal_accuracy_m)
            messages.append(
                f"GPS,{gps.latitude:.8f},{gps.longitude:.8f},"
                f"{heading_deg:.2f},{fix_type},{h_acc_mm}"
            )
            if isinstance(rtk, RtkStatus):
                carrier = CARRIER_NAMES.get(rtk.carrier_solution, "UNKNOWN")
                messages.append(
                    f"GPSDBG,{gps.latitude:.8f},{gps.longitude:.8f},"
                    f"{gps.altitude:.3f},{heading_deg:.2f},{rtk.fix_type},{carrier},"
                    f"{int(rtk.carrier_solution != RtkStatus.CARRIER_NONE)},"
                    f"{rtk.satellites},{_accuracy_mm(rtk.horizontal_accuracy_m)},"
                    f"{_accuracy_mm(rtk.vertical_accuracy_m)},"
                    f"{rtk.ground_speed_mps:.3f},-1.0,{_age_ms(telemetry.gps, now)}"
                )

        if isinstance(rtk, RtkStatus):
            messages.append(
                f"RTK,{CARRIER_NAMES.get(rtk.carrier_solution, 'UNKNOWN')},"
                f"{rtk.horizontal_accuracy_m:.3f},{rtk.vertical_accuracy_m:.3f},"
                f"{rtk.rtcm_age_ms},{rtk.satellites},{rtk.fix_type}"
            )

        imu = telemetry.imu.message if telemetry.imu else None
        if isinstance(imu, Imu):
            yaw_deg, orientation_valid = _quaternion_yaw_deg(imu)
            heading_initialized = bool(
                isinstance(safety, SafetyState) and safety.heading_initialized
            )
            age = _age_ms(telemetry.imu, now)
            # Legacy IMU yaw is explicitly marked not fresh until the Safety
            # state confirms heading initialization and covariance is present.
            yaw_fresh = orientation_valid and heading_initialized and age <= 250
            messages.append(f"IMU,{yaw_deg:.2f},{age},{int(yaw_fresh)}")
            messages.append(
                "IMU_SUMMARY,"
                f"{age},{imu.angular_velocity.x:.5f},{imu.angular_velocity.y:.5f},"
                f"{imu.angular_velocity.z:.5f},{imu.linear_acceleration.x:.5f},"
                f"{imu.linear_acceleration.y:.5f},{imu.linear_acceleration.z:.5f},"
                f"{int(orientation_valid)},{int(heading_initialized)}"
            )

        power = telemetry.power.message if telemetry.power else None
        if isinstance(power, PowerStatus):
            current = f"{power.battery_current_a:.3f}" if power.current_available else "NA"
            state_of_charge = (
                f"{power.state_of_charge_percent:.1f}"
                if power.state_of_charge_available
                else "NA"
            )
            messages.append(
                f"POWER,{power.battery_voltage:.3f},{current},{state_of_charge},"
                f"{int(power.undervoltage)}"
            )
            if power.state_of_charge_available and math.isfinite(
                power.state_of_charge_percent
            ):
                percent = int(round(max(0.0, min(100.0, power.state_of_charge_percent))))
                messages.append(f"BAT_PCT,{percent}")

        relays = telemetry.relays.message if telemetry.relays else None
        if isinstance(relays, RelayStatus):
            messages.append(f"RELAY,{relays.available_mask},{relays.active_mask}")

        robot = telemetry.robot.message if telemetry.robot else None
        if isinstance(robot, RobotStatus):
            messages.append(
                f"ROBOT,{ROBOT_STATE_NAMES.get(robot.state, 'UNKNOWN')},"
                f"{int(robot.connected)},{int(robot.armed)},{int(robot.estop)},"
                f"{robot.fault_code},{_csv_text(robot.fault_reason)}"
            )

        fault = telemetry.fault.message if telemetry.fault else None
        if isinstance(fault, FaultEvent):
            messages.append(
                f"FAULT_STATE,{fault.fault_code},{_csv_text(fault.fault_name)},"
                f"{_csv_text(fault.detail)},{int(fault.latched)}"
            )

        safety_name = (
            SAFETY_STATE_NAMES.get(safety.state, "UNKNOWN")
            if isinstance(safety, SafetyState)
            else "UNKNOWN"
        )
        safety_reason = (
            _csv_text(safety.current_candidate_status)
            if isinstance(safety, SafetyState)
            else "NO_SAFETY_STATE"
        )
        messages.append(f"SAFETY,{safety_name},{safety_reason}")
        messages.append(
            f"ROUTE_STATE,{route.status},{route.received_count},"
            f"{route.expected_count},{int(route.valid)},{_csv_text(route.error)}"
        )
        waypoint_total = route.expected_count if route.valid else 0
        messages.append(
            f"NAV,{self._nav_state},-1,{waypoint_total},-1.0,0.0,0.0,0,0,"
            f"{_csv_text(self._nav_reason)}"
        )
        self._websocket.broadcast(messages)

    def destroy_node(self) -> bool:
        if self._destroying:
            return True
        self._destroying = True
        self._release_manual_owner()
        self._issue_stop("gateway shutdown")
        if hasattr(self, "_websocket"):
            self._websocket.stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[GatewayNode] = None
    try:
        node = GatewayNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
