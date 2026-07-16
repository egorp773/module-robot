#!/usr/bin/env bash
set -Eeuo pipefail

die() { printf '[pre_arm_hardware_probe] ERROR: %s\n' "$*" >&2; exit 1; }
log() { printf '[pre_arm_hardware_probe] %s\n' "$*"; }

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace="$(cd -- "$script_dir/../ros2_ws" && pwd -P)"
report_dir="${MODULE_ROBOT_REPORT_DIR:-${XDG_STATE_HOME:-$HOME/.local/state}/module-robot}"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
umask 077
mkdir -p -- "$report_dir"
report_file="$(mktemp "${report_dir}/pre_arm_hardware_probe_${timestamp}_XXXXXX.txt")" || \
  die "Cannot create a unique report in $report_dir"
exec 3>&1 4>&2
exec > >(tee "$report_file") 2>&1
report_writer_pid=$!

best_effort_safe_state() {
  if command -v ros2 >/dev/null 2>&1; then
    timeout 3 ros2 service call /safety/stop std_srvs/srv/Trigger '{}' >/dev/null 2>&1 || true
    timeout 3 ros2 service call /safety/disarm module_robot_msgs/srv/Disarm '{}' >/dev/null 2>&1 || true
  fi
}
on_signal() {
  local status="$1"
  trap - INT TERM
  exit "$status"
}
finalize() {
  local status=$? writer_status=0
  trap - EXIT INT TERM
  set +e
  log 'Final best-effort STOP and DISARM cleanup.'
  best_effort_safe_state

  # Close both write ends of the report pipe, then wait for tee. A disk/I/O
  # failure must override an otherwise successful hardware result.
  exec 1>&3 2>&4
  wait "$report_writer_pid"
  writer_status=$?
  if ((writer_status != 0)); then
    printf '[pre_arm_hardware_probe] ERROR: report writer failed with exit %d: %s\n' \
      "$writer_status" "$report_file" >&2
    status=1
  elif [[ ! -s "$report_file" ]]; then
    printf '[pre_arm_hardware_probe] ERROR: report is empty: %s\n' "$report_file" >&2
    status=1
  fi
  exit "$status"
}
trap finalize EXIT
trap 'on_signal 130' INT
trap 'on_signal 143' TERM

log "Timestamp (UTC): $timestamp"
log "Report: $report_file"
log 'This probe is observation-only: it removes motion authority and never enables motion or relays.'

[[ -r /opt/ros/jazzy/setup.bash ]] || die 'ROS 2 Jazzy is not installed'
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
[[ -r "$workspace/install/setup.bash" ]] || die "Workspace is not built: $workspace"
# shellcheck disable=SC1090
source "$workspace/install/setup.bash"

required_topics=(
  /esp32/status
  /motor/status
  /power/status
  /esp32/protocol_stats
  /relay/status
)
topic_list="$(timeout 5 ros2 topic list)" || die 'Unable to query the ROS graph'
for topic in "${required_topics[@]}"; do
  grep -Fqx -- "$topic" <<<"$topic_list" || die "Required topic is absent: $topic"
  log "Found topic: $topic"
done

service_type="$(timeout 5 ros2 service type /safety/disarm)" || die '/safety/disarm is unavailable'
[[ "$service_type" == 'module_robot_msgs/srv/Disarm' ]] || \
  die "Unexpected /safety/disarm type: $service_type"
log 'Forcing DISARM before telemetry observation'
disarm_response="$(timeout 8 ros2 service call /safety/disarm module_robot_msgs/srv/Disarm '{}')" || \
  die 'DISARM service call failed or timed out'
printf '%s\n' "$disarm_response"
grep -Eq 'success[=:][[:space:]]*(true|True)' <<<"$disarm_response" || \
  die 'Safety rejected DISARM'

log 'Starting typed telemetry observation (10 seconds after DDS warm-up)'
python3 -u - <<'PY'
import math
import sys
import time

import rclpy
from module_robot_msgs.msg import MotorStatus, PowerStatus, ProtocolStats, RelayStatus, RobotStatus
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


OBSERVATION_SECONDS = 10.0
WARMUP_SECONDS = 2.0
MAX_FEEDBACK_AGE_MS = 500
MAX_FEEDBACK_GAP_S = 0.5
RAW_ZERO_LIMIT = 5


class HardwareProbe(Node):
    def __init__(self):
        super().__init__("pre_arm_hardware_probe")
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
        self.status = []
        self.motor = []
        self.power = []
        self.protocol = []
        self.relay = []
        self._probe_subscriptions = [
            self.create_subscription(
                RobotStatus, "/esp32/status", self._on_status, state_qos
            ),
            self.create_subscription(
                MotorStatus, "/motor/status", self._on_motor, sensor_qos
            ),
            self.create_subscription(
                PowerStatus, "/power/status", self._on_power, state_qos
            ),
            self.create_subscription(
                ProtocolStats,
                "/esp32/protocol_stats",
                self._on_protocol,
                state_qos,
            ),
            self.create_subscription(
                RelayStatus, "/relay/status", self._on_relay, state_qos
            ),
        ]

    def clear(self):
        self.status.clear()
        self.motor.clear()
        self.power.clear()
        self.protocol.clear()
        self.relay.clear()

    def _on_status(self, msg):
        self.status.append(
            (
                time.monotonic(),
                {
                    "state": msg.state,
                    "connected": msg.connected,
                    "armed": msg.armed,
                    "applied_left": msg.applied_left_command,
                    "applied_right": msg.applied_right_command,
                    "uart_speed": msg.uart_speed,
                    "uart_steer": msg.uart_steer,
                    "motor_age_ms": msg.last_motor_feedback_age_ms,
                    "watchdog_trips": msg.watchdog_trips,
                },
            )
        )

    def _on_motor(self, msg):
        self.motor.append(
            (
                time.monotonic(),
                {
                    "sensor_us": msg.sensor_monotonic_us,
                    "left": msg.left_feedback,
                    "right": msg.right_feedback,
                    "battery_v": msg.battery_voltage,
                    "temperature_c": msg.board_temperature_c,
                    "temperature_available": msg.board_temperature_available,
                    "controller_fault": msg.controller_fault,
                    "valid_frames": msg.uart_valid_frames,
                },
            )
        )

    def _on_power(self, msg):
        self.power.append(
            (
                time.monotonic(),
                {
                    "battery_v": msg.battery_voltage,
                    "voltage_available": msg.voltage_available,
                },
            )
        )

    def _on_protocol(self, msg):
        self.protocol.append(
            (
                time.monotonic(),
                {
                    "crc_errors": msg.crc_errors,
                    "cobs_errors": msg.cobs_errors,
                    "length_errors": msg.length_errors,
                    "esp_crc_errors": msg.esp_crc_errors,
                    "esp_cobs_errors": msg.esp_cobs_errors,
                    "esp_length_errors": msg.esp_length_errors,
                },
            )
        )

    def _on_relay(self, msg):
        self.relay.append((time.monotonic(), {"active_mask": msg.active_mask}))


def sample_values(samples, field):
    return [sample[field] for _, sample in samples]


failures = []


def check(condition, message):
    if condition:
        print(f"[PASS] {message}")
    else:
        print(f"[FAIL] {message}")
        failures.append(message)


rclpy.init()
node = HardwareProbe()
try:
    warmup_end = time.monotonic() + WARMUP_SECONDS
    while time.monotonic() < warmup_end:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.clear()
    observation_start = time.monotonic()
    observation_end = observation_start + OBSERVATION_SECONDS
    while time.monotonic() < observation_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    observation_finish = time.monotonic()

    print(f"Observation duration: {observation_finish - observation_start:.3f} s")
    print(
        "Samples: "
        f"status={len(node.status)}, motor={len(node.motor)}, power={len(node.power)}, "
        f"protocol={len(node.protocol)}, relay={len(node.relay)}"
    )

    check(len(node.status) >= 2, "ESP32 status telemetry received repeatedly")
    check(len(node.motor) >= 2, "motor telemetry received repeatedly")
    check(len(node.power) >= 2, "power telemetry received repeatedly")
    check(len(node.protocol) >= 2, "protocol statistics received repeatedly")
    check(len(node.relay) >= 2, "relay telemetry received repeatedly")

    if node.status:
        states = sample_values(node.status, "state")
        check(all(sample_values(node.status, "connected")), "ESP32 remained connected")
        check(
            all(state == RobotStatus.STATE_DISARMED for state in states),
            f"ESP32 remained DISARMED (observed states: {sorted(set(states))})",
        )
        check(not any(sample_values(node.status, "armed")), "ESP32 never reported ARMED")
        for field, label in (
            ("applied_left", "applied left command"),
            ("applied_right", "applied right command"),
            ("uart_speed", "UART speed"),
            ("uart_steer", "UART steer"),
        ):
            values = sample_values(node.status, field)
            check(all(value == 0 for value in values), f"{label} stayed zero")
        ages = sample_values(node.status, "motor_age_ms")
        check(
            all(age <= MAX_FEEDBACK_AGE_MS for age in ages),
            f"motor feedback age stayed <= {MAX_FEEDBACK_AGE_MS} ms "
            f"(observed {min(ages)}..{max(ages)} ms)",
        )
        watchdog = sample_values(node.status, "watchdog_trips")
        check(
            len(set(watchdog)) == 1,
            f"watchdog count did not change (observed {watchdog[0]}..{watchdog[-1]})",
        )

    if node.motor:
        motor_times = [stamp for stamp, _ in node.motor]
        gaps = [later - earlier for earlier, later in zip(motor_times, motor_times[1:])]
        first_delay = motor_times[0] - observation_start
        final_age = observation_finish - motor_times[-1]
        max_gap = max(gaps, default=0.0)
        minimum_samples = math.floor(OBSERVATION_SECONDS / MAX_FEEDBACK_GAP_S)
        check(
            len(node.motor) >= minimum_samples
            and first_delay <= MAX_FEEDBACK_GAP_S
            and final_age <= MAX_FEEDBACK_GAP_S
            and max_gap <= MAX_FEEDBACK_GAP_S,
            "motor feedback was regular: "
            f"count={len(node.motor)}, first_delay={first_delay:.3f}s, "
            f"max_gap={max_gap:.3f}s, final_age={final_age:.3f}s",
        )
        left = sample_values(node.motor, "left")
        right = sample_values(node.motor, "right")
        print(f"Raw feedback at rest: left min/max={min(left)}/{max(left)}")
        print(f"Raw feedback at rest: right min/max={min(right)}/{max(right)}")
        check(
            min(left) >= -RAW_ZERO_LIMIT
            and max(left) <= RAW_ZERO_LIMIT
            and min(right) >= -RAW_ZERO_LIMIT
            and max(right) <= RAW_ZERO_LIMIT,
            f"raw feedback stayed within +/-{RAW_ZERO_LIMIT}",
        )
        valid_frames = sample_values(node.motor, "valid_frames")
        check(
            len(set(valid_frames)) >= 2,
            f"hoverboard valid-frame counter advanced (observed {valid_frames[0]}..{valid_frames[-1]})",
        )
        faults = sample_values(node.motor, "controller_fault")
        check(
            all(value == 0 for value in faults),
            f"motor controller fault stayed zero (observed {sorted(set(faults))})",
        )
        temperatures = [
            sample["temperature_c"]
            for _, sample in node.motor
            if sample["temperature_available"] and math.isfinite(sample["temperature_c"])
        ]
        check(
            len(temperatures) == len(node.motor),
            "board temperature was available and finite for every motor sample",
        )
        if temperatures:
            print(
                f"Board temperature: {temperatures[-1]:.1f} C "
                f"(range {min(temperatures):.1f}..{max(temperatures):.1f} C)"
            )

    if node.power:
        voltage_available = sample_values(node.power, "voltage_available")
        voltages = sample_values(node.power, "battery_v")
        valid_voltages = [
            value
            for value, available in zip(voltages, voltage_available)
            if available and math.isfinite(value)
        ]
        check(
            len(valid_voltages) == len(node.power),
            "battery voltage was available and finite for every power sample",
        )
        if valid_voltages:
            print(
                f"Battery voltage: {valid_voltages[-1]:.2f} V "
                f"(range {min(valid_voltages):.2f}..{max(valid_voltages):.2f} V)"
            )

    if node.protocol:
        for field, label in (
            ("crc_errors", "Pi CRC errors"),
            ("cobs_errors", "Pi COBS errors"),
            ("length_errors", "Pi length errors"),
            ("esp_crc_errors", "ESP32 CRC errors"),
            ("esp_cobs_errors", "ESP32 COBS errors"),
            ("esp_length_errors", "ESP32 length errors"),
        ):
            values = sample_values(node.protocol, field)
            check(
                len(set(values)) == 1,
                f"{label} did not change (observed {values[0]}..{values[-1]})",
            )

    if node.relay:
        masks = sample_values(node.relay, "active_mask")
        check(
            all(mask == 0 for mask in masks),
            f"relay active mask stayed zero (observed {sorted(set(masks))})",
        )
finally:
    node.destroy_node()
    rclpy.shutdown()

if failures:
    print(f"RESULT: FAIL ({len(failures)} failed checks)")
    sys.exit(1)

print("RESULT: PASS")
PY

log 'Probe checks completed successfully.'
