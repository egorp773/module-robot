#!/usr/bin/env bash
set -Eeuo pipefail

die() { printf '[stop_test] ERROR: %s\n' "$*" >&2; exit 1; }
log() { printf '[stop_test] %s\n' "$*"; }

[[ -r /opt/ros/jazzy/setup.bash ]] || die 'ROS 2 Jazzy is not installed'
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace="$(cd -- "$script_dir/../ros2_ws" && pwd -P)"
[[ -r "$workspace/install/setup.bash" ]] || die 'Workspace is not built'
# shellcheck disable=SC1090
source "$workspace/install/setup.bash"

pub_pid=''
last_service_response=''
twist='{header: {frame_id: base_link}, twist: {linear: {x: 0.03}, angular: {z: 0.0}}}'
zero='{header: {frame_id: base_link}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}'

stop_publisher() {
  [[ -n "$pub_pid" ]] && kill -INT "$pub_pid" 2>/dev/null || true
  [[ -n "$pub_pid" ]] && wait "$pub_pid" 2>/dev/null || true
  pub_pid=''
}
safe_cleanup() {
  stop_publisher
  timeout 2 ros2 topic pub --once /cmd_vel_manual geometry_msgs/msg/TwistStamped "$zero" >/dev/null 2>&1 || true
  timeout 2 ros2 service call /safety/stop std_srvs/srv/Trigger '{}' >/dev/null 2>&1 || true
  timeout 2 ros2 service call /safety/disarm module_robot_msgs/srv/Disarm '{}' >/dev/null 2>&1 || true
}
exit_on_signal() {
  local status="$1"
  # A terminal interrupt is a terminal condition for this test. Continuing at
  # the next scenario after a cleanup would allow an unexpected re-ARM.
  trap - EXIT INT TERM
  safe_cleanup
  exit "$status"
}
trap safe_cleanup EXIT
trap 'exit_on_signal 130' INT
trap 'exit_on_signal 143' TERM

call_expect_success() {
  local service="$1" service_type="$2" request="$3"
  last_service_response="$(timeout 8 ros2 service call "$service" "$service_type" "$request")" || \
    die "Service call failed or timed out: $service"
  grep -Eq 'success[=:][[:space:]]*(true|True)' <<<"$last_service_response" || \
    die "Service rejected the request: $service: $last_service_response"
}

wait_service() {
  local service="$1"
  for _ in {1..20}; do
    ros2 service type "$service" 2>/dev/null | grep -q . && return 0
    sleep 0.5
  done
  return 1
}
wait_connected() {
  local status
  for _ in {1..30}; do
    status="$(timeout 2 ros2 topic echo --once /esp32/status 2>/dev/null || true)"
    grep -q 'connected: true' <<<"$status" && return 0
    sleep 0.5
  done
  return 1
}
reset_to_disarmed() {
  local status state answer
  status="$(timeout 5 ros2 topic echo --once /esp32/status 2>/dev/null)" || die 'No ESP32 status before ARM'
  state="$(awk '/^state:[[:space:]]*[0-9]+/{print $2; exit}' <<<"$status")"
  if [[ "$state" == 5 ]]; then
    die 'ESTOP is latched. Investigate it and issue a separate explicit RESET_ESTOP before restarting this script.'
  fi
  if [[ "$state" == 4 ]]; then
    printf '%s\n' "$status"
    read -r -p 'FAULT is latched; after understanding it type RESET_FAULT: ' answer
    [[ "$answer" == RESET_FAULT ]] || die 'Fault remains latched'
    call_expect_success /safety/reset_fault module_robot_msgs/srv/ResetFault '{}'
  fi
  call_expect_success /safety/disarm module_robot_msgs/srv/Disarm '{}'
  sleep 0.3
  require_observed_zero 'pre-arm DISARM' 2 false
}
arm_manual() {
  reset_to_disarmed
  local nonce=$(( ( $(date +%s) ^ $$ ^ RANDOM ) & 0xFFFFFFFF ))
  ((nonce != 0)) || nonce=1
  call_expect_success /safety/arm module_robot_msgs/srv/Arm \
    "{arm_nonce: ${nonce}, requested_mode: 1}"
  local status confirmed=false
  for _ in {1..15}; do
    status="$(timeout 2 ros2 topic echo --once /esp32/status 2>/dev/null || true)"
    if grep -Eq 'state:[[:space:]]*3([[:space:]]|$)' <<<"$status" &&
       grep -q 'armed: true' <<<"$status"; then
      confirmed=true
      break
    fi
    sleep 0.2
  done
  "$confirmed" || die 'ESP32 did not confirm ARMED; refusing to start motion publisher'
}
start_motion() {
  ros2 topic pub --rate 50 /cmd_vel_manual geometry_msgs/msg/TwistStamped "$twist" >/dev/null 2>&1 &
  pub_pid=$!
  sleep 0.8
  if ! kill -0 "$pub_pid" 2>/dev/null; then
    wait "$pub_pid" 2>/dev/null || true
    pub_pid=''
    die 'cmd_vel publisher exited before fault injection'
  fi
}
require_observed_zero() {
  local label="$1" expected_state="${2:-}" require_visual="${3:-true}" status
  status="$(timeout 8 ros2 topic echo --once /esp32/status 2>/dev/null || true)"
  if [[ -n "$status" ]] && grep -q 'connected: true' <<<"$status"; then
    grep -Eq 'applied_left_command:[[:space:]]*0([[:space:]]|$)' <<<"$status" || {
      printf '%s\n' "$status"
      die "$label: connected telemetry reports a non-zero left command"
    }
    grep -Eq 'applied_right_command:[[:space:]]*0([[:space:]]|$)' <<<"$status" || {
      printf '%s\n' "$status"
      die "$label: connected telemetry reports a non-zero right command"
    }
    grep -Eq 'uart_speed:[[:space:]]*0([[:space:]]|$)' <<<"$status" || {
      printf '%s\n' "$status"
      die "$label: connected telemetry reports a non-zero UART speed"
    }
    grep -Eq 'uart_steer:[[:space:]]*0([[:space:]]|$)' <<<"$status" || {
      printf '%s\n' "$status"
      die "$label: connected telemetry reports a non-zero UART steer"
    }
    if [[ -n "$expected_state" ]]; then
      grep -Eq "state:[[:space:]]*${expected_state}([[:space:]]|$)" <<<"$status" || {
        printf '%s\n' "$status"
        die "$label: expected ESP32 state ${expected_state}"
      }
    fi
    if [[ "$expected_state" == 5 ]]; then
      grep -q 'estop: true' <<<"$status" || die "$label: ESTOP state is not latched in telemetry"
    fi
    if [[ "$expected_state" == 4 ]]; then
      grep -Eq 'fault_reason:[[:space:]]*CMD_VEL_TIMEOUT([[:space:]]|$)' <<<"$status" || {
        printf '%s\n' "$status"
        die "$label: expected latched CMD_VEL_TIMEOUT fault"
      }
    fi
    log "$label: connected telemetry reports hard zero"
  else
    printf '%s\n' "$status"
    [[ -z "$expected_state" ]] || die "$label: connected telemetry/state ${expected_state} is unavailable"
    log "$label: zero cannot be proved from telemetry; physical verification is mandatory"
  fi
  if [[ "$require_visual" == true ]]; then
    read -r -p "Visually verify both tracks are stopped; type ZERO for ${label}: " answer
    [[ "$answer" == ZERO ]] || die "$label was not physically confirmed"
  fi
}

cat <<'EOF'
DESTRUCTIVE SAFETY TEST — the tracks will receive a 0.03 m/s request.
The robot MUST be securely lifted, attachment power removed, area cleared, and
a physical battery disconnect held by an operator. Tests deliberately kill the
bridge and ask for USB removal. They are never suitable for boot automation.
EOF
read -r -p 'Type RUN_WATCHDOG_STOP_TESTS exactly: ' confirmation
[[ "$confirmation" == RUN_WATCHDOG_STOP_TESTS ]] || die 'Safety setup not confirmed'
wait_service /safety/arm || die 'Manual bringup is not running'
for service in /safety/disarm /safety/stop /safety/estop /safety/reset_fault /safety/reset_estop; do
  wait_service "$service" || die "Required service is unavailable: $service"
done
systemctl is-active --quiet module-robot-bridge.service || \
  die 'Start (do not enable) module-robot-bridge.service before this test'
systemctl is-active --quiet module-robot-bringup.service || \
  die 'Start (do not enable) module-robot-bringup.service before this test'
# Authenticate while outputs are still zero. Never let a password prompt delay
# the bridge-crash injection after motion has started.
sudo -v || die 'sudo authorization is required for the bridge crash scenario'

log 'Scenario 1/5: stale ROS cmd_vel'
arm_manual
start_motion
stop_publisher
# Intentionally do not publish zero: Safety must detect source staleness and STOP.
sleep 0.7
require_observed_zero 'stale cmd_vel'
safe_cleanup

log 'Scenario 2/5: DISARM bypasses the ramp'
arm_manual
start_motion
call_expect_success /safety/disarm module_robot_msgs/srv/Disarm '{}'
stop_publisher
sleep 0.3
require_observed_zero 'DISARM' 2

log 'Scenario 3/5: latched ESTOP bypasses the ramp'
arm_manual
start_motion
call_expect_success /safety/estop std_srvs/srv/Trigger '{}'
stop_publisher
sleep 0.3
require_observed_zero 'ESTOP' 5
read -r -p 'Type RESET_ESTOP to issue the separate explicit reset: ' answer
[[ "$answer" == RESET_ESTOP ]] || die 'ESTOP remains latched; ending safely'
call_expect_success /safety/reset_estop module_robot_msgs/srv/ResetEstop '{}'
call_expect_success /safety/disarm module_robot_msgs/srv/Disarm '{}'
sleep 0.3
require_observed_zero 'RESET_ESTOP result' 2 false

log 'Scenario 4/5: bridge process crash'
arm_manual
start_motion
sudo systemctl kill --kill-whom=all --signal=SIGKILL module-robot-bridge.service
stop_publisher
sleep 3
wait_service /esp32/disarm || die 'Bridge did not recover after process crash'
wait_connected || die 'ESP32 did not reconnect after bridge process crash'
require_observed_zero 'bridge crash/restart' 4
safe_cleanup

log 'Scenario 5/5: physical USB disconnect'
arm_manual
start_motion
read -r -p 'Unplug only the ESP32 USB data cable now, wait >1 s, then type UNPLUGGED: ' answer
[[ "$answer" == UNPLUGGED ]] || die 'USB disconnect was not confirmed'
stop_publisher
sleep 1
require_observed_zero 'USB disconnect'
read -r -p 'Reconnect USB, wait for /dev/module-esp32, then type RECONNECTED: ' answer
[[ "$answer" == RECONNECTED ]] || die 'Reconnect not confirmed'
wait_service /esp32/disarm || die 'Bridge services did not recover after reconnect'
wait_connected || die 'ESP32 did not report a completed HELLO after USB reconnect'
require_observed_zero 'USB reconnect preserves timeout fault' 4 false
reset_to_disarmed
safe_cleanup

trap - EXIT INT TERM
log 'All five STOP scenarios were operator-confirmed; final state is DISARMED. Preserve logs with diagnostics.sh.'
