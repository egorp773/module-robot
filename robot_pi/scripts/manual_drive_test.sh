#!/usr/bin/env bash
set -Eeuo pipefail

die() { printf '[manual_drive_test] ERROR: %s\n' "$*" >&2; exit 1; }
log() { printf '[manual_drive_test] %s\n' "$*"; }

[[ -r /opt/ros/jazzy/setup.bash ]] || die 'ROS 2 Jazzy is not installed'
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace="$(cd -- "$script_dir/../ros2_ws" && pwd -P)"
[[ -r "$workspace/install/setup.bash" ]] || die 'Workspace is not built'
# shellcheck disable=SC1090
source "$workspace/install/setup.bash"

armed=false
pub_pid=''
last_service_response=''
twist_message() {
  printf '{header: {frame_id: base_link}, twist: {linear: {x: %s}, angular: {z: %s}}}' "$1" "$2"
}
publish_for() {
  local linear="$1" angular="$2" seconds="$3"
  ros2 topic pub --rate 50 /cmd_vel_manual geometry_msgs/msg/TwistStamped \
    "$(twist_message "$linear" "$angular")" >/dev/null 2>&1 &
  pub_pid=$!
  sleep "$seconds"
  if ! kill -0 "$pub_pid" 2>/dev/null; then
    wait "$pub_pid" 2>/dev/null || true
    pub_pid=''
    die 'cmd_vel publisher exited before the requested interval completed'
  fi
  kill -INT "$pub_pid" 2>/dev/null || true
  wait "$pub_pid" 2>/dev/null || true
  pub_pid=''
}
best_effort_stop() {
  [[ -n "$pub_pid" ]] && kill -INT "$pub_pid" 2>/dev/null || true
  [[ -n "$pub_pid" ]] && wait "$pub_pid" 2>/dev/null || true
  pub_pid=''
  timeout 2 ros2 topic pub --once /cmd_vel_manual geometry_msgs/msg/TwistStamped \
    "$(twist_message 0.0 0.0)" >/dev/null 2>&1 || true
  timeout 2 ros2 service call /safety/stop std_srvs/srv/Trigger '{}' >/dev/null 2>&1 || true
}
call_expect_success() {
  local service="$1" service_type="$2" request="$3"
  last_service_response="$(timeout 8 ros2 service call "$service" "$service_type" "$request")" || \
    die "Service call failed or timed out: $service"
  printf '%s\n' "$last_service_response"
  grep -Eq 'success[=:][[:space:]]*(true|True)' <<<"$last_service_response" || \
    die "Service rejected the request: $service"
}
hard_stop() {
  [[ -n "$pub_pid" ]] && kill -INT "$pub_pid" 2>/dev/null || true
  [[ -n "$pub_pid" ]] && wait "$pub_pid" 2>/dev/null || true
  pub_pid=''
  timeout 2 ros2 topic pub --once /cmd_vel_manual geometry_msgs/msg/TwistStamped \
    "$(twist_message 0.0 0.0)" >/dev/null 2>&1 || \
    die 'Failed to publish the zero manual command'
  call_expect_success /safety/stop std_srvs/srv/Trigger '{}'
}
cleanup() {
  best_effort_stop
  if "$armed"; then
    timeout 2 ros2 service call /safety/disarm module_robot_msgs/srv/Disarm '{}' >/dev/null 2>&1 || true
  fi
}
exit_on_signal() {
  local status="$1"
  # Do not resume the motion sequence after an interactive interrupt. Remove
  # the traps first so cleanup itself cannot recurse if the terminal closes.
  trap - EXIT INT TERM
  cleanup
  exit "$status"
}
trap cleanup EXIT
trap 'exit_on_signal 130' INT
trap 'exit_on_signal 143' TERM

timeout 5 ros2 service type /safety/arm >/dev/null || die '/safety/arm is unavailable; start manual bringup first'
status="$(timeout 5 ros2 topic echo --once /esp32/status 2>/dev/null)" || die 'No /esp32/status sample; check serial connection'
printf '%s\n' "$status"
grep -q 'connected: true' <<<"$status" || die 'ESP32 is not connected'

log 'Forcing the known-safe DISARMED state before any prompt'
call_expect_success /safety/disarm module_robot_msgs/srv/Disarm '{}'
sleep 0.3
status="$(timeout 5 ros2 topic echo --once /esp32/status 2>/dev/null)" || die 'Status disappeared after DISARM'
grep -Eq 'state:[[:space:]]*2([[:space:]]|$)' <<<"$status" || die 'ESP32 did not report DISARMED (state=2)'
grep -Eq 'applied_left_command:[[:space:]]*0([[:space:]]|$)' <<<"$status" || die 'Left command is not zero while DISARMED'
grep -Eq 'applied_right_command:[[:space:]]*0([[:space:]]|$)' <<<"$status" || die 'Right command is not zero while DISARMED'
grep -Eq 'uart_speed:[[:space:]]*0([[:space:]]|$)' <<<"$status" || die 'UART speed is not zero while DISARMED'
grep -Eq 'uart_steer:[[:space:]]*0([[:space:]]|$)' <<<"$status" || die 'UART steer is not zero while DISARMED'
printf '%s\n' "$status"

cat <<'EOF'

This test will ARM the robot and request 0.03 m/s forward for one second.
Before continuing:
  * put the robot on blocks so both tracks are clear of people and ground;
  * keep a physical power disconnect/ESTOP within reach;
  * remove the attachment and switch its power off;
  * have a second person watch the tracks if possible.
EOF
read -r -p 'Type I_HAVE_LIFTED_THE_ROBOT exactly: ' confirmation
[[ "$confirmation" == I_HAVE_LIFTED_THE_ROBOT ]] || die 'Operator did not confirm the safe setup'

nonce="$(( ( $(date +%s) ^ $$ ) & 0xFFFFFFFF ))"
((nonce != 0)) || nonce=1
log 'Requesting operator MANUAL ARM; ARM itself must not move the robot'
# From this point cleanup must attempt DISARM even if the CLI is interrupted
# after the request reaches Safety but before its response reaches this shell.
armed=true
call_expect_success /safety/arm module_robot_msgs/srv/Arm \
  "{arm_nonce: ${nonce}, requested_mode: 1}"
armed_confirmed=false
for _ in {1..15}; do
  status="$(timeout 2 ros2 topic echo --once /esp32/status 2>/dev/null || true)"
  if grep -Eq 'state:[[:space:]]*3([[:space:]]|$)' <<<"$status" &&
     grep -q 'armed: true' <<<"$status"; then
    armed_confirmed=true
    break
  fi
  sleep 0.2
done
"$armed_confirmed" || die 'ESP32 did not confirm ARMED; no motion command was sent'

log 'Sending zero commands for one second'
publish_for 0.0 0.0 1.0
status="$(timeout 5 ros2 topic echo --once /esp32/status 2>/dev/null)" || die 'No status after the ARMED zero command'
grep -Eq 'state:[[:space:]]*3([[:space:]]|$)' <<<"$status" || die 'ESP32 left ARMED during the zero-command check'
grep -Eq 'applied_left_command:[[:space:]]*0([[:space:]]|$)' <<<"$status" || die 'Left output moved during the zero-command check'
grep -Eq 'applied_right_command:[[:space:]]*0([[:space:]]|$)' <<<"$status" || die 'Right output moved during the zero-command check'
grep -Eq 'uart_speed:[[:space:]]*0([[:space:]]|$)' <<<"$status" || die 'UART speed moved during the zero-command check'
grep -Eq 'uart_steer:[[:space:]]*0([[:space:]]|$)' <<<"$status" || die 'UART steer moved during the zero-command check'
log 'Sending the low forward command for one second'
publish_for 0.03 0.0 1.0
log 'Sending hard STOP (no ramp-down)'
hard_stop
sleep 0.4

log 'Motor feedback after STOP:'
timeout 5 ros2 topic echo --once /motor/status || die 'No motor feedback after STOP'
status="$(timeout 5 ros2 topic echo --once /esp32/status 2>/dev/null)" || die 'No ESP32 status after STOP'
printf '%s\n' "$status"
grep -Eq 'state:[[:space:]]*3([[:space:]]|$)' <<<"$status" || die 'STOP did not preserve ESP32 ARMED state'
grep -q 'armed: true' <<<"$status" || die 'ESP32 no longer reports ARMED after STOP'
grep -Eq 'applied_left_command:[[:space:]]*0([[:space:]]|$)' <<<"$status" || die 'Left command did not reach zero'
grep -Eq 'applied_right_command:[[:space:]]*0([[:space:]]|$)' <<<"$status" || die 'Right command did not reach zero'
grep -Eq 'uart_speed:[[:space:]]*0([[:space:]]|$)' <<<"$status" || die 'UART speed did not reach zero'
grep -Eq 'uart_steer:[[:space:]]*0([[:space:]]|$)' <<<"$status" || die 'UART steer did not reach zero'
motor_age_ms="$(awk '/^last_motor_feedback_age_ms:[[:space:]]*[0-9]+/{print $2; exit}' <<<"$status")"
[[ "$motor_age_ms" =~ ^[0-9]+$ ]] || die 'Motor feedback age is unavailable after STOP'
((motor_age_ms <= 500)) || die "Motor feedback is stale after STOP (${motor_age_ms} ms)"

log 'DISARMING'
call_expect_success /safety/disarm module_robot_msgs/srv/Disarm '{}'
sleep 0.3
status="$(timeout 5 ros2 topic echo --once /esp32/status 2>/dev/null)" || die 'No ESP32 status after final DISARM'
grep -Eq 'state:[[:space:]]*2([[:space:]]|$)' <<<"$status" || die 'Final state is not DISARMED'
grep -Eq 'applied_left_command:[[:space:]]*0([[:space:]]|$)' <<<"$status" || die 'Left command is non-zero after final DISARM'
grep -Eq 'applied_right_command:[[:space:]]*0([[:space:]]|$)' <<<"$status" || die 'Right command is non-zero after final DISARM'
grep -Eq 'uart_speed:[[:space:]]*0([[:space:]]|$)' <<<"$status" || die 'UART speed is non-zero after final DISARM'
grep -Eq 'uart_steer:[[:space:]]*0([[:space:]]|$)' <<<"$status" || die 'UART steer is non-zero after final DISARM'
armed=false
trap - EXIT INT TERM
log 'Manual drive sequence completed. Track signs still require visual calibration.'
