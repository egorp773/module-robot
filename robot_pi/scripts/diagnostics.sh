#!/usr/bin/env bash
set -uo pipefail

section() { printf '\n=== %s ===\n' "$*"; }
run() { printf '$ %s\n' "$*"; "$@" 2>&1 || printf '[WARN] command failed (exit %d)\n' "$?"; }

if [[ -r /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
else
  printf '[FAIL] ROS 2 Jazzy is not installed\n' >&2
  exit 1
fi
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace="$(cd -- "$script_dir/../ros2_ws" && pwd -P)"
if [[ -r "$workspace/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "$workspace/install/setup.bash"
else
  printf '[WARN] Workspace has not been built: %s\n' "$workspace"
fi

section 'Host'
run uname -a
run timedatectl status
run df -h /
run id
run lsusb
run ls -l /dev/module-esp32
run udevadm info --query=property --name=/dev/module-esp32
if command -v vcgencmd >/dev/null 2>&1; then
  run vcgencmd get_throttled
fi
run journalctl -k -b -n 100 --no-pager

section 'ROS graph'
run ros2 node list
run ros2 topic list -t
run ros2 service list -t

section 'One-shot safety telemetry'
for topic in /esp32/status /esp32/protocol_stats /motor/status /rtk/status /safety/state /diagnostics; do
  printf '%s\n' "--- $topic"
  timeout 3 ros2 topic echo --once "$topic" 2>&1 || printf '[WARN] no sample from %s\n' "$topic"
done

section 'Rates (five-second samples)'
for topic in /imu/data_raw /gps/fix /motor/status /cmd_vel_safe; do
  printf '%s\n' "--- $topic"
  timeout 5 ros2 topic hz "$topic" 2>&1 || true
done

section 'ROS doctor'
run ros2 doctor --report

section 'Recent service logs'
printf '$ systemctl is-enabled module-robot-bridge.service module-robot-bringup.service\n'
systemctl is-enabled module-robot-bridge.service module-robot-bringup.service 2>&1 || true
run systemctl status module-robot-bridge.service module-robot-bringup.service --no-pager
run journalctl -u module-robot-bridge.service -u module-robot-bringup.service -n 100 --no-pager
