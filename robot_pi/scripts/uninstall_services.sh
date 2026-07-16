#!/usr/bin/env bash
set -Eeuo pipefail

[[ ${EUID} -eq 0 ]] || { printf 'Run with sudo\n' >&2; exit 1; }
units=(module-robot-bringup.service module-robot-bridge.service)

# Best effort only: the bridge shutdown handler and ESP32 watchdog remain the
# authoritative fallback if ROS discovery or a service call is unavailable.
if [[ -r /etc/module-robot/runtime.env && -r /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /etc/module-robot/runtime.env
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  if [[ -n "${MODULE_ROBOT_ROOT:-}" && -r "${MODULE_ROBOT_ROOT}/robot_pi/ros2_ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "${MODULE_ROBOT_ROOT}/robot_pi/ros2_ws/install/setup.bash"
    timeout 3 ros2 service call /safety/stop std_srvs/srv/Trigger '{}' >/dev/null 2>&1 || true
    timeout 3 ros2 service call /safety/disarm module_robot_msgs/srv/Disarm '{}' >/dev/null 2>&1 || true
  fi
fi

for unit in "${units[@]}"; do
  systemctl stop "$unit" 2>/dev/null || true
done
for unit in "${units[@]}"; do
  systemctl disable "$unit" 2>/dev/null || true
  rm -f "/etc/systemd/system/$unit"
done
systemctl daemon-reload
systemctl reset-failed "${units[@]}" 2>/dev/null || true
printf 'Module Robot services removed. Workspace, udev rule, configs and logs were preserved.\n'
