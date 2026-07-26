#!/usr/bin/env bash
set -euo pipefail

die() { printf '[manual_bringup] ERROR: %s\n' "$*" >&2; exit 1; }
log() { printf '[manual_bringup] %s\n' "$*"; }

(($# == 0)) || die 'This fixed manual-only launcher does not accept overrides'
[[ -r /opt/ros/jazzy/setup.bash ]] || die 'ROS 2 Jazzy is not installed'
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace="$(cd -- "$script_dir/../ros2_ws" && pwd -P)"
[[ -r "$workspace/install/setup.bash" ]] || die 'Workspace is not built'
[[ -e /dev/module-esp32 ]] || die '/dev/module-esp32 is missing'

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1090
source "$workspace/install/setup.bash"
set -u

nodes="$(timeout 6 ros2 node list 2>/dev/null || true)"
for node in /esp32_bridge /safety_node /readiness_monitor; do
  if grep -Fxq "$node" <<<"$nodes"; then
    die "$node is already running; refusing to start a second bringup"
  fi
done

log 'Waking the motor controller through the reviewed zero-first ESP32 reset'
python3 "$script_dir/wake_motor_controller.py"
log 'Starting one manual-only bringup; Ctrl+C stops the foreground launch'
exec ros2 launch module_robot_bringup manual_bringup.launch.py \
  start_description:=false \
  start_gateway:=false \
  start_tools:=true
