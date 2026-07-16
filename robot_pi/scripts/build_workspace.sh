#!/usr/bin/env bash
set -Eeuo pipefail

log() { printf '[build_workspace] %s\n' "$*"; }
die() { printf '[build_workspace] ERROR: %s\n' "$*" >&2; exit 1; }

[[ "$(uname -s)" == Linux ]] || die 'Build this ROS workspace on the Raspberry Pi, not Windows'
[[ -r /opt/ros/jazzy/setup.bash ]] || die 'ROS 2 Jazzy is not installed; run install_pi.sh first'
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
[[ "${ROS_DISTRO:-}" == jazzy ]] || die "Expected ROS_DISTRO=jazzy, got ${ROS_DISTRO:-unset}"

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace="$(cd -- "${script_dir}/../ros2_ws" && pwd -P)"
[[ -d "$workspace/src" ]] || die "Workspace src directory is missing: $workspace/src"

log 'Resolving package dependencies with rosdep'
rosdep install --from-paths "$workspace/src" --ignore-src --rosdistro jazzy -r -y

log 'Building the ROS 2 workspace'
cd "$workspace"
colcon build --symlink-install --event-handlers console_cohesion+ "$@"
log "Build complete. Run: source ${workspace}/install/setup.bash"

