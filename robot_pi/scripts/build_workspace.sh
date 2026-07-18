#!/usr/bin/env bash
set -Eeuo pipefail

log() { printf '[build_workspace] %s\n' "$*"; }
die() { printf '[build_workspace] ERROR: %s\n' "$*" >&2; exit 1; }
usage() {
  cat <<'EOF'
Usage: ./build_workspace.sh [--stage manual|full]

Stages:
  manual  Build and test only commissioning/manual-control packages (default).
  full    Build and test the complete workspace.
EOF
}

stage=manual
while (($#)); do
  case "$1" in
    --stage)
      (($# >= 2)) || die '--stage requires manual or full'
      stage="$2"
      shift 2
      ;;
    --stage=*)
      stage="${1#*=}"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "Unknown argument: $1"
      ;;
  esac
done

case "$stage" in
  manual|full) ;;
  *) die "Invalid stage '${stage}'; expected manual or full" ;;
esac

[[ "$(uname -s)" == Linux ]] || die 'Build this ROS workspace on the Raspberry Pi, not Windows'
[[ -r /opt/ros/jazzy/setup.bash ]] || die 'ROS 2 Jazzy is not installed; run install_pi.sh first'
# shellcheck disable=SC1091
set +u
source /opt/ros/jazzy/setup.bash
set -u
[[ "${ROS_DISTRO:-}" == jazzy ]] || die "Expected ROS_DISTRO=jazzy, got ${ROS_DISTRO:-unset}"

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace="$(cd -- "${script_dir}/../ros2_ws" && pwd -P)"
[[ -d "$workspace/src" ]] || die "Workspace src directory is missing: $workspace/src"

manual_packages=(
  module_robot_msgs
  module_robot_esp32_bridge
  module_robot_safety
  module_robot_gateway
  module_robot_bringup
  module_robot_tools
)
manual_rosdep_skip_keys=(
  module_robot_description
  module_robot_localization
  module_robot_navigation
)

selection_args=()
log "Resolving package dependencies for stage ${stage} with rosdep"
if [[ "$stage" == manual ]]; then
  manual_package_paths=()
  for package in "${manual_packages[@]}"; do
    package_path="${workspace}/src/${package}"
    [[ -f "${package_path}/package.xml" ]] || die "Manual package is missing: ${package_path}"
    manual_package_paths+=("$package_path")
  done
  rosdep install \
    --from-paths "${manual_package_paths[@]}" \
    --ignore-src \
    --rosdistro jazzy \
    --skip-keys="${manual_rosdep_skip_keys[*]}" \
    -y
  # Exact package selection prevents module_robot_bringup's guarded autonomous
  # exec dependencies from pulling Nav2/localization into commissioning builds.
  selection_args=(--packages-select "${manual_packages[@]}")
else
  rosdep install --from-paths "$workspace/src" --ignore-src --rosdistro jazzy -y
fi

log "Building ROS 2 workspace stage ${stage}"
cd "$workspace"
colcon build \
  --symlink-install \
  --event-handlers console_cohesion+ \
  "${selection_args[@]}"

log "Running ROS 2 tests for stage ${stage}"
colcon test \
  --event-handlers console_cohesion+ \
  "${selection_args[@]}"

# colcon test may finish after producing failing test result files. This
# command is therefore an explicit fail gate and must remain under set -e.
colcon test-result --verbose

log "Build and tests complete. Run: source ${workspace}/install/setup.bash"
