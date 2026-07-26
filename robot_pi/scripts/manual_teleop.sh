#!/usr/bin/env bash
set -Eeuo pipefail

die() { printf '[manual_teleop] ERROR: %s\n' "$*" >&2; exit 1; }

[[ -t 0 && -t 1 ]] || die 'an interactive SSH terminal is required'
[[ -r /opt/ros/jazzy/setup.bash ]] || die 'ROS 2 Jazzy is not installed'

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace="$(cd -- "$script_dir/../ros2_ws" && pwd -P)"
[[ -r "$workspace/install/setup.bash" ]] || die 'workspace is not built'

# ROS setup scripts can reference unset variables even though this wrapper is
# intentionally strict everywhere else.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1090
source "$workspace/install/setup.bash"
set -u

exec python3 -u "$script_dir/manual_teleop.py" "$@"
