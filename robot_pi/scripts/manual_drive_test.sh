#!/usr/bin/env bash
set -Eeuo pipefail

die() { printf '[manual_drive_test] ERROR: %s\n' "$*" >&2; exit 1; }

[[ -r /opt/ros/jazzy/setup.bash ]] || die 'ROS 2 Jazzy is not installed'
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace="$(cd -- "$script_dir/../ros2_ws" && pwd -P)"
[[ -r "$workspace/install/setup.bash" ]] || die 'Workspace is not built'

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1090
source "$workspace/install/setup.bash"
set -u

cat <<'EOF'

This test will MANUAL ARM the robot and request 0.03 m/s forward for one
second. It records typed telemetry and always finishes with zero, STOP, and
DISARM. Both tracks must be lifted and clear; attachment power must be off.
EOF

read -r -p 'Type I_HAVE_LIFTED_THE_ROBOT exactly: ' confirmation
[[ "$confirmation" == I_HAVE_LIFTED_THE_ROBOT ]] || \
  die 'Operator did not confirm the safe setup'

exec python3 -u "$script_dir/manual_calibration_test.py" forward \
  --speed 0.03 \
  --duration 1 \
  --confirm-lifted "$confirmation"
