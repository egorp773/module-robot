#!/usr/bin/env bash
set -Eeuo pipefail

log() { printf '[configure_pi] %s\n' "$*"; }
die() { printf '[configure_pi] ERROR: %s\n' "$*" >&2; exit 1; }

[[ ${EUID} -eq 0 ]] || die 'Run with sudo'
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
default_root="$(cd -- "${script_dir}/../.." && pwd -P)"
robot_user="${SUDO_USER:-}"
repo_root="$default_root"
install_units=false

while (($#)); do
  case "$1" in
    --user) [[ $# -ge 2 ]] || die '--user needs a value'; robot_user="$2"; shift 2 ;;
    --repo-root) [[ $# -ge 2 ]] || die '--repo-root needs a value'; repo_root="$(realpath "$2")"; shift 2 ;;
    --install-systemd) install_units=true; shift ;;
    -h|--help)
      printf 'Usage: sudo %s --user USER [--repo-root PATH] [--install-systemd]\n' "$0"
      exit 0 ;;
    *) die "Unknown argument: $1" ;;
  esac
done

[[ -n "$robot_user" ]] || die 'Specify the non-root service account with --user USER'
[[ "$robot_user" != root ]] || die 'Robot services must not run as root'
[[ "$robot_user" =~ ^[a-z_][a-z0-9_-]*$ ]] || die 'Unsupported service-account name'
id "$robot_user" >/dev/null 2>&1 || die "User does not exist: $robot_user"
[[ -d "$repo_root/robot_pi/ros2_ws/src" ]] || die "Not a module robot repository: $repo_root"
[[ "$repo_root" =~ ^/[A-Za-z0-9._/-]+$ ]] || \
  die 'Repository path must be absolute and contain only letters, digits, dot, underscore, dash and slash'

usermod -aG dialout "$robot_user"
install -d -m 0755 /etc/module-robot
unit_tmp=''
runtime_tmp="$(mktemp /etc/module-robot/runtime.env.XXXXXX)"
cleanup_temp() {
  [[ -z "$runtime_tmp" ]] || rm -f -- "$runtime_tmp"
  [[ -z "$unit_tmp" ]] || rm -f -- "$unit_tmp"
}
trap cleanup_temp EXIT
printf '%s\n' \
  'ROS_DISTRO=jazzy' \
  'ROS_DOMAIN_ID=0' \
  'RMW_IMPLEMENTATION=rmw_fastrtps_cpp' \
  "MODULE_ROBOT_ROOT=${repo_root}" > "$runtime_tmp"
chmod 0644 "$runtime_tmp"
mv -f "$runtime_tmp" /etc/module-robot/runtime.env
runtime_tmp=''

if "$install_units"; then
  [[ -r /opt/ros/jazzy/setup.bash ]] || die 'ROS 2 Jazzy is not installed; refusing to install unusable units'
  [[ -r "$repo_root/robot_pi/ros2_ws/install/setup.bash" ]] || \
    die 'Workspace is not built; run build_workspace.sh before installing units'
  escaped_root="$(printf '%s' "$repo_root" | sed 's/[&|]/\\&/g')"
  for unit in module-robot-bridge.service module-robot-bringup.service; do
    unit_tmp="$(mktemp "/etc/systemd/system/${unit}.XXXXXX")"
    sed -e "s|@ROBOT_USER@|${robot_user}|g" \
        -e "s|@ROBOT_ROOT@|${escaped_root}|g" \
        "$repo_root/robot_pi/systemd/$unit" > "$unit_tmp"
    if grep -Eq '@ROBOT_(USER|ROOT)@' "$unit_tmp"; then
      die "Unresolved placeholder while rendering $unit"
    fi
    chmod 0644 "$unit_tmp"
    mv -f "$unit_tmp" "/etc/systemd/system/$unit"
    unit_tmp=''
  done
  systemctl daemon-reload
  log 'Systemd units installed; this script did not enable or start them'
  for unit in module-robot-bridge.service module-robot-bringup.service; do
    if systemctl is-enabled --quiet "$unit" 2>/dev/null; then
      log "WARNING: $unit was already enabled; this script preserved that operator state"
    fi
  done
fi

trap - EXIT
log "Configured user ${robot_user}; log out/in (or reboot) for dialout membership to apply"
