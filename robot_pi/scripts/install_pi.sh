#!/usr/bin/env bash
set -Eeuo pipefail

readonly EXPECTED_OS_ID="ubuntu"
readonly EXPECTED_VERSION="24.04"
readonly EXPECTED_CODENAME="noble"
readonly EXPECTED_ARCH="arm64"

log() { printf '[install_pi] %s\n' "$*"; }
die() { printf '[install_pi] ERROR: %s\n' "$*" >&2; exit 1; }

[[ -r /etc/os-release ]] || die '/etc/os-release is missing'
# shellcheck disable=SC1091
source /etc/os-release
[[ "${ID:-}" == "$EXPECTED_OS_ID" ]] || die "Ubuntu is required; detected ID=${ID:-unknown}"
[[ "${VERSION_ID:-}" == "$EXPECTED_VERSION" ]] || die "Ubuntu 24.04 is required; detected ${VERSION_ID:-unknown}"
[[ "${VERSION_CODENAME:-}" == "$EXPECTED_CODENAME" ]] || die "Ubuntu Noble is required; detected ${VERSION_CODENAME:-unknown}"
[[ "$(dpkg --print-architecture)" == "$EXPECTED_ARCH" ]] || die "ARM64 is required; detected $(dpkg --print-architecture)"
case "$(uname -m)" in
  aarch64|arm64) ;;
  *) die "64-bit ARM kernel is required; detected $(uname -m)" ;;
esac

if [[ ${EUID} -eq 0 ]]; then
  SUDO=()
  target_user="${SUDO_USER:-root}"
else
  command -v sudo >/dev/null 2>&1 || die 'sudo is required'
  SUDO=(sudo)
  target_user="${USER:-$(id -un)}"
fi

log 'Refreshing Ubuntu package metadata'
"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y ca-certificates curl software-properties-common
"${SUDO[@]}" add-apt-repository -y universe

# Current official ROS instructions use ros2-apt-source so key/source updates
# remain package-managed; do not reintroduce apt-key or a hand-written source.
if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -q 'install ok installed'; then
  log 'Installing the official ROS 2 apt-source package'
  release_json="$(curl --fail --silent --show-error --location \
    https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest)"
  ros_apt_source_version="$(printf '%s' "$release_json" | sed -n 's/.*"tag_name"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p' | head -n1)"
  [[ -n "$ros_apt_source_version" ]] || die 'Could not determine the latest ros-apt-source release'
  ros_apt_source_url="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_source_version}/ros2-apt-source_${ros_apt_source_version}.noble_all.deb"
  tmp_deb="$(mktemp --suffix=.deb)"
  trap 'rm -f "${tmp_deb:-}"' EXIT
  curl --fail --show-error --location "$ros_apt_source_url" --output "$tmp_deb"
  dpkg-deb --info "$tmp_deb" >/dev/null || die 'Downloaded ros2-apt-source file is not a valid Debian package'
  "${SUDO[@]}" dpkg -i "$tmp_deb"
  rm -f "$tmp_deb"
  trap - EXIT
else
  log 'ros2-apt-source is already installed'
fi

"${SUDO[@]}" apt-get update

packages=(
  ros2-apt-source
  ros-jazzy-ros-base
  ros-dev-tools
  python3-colcon-common-extensions
  python3-rosdep
  python3-vcstool
  python3-serial
  python3-pip
  python3-venv
  python3-yaml
  python3-websockets
  git
  curl
  build-essential
  cmake
  ninja-build
  usbutils
  lsof
  ros-jazzy-navigation2
  ros-jazzy-nav2-bringup
  ros-jazzy-robot-localization
  ros-jazzy-twist-mux
  ros-jazzy-teleop-twist-keyboard
  ros-jazzy-robot-state-publisher
  ros-jazzy-xacro
  ros-jazzy-tf2-tools
  ros-jazzy-diagnostic-updater
  ros-jazzy-diagnostic-aggregator
  ros-jazzy-rviz2
)

missing=()
for package in "${packages[@]}"; do
  apt-cache show "$package" >/dev/null 2>&1 || missing+=("$package")
done
if ((${#missing[@]})); then
  printf '[install_pi] ERROR: required apt packages are unavailable:\n' >&2
  printf '  - %s\n' "${missing[@]}" >&2
  die 'Check network access and the ros2-apt-source installation; no package error was hidden'
fi

log 'Installing ROS 2 Jazzy and robot dependencies'
"${SUDO[@]}" apt-get install -y "${packages[@]}"

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  log 'Initializing rosdep'
  "${SUDO[@]}" rosdep init
else
  log 'rosdep is already initialized'
fi

log "Updating rosdep cache for ${target_user}"
if [[ "$target_user" == root ]]; then
  rosdep update
else
  sudo -H -u "$target_user" rosdep update
fi

target_home="$(getent passwd "$target_user" | cut -d: -f6)"
[[ -n "$target_home" ]] || die "Cannot determine home directory for ${target_user}"
bashrc="${target_home}/.bashrc"
setup_line='source /opt/ros/jazzy/setup.bash'
"${SUDO[@]}" touch "$bashrc"
if ! "${SUDO[@]}" grep -Fqx "$setup_line" "$bashrc"; then
  printf '%s\n' "$setup_line" | "${SUDO[@]}" tee -a "$bashrc" >/dev/null
fi
"${SUDO[@]}" chown "$target_user":"$(id -gn "$target_user")" "$bashrc"

log 'Installation complete. This script did not enable or start robot services.'
log 'Reboot, connect ESP32 over USB, then run setup_udev.sh and build_workspace.sh.'
