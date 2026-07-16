#!/usr/bin/env bash
set -Eeuo pipefail

log() { printf '[setup_udev] %s\n' "$*"; }
die() { printf '[setup_udev] ERROR: %s\n' "$*" >&2; exit 1; }

usage() {
  cat <<EOF
Usage:
  $0 --device /dev/ttyUSB0
  sudo $0 --device /dev/ttyUSB0 --install-by-serial VID PID SERIAL
  sudo $0 --device /dev/ttyUSB0 --install-by-path VID PID ID_PATH

The inspection form prints the connected device properties and recommends the
safe identity to use. A USB serial number is preferred. ID_PATH is accepted
only when ID_SERIAL_SHORT is absent and binds the ESP32 to that physical USB
port, or to the full hub-port topology. VID/PID alone are never sufficient.

The device must always be selected explicitly. This script never guesses which
tty is the ESP32, even if only one candidate is currently connected.
EOF
}

device=''
install_mode='inspect'
install_requested=false
install_values=()

while (($#)); do
  case "$1" in
    --device)
      [[ $# -ge 2 && -n "$2" ]] || die '--device needs a value'
      [[ -z "$device" ]] || die '--device may be specified only once'
      device="$2"
      shift 2
      ;;
    --install-by-serial)
      [[ "$install_requested" == false ]] || die 'Choose exactly one install method'
      [[ $# -ge 4 ]] || die '--install-by-serial needs VID PID SERIAL'
      install_requested=true
      install_mode='serial'
      install_values=("$2" "$3" "$4")
      shift 4
      ;;
    --install-by-path)
      [[ "$install_requested" == false ]] || die 'Choose exactly one install method'
      [[ $# -ge 4 ]] || die '--install-by-path needs VID PID ID_PATH'
      install_requested=true
      install_mode='path'
      install_values=("$2" "$3" "$4")
      shift 4
      ;;
    --install)
      die '--install was replaced by --install-by-serial or --install-by-path'
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

command -v udevadm >/dev/null 2>&1 || die 'udevadm is required'

log 'Connected USB devices:'
if command -v lsusb >/dev/null 2>&1; then
  lsusb || true
else
  log 'lsusb is unavailable; udev properties can still be inspected'
fi
printf '\n'

mapfile -t candidates < <(
  find /dev -maxdepth 1 \( -name 'ttyACM*' -o -name 'ttyUSB*' \) -print 2>/dev/null | sort
)
if ((${#candidates[@]})); then
  log "Serial candidates: ${candidates[*]}"
else
  log 'No /dev/ttyACM* or /dev/ttyUSB* candidates are currently connected'
fi

[[ -n "$device" ]] || {
  usage >&2
  die 'Select the ESP32 explicitly with --device DEVICE; no candidate was auto-selected'
}
[[ "$device" == /dev/* ]] || die '--device must be an absolute path below /dev'
[[ -e "$device" ]] || die "Device does not exist: $device"

resolved_device="$(readlink -f -- "$device")" || die "Cannot resolve device: $device"
[[ -n "$resolved_device" && -e "$resolved_device" ]] || die "Cannot resolve device: $device"

properties="$(udevadm info --query=property --name="$resolved_device")" || \
  die "Cannot read udev properties for $resolved_device"
sysfs_path="$(udevadm info --query=path --name="$resolved_device")" || \
  die "Cannot read the sysfs path for $resolved_device"

property_from() {
  local key="$1" source="$2"
  awk -F= -v key="$key" '
    $1 == key {
      sub(/^[^=]*=/, "")
      print
      exit
    }
  ' <<<"$source"
}

property_value() {
  property_from "$1" "$properties"
}

actual_vid="$(property_value ID_VENDOR_ID)"
actual_pid="$(property_value ID_MODEL_ID)"
actual_serial="$(property_value ID_SERIAL_SHORT)"
actual_path="$(property_value ID_PATH)"
actual_subsystem="$(property_value SUBSYSTEM)"

[[ "$actual_subsystem" == tty ]] || \
  die "$resolved_device is not a tty device (SUBSYSTEM=${actual_subsystem:-missing})"

log "Selected device: $device -> $resolved_device"
log 'Relevant udev properties:'
printf '%s\n' "$properties" | \
  grep -E '^(DEVNAME|SUBSYSTEM|ID_BUS|ID_VENDOR_ID|ID_MODEL_ID|ID_VENDOR|ID_MODEL|ID_SERIAL_SHORT|ID_SERIAL|ID_PATH|ID_PATH_TAG)=' || true
log "Sysfs path: $sysfs_path"

if [[ -z "$actual_vid" || -z "$actual_pid" ]]; then
  preferred_mode='none'
  log 'ID_VENDOR_ID or ID_MODEL_ID is absent; a safe persistent rule cannot be installed.'
elif [[ -n "$actual_serial" ]]; then
  preferred_mode='serial'
  log 'Preferred identity: USB serial number (stable if the ESP32 is moved to another Pi USB port).'
  printf '[setup_udev] Recommended command:\n  sudo %q --device %q --install-by-serial %q %q %q\n' \
    "$0" "$device" "$actual_vid" "$actual_pid" "$actual_serial"
elif [[ -n "$actual_path" ]]; then
  preferred_mode='path'
  log 'ID_SERIAL_SHORT is absent; safe fallback: ID_PATH.'
  log 'ID_PATH binds /dev/module-esp32 to this physical Pi USB port (or full hub-port topology).'
  printf '[setup_udev] Recommended command:\n  sudo %q --device %q --install-by-path %q %q %q\n' \
    "$0" "$device" "$actual_vid" "$actual_pid" "$actual_path"
else
  preferred_mode='none'
  log 'No ID_SERIAL_SHORT or ID_PATH is available; a safe persistent rule cannot be installed.'
fi

if [[ "$install_requested" == false ]]; then
  log 'Inspection only; no udev rule was installed'
  exit 0
fi

[[ ${EUID} -eq 0 ]] || die 'Installation must run with sudo'
[[ -n "$actual_vid" ]] || die "$resolved_device has no ID_VENDOR_ID"
[[ -n "$actual_pid" ]] || die "$resolved_device has no ID_MODEL_ID"

vid="${install_values[0],,}"
pid="${install_values[1],,}"
identity="${install_values[2]}"

[[ "$vid" =~ ^[0-9a-f]{4}$ ]] || die 'VID must be exactly four hexadecimal characters'
[[ "$pid" =~ ^[0-9a-f]{4}$ ]] || die 'PID must be exactly four hexadecimal characters'
[[ "${actual_vid,,}" == "$vid" ]] || \
  die "VID does not match $resolved_device: supplied '$vid', connected '${actual_vid:-missing}'"
[[ "${actual_pid,,}" == "$pid" ]] || \
  die "PID does not match $resolved_device: supplied '$pid', connected '${actual_pid:-missing}'"

# These conservative character sets prevent quotes, backslashes, newlines, or
# udev syntax from being injected into the generated rule.
safe_serial_regex='^[A-Za-z0-9._:+/@-]+$'
safe_path_regex='^[A-Za-z0-9._:+/@-]+$'

case "$install_mode" in
  serial)
    [[ "$preferred_mode" == serial ]] || \
      die '--install-by-serial requires a connected device with ID_SERIAL_SHORT'
    [[ -n "$identity" && "$identity" != TODO_* ]] || die 'A real USB serial number is required'
    [[ "$identity" =~ $safe_serial_regex ]] || \
      die 'USB serial contains characters that cannot be rendered safely'
    [[ "$identity" == "$actual_serial" ]] || \
      die "Serial does not match $resolved_device: supplied '$identity', connected '$actual_serial'"
    # Match the same normalized udev property that was inspected and
    # cross-checked above, rather than assuming raw sysfs ATTRS{serial} is
    # byte-for-byte identical to ID_SERIAL_SHORT.
    match_clause="ENV{ID_SERIAL_SHORT}==\"${identity}\""
    ;;
  path)
    [[ -z "$actual_serial" ]] || \
      die "ID_SERIAL_SHORT is present ('$actual_serial'); use the preferred --install-by-serial method"
    [[ "$preferred_mode" == path ]] || \
      die '--install-by-path requires a connected device with ID_PATH and no ID_SERIAL_SHORT'
    [[ -n "$identity" && "$identity" != TODO_* ]] || die 'A real ID_PATH is required'
    [[ "$identity" =~ $safe_path_regex ]] || \
      die 'ID_PATH contains characters that cannot be rendered safely'
    [[ "$identity" == "$actual_path" ]] || \
      die "ID_PATH does not match $resolved_device: supplied '$identity', connected '$actual_path'"
    match_clause="ENV{ID_PATH}==\"${identity}\""
    ;;
  *)
    die "Internal error: unsupported install mode '$install_mode'"
    ;;
esac

# Fail closed if the proposed identity currently selects more than one tty.
# Explicit --device prevents guessing, while this check prevents two devices
# or two interfaces from racing for the same persistent symlink.
matching_candidates=()
for candidate in "${candidates[@]}"; do
  candidate_properties="$(udevadm info --query=property --name="$candidate")" || \
    die "Cannot cross-check connected tty candidate: $candidate"
  candidate_vid="$(property_from ID_VENDOR_ID "$candidate_properties")"
  candidate_pid="$(property_from ID_MODEL_ID "$candidate_properties")"
  [[ "${candidate_vid,,}" == "$vid" && "${candidate_pid,,}" == "$pid" ]] || continue

  if [[ "$install_mode" == serial ]]; then
    candidate_identity="$(property_from ID_SERIAL_SHORT "$candidate_properties")"
  else
    candidate_identity="$(property_from ID_PATH "$candidate_properties")"
  fi
  [[ "$candidate_identity" == "$identity" ]] && matching_candidates+=("$candidate")
done

if ((${#matching_candidates[@]} != 1)); then
  die "Proposed ${install_mode} identity matches ${#matching_candidates[@]} tty candidates (${matching_candidates[*]:-none}); require one unique device"
fi
matched_resolved="$(readlink -f -- "${matching_candidates[0]}")" || \
  die "Cannot resolve matched tty candidate: ${matching_candidates[0]}"
[[ "$matched_resolved" == "$resolved_device" ]] || \
  die "Proposed identity selects ${matching_candidates[0]}, not explicit device $device"

target='/etc/udev/rules.d/99-module-robot.rules'
# Refuse both a same-name override and any installed rule already claiming the
# persistent symlink. Search all normal udev rule locations and deduplicate the
# /lib -> /usr/lib alias by canonical path.
rule_dirs=(
  /etc/udev/rules.d
  /run/udev/rules.d
  /usr/local/lib/udev/rules.d
  /usr/lib/udev/rules.d
  /lib/udev/rules.d
)
conflicting_rules=()
declare -A seen_rules=()
shopt -s nullglob
for rule_dir in "${rule_dirs[@]}"; do
  [[ -d "$rule_dir" ]] || continue
  for rule in "$rule_dir"/*.rules; do
    canonical_rule="$(readlink -f -- "$rule")" || die "Cannot inspect udev rule: $rule"
    [[ -z "${seen_rules[$canonical_rule]:-}" ]] || continue
    seen_rules[$canonical_rule]=1
    if [[ "${rule##*/}" == 99-module-robot.rules ]] || grep -Fq 'module-esp32' "$rule"; then
      conflicting_rules+=("$rule")
    fi
  done
done
shopt -u nullglob

if ((${#conflicting_rules[@]})); then
  printf '[setup_udev] ERROR: existing/conflicting udev rules were not replaced:\n' >&2
  printf '  - %s\n' "${conflicting_rules[@]}" >&2
  die 'Inspect and remove or rename the intended old rule explicitly before retrying'
fi

umask 022
rule_tmp="$(mktemp /etc/udev/rules.d/.99-module-robot.rules.XXXXXX)"
cleanup() { rm -f -- "${rule_tmp:-}"; }
trap cleanup EXIT

{
  printf '# Generated by module-robot setup_udev.sh; do not edit identity values blindly.\n'
  if [[ "$install_mode" == path ]]; then
    printf '# ID_PATH fallback: changing the direct port or hub-port topology invalidates this match.\n'
  fi
  printf 'SUBSYSTEM=="tty", ATTRS{idVendor}=="%s", ATTRS{idProduct}=="%s", %s, SYMLINK+="module-esp32", GROUP="dialout", MODE="0660", TAG+="systemd"\n' \
    "$vid" "$pid" "$match_clause"
} >"$rule_tmp"
chmod 0644 "$rule_tmp"

# A hard link installs the completed file atomically and fails rather than
# replacing a target created between the explicit check above and this point.
if ! ln -- "$rule_tmp" "$target"; then
  die "Rule was not installed because $target now exists or cannot be created; nothing was replaced"
fi
rm -f -- "$rule_tmp"
rule_tmp=''
trap - EXIT

if ! udevadm control --reload-rules; then
  die "Rule is installed at $target, but udev reload failed; inspect it and remove it explicitly before retrying"
fi
log "Installed $target using $install_mode identity; udev rules were reloaded."
log 'Unplug and reconnect the ESP32. Replug is required before verification.'
log 'Then verify: ls -l /dev/module-esp32 && udevadm info --query=property --name=/dev/module-esp32'
