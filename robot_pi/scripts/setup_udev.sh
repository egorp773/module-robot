#!/usr/bin/env bash
set -Eeuo pipefail

log() { printf '[setup_udev] %s\n' "$*"; }
die() { printf '[setup_udev] ERROR: %s\n' "$*" >&2; exit 1; }

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
template="${script_dir}/../udev/99-module-robot.rules"
device="${MODULE_ESP32_TTY:-}"
install_values=()

while (($#)); do
  case "$1" in
    --device) [[ $# -ge 2 ]] || die '--device needs a value'; device="$2"; shift 2 ;;
    --install)
      [[ $# -ge 4 ]] || die '--install needs VID PID SERIAL'
      install_values=("$2" "$3" "$4"); shift 4 ;;
    -h|--help)
      cat <<EOF
Usage:
  $0 [--device /dev/ttyACM0]
  sudo $0 --device /dev/ttyACM0 --install VID PID SERIAL

Without --install the script only inspects hardware. The install form renders
the udev template and cross-checks all three IDs against --device (or the only
auto-detected serial candidate).
EOF
      exit 0 ;;
    *) die "Unknown argument: $1" ;;
  esac
done

log 'USB devices (use this to identify the ESP32):'
lsusb || true
printf '\n'

if [[ -z "$device" ]]; then
  mapfile -t candidates < <(find /dev -maxdepth 1 \( -name 'ttyACM*' -o -name 'ttyUSB*' \) -print 2>/dev/null | sort)
  if ((${#candidates[@]})); then
    log "Serial candidates: ${candidates[*]}"
    if ((${#candidates[@]} == 1)); then
      device="${candidates[0]}"
      log "Auto-selected the only serial candidate: $device"
    fi
  else
    log 'No ttyACM/ttyUSB device is connected'
  fi
fi

if [[ -n "$device" && -e "$device" ]]; then
  log "udevadm properties for ${device}:"
  udevadm info --query=property --name="$device" | \
    grep -E '^(DEVNAME|ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_SERIAL)=' || true
  log "Full path: $(udevadm info --query=path --name="$device")"
elif [[ -n "$device" ]]; then
  die "Device does not exist: $device"
fi

if ((${#install_values[@]})); then
  [[ ${EUID} -eq 0 ]] || die 'The --install form must run with sudo'
  [[ -n "$device" && -e "$device" ]] || \
    die '--install requires --device DEVICE (or exactly one auto-detected serial candidate)'
  vid="${install_values[0],,}"
  pid="${install_values[1],,}"
  serial="${install_values[2]}"
  [[ "$vid" =~ ^[0-9a-f]{4}$ ]] || die 'VID must be four hexadecimal characters'
  [[ "$pid" =~ ^[0-9a-f]{4}$ ]] || die 'PID must be four hexadecimal characters'
  [[ "$serial" != TODO_* && -n "$serial" ]] || die 'A real, non-placeholder USB serial is required'
  [[ "$serial" =~ ^[A-Za-z0-9._:-]+$ ]] || die 'USB serial contains unsupported characters'

  properties="$(udevadm info --query=property --name="$device")"
  actual_vid="$(sed -n 's/^ID_VENDOR_ID=//p' <<<"$properties" | head -n1)"
  actual_pid="$(sed -n 's/^ID_MODEL_ID=//p' <<<"$properties" | head -n1)"
  actual_serial="$(sed -n 's/^ID_SERIAL_SHORT=//p' <<<"$properties" | head -n1)"
  [[ "${actual_vid,,}" == "$vid" ]] || die "VID does not match $device (${actual_vid:-missing})"
  [[ "${actual_pid,,}" == "$pid" ]] || die "PID does not match $device (${actual_pid:-missing})"
  [[ "$actual_serial" == "$serial" ]] || die "Serial does not match $device (${actual_serial:-missing})"

  rule_tmp="$(mktemp /etc/udev/rules.d/99-module-robot.rules.XXXXXX)"
  trap 'rm -f -- "${rule_tmp:-}"' EXIT
  sed -e "s/TODO_USB_VENDOR_ID/${vid}/g" \
      -e "s/TODO_USB_PRODUCT_ID/${pid}/g" \
      -e "s/TODO_USB_SERIAL/${serial}/g" \
      "$template" > "$rule_tmp"
  grep -q 'TODO_USB_' "$rule_tmp" && die 'Rendered rule still contains a USB placeholder'
  chmod 0644 "$rule_tmp"
  mv -f "$rule_tmp" /etc/udev/rules.d/99-module-robot.rules
  rule_tmp=''
  trap - EXIT
  udevadm control --reload-rules
  udevadm trigger --subsystem-match=tty
  log 'Rule installed. Replug ESP32, then verify: ls -l /dev/module-esp32'
else
  log 'Inspection only; no udev rule was installed'
  log "After confirming values: sudo $0 --device DEVICE --install VID PID SERIAL"
fi
