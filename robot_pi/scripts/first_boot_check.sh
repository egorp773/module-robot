#!/usr/bin/env bash
set -uo pipefail

failures=0
warnings=0
ok() { printf '[ OK ] %s\n' "$*"; }
warn() { printf '[WARN] %s\n' "$*"; warnings=$((warnings + 1)); }
fail() { printf '[FAIL] %s\n' "$*"; failures=$((failures + 1)); }

if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release
  if [[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 24.04 && "${VERSION_CODENAME:-}" == noble ]]; then
    ok 'Ubuntu 24.04 Noble'
  else
    fail "Expected Ubuntu 24.04 Noble; detected ${PRETTY_NAME:-unknown}"
  fi
else
  fail '/etc/os-release is unavailable'
fi

case "$(uname -m)" in aarch64|arm64) ok "ARM64 kernel ($(uname -m))" ;; *) fail "Expected ARM64; got $(uname -m)" ;; esac
if command -v dpkg >/dev/null 2>&1; then
  [[ "$(dpkg --print-architecture)" == arm64 ]] && ok 'ARM64 userspace (dpkg)' || \
    fail "Expected dpkg architecture arm64; got $(dpkg --print-architecture)"
else
  fail 'dpkg is unavailable; this is not a supported Ubuntu installation'
fi

if [[ -r /proc/device-tree/model ]]; then
  pi_model="$(tr -d '\0' < /proc/device-tree/model)"
  [[ "$pi_model" == *'Raspberry Pi 4 Model B'* ]] && ok "$pi_model" || \
    warn "Target is Raspberry Pi 4 Model B; detected ${pi_model:-unknown model}"
else
  warn 'Raspberry Pi model information is unavailable'
fi

mem_total_kb="$(awk '/^MemTotal:/{print $2; exit}' /proc/meminfo 2>/dev/null || true)"
if [[ "$mem_total_kb" =~ ^[0-9]+$ ]] && ((mem_total_kb >= 7 * 1024 * 1024)); then
  ok "8 GB-class RAM detected ($((mem_total_kb / 1024)) MiB usable)"
else
  warn "Target is the 8 GB Pi; usable RAM is ${mem_total_kb:-unknown} KiB"
fi

if [[ -r /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  [[ "${ROS_DISTRO:-}" == jazzy ]] && ok 'ROS 2 Jazzy' || fail 'ROS setup did not select Jazzy'
else
  fail '/opt/ros/jazzy/setup.bash is missing'
fi

free_kb="$(df -Pk / | awk 'NR==2 {print $4}')"
if [[ "$free_kb" =~ ^[0-9]+$ ]] && ((free_kb >= 5 * 1024 * 1024)); then
  ok "Root filesystem free: $((free_kb / 1024 / 1024)) GiB"
else
  warn "Less than 5 GiB free on root filesystem (${free_kb:-unknown} KiB)"
fi

if [[ -r /sys/class/thermal/thermal_zone0/temp ]]; then
  temp_milli="$(< /sys/class/thermal/thermal_zone0/temp)"
  temp_c="$(awk -v t="$temp_milli" 'BEGIN {printf "%.1f", t/1000}')"
  awk -v t="$temp_milli" 'BEGIN {exit !(t < 80000)}' && ok "Pi temperature: ${temp_c} C" || warn "High Pi temperature: ${temp_c} C"
else
  warn 'Pi temperature sensor is unavailable'
fi

if command -v vcgencmd >/dev/null 2>&1; then
  throttled="$(vcgencmd get_throttled 2>/dev/null || true)"
  [[ "$throttled" == 'throttled=0x0' ]] && ok 'No Pi undervoltage/throttling flags' || warn "Power/throttling status: ${throttled:-unavailable}"
else
  kernel_power_events="$(journalctl -k -b --no-pager 2>/dev/null | \
    grep -Ei 'under.?voltage|brownout|throttl' | tail -n 5 || true)"
  if [[ -n "$kernel_power_events" ]]; then
    warn "Kernel reported possible power/throttling events: ${kernel_power_events//$'\n'/; }"
  else
    warn 'vcgencmd unavailable; no current kernel power warning found, but historical throttle flags could not be read'
  fi
fi

if command -v lsusb >/dev/null 2>&1 && lsusb | grep -Eqi 'Espressif|CP210|CH340|USB Serial|UART'; then
  ok 'A likely ESP32/USB-UART device is visible'
else
  warn 'No recognizable ESP32/USB-UART entry found in lsusb'
fi

if [[ -e /dev/module-esp32 ]]; then
  ok "/dev/module-esp32 -> $(readlink -f /dev/module-esp32)"
  [[ -r /dev/module-esp32 && -w /dev/module-esp32 ]] && ok 'Current user has serial read/write permission' || fail "No read/write permission for $(id -un); check dialout membership and re-login"
else
  fail '/dev/module-esp32 is missing; run setup_udev.sh and replug ESP32'
fi

id -nG | tr ' ' '\n' | grep -qx dialout && ok 'Current user is in dialout' || fail 'Current user is not in dialout'

if ip route show default 2>/dev/null | grep -q '^default'; then ok 'Default network route exists'; else warn 'No default network route'; fi
if getent hosts packages.ros.org >/dev/null 2>&1; then ok 'DNS/network reaches packages.ros.org'; else warn 'Cannot resolve packages.ros.org'; fi

if timedatectl show -p NTPSynchronized --value 2>/dev/null | grep -qx yes; then
  ok 'System clock is NTP synchronized'
else
  warn 'System clock is not reported as synchronized'
fi
printf '[INFO] system time: %s\n' "$(date --iso-8601=seconds)"

# Commissioning units may be installed for a supervised STOP test, but the
# repository must not silently arrange an automatic boot start.
for unit in module-robot-bridge.service module-robot-bringup.service; do
  unit_enablement="$(systemctl is-enabled "$unit" 2>/dev/null || true)"
  case "$unit_enablement" in
    enabled|enabled-runtime|linked|linked-runtime|alias)
      fail "$unit is ${unit_enablement}; disable it until the manual and STOP gates pass"
      ;;
    disabled)
      ok "$unit is installed and disabled"
      ;;
    not-found|'')
      ok "$unit is not installed/enabled yet"
      ;;
    masked|masked-runtime)
      warn "$unit is ${unit_enablement}; it cannot be started for commissioning until unmasked"
      ;;
    *)
      warn "$unit enablement state is ${unit_enablement}; review it before motion"
      ;;
  esac
done

printf '\nSummary: %d failure(s), %d warning(s).\n' "$failures" "$warnings"
((failures == 0))
