# Troubleshooting

Always make the robot mechanically safe first: attachment power off, tracks
lifted or traction power removed. Do not debug a serial or ROS problem while an
old motion command could move the machine.

## `/dev/module-esp32` is missing

```bash
lsusb
dmesg --ctime | tail -n 100
./robot_pi/scripts/setup_udev.sh --device /dev/ttyUSB0
```

Select the actual tty explicitly; the script never guesses among candidates.
If `ID_SERIAL_SHORT` is present, use its recommended `--install-by-serial`
command. Only when it is absent may `--install-by-path` use the reported
`ID_PATH`; that fallback binds the name to one physical Pi USB port (or the full
hub-port topology). If any
VID/PID/serial/path value differs from an installed rule, determine why rather
than weakening the match. The installer refuses VID/PID-only rules and refuses
to overwrite an existing rule. Replug the device after a deliberate rule
change.

## Permission denied opening serial

```bash
ls -l /dev/module-esp32
id
getent group dialout
```

Run `configure_pi.sh --user <name>`, then log out and back in. Do not run the ROS
stack as root to bypass permissions. Check that ModemManager or another program
is not opening the port with `sudo lsof /dev/module-esp32`.

## HELLO never completes

Confirm ESP32 is flashed with `pi_bridge`, baud is 460800 at both ends and no
serial monitor owns the port. Inspect `/esp32/protocol_stats`, `/diagnostics`
and bridge journald logs. A protocol-version mismatch must remain
DISCONNECTED/DISARMED; do not force it to arm.

Do not expect readable ESP32 console text on USB UART0. That channel is
binary-only COBS framing, and mixing text with frames is prohibited. Boot,
reset, state, fault, watchdog, malformed-frame summaries and sensor transitions
arrive as protocol telemetry and are rendered by the bridge. IMU, GNSS and
motor-TX text spam is intentionally absent.

## CRC/malformed counts rise

Check USB cable length/quality, connectors, ground noise and power integrity.
Verify both implementations use little-endian header/payload, CRC16-CCITT over
header+payload, CRC bytes before COBS, and a `0x00` delimiter. Never treat a
bad-CRC packet as a watchdog refresh.

## Reconnect loop

```bash
journalctl -k -f
journalctl -u module-robot-bridge.service -f
udevadm monitor --kernel --udev --property
```

Look for Pi undervoltage, ESP32 brownout, USB resets or another port owner.
Reconnect may restore telemetry only; expected state is DISARMED, latched FAULT
or latched ESTOP as appropriate, never ARMED. A disconnect during motion should
produce `CMD_VEL_TIMEOUT`; HELLO must not silently clear it.

## ARM is rejected

Read `/esp32/status`, `/safety/state`, `/motor/status` and `/diagnostics`.
Normal rejection causes include no HELLO, ESTOP/fault latch, stale motor
feedback, non-zero feedback/target, or operator mode not selected. Reset only
the condition you have physically understood. RESET_FAULT and RESET_ESTOP do
not arm.

## CMD_VEL timeout fault

This is a successful safety response, not a reason to relax the 300 ms limit.
Find why valid 50 Hz commands stopped: bridge scheduling, USB loss, safety
source staleness, parser rejection or process crash. STOP/DISARM, fix the cause,
explicitly reset the fault, ARM again, then send a new sequence.

## IMU rate/yaw is wrong

Do not enable navsat_transform. Check physical mounting, frame axes, timestamps,
calibration/accuracy and covariance. Angular velocity and acceleration can be
validated before absolute yaw. Prove ENU/REP-103 behavior with controlled
manual rotations; never copy the legacy compass offset as a new measurement.

## GNSS fix but no RTK FIXED

`NavSatFix` alone cannot describe carrier solution. Inspect `/rtk/status`, hAcc,
satellite count, correction/RTCM age, antenna view and base corrections. Manual
mode does not require RTK FIXED. Autonomous mode correctly refuses it.

The initial USB protocol does not carry RTCM corrections from Pi to ESP32. A
finite `/rtk/status.rtcm_age_ms` is evidence that F9P received valid RTCM via an
external path; an infinite/stale age is expected if no such path exists. Do not
re-enable the old ESP32 Wi-Fi stack as an implicit workaround.

## Nav2/autonomous launch refuses TODO values

That refusal is intentional. Measure every chassis/sensor transform, replace
`TODO_MEASURE`, set the documented measurement status and rerun preflight. Do
not replace placeholders with guessed zeros. The manual bridge remains usable.

## Systemd restart limit reached

```bash
systemctl status module-robot-bridge.service module-robot-bringup.service
journalctl -u module-robot-bridge.service -u module-robot-bringup.service -b
sudo systemctl reset-failed module-robot-bridge.service module-robot-bringup.service
```

Reset only after fixing the underlying error. Services are deliberately not
enabled by installation; use `uninstall_services.sh` to remove rendered units
without deleting configs or logs.

## Gateway cannot bind WebSocket port 81

Port 81 is retained for legacy Flutter compatibility and is privileged on the
default Ubuntu configuration. Do not run the ROS stack as root. The
commissioning `module-robot-bringup.service` keeps the gateway disabled and is
not granted `CAP_NET_BIND_SERVICE`. For a later manual developer launch on a
controlled bench network, select an unprivileged port above 1024 and configure
the client to match.

Do not expose port 81 through the router or to an untrusted Wi-Fi network. The
legacy compatibility protocol is not a replacement for authentication or a
physical ESTOP.

## A commissioning script reports that a service rejected STOP/ARM

The ROS CLI can exit successfully even when a service response contains
`success=false`. The commissioning scripts inspect that response and stop on a
rejection. Read the returned message plus `/safety/state`, `/esp32/status` and
`/diagnostics`; do not bypass the check or replace it with a direct ESP32 motor
command.

## Collect a support bundle

```bash
cd ~/module
./robot_pi/scripts/first_boot_check.sh | tee first-boot.txt
./robot_pi/scripts/diagnostics.sh | tee diagnostics.txt
git status --short
```

Also record `pio run -e pi_bridge` build ID, ESP32 reset reason, wiring changes,
and the exact physical event. Remove passwords, tokens and precise private GNSS
coordinates before sharing logs.
