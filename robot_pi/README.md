# Module Robot: Raspberry Pi control architecture

This directory is the staged replacement for the tightly coupled rover control
stack. It targets Raspberry Pi 4 (8 GB), Ubuntu Server 24.04 ARM64 and ROS 2
Jazzy. It is intentionally **manual-first** and is not field-ready until the
acceptance gates have been passed on the physical robot.

```text
Flutter / laptop
        |
        v
WebSocket gateway --> ROS 2 safety mux --> /cmd_vel_safe
                                             |
                                             v
                                  ESP32 serial bridge
                                      COBS + CRC16
                                             |
                                             v
ESP32 hardware controller --> hoverboard UART --> tracks
```

## Non-negotiable safety invariants

- Raspberry Pi serial `CMD_VEL` is the only source of non-zero ESP32 motion.
- ESP32 boots at hard zero, relays off and never arms automatically.
- A successful first `HELLO` reaches `DISARMED`, never ARMED. Reconnect reaches
  DISARMED only when no latch exists; a prior FAULT or ESTOP remains latched.
- Commands are sent at 50 Hz. A valid-command gap greater than 300 ms causes an
  immediate zero UART frame, cleared ramp state and latched
  `CMD_VEL_TIMEOUT` fault.
- CRC failures, duplicates, out-of-order packets and heartbeats never refresh
  the motor watchdog.
- STOP, DISARM, ESTOP and timeout bypass the acceleration ramp.
- ESTOP is separately latched; reset returns only to DISARMED.
- The ROS safety node selects exactly one of manual, Nav2 or zero. It never adds
  velocity sources.
- Neither the serial bridge nor safety node performs automatic ARM.
- Autonomous launch remains disabled while any `TODO_MEASURE` is present.

The legacy `rtk_firmware` environment `rover` remains available and unchanged
in purpose. The invariants above apply to `pi_bridge`, not retroactively to the
legacy image. Treat `rover` as a build/rollback artifact, never as proof of the
new STOP case; select the new firmware explicitly with `pio run -e pi_bridge`.

## Packages

| Package | Responsibility |
| --- | --- |
| `module_robot_msgs` | Robot-specific status, fault, relay and service interfaces |
| `module_robot_description` | Frame/Xacro skeleton without invented dimensions |
| `module_robot_esp32_bridge` | Binary serial transport, handshake, telemetry and explicit hardware services |
| `module_robot_safety` | Manual/auto command arbitration and safety gates |
| `module_robot_localization` | Disabled-by-default EKF/navsat templates and heading initializer skeleton |
| `module_robot_navigation` | Mapless FollowPath/RPP and velocity smoother templates |
| `module_robot_gateway` | Legacy Flutter WebSocket compatibility without direct motor access |
| `module_robot_bringup` | Manual-safe default launch and separate autonomous preflight |
| `module_robot_tools` | Commissioning and diagnostic helpers |

## ROS interfaces

Command flow:

```text
/cmd_vel_manual ----\
                     > module_robot_safety --> /cmd_vel_safe --> serial bridge
/cmd_vel_nav -------/
```

The bridge publishes `/imu/data_raw`, `/gps/fix`, `/rtk/status`,
`/motor/status`, `/power/status`, `/relay/status`, `/esp32/status`,
`/esp32/protocol_stats` and `/diagnostics`. Hardware control is exposed only as
explicit `/esp32/arm`, `/esp32/disarm`, `/esp32/stop`, `/esp32/estop`,
`/esp32/reset_fault`, `/esp32/reset_estop` and `/esp32/set_relays` services.
Operator-facing safety services live under `/safety/*`.

USB Serial/UART0 is binary-only: every byte belongs to the versioned
COBS/CRC16 protocol. Arbitrary boot banners, `printf` console text and console
commands are forbidden because they would corrupt framing. Boot/state/fault,
watchdog, malformed-frame and sensor-availability transitions are represented
by `STATUS`, `FAULT_EVENT`, `ESTOP_EVENT` and `DIAGNOSTICS`, then logged by the
Pi bridge through ROS diagnostics/journald. There is no per-IMU, per-GNSS or
per-`MOTOR_TX` text stream.

The gateway accepts the existing `M`, `STOP`, `ROUTE_*` and `NAV_*` messages,
but `NAV_START` returns `AUTO_NOT_READY` at this stage. Route upload is stored
and validated only; it cannot bypass `/cmd_vel_safe`.

The compatibility WebSocket is not an Internet security boundary. Use it only
on a controlled robot LAN, block inbound WAN access, and disarm when no operator
is actively driving. Once manual ARM has been granted, an untrusted client able
to reach the legacy endpoint must be treated as a motion-command risk.

## First use

Follow [FIRST_BOOT.md](FIRST_BOOT.md), then execute the gates in
[ACCEPTANCE_TESTS.md](ACCEPTANCE_TESTS.md) in order. Gates 1 and 2 are
non-motion checks. The first lifted, low-speed manual pulse is then used to
prove the command path before the Gate 3 watchdog/fault injections. The robot
must not be placed on the ground, and autonomy must remain inhibited, until
manual control and every required STOP case have both been proved.

Useful entrypoints on the Pi:

```bash
chmod +x robot_pi/scripts/*.sh robot_pi/scripts/*.py
./robot_pi/scripts/install_pi.sh
sudo ./robot_pi/scripts/setup_udev.sh --device /dev/ttyACM0
./robot_pi/scripts/build_workspace.sh
./robot_pi/scripts/first_boot_check.sh
./robot_pi/scripts/diagnostics.sh
```

Systemd templates are rendered by `configure_pi.sh`, but are deliberately not
enabled or started. No Docker, micro-ROS or `ros2_control` is used in stage 1.

Flash the dedicated ESP32 image explicitly from a PlatformIO workstation:

```powershell
cd C:\robot\module\rtk_firmware
pio run -e pi_bridge -t upload --upload-port COMx
```

Never use this command with guessed ports or with traction/attachment power
connected. The legacy `rover` image remains a separate, deliberate selection.

## Configuration policy

`config/serial.yaml` and safety speed clamps contain commissioning defaults.
Dimensions, signs, scales, deadband and footprint are not facts until measured;
they remain `TODO_MEASURE` or `TODO_CALIBRATE`. Record the measurement method
and date when replacing a placeholder. Never change a TODO merely to make
autonomous preflight pass.

`config/velocity_limits.yaml` is a calibration record/template, not an
automatic `SET_LIMITS` command. The bridge deliberately does not push it at
startup or reconnect. First lifted movement remains inside the conservative
compiled ESP32 hard clamps; track signs, width, scale and deadband still require
Gate 4 measurement. Applying calibrated limits later must be an explicit,
reviewed operation (or reviewed firmware update), never reconnect behavior.
