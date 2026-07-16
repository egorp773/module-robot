# First boot on Raspberry Pi 4

These steps assume little Linux experience. Commands are entered one line at a
time. Commissioning must use the reviewed `raspberry-pi-migration` branch until
the migration is explicitly merged to the default branch.

## 1. Write Ubuntu Server

1. Install Raspberry Pi Imager on a laptop.
2. Select Raspberry Pi 4, **Ubuntu Server 24.04 LTS (64-bit)** and the SD card.
3. In Imager settings set hostname `module-robot`, username `egor`, a strong
   password, locale/timezone and Wi-Fi if required.
4. Enable SSH with password or, preferably, an SSH public key.
5. Write and verify the card. Insert it into the powered-off Pi.

Do not connect motors, relays or the ESP32 yet. Power the Pi from the checked
5.1 V DC-DC converter through USB-C.

## 2. Find and connect to the Pi

Wait two minutes. In PowerShell on the laptop:

```powershell
ping module-robot.local
ssh egor@module-robot.local
```

If `.local` does not resolve, find the address in the router's client list and
use `ssh egor@192.168.x.y`. The first SSH connection asks whether to trust the
host key; verify the displayed address, type `yes`, then enter the password.

## 3. Clone the repository

```bash
sudo apt-get update
sudo apt-get install -y git
git clone --branch raspberry-pi-migration --single-branch \
  https://github.com/egorp773/module-robot.git ~/module
cd ~/module
git branch --show-current
git rev-parse --short HEAD
chmod +x robot_pi/scripts/*.sh robot_pi/scripts/*.py
```

The branch verification command must print `raspberry-pi-migration`. Record that
branch and the short commit ID in the commissioning log. Stop if another branch
is shown; do not use a normal default-branch clone while this patch is not in
`main`.
Never paste repository credentials into a tracked configuration file.

## 4. Install Ubuntu and ROS dependencies

```bash
cd ~/module
./robot_pi/scripts/install_pi.sh --stage manual
sudo reboot
```

Reconnect with SSH after about two minutes. The installer refuses non-Noble or
non-ARM64 systems, prints `df -h /`, requires at least 3 GiB free for the manual
stage, checks every required apt package and does not start robot services. The
default is also `manual`, but spelling out the stage makes the commissioning
record unambiguous.

The manual stage installs only the ROS base/build/test and runtime dependencies
for messages, ESP32 bridge, safety, gateway package, manual bringup, tools and
diagnostics. It intentionally excludes Navigation2, robot_localization and
RViz. Much later, after the manual gates and before Gate 9 localization or
autonomy work, the full dependency stage may be installed explicitly; it
requires at least 8 GiB free:

```bash
./robot_pi/scripts/install_pi.sh --stage full
```

## 5. Flash the dedicated ESP32 firmware

Return to PowerShell on the development laptop (these are not SSH commands).
PlatformIO must already be installed there. Keep traction and attachment power
disconnected, find the actual ESP32 COM port first, and never upload to an
unrelated serial device.

```powershell
cd C:\robot\module\rtk_firmware
pio device list
pio run -e pi_bridge
pio run -e pi_bridge -t upload --upload-port COMx
```

Replace `COMx` with the observed port. Record the repository revision and build
identifier. Do not select `rover`: that is the preserved legacy image, not the
Pi hardware-controller image. If `pi_bridge` is already flashed with the exact
verified build, record that fact instead of uploading it again.

Return to the SSH session for the remaining Raspberry Pi steps.

## 6. Connect ESP32 USB and create the stable device name

With motor and attachment power still disconnected, connect:

```text
Raspberry Pi USB-A -> ESP32 USB
```

List tty candidates, select the ESP32 deliberately, and inspect it. The script
never chooses among connected tty devices:

```bash
cd ~/module
./robot_pi/scripts/setup_udev.sh --device /dev/ttyUSB0
```

Use the actual path shown on your Pi; it may instead be `/dev/ttyACM0`. The
inspection prints a recommended command. When `ID_SERIAL_SHORT` exists, retain
the preferred VID+PID+serial identity:

```bash
sudo ./robot_pi/scripts/setup_udev.sh --device /dev/ttyUSB0 \
  --install-by-serial VID PID SERIAL
```

Only when `ID_SERIAL_SHORT` is absent, use the printed `ID_PATH` fallback:

```bash
sudo ./robot_pi/scripts/setup_udev.sh --device /dev/ttyUSB0 \
  --install-by-path VID PID ID_PATH
```

`ID_PATH` binds `/dev/module-esp32` to that physical Pi USB port, or to the full
hub-port topology when a hub is present. Moving the cable intentionally breaks the match. Both
installation modes cross-check every supplied value against the selected
device; VID+PID alone is rejected. The script refuses to replace an existing
rule without an explicit operator decision. After it reloads the rules,
unplug/replug USB as requested and verify:

```bash
ls -l /dev/module-esp32
sudo ./robot_pi/scripts/configure_pi.sh --user egor --repo-root ~/module
```

Log out and back in (or reboot) so `dialout` membership applies.

Before any ROS workspace build, reconnect and complete the power gate:

```bash
cd ~/module
./robot_pi/scripts/first_boot_check.sh
```

Resolve every `[FAIL]`. Warnings about power, time or USB stability also need a
decision. Record the power-cycle and undervoltage evidence required by Gate 1,
then proceed to the ROS build and tests.

## 7. Build the ROS workspace

```bash
cd ~/module
./robot_pi/scripts/build_workspace.sh --stage manual
source ~/module/robot_pi/ros2_ws/install/setup.bash
```

The manual build uses an exact standard colcon package selection:
`module_robot_msgs`, `module_robot_esp32_bridge`, `module_robot_safety`,
`module_robot_gateway`, `module_robot_bringup` and `module_robot_tools`.
`rosdep` resolves only this manual set and deliberately skips the optional
description plus localization/navigation branches referenced by the guarded
bringup manifest.
The script then runs `colcon test` and `colcon test-result --verbose`; any failed
test makes the command fail. Do not try to build this Jazzy workspace on
Windows. A later full build is explicit:

```bash
./robot_pi/scripts/build_workspace.sh --stage full
```

## 8. Run non-motion checks

The successful Pi build and test result from step 7 is Gate 2.

Before starting the ROS bridge, verify one safe binary-protocol round trip:

```bash
python3 ~/module/robot_pi/scripts/serial_loopback_test.py
```

It sends HELLO and REQUEST_STATUS, requires DISARMED/hard-zero, prints decoder
statistics, then attempts STOP and DISARM as best-effort cleanup. It does not
prove acknowledgements, reconnect or relay behavior; those belong to later
gates. Close every serial monitor first. This is Gate 3.

Terminal 1 — serial bridge only:

```bash
source /opt/ros/jazzy/setup.bash
source ~/module/robot_pi/ros2_ws/install/setup.bash
ros2 launch module_robot_esp32_bridge bridge.launch.py \
  config:=$HOME/module/robot_pi/config/serial.yaml
```

Terminal 2 — inspect state:

```bash
source /opt/ros/jazzy/setup.bash
source ~/module/robot_pi/ros2_ws/install/setup.bash
ros2 topic echo /esp32/status
```

Expected after HELLO is `connected: true`, `state: 2` (DISARMED), zero applied
commands and relays off. The bridge must never ARM itself.

Collect diagnostics:

```bash
cd ~/module
./robot_pi/scripts/diagnostics.sh | tee first-boot-diagnostics.txt
```

## 9. Start the manual-safe stack

Stop the standalone bridge with `Ctrl+C`, then:

```bash
source /opt/ros/jazzy/setup.bash
source ~/module/robot_pi/ros2_ws/install/setup.bash
ros2 launch module_robot_bringup manual_bringup.launch.py \
  bridge_config:=$HOME/module/robot_pi/config/serial.yaml \
  safety_config:=$HOME/module/robot_pi/config/safety.yaml \
  start_gateway:=false
```

Do not launch the autonomous bringup. Do not enable systemd yet. The first
manual test is operated from the laptop over SSH using only ROS topics and
services. Do not connect the phone and do not start the WebSocket gateway.

## 10. Run the no-motion pre-ARM probe

Keep traction physically unable to move the robot. With the manual-safe stack
running, open another laptop SSH terminal and run:

```bash
cd ~/module
./robot_pi/scripts/pre_arm_hardware_probe.sh
```

The probe forces DISARM, never calls ARM or publishes CMD_VEL, watches hardware
telemetry for at least ten seconds, checks hard-zero/status/error counters and
writes a private timestamped text report under
`~/.local/state/module-robot/` (or `$MODULE_ROBOT_REPORT_DIR`). Resolve every
failure; this is Gate 4.
Next prove STOP and DISARM while the robot has never received a non-zero command:

```bash
for attempt in 1 2 3; do
  ros2 service call /safety/stop std_srvs/srv/Trigger '{}'
done
ros2 service call /safety/disarm module_robot_msgs/srv/Disarm '{}'
ros2 topic echo --once /esp32/status
./robot_pi/scripts/pre_arm_hardware_probe.sh
```

Do not proceed unless the status remains connected, DISARMED and zero on both
applied commands and UART speed/steer, every STOP response succeeds, and the
second probe report again proves stable counters and relays off. This zero-only
exercise is Gate 5.

## 11. First lifted movement and STOP proof

Read [ACCEPTANCE_TESTS.md](ACCEPTANCE_TESTS.md). Complete Gates 1 and 2 before
the protocol loopback, then complete the pre-ARM and no-motion STOP gates. Only
after Gates 1 through 5 pass may you move the robot to a controlled area, lift
it on secure blocks, remove attachment power and keep a physical disconnect
ready. The tracks must not touch the ground. In a separate terminal run:

```bash
cd ~/module
./robot_pi/scripts/manual_drive_test.sh
```

The script first forces and verifies DISARMED, prints status, requires an exact
operator confirmation, arms, sends zero, requests only 0.03 m/s for one second,
hard-stops, checks feedback and disarms. It never runs at boot.

After that first manual pulse succeeds, stop the manual launch with `Ctrl+C`.
Render the systemd units so `stop_test.sh` can kill and observe a supervised
bridge service. Rendering and manually starting a unit do not enable it at
boot:

```bash
sudo ./robot_pi/scripts/configure_pi.sh --user egor --repo-root ~/module --install-systemd
systemctl is-enabled module-robot-bridge.service module-robot-bringup.service
sudo systemctl start module-robot-bridge.service module-robot-bringup.service
systemctl status module-robot-bridge.service module-robot-bringup.service --no-pager
./robot_pi/scripts/stop_test.sh
sudo systemctl stop module-robot-bringup.service module-robot-bridge.service
```

The expected `is-enabled` result is `disabled`. The bringup unit explicitly
keeps `start_gateway:=false`. `stop_test.sh` deliberately
commands a very low speed, kills the bridge, and asks for a USB disconnect; its
exact mechanical confirmation is mandatory. It does not reboot the Pi. Perform
the separate Pi reboot/power-loss item in Gate 7 only under the same lifted,
supervised conditions and record physical zero.

Do not put the robot on the ground until all Gate 7 STOP cases pass. Do not set
`autonomous_enabled: true` or enable autonomous services until manual control,
watchdog STOP, measurements, sensors and localization have passed their gates.
Systemd enablement is a later, explicit operator decision; none of these scripts
enables a service.

## 12. Gateway remains a later manual task

The commissioning systemd unit does not start the WebSocket gateway. It listens
on `0.0.0.0:81` without authentication, so even a manual gateway launch is
deferred until the ROS-only laptop tests are complete and the network is
controlled. Before field use from the phone, create and review a separate
authentication/session-authority task. Do not add an ad-hoc authentication
framework now and do not change Flutter as part of commissioning.
