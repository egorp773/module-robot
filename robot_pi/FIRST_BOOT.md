# First boot on Raspberry Pi 4

These steps assume little Linux experience. Commands are entered one line at a
time. Text such as `<REPOSITORY_URL>` must be replaced with the real value.

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
git clone <REPOSITORY_URL> ~/module
cd ~/module
chmod +x robot_pi/scripts/*.sh robot_pi/scripts/*.py
```

Never paste repository credentials into a tracked configuration file.

## 4. Install Ubuntu and ROS dependencies

```bash
cd ~/module
./robot_pi/scripts/install_pi.sh
sudo reboot
```

Reconnect with SSH after about two minutes. The installer refuses non-Noble or
non-ARM64 systems, checks every required apt package and does not start robot
services.

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

Inspect candidates (the command makes no changes):

```bash
cd ~/module
./robot_pi/scripts/setup_udev.sh
./robot_pi/scripts/setup_udev.sh --device /dev/ttyACM0
```

Use the actual device shown on your Pi; it may be `/dev/ttyUSB0`. Record
`ID_VENDOR_ID`, `ID_MODEL_ID` and `ID_SERIAL_SHORT`, then install the rule:

```bash
sudo ./robot_pi/scripts/setup_udev.sh --device /dev/ttyACM0 --install VID PID SERIAL
```

Use the same actual device path inspected above and replace all three values.
The script refuses a mismatch. Unplug/replug USB and verify:

```bash
ls -l /dev/module-esp32
sudo ./robot_pi/scripts/configure_pi.sh --user egor --repo-root ~/module
```

Log out and back in (or reboot) so `dialout` membership applies.

## 7. Build the ROS workspace

```bash
cd ~/module
./robot_pi/scripts/build_workspace.sh
source ~/module/robot_pi/ros2_ws/install/setup.bash
```

Do not try to build this Jazzy workspace on Windows.

## 8. Run non-motion checks

```bash
cd ~/module
./robot_pi/scripts/first_boot_check.sh
```

Resolve every `[FAIL]`. Warnings about power, time or USB stability also need a
decision before motion.

Before starting the ROS bridge, verify one safe binary-protocol round trip:

```bash
python3 ~/module/robot_pi/scripts/serial_loopback_test.py
```

It sends only HELLO and REQUEST_STATUS, requires DISARMED/hard-zero, then sends
STOP and DISARM. Close every serial monitor first.

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
  safety_config:=$HOME/module/robot_pi/config/safety.yaml
```

Do not launch the autonomous bringup. Do not enable systemd yet.

## 10. First lifted movement and STOP proof

Read [ACCEPTANCE_TESTS.md](ACCEPTANCE_TESTS.md). Complete Gates 1 and 2 before
any non-zero command. Then move the robot to a controlled area, lift it on
secure blocks, remove attachment power and keep a physical disconnect ready.
The tracks must not touch the ground. In a separate terminal run:

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

The expected `is-enabled` result is `disabled`. `stop_test.sh` deliberately
commands a very low speed, kills the bridge, and asks for a USB disconnect; its
exact mechanical confirmation is mandatory. It does not reboot the Pi. Perform
the separate Pi reboot/power-loss item in Gate 3 only under the same lifted,
supervised conditions and record physical zero.

Do not put the robot on the ground until all Gate 3 STOP cases pass. Do not set
`autonomous_enabled: true` or enable autonomous services until manual control,
watchdog STOP, measurements, sensors and localization have passed their gates.
Systemd enablement is a later, explicit operator decision; none of these scripts
enables a service.
