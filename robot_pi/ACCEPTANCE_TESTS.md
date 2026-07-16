# Commissioning acceptance gates

Run these gates strictly in order and stop at the first failure. Record date,
operator, Git branch and commit, ESP32 build ID, config hashes, equipment and
evidence. Never interpret one successful run as permission to skip a cold-boot
or fault-injection repeat.

```text
Наличие успешной PlatformIO сборки не означает, что ROS workspace проверен.
```

## Gate 1 -- Power

- [ ] Pi survives cold boot and motor-controller power transitions without reboot.
- [ ] `vcgencmd get_throttled` is `0x0`, or kernel evidence proves no undervoltage.
- [ ] ESP32 reset reason shows no brownout during repeated tests.
- [ ] USB remains enumerated without repeated resets in kernel logs.
- [ ] Relays and hoverboard UART command stay physically off/zero at reboot.

Evidence: `first_boot_check.sh`, `journalctl -k`, multimeter/scope readings and
at least ten power cycles. Do not proceed while power integrity is uncertain.

## Gate 2 -- ROS build and tests

On the Raspberry Pi, from the reviewed `raspberry-pi-migration` checkout:

```bash
./robot_pi/scripts/install_pi.sh --stage manual
./robot_pi/scripts/build_workspace.sh --stage manual
```

- [ ] Branch and short commit match the commissioning record.
- [ ] Manual `rosdep install` completes without unresolved dependencies.
- [ ] All six manual packages build successfully.
- [ ] `colcon test` completes and `colcon test-result --verbose` reports no failure.
- [ ] The resulting workspace setup can be sourced in a new shell.

Do not substitute a Windows `compileall` or either PlatformIO build for this
gate. Full-stage Nav2/localization/RViz installation and build happen only much
later with `--stage full`.

## Gate 3 -- Binary protocol loopback

Keep traction and attachment power disconnected. Stop the ROS bridge so it does
not share the serial device, then run:

```bash
python3 robot_pi/scripts/serial_loopback_test.py
```

- [ ] HELLO/HELLO_ACK negotiates protocol version 1.
- [ ] REQUEST_STATUS reports connected DISARMED, never ARMED.
- [ ] Applied commands and UART speed/steer remain zero.
- [ ] The decoder statistics printed for this transaction contain no framing/CRC error.

Shared ESP32/Python golden vectors and parser unit tests are supporting evidence;
the physical USB loopback is still mandatory. STOP and DISARM in this script
are best-effort cleanup only; their acknowledgement, relay state and reconnect
behavior are not claimed by Gate 3.

## Gate 4 -- Pre-ARM hardware probe

Start manual bringup with `start_gateway:=false`, keep the robot unable to move,
and run:

```bash
./robot_pi/scripts/pre_arm_hardware_probe.sh
```

- [ ] The probe forces and confirms DISARMED.
- [ ] ESP32, motor, power, relay and protocol topics remain available for at least 10 s.
- [ ] Applied left/right and UART speed/steer stay zero for the full observation.
- [ ] Motor feedback is regular and never older than 500 ms.
- [ ] Raw stationary left/right feedback both stay within -5..+5; actual min/max are recorded.
- [ ] Battery voltage and finite board temperature are recorded; unavailable telemetry fails closed.
- [ ] Controller fault and relay active mask remain zero.
- [ ] CRC/COBS/length error counters and watchdog trips do not increase.
- [ ] The timestamped probe report is saved with the commissioning evidence.

The probe never calls ARM, sends CMD_VEL, clears a latch or enables a relay. Any
failure is investigated; do not widen the firmware feedback threshold to pass
this gate.

## Gate 5 -- STOP without movement

The robot must still never have received a non-zero command in this commissioning
run. From the laptop SSH session, use only ROS services/topics; keep the phone
and unauthenticated WebSocket gateway disconnected.

```bash
for attempt in 1 2 3; do
  ros2 service call /safety/stop std_srvs/srv/Trigger '{}'
done
ros2 service call /safety/disarm module_robot_msgs/srv/Disarm '{}'
ros2 topic echo --once /esp32/status
./robot_pi/scripts/pre_arm_hardware_probe.sh
```

- [ ] Repeated STOP is idempotent and every response is successful.
- [ ] DISARM leaves ESP32 DISARMED and does not clear an unexplained latch.
- [ ] Applied commands and UART speed/steer remain hard zero.
- [ ] Relay active mask remains zero.
- [ ] A second timestamped probe report proves relay zero and stable watchdog/protocol counters.

Do not run `manual_drive_test.sh` or `stop_test.sh` before this gate passes.

## Gate 6 -- Lifted manual pulse

Use secure blocks in a controlled area, remove attachment power, keep a physical
traction disconnect ready and ensure neither track can touch the ground. Then:

```bash
./robot_pi/scripts/manual_drive_test.sh
```

- [ ] Operator confirmation is required before ARM.
- [ ] ARM alone creates no movement.
- [ ] The brief 0.03 m/s request is the first intentional non-zero command.
- [ ] STOP produces immediate zero without ramp-down.
- [ ] Feedback and physical track response are recorded on the same timeline.
- [ ] Final state is DISARMED and relays remain off.

Unexpected direction, asymmetry or feedback is a failure. Keep the robot lifted;
calibration happens from recorded observations, never by guessing values.

## Gate 7 -- USB disconnect, bridge crash and Pi reboot STOP

Keep the Gate 6 lifted setup. `stop_test.sh` covers stale ROS command, DISARM,
ESTOP, bridge SIGKILL/restart and supervised physical USB disconnect. Pi reboot
or power loss is a separate supervised test and is never automated here.

- [ ] Valid CMD_VEL gap over 300 ms causes immediate zero, cleared ramp and latched timeout fault.
- [ ] Physical USB disconnect produces physical zero.
- [ ] Bridge SIGKILL produces physical zero.
- [ ] Pi reboot/power loss produces physical zero.
- [ ] ROS command staleness, DISARM and ESTOP each produce hard zero.
- [ ] RESET_ESTOP returns only DISARMED.
- [ ] Reconnect or a new packet never resumes old motion.
- [ ] Fault reset still requires separate ARM and a new CMD_VEL sequence.

When telemetry disappears, require both physical observation and independent
motor-controller evidence. Lowering the robot before every item passes fails the
gate.

## Gate 8 -- Ground movement

Only after Gates 1--7 pass may the robot touch the ground. Use an open controlled
area, the smallest useful limit, a spotter and a physical emergency disconnect.

- [ ] Verify forward and reverse individually.
- [ ] Verify left/right differential and both turn directions.
- [ ] Calibrate and review signs, swap, scales, deadband, track width and maximum percent.
- [ ] Repeat hard-STOP tests after every motor calibration change.
- [ ] Confirm no attachment relay activates across reboot or reconnect.

Ground success authorizes only the tested manual configuration, not autonomous
operation or unattended use.

## Gate 9 -- Localization and autonomy later

This gate is intentionally deferred. Before any autonomous motion:

- [ ] IMU rate, units, mounting, covariance and ENU yaw convention are physically proven.
- [ ] GNSS rate, hAcc/vAcc covariance, RTK carrier state and RTCM age are valid.
- [ ] Every `TODO_MEASURE`/`TODO_CALIBRATE` is replaced by reviewed evidence.
- [ ] TF has one owner per transform; odom is continuous and GPS conversion surveyed.
- [ ] Heading initialization meets displacement/jitter quality thresholds.
- [ ] Full-stage ROS install/build/tests pass on the Pi.
- [ ] Straight, rotation, L-shape, square, repeated path and fault-injection trials pass.

Obstacle avoidance is not present. Gateway authentication/session authority is a
separate prerequisite before field use from a phone. Passing this gate authorizes
only the explicitly tested controlled scenario.
