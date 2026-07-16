# Migration plan

The migration is additive. Existing ESP32 firmware is not deleted, and the
PlatformIO `rover` environment remains the legacy rollback/build image. It does
not inherit the `pi_bridge` safety case. Flash it only as a deliberate,
traction-disabled recovery operation, then re-run the safety case appropriate
to that firmware; it is not an operational fallback for guaranteed STOP.

## Stage 0 -- preserve, power and inventory

1. Record the current `rover` firmware build identifier and physical wiring.
2. Build both `rover` and `pi_bridge` from the same repository revision.
3. Capture ESP32 USB VID/PID plus serial, or ID_PATH only when serial is absent.
4. Measure power rails with motors disabled.
5. Leave the tracks lifted and attachment power disconnected.

Exit criterion: acceptance Gate 1 (power) passes and neither image changes
relay/motor state unexpectedly at boot.

## Stage 1 -- manual commissioning and guaranteed STOP

1. Clone the reviewed `raspberry-pi-migration` branch and record its commit.
2. Install/build/test `--stage manual` on the Pi; this is Gate 2.
3. Flash `pi_bridge`, keep systemd disabled, and run the binary loopback Gate 3.
4. Start manual bringup with `start_gateway:=false`; run the no-motion pre-ARM
   probe (Gate 4) and zero-only STOP/DISARM proof (Gate 5).
5. With the robot securely lifted, run the one-second manual pulse as the first
   intentional non-zero command (Gate 6).
6. Keep it lifted for stale-command, USB, bridge-crash and separately supervised
   Pi reboot/power-loss STOP tests (Gate 7).

Exit criterion: Pi manual `TwistStamped` reaches the motor controller, every
loss-of-command case produces hard zero, and recovery always requires explicit
ARM. On-ground driving remains prohibited until this exit criterion passes.

## Stage 2 -- measured ground manual control and gateway preparation

1. From lifted evidence, calibrate one drivetrain parameter at a time; do not
   guess signs, scales, deadband, track width or maximum percent.
2. Repeat STOP proof after every motor-path calibration change.
3. Perform controlled ground direction/scale checks for Gate 8 with a spotter
   and physical emergency disconnect.
4. Validate route upload without executing it.
5. Measure IMU axes/mounting and GNSS antenna offsets for the later Gate 9.
6. Define and review RTCM transport before expecting Pi-sourced RTK. Protocol v1
   intentionally contains no `RTCM_DATA`; do not tunnel unbounded correction
   bytes through motion frames.

The WebSocket gateway stays out of systemd and is not used for the first manual
test. A later bench launch must be manual and on a controlled network. Field
phone control requires a separate authentication/session-authority safety task;
this commissioning patch does not change Flutter or add an auth framework.

Exit criterion: Gate 8 passes with recorded calibration evidence. This
authorizes only the tested manual configuration.

## Stage 3 -- localization

1. Replace all relevant `TODO_MEASURE` values with physical measurements.
2. Prove base_link/ENU axis conventions using manual motion.
3. Enable local EKF without GPS; prove continuous `odom -> base_link`.
4. Run the heading initializer manually through the safety interface.
5. Only after yaw validation, enable navsat_transform and global EKF.
6. Install and test the explicit full ROS stage on the Pi.

Exit criterion: the localization portion of Gate 9 passes repeatedly after cold
boot. It does not authorize autonomous movement by itself.

## Stage 4 -- path following

1. Keep `autonomous_enabled=false` until an operator reviews the complete
   preflight report.
2. Convert validated Flutter GPS waypoints to `nav_msgs/Path`.
3. Send that path directly to Nav2 FollowPath; do not run a global planner that
   can cut the supplied snake route.
4. Begin with 1 m straight and rotate-in-place tests, then L-shape and square.
5. Inject sensor, serial and process faults during every progression.

Exit criterion: the autonomous portion of Gate 9 passes. This still does not
authorize obstacle-rich or unattended operation.

## Explicitly deferred

SLAM, AMCL, lidar/camera layers, obstacle avoidance, docking, charging, cloud
services, OTA firmware, automatic startup and automatic post-fault motion are
outside this migration. Add them only under separate reviewed safety cases.
