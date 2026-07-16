# Migration plan

The migration is additive. Existing ESP32 firmware is not deleted, and the
PlatformIO `rover` environment remains the legacy rollback/build image. It does
not inherit the `pi_bridge` safety case. Flash it only as a deliberate,
traction-disabled recovery operation, then re-run the safety case appropriate
to that firmware; it is not an operational fallback for guaranteed STOP.

## Stage 0 — preserve and inventory

1. Record the current `rover` firmware build identifier and physical wiring.
2. Build both `rover` and `pi_bridge` from the same repository revision.
3. Capture ESP32 USB VID, PID and serial; render the udev rule.
4. Measure power rails with motors disabled.
5. Leave the tracks lifted and attachment power disconnected.

Exit criterion: Gate 1 (power) passes and neither image changes relay/motor
state unexpectedly at boot.

## Stage 1 — serial bridge and guaranteed STOP

1. Flash `pi_bridge`; do not enable systemd.
2. Start only `module_robot_esp32_bridge` and observe `HELLO_ACK`/status.
3. Verify a clean boot/reconnect remains DISARMED, while an existing FAULT or
   ESTOP latch survives reconnect; no path may reconnect into ARMED.
4. Verify protocol corruption, stale/duplicate sequences and reconnect behavior.
5. With Gates 1 and 2 recorded, stop the bridge-only launch and start the
   manual-safe bringup (bridge plus safety, no autonomy). Run the one-second
   manual drive script with the robot securely lifted; this is the first
   intentional non-zero command.
6. Keep the robot lifted and run all Gate 3 fault injections, including the
   separate Pi reboot/power-loss check that cannot be automated safely.

Exit criterion: Pi manual `TwistStamped` reaches the motor controller, every
loss-of-command case produces hard zero, and recovery always requires explicit
ARM. On-ground driving remains prohibited until this exit criterion passes.

## Stage 2 — gateway compatibility and sensors

1. Start the safety node and gateway; keep autonomous start disabled.
2. Compare legacy Flutter `M,left,right` semantics against calibrated low-speed
   `linear/angular` conversion.
3. Validate route upload without executing it.
4. Measure IMU axes/mounting and GNSS antenna offsets.
5. Validate IMU, GNSS, RTK and motor feedback rates/covariances.
6. Replace drivetrain TODOs only from recorded Gate 4 measurements. Review and
   apply `SET_LIMITS` explicitly; never auto-apply calibration on reconnect.
7. Define and review the RTCM transport before expecting Pi-sourced RTK. Wire
   protocol v1 intentionally contains no `RTCM_DATA`; use a proven external F9P
   correction input meanwhile, or introduce a bounded protocol-v2 message as a
   separate change. Never tunnel unbounded correction bytes through CMD frames.

Exit criterion: Gates 4 and 5 pass; signs and scales are documented rather than
assumed.

## Stage 3 — localization

1. Replace all relevant `TODO_MEASURE` values with physical measurements.
2. Prove base_link/ENU axis conventions using manual motion.
3. Enable local EKF without GPS; prove continuous `odom -> base_link`.
4. Run the heading initializer manually through the safety interface.
5. Only after yaw validation, enable navsat_transform and global EKF.

Exit criterion: Gate 6 passes repeatedly after cold boot.

## Stage 4 — path following

1. Keep `autonomous_enabled=false` until an operator reviews the complete
   preflight report.
2. Convert validated Flutter GPS waypoints to `nav_msgs/Path`.
3. Send that path directly to Nav2 FollowPath; do not run a global planner that
   can cut the supplied snake route.
4. Begin with 1 m straight and rotate-in-place tests, then L-shape and square.
5. Inject sensor, serial and process faults during every progression.

Exit criterion: Gate 7 passes. This still does not authorize obstacle-rich or
unattended operation.

## Explicitly deferred

SLAM, AMCL, lidar/camera layers, obstacle avoidance, docking, charging, cloud
services, OTA firmware, automatic startup and automatic post-fault motion are
outside this migration. Add them only under separate reviewed safety cases.
