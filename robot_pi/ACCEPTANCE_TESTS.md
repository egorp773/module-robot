# Acceptance gates

Run gates in order. Stop at the first failure. Record date, operator, firmware
build IDs, config hashes, equipment and evidence (logs/video/measurements). A
single successful run does not justify skipping cold-boot and fault-injection
repeats.

## Gate 1 — Power

- [ ] Pi survives cold boot and motor-controller power transitions without reboot.
- [ ] `vcgencmd get_throttled` is `0x0`, or kernel evidence proves no undervoltage.
- [ ] ESP32 reset reason shows no brownout during repeated tests.
- [ ] USB remains enumerated; no repeated disconnect/reconnect in kernel logs.
- [ ] Relays and UART motor command remain physically off/zero at every reboot.

Evidence: `first_boot_check.sh`, `journalctl -k`, multimeter/scope readings and
ten power cycles.

## Gate 2 — Protocol

- [ ] HELLO/HELLO_ACK negotiates protocol version 1.
- [ ] With no existing latch, handshake/reconnect reaches DISARMED and never ARMED.
- [ ] Reconnect preserves any pre-existing FAULT or ESTOP latch.
- [ ] CRC errors are zero or explained/rare under expected cable/load conditions.
- [ ] Split frames, multiple frames/read and malformed frames do not break parsing.
- [ ] Duplicate, stale and out-of-order CMD_VEL sequences are counted and ignored.
- [ ] Sequence wraparound accepts only the mathematically newer command.
- [ ] Unknown types/oversize payloads are counted and never affect motion watchdog.
- [ ] USB reconnect restores telemetry but never ARM or relay state.

Evidence: protocol stats plus shared ESP32/Python golden vectors.

## Gate 3 — guaranteed STOP

Run with tracks lifted and video showing commands, UART/motor feedback and
physical tracks on the same timeline.

The first intentional non-zero command is the one-second lifted
`manual_drive_test.sh` pulse after Gates 1 and 2. Keep the same mechanical setup
for `stop_test.sh`; that script covers stale ROS command, DISARM, ESTOP, bridge
SIGKILL/restart and physical USB disconnect. Pi reboot/power loss is a separate
supervised test and is never automated by the repository.

- [ ] Valid CMD_VEL gap >300 ms -> immediate zero, cleared ramp and latched timeout fault.
- [ ] Physical USB disconnect -> zero.
- [ ] SIGKILL of bridge -> zero.
- [ ] Pi reboot/power loss -> zero.
- [ ] ROS cmd source stale -> STOP/zero.
- [ ] DISARM -> hard zero without ramp.
- [ ] ESTOP -> hard zero and latch.
- [ ] RESET_ESTOP returns only DISARMED.
- [ ] Reconnect or new CMD_VEL never resumes old motion.
- [ ] Reset fault still requires a separate ARM and separate CMD_VEL.

Use `stop_test.sh` only after its mechanical prerequisites are met. If connected
telemetry reports either applied command as non-zero after a STOP event, the
gate fails even if the tracks appear still. When USB/Pi loss makes telemetry
unavailable, physical zero and motor-controller evidence are both required.
Do not lower the robot onto the ground before this entire gate passes.

## Gate 4 — manual signs and scale

At the smallest useful command, test each independently and record encoder/
feedback signs. Do not alter multiple sign parameters between observations.

- [ ] forward;
- [ ] reverse;
- [ ] left track differential;
- [ ] right track differential;
- [ ] turn left;
- [ ] turn right.

Then calibrate `left_sign`, `right_sign`, `swap_left_right`, scales, deadband,
track width and maximum percent. `velocity_limits.yaml` is not applied
automatically; record the reviewed explicit `SET_LIMITS`/firmware procedure and
prove reboot/reconnect does not restore an old motion or relay state. Retest
STOP after every motor-path change.

## Gate 5 — sensors

- [ ] IMU messages sustain 50 Hz without console spam or serial starvation.
- [ ] Axis mapping and SI units match REP-103/ENU documentation.
- [ ] Quaternion and covariance reflect measured validity; yaw is not assumed.
- [ ] GNSS sustains its configured 5–10 Hz rate.
- [ ] NavSatFix covariance derives from hAcc/vAcc; unavailable vertical accuracy has large variance, not zero.
- [ ] RTK carrier state reaches FIXED and is reported separately.
- [ ] Motor feedback remains fresh under command/telemetry load.
- [ ] Sensor-unavailable transitions produce one diagnostic transition, not spam.

## Gate 6 — localization

- [ ] TF tree has one owner per transform and no loops.
- [ ] `odom -> base_link` is continuous without GPS jumps.
- [ ] GNSS-to-ENU conversion is checked against surveyed movements.
- [ ] IMU mounting and yaw direction are physically proven.
- [ ] Heading initializer meets displacement/jitter thresholds and reports quality.
- [ ] `map -> odom` changes are explainable and do not move base frames discontinuously.
- [ ] Cold-start and restart behavior are repeatable.

Do not enable navsat_transform until yaw convention passes.

## Gate 7 — autonomous path following

Use an open controlled outdoor area, low limits, physical ESTOP and spotter:

- [ ] 1 m straight path;
- [ ] rotate in place both directions;
- [ ] L-shape;
- [ ] square;
- [ ] repeated uploaded route without global-planner corner cutting;
- [ ] pause/resume requires explicit state and never resumes after fault;
- [ ] inject GNSS, IMU, RTK, localization, Nav2, serial and process faults;
- [ ] every violation produces immediate zero and the documented latch level.

Obstacle avoidance is not present. Passing Gate 7 authorizes only the tested
controlled scenario, not unattended operation.
