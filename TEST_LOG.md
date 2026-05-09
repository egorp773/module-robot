# Test Log

This file records real tests only.

Rule: if a test was not actually run, do not write PASS.
Use NOT RUN, BLOCKED, FAIL, PARTIAL, or OBSERVED instead.

## How to write results

- Include date and time.
- Include firmware/app version or file state if known.
- Include hardware setup.
- Include what was done.
- Include expected and actual result.
- Include whether the result changes `IMPLEMENTATION_STATUS.md`.

## Test entry template

```text
Date/time:
Test:
Area: firmware / app / hardware / safety / route / GPS / RTK
Code state:
Hardware:
Conditions:
Steps:
Expected result:
Actual result:
Result: PASS / FAIL / PARTIAL / BLOCKED / NOT RUN / OBSERVED
Issues found:
Status update needed:
Next action:
```

## Current baseline notes

### 2026-04-27 - Firmware PlatformIO build

Date/time: 2026-04-27, Europe/Moscow timezone

Test: firmware build

Area: firmware

Code state: local working tree after adding `rtk_firmware/platformio.ini` and fixing known build blockers.

Hardware: no hardware used.

Conditions: PlatformIO Core 6.1.19, env `esp32dev`, Arduino framework.

Steps:

- Ran `pio run`.
- First build attempt exposed an incomplete local PlatformIO Arduino framework package: `pins_arduino.h` missing.
- Removed the corrupted local framework package from PlatformIO cache.
- Re-ran `pio run`.

Expected result: firmware compiles without duplicate symbol/config errors and without unresolved motor failsafe/smooth-stop symbols.

Actual result: build succeeded, `firmware.bin` was created under `.pio/build/esp32dev/`.

Follow-up verification on 2026-04-27:

- `rtk_firmware/platformio.ini` was updated to place build output in `D:/rn-cache/module_robot_pio_build` because `C:` ran out of space during later builds.
- `pio run` succeeded again with output under `D:/rn-cache/module_robot_pio_build/esp32dev/firmware.bin`.

Result: PASS

Issues found:

- Local PlatformIO cache had a damaged `framework-arduinoespressif32` package and had to be reinstalled.
- This is not a hardware test.

Status update needed: `IMPLEMENTATION_STATUS.md` should list firmware build as confirmed build-only status.

Next action: Phase 2 manual mode stabilization and protocol verification.

### 2026-04-27 - Manual WebSocket protocol code review

Date/time: 2026-04-27, Europe/Moscow timezone

Test: manual protocol consistency check

Area: firmware/app

Code state: local working tree after firmware build fix.

Hardware: no hardware used.

Conditions: source review only; no APK build and no robot connection.

Steps:

- Compared `firmware/websocket.cpp` command parser with `module_app/lib/core/wifi_connection.dart`.
- Confirmed app sends `M,<left>,<right>`, `STOP`, `ATTACHMENT_ON/OFF`, `MOUNT_ON/OFF`, `PING`, route commands, and NAV commands.
- Confirmed firmware accepts the same text commands.

Expected result: manual app command names match firmware command parser.

Actual result: manual command names match at code level.

Result: OBSERVED

Issues found:

- This is not a hardware test.
- Motor direction, relay active level, disconnect behavior, and command timeout still need real tests.

Status update needed: keep manual mode as current MVP, but do not mark detailed manual tests as PASS.

Next action: commit firmware build fix, then continue with manual/failsafe hardening without APK build.

### 2026-04-27 - Flutter manual connection safety check

Date/time: 2026-04-27, Europe/Moscow timezone

Test: manual connection state code check

Area: app/manual/safety

Code state: after removing simulated connected state from `wifi_connection.dart`.

Hardware: no hardware used.

Conditions: static Dart analysis of changed file only. APK build was intentionally not run.

Steps:

- Removed behavior where disabled Wi-Fi preflight set `isConnected=true` without a real WebSocket channel.
- Kept the setting as "skip preflight", but still require the normal WebSocket connection and `STATE,CONNECTED` flow.
- Ran `dart analyze lib/core/wifi_connection.dart`.

Expected result: changed file has no Dart syntax/static errors and app cannot report connected without a real WebSocket channel through this branch.

Actual result: `dart analyze lib/core/wifi_connection.dart` completed with 9 existing `avoid_print` info findings and no errors.

Result: PARTIAL

Issues found:

- This is not a hardware/app runtime test.
- Existing `avoid_print` analyzer info remains.
- No APK build was run per user instruction.

Status update needed: manual mode is safer at code level, but still needs real robot tests for connection, movement, stop, attachment, and disconnect.

Next action: continue Phase 2/3 with firmware failsafe and manual tests when hardware is available.

### 2026-04-27 - Firmware disconnect failsafe build check

Date/time: 2026-04-27, Europe/Moscow timezone

Test: firmware disconnect failsafe compile check

Area: firmware/safety

Code state: after updating WebSocket disconnect handling.

Hardware: no hardware used.

Conditions: PlatformIO `esp32dev`, build output on `D:/rn-cache/module_robot_pio_build`.

Steps:

- Added attachment and mount off commands to WebSocket disconnect handling.
- Ran `pio run`.

Expected result: firmware compiles after disconnect failsafe hardening.

Actual result: build succeeded and `firmware.bin` was created.

Result: PASS

Issues found:

- This is not a hardware failsafe test.
- Need real tests for moving disconnect, attachment-on disconnect, and relay active level.

Status update needed: mark disconnect attachment-off behavior as implemented in code but unverified on hardware.

Next action: continue manual/failsafe hardware test preparation; do not mark failsafe PASS until robot test.

### 2026-04-27 - Auto UI workflow static check

Date/time: 2026-04-27, Europe/Moscow timezone

Test: auto map screen workflow code check

Area: app/route

Code state: after updating `module_app/lib/features/auto/auto_map_screen.dart`.

Hardware: no hardware used.

Conditions: static Dart analysis of changed file only. APK build was intentionally not run.

Steps:

- Added Build route, Send route, Start, Pause, and Stop controls to the auto map screen.
- Added visible route point count, sent state, NAV mode, waypoint index/total, GPS status, and workflow error state.
- Changed the auto screen route build call away from the old `lineStep 44.0` value to a draft `0.42` meter spacing constant.
- Kept `NAV_START` blocked in the UI until GPS/local-meter route workflow is confirmed.
- Ran `dart analyze module_app/lib/features/auto/auto_map_screen.dart`.

Expected result: changed file has no Dart syntax/static errors and the UI no longer lacks the basic autonomous workflow controls.

Actual result: analyzer completed with no errors; existing/deprecated style info remains, mostly `withOpacity`.

Result: PARTIAL

Issues found:

- This is not a device UI test.
- This is not a robot/WebSocket runtime test.
- Route upload is still draft and uses map/local route points, not confirmed GPS waypoints.
- No APK build was run per user instruction.

Status update needed: mark auto UI workflow as code-level partial, not confirmed autonomous operation.

Next action: verify UI layout/runtime without APK build, then continue route planner conversion to local meters.

### 2026-04-27 - Cleaning route planner local-meter cleanup

Date/time: 2026-04-27, Europe/Moscow timezone

Test: route planner static code check

Area: app/route

Code state: after updating `module_app/lib/core/cleaning_route_planner.dart`.

Hardware: no hardware used.

Conditions: static Dart analysis only. APK build was intentionally not run.

Steps:

- Changed `CleaningRoutePlanner` default line spacing from old `44.0` to `0.42` local meters.
- Added validation for positive finite `lineStep`.
- Changed approximate inner border pass from the old `step / 100` behavior to a local-meter offset toward the centroid.
- Added debug diagnostics for line step, zone bbox, forbidden count, snake segment count, route point count, distance, and cleaning segment count.
- Ran `dart analyze module_app/lib/core/cleaning_route_planner.dart module_app/lib/features/auto/auto_map_screen.dart`.

Expected result: changed files have no Dart syntax/static errors and active planner no longer uses `44.0` as route spacing.

Actual result: analyzer completed with no errors; existing info findings remain (`avoid_print`, deprecated `withOpacity`).

Result: PARTIAL

Issues found:

- This is not a route quality test.
- This is not a GPS/local-map test.
- Existing saved maps may still be conceptual/cell maps rather than measured local-meter maps.
- No APK build was run per user instruction.

Status update needed: mark route planner local-meter conversion as code-level partial only.

Next action: add a repeatable route planner fixture and then continue GPS connection/display phases.

### 2026-04-27 - GPS lat/lon display static check

Date/time: 2026-04-27, Europe/Moscow timezone

Test: app GPS display code check

Area: app/GPS

Code state: after updating auto map workflow panel lat/lon display.

Hardware: no hardware used.

Conditions: static Dart analysis only. APK build was intentionally not run.

Steps:

- Confirmed `wifi_connection.dart` already parses `GPS,<lat>,<lon>,<heading>,<fixType>,<hAcc>` telemetry.
- Added lat/lon display to the auto map workflow panel when telemetry values exist.
- Kept no-data state as `LL: -`.
- Ran `dart analyze module_app/lib/features/auto/auto_map_screen.dart`.

Expected result: changed file has no Dart syntax/static errors and app UI has a place to show GPS lat/lon.

Actual result: analyzer completed with no errors; existing deprecated `withOpacity` info remains.

Result: PARTIAL

Issues found:

- This is not a GPS hardware test.
- This does not confirm GPS UART wiring, fix quality, or field localization.
- No APK build was run per user instruction.

Status update needed: GPS display is code-level partial only.

Next action: physical GPS connection test, then verify real lat/lon telemetry in the app.

### 2026-04-27 - Guarded route upload projection check

Date/time: 2026-04-27, Europe/Moscow timezone

Test: route upload safety code check

Area: app/route/GPS/safety

Code state: after guarding auto route upload.

Hardware: no hardware used.

Conditions: static Dart analysis only. APK build was intentionally not run.

Steps:

- Blocked route upload when the loaded map is not GPS-based or lacks `refLat/refLon`.
- Converted local-meter route points to GPS lat/lon through `GpsProjection` before sending `ROUTE_WP`.
- Kept `NAV_START` blocked until the GPS route workflow is confirmed.
- Ran `dart analyze module_app/lib/features/auto/auto_map_screen.dart`.

Expected result: changed file has no Dart syntax/static errors and app no longer sends local x/y directly as fake lat/lon.

Actual result: analyzer completed with no errors; existing deprecated `withOpacity` info remains.

Result: PARTIAL

Issues found:

- This is not a route upload runtime test.
- This is not a GPS origin/perimeter test.
- No APK build was run per user instruction.

Status update needed: route upload is safer at code level, but still unverified on robot.

Next action: real GPS origin workflow and route upload test after hardware GPS works.

### 2026-04-26 - Documentation initialization

Area: documentation

Steps: created source-of-truth project documentation files.

Actual result: documentation structure exists and has been expanded with known project state.

Result: OBSERVED

Status update needed: no feature status should be upgraded from this entry.

### Manual control MVP - user-provided current state

Area: app/firmware

Actual result: user reports manual control from Flutter over Wi-Fi/WebSocket works, and attachment on/off works.

Result: OBSERVED

Issues found: exact protocol and safety behavior still need confirmation from code and real test entries.

Status update needed: `IMPLEMENTATION_STATUS.md` lists this as confirmed current MVP based on user-provided state, but detailed PASS tests still need to be recorded.

### 2026-05-09 - RTK/autopilot navigation follow-up static check

Date/time: 2026-05-09, Europe/Moscow timezone

Test: source review and local code hardening before the next rover navigation run

Area: RTK/autopilot route following

Hardware: no hardware used in this step.

Steps:

- Reviewed the latest autopilot notes and current diff.
- Found that the notes said RTCM buffers were widened, but active `base.cpp` and `rover.cpp` still used `512` byte buffers.
- Increased RTCM frame/packet buffers to `1200` bytes and made rover drop/report oversize or short UDP reads instead of forwarding a truncated RTCM packet to F9P.
- Rejected negative `ROUTE_WP` indexes on the rover.
- Made immediate `NAV,ARRIVED` notifications use the full `NAV,<state>,<wpIdx>,<wpTotal>,<dist>` shape expected by Flutter.
- Updated `PROTOCOL.md` and `IMPLEMENTATION_STATUS.md` for the local-meter route protocol and unblocked `NAV_START` workflow.

Expected result: code matches the intended local-meter navigation protocol and no RTCM packet is silently truncated before F9P.

Actual result:

- `pio run -e base` passed.
- `pio run -e rover` passed.
- `flutter test test/gps_navigation_test.dart test/gps_debug_map_points_test.dart` passed.
- `flutter analyze --no-fatal-infos lib/core/wifi_connection.dart lib/features/auto/auto_map_screen.dart lib/features/gps/gps_debug_screen.dart lib/core/gps_navigation.dart` passed.
- Plain `flutter analyze` still reports existing info-level `withOpacity` deprecations in `auto_map_screen.dart`.

Result: CODE PASS / HARDWARE PENDING

Issues found:

- This is not a hardware navigation test.
- RTK/F9P decoding still must be verified by rover logs showing `f9pMessages` increasing and carrier `float` or `fixed`.

Next action: flash `base` and `rover`, then monitor COM logs before any motor movement.

### 2026-05-09 - RTK/autopilot flash and COM log check

Date/time: 2026-05-09, Europe/Moscow timezone

Test: flash updated RTK base/rover firmware and inspect serial logs

Area: RTK transport, F9P decode proof, rover navigation readiness

Hardware:

- Base on `COM4`.
- Rover on `COM6`.
- Wi-Fi `Xiaomi_6A92`.

Steps:

- Flashed base: `pio run -e base -t upload --upload-port COM4`.
- Flashed rover: `pio run -e rover -t upload --upload-port COM6`.
- Captured short and longer monitor logs under `test_results/2026-05-09_navigation_followup/`.
- Added rover GPS fallback/watchdog: GGA fallback remains enabled and NMEA is parsed when UBX NAV-PVT is stale.
- Added base RTCM parser resync after CRC fail and flashed base again.

Observed logs:

- Base connects at `192.168.31.207`.
- Rover connects at `192.168.31.222`.
- Rover receives RTCM UDP with relay errors `read=0 write=0 oversize=0`.
- Rover F9P now confirms internal RTCM decode: `F9P decoded` increases and `crcFail=0`.
- Rover IMU is fresh.
- Rover GPS fallback remains fresh through NMEA GGA when UBX NAV-PVT is absent/stale.

Remaining problem:

- Rover `Carrier` remains `0 (NONE)`.
- Rover quality remains `DEGRADED`; no motor navigation should be started yet.
- Base still outputs/parses very sparse MSM correction frames: mostly `1005` and `1230`, with rare `1074/1084`.
- Base RTCM parser still sees many CRC-fail candidates in the mixed UBX/RTCM stream, though forwarded packets have no UDP errors.

Result: PARTIAL PASS

Conclusion:

- The old physical/software blocker `RTCM Fresh but F9P decoded=0` is fixed.
- The current blocker is RTK correction quality from the base: the rover F9P decodes RTCM, but does not receive enough usable observation corrections to enter RTK float/fixed.

Next action:

- Do not start autonomous motors.
- Fix base RTCM output/configuration so MSM messages (`1074`, `1084`, and ideally other enabled constellations) repeat continuously after survey-in valid.
- Then re-test until rover shows `carrier=float` or `carrier=fixed`.

### 2026-05-09 - RTK base raw RTCM stream after survey-in

Date/time: 2026-05-09, Europe/Moscow timezone

Test: flash base raw RTCM forwarding change and inspect base/rover serial logs

Area: RTK base correction stream, rover carrier solution

Hardware:

- Base on `COM4`.
- Rover on `COM6`.
- Wi-Fi `Xiaomi_6A92`.

Steps:

- Changed base firmware so after `svinValid=1` and UART `RTCM3`-only output, the ESP32 forwards raw RTCM bytes to rover in UDP chunks instead of gating forwarding on the ESP32 RTCM frame parser.
- Built base: `pio run -e base`.
- Flashed base: `pio run -e base -t upload --upload-port COM4`.
- Captured 150 seconds of paired logs:
  - `test_results/2026-05-09_navigation_followup/base_COM4_after_raw_forward.log`
  - `test_results/2026-05-09_navigation_followup/rover_COM6_after_raw_forward.log`
- Reflashed the same transport with parse-only RTCM type counters in raw mode and captured a second paired log:
  - `test_results/2026-05-09_navigation_followup/base_COM4_after_raw_forward_typecount.log`
  - `test_results/2026-05-09_navigation_followup/rover_COM6_after_raw_forward_typecount.log`

Observed logs:

- Base reached `svinValid=1` at `dur=60s`, final repeat log `svinAcc=1.292m`.
- Base printed `BASE: RTCM raw forwarding enabled`.
- Base raw stream stayed continuous: final repeat log `rawPkts=1847`, `rawBytes=145626`, `udpErr=0`, `rtcmAge=15ms`.
- Base RTCM type counters increased continuously after raw mode: final repeat log `t1074=59`, `t1084=59`, `t1094=59`, `t1124=59`.
- Rover switched from `Carrier: 0 (NONE)` to `Carrier: 2 (FIXED)` immediately after raw forwarding enabled.
- Rover final status: `Fix: 3`, `Carrier: 2 (FIXED)`, `hAcc: 14 mm`, `Quality: RTK_FIXED`.
- Rover RTCM relay remained clean: `read=0 write=0 oversize=0`.
- Rover F9P decode continued increasing with no RTCM CRC failures: repeat log reached `F9P decoded: msgs=1297 crcFail=0`.

Result: PASS for RTK correction delivery and rover carrier lock on the bench.

Conclusion:

- The previous blocker `rover carrier none` is resolved in this hardware setup.
- The root issue was the base ESP32 RTCM parser dropping/corrupting useful long correction traffic before forwarding. Raw stream forwarding after RTCM-only UART output fixes it.
- Autonomous motor navigation is still not validated by this test. The next movement test must start as a controlled dry run with RTK fixed and attachment disabled.

### 2026-05-09 - Rover motor protocol and onboard planner fix

Date/time: 2026-05-09, Europe/Moscow timezone

Test: fix rover motor UART protocol, add manual command support, move route planning onboard, flash rover, and inspect serial log

Area: rover motor controller link, app/autonomy responsibility split

Hardware:

- Rover on `COM6`.
- Wi-Fi `Xiaomi_6A92`.

Steps:

- Fixed hoverboard motor command frame in `rtk_firmware/src/rover.cpp` to match the old working packed little-endian `0xABCD` protocol and field XOR checksum.
- Added rover handling for manual `M,<left>,<right>` commands with a 400 ms timeout.
- Moved motor send/receive to a 20 ms loop.
- Added motor feedback parsing and `MOTOR,<...>` telemetry.
- Added `AREA_BEGIN` / `AREA_PT` / `AREA_END` commands so the app sends the cleaning polygon and the rover builds the snake route onboard.
- Updated auto UI so `Send zone` uploads the zone, not prebuilt waypoints, and `Start` is blocked unless RTK, IMU, and motor feedback are ready.
- Built rover: `pio run -e rover`.
- Flashed rover: `pio run -e rover -t upload --upload-port COM6`.
- Captured log:
  - `test_results/2026-05-09_navigation_followup/rover_COM6_after_motor_area_fix.log`

Observed logs:

- Rover boots and initializes `Motor UART: RX=16 TX=17 115200 baud`.
- `MOTOR: UART2 initialized`.
- Motor controller feedback is present: `Motor FB: fresh=1 cmd=(0,0) speed=(0,0) bat=4060..4068 temp=339..357`.
- RTK remained available during the check: `Carrier: 2 (FIXED)`, `Quality: RTK_FIXED` / occasional `RTK_FLOAT`.

Result: PASS for motor-controller UART link at zero command and firmware/app build checks.

Conclusion:

- The three-beep/no-wheel-response symptom was consistent with the RTK rover firmware sending an invalid hoverboard frame. The firmware now uses the old working binary frame format and the controller replies.
- Movement itself still needs a controlled manual test: start with wheels lifted or tracks clear, send a tiny manual command, then STOP.

### 2026-05-09 - Navigation ownership cleanup

Date/time: 2026-05-09, Europe/Moscow timezone

Test: remove duplicated app-side autonomous navigation and old firmware tree

Area: app/firmware architecture

Steps:

- Removed `module_app/lib/core/gps_navigation.dart` and the old app tests for app-side motor command mapping.
- Added `module_app/lib/core/gps_display_math.dart` for monitor-only distance, bearing, heading error, and local x/y display.
- Updated GPS debug navigation panel so the app sends `AREA_BEGIN` / `AREA_PT` / `AREA_END`, `NAV_START`, `NAV_STOP`, `CAL_IMU`, and `IMU_RESET`; rover firmware owns route following, IMU offset, and motor output.
- Removed app-side direct `ROUTE_*` helpers from `wifi_connection.dart`; `ROUTE_*` remains only a rover firmware fallback/parser path.
- Removed old root `firmware/` and root `platformio.ini`; active firmware is now only `rtk_firmware/`.
- Updated protocol/status/architecture docs to point at `rtk_firmware/`.

Verification:

- `flutter analyze --no-fatal-infos lib/core/gps_display_math.dart lib/core/wifi_connection.dart lib/features/gps/gps_debug_screen.dart lib/features/auto/auto_map_screen.dart`: PASS, only existing `withOpacity` infos in `auto_map_screen.dart`.
- `flutter test test/gps_display_math_test.dart test/gps_debug_map_points_test.dart test/gps_perimeter_storage_test.dart`: PASS.
- `pio run -e rover`: PASS.
- `pio run -e base`: PASS.

Result: PASS for code-level ownership cleanup.

Conclusion:

- The app no longer computes autonomous motor commands or stores local IMU calibration offset.
- Navigation authority is now single-owner: `rtk_firmware/src/rover.cpp`.

## Required future tests

- Firmware build test.
- Manual forward/back/left/right/stop test.
- Attachment on/off test.
- Stop while moving test.
- WebSocket disconnect failsafe test.
- Heartbeat or command timeout test.
- Invalid command safety test.
- GPS raw UART test.
- GPS fix/no-fix app display test.
- Local x/y projection test.
- GPS perimeter recording test.
- Route upload test.
- Autonomous dry run without attachment.
- Autonomous run with attachment after dry run passes.
- RTK field test after ordinary GPS works.
