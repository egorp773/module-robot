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

Code state: local working tree after adding `platformio.ini` and fixing known build blockers.

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

- `platformio.ini` was updated to place build output in `D:/rn-cache/module_robot_pio_build` because `C:` ran out of space during later builds.
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
