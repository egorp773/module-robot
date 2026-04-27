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
