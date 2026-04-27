# Implementation Status

This file is the main truth for what works, what exists only in code, what is broken, and what is planned.

## Status rules

- "Confirmed working" means tested on the real project and documented in `TEST_LOG.md`, or explicitly provided by the user as current honest state.
- "Implemented but unverified" means code or UI exists, but hardware/field behavior is not proven.
- "Known broken" means future Codex should fix it before building on top of it.
- "Planned" means do not claim it works.

## Confirmed working

### Firmware build

Status: confirmed working as a build-only test.

Evidence:

- `pio run` completed successfully on 2026-04-27.
- Environment: PlatformIO `esp32dev`, Arduino framework.
- Output firmware artifact was created under `D:/rn-cache/module_robot_pio_build/esp32dev/`.

Limits:

- This does not confirm hardware behavior.
- This does not confirm GPS, IMU, WebSocket runtime, motor direction, or attachment wiring.
- `.pio/` is generated and intentionally ignored by git.
- PlatformIO build output is intentionally outside the repo on `D:` to avoid filling `C:`.

### Manual app-to-robot control

Status: working MVP behavior.

What is known:

- The Flutter app can manually control the robot over Wi-Fi/WebSocket.
- The operator can drive manually from the app.
- This is the current baseline workflow.

Limits:

- The exact current command names must be confirmed from code before changing protocol docs.
- Safety behavior still needs systematic test logging.

### Attachment on/off

Status: working MVP behavior.

What is known:

- The app can switch the working attachment/nozzle/tool on and off.
- This may involve relay or output control depending on the current firmware wiring.

Limits:

- Exact pin and relay details are still TODO in `HARDWARE.md`.
- Attachment must not be enabled during early autonomous tests.

## Implemented in code but unverified

### Modular ESP32 firmware

Status: present under `firmware/`, but build needs repair.

Known areas:

- Motors.
- WebSocket.
- Navigation.
- Failsafe/smooth stop declarations.
- Configuration constants.

Current build status:

- Builds with `pio run`.
- Build output is configured on `D:/rn-cache/module_robot_pio_build`.

Implemented but unverified safety behavior:

- Manual command timeout zeros motor targets after `CMD_TIMEOUT_MS`.
- WebSocket disconnect stops motors, stops NAV, and switches attachment/mount outputs off.

Risk:

- Runtime behavior is not hardware-tested yet.
- Relay active levels and motor directions are not confirmed.

### Flutter app

Status: present under `module_app/`.

Known areas:

- Manual control.
- Auto/mapping screens.
- Route planner.
- GPS projection helper.
- Map storage helper.

Risk:

- The auto workflow is incomplete.
- The UI does not yet expose all required autonomous state.

Recent code-level hardening:

- Disabled the unsafe simulated connected state in `wifi_connection.dart`.
- Turning off Wi-Fi preflight now only skips the preliminary check; the app must still create a real WebSocket channel and receive connection state.
- Verified changed file with `dart analyze lib/core/wifi_connection.dart`; no errors, existing `avoid_print` info remains.
- Auto map screen now exposes a draft workflow: Build route, Send route, Start, Pause, Stop.
- Auto map screen shows route point count, sent state, NAV mode, waypoint index/total, GPS status, lat/lon if telemetry is present, and workflow errors.
- `NAV_START` from the auto map screen is blocked in the UI until the GPS route workflow is confirmed.
- Verified changed auto map file with `dart analyze module_app/lib/features/auto/auto_map_screen.dart`; no errors, existing/deprecated style info remains.

### Route generation

Status: partially implemented, not reliable, but old `44.0` default has been removed from the active cleaning planner.

Known issues:

- Snake route generation is poor.
- `lineStep 44.0` is an old wrong value and must not be reintroduced.
- `CleaningRoutePlanner` now defaults to `0.42` local meters and validates positive finite spacing.
- Border-pass offset now uses the local-meter step approximately instead of the old `step / 100` behavior.
- Future route generation still must be tied to real GPS-derived local x/y maps.
- Target cleaning line spacing is approximately `0.40` to `0.45` m.
- Route diagnostics are logged when debug mode is enabled: line step, zone bounding box, forbidden count, segment count, route point count, distance, and cleaning segment count.

Limits:

- This is a code-level planner cleanup only.
- Existing saved manual maps may still be conceptual/cell maps, not measured local-meter maps.
- No real polygon/GPS perimeter test has confirmed physical route quality.

### GPS projection and map storage

Status: code exists, workflow incomplete.

Known files:

- `gps_projection.dart` exists.
- `map_storage.dart` exists.
- `wifi_connection.dart` parses `GPS,<lat>,<lon>,<heading>,<fixType>,<hAcc>` telemetry.
- Auto map workflow panel displays lat/lon only when telemetry fields are present.

Limits:

- GPS has not been physically connected.
- Manual map recording is mostly conditional/simulated.
- It is not a confirmed GPS perimeter recording workflow.
- Displaying lat/lon in the app does not prove GPS wiring, fix quality, or field localization.

## Known broken / must fix

### Firmware build issues - resolved for compile

- `g_targetLeft` and `g_targetRight` ownership is now in `motors.cpp`.
- `MAX_WS_MSG` is now taken from `config.h`.
- `motors_request_smooth_stop`, `motors_check_failsafe`, and `g_lastCmdMs` now have compiled definitions.
- PlatformIO project config exists in `platformio.ini`.

Impact:

- Compile blocker is resolved.
- Runtime safety still cannot be trusted until hardware tests are run.

### Auto UI workflow gaps

Status: code-level partial, not hardware-tested.

Implemented in UI:

- Build route.
- Send route.
- Start.
- Pause.
- Stop.
- Route point count.
- Route sent/not sent.
- Current NAV mode.
- Current waypoint.
- GPS status.
- Workflow errors.

Limits:

- Route sending is still a draft protocol workflow.
- Route upload is blocked unless the map is marked GPS-based and has `refLat/refLon`.
- When a GPS origin exists, route upload converts local-meter points to lat/lon with `GpsProjection`.
- Start control is visible, but the UI intentionally blocks `NAV_START` until GPS/local-meter route workflow is confirmed.
- No real robot autonomous test has been run.
- Do not mark autonomy or GPS route following as working from this UI change.

### Mapping/route issues

- Current snake route is not good enough.
- Current line step is wrong if using `44.0`; active cleaning planner default is now `0.42`.
- Route generation must be converted to local metric x/y coordinates.
- Robot marker on auto map must not be presented as reliable GPS position yet.

## Not tested on hardware

- GPS physical wiring.
- GPS UART data.
- GPS fix/no-fix behavior.
- lat/lon telemetry in Flutter.
- local x/y origin workflow.
- GPS perimeter recording.
- waypoint upload to ESP32.
- autonomous route following.
- RTK in field conditions.
- autonomous movement with attachment enabled.

## Planned

- Stabilize manual mode after firmware build.
- Verify stop/failsafe behavior with real tests.
- Complete auto UI workflow.
- Convert route planner to local meters.
- Wire and test GPS.
- Show lat/lon in the app.
- Convert GPS to local x/y from a selected origin.
- Record GPS perimeter.
- Send waypoints to ESP32.
- Test autonomy without attachment.
- Test attachment only after autonomous movement is safe.
- Add RTK only after normal GPS works.

## Do not assume

- Do not assume any auto feature is ready.
- Do not assume GPS exists just because projection code exists.
- Do not assume `sound/` is valid current firmware.
- Do not assume route planner output is physically meaningful until fixed and tested.
- Do not mark any untested item as PASS.
