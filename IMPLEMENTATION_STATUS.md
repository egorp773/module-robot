# Implementation Status

This file is the main truth for what works, what exists only in code, what is broken, and what is planned.

## Status rules

- "Confirmed working" means tested on the real project and documented in `TEST_LOG.md`, or explicitly provided by the user as current honest state.
- "Implemented but unverified" means code or UI exists, but hardware/field behavior is not proven.
- "Known broken" means future Codex should fix it before building on top of it.
- "Planned" means do not claim it works.

## Confirmed working

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

Risk:

- The firmware has known symbol/config conflicts and incomplete moved functions.

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

### Route generation

Status: partially implemented, not reliable.

Known issues:

- Snake route generation is poor.
- `lineStep 44.0` is an old wrong value.
- Future route generation must use local meters.
- Target cleaning line spacing is approximately `0.40` to `0.45` m.

### GPS projection and map storage

Status: code exists, workflow incomplete.

Known files:

- `gps_projection.dart` exists.
- `map_storage.dart` exists.

Limits:

- GPS has not been physically connected.
- Manual map recording is mostly conditional/simulated.
- It is not a confirmed GPS perimeter recording workflow.

## Known broken / must fix

### Firmware build issues

- `g_targetLeft` and `g_targetRight` are defined twice in `motors.cpp` and `nav.cpp`.
- `MAX_WS_MSG` conflicts between `config.h` and `websocket.cpp`.
- `motors_request_smooth_stop` is declared but not fully implemented or moved.
- `motors_check_failsafe` is declared but not fully implemented or moved.
- `g_lastCmdMs` is declared but not fully implemented or moved.

Impact:

- Firmware build and runtime safety cannot be trusted until these are resolved.

### Auto UI workflow gaps

Missing required workflow:

- Build route.
- Send route.
- Start.
- Pause.
- Stop.

Missing required status:

- route point count,
- route sent/not sent,
- current NAV mode,
- current waypoint,
- GPS status,
- errors.

### Mapping/route issues

- Current snake route is not good enough.
- Current line step is wrong if using `44.0`.
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

- Fix firmware build.
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

