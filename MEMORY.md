# Project Memory

Always read this file first before working on the project.

## Project identity

This repository is an MVP for a modular outdoor cleaning robot.
The current target use case is a small robotic snow-cleaner / yard-cleaner module.
The platform is built around an ESP32 firmware and a Flutter mobile/web application.

The project is not a finished autonomous robot.
The confirmed useful MVP today is manual control from the app over Wi-Fi/WebSocket plus attachment on/off control.
Autonomous navigation, GPS mapping, snake-route generation, and RTK are future work or partially coded experiments.

## Repository layout

- `firmware/` is the new modular ESP32 firmware.
- `module_app/` is the Flutter application.
- `sound/` is a legacy monolithic ESP32 sketch/reference.
- `docs/ai_skills/` contains short task-specific instructions for future Codex sessions.
- Git repository exists at `C:\robot\module` and is connected to GitHub: `https://github.com/egorp773/module-robot`.

## Source of truth documents

- `MEMORY.md`: first-read project memory and current truth.
- `IMPLEMENTATION_STATUS.md`: what is confirmed, unverified, broken, and planned.
- `DECISIONS.md`: decisions that must not be silently reversed.
- `ROADMAP.md`: development order and exit criteria.
- `CODEX_TASKS.md`: concrete task queue for Codex.
- `ARCHITECTURE.md`: how ESP32, Flutter, motors, GPS, route planning, and attachments fit together.
- `PROTOCOL.md`: draft WebSocket commands and telemetry.
- `SAFETY.md`: safety rules, stop/failsafe requirements, and autonomy restrictions.
- `HARDWARE.md`: hardware assumptions and TODOs for pins/power/sensors.
- `TEST_LOG.md`: real test results only.

## Current honest state

Confirmed working:

- Manual control from the Flutter app over Wi-Fi/WebSocket.
- Attachment/nozzle/working tool on/off control from the app.
- The project has an ESP32 firmware side and a Flutter app side.

Implemented or started in code but not proven as product-ready:

- New modular firmware exists under `firmware/`.
- Flutter app exists under `module_app/`.
- Autonomous navigation concepts are partially present.
- Snake/lawnmower route generation is partially present.
- Mapping/storage/projection code exists on the app side.
- GPS workflow has some code shape, but the real hardware workflow is not complete.

Known broken or not reliable:

- Firmware build now passes with PlatformIO (`pio run`) as of 2026-04-27.
- Previous build blockers were fixed: duplicate `g_targetLeft/g_targetRight`, local `MAX_WS_MSG` conflict, and missing motor failsafe/smooth-stop symbols.
- Motor command timeout/failsafe is compiled, but not verified on hardware.
- Current snake route generation is poor.
- `lineStep 44.0` is an old wrong value.
- Auto-map robot position must not be treated as reliable GPS position.

Not tested on hardware:

- GPS physical connection.
- GPS telemetry in the app with real lat/lon.
- RTK in the field.
- Autonomous navigation.
- Snake-route driving.
- GPS perimeter recording.
- Sending and following real waypoints.
- Failsafe behavior must still be verified after firmware build is fixed.

## Important architecture notes

The new architecture is modular:

- `firmware/` should become the active ESP32 firmware.
- `module_app/` should remain the active UI/control application.
- `sound/` is not the active architecture.
- Use `sound/` only as legacy reference for motor control, WebSocket behavior, sound, relay, and older implementation details.

Do not copy the monolithic `sound/` structure back into `firmware/`.
If a useful low-level behavior exists in `sound/`, port the idea into the modular firmware shape.

## Route and mapping truth

Future route planning must use local meters, not fake scaled GPS degrees.
The likely future cleaning line spacing is around `0.40` to `0.45` meters.
The old `lineStep 44.0` value is considered wrong and must not be preserved as a real-world spacing.

Manual map recording is currently conditional/simulated.
It is not yet a full GPS perimeter recording workflow.
Until GPS is physically connected and verified, any map screen robot marker is UI state or simulation, not a trustworthy robot position.

## GPS and RTK truth

GPS has not been physically connected and tested.
GPS code and projection helpers do not prove that the robot can localize.
The next GPS milestone is ordinary GPS first:

- wire GPS,
- confirm serial data,
- parse fix status,
- show lat/lon in the app,
- create local x/y from an origin,
- record a GPS perimeter.

RTK comes only after ordinary GPS works.
Do not introduce RTK assumptions into the main workflow before normal GPS has passed real tests.

## Flutter truth

The manual control UI is the most valuable current workflow.
The auto screen is incomplete.
The auto screen needs a complete workflow:

- Build route.
- Send route.
- Start.
- Pause.
- Stop.

The auto screen must show:

- route point count,
- whether the route was sent,
- current NAV mode,
- current waypoint index/point,
- GPS status,
- errors.

App files known to matter:

- `module_app/` contains the Flutter app.
- `cleaning_route_planner.dart` must be moved to local-meter route planning.
- `gps_projection.dart` exists, but the GPS workflow is not complete.
- `map_storage.dart` exists, but full GPS map recording is not complete.

## Firmware truth

The first firmware priority is build correctness.
Known firmware issues to fix before feature work:

- remove duplicate `g_targetLeft/g_targetRight` definitions,
- resolve `MAX_WS_MSG` ownership,
- finish or remove incomplete declarations around smooth stop, failsafe, and command timestamp state.

After it builds, stabilize manual mode before touching autonomy.

Current firmware build command:

- `pio run`

Current firmware build status:

- PASS on 2026-04-27 for `esp32dev` PlatformIO environment.
- This is a compile/build result only, not a hardware validation.

## Main development order

1. Fix firmware build.
2. Stabilize manual mode.
3. Add and verify failsafe.
4. Fix auto UI workflow.
5. Fix route planner to use local meters.
6. Connect GPS.
7. Display lat/lon in the app.
8. Compute local x/y using an origin.
9. Record GPS perimeter.
10. Send waypoints to ESP32.
11. Test autonomous movement without attachment.
12. Test autonomous movement with attachment only after previous tests pass.
13. RTK only after ordinary GPS.

## Do not assume

- Do not assume GPS works.
- Do not assume RTK works.
- Do not assume route following is safe.
- Do not assume snake route generation is correct.
- Do not assume map recording is real GPS recording.
- Do not assume firmware behavior is safe just because it compiles.
- Do not assume failsafe works until tested.
- Do not assume `sound/` is current architecture.
- Do not assume a PASS result without a real entry in `TEST_LOG.md`.
