# Roadmap

This roadmap defines the required development order. Do not skip safety gates.

## Phase 1 - Fix firmware build

Goal: make `firmware/` compile as the active ESP32 firmware.

Work:

- Resolve duplicate `g_targetLeft` and `g_targetRight` definitions.
- Decide ownership of motor target state.
- Resolve `MAX_WS_MSG` conflict between `config.h` and `websocket.cpp`.
- Finish or correctly move `motors_request_smooth_stop`.
- Finish or correctly move `motors_check_failsafe`.
- Finish or correctly move `g_lastCmdMs`.
- Remove stale declarations if they are no longer valid.
- Build the firmware from a clean state.

Exit criteria:

- Firmware builds without duplicate symbol/config errors.
- No unresolved references for motor smooth stop, failsafe, or command timestamp state.
- The build command and result are recorded in `TEST_LOG.md`.

## Phase 2 - Stabilize manual mode

Goal: keep the current MVP reliable.

Work:

- Confirm WebSocket connection from `module_app/` to ESP32.
- Confirm manual forward/back/left/right/stop behavior.
- Confirm attachment on/off behavior.
- Confirm app UI reflects connected/disconnected state.
- Confirm manual controls do not rely on the incomplete auto workflow.

Exit criteria:

- Manual movement test is recorded in `TEST_LOG.md`.
- Stop during manual movement is recorded in `TEST_LOG.md`.
- Attachment on/off test is recorded in `TEST_LOG.md`.
- `IMPLEMENTATION_STATUS.md` is updated with tested details.

## Phase 3 - Add and verify failsafe

Goal: robot stops when control state is uncertain.

Work:

- Implement or complete command timeout.
- Implement or complete WebSocket disconnect stop.
- Implement or complete heartbeat timeout if heartbeat is part of active protocol.
- Ensure invalid commands do not move motors.
- Ensure startup state keeps motors off.
- Ensure attachment defaults to off after reset.

Exit criteria:

- Lost WebSocket test recorded.
- Lost heartbeat or command timeout test recorded.
- Invalid command test recorded.
- Startup safe state test recorded.
- Safety docs updated with actual timeout values.

## Phase 4 - Fix auto UI workflow

Goal: make the auto screen honest and operational without pretending GPS autonomy is ready.

Status: code-level partial done on 2026-04-27; runtime/device verification still needed.

Work:

- Add or fix Build route. Done in `auto_map_screen.dart`.
- Add or fix Send route. Done as draft WebSocket route upload.
- Add or fix Start. Control is present; `NAV_START` is intentionally blocked until GPS/local-meter route workflow is confirmed.
- Add or fix Pause. Done.
- Add or fix Stop. Done.
- Show route point count. Done.
- Show whether route was sent. Done.
- Show current NAV mode. Done.
- Show current waypoint index/point. Done.
- Show GPS status. Done.
- Show errors. Done.
- Clearly separate simulated/conditional map state from confirmed GPS state. Partially done through draft status/errors; still needs runtime review.

Exit criteria:

- User can build a route and see point count.
- User can see whether route was sent.
- User can see start, pause, and stop controls; actual `NAV_START` remains blocked until later safety phases.
- UI does not label simulated map position as real GPS.
- `TEST_LOG.md` contains at least a static check entry. Done.
- Device/runtime layout check is still required before calling this phase complete.

## Phase 5 - Fix route planner in local meters

Goal: route generation uses physical local coordinates.

Work:

- Update `cleaning_route_planner.dart` to operate in local x/y meters.
- Remove dependency on the old wrong `lineStep 44.0` idea.
- Use approximate line spacing around `0.40` to `0.45` m as the starting range.
- Keep route generation deterministic and inspectable.
- Add route diagnostics: point count, line count, bounding box, spacing.

Exit criteria:

- Route planner input and output are documented as local meters.
- Generated snake route is visually and numerically reasonable for a test polygon.
- Point count is shown in UI or logs.
- Old `44.0` spacing is not used as a real route spacing.

## Phase 6 - Connect ordinary GPS

Goal: receive real GPS data from hardware.

Work:

- Fill GPS module model in `HARDWARE.md`.
- Fill GPS UART pins and baud rate.
- Wire GPS to ESP32.
- Confirm raw GPS serial data.
- Parse fix status, lat, lon, and accuracy if available.
- Send GPS status to Flutter.

Exit criteria:

- GPS raw data test recorded.
- GPS fix/no-fix behavior recorded.
- Hardware pins and baud are documented.
- App can show GPS status without crashing.

## Phase 7 - Show lat/lon in the app

Goal: operator can see real geographic position.

Work:

- Add or finish telemetry parsing in Flutter.
- Display lat/lon only when fix is valid.
- Display no-fix clearly.
- Display accuracy if available.
- Log protocol examples in `PROTOCOL.md`.

Exit criteria:

- App screenshot/manual observation confirms lat/lon display.
- No-fix state is visible.
- `TEST_LOG.md` records the test.

## Phase 8 - Local x/y from origin

Goal: convert GPS to local metric coordinates.

Work:

- Use an origin lat/lon.
- Convert current GPS to local x/y meters.
- Keep projection code explicit and testable.
- Do not mix degrees and meters in route planner.

Exit criteria:

- Known small GPS movement changes local x/y plausibly.
- Origin selection is documented.
- App can show or log local x/y.

## Phase 9 - GPS perimeter recording

Goal: record a real perimeter from GPS positions.

Work:

- Start/stop perimeter recording in the app.
- Record only valid GPS fixes.
- Store local x/y points.
- Save/load through `map_storage.dart`.
- Reject or warn about bad/no fix.

Exit criteria:

- A real walked/driven perimeter is recorded.
- Stored map reloads.
- Route planner can consume the recorded local-meter polygon.

## Phase 10 - Send waypoints

Goal: send generated local-meter waypoints to ESP32.

Work:

- Define waypoint protocol.
- Add route id or sequence if needed.
- Show send progress/status.
- Confirm ESP32 stores or acknowledges route.

Exit criteria:

- Route upload test recorded.
- Point count on ESP32 matches point count in Flutter.
- Protocol docs updated with actual messages.

## Phase 11 - Test autonomy without attachment

Goal: verify movement logic with the working tool off.

Work:

- Use low speed.
- Use short route.
- Keep manual stop available.
- Confirm pause/stop behavior.
- Confirm robot does not continue after app disconnect.

Exit criteria:

- Autonomous dry run recorded.
- Stop during autonomy recorded.
- Attachment remains off.

## Phase 12 - Test autonomy with attachment

Goal: enable the working attachment only after movement safety is proven.

Work:

- Repeat the same short route.
- Enable attachment only when operator is ready.
- Keep stop visible and tested.
- Watch power draw and mechanical behavior.

Exit criteria:

- Attachment autonomy test recorded.
- No unresolved safety failures from dry run remain.

## Phase 13 - RTK after ordinary GPS

Goal: improve positioning only after normal GPS workflow is proven.

Work:

- Select RTK module/base/correction method.
- Record setup in `HARDWARE.md`.
- Verify correction status.
- Compare normal GPS and RTK local x/y stability.

Exit criteria:

- RTK field test recorded.
- Correction status is visible in app or logs.
- RTK is not required for manual MVP.

## Do not assume

- Do not skip from manual mode to autonomy.
- Do not test autonomy with attachment before dry autonomy.
- Do not work on RTK before ordinary GPS.
- Do not treat old route spacing as valid.
