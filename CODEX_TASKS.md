# Codex Tasks

This file contains concrete tasks for future Codex sessions.
Before starting any task, read `CODEX_START_HERE.md`, `MEMORY.md`, `IMPLEMENTATION_STATUS.md`, `SAFETY.md`, and the relevant skill file under `docs/ai_skills/`.

## Working rules

- Do not edit `sound/` unless the user explicitly asks for legacy reference changes.
- Do not treat `sound/` as active firmware.
- Do not edit code just because documentation mentions a file; follow the user's task.
- If firmware protocol changes, update `PROTOCOL.md`.
- If pins or wiring change, update `HARDWARE.md`.
- If safety behavior changes, update `SAFETY.md`.
- If a real test is run, update `TEST_LOG.md`.
- If a feature becomes confirmed, update `IMPLEMENTATION_STATUS.md`.

## Immediate priority tasks

### TASK-001 - Fix firmware build

Status: done on 2026-04-27

Scope: `firmware/` only.

Problem:

- `g_targetLeft/g_targetRight` are defined twice in `motors.cpp` and `nav.cpp`.
- `MAX_WS_MSG` conflicts between `config.h` and `websocket.cpp`.
- `motors_request_smooth_stop`, `motors_check_failsafe`, and `g_lastCmdMs` are declared but incomplete or not fully moved.

Done when:

- Firmware compiles. Done with `pio run`.
- Build command/result is logged in `TEST_LOG.md`. Done.
- No active architecture is moved back into `sound/`. Done.

Notes:

- Added `platformio.ini`.
- `platformio.ini` uses `D:/rn-cache/module_robot_pio_build` for PlatformIO build output to avoid filling `C:`.
- Fixed duplicate motor target definitions by keeping ownership in `motors.cpp`.
- Removed local `MAX_WS_MSG` shadow definition from `websocket.cpp`.
- Added compiled definitions for motor smooth stop, command timestamp, and failsafe check.

### TASK-002 - Stabilize manual mode

Status: open

Scope: `firmware/` and `module_app/` only if required.

Work:

- Confirm manual WebSocket commands.
- Confirm motor directions.
- Confirm stop behavior.
- Confirm attachment on/off behavior.
- Confirm app connection state.

Done when:

- Manual test is recorded in `TEST_LOG.md`.
- `PROTOCOL.md` reflects actual commands.
- `IMPLEMENTATION_STATUS.md` has exact confirmed details.

Progress:

- Manual command names were verified by source review and documented in `PROTOCOL.md`.
- App no longer reports `isConnected=true` from the disabled Wi-Fi preflight branch without a real WebSocket channel.
- Hardware tests are still required before marking this task done.

### TASK-003 - Implement and test failsafe

Status: in progress

Work:

- Command timeout.
- WebSocket disconnect stop.
- Heartbeat timeout if used.
- Invalid command rejection.
- Startup safe state.
- Attachment off on reset/failsafe if applicable.

Done when:

- Each safety behavior has a real test log entry.
- `SAFETY.md` contains actual timeout values.

Progress:

- Command timeout compiles.
- WebSocket disconnect now stops motors/NAV and switches attachment/mount outputs off.
- Firmware builds after the change.
- Real hardware tests are still required.

### TASK-004 - Fix auto screen workflow

Status: code-level partial done on 2026-04-27; runtime/hardware unverified.

Scope: `module_app/`.

Work:

- Build route. Done in `auto_map_screen.dart`.
- Send route. Done as draft route upload through existing WebSocket commands.
- Start. Control is present, but `NAV_START` is intentionally blocked until GPS/local-meter route workflow is confirmed.
- Pause. Done through existing `NAV_PAUSE` command.
- Stop. Done through existing `NAV_STOP` command.
- Show route point count. Done.
- Show route sent state. Done.
- Show NAV mode. Done.
- Show current waypoint. Done.
- Show GPS status. Done.
- Show errors. Done.

Done when:

- The UI workflow is visible and usable.
- The UI does not claim simulated map state is real GPS.

Remaining:

- Run on a device/emulator for visual layout verification.
- Test with a real WebSocket robot session.
- Confirm route upload behavior against firmware logs.
- Keep autonomy marked unverified until GPS/local-meter routing is complete.

### TASK-005 - Convert route planner to local meters

Status: code-level partial done on 2026-04-27; GPS/local-map workflow still open.

Scope: likely `cleaning_route_planner.dart` and related tests/helpers.

Work:

- Use local x/y meters. Partially done by treating `CleaningRoutePlanner` spacing as local meters.
- Remove wrong `lineStep 44.0` semantics. Done in active cleaning planner default and auto map call.
- Use starting line spacing around `0.40` to `0.45` m. Done with `0.42`.
- Add route diagnostics. Done for debug logging.

Done when:

- Route planner output is local-meter waypoints.
- A simple polygon produces a reasonable snake route.

Remaining:

- Confirm all map inputs are true local-meter coordinates, not old conceptual cells.
- Add a repeatable route planner test fixture.
- Verify route visually on device/emulator.
- Feed planner from a real GPS perimeter after GPS phases are complete.

### TASK-006 - Connect and display GPS

Status: blocked until hardware is connected.

Work:

- Document GPS model, UART pins, and baud.
- Confirm raw GPS data.
- Parse fix, lat, lon, accuracy if available.
- Display lat/lon and GPS status in app.

Done when:

- GPS hardware test is recorded.
- App shows lat/lon only with valid fix.

### TASK-007 - Build local x/y and perimeter recording

Status: blocked until GPS works.

Work:

- Select origin.
- Convert GPS to local x/y.
- Record perimeter points.
- Store/load map.
- Feed route planner from recorded polygon.

Done when:

- Real GPS perimeter test is recorded.

### TASK-008 - Waypoint upload and dry autonomy

Status: blocked until route planner and GPS perimeter work.

Work:

- Define waypoint upload protocol.
- Send waypoints to ESP32.
- Test movement without attachment.
- Verify stop/pause/disconnect behavior.

Done when:

- Dry autonomy test passes and is logged.

## Documentation tasks

- Keep `CODEX_START_HERE.md` aligned with `MEMORY.md`.
- Keep `PROTOCOL.md` marked draft until command names are verified from code.
- Keep `HARDWARE.md` TODO fields explicit until real wiring is known.
- Keep `TEST_LOG.md` honest: no PASS without a real run.

## Do not assume

- Do not assume the next task is autonomy.
- Do not assume GPS or RTK is ready.
- Do not assume code in `sound/` should be edited.
- Do not assume there is git history to inspect.
