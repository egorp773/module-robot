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

Status: open

Scope: `firmware/` only.

Problem:

- `g_targetLeft/g_targetRight` are defined twice in `motors.cpp` and `nav.cpp`.
- `MAX_WS_MSG` conflicts between `config.h` and `websocket.cpp`.
- `motors_request_smooth_stop`, `motors_check_failsafe`, and `g_lastCmdMs` are declared but incomplete or not fully moved.

Done when:

- Firmware compiles.
- Build command/result is logged in `TEST_LOG.md`.
- No active architecture is moved back into `sound/`.

### TASK-002 - Stabilize manual mode

Status: open after TASK-001.

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

### TASK-003 - Implement and test failsafe

Status: open after manual mode is stable.

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

### TASK-004 - Fix auto screen workflow

Status: open after manual/failsafe work.

Scope: `module_app/`.

Work:

- Build route.
- Send route.
- Start.
- Pause.
- Stop.
- Show route point count.
- Show route sent state.
- Show NAV mode.
- Show current waypoint.
- Show GPS status.
- Show errors.

Done when:

- The UI workflow is visible and usable.
- The UI does not claim simulated map state is real GPS.

### TASK-005 - Convert route planner to local meters

Status: open after auto UI basics.

Scope: likely `cleaning_route_planner.dart` and related tests/helpers.

Work:

- Use local x/y meters.
- Remove wrong `lineStep 44.0` semantics.
- Use starting line spacing around `0.40` to `0.45` m.
- Add route diagnostics.

Done when:

- Route planner output is local-meter waypoints.
- A simple polygon produces a reasonable snake route.

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

