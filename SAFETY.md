# Safety

This robot can move and can control a working attachment.
Safety work has priority over autonomous features.

## Core principle

If state is uncertain, stop motion and keep the attachment off.

Uncertain state includes:

- app disconnected,
- heartbeat or command timeout,
- invalid command,
- unknown NAV state,
- GPS no-fix,
- route not loaded,
- waypoint out of range,
- startup/reset,
- firmware parse error.

## Current safety status

Confirmed:

- Manual stop is expected as part of MVP manual control, but detailed safety tests still need logging.

Not confirmed:

- WebSocket disconnect failsafe.
- Heartbeat failsafe.
- Command duration timeout.
- Startup safe state.
- Invalid command behavior.
- Attachment off on failsafe.
- Autonomous stop/pause behavior.

Implemented in code but not hardware-tested:

- On WebSocket disconnect, firmware stops motor targets, calls `nav_stop()`, and switches attachment/mount outputs off.
- Manual command timeout is compiled through `motors_check_failsafe()`.
- Startup initializes attachment and mount outputs off.

## Hard restrictions

- Do not test autonomous movement with the attachment enabled until dry autonomy passes.
- Do not treat GPS position as valid without GPS fix and accuracy/status.
- Do not use RTK assumptions before ordinary GPS works.
- Do not keep moving after app disconnect.
- Do not keep moving after command timeout.
- Do not start NAV without a valid loaded route.
- Do not start NAV from simulated/conditional map state as if it were GPS.
- Do not use `sound/` legacy code as safety proof.

## Required ESP32 safety behavior

### Startup safe state

On boot/reset:

- drive motors off,
- target speeds zero,
- attachment off,
- NAV mode idle/stopped,
- no route movement until explicit valid command.

### Stop command

Stop command must:

- immediately set motor targets to zero,
- override manual and NAV commands,
- be accepted in any mode,
- be safe if repeated,
- preferably disable or pause NAV.

### Command timeout

Manual drive should not be an infinite command.
Either each command has `duration_ms`, or the firmware uses `g_lastCmdMs` to stop when updates stop.

Known issue:

- `g_lastCmdMs` is declared but not fully implemented/moved.

### WebSocket disconnect

When app connection is lost:

- stop motors,
- stop NAV,
- switch attachment off,
- switch mount off,
- prevent new movement until a valid connection/command returns,
- report status after reconnect if possible.

### Heartbeat timeout

If heartbeat is used:

- heartbeat timeout value must be documented,
- timeout must stop motors,
- timeout should not require the app to send stop.

### Invalid command behavior

For invalid JSON, unknown command, bad range, or missing fields:

- reject command,
- send error if possible,
- do not move motors,
- do not enable attachment.

### Attachment safety

Attachment/nozzle/tool output should:

- default off,
- be controlled explicitly,
- turn off on startup,
- turn off on WebSocket disconnect failsafe,
- never be enabled automatically during first autonomous tests.

## Required Flutter safety behavior

Flutter UI should:

- always expose Stop on manual and auto screens,
- show connection state,
- show GPS status,
- show NAV mode,
- show errors,
- show route sent/not sent,
- avoid presenting simulated map position as real GPS.

## Autonomy safety gates

Before dry autonomy:

- firmware builds,
- manual mode stable,
- stop tested,
- disconnect/timeout failsafe tested,
- route planner produces local-meter points,
- auto UI has Start/Pause/Stop.

Before autonomy with attachment:

- dry autonomy pass,
- stop during autonomy pass,
- pause behavior pass,
- disconnect during autonomy pass,
- attachment on/off manual test pass.

Before RTK:

- ordinary GPS connected,
- lat/lon displayed,
- local x/y origin works,
- GPS perimeter recording works.

## Test requirements

Each safety behavior must have a `TEST_LOG.md` entry.
Do not write PASS for a safety behavior that was not run.

Minimum safety tests:

- startup safe state,
- manual stop while moving,
- stop with attachment enabled,
- app disconnect while moving,
- heartbeat/command timeout,
- invalid command,
- NAV stop during movement,
- attachment remains off during dry autonomy.

## Do not assume

- Do not assume code compiles means safe.
- Do not assume manual control implies failsafe works.
- Do not assume GPS no-fix is harmless in auto mode.
- Do not assume operator can recover if stop is not visible.
