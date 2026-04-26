# ESP32 Firmware Skill

Use this when working in `firmware/`.
Do not edit `sound/` unless the user explicitly asks for legacy work.

## Active architecture

`firmware/` is the active modular ESP32 firmware.
`sound/` is legacy reference only.

Use `sound/` only to understand old behavior for:

- motor control,
- WebSocket command handling,
- sound,
- relay/attachment,
- older safety ideas.

Do not copy the monolithic `sound/` structure into `firmware/`.

## Current firmware priority

First priority: make firmware build.

Known issues:

- `g_targetLeft/g_targetRight` are defined twice in `motors.cpp` and `nav.cpp`.
- `MAX_WS_MSG` conflicts between `config.h` and `websocket.cpp`.
- `motors_request_smooth_stop` is declared but not fully implemented or moved.
- `motors_check_failsafe` is declared but not fully implemented or moved.
- `g_lastCmdMs` is declared but not fully implemented or moved.

## Expected module responsibilities

- `config.h`: shared constants and pin/config values.
- `websocket.cpp`: transport, message receive/send, protocol parsing or dispatch.
- `motors.cpp`: motor targets, PWM/DIR output, stop/smooth stop, motor failsafe.
- `nav.cpp`: navigation mode, route following, waypoint state.
- GPS module/files: future serial parsing and telemetry.
- Attachment/relay module/files: tool on/off output.

Confirm actual filenames before editing.

## Safety requirements

- Startup motors off.
- Attachment off by default.
- Stop command overrides everything.
- Invalid commands do not move motors.
- Command timeout stops motors.
- Disconnect stops motors.
- NAV must not start without a valid route.

## Firmware work checklist

- Read `../../MEMORY.md`.
- Read `../../IMPLEMENTATION_STATUS.md`.
- Read `../../PROTOCOL.md`.
- Read `../../SAFETY.md`.
- Inspect current code before changing symbols.
- Fix build before adding features.
- Update docs when protocol, pins, or safety behavior changes.

## Do not assume

- Do not assume firmware compiles.
- Do not assume old `sound/` pins are current.
- Do not assume GPS is wired.
- Do not assume RTK exists.
- Do not assume autonomous route following is ready.

