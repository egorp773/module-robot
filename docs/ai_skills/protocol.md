# Protocol Skill

Use this for WebSocket command or telemetry work.

## Source of truth

Read `../../PROTOCOL.md`.
It is a draft until exact command names are confirmed from code.

## Current protocol reality

Known current behavior:

- Flutter app controls robot over Wi-Fi/WebSocket.
- Attachment on/off works from the app.

Unconfirmed details:

- exact command names,
- speed ranges,
- heartbeat implementation,
- route upload format,
- NAV status messages,
- GPS telemetry format.

## Required command groups

Manual:

- drive left/right or direction/speed,
- stop,
- attachment on/off.

Safety:

- heartbeat or command timeout,
- error responses,
- status messages.

Auto:

- route clear/upload/commit,
- nav start,
- nav pause,
- nav stop,
- current waypoint telemetry.

GPS:

- fix/no-fix,
- lat/lon,
- accuracy,
- local x/y after origin exists.

## Protocol risks

- `MAX_WS_MSG` conflict in firmware.
- Large route arrays may exceed message size.
- App must know whether route was actually sent.
- ESP32 should acknowledge route count.

## Do not assume

- Do not assume draft examples are exact code.
- Do not assume route upload exists.
- Do not assume GPS telemetry exists.
- Do not assume heartbeat exists.

