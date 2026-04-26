# Protocol

This file describes the intended WebSocket protocol between `module_app/` and `firmware/`.

Status: draft.
Exact command names and payloads must be confirmed from code before treating this as final.

## Transport

- Link: Wi-Fi/WebSocket.
- Direction: Flutter app to ESP32 for commands.
- Direction: ESP32 to Flutter app for status, telemetry, errors, and acknowledgements.
- Encoding: expected JSON unless code proves otherwise.

## Protocol rules

- Every command should have a command/type field.
- Stop must be accepted at any time.
- Unknown commands must not move motors.
- Invalid parameter ranges must not move motors.
- Manual drive commands should be time-limited or refreshed frequently.
- ESP32 should stop motion on WebSocket disconnect or command timeout.
- Attachment control must be explicit and should default to off.

## Draft app -> ESP32 commands

These examples are protocol intent, not confirmed final code.

### Manual drive

```json
{
  "type": "manual_drive",
  "left": 0.0,
  "right": 0.0,
  "duration_ms": 200
}
```

Meaning:

- `left`: left motor target, likely normalized from `-1.0` to `1.0`.
- `right`: right motor target, likely normalized from `-1.0` to `1.0`.
- `duration_ms`: command validity window.

Confirm in code:

- actual command name,
- actual speed range,
- whether differential/tank drive or direction commands are used,
- whether duration is implemented.

### Stop

```json
{
  "type": "stop"
}
```

Meaning: immediately stop drive motors.

Required behavior:

- Has priority over manual and autonomous movement.
- Should also put NAV into safe paused/stopped state if autonomy exists.

### Attachment control

```json
{
  "type": "attachment",
  "enabled": false
}
```

Meaning: switch cleaning attachment/nozzle/tool relay/output.

Confirm in code:

- actual command name,
- actual relay/output behavior,
- whether attachment auto-disables on failsafe.

### Heartbeat

```json
{
  "type": "heartbeat",
  "ts": 0
}
```

Meaning: app is alive and connected.

Confirm in code:

- whether heartbeat exists,
- timeout value,
- whether manual drive commands themselves refresh `g_lastCmdMs`.

### Build/send route workflow commands

The auto workflow needs commands similar to:

```json
{
  "type": "route_clear"
}
```

```json
{
  "type": "route_point",
  "index": 0,
  "x": 0.0,
  "y": 0.0
}
```

```json
{
  "type": "route_commit",
  "count": 10
}
```

These are draft shapes only.
Actual upload protocol must be confirmed or implemented deliberately.

### NAV control

Draft commands:

```json
{ "type": "nav_start" }
```

```json
{ "type": "nav_pause" }
```

```json
{ "type": "nav_stop" }
```

Required:

- `nav_stop` must stop motion.
- `nav_pause` should stop motion but preserve route state if safe.
- `nav_start` must be blocked if no valid route is loaded.

## Draft ESP32 -> app telemetry

### Status

```json
{
  "type": "status",
  "mode": "manual",
  "connected": true,
  "motors_enabled": false,
  "attachment_enabled": false,
  "failsafe": false
}
```

### NAV status

```json
{
  "type": "nav_status",
  "mode": "idle",
  "route_loaded": false,
  "route_points": 0,
  "current_waypoint": null
}
```

Needed by auto UI:

- current NAV mode,
- current waypoint,
- route loaded/sent state,
- errors.

### GPS

```json
{
  "type": "gps",
  "fix": false,
  "lat": null,
  "lon": null,
  "accuracy_m": null,
  "satellites": null
}
```

GPS is not physically tested yet.
Do not present GPS telemetry as confirmed until real tests exist.

### Local position

```json
{
  "type": "local_position",
  "valid": false,
  "x": null,
  "y": null,
  "origin_set": false
}
```

Local x/y depends on GPS origin workflow and is not confirmed yet.

### Error

```json
{
  "type": "error",
  "code": "unknown_command",
  "message": "Unknown command"
}
```

## Known protocol risks

- `MAX_WS_MSG` currently conflicts between firmware config and WebSocket code.
- Auto route upload may exceed message size if sent as one large JSON array.
- Route upload should probably be chunked or sequenced.
- App must show route sent/not sent state.
- ESP32 should acknowledge route count or reject invalid route.

## Do not assume

- Do not assume these draft names match code.
- Do not assume heartbeat exists until verified.
- Do not assume GPS messages are live.
- Do not assume waypoint upload is implemented.

