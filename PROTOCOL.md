# Protocol

This file describes the WebSocket protocol between `module_app/` and `firmware/`.

Status: code-confirmed draft as of 2026-04-27.
The active protocol is line-based text, not JSON.
Keep this file synchronized with `firmware/websocket.cpp`, `firmware/telemetry.cpp`, and `module_app/lib/core/wifi_connection.dart`.

## Transport

- Link: Wi-Fi/WebSocket.
- ESP32 endpoint: `ws://192.168.4.1:81/ws`.
- ESP32 also exposes HTTP `GET /ping` returning `OK`.
- Encoding: UTF-8 text lines.
- Message separator: one WebSocket text message per command.
- Maximum inbound message length: `MAX_WS_MSG` from `firmware/config.h`, currently `256`.

## Protocol rules

- Stop must be accepted at any time.
- Unknown commands must not move motors.
- Manual drive commands are refreshed by the app while the user is driving.
- ESP32 has a motor command timeout (`CMD_TIMEOUT_MS`, currently `400`) that zeros motor targets if movement commands stop.
- ESP32 stops motors and calls `nav_stop()` on WebSocket disconnect.
- Attachment and mount controls are explicit commands.
- This protocol is still not hardware-verified after the firmware build fix.

## App -> ESP32 commands

### Ping

```text
PING
```

Firmware response:

```text
PONG
```

Current app behavior: sends `PING` after connection as an optional check. It does not currently run a periodic heartbeat loop.

### Manual drive

```text
M,<left>,<right>
```

Example:

```text
M,60,60
```

Flutter sends `left` and `right` in the range `-100..100`.
Firmware divides incoming values by `INPUT_DIV` and clamps to `-MAX_SPEED_PERCENT..MAX_SPEED_PERCENT`.

Current config:

- `INPUT_DIV = 2`
- `MAX_SPEED_PERCENT = 70`
- `CMD_TIMEOUT_MS = 400`

Meaning:

- positive values drive forward if wiring/motor direction is correct,
- negative values drive backward,
- different left/right values turn.

Hardware tests still need to confirm physical motor direction.

### Stop

```text
STOP
```

Firmware response:

```text
OK STOP
```

Firmware behavior:

- calls `motors_request_smooth_stop("manual stop")`,
- sets left/right targets to zero.

### Attachment control

```text
ATTACHMENT_ON
ATTACHMENT_OFF
```

Firmware responses:

```text
OK ATTACHMENT_ON
OK ATTACHMENT_OFF
```

Firmware behavior:

- calls `setAttachment(true/false)`,
- uses `PIN_RELAY_ATTACH`.

Hardware TODO:

- confirm relay pin,
- confirm active level,
- confirm attachment defaults off on boot and failsafe.

### Mount control

```text
MOUNT_ON
MOUNT_OFF
```

Firmware responses:

```text
OK MOUNT_ON
OK MOUNT_OFF
```

Firmware behavior:

- calls `setMount(true/false)`,
- uses `PIN_RELAY_MOUNT`.

### Sound

```text
SOUND:<id>
SOUND,<id>
```

Valid ids: `1..4`.

Firmware responses:

```text
OK SOUND
ERR SOUND_RANGE
```

### Route upload

RTK/autopilot route commands use local-meter coordinates. The app sends the
GPS origin once, then sends each waypoint as local `x/y` meters relative to
that origin.

Preferred current flow is robot-side planning: the app sends a cleaning zone
polygon and the rover builds the snake route onboard. Direct waypoint upload is
kept as a lower-level fallback.

### Robot-side area planning

```text
AREA_BEGIN,<count>,<originLat>,<originLon>,<lineStepMeters>
AREA_PT,<index>,<x_m>,<y_m>
AREA_END
```

Example:

```text
AREA_BEGIN,4,55.12345678,37.12345678,0.420
AREA_PT,0,0.000,0.000
AREA_PT,1,4.000,0.000
AREA_PT,2,4.000,2.000
AREA_PT,3,0.000,2.000
AREA_END
```

Firmware responses:

```text
OK
OK,ROUTE,<count>
ERR,AREA_BEGIN
ERR,AREA_PT
ERR,PLAN_FAILED
```

Rules:

- `count` must be `3..32`.
- points are local meters relative to `originLat/originLon`.
- the rover starts the generated route from its current estimated local position.
- the generated route is stored inside rover firmware and then started with `NAV_START`.

```text
ROUTE_BEGIN,<count>,<originLat>,<originLon>
```

```text
ROUTE_WP,<index>,<x_m>,<y_m>
```

```text
ROUTE_END
```

Example:

```text
ROUTE_BEGIN,4,55.12345678,37.12345678
ROUTE_WP,0,0.000,0.000
ROUTE_WP,1,2.500,0.000
ROUTE_WP,2,2.500,1.000
ROUTE_WP,3,0.000,1.000
ROUTE_END
```

Firmware responses:

```text
OK
ERR,INVALID
ERR,ROUTE_INCOMPLETE
```

Rules:

- `count` must be `1..160`.
- `originLat` and `originLon` must be non-zero.
- waypoint indexes outside `0..count-1` are rejected.
- `NAV_START` is accepted only after every waypoint has been received.

### NAV control

```text
NAV_START
NAV_PAUSE
NAV_RESUME
NAV_STOP
```

Firmware responses:

```text
OK
ERR,NO_ROUTE
```

Safety status:

- Compiles.
- Rover autopilot code exists but is not a completed field test.
- `NAV_START` can enter RUNNING with a valid route, but motor output remains
  zero while rover quality is `ERROR` or `LOST_WAIT`.
- Do not start motor tests until RTK/heading state is verified on hardware.

## ESP32 -> app messages

### Connection state

On WebSocket connect:

```text
STATE,CONNECTED
```

The Flutter app waits for this or another state/connected message before setting `isConnected = true`.

### Battery percent

```text
BAT_PCT,<percent>
```

Example:

```text
BAT_PCT,75
```

### Battery verbose

```text
BAT,V=<voltage>V,P=<percent>%,temp=<temp>C
```

Example:

```text
BAT,V=39.20V,P=75%,temp=31.0C
```

### GPS telemetry

```text
GPS,<lat>,<lon>,<heading>,<fixType>,<hAcc>
```

Example:

```text
GPS,55.12345678,37.12345678,90.00,3,450
```

GPS status:

- Code exists.
- GPS hardware has not been physically connected and tested.
- App must not present map position as trustworthy GPS until tests pass.

### GPS debug telemetry

The RTK rover sends extended GPS status for field debugging:

```text
GPSDBG,<lat>,<lon>,<heightM>,<heading>,<fixType>,<carrier>,<diff>,<numSV>,<hAccMm>,<vAccMm>,<speedMps>,<pDop>,<ageMs>
```

Example:

```text
GPSDBG,55.12345678,37.12345678,184.250,90.00,3,fixed,1,22,14,22,0.020,0.85,41
```

`carrier` is `none`, `float`, or `fixed`. The Flutter GPS debug screen can use
both `GPS` and `GPSDBG`.

### RTCM telemetry

```text
RTCM,<bytesTotal>,<ageMs>,<transportAgeMs>,<f9pAgeMs>,<source>,<f9pMessages>,<crcFail>,<lastType>
```

`source` is currently `udp` or `none` in the active RTK rover firmware.
`f9pMessages` is the important proof that the rover F9P decoded RTCM internally
through `UBX-RXM-RTCM`.

### IMU telemetry

```text
IMU,<yaw>,<ageMs>,<fresh>
```

Example:

```text
IMU,182.50,12,1
```

IMU status:

- Code exists for BNO08x rotation vector.
- Hardware presence and orientation are not confirmed in `HARDWARE.md`.

### NAV telemetry

```text
NAV,<state>,<wpIdx>,<wpTotal>,<distToWp>
```

Example:

```text
NAV,RUNNING,2,18,0.73
```

States:

- `IDLE`
- `RUNNING`
- `PAUSED`
- `DONE`
- `ERROR`

The auto UI should show this state, current waypoint, total waypoints, and errors.

### Motor telemetry

```text
MOTOR,<left>,<right>,<feedback>,<speedL>,<speedR>,<batRaw>,<tempRaw>
```

Example:

```text
MOTOR,0,0,1,0,0,4060,355
```

`feedback=1` means the hoverboard motor controller is replying on UART. If it is
`0`, do not start autonomous movement.

### Generic errors

Known current errors:

```text
ERR,TOO_LONG
ERR,UNKNOWN
ERR SOUND_RANGE
ERR ROUTE_WP_FORMAT
ERR ROUTE_WP_FULL
```

## Known protocol risks

- Protocol is text-based but older documentation and future designs may mention JSON. Do not mix them accidentally.
- Route and area upload use local meters relative to a GPS origin.
- `PING` is not a full heartbeat/failsafe loop.
- App must keep sending manual `M,left,right` updates during movement; firmware timeout stops stale movement commands.
- Hardware behavior still needs real tests.

## Do not assume

- Do not assume GPS telemetry means GPS hardware works.
- Do not assume NAV commands are safe to run on the robot.
- Do not assume route waypoints are local meters yet.
- Do not assume `PING` is a safety heartbeat.
