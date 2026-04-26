# Architecture

## System overview

The project is an MVP modular robot cleaner/snow-cleaner.
It has three major software areas:

- `firmware/`: active modular ESP32 firmware.
- `module_app/`: Flutter app for control, monitoring, mapping, and future route workflows.
- `sound/`: legacy monolithic firmware reference only.

The current practical architecture is manual control first.
Autonomy is a later layer that depends on GPS, route planning, waypoint upload, and safety verification.

## High-level diagram

```text
Operator
  |
  v
Flutter app (`module_app/`)
  |
  | Wi-Fi / WebSocket
  v
ESP32 firmware (`firmware/`)
  |
  +-- motor driver -> drive motors
  +-- relay/output -> attachment on/off
  +-- GPS UART -> future lat/lon and local x/y
  +-- IMU I2C -> future motion/orientation data
  +-- safety logic -> stop/failsafe decisions
```

## Active firmware

`firmware/` is the target ESP32 firmware.
It should contain modular responsibilities:

- config constants,
- WebSocket server/client handling,
- command parsing,
- motor control,
- attachment/relay control,
- navigation state,
- route/waypoint handling,
- GPS telemetry,
- failsafe/safety logic.

Known firmware files from current project notes:

- `motors.cpp`: motor control and target speed state.
- `nav.cpp`: navigation behavior and possibly target state conflict.
- `config.h`: shared constants.
- `websocket.cpp`: WebSocket message handling and possible local message size conflict.

Known issue:

- Some state and declarations are split incorrectly. Fix build before adding features.

## Legacy firmware reference

`sound/` is a legacy monolithic implementation.
It may contain useful reference behavior for:

- motors,
- WebSocket,
- sound,
- relay/attachment control,
- older command shapes.

It must not be treated as the current architecture.
Do not move the project back to a monolithic sketch.

## Flutter app

`module_app/` is the active app.
It should provide:

- manual control screen,
- connection status,
- stop control,
- attachment on/off,
- telemetry display,
- GPS status display,
- auto screen workflow,
- route visualization,
- route upload,
- mapping/perimeter recording.

Current app truth:

- Manual control is the useful MVP.
- Auto screen is incomplete.
- Route building/sending/starting/pausing/stopping needs a complete workflow.
- GPS projection/storage helpers exist but do not prove real GPS mapping.

Known app files from current project notes:

- `cleaning_route_planner.dart`: route planning; must be converted to local meters.
- `gps_projection.dart`: projection helper; workflow incomplete.
- `map_storage.dart`: map persistence; full GPS perimeter workflow incomplete.

## Manual control flow

```text
User presses control in Flutter
  -> app sends WebSocket command
  -> ESP32 parses command
  -> motor control updates left/right target
  -> failsafe timestamp updates
  -> motors receive PWM/DIR output
  -> app shows connection/status
```

Manual stop must be available even when auto UI is present.
Attachment on/off is separate from drive motion and should default to off on startup/failsafe.

## Auto route flow target

Future intended flow:

```text
GPS perimeter or local polygon
  -> local x/y meters
  -> route planner builds snake waypoints
  -> app shows point count
  -> app sends waypoints to ESP32
  -> ESP32 stores/acks route
  -> user starts NAV
  -> ESP32 follows waypoint sequence
  -> user can pause/stop at any time
```

This is not confirmed working yet.

## Coordinate systems

Use local meters for route planning and navigation.

Expected coordinate flow:

- GPS provides lat/lon.
- User or system selects origin lat/lon.
- `gps_projection.dart` converts lat/lon to local x/y meters.
- `cleaning_route_planner.dart` operates on local x/y.
- ESP32 receives local-meter waypoints or another clearly documented coordinate format.

Do not mix degree values with meter spacing.
Do not preserve `lineStep 44.0` as a meaningful physical spacing.

## GPS role

GPS is planned but not physically tested.
The first GPS goal is basic lat/lon display.
Only after that should the project record perimeters and generate real-world routes.

RTK is not part of the current confirmed architecture.
RTK is a later precision upgrade.

## Safety architecture

Safety must be implemented at both layers:

- Flutter: visible stop, connection status, error status, no fake GPS certainty.
- ESP32: startup safe state, command timeout, disconnect stop, heartbeat/failsafe, invalid command rejection.

ESP32 must not rely only on the app to be safe.

## Do not assume

- Do not assume auto navigation is ready because route code exists.
- Do not assume app map position is real GPS.
- Do not assume GPS has been wired.
- Do not assume `sound/` code is active.
- Do not assume attachment should run during early autonomy.

