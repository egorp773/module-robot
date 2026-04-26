# Mapping And Route Skill

Use this for map recording, route planning, snake paths, waypoint generation, or auto UI route display.

## Current truth

Mapping and route planning are not product-ready.
Manual map recording is mostly conditional/simulated.
It is not a full GPS perimeter recording workflow.

The current snake route is poor.
The old `lineStep 44.0` value is wrong.

## Required coordinate model

Use local meters.

Expected flow:

- GPS lat/lon with valid fix.
- Select local origin.
- Convert lat/lon to local x/y meters.
- Record perimeter in local x/y.
- Generate snake route in local x/y.
- Send local-meter waypoints or a clearly documented format to ESP32.

## Route planner requirements

- `cleaning_route_planner.dart` should operate in meters.
- Starting line spacing should be around `0.40` to `0.45` m.
- Route output should be inspectable.
- Route UI should show point count.
- Route UI should show whether route was sent.

## Auto UI requirements

- Build route.
- Send route.
- Start.
- Pause.
- Stop.
- Show NAV mode.
- Show current waypoint.
- Show GPS status.
- Show errors.

## Do not assume

- Do not assume map marker is real GPS.
- Do not assume route is safe to drive.
- Do not assume `lineStep 44.0` is meaningful.
- Do not assume GPS perimeter recording exists.

