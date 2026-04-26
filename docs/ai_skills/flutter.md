# Flutter App Skill

Use this when working in `module_app/`.

## Current app truth

The app's most important current workflow is manual control over Wi-Fi/WebSocket.
Attachment on/off is part of the MVP.

The auto screen is incomplete and must not imply that autonomy is ready.

## Manual UI requirements

- Show connection state.
- Provide movement controls.
- Provide visible Stop.
- Provide attachment on/off control.
- Reflect errors or disconnected state.
- Avoid sending endless motion without refresh/timeout behavior.

## Auto UI requirements

The auto screen needs this workflow:

- Build route.
- Send route.
- Start.
- Pause.
- Stop.

The auto screen must show:

- route point count,
- route sent/not sent,
- current NAV mode,
- current waypoint index/point,
- GPS status,
- errors.

## Mapping/GPS truth

- `gps_projection.dart` exists, but GPS workflow is not complete.
- `map_storage.dart` exists, but full GPS map recording is not complete.
- Manual map recording is currently conditional/simulated, not confirmed GPS perimeter recording.
- Robot marker on auto map must not be presented as reliable GPS position yet.

## Route planner truth

- `cleaning_route_planner.dart` must use local meters.
- Old `lineStep 44.0` is wrong.
- Starting line spacing should be about `0.40` to `0.45` m.

## Flutter work checklist

- Read `../../PROTOCOL.md` before changing messages.
- Read `mapping_route.md` before route/map changes.
- Keep manual Stop visible.
- Add honest status labels for unready GPS/autonomy.
- Update `../../IMPLEMENTATION_STATUS.md` only after tests.

## Do not assume

- Do not assume GPS position is real.
- Do not assume route was sent unless protocol confirms it.
- Do not assume NAV mode from UI state alone.
- Do not assume the app can safely start autonomy before firmware safety tests.

