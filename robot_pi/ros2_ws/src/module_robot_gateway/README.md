# module_robot_gateway

This package is the stage-1 compatibility boundary between the existing Flutter
WebSocket client and ROS 2. It never opens a serial device, never calls an ESP32
ARM service, and never publishes `/cmd_vel_safe`. All manual motion is published
as `geometry_msgs/TwistStamped` on `/cmd_vel_manual` and therefore still passes
through `module_robot_safety`.

## Safety behaviour

- `M,<left>,<right>` accepts finite percentages in `[-100, 100]`. It uses a
  configurable differential mapping and conservative linear/angular clamps:
  `v = (left + right) / 2 * max_linear` and
  `omega = (right - left) / 2 * max_angular` after normalization/deadband.
- `STOP`, `NAV_PAUSE`, `NAV_STOP`, a manual-command timeout, and every client
  disconnect immediately publish a zero Twist and request `/safety/stop`
  (`std_srvs/Trigger`). Zero publication does not depend on service availability.
- `NAV_START` and `NAV_RESUME` return `AUTO_NOT_READY` in stage 1. The
  `autonomous_start_enabled` parameter is deliberately non-operative.
- Client send queues and inbound WebSocket messages are bounded. Telemetry is
  rate-limited and old telemetry snapshots are dropped before command replies.
- The `/ws` endpoint emits the legacy `STATE,CONNECTED` readiness token and
  answers Flutter's zero-motion `PING` probes with `PONG`.

The legacy protocol has no authentication. Bind it only to the robot's trusted
local network and enforce host firewall/network isolation; Internet exposure is
not supported. ESP32 arming remains an explicit, separate operator operation.

## Route compatibility

The required format is `ROUTE_BEGIN,<N>`, followed by
`ROUTE_WP,<index>,<latitude>,<longitude>` and `ROUTE_END`. The validated path is
expressed in a local tangent-plane approximation whose origin is the first GPS
waypoint.

The currently deployed Flutter coordinate form is also accepted:
`ROUTE_BEGIN,<N>,<origin_lat>,<origin_lon>` selects legacy-local mode, where each
`ROUTE_WP` contains local X/Y metres. The supplied origin is retained and points
are also converted to latitude/longitude for a canonical stored route. Neither
upload form starts motion. Routes are held in memory for stage 1 and published
on `/route/path`; validity is latched on `/route/valid`. Legacy boundary and
forbidden-zone extensions are intentionally outside this stage-1 command set.

Validation covers finite numeric values, geographic bounds, count/index bounds,
duplicate indices and points, completeness, upload ownership, and minimum and
maximum segment lengths.

## ROS interfaces

Published:

- `/cmd_vel_manual` (`geometry_msgs/TwistStamped`)
- `/route/path` (`nav_msgs/Path`)
- `/route/valid` (`std_msgs/Bool`, transient local)

Used:

- `/safety/stop` (`std_srvs/Trigger`)
- `/gps/fix`, `/imu/data_raw`, `/rtk/status`, `/power/status`,
  `/relay/status`, `/esp32/status`, `/esp32/fault_event`, `/safety/state`

Telemetry uses bounded per-client queues and includes legacy-compatible GPS,
GPSDBG, IMU, BAT_PCT, and NAV records plus explicit RTK, power, relay, robot,
fault, route, and IMU-summary records.

## Configuration mapping

`config/gateway.yaml` mirrors all gateway-related keys in the repository-level
`robot_pi/config/network.yaml`: bind address, port/path, client/message/queue
limits, telemetry rate, route limits, stage-1 autonomy switch, hostname, and the
no-credentials/no-cloud policy. Package-only motion, route-conversion, timeout,
and frame parameters are appended in `gateway.yaml`. Keep mirrored values equal
when changing deployment configuration.
