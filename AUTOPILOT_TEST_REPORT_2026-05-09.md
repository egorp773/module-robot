# ESP32 Rover Autopilot Test Report - 2026-05-09

Workspace: `C:\robot\module`

## Goal

Make `rtk_firmware/src/rover.cpp` the single autonomous rover autopilot:

- phone uploads map route and sends START/PAUSE/STOP only;
- rover uses local-meter route coordinates;
- rover estimates local x/y, heading, speed, and navigation quality;
- GPS/RTK gaps degrade motion instead of causing one missed packet to stop the robot;
- long GPS/RTK loss still stops the robot.

## External References Checked

- u-blox ZED-F9P integration/interface material: `NAV-PVT` carries `fixType`, `hAcc`, `headMot`, `diffSoln`, `carrSoln`; `RXM-RTCM` is the receiver-side proof that RTCM reached and decoded inside the F9P.
- SparkFun u-blox GNSS Arduino library: use UBX instead of NMEA for robust GNSS state, and use automatic UBX navigation messages where possible.
- ROS Nav2 regulated pure pursuit: path following should use a lookahead target and speed constraints, not direct bearing to a single waypoint.
- ArduPilot rover/GPS docs: navigation must distinguish degraded GPS/EKF state from total GPS loss and apply failsafe behavior.
- OpenMower / AgOpenGPS style projects: route planning can stay offboard, but path tracking and motor decisions must run on the robot.

Practical design decision from those references: the rover must not treat "RTCM packet arrived at ESP32" as RTK quality. It needs F9P quality (`carrSoln`, `hAcc`, freshness) and ideally `UBX-RXM-RTCM`.

## Code Changes Made

### `rtk_firmware/src/rover.cpp`

- Fixed rover static IP typo: `RTK_ROVER_IP_B`, not `RTK_ROUTER_IP_B`.
- Added `Preferences` storage for rover-side nav/IMU config:
  - IMU yaw offset;
  - invert yaw;
  - invert forward;
  - invert steering;
  - forward/turn scale.
- Added `NAV_CFG` command handling from the app.
- Added proper `NAV_STOP`, `NAV_PAUSE`, `NAV_RESUME` handling.
- Changed route protocol to local meters:
  - `ROUTE_BEGIN,<count>,<originLat>,<originLon>`;
  - `ROUTE_WP,<idx>,<x_m>,<y_m>`;
  - `ROUTE_END`.
- Added route completeness checks before `NAV_START`.
- Reworked estimator:
  - local origin;
  - local x/y;
  - IMU heading;
  - short dead-reckoning without cumulative dt runaway;
  - `RTK_FIXED`, `RTK_FLOAT`, `DEGRADED`, `HOLD_SHORT`, `LOST_WAIT`.
- Replaced waypoint bearing control with segment lookahead / pure-pursuit style target selection:
  - nearest route segment;
  - lookahead point;
  - cross-track error in meters;
  - heading error;
  - speed reduction by RTK quality, heading error, final approach.
- Added UBX configuration attempts for:
  - `UBX-NAV-PVT`;
  - `UBX-RXM-RTCM`;
  - `UBX-NAV-RELPOSNED`;
  - NMEA output disable.
- Fixed RTCM message type decoding: RTCM type is `(byte3 << 4) | (byte4 >> 4)`.
- Added NMEA fallback:
  - GGA quality `4` -> RTK fixed;
  - GGA quality `5` -> RTK float;
  - HDOP-derived fallback `hAcc` when UBX is not available.
- Restored Flutter-compatible telemetry:
  - `TEL,...`;
  - `GPSDBG,...`;
  - `RTCM,...`;
  - `IMU,yaw,age,fresh`;
  - `NAV,state,wpIdx,wpTotal,dist`.

### `module_app`

- `WifiConnectionNotifier.sendRouteBegin` now requires origin lat/lon.
- `sendRouteWaypoint` now sends local meter x/y instead of lat/lon.
- `auto_map_screen.dart` sends route points directly as local meters and allows `NAV_START`.
- `gps_debug_screen.dart` converts GPS route points to local meters before rover upload.

## Test Results

### Build

Passed:

```text
cd C:\robot\module\rtk_firmware
pio run -e base
pio run -e rover
```

Passed after changes:

```text
cd C:\robot\module\module_app
flutter analyze --no-fatal-infos lib/core/wifi_connection.dart lib/features/auto/auto_map_screen.dart lib/features/gps/gps_debug_screen.dart
flutter test test/gps_navigation_test.dart test/gps_debug_map_points_test.dart test/gps_perimeter_storage_test.dart
```

Note: plain `flutter analyze` still returns info-level `withOpacity` deprecation warnings in `auto_map_screen.dart`. These are pre-existing UI warnings and not route/autopilot errors.

### Upload

Passed:

```text
pio run -e base -t upload --upload-port COM4
pio run -e rover -t upload --upload-port COM6
```

### Hardware Logs

Saved files:

- `test_results/2026-05-09_autopilot/base_COM4.log`
- `test_results/2026-05-09_autopilot/rover_COM6.log`
- `test_results/2026-05-09_autopilot/rover_COM6_after_ubx_fix.log`
- `test_results/2026-05-09_autopilot/rover_COM6_final_diag.log`

Base result:

```text
WiFi: CONNECTED
IP=192.168.31.207
SVIN valid=1
RTCM frames/pkts/bytes increasing
udpErr=0
```

Rover result after the UBX/NMEA fallback fix:

```text
WiFi: 192.168.31.222
GPS Fix: 3
Carrier: 0 (NONE)
hAcc: 510 mm from NMEA HDOP fallback
Raw increasing
UBX: 0
NMEA increasing
RTCM Fresh: 1
RTCM bytes/messages increasing
F9P decoded: msgs=0 crcFail=0
IMU Fresh: 1
Quality: DEGRADED
```

## Current Blocker

The rover is receiving GPS bytes from F9P and RTCM packets from the base, but F9P is not outputting UBX and is not reporting RTK carrier solution.

The important symptom:

```text
Raw: increasing
UBX: 0
NMEA: increasing
RTCM Fresh: 1
Carrier: 0
Quality: DEGRADED
```

This means the software path `base -> Wi-Fi -> rover ESP32` works, but there is no proof that correction data and config commands are reaching/decoding inside the rover F9P.

Most likely causes to check before changing navigation code again:

1. ESP32 rover `GPIO5` TX is not connected to F9P RX, is connected to the wrong F9P port, or has bad ground/reference.
2. F9P rover UART receiving side is not enabled for UBX+RTCM3 input.
3. The F9P port connected to ESP32 is not the configured UART1 at 38400 baud.
4. The base is sending only sparse RTCM types. In the short base log, `1005` and `1230` dominate; MSM messages are present but rare. Longer logging is needed after F9P RX is confirmed.

## Do Not Repeat These Loops

- Do not tune pure pursuit while `UBX: 0` on rover. Navigation math is not the blocker when the receiver is still NMEA-only.
- Do not trust ESP32 `RTCM Fresh: 1` as RTK-ready. It only means ESP32 got UDP RTCM. Need `UBX-RXM-RTCM` or `NAV-PVT carrSoln`.
- Do not treat `Carrier: 0` + NMEA fallback hAcc as safe autonomous RTK. It is only `DEGRADED`.
- Do not start motor tests until rover shows at least `RTK_FLOAT` or `RTK_FIXED` through UBX/NAV-PVT or clear NMEA GGA quality 4/5.
- If `RTCM Fresh: 1` but `F9P RTCM msgs=0` or `UBX=0`, check the physical ESP32 TX -> F9P RX path first.

## Next Hardware Check

Before more firmware changes:

1. Verify rover wiring:
   - ESP32 GPIO4 RX <- F9P TX works because NMEA is visible.
   - ESP32 GPIO5 TX -> F9P RX must be checked with continuity/wiring.
   - GND common.
2. In u-center, connect to the rover F9P and confirm the same UART connected to ESP32:
   - baud `38400`;
   - input protocols: UBX + RTCM3;
   - output protocols: UBX, or UBX + NMEA temporarily for debugging;
   - enable `UBX-NAV-PVT`;
   - enable `UBX-RXM-RTCM`.
3. Reboot rover and capture COM6 again.
4. Expected pass criteria:
   - `UBX` counter increases;
   - `F9P decoded: msgs=...` increases in rover status;
   - `Carrier` becomes `1` float or `2` fixed;
   - `Quality` becomes `RTK_FLOAT` or `RTK_FIXED`.

## Rewire Check - 2026-05-09 16:50

After the user reconnected wires, new logs were captured:

- `test_results/2026-05-09_autopilot/base_COM4_after_rewire.log`
- `test_results/2026-05-09_autopilot/rover_COM6_after_rewire.log`

Result:

```text
Base:
WiFi: CONNECTED
SVIN valid=1
RTCM pkts/bytes increasing
udpErr=0

Rover:
Raw increasing
UBX: 0
NMEA increasing
RTCM Fresh: 1
F9P decoded: msgs=0 crcFail=0
Carrier: 0 (NONE)
Quality: DEGRADED
```

Conclusion: the rewire did not fix the F9P input/output problem. The rover still receives NMEA from F9P and RTCM on ESP32, but the F9P does not output UBX and does not confirm RTCM decoding. Autonomous motor start remains blocked.

Next specific checks:

1. Power-cycle the rover after wiring changes, not only reconnect wires while firmware is already running.
2. Verify ESP32 `GPIO5 TX` really goes to rover F9P `RX` on the same UART that outputs NMEA to ESP32 `GPIO4 RX`.
3. Verify F9P RX voltage/pin orientation and common ground.
4. In u-center, enable UBX output and RTCM3 input on the exact F9P port connected to ESP32.

## Follow-up Code Fix - 2026-05-09

Static review found one mismatch between this report and the active code:
the report said RTCM buffers were enlarged, but `rtk_firmware/src/base.cpp`
and `rtk_firmware/src/rover.cpp` still used 512-byte RTCM buffers.

Changes made:

- base RTCM frame buffer increased to 1200 bytes;
- rover UDP RTCM packet buffer increased to 1200 bytes;
- rover now drops and reports oversize or short-read RTCM packets instead of
  forwarding truncated correction data into F9P;
- rover rejects negative `ROUTE_WP` indexes;
- immediate `NAV,ARRIVED` broadcasts now use the full
  `NAV,<state>,<wpIdx>,<wpTotal>,<dist>` shape expected by Flutter;
- `PROTOCOL.md`, `IMPLEMENTATION_STATUS.md`, and `TEST_LOG.md` were updated
  to reflect the local-meter route protocol.

Verification:

```text
pio run -e base
pio run -e rover
flutter analyze --no-fatal-infos lib/core/wifi_connection.dart lib/features/auto/auto_map_screen.dart lib/features/gps/gps_debug_screen.dart lib/core/gps_navigation.dart
flutter test test/gps_navigation_test.dart test/gps_debug_map_points_test.dart
```

All passed. Plain `flutter analyze` still reports existing info-level
`withOpacity` deprecations in `auto_map_screen.dart`.

## Hardware Follow-up - 2026-05-09

Flashed:

```text
pio run -e base -t upload --upload-port COM4
pio run -e rover -t upload --upload-port COM6
```

Logs saved under:

```text
test_results/2026-05-09_navigation_followup/
```

Important result:

```text
Rover:
UBX initially worked in one run, and after fallback changes NMEA GGA keeps GPS fresh.
RTCM Fresh: 1
Relay errors: read=0 write=0 oversize=0
F9P decoded: increasing
F9P crcFail: 0
Carrier: 0 (NONE)
Quality: DEGRADED
```

This fixes the previous hard blocker where the rover ESP32 received RTCM but
the F9P did not confirm decoding it. The ESP32 TX -> F9P RX path is now proven.

Current blocker:

```text
Base RTCM types: mostly 1005 and 1230, rare 1074/1084
Rover carrier: NONE
```

The rover F9P decodes RTCM, but the base is not providing a continuous useful
MSM observation stream. Do not start autonomous motors until rover carrier is
`float` or `fixed`.

Additional code changes from this follow-up:

- rover keeps NMEA GGA as a fallback when UBX NAV-PVT is stale;
- rover re-requests NAV-PVT/RXM-RTCM output if NAV-PVT goes stale;
- base RTCM parser attempts resync after CRC fail in the mixed UBX/RTCM stream.

## RTK Fixed Follow-up - 2026-05-09

Final change in this pass:

- After Survey-In becomes valid, the base switches the F9P UART to RTCM3-only output and forwards the raw RTCM byte stream to the rover in UDP chunks.
- The base no longer depends on the ESP32 RTCM frame parser to decide which correction frames are forwarded after `svinValid=1`.

Verification:

```text
pio run -e base
pio run -e base -t upload --upload-port COM4
```

Logs:

```text
test_results/2026-05-09_navigation_followup/base_COM4_after_raw_forward.log
test_results/2026-05-09_navigation_followup/rover_COM6_after_raw_forward.log
test_results/2026-05-09_navigation_followup/base_COM4_after_raw_forward_typecount.log
test_results/2026-05-09_navigation_followup/rover_COM6_after_raw_forward_typecount.log
```

Result:

```text
Base:
svinValid=1, svinDur=60s, svinAcc=1.292m
RTCM raw forwarding enabled
rawPkts=1847 rawBytes=145626 udpErr=0 rtcmAge=15ms
t1074=59 t1084=59 t1094=59 t1124=59

Rover:
Fix: 3
Carrier: 2 (FIXED)
hAcc: 14 mm
Quality: RTK_FIXED
RTCM relay errors: read=0 write=0 oversize=0
F9P decoded: msgs=1297 crcFail=0
```

Conclusion: RTK correction delivery is now proven on the bench and the rover reached RTK fixed. Autonomous movement is still not proven; the next test should be a controlled dry run with attachment disabled and failsafe/stop behavior monitored.
