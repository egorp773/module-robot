# GPS And RTK Skill

Use this for GPS wiring, GPS telemetry, local projection, perimeter recording, or RTK planning.

## Current truth

GPS has not been physically connected and tested.
RTK has not been tested in the field.

GPS-related code may exist, but code does not prove hardware operation.

## GPS development order

1. Document GPS model.
2. Document UART pins and baud.
3. Wire GPS.
4. Confirm raw serial data.
5. Parse fix/no-fix.
6. Parse lat/lon.
7. Display lat/lon in the app.
8. Select origin.
9. Convert to local x/y.
10. Record GPS perimeter.

## RTK development order

RTK comes only after ordinary GPS works.

Do not design the main workflow around RTK until:

- normal GPS fix is visible,
- local x/y works,
- perimeter recording works,
- basic route generation works.

## App behavior

- Show no-fix clearly.
- Show lat/lon only when valid.
- Show accuracy/status if available.
- Do not present simulated map position as GPS.

## Hardware TODOs

- GPS module model.
- GPS UART TX/RX pins.
- GPS baud.
- GPS power.
- Antenna placement.
- RTK module/base/corrections later.

## Do not assume

- Do not assume GPS works because `gps_projection.dart` exists.
- Do not assume RTK accuracy.
- Do not assume outdoor fix indoors.
- Do not assume local x/y before origin is set.

