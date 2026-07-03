# BNO085 heading policy

## What counts as absolute heading

- `SH2_ROTATION_VECTOR` is the primary absolute heading source.
- `SH2_GEOMAGNETIC_ROTATION_VECTOR` is the fallback absolute source.
- `SH2_GAME_ROTATION_VECTOR` is relative only. It must not seed north.
- `SH2_GYROSCOPE_CALIBRATED` is only yaw-rate for short-term prediction.

## Why game rotation vector is not north

`SH2_GAME_ROTATION_VECTOR` does not use the magnetometer. Its zero is arbitrary after power-on and can drift. It is useful for smooth motion, but it is not a world-referenced heading.

## Startup policy

The rover should not start navigation until:

- absolute heading is available,
- accuracy is good,
- the heading is stable for a short startup window,
- the estimator has been seeded from the absolute heading.

If only game rotation vector is available, the rover must block autonomy.

## Calibration and DCD

The current Adafruit BNO08x package exposes SH-2 calibration and tare functions through the low-level `sh2.h` API:

- `sh2_startCal`
- `sh2_finishCal`
- `sh2_saveDcdNow`
- `sh2_setTareNow`
- `sh2_persistTare`
- `sh2_clearTare`
- `sh2_clearDcdAndReset`

So the project does not need a fake calibration flag. The code should call the real SH-2 functions and log the return codes.

## Recommended commands

Serial monitor or WebSocket:

- `IMU_STATUS`
- `IMU_CAL_START`
- `IMU_CAL_SAVE`
- `IMU_CAL_CLEAR`
- `IMU_TARE_YAW`
- `IMU_TARE_PERSIST`

For a quick check:

1. Power on the robot.
2. Run `IMU_STATUS`.
3. Wait for `state=ABSOLUTE_OK`.
4. If state stays `RELATIVE_ONLY`, `ABSOLUTE_UNCALIBRATED`, or `MAG_DISTURBED`, do not start navigation.

## What to look for in logs

- `state=ABSOLUTE_OK`
- `source=ROTATION_VECTOR` or `GEOMAGNETIC_ROTATION_VECTOR`
- small `acc`
- stable `magNorm`
- `headingUsedByEstimator=1` only after absolute heading is ready

