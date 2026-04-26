# Hardware

This file records hardware truth and TODOs.
Do not invent pin values. Fill TODOs only from the real build or code confirmed against wiring.

## Robot

Type: MVP modular robotic snow-cleaner / outdoor cleaner.

Drive: differential/tank-style drive is assumed from left/right motor references, but exact hardware must be confirmed.

Working attachment: cleaning nozzle/tool controlled from app, likely through relay or output, exact wiring TODO.

## Controller

- ESP32 board: TODO
- ESP32 power input: TODO
- Logic level: TODO, likely 3.3 V for ESP32 IO
- Wi-Fi mode: TODO, confirm AP/client behavior from firmware
- USB/programming port: TODO

## Motor driver

- Motor driver model: TODO
- Motor voltage: TODO
- Motor current rating: TODO
- Left motor PWM pin: TODO
- Left motor direction pin(s): TODO
- Right motor PWM pin: TODO
- Right motor direction pin(s): TODO
- Enable/standby pin: TODO
- Common ground with ESP32: TODO, must be confirmed

## Attachment / relay / output

- Attachment type: TODO
- Relay/module model: TODO
- Relay GPIO: TODO
- Active level: TODO
- Default boot state: TODO
- Failsafe state: TODO, should be off
- Power source: TODO

## GPS

GPS has not been physically connected or tested.

- GPS module model: TODO
- GPS interface: TODO, expected UART unless hardware says otherwise
- GPS UART TX pin to ESP32 RX: TODO
- GPS UART RX pin from ESP32 TX: TODO
- GPS baud rate: TODO
- GPS power voltage: TODO
- Antenna placement: TODO
- Fix LED/status behavior: TODO

## IMU

IMU is part of the architecture but not confirmed in the current honest state.

- IMU model: TODO
- IMU interface: TODO, likely I2C
- IMU SDA pin: TODO
- IMU SCL pin: TODO
- IMU I2C address: TODO
- IMU power voltage: TODO
- Mount orientation: TODO

## Power

- Battery type/cell count: TODO
- Battery voltage range: TODO
- Motor power rail: TODO
- ESP32 regulator/DC-DC: TODO
- Attachment power rail: TODO
- Fuse/protection: TODO
- Emergency physical switch: TODO
- Voltage measurement input: TODO

## Pin table

| Function | GPIO | Direction | Component | Status |
| --- | --- | --- | --- | --- |
| Left motor PWM | TODO | output | motor driver | TODO |
| Left motor DIR A | TODO | output | motor driver | TODO |
| Left motor DIR B | TODO | output | motor driver | TODO |
| Right motor PWM | TODO | output | motor driver | TODO |
| Right motor DIR A | TODO | output | motor driver | TODO |
| Right motor DIR B | TODO | output | motor driver | TODO |
| Motor enable/standby | TODO | output | motor driver | TODO |
| Attachment relay | TODO | output | relay/tool | TODO |
| GPS RX on ESP32 | TODO | input | GPS TX | TODO |
| GPS TX on ESP32 | TODO | output | GPS RX | TODO |
| IMU SDA | TODO | bidirectional | IMU | TODO |
| IMU SCL | TODO | output | IMU | TODO |
| Battery voltage sense | TODO | input | divider/ADC | TODO |

## Hardware risks

- Motor startup current can brown out ESP32.
- Motor noise can disturb GPS/IMU.
- Missing common ground can break motor control.
- Reversed motor polarity can invert navigation.
- Relay active-low/active-high confusion can enable attachment unexpectedly.
- GPS antenna placement can prevent fix.
- Outdoor snow/water exposure can short electronics.

## Do not assume

- Do not assume pins from old `sound/` firmware are current.
- Do not assume GPS UART pins before wiring is confirmed.
- Do not assume IMU exists on the current build.
- Do not assume relay active level.
- Do not assume power rails are safe for motors and ESP32 together.

