# Wiring and power boundary

Disconnect the traction battery before changing wiring. This document records
the software-side pin contract; verify every pin against the physical loom and
board labels before applying power.

## Topology

```text
checked DC-DC 5.1 V           ----> Raspberry Pi USB-C power
Raspberry Pi USB-A           ----> ESP32 USB data/power interface
ESP32 UART2                  ----> hoverboard motor controller UART
ESP32 UART1                  <----> ZED-F9P UART
ESP32 I2C                    <----> BNO085
ESP32 GPIO                   ----> attachment/mount relay inputs
```

Code-confirmed legacy pin definitions reused by `pi_bridge` are:

| Function | ESP32 pin | Direction |
| --- | ---: | --- |
| F9P TX -> ESP32 UART1 RX | GPIO 4 | input |
| ESP32 UART1 TX -> F9P RX | GPIO 5 | output |
| hoverboard TX -> ESP32 UART2 RX | GPIO 16 | input |
| ESP32 UART2 TX -> hoverboard RX | GPIO 17 | output |
| BNO085 SDA | GPIO 21 | bidirectional |
| BNO085 SCL | GPIO 22 | clock output |
| attachment relay command | GPIO 32 | output |
| mount relay command | GPIO 33 | output |

The hoverboard UART is currently 115200 baud. F9P initialization may begin at
38400 and the reused GNSS driver moves it to 115200. These facts come from the
existing firmware, not from a new wiring assumption. Confirm ESP32 board pin
labels and voltage levels physically.

Protocol v1 reports `RXM-RTCM` age but does not define a Pi-to-ESP `RTCM_DATA`
message. Therefore RTK FLOAT/FIXED requires that the F9P already receive
corrections through a separate proven hardware/radio port. If corrections are
supposed to originate on the Pi, design a bounded, CRC-protected protocol-v2
transport before Gate 9 autonomy; the removed legacy Wi-Fi RTCM path is not active.

## UART crossing

UART signals cross:

```text
ESP32 TX -> peripheral RX
ESP32 RX <- peripheral TX
GND      -- peripheral GND
```

Do not connect a 5 V UART output directly to ESP32 GPIO. Verify that F9P and
hoverboard UART logic levels are 3.3 V compatible; use a proper level shifter if
they are not. Never connect motor power to a GPIO or USB pin.

## Power rules

- Do not power Raspberry Pi from the ESP32 regulator or 5 V pin.
- Do not tie outputs of two independent 5 V supplies together.
- Determine whether the ESP32 USB connector and its external 5 V/VIN input are
  electrically common before connecting both. A standard USB cable carries
  5 V as well as data; prevent back-feeding with a board-approved power design,
  not by cutting grounds or improvising signal wiring.
- Check DC-DC output with a multimeter before connecting the Pi: polarity,
  approximately 5.1 V at no load, and acceptable loaded voltage/ripple.
- Raspberry Pi, ESP32, sensors and motor-controller logic require a deliberate
  common signal ground. Design it to avoid motor-current return through USB or
  thin logic wiring.
- Keep high-current motor wiring separated from GNSS antenna/I2C/UART wiring.
- Fuse traction and logic branches appropriately; size wire and connectors for
  real current, not average current.
- Do not feed 5 V into ESP32 signal GPIO.
- Relays must default physically safe. `pi_bridge` drives both relay commands
  inactive at boot, but active polarity and relay-board behavior still require
  a meter test with attachment power disconnected.

## Bring-up order

1. Traction and attachment power disconnected: validate DC-DC voltage/polarity.
2. Power Pi alone; check throttling/undervoltage logs.
3. Connect ESP32 USB only; repeat checks and observe reset reason/brownouts in
   binary `STATUS`/`DIAGNOSTICS` through the bridge (USB UART0 is not a text console).
4. Connect BNO085 and F9P; verify sensor status without motors.
5. Connect hoverboard UART with traction output mechanically safe; verify zero
   frames and feedback before ARM.
6. Connect relay inputs with the load power isolated; prove reboot/reconnect
   leaves them off.
7. Apply motor power with tracks lifted and a physical disconnect ready.

Any Pi undervoltage, USB churn, ESP32 brownout or unexplained relay pulse fails
Gate 1. Fix power integrity before debugging software.
