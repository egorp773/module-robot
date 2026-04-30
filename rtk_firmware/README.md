# ESP32 ZED-F9P RTK Pair

Two separate Arduino/PlatformIO firmwares for two ESP32 + ZED-F9P modules:

- `base`: stationary RTK base. It runs survey-in on its F9P, enables RTCM3,
  connects to the rover Wi-Fi, and sends RTCM corrections to the rover via UDP.
- `rover`: moving RTK rover. It creates the Wi-Fi AP for the iPhone/app, receives
  RTCM from the base via UDP, forwards RTCM to its F9P, and exposes GPS/RTK
  telemetry over WebSocket for the Flutter app.

## Wiring

For both ESP32 boards:

- F9P TX -> ESP32 GPIO 4
- F9P RX -> ESP32 GPIO 5
- GND common
- Stable 3.3 V power for the F9P
- Outdoor antenna placement is required for real RTK tests

## Network

Rover:

- Wi-Fi AP SSID: `RTK-Rover`
- Password: `rtk-rover-123`
- WebSocket for app: `ws://192.168.4.1:81/ws`
- RTCM UDP input: `192.168.4.1:2101`

Base:

- Connects to `RTK-Rover`
- Sends RTCM UDP packets to `192.168.4.1:2101`

Connect the iPhone to `RTK-Rover`, open the app, and use `GPS Отладка`.

## Build And Upload

On Windows PowerShell:

```powershell
cd C:\robot\module\rtk_firmware
$env:PYTHONIOENCODING='utf-8'
$env:PYTHONUTF8='1'

# Upload the stationary base ESP32
pio run -e base -j 1 -t upload

# Upload the moving rover ESP32
pio run -e rover -j 1 -t upload
```

`-j 1` is intentional on this Windows setup. Parallel PlatformIO builds were
seen hanging on the ESP32 toolchain cache.

Serial monitor:

```powershell
pio device monitor -b 115200
```

## Base Behavior

The base starts survey-in automatically:

- minimum time: `60 s`
- required mean accuracy: `5.0 m`

When survey-in becomes valid, the F9P enters TIME mode and starts emitting RTCM.
The firmware forwards RTCM messages over UDP. You can tune survey-in constants at
the top of `src/base.cpp`.

These defaults are intentionally fast for field testing. For better absolute
base accuracy later, change them back to something like `300 s / 2.0 m`.

## Rover Behavior

The rover forwards all received RTCM UDP payloads to its F9P. The app sees:

```text
GPS,<lat>,<lon>,<heading>,<fixType>,<hAccMm>
GPSDBG,<lat>,<lon>,<heightM>,<heading>,<fixType>,<carrier>,<diff>,<numSV>,<hAccMm>,<vAccMm>,<speedMps>,<pDop>,<ageMs>
RTCM,<bytesTotal>,<ageMs>
```

`carrier` is `none`, `float`, or `fixed`.

## Field Notes

- Start the base first and let survey-in finish.
- Keep the base antenna fixed. If you move it, restart survey-in.
- Then start the rover and app.
- RTK fixed requires open sky, good antennas, and continuous RTCM reception.
