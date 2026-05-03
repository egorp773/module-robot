# RTK/GPS rebuild plan

Цель: робот едет по маршруту автономно, телефон только загружает точки и
показывает телеметрию. RTCM идет напрямую base -> rover без зависимости от
приложения.

## Опорная архитектура

- Base ZED-F9P завершает Survey-In, отдает RTCM3, ESP32 базы передает поправки
  напрямую роверу.
- Rover ESP32 принимает RTCM, сразу пишет их в UART F9P и отдельно проверяет,
  что F9P реально декодирует RTCM через UBX-RXM-RTCM.
- Rover сам считает позицию, курс, отклонение от траектории и команды моторов.
- Phone/WebSocket не участвует в управлении движением и восстановлении RTCM.

## Источники

- u-blox ZED-F9P Integration Manual: RTK base/rover, Survey-In, RTCM output.
- u-blox ZED-F9P Interface Description: NAV-PVT carrSoln/diffSoln,
  RXM-RTCM status.
- Espressif Wi-Fi docs: отключение power save для минимальной задержки.
- BNO08x/BNO085 datasheet: Rotation Vector vs Game Rotation Vector.
- CMU Pure Pursuit и Stanford Stanley: tracking пути по lookahead/cross-track.

## Найденные минусы

- RTCM age смешивает возраст транспорта и возраст RTCM, принятый F9P. Это
  мешает понять, где именно зависло: Wi-Fi, ESP32 или F9P UART.
- В незавершенной TCP-правке базы один `write()` может отправить часть RTCM
  кадра. Такой кадр нельзя считать доставленным.
- Телефон сейчас может отправлять `UDP_RESET` при старом RTCM. Это ломает идею
  независимого ровера и может создавать лишние reconnect/reset циклы.
- UART GNSS на 38400 потенциально узкий для RTCM MSM + UBX. Миграция на более
  высокий baud нужна отдельным безопасным шагом.
- Навигация слишком резко стопорится на кратком провале RTK. Нужен state
  machine с коротким hold/degrade, но без слепого движения при долгой потере.
- IMU offset задается приложением. Дальше его надо хранить и калибровать на
  ровере.

## Этап 1: стабильный RTCM exchange

- TCP primary base -> rover.
- UDP unicast fallback, broadcast только редкий recovery fallback.
- TCP кадр считается успешным только если отправлен полностью.
- Rover не пишет дубликаты UDP в F9P, если TCP свежий.
- Телефон не сбрасывает RTCM transport.
- Telemetry отдельно показывает:
  - effective RTCM age;
  - transport RTCM age;
  - F9P decoded RTCM age;
  - source tcp/udp;
  - F9P RTCM count, crcFail, lastType.

## Этап 2: доказательные тесты

- `pio run -e base`
- `pio run -e rover`
- На железе: база COM4, ровер COM6, телефон можно отключить после `NAV_START`.
- 10 минут наблюдения: `transportAge < 2000ms`, `f9pAge < 3000ms`,
  `carrSoln=fixed/float`, без роста `crcFail`.

## Этап 3: навигация

- Ввести RTK state machine: RUNNING_FIXED, RUNNING_FLOAT_SLOW, HOLD_RTK_SHORT,
  WAIT_RTK.
- Управление по BNO085 yaw + RTK position; GPS heading не использовать как
  heading робота.
- Pure pursuit lookahead + cross-track correction.
- Drift/yaw correction только по устойчивому RTK motion vector.
- Persist IMU offset на ровере.

## Текущий старт

Первым делом реализуется этап 1, потому что без стабильного RTCM любые правки
навигации будут маскировать реальную проблему.
