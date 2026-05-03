# RTK / GPS / Robot Current State

Дата: 2026-05-02  
Рабочая папка: `C:\robot\module`

## Коротко

Система сейчас состоит из двух ESP32 + двух u-blox ZED-F9P:

- база: `COM4`, IP `192.168.31.207`;
- ровер: `COM6`, IP `192.168.31.222`;
- Wi-Fi: `Xiaomi_6A92`;
- UDP RTCM: база -> ровер `192.168.31.222:2101`;
- broadcast RTCM: база -> `192.168.31.255:2101`;
- TCP RTCM: ровер подключается к базе `192.168.31.207:2102`;
- приложение Flutter получает GPS/RTK/IMU/моторы через WebSocket ровера `ws://192.168.31.222:81/ws`.

Главная проблема, которую отлаживали: ровер иногда терял свежий RTCM, приложение зависало по RTK age, а управление ехало не туда из-за некалиброванного IMU yaw.

## Что изменено в прошивках

### `rtk_firmware/src/base.cpp`

Добавлено:

- отправка RTCM не только unicast на `192.168.31.222:2101`, но и broadcast на `192.168.31.255:2101`;
- TCP RTCM server на порту `2102`;
- статический IP базы `192.168.31.207`;
- логирование:
  - `tcpClient`;
  - `tcpOk`;
  - `tcpFail`;
  - `bcastOk`;
  - `bcastFail`;
  - `udpOk`;
  - `udpFail`.

Важно: TCP server сначала был запущен слишком рано и база уходила в reboot loop с ошибкой:

```text
assert failed: tcpip_send_msg_wait_sem ... Invalid mbox
```

Это исправлено: теперь сначала вызывается `connectWiFi()`, потом `rtcmTcpServer.begin()`.

### `rtk_firmware/src/rover.cpp`

Добавлено/изменено:

- GPS UART зафиксирован на `38400`, автоперебор `9600/115200` убран;
- GPS работает только на `GPIO4/GPIO5`;
- RTCM UDP reader читает пакеты без обрезания, буфер увеличен до `1200`;
- ровер подключается к TCP RTCM server базы `192.168.31.207:2102`;
- если TCP доступен, RTCM берётся из TCP;
- UDP остаётся запасным каналом;
- RTCM watchdog ускорен: stale threshold уменьшен до `5000 ms`;
- добавлен `rtcmSrc=tcp/udp/none`;
- добавлены счётчики:
  - `tcpReconnect`;
  - `tcpBytes`;
  - `tcpReads`;
  - `udpRestart`;
- BNO085 переключён на `SH2_GAME_ROTATION_VECTOR`;
- IMU watchdog пере-включает game rotation vector, если yaw stale;
- убрана повторная reconfigure-проверка F9P, которая могла мешать UART/RTCM.

## Что изменено в Flutter

Файлы:

- `module_app/lib/core/gps_navigation.dart`
- `module_app/lib/features/gps/gps_debug_screen.dart`
- `module_app/test/gps_navigation_test.dart`

Добавлено:

- `HeadingCalibration`:
  - offset градусов;
  - invert yaw;
  - выравнивание raw IMU yaw на азимут к выбранной цели;
- настройки моторов:
  - invert forward;
  - invert steering;
  - forward percent;
  - turn percent;
- сохранение настроек навигации через `SharedPreferences`;
- кнопка калибровки IMU в GPS Debug;
- переключатели:
  - `Invert IMU yaw`;
  - `Invert forward`;
  - `Invert steering`;
- навигация теперь использует BNO085 yaw с offset, если IMU есть;
- если IMU нет, fallback на GPS heading;
- добавлены тесты на heading calibration и motor inversion.

Проверки прошли:

```text
flutter analyze lib/core/gps_navigation.dart lib/features/gps/gps_debug_screen.dart test/gps_navigation_test.dart
No issues found

flutter test test/gps_navigation_test.dart
All tests passed
```

## Последние важные логи

### База после разделения TCP/UDP портов

База работает стабильно:

```text
BASE wifi=connected ip=192.168.31.207 rover=192.168.31.222:2101 bcast=192.168.31.255:2101 tcpClient=1 tcpOk=257 tcpFail=0 ... svin_valid=1 ... rtcm=54927bytes/556pkts udpOk=556 udpFail=0 bcastOk=556 bcastFail=0
```

Это значит:

- база подключена к Wi-Fi;
- Survey-In завершён: `svin_valid=1`;
- база шлёт RTCM;
- UDP и broadcast отправка без fail;
- TCP client подключен;
- TCP RTCM уходит без fail.

### Ровер после фикса GPS baud

Ровер вернулся на правильный GPS baud:

```text
port=GPIO4/GPIO5 baud=38400 src=NMEA parsed=...
```

TCP RTCM приходил:

```text
rtcmSrc=tcp rtcm=997bytes/5pkts
```

Был RTK float/fixed в разные моменты:

```text
carrier=float hAcc=28mm
```

Ранее после UDP/TCP восстановления был зафиксирован хороший результат:

```text
carrier=fixed hAcc=14mm rtcmSrc=tcp age=832ms imu=ok
```

После последней прошивки IMU тоже ожил:

```text
imu=ok yaw=0.16 age=2ms
```

## Текущая нестабильность

TCP RTCM стал намного лучше после разделения портов:

- UDP: `2101`;
- TCP: `2102`.

Но до прерывания ещё оставался риск:

- ровер иногда сбрасывал TCP соединение;
- база писала:

```text
write(): fail ... Connection reset by peer
WARN base TCP RTCM failed written=0/158
```

После этого ровер переподключался, база снова видела:

```text
RTCM TCP client connected from 192.168.31.222
tcpClient=1 tcpOk растёт
```

То есть восстановление есть, но нужно ещё понаблюдать 2-5 минут, чтобы понять, исчезли ли долгие провалы RTCM после последнего фикса.

## Что прошито сейчас

Прошиты обе платы:

- база на `COM4`;
- ровер на `COM6`.

Последние команды, которые успешно выполнялись:

```powershell
cd C:\robot\module\rtk_firmware
pio run -e base -t upload --upload-port COM4
pio run -e rover -t upload --upload-port COM6
```

## Как проверить сейчас

### Проверка базы

```powershell
cd C:\robot\module\rtk_firmware
pio device monitor -p COM4 -b 115200
```

Нормально, если видно:

```text
wifi=connected
svin_valid=1
rtcm растёт
udpOk растёт
bcastOk растёт
tcpClient=1
tcpOk растёт
tcpFail не растёт постоянно
```

### Проверка ровера

```powershell
cd C:\robot\module\rtk_firmware
pio device monitor -p COM6 -b 115200
```

Нормально, если видно:

```text
wifi=connected
port=GPIO4/GPIO5
baud=38400
src=NMEA или src=UBX
parsed растёт
rtcmSrc=tcp
rtcm age 0-1500ms
carrier=fixed
hAcc примерно 14-30mm
imu=ok
imu age меньше 1000ms
```

Плохо, если видно:

```text
baud=115200
src=none
parsed=0
rtcm age растёт больше 5000ms
imu=stale
```

## Команды сборки/прошивки

### База

```powershell
cd C:\robot\module\rtk_firmware
pio run -e base
pio run -e base -t upload --upload-port COM4
```

### Ровер

```powershell
cd C:\robot\module\rtk_firmware
pio run -e rover
pio run -e rover -t upload --upload-port COM6
```

### Flutter проверки

```powershell
cd C:\robot\module\module_app
flutter analyze lib/core/gps_navigation.dart lib/features/gps/gps_debug_screen.dart test/gps_navigation_test.dart
flutter test test/gps_navigation_test.dart
```

## Как калибровать IMU перед ездой

1. Открыть GPS Debug в приложении.
2. Выбрать цель маршрута.
3. Физически поставить нос робота точно в сторону выбранной цели.
4. Нажать `Калибр IMU`.
5. Сначала моторы не включать, проверить:
   - азимут на цель;
   - курс;
   - ошибка курса.
6. Если ошибка уменьшается при повороте носа к цели, IMU направление правильное.
7. Если ошибка растёт в обратную сторону, включить `Invert IMU yaw` и снова нажать `Калибр IMU`.
8. Если робот едет назад вместо вперёд, включить `Invert forward`.
9. Если поворачивает не в ту сторону, включить `Invert steering`.

Настройки сохраняются в приложении и не должны пропадать после перезапуска.

## Важная инженерная заметка

BNO085 yaw не является “севером” сам по себе. Это локальный угол корпуса/датчика. Поэтому без калибровки offset робот может ошибаться на 90-180 градусов. Приложение теперь делает правильную вещь: берёт raw yaw от BNO085 и добавляет offset, выставленный по направлению носа на выбранную цель.

## Что делать дальше

1. Не менять прошивки сразу.
2. Включить оба монитора или приложение.
3. Смотреть 2-5 минут:
   - на базе `tcpClient=1`, `tcpOk` растёт;
   - на ровере `rtcmSrc=tcp`, `rtcm age` держится меньше `1500ms`;
   - `carrier=fixed`;
   - `hAcc` около `14-30mm`;
   - `imu=ok`, age маленький.
4. Если RTK снова пропадает:
   - сначала посмотреть, растёт ли `tcpFail` на базе;
   - потом посмотреть, переподключается ли `tcpReconnect` на ровере;
   - если `rtcmSrc=tcp`, но `f9pRtcm=0`, проверить физическую линию ESP32 TX GPIO5 -> F9P RX.
5. Если робот едет не туда:
   - не править математику сразу;
   - сначала сделать IMU calibration;
   - потом проверить `Invert steering`;
   - потом `Invert forward`.

## Git состояние

Последние известные локальные изменения не были закоммичены после этой отладки.

Изменены как минимум:

- `rtk_firmware/src/base.cpp`
- `rtk_firmware/src/rover.cpp`
- `rtk_firmware/src/rtk_config.example.h`
- `module_app/lib/core/gps_navigation.dart`
- `module_app/lib/features/gps/gps_debug_screen.dart`
- `module_app/test/gps_navigation_test.dart`

Перед коммитом нужно выполнить:

```powershell
cd C:\robot\module
git status --short
git diff -- rtk_firmware/src/base.cpp rtk_firmware/src/rover.cpp rtk_firmware/src/rtk_config.example.h module_app/lib/core/gps_navigation.dart module_app/lib/features/gps/gps_debug_screen.dart module_app/test/gps_navigation_test.dart
```

Рекомендуемый коммит:

```powershell
git add rtk_firmware/src/base.cpp rtk_firmware/src/rover.cpp rtk_firmware/src/rtk_config.example.h module_app/lib/core/gps_navigation.dart module_app/lib/features/gps/gps_debug_screen.dart module_app/test/gps_navigation_test.dart RTK_CURRENT_STATE.md
git commit -m "Stabilize RTK transport and add IMU calibration controls"
```

