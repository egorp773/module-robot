# MVP: Автономный робот-снегоуборщик — Полный план

## Контекст

Модульный робот на гусеницах с насадкой снегоуборщика. Цель MVP — робот автономно покрывает заданную территорию змейкой без пропусков. Пользователь объезжает границы вручную, приложение записывает GPS-точки и строит полигон. Маршрут змейки отправляется роботу через WiFi WebSocket, робот следует по точкам с PID-навигацией.

**Что уже есть:**
- ESP32 прошивка (`sound/sound.ino`): WiFi AP + WebSocket, управление моторами гироскутера по UART, батарея, реле, звук
- Flutter-приложение: ручное управление, рисование зон на сетке (в пиксельных координатах-ячейках), сохранение карт, алгоритм змейки
- Аппаратура: два модуля u-blox ZED-F9P-04B-01, BNO085 IMU

**Проблема:** Карты сейчас в условных ячейках, нет GPS, нет IMU, нет автономной навигации. Нужно всё связать.

---

## 1. Распиновка и подключение железа

### u-blox ZED-F9P-04B-01 (rover на роботе)

```
Левая сторона (сверху вниз):        Правая сторона (сверху вниз):
GND                                  5V
3V3                                  3V3
SDA                                  TX2
SCL                                  RX2
FENCE                                CS
RTK                                  RX/MOSI
PPS                                  TX/MISO
RST                                  SCK
SAFE
INT
```

### Назначение пинов ESP32

| Модуль | Интерфейс | ESP32 пин | F9P/BNO пин | Примечание |
|--------|-----------|-----------|-------------|------------|
| F9P GPS (rover) | UART1 | GPIO4 (RX) ← | TX/MISO | ESP принимает данные от F9P |
| F9P GPS (rover) | UART1 | GPIO5 (TX) → | RX/MOSI | ESP отправляет RTCM на F9P |
| F9P GPS | Питание | 5V | 5V | Питание от 5V ESP32 |
| F9P GPS | GND | GND | GND | Общая земля |
| Моторы гироскутер | UART2 | GPIO16 (RX), GPIO17 (TX) | — | Без изменений (уже работает) |
| BNO085 IMU | I2C | GPIO21 (SDA), GPIO22 (SCL) | SDA, SCL | Стандартные I2C пины |
| BNO085 IMU | Питание | 3V3 | 3V3 | |
| BNO085 IMU | INT | GPIO23 (опционально) | INT | Прерывание готовности данных |
| Звук MAX98357A | I2S | GPIO25, GPIO26, GPIO27 | — | Без изменений |
| Реле ATTACHMENT | GPIO | GPIO32 | — | Без изменений |
| Реле MOUNT | GPIO | GPIO33 | — | Без изменений |

**Важно по F9P:**
- Используем пины TX/MISO и RX/MOSI — это UART1 модуля F9P (основной порт данных)
- TX2/RX2 — это UART2 модуля F9P, можно использовать для RTCM от базы (альтернатива WiFi UDP)
- PPS — импульс 1 Гц, можно подключить для точной синхронизации (не обязательно для MVP)
- RTK — светодиод статуса RTK (не подключаем)

### RTCM от базовой станции

Базовый F9P подключён к отдельному ESP32 (или RPi), который отправляет RTCM поправки по WiFi UDP на робота (порт 2101). Робот-ESP32 принимает UDP пакеты и пересылает их на F9P rover через UART1 TX (GPIO5).

---

## 2. Структура прошивки ESP32

Новый проект `firmware/` (модульная структура):

```
firmware/
  firmware.ino          — setup(), loop(), подключение всех модулей
  config.h              — Все пины, константы, параметры PID
  motors.h / motors.cpp — Протокол гироскутера, ramping, failsafe (из sound.ino)
  gps.h / gps.cpp       — UBX-NAV-PVT парсер, RTCM relay через UDP
  imu.h / imu.cpp       — BNO085 I2C, чтение heading (yaw)
  nav.h / nav.cpp        — Waypoint follower, PID контроллер, машина состояний
  websocket.h / websocket.cpp — WiFi AP, WebSocket сервер, парсер команд
  sound.h / sound.cpp   — I2S + LittleFS WAV (из sound.ino)
  telemetry.h / telemetry.cpp — Трансляция GPS/IMU/NAV/батареи в приложение
  data/                 — WAV файлы (скопировать из sound/data/)
```

### config.h — Ключевые константы

```cpp
// Пины
#define PIN_GPS_RX        4     // ESP32 RX ← F9P TX/MISO
#define PIN_GPS_TX        5     // ESP32 TX → F9P RX/MOSI
#define PIN_MOTOR_RX      16    // ESP32 RX ← Мотор
#define PIN_MOTOR_TX      17    // ESP32 TX → Мотор
#define PIN_IMU_SDA       21
#define PIN_IMU_SCL       22
#define PIN_I2S_DIN       27
#define PIN_I2S_BCLK      26
#define PIN_I2S_LRCK      25
#define PIN_RELAY_ATTACH  32
#define PIN_RELAY_MOUNT   33

// UART
#define GPS_BAUD          38400   // F9P по умолчанию
#define MOTOR_BAUD        115200

// WiFi
#define WIFI_SSID         "Robot"
#define WIFI_PASS         "CHANGE_ME_MIN_8_CHARS"
#define WS_PORT           81
#define RTCM_UDP_PORT     2101

// Навигация
#define NAV_UPDATE_MS     100     // PID цикл 10 Гц
#define GPS_BROADCAST_MS  200     // Отправка GPS в приложение 5 Гц
#define ARRIVAL_RADIUS    0.15    // Радиус достижения точки, метры
#define MAX_WAYPOINTS     500
#define NOMINAL_SPEED     30      // Базовая скорость (-100..100)
#define MIN_GPS_ACC_MM    500     // Макс. допустимая погрешность GPS, мм

// PID
#define PID_KP_HEADING    2.0
#define PID_KI_HEADING    0.1
#define PID_KD_HEADING    0.5
```

### firmware.ino — Главный цикл

```
setup():
  Serial.begin(115200)          // отладка USB
  motors_init()                 // Serial2 GPIO16/17
  gps_init()                    // Serial1 GPIO4/5 + UDP 2101
  imu_init()                    // I2C GPIO21/22
  sound_init()                  // I2S + LittleFS
  websocket_init()              // WiFi AP + WS :81
  nav_init()                    // Сброс PID

loop():
  websocket_cleanup()
  gps_update()                  // Парсинг UBX байтов (неблокирующий)
  imu_update()                  // Чтение BNO085 heading
  rtcm_relay()                  // UDP → Serial1 TX
  nav_update()                  // PID → g_targetLeft/Right (только в NAV_RUNNING)
  motors_update_ramp()          // Плавное изменение скорости
  motors_send()                 // Отправка на мотор-контроллер
  motors_receive_feedback()     // Обратная связь от мотора
  telemetry_update()            // GPS/IMU/NAV/Battery → WebSocket
```

### gps.cpp — Парсер UBX-NAV-PVT

Конечный автомат для парсинга бинарного протокола UBX:
- Синхронизация: 0xB5 0x62
- Class 0x01, ID 0x07 (NAV-PVT), длина 92 байта
- Извлекаемые поля:
  - `byte[24-27]`: lat (int32, масштаб 1e-7 градусов)
  - `byte[20-23]`: lon (int32, масштаб 1e-7 градусов)
  - `byte[60-63]`: headMot (int32, масштаб 1e-5 градусов)
  - `byte[20]`: fixType (0=нет, 3=3D, 4=GNSS+DR)
  - `byte[21]`: flags (bit1 = diffSoln → RTK)
  - `byte[40-43]`: hAcc (uint32, мм)

При старте отправляет UBX-CFG-VALSET для настройки F9P:
- Включить UBX-NAV-PVT на UART1 с частотой 5 Гц
- Отключить NMEA (не нужен)

### imu.cpp — BNO085

Используем библиотеку Adafruit BNO08x:
- Запрос `SH2_ROTATION_VECTOR` на 50 Гц
- Извлечение yaw из кватерниона: `yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy² + qz²))`
- Результат в градусах 0-360 (0=север, по часовой стрелке)
- Фузия с GPS heading: IMU для быстрых изменений, GPS для коррекции дрифта

### nav.cpp — PID навигация по точкам

Машина состояний:
```
NAV_IDLE → (NAV_START) → NAV_RUNNING → (достигли все точки) → NAV_DONE
                            ↕ (NAV_PAUSE/RESUME)
                            → (NAV_STOP / ошибка GPS) → NAV_IDLE / NAV_ERROR
```

PID-контроллер:
```
bearingToWP = atan2(east_diff, north_diff)  // целевой курс
headingError = normalize(bearingToWP - currentHeading)  // [-180, +180]

steerCorrection = Kp * headingError + Ki * integral + Kd * derivative

leftSpeed  = baseSpeed - steerCorrection
rightSpeed = baseSpeed + steerCorrection
```

Модуляция скорости:
- Снижение при большом отклонении курса: `baseSpeed *= cos(headingError)`
- Снижение при приближении к точке (последний 1 м)
- Стоп при `distance < ARRIVAL_RADIUS` (0.15 м)

Безопасность:
- GPS accuracy > 500мм более 2 сек → NAV_ERROR, стоп
- Нет GPS фикса > 1 сек → стоп моторов
- Отключение WebSocket → NAV_IDLE, стоп
- Потеря BNO085 → только GPS heading, снижение скорости

---

## 3. Протокол WebSocket (полный)

### Приложение → ESP32

| Команда | Формат | Описание |
|---------|--------|----------|
| Движение | `M,<left>,<right>` | Ручное управление, -100..100 (есть) |
| Стоп | `STOP` | Плавная остановка (есть) |
| Пинг | `PING` | Проверка связи (есть) |
| Насадка | `ATTACHMENT_ON/OFF` | Управление реле (есть) |
| Крепление | `MOUNT_ON/OFF` | Управление реле (есть) |
| Звук | `SOUND:<1-4>` | Воспроизведение звука (есть) |
| Начало маршрута | `ROUTE_BEGIN,<N>` | **НОВОЕ.** Очистить буфер, подготовить N точек |
| Точка маршрута | `ROUTE_WP,<i>,<lat>,<lon>` | **НОВОЕ.** Добавить точку (8 знаков после запятой) |
| Конец маршрута | `ROUTE_END` | **НОВОЕ.** Завершить загрузку маршрута |
| Старт авто | `NAV_START` | **НОВОЕ.** Начать автономное движение |
| Пауза | `NAV_PAUSE` | **НОВОЕ.** Пауза |
| Продолжить | `NAV_RESUME` | **НОВОЕ.** Продолжить после паузы |
| Стоп авто | `NAV_STOP` | **НОВОЕ.** Остановить автономный режим |

### ESP32 → Приложение

| Сообщение | Формат | Частота | Описание |
|-----------|--------|---------|----------|
| Подключение | `STATE,CONNECTED` | При подключении | Есть |
| Понг | `PONG` | По запросу | Есть |
| Батарея | `BAT_PCT,<int>` | 500мс | Есть |
| GPS позиция | `GPS,<lat>,<lon>,<heading>,<fixType>,<hAcc>` | 200мс | **НОВОЕ** |
| IMU курс | `IMU,<yaw>` | 200мс | **НОВОЕ** |
| Статус навигации | `NAV,<state>,<wpIdx>,<wpTotal>,<distToWp>` | 500мс | **НОВОЕ** |
| Точка достигнута | `NAV_WP,<index>` | По событию | **НОВОЕ** |
| Маршрут загружен | `OK ROUTE,<count>` | По событию | **НОВОЕ** |
| Ошибка | `ERR,<msg>` | По событию | **НОВОЕ** |

---

## 4. Изменения Flutter-приложения

### 4.1 Новый файл: `lib/core/gps_projection.dart`

Проекция GPS ↔ локальные метры (плоская Земля, точность <1мм на площади до 1 км²):

```dart
class GpsProjection {
  final double refLat, refLon;  // опорная точка (первая точка периметра)
  late final double mPerDegLat;  // метров на градус широты
  late final double mPerDegLon;  // метров на градус долготы

  // GPS → локальные метры: Offset(east, north)
  Offset toLocal(double lat, double lon);

  // Локальные метры → GPS: (lat, lon)
  (double, double) toGps(Offset local);
}
```

### 4.2 `lib/core/wifi_connection.dart` — Новые поля и парсинг

Добавить в `WifiConnectionState`:
```dart
double? gpsLat, gpsLon, gpsHeading;
int? gpsFixType, gpsAccuracy;
double? imuYaw;
String? navState;  // IDLE, RUNNING, PAUSED, DONE, ERROR
int? navWpIndex, navWpTotal;
double? navDistToWp;
```

Добавить парсинг `GPS,...`, `IMU,...`, `NAV,...` в stream listener.

Добавить методы отправки:
```dart
sendRouteBegin(int count)
sendRouteWaypoint(int index, double lat, double lon)
sendRouteEnd()
sendNavStart() / sendNavPause() / sendNavResume() / sendNavStop()
```

### 4.3 `lib/core/map_storage.dart` — Поддержка GPS-карт

Добавить поля в JSON:
```json
{
  "coordinateType": "gps",    // "cell" для старых карт
  "refLat": 55.12345678,
  "refLon": 37.98765432,
  "perimeter": [{"lat": ..., "lon": ...}, ...],  // исходные GPS-точки
  ...  // zones/forbiddens хранятся в локальных метрах (Offset)
}
```

Обратная совместимость: `coordinateType` по умолчанию `"cell"`.

### 4.4 `lib/features/manual/manual_control_screen.dart` — Запись периметра

Новый режим "Запись периметра":
- Кнопка "Записать периметр" в панели инструментов
- При включении: красный пульсирующий индикатор, счётчик точек
- Приложение слушает `GPS,...` из WebSocket, записывает точки каждые ~0.5м
- "Стоп запись": упрощение полигона (Douglas-Peucker, ~0.3м допуск), сохранение как зона
- Первая точка = `refLat/refLon` для проекции

### 4.5 `lib/core/cleaning_route_planner.dart` — Параметр lineStep

Единственное изменение: вызывающий код передаёт `lineStep = 0.50` (метры) для GPS-карт вместо `44.0` (ячейки). Алгоритм змейки работает одинаково — он оперирует `Offset` координатами, неважно метры это или ячейки.

### 4.6 `lib/features/auto/auto_map_screen.dart` — Автономный режим

- Отображение позиции робота на карте (GPS → локальные метры → экран)
- Кнопка "Отправить маршрут" → конвертация точек маршрута из метров в GPS → загрузка через `ROUTE_BEGIN/WP/END`
- Кнопки "Старт" / "Пауза" / "Стоп"
- Прогресс: `wpIndex / wpTotal`, подсветка пройденных точек
- След робота в реальном времени

### 4.7 `lib/features/home/home_screen.dart` — GPS индикатор

Маленький индикатор рядом с батареей: тип фикса (Нет / 3D / RTK Float / RTK Fixed) и точность в см.

---

## 5. Порядок реализации (по фазам)

### Фаза 1: GPS-телеметрия (ESP32 + чтение в приложении)
**ESP32:** Рефакторинг sound.ino → модульная структура. GPS парсер на Serial1. Трансляция `GPS,...` в WebSocket.
**Flutter:** Парсинг GPS в wifi_connection.dart. GPS индикатор на домашнем экране.
**Тест:** Подключить F9P, увидеть координаты в терминале приложения.

**Файлы:**
- СОЗДАТЬ: `firmware/firmware.ino`, `config.h`, `motors.cpp`, `gps.cpp`, `websocket.cpp`, `sound.cpp`, `telemetry.cpp`
- ИЗМЕНИТЬ: `wifi_connection.dart`, `home_screen.dart`

### Фаза 2: IMU
**ESP32:** BNO085 через I2C, трансляция `IMU,yaw`.
**Тест:** Вращать робота, видеть изменение курса.

**Файлы:**
- СОЗДАТЬ: `firmware/imu.cpp`

### Фаза 3: Запись периметра
**Flutter:** `gps_projection.dart`, режим записи в manual_control_screen, сохранение GPS-карты.
**Тест:** Объехать двор, увидеть полигон на карте.

**Файлы:**
- СОЗДАТЬ: `gps_projection.dart`
- ИЗМЕНИТЬ: `manual_control_screen.dart`, `map_storage.dart`

### Фаза 4: Маршрут змейки в GPS
**Flutter:** Построение маршрута в метрах, конвертация в GPS, отображение.
**Тест:** Маршрут корректно отображается на GPS-карте.

**Файлы:**
- ИЗМЕНИТЬ: `auto_map_screen.dart`, `cleaning_route_planner.dart` (только lineStep)

### Фаза 5: Загрузка маршрута на робота
**ESP32:** Команды `ROUTE_BEGIN/WP/END`, хранение точек в памяти.
**Flutter:** Кнопка "Отправить", подтверждение `OK ROUTE,N`.

**Файлы:**
- ИЗМЕНИТЬ: `firmware/websocket.cpp`, `firmware/nav.cpp`
- ИЗМЕНИТЬ: `auto_map_screen.dart`, `wifi_connection.dart`

### Фаза 6: Автономная навигация
**ESP32:** PID контроллер, waypoint follower, `NAV_START/PAUSE/STOP`.
**Flutter:** Кнопки управления, прогресс, позиция в реальном времени.

**Файлы:**
- СОЗДАТЬ/ДОПИСАТЬ: `firmware/nav.cpp`
- ИЗМЕНИТЬ: `auto_map_screen.dart`

### Фаза 7: RTCM relay (RTK фикс)
**ESP32:** UDP слушатель на порту 2101, пересылка на Serial1 TX.
**Тест:** GPS статус переходит в RTK Fixed.

### Фаза 8: Полировка
- Тюнинг PID на поле
- Возврат на старт по завершении
- Авто вкл/выкл насадки
- Звуковые сигналы событий

---

## 6. Верификация (как тестировать)

1. **Фаза 1:** Подключить F9P к ESP32, запустить прошивку → в терминале приложения должны появиться строки `GPS,55.xxx,37.xxx,...`
2. **Фаза 2:** Вращать робота → `IMU,xxx` изменяется плавно 0-360
3. **Фаза 3:** Объехать территорию вручную → на карте появляется полигон, сохраняется и загружается
4. **Фаза 4:** Нажать "Построить маршрут" на GPS-карте → видна змейка с шагом ~50 см
5. **Фаза 5:** Нажать "Отправить" → в терминале `OK ROUTE,N` с правильным числом точек
6. **Фаза 6:** Поставить робота на открытой площадке, загрузить маршрут из 4 точек (квадрат 3x3м), нажать "Старт" → робот проезжает по точкам
7. **Фаза 7:** Подключить базу → GPS accuracy падает до ~20мм, статус RTK Fixed
8. **Полный тест:** Записать периметр двора → построить змейку → отправить → старт → робот проезжает всю территорию
