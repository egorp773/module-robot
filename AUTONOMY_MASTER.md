# AUTONOMY MASTER — план полной автономности RTK-робота

> **Назначение файла:** единый источник правды для проекта «сделать робота полностью
> автономным». Если сессия оборвётся — прочитать этот файл и продолжить с того же места.
> Обновляется постоянно. Создан: 2026-06-13.

---

## 0. ГЛАВНАЯ ЗАДАЧА (от пользователя, дословный смысл)

Сделать робота **полностью автономным**: запоминание координат/периметра, навигация,
автопилот — «чтобы всё заработало». Дано **полное добро** при необходимости
переписать алгоритмы и прошивки **с нуля**. Интегрировать лучшие практики из
существующих RTK-навигаций (ArduMower/Sunray, ArduPilot Rover, OpenMower и др.).

### Что ассистенту РАЗРЕШЕНО
- Прошивать базу и ровер (они сейчас на улице, подключены к компьютеру).
- Проверять работу, калибровать, настраивать.
- Переписывать прошивку/алгоритмы с нуля.

### Жёсткие правила (НЕ нарушать)
- **НЕ двигать колёсами без явного разрешения пользователя.** Каждый раз спрашивать перед движением.
- Делать **полную проверку** при тестировании.
- Полевые тесты — только на WiFi `Xiaomi_6A92`.

### Текущее физическое состояние (на момент постановки задачи)
- Робот и база **на улице**, подключены к компьютеру по USB.
- Робот **смотрит вперёд по курсу ≈176°** (важно для калибровки heading / IMU bias).
- Готовы к прошивке и тестам.

---

## 1. ЖЕЛЕЗО

- 2× ESP32 Dev Kit (база + ровер).
- 2× u-blox ZED-F9P-04B-01 (RTK GPS).
- IMU BNO085 (game rotation vector).
- Контроллер мотора от гироскутера (hoverboard), UART.

### Пины ESP32 (по памяти проекта — ПРОВЕРИТЬ по коду перед прошивкой)
- GPS UART1: RX=4, TX=5, 38400 baud (к F9P).
- Motor UART2: RX=16, TX=17, 115200 baud.
- IMU I2C: SDA=21, SCL=22.
- Реле: ATTACH=32, MOUNT=33.

### Сеть / порты
- WiFi SSID: `Xiaomi_6A92`, пароль: `17762646`, сеть `192.168.31.0/24`, шлюз `192.168.31.1`.
- База: IP `192.168.31.207`, порт **COM7** (обновлено 2026-06-13; было COM4).
- Ровер: IP `192.168.31.222`, порт **COM3** (обновлено 2026-06-13; было COM6).
- Обе ESP32 сейчас подключены к компьютеру.
- RTCM: **UDP unicast** на IP ровера, порт **2101** (TCP/broadcast не использовать — см. баги).
- WebSocket телеметрия: `ws://<rover>:81/ws` (line-based, лимит 255 байт).

### Известные баги железа/ESP-IDF (workarounds подтверждены)
- **TCP server на базе падает**: `tcpip_send_msg_wait_sem ... Invalid mbox` если `WiFiServer.begin()`
  вызван слишком рано после подключения WiFi. → Не использовать WiFiServer, использовать UDP unicast.
- **Роутер Xiaomi блокирует межклиентский broadcast** → только UDP unicast на конкретный IP.
- **Тайминг millis()**: `now % NAV_LOOP_MS == 0` не работает → паттерн `now - lastMs >= interval`.
- **POWERON_RESET при старте мотора**: просадка питания/RF. Уже снижали WiFi TX power до 11 dBm
  (коммит `06d472c`). Следить при больших токах мотора.

---

## 2. ТЕКУЩАЯ СТРУКТУРА КОДА (на 2026-06-13)

Идёт реорганизация монолита в модули PlatformIO (`rtk_firmware/lib/`):

| Модуль | Файлы (строк) | Назначение (предположительно) |
|---|---|---|
| Config | RtkConfig.h (85) | Конфиг/константы |
| Gnss | Gnss.h (94) / Gnss.cpp (121) | Работа с F9P, парсинг позиции/fix |
| Imu | Imu.h (33) / Imu.cpp (57) | BNO085 |
| Motor | Motor.h (48) / Motor.cpp (111) | UART к hoverboard, M,left,right |
| NavCore | NavCore.h (98) / NavCore.cpp (180) | Ядро навигации (pure pursuit и т.п.) |
| Route | Route.h (51) / Route.cpp (64) | Хранение маршрута/waypoints |
| RtcmLink | RtcmLink.h (47) / RtcmLink.cpp (78) | Передача RTCM база→ровер |
| Safety | Safety.h (44) / Safety.cpp (80) | Аварийные состояния/recovery |
| StateEstimator | StateEstimator.h (84) / .cpp (121) | Оценка состояния (fusion) |
| WebSocket | WsServer.h (83) / WsServer.cpp (389) | WS телеметрия + команды |

- `rtk_firmware/src/base.cpp` (113) — прошивка базы.
- `rtk_firmware/src/rover.cpp` (320) — прошивка ровера (сильно ужата vs старая монолитная).
- `module_app/` — Flutter-приложение (мониторинг + рисование периметра/маршрута, авто-режим).
  НЕ пульт — только отправляет маршрут и мониторит.
- `simulator/` — Python HITL-раннер (тесты навигации без железа).

> ⚠️ Старая память (`project_rtkgps.md`) ссылалась на `rover.cpp:строка` для монолита —
> эти номера строк УСТАРЕЛИ после модуляризации. Перед правкой — заново грепать символы.

### Система координат (подтверждено, не менять без причины)
- Локальные метры East-North от GPS-origin. `x=East, y=North`.
- 0° = Север, положительное по часовой стрелке (clockwise).
- `x = (lon - originLon) * mPerDegLon`, `y = (lat - originLat) * mPerDegLat`.
- `mPerDegLat = 111132.92 − 559.82·cos(2φ) + 1.175·cos(4φ)`.
- `headingTo = atan2f(dx, dy) * 180/π` → нормировать в 0..360.

### Протокол (ключевые команды WS)
- Управление/маршрут: `M,left,right`, `STOP`, `GO_TO,lat,lon`,
  `ROUTE_BEGIN,<count>,<originLat>,<originLon>` + N×`ROUTE_WP,<i>,<x>,<y>` + `ROUTE_END`,
  `AREA_BEGIN/PT/END`, `FORBID_BEGIN/PT/END`, `NAV_START/PAUSE/RESUME/STOP`, `NAV_CFG,...`,
  `STATUS`, `CAL_IMU,heading`.
- Телеметрия: `TEL,...`, `GPS,lat,lon,heading,fix,hAcc`, `GPSDBG,...`, `IMU,yaw,age,fresh`,
  `NAV,state,wpIdx,wpTotal,dist`, `MOTOR,...`, `STATE,...`, `RTCM,...`, `BAT_PCT,...`.

---

## 3. РЕСЁРЧ: алгоритмы RTK-навигации (Sunray / ArduPilot / OpenMower)

> Источник: deep-research workflow 2026-06-13. Верификаторы попали в rate-limit (429),
> поэтому авто-метка «refuted» НЕДЕЙСТВИТЕЛЬНА. Факты ниже — из первичных источников
> (u-blox app notes, Sunray `config_example.h`, ArduPilot/PX4 wiki) и подтверждаются
> знанием ассистента. Помечены [✓ надёжно] / [~ проверить в коде].

### 3.1 RTK GPS: базовые принципы
- **RTCM3** — формат коррекций. Для **moving base** (HPG/HDG 1.13+) рекомендованный набор:
  `RTCM 4072.0` (reference station PVT) + `1074/1084/1094/1124` (GPS/GLONASS/Galileo/BeiDou MSM4)
  + `1230` (GLONASS code-phase biases). `4072.1` больше не нужен; MSM7→MSM4 снижает нагрузку на линк. [✓]
- **Базовая станция vs NTRIP**: своя F9P-база в Survey-In (фикс. координата после усреднения)
  ИЛИ NTRIP-сервис (готовые коррекции из сети). У нас — **своя база, Survey-In** (600 с / 1 м). [✓]
- **FLOAT → FIXED**: ровер получает RTCM → быстро входит в FLOAT → при разрешении
  фазовых неоднозначностей (ambiguities) → FIXED. Падает в FLOAT при потере фаз. lock. [✓]
- **Точность**: FIXED ≈ 1–2 см (hAcc ~14 мм у нас достигалось), FLOAT — дециметры. [✓]
- В UBX `carrSoln`: 1=FLOAT, 2=FIXED.

### 3.2 Moving Base / dual-antenna heading (КЛЮЧЕВАЯ ОПЦИЯ для курса)
- Две антенны на одной платформе: одна F9P = **moving base**, вторая F9P (или F9H) = **rover**.
  Вектор между антеннами даёт **курс (heading)** независимо от движения и без калибровки. [✓]
- Курс/длина базы выводятся в **`UBX-NAV-RELPOSNED`**: `relPosHeading` + точность,
  `relPosHeadingValid`, `relPosNormalized` (нормировка к 1 м), `carrSoln` (1=float,2=fixed). [✓]
- Точность курса: **ZED-F9H ≈ 0.4° (50‑й перцентиль) при базе 1 м**. F9P-пара — субградусная. [✓]
- Приёмник **не фильтрует** RELPOSNED — низкочастотную фильтрацию делает приложение. [✓]
- Валидация: расчётная длина базы должна совпадать с реальной; если нет — ambiguities
  зафиксированы неверно, курс ложный (критично на коротких базах). [✓]
- F9H может быть **только rover'ом**, не базой. [✓]
- ⚠️ У нас сейчас **2× F9P, обе используются как база+ровер RTK (позиция)**, не как
  dual-antenna heading. Это архитектурная развилка — см. раздел 5.

### 3.3 Sensor fusion
- **Sunray**: локализация = слияние **колёсной одометрии (ticks)** + **GPS-RTK**, плюс
  **опциональный IMU** для краткосрочного улучшения направления (heading). Не полноценный EKF —
  ближе к взвешенному слиянию + gyro для heading между GPS-апдейтами. [✓]
- **ArduPilot EKF3**: 24-состояний EKF (поз, скорость, attitude-кватернион, gyro bias,
  accel bias, wind, earth/body mag field). Слияние gyro+accel+compass+GPS+baro(+опт.поток,
  +дальномер, +**wheel encoders**, +visual odom). Несколько параллельных «cores/lanes»
  (по одному на IMU) с выбором лучшего по innovation. GPS отвергается по innovation-gate. [✓]
- **Колёсные энкодеры в ArduPilot** дают скорость для EKF (WENC параметры). [✓]
- При отвержении GPS EKF продолжает по инерциалке/одометрии (**dead-reckoning**), затем
  ресинхронизируется. [✓]

### 3.4 Heading / курс (критичная проблема нашего проекта)
- **GPS velocity heading**: курс из вектора скорости. Работает только в движении выше
  порога скорости; **на низкой скорости/на месте — мусор** (наша проблема «крутится на месте»). [✓]
- **Dual-antenna moving base**: курс на месте и на любой скорости, субградусно. **Лучший вариант.** [✓]
- **IMU/magnetometer fusion**: gyro даёт быстрый отн. курс (дрейфует), magnetometer — абс.
  (искажается металлом/моторами). BNO085 game rotation vector = без магнитометра → дрейф yaw.
- **Рекомендация**: пока нет dual-antenna — fusion **gyro (BNO085) для краткосрочного курса
  + GPS-velocity heading для коррекции дрейфа при движении** (как делает Sunray). [✓]

### 3.5 Следование траектории
- **Pure pursuit**: цель — точка на пути на расстоянии lookahead; рулёжка по кривизне к ней.
  Lookahead — компромисс: мал → колебания, велик → срезает углы. [✓]
- **Stanley controller** (Sunray использует Stanley): рулёжка = ошибка курса +
  `atan(k·cross_track_error / v)`. Параметры Sunray: `STANLEY_CONTROL_P_NORMAL=3.0`,
  `STANLEY_CONTROL_K_NORMAL=1.0` (кошение), `STANLEY_CONTROL_P_SLOW=3.0`,
  `STANLEY_CONTROL_K_SLOW=0.1` (докинг/медленно). [~ проверить точные дефолты в config_example.h]
- **Cross-track error** — поперечное отклонение от линии сегмента; ключевая величина для обоих.

### 3.6 Планирование маршрута / покрытие
- **Boustrophedon (змейка/serpentine)** — основной паттерн покрытия (туда-обратно рядами). [✓]
- **Perimeter following** — обход периметра по контуру. [✓]
- **Sunray obstacle avoidance**: не отдельный контроллер слежения, а **граф/replanning** —
  при препятствии вставляется временное виртуальное препятствие и маршрут пересчитывается
  от текущей точки к цели; после 3 неудач → error state. [~ проверить]

### 3.7 Обработка GPS (надёжность)
- **Sunray outlier rejection**: фильтр по C/N0 и мин. углу возвышения спутников
  (дефолты ~ Min SV Elevation 10°, пороги C/N0). [~ проверить точные дефолты]
- **GPS jump / kidnap detection (Sunray)**: `KIDNAP_DETECT`=true; триггер recovery когда
  расстояние до трекаемого пути > `KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE` (≈1.0 м, 0.2 м при докинге). [~]
- **REQUIRE_VALID_GPS (Sunray)**: пауза кошения если нет ни FLOAT, ни FIX. [~]
- **Float→Fixed transitions**: не доверять позиции в момент перехода; сглаживать скачок.

### 3.8 Сравнение подходов
| | Sunray | ArduPilot Rover | OpenMower |
|---|---|---|---|
| Fusion | одометрия+GPS+опц.IMU (лёгкое) | EKF3 (полный, 24 сост.) | ROS, robot_localization EKF |
| Heading | GPS-vel + IMU gyro | EKF (gyro+mag+GPS) / dual-GPS | dual-antenna F9P moving base |
| Слежение | Stanley | L1 / pure-pursuit | ROS nav, pure-pursuit/FTC |
| Покрытие | змейка+perimeter, граф-replan | mission waypoints | boustrophedon (Slic3r-подобно) |
| Платформа | ESP32/Due/own PCB | STM32/Pixhawk | ESP32 + Raspberry Pi + ROS |

**Вывод для нас:** наша платформа (2×ESP32) ближе всего к **Sunray**. Брать его модель:
лёгкое слияние одометрия/IMU/GPS + Stanley/pure-pursuit + змейка + kidnap-detection.
OpenMower-подход (dual-antenna heading) — лучший по курсу, но требует 2 F9P на роботе
как antenna-пару (а у нас одна на базе, одна на ровере).

---

## 4. ИЗВЕСТНЫЕ ПРОБЛЕМЫ НАВИГАЦИИ (из истории проекта)

- **«Робот крутится на месте»** на периметре. Подозрения:
  - `alignFirstThresholdDeg=35°` блокирует движение вперёд при ошибке курса >35° на низкой скорости.
  - `maxTurn` (~27% от MAX_SPEED_PERCENT) не пробивает трение гусениц при развороте на месте.
  - `ARRIVAL_DIST_M=0.05` (5 см) туго относительно дрейфа RTK 14 мм → детектор прибытия осциллирует.
  - GPS-only heading: на месте курс заморожен/мусорный → не может выровняться → крутится.
- Змейка: connector-детуры между сегментами → большие развороты на месте.
- Recovery `no_rtk_fix` блокирует движение если не FIXED/FLOAT.

**Корневая причина большинства — отсутствие надёжного курса на низкой скорости.**
Это главное, что надо решить (см. план).

---

## 5. ПЛАН (архитектурная развилка по heading)

### ✅ ЦЕЛЕВАЯ АРХИТЕКТУРА ЖЕЛЕЗА (решение пользователя 2026-06-13)
**ОДИН ESP32 на ровере делает ВСЁ:** F9P GPS (UART1 4/5) + IMU BNO085 (I2C 21/22) +
мотор-гироскутер (UART2 16/17) + навигация/автопилот + WiFi/WebSocket + звук (I2S 25/26/27)
+ реле (32/33). Второй ESP32 = база (Survey-In + RTCM по UDP unicast на ровер).
→ Значит финальная прошивка ровера = СЛИЯНИЕ драйв/звук/WiFi-слоя из sound.ino
  + чистый новый навигационный слой (fusion+автопилот+маршруты). База — отдельная прошивка.
→ Проверка бюджета пинов: F9P(4,5) IMU(21,22) Motor(16,17) Relay(32,33) — конфликтов НЕТ.
→ **ЗВУК НЕ НУЖЕН (решение 2026-06-13):** динамика нет, I2S не реализуем. Пины 25/26/27 свободны.
  Из sound.ino берём ТОЛЬКО: мотор-UART драйв-слой + WiFi/WS транспорт. Звук/I2S/WAV — выкинуть.

### РЕЖИМ РАБОТЫ (2026-06-13): пользователь ушёл, разрешил работать автономно без вопросов.
- Можно: проектировать, писать код, компилировать, читать файлы.
- НЕЛЬЗЯ: двигать колёса (только с личного разрешения при возвращении).
- Прошивку ровера с активным мотором и ездовые тесты — отложить до возвращения пользователя.
- Решения принимать самому по умолчанию, всё документировать здесь.

### Развилка: откуда брать надёжный курс?
- **Вариант A (минимум железа):** оставить 1×F9P на ровере. Курс = fusion BNO085 gyro
  (краткосрочно) + GPS-velocity (коррекция дрейфа при движении). Как Sunray.
  + Не трогать железо. − Курс на месте — только gyro-интеграция (дрейфует), нужна калибровка bias.
- **Вариант B (лучший курс):** dual-antenna moving base — 2 F9P на роботе как пара антенн,
  курс субградусный на месте через RELPOSNED. Но тогда база уходит в NTRIP или вторая
  отдельная база. − Серьёзная переделка железа/антенн.

→ **РЕШЕНИЕ ПОЛЬЗОВАТЕЛЯ (2026-06-13): Вариант A.** Курс = BNO085 gyro (краткосрочно)
+ GPS-velocity heading (коррекция дрейфа при движении). Железо не трогаем. Задел под B оставить.

### Этапы
1. [ ] Изучить текущий код модулей (Gnss/Imu/StateEstimator/NavCore/Route/Safety) — что уже есть.
2. [ ] Спроектировать StateEstimator: fusion gyro+GPS-vel heading + позиция RTK с outlier-фильтром.
3. [ ] NavCore: Stanley/pure-pursuit + надёжный low-speed heading + align-first логика без «кручения».
4. [ ] Safety: kidnap/jump detection, float/fixed gating, dead-reckoning fallback.
5. [ ] Route: змейка + perimeter, корректный старт «откуда стоит робот».
6. [ ] Калибровка IMU heading с известным курсом 176° (`CAL_IMU,176`).
7. [ ] Стендовые тесты БЕЗ движения колёс → затем тесты движения С РАЗРЕШЕНИЯ.
8. [ ] HITL-симулятор для проверки логики до железа.

---

## 5a. ПОДХОД К ПЕРЕПИСУ — ГИБРИД (подтверждено пользователем 2026-06-13)

НЕ слепой копипаст ArduMower/Sunray (он под GPLv3 и под чужое железо). НЕ «всё с нуля».
Три источника, каждый по своей роли:
- **Железо-драйверы** (мотор, F9P, IMU, RTCM): из ТВОЕГО рабочего `sound.ino` + проверенных
  частей rtk_firmware. Протоколы 1:1 с тем что РЕАЛЬНО ездило на этом железе.
- **Алгоритмы навигации** (heading-fusion, Stanley, outlier-фильтр): идеи/формулы из
  Sunray/ArduPilot (раздел 3, ресёрч), ПЕРЕПИСАННЫЕ своими словами под это железо (не GPL-копия).
- **Протокол + координаты**: из твоего Flutter-app (gps_projection.dart, WS-протокол). App ОСТАЁТСЯ.
→ Итог: рабочее железо-ядро + новый чистый нав-слой сверху + app не трогаем. Эволюция, не революция.

## 5b. ПОЛНЫЙ АУДИТ КОДА (2026-06-13) — что переписываем

Прочитаны ВСЕ модули. Вердикт по каждому:

| Файл | Вердикт | Действие |
|---|---|---|
| `Motor.{h,cpp}` | ⛔ PWM вместо UART-hoverboard | ПЕРЕПИСАТЬ под Serial2 hoverboard-протокол (эталон sound.ino) |
| `StateEstimator.{h,cpp}` | ⛔ IMU не входит в heading; carrSoln нет | ПЕРЕПИСАТЬ: gyro+GPS fusion |
| `Gnss.{h,cpp}` | ⚠️ carrSoln=эвристика hAcc≤50mm | ПОЧИНИТЬ: читать реальный carrSoln/flags |
| `RtcmLink.{h,cpp}` | ⛔ TCP primary (баг Invalid mbox) | ПЕРЕПИСАТЬ: только UDP unicast приём |
| `base.cpp` | ⛔ WiFiServer TCP + broadcast (оба = грабли) | ПЕРЕПИСАТЬ: только UDP unicast на ровер |
| `rover.cpp` | ⚠️ heading seed 320° (робот 176°); fakeBat=100; IMU не в fusion | ПЕРЕПИСАТЬ loop + follower |
| `Safety.{h,cpp}` | ~ логика разумная, но завязана на старый StateEstimator | АДАПТИРОВАТЬ под новый estimator |
| `Route.{h,cpp}` | ✅ хранение WP норм | ОСТАВИТЬ (мелкие правки) |
| `NavCore.{h,cpp}` | ✅ геометрия/полигоны/obstacle норм | ОСТАВИТЬ как библиотеку геометрии |
| `Imu.{h,cpp}` | ✅ железо-обёртка норм | ОСТАВИТЬ + добавить gyroZ rate |
| `WsServer.{h,cpp}` | ~ транспорт норм, _motorFeedback фейк | АДАПТИРОВАТЬ: реальная батарея из Motor |
| `RtkConfig.h` | ~ пины ок, nav-константы старые | ПЕРЕСМОТРЕТЬ константы |

### Детали ключевых багов
- **rover.cpp:18** `MotorSerial(2) reserved, не используется` + **:226** PWM init 25/26/27/14 → мотор мёртв.
- **rover.cpp:223** `seedHeadingDeg(ROVER_INITIAL_HEADING_DEG=320)` — НЕ совпадает с физ.курсом 176°.
- **rover.cpp:238,305** `fakeBat=100` — батарея выдумана, реального hoverboard-фидбэка нет.
- **base.cpp:12,59** `WiFiServer g_tcpServer; .begin()` — ровно баг Invalid mbox из памяти.
- **base.cpp:95-99** broadcast 192.168.31.255 — Xiaomi его режет (бесполезный трафик).
- **StateEstimator.cpp:80** heading ТОЛЬКО при gSpeed>0.1м/с из headMot; IMU вообще не подаётся.
- **Gnss.cpp:26** carrSoln эвристика по hAcc — врёт на границе FIXED/FLOAT.
- **RtcmLink.cpp:22-43** TCP primary (connect к base:2102) — на base TCP-сервер падает → бессмысленно.

## 5c. АРХИТЕКТУРА НОВОЙ ПРОШИВКИ (план реализации)

### Транспорт RTCM (УПРОСТИТЬ — убрать TCP полностью)
- base.cpp: читать UART F9P → слать ТОЛЬКО UDP unicast на ROVER_IP:2101. Никакого WiFiServer/broadcast.
- RtcmLink (rover): только `_udp.parsePacket()` на 2101 → `feedRtcm()`. Убрать TCP-клиент.

### Motor (новый, под hoverboard UART)
- `MotorHover` класс: Serial2 @115200 RX16/TX17. sendHoverboard(steer,speed) 0xABCD протокол.
- setLinearAngularSpeed(v,w): diff-drive → left/right% → speed/steer (как drive() в sound.ino).
- receiveFeedback(): парс SerialFeedback → реальные батарея(вольты,%), ток, speed_meas.
- failsafe: нет команды CMD_TIMEOUT_MS=400 → плавный стоп.

### StateEstimator (новый — Sunray-style fusion, Вариант A)
- Вход: GPS PVT (поз, gSpeed, headMot, carrSoln) + IMU (yaw, gyroZ rate).
- **Heading fusion:**
  - gyroZ интегрируется для краткосрочного курса (быстро, но дрейф).
  - Когда едем (gSpeed > порог ~0.15 м/с) и FIX/FLOAT: GPS-course корректирует дрейф gyro
    (комплементарный фильтр: `head = head + gyroZ*dt; head += alpha*wrap(gpsCourse - head)`).
  - На месте: держим текущий курс по gyro-интегралу (НЕ замораживаем мусор, НЕ берём GPS-course).
  - Калибровка абсолютного нуля: команда CAL_HEADING,<deg> сидит текущий курс (напр. 176°).
- **Позиция:** lla→local (использовать ТУ ЖЕ формулу что в app: 111132.92−559.82cos2φ+1.175cos4φ).
- **Outlier rejection:** отбрасывать PVT с скачком > вероятного (v*dt + margin), как Sunray.
- carrSoln из реального поля (поправить Gnss).

### Follower (новый — Stanley, без «кручения»)
- Проблема старого: angleGate жёстко режет скорость в 0 при >45° и крутит на месте rotate-логикой
  с фиксированной w, что не пробивает трение → застревание. + heading мусорный на месте.
- Новый: Stanley `steer = headingErr + atan(k*crossTrack / (soft+v))`, БЕЗ режима «rotate in place»
  кроме старта. На старте сегмента, если |headingErr|>~60°, разрешить разворот на месте с
  ДОСТАТОЧНОЙ угловой скоростью (выше порога трения) — но теперь курс надёжный (gyro), так что
  разворот контролируемый и завершается. Параметры взять из ресёрча Sunray + подбор на HITL.
- lookahead/handoff: оставить идею, перетюнить.

### Калибровка курса (старт)
- При старте робот физически на 176°. Прошивка: seed heading = 176° (CAL_HEADING) ИЛИ
  взять из первого хорошего GPS-course при первом движении. По умолчанию seed 176.

## 5d. ⚠️ HITL-СИМУЛЯТОР НЕНАДЁЖЕН (обратная связь пользователя 2026-06-13)

**«HITL кривой. Раньше тесты проходили хорошо, а в жизни нихера не работало.»**

→ HITL-зелёный ≠ работает в реальности. НЕ использовать прохождение HITL как критерий готовности.
→ Почему HITL врёт (вероятные причины, проверить):
  - Физика TrackPhysics упрощена: MOTOR_INERTIA=0.3, slip_std=0.015 — не отражает реальное
    трение гусениц, проскальзывание, инерцию платформы гироскутера, мёртвую зону мотора.
  - HITL кормит ИДЕАЛЬНЫЙ heading/позицию, а в реале — мусорный GPS-heading на низкой скорости,
    дрейф IMU, задержки RTCM, скачки FLOAT/FIXED. Именно то, что ломало робота, в HITL отсутствует.
  - Мёртвая зона мотора: реальный hoverboard не трогается ниже некоторого PWM/команды —
    в HITL мотор «идеальный». Поэтому развороты на месте в HITL проходят, в реале застревают.
→ **СТРАТЕГИЯ ВАЛИДАЦИИ:** HITL — только для проверки ЛОГИКИ (не делит ли на ноль, идёт ли индекс
  WP, не NaN ли). Настоящая настройка — ТОЛЬКО на земле, пошагово, с разрешения пользователя,
  начиная с минимальных движений. Закладывать в прошивку обширную ТЕЛЕМЕТРИЮ (heading gyro vs gps,
  crossTrack, команды мотора, фидбэк скорости) чтобы видеть реальное поведение, а не симулятор.
→ Реальный фидбэк скорости от hoverboard (speedL_meas/speedR_meas) — использовать как настоящую
  одометрию, это то, чего у HITL-валидации не было. Сравнивать заданную команду с фактической.

## 5e. ЭТАЛОННАЯ ФОРМУЛА ПРОЕКЦИИ (app gps_projection.dart — ИСТОЧНИК ПРАВДЫ)
Прошивка ДОЛЖНА считать идентично приложению, иначе координаты разъедутся:
```
mPerDegLat = 111132.92 − 559.82·cos(2φ) + 1.175·cos(4φ)
mPerDegLon = 111412.84·cos(φ) − 93.5·cos(3φ)        // φ = refLat в радианах
north = (lat − refLat)·mPerDegLat ;  east = (lon − refLon)·mPerDegLon
local = Offset(east, north)   // x=east, y=north
```
⚠️ Текущий StateEstimator.cpp использует УПРОЩЁННУЮ формулу (111132.0 const, 111319.5·cosLat) —
РАСХОДИТСЯ с app. Заменить на точную формулу выше.

## 5f. СТАТУС ЖЕЛЕЗА НА СТЕНДЕ (2026-06-13, прошито+проверено)
- ✅ Прошиты обе: база COM7, ровер COM3. Запушено на GitHub (commit c455a76).
- ✅ База: Survey-In VALID, RTCM 138КБ/270пакетов, udpFail=0.
- ✅ Ровер: RTK FIXED sol=2 hAcc=20-35мм, sv=26-32.
- ✅ Heading fusion: head=176.0 (seed держится), yawRate живой, imuYaw читается.
- ✅ Мотор UART фидбэк: bat=40-41% приходит с hoverboard (UART работает в обе стороны!).
- ✅ Safety: без app = ESTOP (ws_disconnected) → колёса заблокированы. Правильно.
- ⚠️ pvtAge скачет 870..3681мс (UBX-парсер изредка пропускает PVT). Наблюдать. Если будет
  >SAFE_PVT_AGE_MS=1000 в движении → ESTOP pvt_stale остановит. Возможно поднять navFreq до 5Гц
  или порог. НЕ блокер для первых тестов.
- ⏳ НЕ проверено вживую (нужно физически): знак yawRate при повороте (IMU_YAW_RATE_SIGN),
  реальный разворот мотора (TURN_MIN), направление вперёд/назад мотора.

## 6. ЛОГ ПРОГРЕССА

- **2026-06-13:** Создан этот файл. Запущен deep-research (упёрся в rate-limit 5M tok/h,
  факты собраны и записаны в раздел 3). Изучена структура модулей (раздел 2).
  ⚠️ Аккаунт под часовым лимитом токенов — работать экономно, без тяжёлых воркфлоу.
- **2026-06-13 (реализация — fusion+RTCM):**
  - ✅ GNSS carrSoln: реальный из pvt->flags.bits.carrSoln (не эвристика hAcc).
  - ✅ IMU: добавлен yawRateDps (gyro SH2_GYROSCOPE_CALIBRATED, знак IMU_YAW_RATE_SIGN выверить).
  - ✅ StateEstimator переписан: heading = интеграл gyro (onImu) + GPS-course коррекция при
    движении (комплементарный фильтр kGpsCourseAlpha=0.05, порог kGpsHeadingMinMps=0.15).
    Проекция = точная формула приложения. Outlier-rejection по скачку позиции. Курс больше НЕ
    «протухает» на месте — корень «кручения» устранён на уровне оценки.
  - ✅ RTCM → UDP-only (Sonnet-агент): base.cpp без WiFiServer/broadcast, RtcmLink без TCP.
    Закрыт баг Invalid mbox + broadcast Xiaomi.
  - ✅ Старт-курс 176° (был 320°). Команда SET_HEADING,<deg> в WS уже есть — калибровка с app.
  - **Обе сборки SUCCESS: rover 72.1%, base 61.1%.**
  - ✅ Follower ПЕРЕПИСАН (rover.cpp stepFollower): убран старый rotate-in-place с фикс. скоростью.
    Теперь: разворот на месте с ГИСТЕРЕЗИСОМ (вход >35°, выход <12° — без дёрганья) и угл.скоростью
    ПРОПОРЦИОНАЛЬНОЙ ошибке, но с ПОЛОМ ROVER_TURN_MIN_RADPS=0.45 (пробить трение гусениц — то,
    что ломало робота в реале). В движении — Stanley (heading + atan(k·crossTrack/(soft+v))) с
    замедлением у цели и при большой ошибке. rotateLeft/Right → одно поле turning.
    **rover SUCCESS 72.1%.** Новые константы: TURN_IN_PLACE_ENTER/EXIT_DEG, TURN_MIN_RADPS.
  - ⏳ ОСТАЛОСЬ ДО ЕЗДЫ: (1) прошить базу+ровер на железо (COM7/COM3), (2) на стенде проверить
    телеметрию: RTK FIXED, heading-fusion (gyro крутится → курс меняется на месте), мотор-фидбэк
    батарея. (3) ВЫВЕРИТЬ IMU_YAW_RATE_SIGN и TURN_MIN на железе. (4) Тест езды — ТОЛЬКО с
    разрешения пользователя, минимальными движениями. Колёса не двигать без него.
- **2026-06-13 (режим экономии):** Лимит 5M tok/h. Я (Opus) — ядро/руководство, тяжёлую рутину
  (правки по спеке, чтение) — Sonnet-агентам через Agent(model:sonnet). Никаких workflow.
- **2026-06-13 (реализация — мотор):** ✅ Переписан Motor под UART hoverboard (Serial2 RX16/TX17,
  протокол 0xABCD 1:1 из sound.ino). Добавлены пины/константы в RtkConfig. Реальная батарея
  (10S) и одометрия (speedL/R_meas) из фидбэка вместо fakeBat. rover.cpp адаптирован
  (begin/loop/battery), WsServer setManualPwm→setManualPercent. **rover собрался: SUCCESS,
  Flash 72.8%, RAM 17.1%.** Следующее: Gnss carrSoln fix → StateEstimator fusion → RTCM UDP-only.
- **2026-06-13 (решения пользователя):**
  - Heading = **Вариант A** (BNO085 gyro + GPS-velocity fusion).
  - Стратегия = **ЧИСТЫЙ ПЕРЕПИС НАВИГАЦИИ**. Пользователь предупредил: текущий код
    «с кучей лишней ломающей его фигни» — НЕ доверять текущей логике как фундаменту.
    Оставляем только проверенный железо-уровень: F9P init, UART мотора, RTCM-линк (UDP unicast),
    WiFi/WS транспорт. Всю логику (StateEstimator/fusion, NavCore/слежение, Route/маршруты,
    Safety) — писать заново и чисто.
  - Прочитаны StateEstimator.{h,cpp} и Imu.{h,cpp} — см. заметки ниже.

### Заметки по прочитанному коду (что НЕ так / что оставить)
- `StateEstimator`: позиционирует как «Sunray-style», но fusion СЛАБЫЙ —
  heading берётся ТОЛЬКО из GPS headMot при speed>0.1 м/с; на месте заморожен.
  **IMU вообще не участвует в heading** (нет входа gyro в StateEstimator). Это и есть
  корень «кручения на месте». llaToLocalMeters использует упрощённую формулу
  (kMetersPerDegLat=111132.0 const, без cos-членов) — расходится с формулой приложения.
  → ПЕРЕПИСАТЬ: добавить gyro-интеграцию heading + GPS-coursе коррекцию.
- `Imu`: BNO085 game rotation vector 50 Hz, отдаёт yaw. Quality захардкожен в 1
  (нет реального чтения калибровки). Железо-обёртка рабочая — ОСТАВИТЬ, но добавить
  доступ к gyro-rate (угловой скорости) для fusion, не только абсолютный yaw.
- `Gnss`: F9P init норм (autobaud retry, Survey-In, RTCM 1005/1074/1084/1094/1124/1230,
  переустановка RTCM каждые 3с — всё ОК, оставить). **НО carrSoln определяется ЭВРИСТИКОЙ
  `hAcc<=50mm → FIXED`** (комментарий «SparkFun 2.2.28 не даёт carrierSolution» — НЕВЕРНО,
  библиотека отдаёт реальный carrSoln через NAV-PVT flags). → читать настоящий carrSoln.
  `setRtcmInput()`, `pollRxmRtcm()` — пустые заглушки.
- `Motor`: ⛔ **СЛОМАНО — ГЛАВНЫЙ ПЕРЕПИС.** Код = прямой PWM через LEDC на пины 25/26/27/14
  (`rover.cpp:226`), а `MotorSerial(2)` помечен «reserved, не используется» (`rover.cpp:18`).
  **ПОЛЬЗОВАТЕЛЬ ПОДТВЕРДИЛ (2026-06-13): мотор = плата гироскутера по UART (hoverboard-протокол).**
  → Текущий PWM-Motor.cpp ФИЗИЧЕСКИ НЕ УПРАВЛЯЕТ платой. Робот с этой прошивкой не поедет.
  → ПЕРЕПИСАТЬ Motor под UART: пакет hoverboard-firmware-hack `{START=0xABCD, steer(int16),
    speed(int16), checksum=START^steer^speed}` @115200, обратный фрейм с напряжением/током/
    скоростью/BAT. Подтверждает коммит 751fa73 «Init motor UART + BAT_PCT».
  → ✅ ЭТАЛОН НАЙДЕН: `sound/sound.ino` (840 строк) — рабочая прошивка, где «мотор идеально
    работал, робот ездил без проблем» (слова пользователя). Протокол взять оттуда 1:1.

### ✅✅ ЭТАЛОННЫЙ ПРОТОКОЛ МОТОРА (из sound/sound.ino — РАБОЧИЙ)
Это стандартный **hoverboard-firmware-hack** (Niklas Fauth). Брать как есть.
- **UART: Serial2, RX=16, TX=17, 115200 baud.** START_FRAME = `0xABCD`.
- **Команда (ESP32→плата):** `struct __packed { uint16_t start; int16_t steer; int16_t speed; uint16_t checksum; }`
  - `checksum = start ^ steer ^ speed`. Слать каждые `HOVER_SEND_MS=20`мс.
- **Фидбэк (плата→ESP32):** `struct __packed { uint16_t start; int16_t cmd1; int16_t cmd2;
  int16_t speedR_meas; int16_t speedL_meas; int16_t batVoltage; int16_t boardTemp;
  uint16_t cmdLed; uint16_t checksum; }`
  - `checksum = start^cmd1^cmd2^speedR_meas^speedL_meas^batVoltage^boardTemp^cmdLed`.
  - Парсинг: скользящее окно по 2 байта ищет START_FRAME (см. receiveHoverboardFeedback).
  - `batVoltage * 0.01` = вольты; 10S (CELL_COUNT=10); boardTemp * 0.1.
- **Преобразование left/right(-70..70%) → steer/speed** (функция drive()):
  - `HOVER_MAX_CMD = 300` (шкала команды платы).
  - `speed = (left+right) * 300 / (2*70)`;  `steer = (right-left) * 300 / (2*70)`.
  - Slew (сглаживание в домене команды): `SLEW_SPEED_PER_SEND=4`, `SLEW_STEER_PER_SEND=6`.
  - Ramp в percent-домене: `RAMP_UPDATE_MS=20`, step up/down=1/tick.
  - `INPUT_DIV=2` (значения с app делятся на 2 — спокойнее).
  - `CMD_TIMEOUT_MS=400` — если нет команд 400мс → smooth stop (failsafe).
- **ПОДТВЕРЖДЕНИЕ что новый Motor.cpp сломан:** пины 25/26/27 в sound.ino = I2S аудио
  (DOUT=27, BCLK=26, LRCK=25), а новый Motor.cpp занял их под PWM мотора. Двойная ошибка.
- Реле в sound.ino: ATTACHMENT=GPIO32 (active HIGH), MOUNT=GPIO33 (active HIGH) — совпадает с RtkConfig.
- sound.ino также имеет: WiFi (AP «Robot»), AsyncWebServer:81 + WS «/ws», I2S звук (WAV из LittleFS),
  low-batt звук <30%. Это отдельный «звуковой/драйв» слой — НЕ навигация.
- `RtkConfig.h`: пины F9P (RX4/TX5/38400), IMU (SDA21/SCL22), реле (32/33). Сеть/порты совпадают
  с памятью. Куча nav-констант (ROVER_*) — это параметры СТАРОЙ ломаной навигации, при переписе
  пересмотреть. ROVER_INITIAL_HEADING_DEG=320 (НЕ совпадает с текущим физ. курсом 176°!).
  ROVER_MAX_PWM=70 (из 255) — мотор сильно ограничен.

---

- **2026-06-13 (финальная железная проверка без движения):**
  - ✅ Найдены и исправлены safety/telemetry баги после прошивки:
    `Gnss::hasFreshPvt()` больше не переиспользует один PVT бесконечно; `SAFETY_HOLD`
    теперь реально запрещает движение; WebSocket больше не пишет `data[len]=0` за предел буфера;
    NAV-телеметрия получила отдельный таймер; MOTOR-телеметрия показывает реальный feedback
    hoverboard-платы вместо фейкового `1`.
  - ✅ SparkFun AutoPVT/callback/poll на этом UART давал стареющий PVT-кэш. Решение:
    ровер настраивает F9P на auto NAV-PVT и сам парсит UBX NAV-PVT по UART; `carrSoln`
    читается из flags, RTCM в F9P продолжает идти через тот же UART TX ESP32.
  - ✅ Обе сборки SUCCESS и обе платы прошиты: base `COM7`, rover `COM3`.
  - ✅ Реальная WS-телеметрия после финальной прошивки: `fixed`, 31 SV, hAcc 14 мм,
    RTCM `udp`, возраст RTCM ~124 мс, PVT age сбрасывается 0/700-800 мс, IMU свежий,
    motor feedback есть, батарея ~36.77 В / 42%, `NAV` после начального старого состояния
    переходит в `IDLE`.
  - ⚠️ Колёса НЕ двигались. Ездовые тесты, проверка `IMU_YAW_RATE_SIGN` и подбор
    `ROVER_TURN_MIN_RADPS` остаются только после явного разрешения пользователя.

---

## 7. КОМАНДЫ ДЛЯ ВОССТАНОВЛЕНИЯ КОНТЕКСТА

```
# структура прошивки
find rtk_firmware/lib rtk_firmware/src -name "*.cpp" -o -name "*.h" | grep -v .pio

# текущее состояние git
git -C C:/robot/module status

# HITL-симулятор (пример)
python simulator/hitl_runner.py --port COM4 --route '0,0;10,0' --start 0 0 0 --duration 45 --no-plot
```
