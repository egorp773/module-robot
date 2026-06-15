# EKF Flash & Field Test Guide

## Что залито
- Commit `f79db5a`: добавлен 5-state EKF (RtkEkf.h/.cpp) с интеграцией в StateEstimator
- Commit `36c2992`: ekf_test.py (юнит-тесты EKF — все 4 прошли)
- Сборка `rover` собирается: RAM 36.7%, Flash 73.6%

## Что делает EKF
- **Predict**: одометрия (v, omega от hoverboard feedback) + IMU gyro (heading)
- **Update**: RTK fix (x, y) + GPS motion heading
- **Outlier rejection**: Mahalanobis gate, chi^2 = 9.21 (3σ)
- **Dead-reckoning**: при пропадании GPS heading держится за счёт IMU, позиция — за счёт одометрии

## Заливка на ESP32

```powershell
cd C:\robot\module\rtk_firmware
C:\Users\vasil\AppData\Local\Programs\Python\Python38\Scripts\pio.exe run -e rover -t upload
```

ESP32 должен быть подключён по USB. COM-порт определяется автоматически (или `pio run -e rover -t upload --upload-port COM6`).

## Smoke-test в гараже (перед полем)

1. Залить прошивку
2. Подключиться Serial monitor (115200 бод) к ESP32
3. Дождаться `[ROVER] ready`
4. Подключиться к WiFi `Xiaomi_6A92` (пароль `17762646`)
5. Открыть Flutter-приложение, дождаться соединения
6. **Ждать RTK_FIXED** в `TEL,...` (поле `sol=2`)
7. Нажать NAV_START
8. В Serial должно появиться:
   ```
   OK,NAV_START
   ```
9. Робот не должен ехать пока не нарисуете маршрут. Нарисовать **3 точки по прямой** (тест)
10. Нажать START в приложении
11. Робот должен проехать 3 точки **без рывков** по карте

## Что смотреть в Serial

```
TEL,fix=2 sol=2 hAcc=0.014 ...   <- RTK_FIXED, точность 14мм
NAV,state=1 wpIdx=0 wpTotal=3    <- навигация активна
GPS,x=0.05 y=0.02 heading=176    <- координаты EKF, не скачут
IMU,yaw=176.2 age=10ms           <- IMU свежий
```

Если `GPS,x=... y=...` скачет больше ±5 см между пакетами — что-то не так с EKF.
Если `heading=...` прыгает ±5° — пересмотреть шумы `kQ_heading`.

## Полевой тест на 96 точках

1. Нарисовать периметр в Flutter-app
2. Запустить plan route → получить 96 точек
3. Дождаться RTK_FIXED
4. **Проверить**: в Serial должна быть `IMU,age=...` (IMU подключён) и `MOTOR,speedL=...` (hoverboard feedback идёт)
5. Если есть `GPS,yaw=...` — это GPS motion heading, для информации
6. NAV_START
7. Наблюдать за картой: траектория должна идти **ровно по точкам**, не петлять
8. Если траектория петляет — замерить `ROVER_WHEEL_CIRCUM_M` (сейчас 0.6, может быть 0.5 или 0.7)

## Если робот петляет или координаты скачут

1. Открыть Serial monitor
2. Найти `GPS,x=... y=...` и `TEL,hAcc=...`
3. Если hAcc > 0.05 → проблема с RTK, проверить base-ESP32, RTCM
4. Если hAcc < 0.02 но x,y скачут → проблема в EKF, тюнить `kQ_x`, `kQ_y`
5. Если heading прыгает → тюнить `kQ_heading`

## Тюнинг (если потребуется)

В `RtkEkf.h` (публичные static):
```cpp
Ekf::kQ_x        = 0.05f;   // ↑ если скачут координаты, ↓ если дёргается
Ekf::kQ_y        = 0.05f;
Ekf::kQ_heading  = 0.02f;   // ↑ если heading плывёт, ↓ если прыгает
Ekf::kQ_v        = 0.10f;
Ekf::kQ_omega    = 0.50f;
```

В `RtkConfig.h`:
```cpp
ROVER_WHEEL_CIRCUM_M 0.6f   // ↑ если робот недоезжает, ↓ если переезжает
ROVER_HOVER_MEAS_TO_RPM 6.0f
```

## Известные ограничения

- EKF требует **постоянной одометрии от hoverboard**. Если feedback пропадает, dead-reckoning держит позицию по последней скорости (1-2 сек до warning).
- `ROVER_WHEEL_CIRCUM_M = 0.6` — приблизительное значение. Точная калибровка по HITL после первого полевого теста.
- EKF не учитывает проскальзывание гусениц. На мокрой траве/льду будет drift.
- Лидар/sonar **не подключены** — обход препятствий не работает, только маршрут.

## Что НЕ сломается

- F9P-парсер, RTCM-релей — не трогал
- WebSocket-протокол — не трогал
- Route pipeline — не трогал
- Flutter-приложение — не трогал
- Старый код (там где был `seedHeadingDeg`) — оставлен как fallback

Всё, что было в MEMORY.md как «работающее» — осталось работать.
