# HITL handoff

Работа велась в `C:\robot\module`.

## Что уже сделано

- В `rtk_firmware/lib/NavigationCore/NavigationCore.h` добавлен `ALIGN_FIRST`.
  - Если `abs(headingError) > 15 deg`, `forwardCmd = 0`.
  - Робот крутится на месте: `left = +turnCmd`, `right = -turnCmd`.
  - Вперед едет только после выравнивания.
- В `rtk_firmware/src/rover.cpp` добавлен USB HITL:
  - `SIM_START,x,y`
  - `SIM_ROUTE,x,y;x,y;...`
  - `GPS,x,y,heading,speed,quality`
  - ESP32 отвечает `MOTORS,left,right`
  - Также печатает `SIM_DBG,...`.
- В `simulator/hitl_runner.py` добавлена работа с реальным ESP32 по `COM4`, физика гусениц, CSV/PNG лог, расчет `Mean stripe cross-track`.

## Проверенные результаты

Одна полоса:

```powershell
python simulator\hitl_runner.py --port COM4 --route '0,0;10,0' --start 0 0 0 --duration 45 --name hitl_strip --no-plot
```

Результат был хороший:

```text
Final: x=10.005 y=-0.001 miss=0.005 m (0.5 cm)
Mean stripe cross-track: 0.020 m (2.0 cm)
```

Две полосы:

```powershell
python simulator\hitl_runner.py --port COM4 --route '0,0;10,0;10,0.42;0,0.42' --start 0 0 0 --duration 75 --name hitl_serpentine --no-plot
```

Результат плохой по маршруту:

```text
Final: x≈10.0 y≈0.33 miss≈10 m
Mean stripe cross-track: 2.9 cm
```

Вывод: первая полоса ровная, но робот слишком долго пытается парковаться в промежуточную точку `(10,0)` и не переходит нормально на connector/следующую полосу.

## Где остановились

В `rtk_firmware/src/rover.cpp` уже добавлена константа:

```cpp
static constexpr float INTERMEDIATE_ADVANCE_MAX_M = 0.35f;  // Segment handoff distance for mowing rows
```

Но логика ее использования еще не дописана.

## Следующий патч

В `navUpdate()` найти блок:

```cpp
const bool finalWaypoint = (g_routeIndex + 1) >= g_routeCount;
const bool intermediateReached =
  !finalWaypoint && distToTarget < (ARRIVAL_DIST_M * 3.0f);

if (intermediateReached || checkArrival(target, 0, g_headingError)) {
```

Заменить расчет `intermediateReached` на:

```cpp
float intermediateAdvanceM = ARRIVAL_DIST_M * 3.0f;
if (g_routeIndex > 0) {
  const LocalCoords prev = g_route[g_routeIndex - 1].pos;
  const float segLen = hypotf(target.x - prev.x, target.y - prev.y);
  intermediateAdvanceM = constrain(
    segLen * 0.08f,
    ARRIVAL_DIST_M * 3.0f,
    INTERMEDIATE_ADVANCE_MAX_M
  );
}

const bool intermediateReached =
  !finalWaypoint && distToTarget < intermediateAdvanceM;
```

Идея: промежуточные точки змейки должны быть handoff-точками, а не финальной парковкой. Для длинной 10 м полосы переключение будет примерно за `0.35 м`, для короткого connector 0.42 м - примерно за `0.15 м`.

## Команды после патча

Собрать и прошить:

```powershell
cd C:\robot\module\rtk_firmware
pio run -e rover -t upload --upload-port COM4
```

Проверить одну полосу:

```powershell
cd C:\robot\module
python simulator\hitl_runner.py --port COM4 --route '0,0;10,0' --start 0 0 0 --duration 45 --name hitl_strip --no-plot
```

Проверить две полосы:

```powershell
python simulator\hitl_runner.py --port COM4 --route '0,0;10,0;10,0.42;0,0.42' --start 0 0 0 --duration 75 --name hitl_serpentine --no-plot
```

Цель:

```text
Mean stripe cross-track < 0.05 m
```

Файлы результатов:

- `C:\robot\module\.pio\build_root\hitl_strip_trace.csv`
- `C:\robot\module\.pio\build_root\hitl_strip_trajectory.png`
- `C:\robot\module\.pio\build_root\hitl_serpentine_trace.csv`
- `C:\robot\module\.pio\build_root\hitl_serpentine_trajectory.png`
