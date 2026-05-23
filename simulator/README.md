# RTK Robot HITL Simulator

Настоящий Hardware-in-the-Loop симулятор для ESP32 гусеничного робота.

## Архитектура

```
┌─────────────────────────────────────────────────────────────────┐
│                        Python (Симулятор)                       │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐        │
│  │  Physics      │  │  GPS         │  │  Visualization│        │
│  │  Simulation   │  │  Emulator    │  │              │        │
│  └──────────────┘  └──────────────┘  └──────────────┘        │
│         ↑                 ↑                 ↑                   │
│         │                 │                 │                   │
└─────────┼─────────────────┼─────────────────┼───────────────────┘
          │                 │                 │
    Motor PWM          NMEA GPS           Telemetry
          ↑                 ↓                 ↑
          │   WebSocket     │     TCP         │
          │   ←──────────   │   ←──────────   │
          ▼                 │                 │
┌─────────────────────────────────────────────────────────────────┐
│                        ESP32 (Реальный код)                      │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐        │
│  │  Navigation   │  │  Motor       │  │  WebSocket   │        │
│  │  Pure Pursuit │  │  Control     │  │  Server      │        │
│  └──────────────┘  └──────────────┘  └──────────────┘        │
│                                                                  │
│  Real firmware runs here - NO navigation in Python!              │
└─────────────────────────────────────────────────────────────────┘
```

## Установка

```bash
cd simulator
pip install numpy matplotlib websockets pyserial
```

## Использование

### SITL режим (без ESP32)

```bash
# Простая симуляция
python simulator.py --sitl --waypoints 0,0,5,0

# С визуализацией
python simulator.py --sitl --waypoints 0,0,5,0,5,5,0,5

# Без GPS шума (идеальные условия)
python simulator.py --sitl --waypoints 5,0 --no-noise
```

### HITL режим (с ESP32)

```bash
# Подключение к ESP32
python simulator.py --hitl --ip 192.168.31.222

# С кастомными портами
python simulator.py --hitl --ip 192.168.31.222 --gps-port 8888 --ws-port 81
```

### Автоматические тесты

```bash
# 20 тестов прямое направление
python auto_test.py --count 20 --scenario straight

# 50 случайных направлений
python auto_test.py --count 50 --scenario random

# Все сценарии
python auto_test.py --count 100 --scenario all

# В headless режиме с JSON выводом
python auto_test.py --count 50 --json results.json
```

### Дебаг навигации

```bash
# Пошаговый дебаг
python nav_debug.py --target 5,0

# Сохранение лога
python nav_debug.py --target 5,0 --save debug_log.json

# Воспроизведение лога
python nav_debug.py --load debug_log.json
```

## Подключение ESP32

### GPS эмуляция

ESP32 ожидает GPS данные через UART (аппаратный serial).

**Вариант 1: TCP-to-Serial мост (рекомендуется)**

На ESP32 нужно добавить TCP сервер для GPS данных. Или использовать USB serial passthrough.

**Вариант 2: Virtual COM Ports**

Используйте COM0COM или similar для создания виртуальной пары COM портов.

### WebSocket для телеметрии

ESP32 уже имеет WebSocket сервер на порту 81. Симулятор подключается для:
- Чтения телеметрии (GPS, NAV, MOTOR)
- Отправки команд (START, GO_TO, ROUTE_*)

## Физика робота

Параметры из реального робота:

```python
WHEELBASE = 0.38  # метры между гусеницами
MAX_SPEED = 0.5  # м/с при 70% PWM
GPS_NOISE = 0.015  # метры, 1-sigma (RTK Fixed)
```

### Формулы

```python
# PWM → скорость
v_left = (left_pwm / 70.0) * 0.5
v_right = (right_pwm / 70.0) * 0.5

# Кинематика
v = (v_left + v_right) / 2
omega = (v_right - v_left) / wheelbase

# Позиция
x += v * sin(heading) * dt
y += v * cos(heading) * dt
heading += degrees(omega) * dt
```

## GPS эмуляция

### NMEA формат

Симулятор генерирует стандартные NMEA строки:

```
$GPGGA,120000.00,5500.0000,N,03700.0000,E,4,15,0.5,100.0,M,0.0,M,0.0,0000*XX
$GPRMC,120000.00,A,5500.0000,N,03700.0000,E,0.5,45.0,010124,,W*XX
$GPVTG,45.0,T,,M,0.5,N,0.9,K*XX
```

### RTK fix quality

| Quality | Описание | hAcc |
|---------|----------|------|
| 0 | No fix | - |
| 1 | GPS fix | ~2m |
| 4 | RTK Fixed | ~15mm |
| 5 | RTK Float | ~50mm |

## Управление

### Клавиатура (в визуализации)

| Клавиша | Действие |
|--------|----------|
| Space | Пауза/Продолжение |
| R | Сброс симуляции |
| Q | Выход |
| S | Старт навигации |

### Мышь

- **ЛКМ** на карте — задать цель
- **ПКМ** — информация о точке

## API для Claude Code

### Headless режим

```bash
python simulator.py --headless --json
```

Возвращает JSON:

```json
{
  "success": true,
  "final_position": [5.02, -0.01],
  "target_position": [5.0, 0.0],
  "error_m": 0.021,
  "time_s": 12.5,
  "oscillation_count": 3,
  "final_state": "ARRIVED"
}
```

### Автотесты

```bash
# Запуск тестов
python auto_test.py --count 100 --json results.json

# Анализ результатов
python auto_test.py --load results.json

# Сравнение двух прогонов
python auto_test.py --compare results1.json --load results2.json
```

## Структура файлов

```
simulator/
├── simulator.py         # Главный симулятор (HITL/SITL)
├── physics.py          # Физика робота (dikferential drive)
├── gps_emulator.py     # Генератор fake GPS NMEA
├── serial_emulator.py  # Virtual serial bridge
├── auto_test.py        # Автоматические тесты
├── nav_debug.py        # Пошаговый дебаггер
├── requirements.txt    # Зависимости
└── README.md           # Этот файл
```

## Тестирование навигации

### Сценарии

| Сценарий | Описание |
|----------|---------|
| straight | Прямо 5м |
| angle | 45° угол (3м × 3м) |
| random | Случайные цели |
| multi_wp | Квадратный маршрут |

### Метрики

- **success_rate** — % попаданий в 10см
- **avg_error** — средняя ошибка, м
- **max_error** — максимальная ошибка, м
- **oscillation_count** — количество осцилляций
- **avg_time** — среднее время прохождения, с

## Troubleshooting

### ESP32 не подключается

1. Проверьте IP адрес
2. Проверьте что ESP32 в той же сети
3. Проверьте что WebSocket сервер запущен

### GPS данные не доходят

1. Проверьте что TCP порт открыт
2. Проверьте firewall
3. Используйте `netstat -an | findstr 8888`

### Симуляция слишком медленная

```bash
# Увеличьте скорость
python simulator.py --sitl --speed 2.0
```

## Расширение

### Добавление нового сценария

```python
# В auto_test.py
elif scenario == 'my_scenario':
    for i in range(count):
        result = self.run_single_test(
            i, scenario,
            start=(x1, y1),
            target=(x2, y2)
        )
```

### Интеграция с CI/CD

```python
# В вашем CI скрипте
result = subprocess.run([
    'python', 'simulator.py',
    '--headless', '--json'
], capture_output=True)

data = json.loads(result.stdout)
assert data['success'], f"Simulation failed: {data}"
assert data['error_m'] < 0.2, f"Error too large: {data['error_m']}m"
```
