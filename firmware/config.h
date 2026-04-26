#ifndef CONFIG_H
#define CONFIG_H

// =======================
// Пины
// =======================
#define PIN_GPS_RX        4     // ESP32 RX ← F9P TX/MISO
#define PIN_GPS_TX        5     // ESP32 TX → F9P RX/MOSI
#define PIN_MOTOR_RX      16    // ESP32 RX ← Мотор гироскутера
#define PIN_MOTOR_TX      17    // ESP32 TX → Мотор гироскутера
#define PIN_IMU_SDA       21
#define PIN_IMU_SCL       22
#define PIN_I2S_DIN       27
#define PIN_I2S_BCLK      26
#define PIN_I2S_LRCK      25
#define PIN_RELAY_ATTACH  32    // active HIGH
#define PIN_RELAY_MOUNT   33    // active HIGH

// =======================
// UART
// =======================
#define GPS_BAUD          38400   // F9P по умолчанию
#define MOTOR_BAUD        115200

// =======================
// WiFi
// =======================
#define WIFI_SSID         "Robot"
#define WIFI_PASS         "CHANGE_ME_MIN_8_CHARS"
#define WS_PORT           81
#define RTCM_UDP_PORT     2101

// =======================
// Навигация
// =======================
#define NAV_UPDATE_MS     100     // PID цикл 10 Гц
#define GPS_BROADCAST_MS  200     // Отправка GPS в приложение 5 Гц
#define IMU_BROADCAST_MS  200     // Отправка IMU в приложение 5 Гц
#define NAV_BROADCAST_MS  500     // Отправка NAV в приложение 2 Гц
#define ARRIVAL_RADIUS    0.15f   // Радиус достижения точки, метры
#define MAX_WAYPOINTS     500
#define NOMINAL_SPEED     30      // Базовая скорость (-100..100)
#define MIN_GPS_ACC_MM    500     // Макс. допустимая погрешность GPS, мм
#define GPS_TIMEOUT_MS    2000    // Таймаут потери GPS для NAV_ERROR

// =======================
// PID
// =======================
#define PID_KP_HEADING    2.0f
#define PID_KI_HEADING    0.1f
#define PID_KD_HEADING    0.5f
#define PID_INTEGRAL_MAX  50.0f   // Анти-windup

// =======================
// Моторы (из sound.ino)
// =======================
#define MAX_SPEED_PERCENT 70
#define HOVER_MAX_CMD     300
#define INPUT_DIV         2
#define HOVER_SEND_MS     20
#define CMD_TIMEOUT_MS    400
#define RAMP_UPDATE_MS    20
#define RAMP_STEP_UP_PER_TICK   1
#define RAMP_STEP_DOWN_PER_TICK 1
#define SLEW_SPEED_PER_SEND 4
#define SLEW_STEER_PER_SEND 6
#define START_FRAME       0xABCD

// =======================
// Батарея
// =======================
#define BAT_SEND_MS       500
#define BAT_VOLT_SCALE    0.01f
#define TEMP_SCALE        0.1f
#define BAT_FILTER_ALPHA  0.25f
#define CELL_COUNT        10
#define SEND_BAT_VERBOSE  1

// =======================
// Звук
// =======================
#define SOUND_VOLUME_DEFAULT   80
#define LOW_BATT_REPEAT_MS     15000
#define LOW_BATT_THRESHOLD_PCT 30

// =======================
// Отладка
// =======================
#define DEBUG_PRINT_MS    200

// =======================
// WebSocket
// =======================
#define MAX_WS_MSG        256     // Макс. длина WS сообщения
#define DISC_GRACE_MS     800     // Задержка звука отключения

#endif // CONFIG_H
