// RtkConfig.h - все пины, baud, константы. Один на rover и base.

#pragma once
#include <Arduino.h>

// ---------------- Pins ----------------
#define PIN_F9P_RX         4    // ESP32 RX  <- F9P TX
#define PIN_F9P_TX         5    // ESP32 TX  -> F9P RX
#define PIN_MOTOR_RX      16    // ESP32 RX  <- hoverboard TX  (Serial2)
#define PIN_MOTOR_TX      17    // ESP32 TX  -> hoverboard RX  (Serial2)
#define PIN_IMU_SDA       21
#define PIN_IMU_SCL       22
#define PIN_RELAY_ATTACH  32
#define PIN_RELAY_MOUNT   33
// Пины 25/26/27 — раньше I2S звук (sound.ino). Звук не используется, пины свободны.

// Знак оси yaw-rate BNO085 относительно heading (0..360, по часовой).
// +1 или -1 — ВЫВЕРИТЬ на железе: повернуть робота по часовой, yawRateDps должен быть > 0.
#define IMU_YAW_RATE_SIGN  (+1)

// ---------------- Serial baud ----------------
#define SERIAL_BAUD       115200
#define F9P_BAUD          38400    // F9P дефолт — важно для cold start

// ---------------- Networking ----------------
#define WIFI_SSID         "Xiaomi_6A92"
#define WIFI_PASS         "17762646"
#define ROVER_IP          "192.168.31.222"
#define BASE_IP           "192.168.31.207"
#define WS_PORT           81
#define RTCM_TCP_PORT     2102
#define RTCM_UDP_PORT     2101
#define RTCM_UDP_FALLBACK 2103

// ---------------- Base/rover behaviour ----------------
#define BASE_SURVEY_ACC_M   2.0f
#define BASE_SURVEY_MIN_S   60
#define BASE_TCP_RX_TIMEOUT_MS 4000

// ---------------- Hoverboard motor protocol (эталон: sound.ino, рабочий) ----------------
#define HOVER_BAUD            115200
#define HOVER_START_FRAME     0xABCD
#define HOVER_MAX_CMD         300     // шкала команды платы (steer/speed)
#define HOVER_MAX_PERCENT     70      // -70..70 % после ограничения
#define HOVER_SEND_MS         20      // период отправки команды
#define HOVER_CMD_TIMEOUT_MS  400     // нет команды -> плавный стоп (failsafe)
#define HOVER_SLEW_SPEED      4       // slew в домене команды за посылку
#define HOVER_SLEW_STEER      6
#define HOVER_BAT_CELLS       10      // 10S
#define HOVER_BAT_VOLT_SCALE  0.01f

// ---------------- Rover behaviour ----------------
#define ROVER_WHEELBASE_M   0.38f
#define ROVER_MAX_PWM       70
#define ROVER_INPUT_DIV     2
#define ROVER_CMD_TIMEOUT_MS 400
#define ROVER_MAX_SPEED_MPS  0.25f
#define ROVER_FLOAT_SPEED    0.12f
#define ROVER_DEGRADED_SPEED 0.07f
#define ROVER_HOLD_SPEED     0.03f
#define ROVER_INITIAL_HEADING_DEG 176.0f  // физ. старт-курс робота (выставлен на улице)
#define ROVER_ARRIVAL_RADIUS 0.10f
#define ROVER_ARRIVAL_CONFIRM_S 1.5f
#define ROVER_LOOKAHEAD_MIN  0.5f
#define ROVER_LOOKAHEAD_MAX  1.2f
#define ROVER_LOOKAHEAD_GAIN 1.5f
#define ROVER_K_HEADING      1.5f
#define ROVER_K_CROSSTRACK   0.15f
#define ROVER_TURN_THRESH_DEG 25.0f
#define ROVER_ROTATE_SPEED_RADPS 0.50f
#define ROVER_MAX_ANGULAR_RADPS 1.20f
// Разворот на месте: гистерезис входа/выхода + МИНИМАЛЬНАЯ угл. скорость (пробить трение гусениц).
// ВЫВЕРИТЬ TURN_MIN на железе: робот должен реально поворачиваться, а не дёргаться на месте.
#define ROVER_TURN_IN_PLACE_ENTER_DEG 35.0f
#define ROVER_TURN_IN_PLACE_EXIT_DEG  12.0f
#define ROVER_TURN_MIN_RADPS          0.45f
#define ROVER_STANLEY_SOFT_SPEED 0.08f
#define ROVER_HANDOFF_FRAC    0.10f   // handoff = segLen * this
#define ROVER_HANDOFF_MAX_M   0.30f
#define ROVER_HANDOFF_MIN_M   0.08f

// ---------------- Telemetry period ----------------
#define TEL_PERIOD_MS       200
#define GPS_PERIOD_MS       200
#define RTCM_PERIOD_MS     1000
#define NAV_PERIOD_MS       500
#define IMU_PERIOD_MS        50

// ---------------- Safety thresholds ----------------
#define SAFE_RTK_AGE_MS    5000
#define SAFE_PVT_AGE_MS    1000
#define SAFE_IMU_AGE_MS     200
#define SAFE_NAV_TIMEOUT_MS 8000
#define SAFE_HACC_FIXED_M  0.05f
#define SAFE_HACC_FLOAT_M  0.20f
#define SAFE_HACC_HOLD_M   1.00f
#define SAFE_NUM_SV         10
#define SAFE_PDOP_MAX       4.0

// ---------------- Logging tag ----------------
#ifndef ROLE_NAME
#  ifdef ROLE_ROVER
#    define ROLE_NAME "ROVER"
#  elif defined(ROLE_BASE)
#    define ROLE_NAME "BASE"
#  else
#    define ROLE_NAME "UNK"
#  endif
#endif
