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
#define IMU_YAW_RATE_SIGN  (-1)

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
// Survey-In: ПРОВЕРЕННЫЕ значения из старой рабочей версии (до рерайта c455a76), когда
// RTK FIXED работал: точность 1.0м / минимум 300с. Codex сломал, поставив 12м → база
// фиксировалась за 0.7с с ошибкой 8м. 3см не сходится вообще (Survey-In логарифмический).
// Для FIXED абсолютная точность базы не критична — RTK относителен; важна стабильность.
#define BASE_SURVEY_ACC_M   1.0f
#define BASE_SURVEY_MIN_S   300
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
// Скорость автономки = ручной 16%. Расчёт: ручной M,16 → /ROVER_INPUT_DIV(2) = 8% →
// pctToHover(8,8) = 8*2*300/(2*70) = команда платы 34/300. Чтобы автономка на полном ходу
// давала ту же команду 34, нужно leftPct=8 при baseSpeed=ROVER_MAX_SPEED_MPS:
//   scale = ROVER_AUTO_MAX_PERCENT / ROVER_MAX_SPEED_MPS = 8 / 0.25 = 32
//   baseSpeed 0.25 * 32 = 8 → pctToHover(8,8) = 34. Совпадает с ручной 16%.
#define ROVER_AUTO_MAX_PERCENT 8
#define ROVER_MAX_SPEED_MPS  0.25f
// FLOAT едет с той же скоростью что и FIXED (на FLOAT hAcc уже ~2см, безопасно) —
// чтобы автономка ехала одинаково как ручной 16% независимо от RTK-режима.
#define ROVER_FLOAT_SPEED    0.25f
#define ROVER_DEGRADED_SPEED 0.07f
#define ROVER_HOLD_SPEED     0.03f
#define ROVER_INITIAL_HEADING_DEG 176.0f  // физ. старт-курс робота (выставлен на улице)
#define ROVER_ARRIVAL_RADIUS 0.10f
#define ROVER_ARRIVAL_CONFIRM_S 1.5f
// Lookahead для Stanley: целевая точка берётся НА СЕГМЕНТЕ на расстоянии lookahead
// вперёд по сегменту, а не сам waypoint — это устраняет прыжки heading на handoff
// и осцилляции в окрестности WP. (Sunray: LOOOKAHEAD=1.0m.)
#define ROVER_LOOKAHEAD_M    0.70f
// Stanley controller (Sunray: STANLEY_CONTROL_P=3.0, K=1.0):
//   w = k_heading*headingErr_rad + atan(k_crosstrack*ct / (k_soft + |v|))
// Усиления увеличены vs старой версии (0.55 / 0.08) — старые были насыщены clamp.
#define ROVER_K_HEADING       2.20f
#define ROVER_K_CROSSTRACK    0.80f
#define ROVER_STANLEY_SOFT_SPEED 0.30f
#define ROVER_TURN_THRESH_DEG 25.0f
#define ROVER_ROTATE_SPEED_RADPS 0.40f
#define ROVER_MAX_ANGULAR_RADPS 0.80f   // ~46°/s — хватает чтобы выйти из 60° ошибки
// Разворот на месте: гистерезис входа/выхода + МИНИМАЛЬНАЯ угл. скорость (пробить трение гусениц).
// ВЫВЕРИТЬ TURN_MIN на железе: робот должен реально поворачиваться, а не дёргаться на месте.
#define ROVER_TURN_IN_PLACE_ENTER_DEG 70.0f
#define ROVER_TURN_IN_PLACE_EXIT_DEG  18.0f
#define ROVER_TURN_MIN_RADPS          0.10f
#define ROVER_TURN_IN_PLACE_TIMEOUT_MS 2500u
#define ROVER_TURN_IN_PLACE_COOLDOWN_MS 1200u
#define ROVER_HANDOFF_FRAC    0.30f   // handoff = segLen * this
#define ROVER_HANDOFF_MAX_M   0.80f
#define ROVER_HANDOFF_MIN_M   0.20f
#define ROVER_BOUNDARY_TOLERANCE_M 0.10f
#define ROVER_BOUNDARY_SAMPLE_M    0.15f
// Crosstrack guard (не fault — мягкая коррекция через 0.3м, fault только через 1.0м).
#define ROVER_CROSSTRACK_SOFT_M    0.30f
#define ROVER_CROSSTRACK_HARD_M    1.00f
#define ROVER_STUCK_TIMEOUT_MS     4000u
#define ROVER_STUCK_MIN_CMD_MPS    0.05f
#define ROVER_STUCK_MIN_MOVE_M     0.05f
// Recovery: после 3 fault подряд в одном WP — замедляемся до degraded и пробуем
// переиграть waypoint, а не финишируем маршрут.
#define ROVER_FAULT_RECOVERY_COUNT 3

// ---------------- Telemetry period ----------------
#define TEL_PERIOD_MS       200
#define GPS_PERIOD_MS       200
#define RTCM_PERIOD_MS     1000
#define NAV_PERIOD_MS       500
#define IMU_PERIOD_MS        50

// ---------------- Safety thresholds ----------------
#define SAFE_RTK_AGE_MS    5000
#define SAFE_PVT_AGE_MS    1000
#define SAFE_ACCEPTED_POS_AGE_MS 1500
#define SAFE_HEADING_AGE_MS 1500
#define SAFE_IMU_AGE_MS     200
#define SAFE_NAV_TIMEOUT_MS 8000
#define SAFE_HACC_FIXED_M  0.02f
#define SAFE_HACC_FLOAT_M  0.02f
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
