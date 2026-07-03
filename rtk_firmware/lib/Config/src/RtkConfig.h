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

// BNO085 heading conventions.
// Raw BNO yaw is converted only by ImuMath::imuRawToRobotHeadingDeg():
//   robotHeading = normalize(IMU_ROT_YAW_SIGN * rawYaw +
//                            IMU_ROT_YAW_OFFSET_DEG +
//                            IMU_COMPASS_YAW_ADJUST_DEG)
// Project convention: x=East, y=North, 0=North, 90=East, clockwise-positive.
#define IMU_ROT_YAW_SIGN          (-1.0f)
#define IMU_ROT_YAW_OFFSET_DEG    172.0f
#define IMU_COMPASS_YAW_ADJUST_DEG  7.7f

// Startup/runtime absolute heading gates.
#define IMU_ABS_YAW_MAX_ACC_RAD              0.5f
#define IMU_ABS_CANDIDATE_RESET_DELTA_DEG   12.0f
#define IMU_ABS_STARTUP_STABLE_MS         1500u
#define IMU_ABS_STARTUP_MAX_DELTA_DEG       10.0f
#define IMU_STARTUP_WAIT_MS               4000u
#define IMU_STARTUP_STATIONARY_MS         1200u
#define IMU_STARTUP_MAX_JUMP_DEG             5.0f
#define IMU_MAG_DISTURBANCE_RATIO            0.25f

// Sign of BNO085 yaw-rate relative to heading (0..360, clockwise-positive).
// Verify on hardware: turn the rover clockwise, yawRateDps must be > 0.
#define IMU_YAW_RATE_SIGN  (-1)

// ---------------- Serial baud ----------------
#define SERIAL_BAUD       115200
#define F9P_BAUD          38400    // F9P дефолт — важно для cold start

// ---------------- Networking ----------------
#define WIFI_SSID         "Xiaomi_D465"
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
#define BASE_SURVEY_FALLBACK_ACC_M 25.0f
#define BASE_SURVEY_FALLBACK_MIN_S 30
#define BASE_SURVEY_FALLBACK_AFTER_S 30
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
// Длина гусеницы по внешнему краю (примерно): 42 + 18 + 42 + 18 = 120 см,
// но эффективная рабочая длина (без скольжения) грубо ~0.6м. Калибруется
// по одометрии: на 1 оборот шкива гусеница проходит ~wheel_circumference.
// Для старта — 0.6м. Точная калибровка по HITL.
#define ROVER_WHEEL_CIRCUM_M 0.6f
#define ROVER_HOVER_MEAS_TO_RPM  6.0f   // speed_meas / 6 ≈ RPM (зависит от прошивки hoverboard)
#define ROVER_MAX_PWM       70
#define ROVER_INPUT_DIV     6
#define ROVER_CMD_TIMEOUT_MS 400
// M,55,55 is divided by ROVER_INPUT_DIV and produces about 9% at the motor
// layer. Field testing established that as the normal straight-drive command.
// Autonomous 0.18 m/s therefore maps to ~9%; the hard 0.25 m/s cap maps to 12%.
#define ROVER_AUTO_MAX_PERCENT 12
#define ROVER_MAX_SPEED_MPS  0.25f
// FLOAT едет с той же скоростью что и FIXED (на FLOAT hAcc уже ~2см, безопасно) —
// чтобы автономка ехала одинаково как ручной 16% независимо от RTK-режима.
#define ROVER_FLOAT_SPEED    0.15f
#define ROVER_DEGRADED_SPEED 0.04f
#define ROVER_HOLD_SPEED     0.02f
// Hoverboard-keepalive: если idle (нет команд) дольше этого времени —
// шлём 3% команды на 2 сек чтобы плата не уходила в сон.
#define ROVER_KEEPALIVE_IDLE_MS    180000u   // 3 минуты
#define ROVER_KEEPALIVE_PULSE_PCT  3
#define ROVER_KEEPALIVE_PULSE_MS   2000u
#define ROVER_INITIAL_HEADING_DEG 124.0f  // fallback, если IMU не дала heading при старте
#define ROVER_ARRIVAL_RADIUS 0.10f
#define ROVER_ARRIVAL_CONFIRM_S 1.5f
// Lookahead для Stanley: целевая точка берётся НА СЕГМЕНТЕ на расстоянии lookahead
// вперёд по сегменту, а не сам waypoint — это устраняет прыжки heading на handoff
// и осцилляции в окрестности WP. (Sunray: LOOOKAHEAD=1.0m.)
#define ROVER_LOOKAHEAD_M    0.70f
// Stanley controller (Sunray: STANLEY_CONTROL_P=3.0, K=1.0):
//   w = k_heading*headingErr_rad + atan(k_crosstrack*ct / (k_soft + |v|))
// Усиления увеличены vs старой версии (0.55 / 0.08) — старые были насыщены clamp.
#define ROVER_K_HEADING       1.00f
#define ROVER_K_CROSSTRACK    0.25f
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

// nav-v2-simple: deliberately small waypoint controller for first field routes.
#define ROVER_V2_ARRIVAL_RADIUS_M          0.35f
#define ROVER_V2_TURN_IN_PLACE_DEG         20.0f
#define ROVER_V2_TURN_RADPS                 0.95f
#define ROVER_V2_FORWARD_MPS                0.18f
#define ROVER_V2_HEADING_KP_RADPS_PER_DEG   0.018f
#define ROVER_V2_MAX_CORRECTION_RADPS       0.35f
#define ROVER_V2_TURN_WATCHDOG_MS            2000u
#define ROVER_V2_TURN_MIN_DELTA_DEG          5.0f
#define ROVER_V2_TURN_MIN_ERROR_IMPROVE_DEG  3.0f
#define ROVER_GO_DEFAULT_DISTANCE_M         1.0f

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
#define SAFE_REJECTED_POSITION_FIXES_MAX 5
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
