// RoverDebug.h — мосты между WsServer и rover.cpp для отладочных команд.
// Позволяет из приложения (через WebSocket) дёргать те же команды, что
// доступны из Serial Monitor: CAL / GO / LOG,0 / LOG,1.

#pragma once
#include <Arduino.h>

namespace roverdbg {

// CAL: reseed estimator from absolute IMU heading if it is ready.
bool handleCal();

// GO: CAL + маршрут (0,0) → (0,3) с boundary 4×4 вокруг + старт.
// Возвращает true если маршрут сформирован и запущен.
bool handleGo();
bool handleGoForward(float distanceM);
bool handleGoNorth(float distanceM);

// LOG,0 / LOG,1: гасит/включает периодический лог в rover.cpp.
void setLogEnabled(bool enabled);

// Краткая диагностика одной строкой — для ответа в WS когда Serial недоступен.
// Пример: "sol=1 hAcc=0.030 imu=125.4 mag=1 acc=0.05 imuAge=18 safety=2 rtk_float_ok".
String diagLine();
String imuZeroLine();
String imuDiagLine();
String imuStatusLine();
String imuCalStartLine();
String imuCalSaveLine();
String imuCalClearLine();
String imuTareYawLine();
String imuTarePersistLine();

}  // namespace roverdbg
