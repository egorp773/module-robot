// RoverDebug.h — мосты между WsServer и rover.cpp для отладочных команд.
// Позволяет из приложения (через WebSocket) дёргать те же команды, что
// доступны из Serial Monitor: CAL / GO / LOG,0 / LOG,1.

#pragma once
#include <Arduino.h>

namespace roverdbg {

// CAL / IMU_CAL_START: start BNO085 dynamic calibration. Allowed from
// every state except "IMU not responding" — this is the operation that
// turns ABSOLUTE_UNCALIBRATED into ABSOLUTE_OK, so the old
// `IMU_ABSOLUTE_OK` gate was a bug.
bool handleCal();
// CAL_HEADING_SEED: strictly-gated — reseed StateEstimator from IMU.
// Only valid when yaw is absolute. Exposed only as `CAL_HEADING_SEED`
// on Serial, never as `CAL`.
bool handleHeadingSeed();
bool handleCalStart();
bool handleCalSave();
bool handleGoLShape(float firstM, float turnDeg, float secondM);
String handleGoLShapeDebugLine(float firstM, float turnDeg, float secondM);
String handleGoSquareDebugLine(float sideM);

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
String imuCalStatusLine();
String imuCalClearLine();
String imuTareYawLine();
String imuTarePersistLine();
String imuSetTrueHeadingLine(float trueHeadingDeg);
String imuClearHeadingCorrectionLine();
String imuHeadingTestLine();
String imuTrustCurrentHeadingOnceLine();
String imuClearManualHeadingTrustLine();
String autoAlignHeadingByRtkLine();
String autoAlignHeadingByRtkLineWs();
String headingStatusLine();
String clearHeadingTrustLine();
String navStartAutoAlignLine();
String navStartAutoAlignLineWs();
String handleNavStartLine();
String handleNavPauseLine();
String handleNavResumeLine();
String queueNavStartLineWs();
String queueNavPauseLineWs();
String queueNavResumeLineWs();
String queueNavStopLineWs();
String queueStopLineWs();
void queueWsDisconnectStop();
String queueGoForwardWs(float distanceM);
String queueGoNorthWs(float distanceM);
String queueGoLShapeWs(float firstM, float turnDeg, float secondM,
                       bool debug);
String queueGoSquareDebugWs(float sideM);
String handleStopLine();
bool routeExecutorActive();

}  // namespace roverdbg
