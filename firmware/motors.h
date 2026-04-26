#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

void motors_init();
void motors_update_ramp();
void motors_send();
void motors_receive_feedback();
void motors_request_smooth_stop(const char* reason);
void motors_check_failsafe();

// Целевые скорости (задаются из nav или ручного управления)
extern volatile int16_t g_targetLeft;
extern volatile int16_t g_targetRight;

// Текущие значения после ramping
extern int16_t g_curLeft;
extern int16_t g_curRight;

// Последняя команда (для failsafe таймаута)
extern volatile uint32_t g_lastCmdMs;

// Телеметрия батареи от контроллера
extern volatile bool g_haveFeedback;
extern float g_batVoltFiltered;
extern int16_t g_batVoltage;
extern int16_t g_boardTemp;

#endif // MOTORS_H
