#include "motors.h"
#include "config.h"

#define START_FRAME 0xABCD

typedef struct __attribute__((packed)) {
  uint16_t start;
  int16_t  steer;
  int16_t  speed;
  uint16_t checksum;
} SerialCommand;

typedef struct __attribute__((packed)) {
  uint16_t start;
  int16_t  cmd1;
  int16_t  cmd2;
  int16_t  speedR_meas;
  int16_t  speedL_meas;
  int16_t  batVoltage;
  int16_t  boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
} SerialFeedback;

volatile int16_t g_targetLeft = 0;
volatile int16_t g_targetRight = 0;
volatile uint32_t g_lastCmdMs = 0;
int16_t g_curLeft = 0;
int16_t g_curRight = 0;

volatile bool g_haveFeedback = false;
float g_batVoltFiltered = 0.0f;
int16_t g_batVoltage = 0;
int16_t g_boardTemp = 0;

static HardwareSerial MotorSerial(2);
static SerialFeedback Feedback{};
static SerialFeedback NewFeedback{};

static uint8_t fb_idx = 0;
static uint16_t fb_bufStartFrame = 0;
static uint8_t* fb_p = nullptr;
static uint8_t fb_in = 0;
static uint8_t fb_in_prev = 0;

static int16_t g_cmdSpeed = 0;
static int16_t g_cmdSteer = 0;

static uint32_t g_lastRampMs = 0;
static uint32_t g_lastSendMs = 0;

static inline int16_t clampi16(int32_t v, int16_t lo, int16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return (int16_t)v;
}

static inline int16_t stepToward16(int16_t cur, int16_t target, int16_t maxDelta) {
  int32_t diff = (int32_t)target - (int32_t)cur;
  if (diff >  maxDelta) diff =  maxDelta;
  if (diff < -maxDelta) diff = -maxDelta;
  return (int16_t)((int32_t)cur + diff);
}

void motors_init() {
  MotorSerial.begin(MOTOR_BAUD, SERIAL_8N1, PIN_MOTOR_RX, PIN_MOTOR_TX);
  Serial.printf("MOTORS: UART2 %d baud, RX=%d TX=%d\n", MOTOR_BAUD, PIN_MOTOR_RX, PIN_MOTOR_TX);

  g_targetLeft = 0;
  g_targetRight = 0;
  g_curLeft = 0;
  g_curRight = 0;
  g_cmdSpeed = 0;
  g_cmdSteer = 0;

  g_lastRampMs = millis();
  g_lastSendMs = millis();
  g_lastCmdMs = millis();
}

void motors_request_smooth_stop(const char* reason) {
  g_targetLeft = 0;
  g_targetRight = 0;
  g_lastCmdMs = millis();
  if (reason && reason[0]) {
    Serial.printf("MOTORS: smooth stop requested: %s\n", reason);
  }
}

void motors_check_failsafe() {
  uint32_t now = millis();
  if (now - g_lastCmdMs <= CMD_TIMEOUT_MS) return;

  if (g_targetLeft != 0 || g_targetRight != 0) {
    g_targetLeft = 0;
    g_targetRight = 0;
    Serial.println("MOTORS: command timeout, stopping");
  }
}

void motors_update_ramp() {
  uint32_t now = millis();
  uint32_t dt = now - g_lastRampMs;
  if (dt < RAMP_UPDATE_MS) return;

  uint32_t ticks = dt / RAMP_UPDATE_MS;
  g_lastRampMs += ticks * RAMP_UPDATE_MS;

  int16_t tL = clampi16(g_targetLeft,  -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  int16_t tR = clampi16(g_targetRight, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);

  // Ramp step calculation
  auto limitForAxis = [](int16_t cur, int16_t target, int ticks) -> int {
    int absCur = abs((int)cur);
    int absTgt = abs((int)target);
    bool up = (absTgt > absCur);
    int step = up ? RAMP_STEP_UP_PER_TICK : RAMP_STEP_DOWN_PER_TICK;
    int maxDelta = ticks * step;
    if (maxDelta < 1) maxDelta = 1;
    return maxDelta;
  };

  int limL = limitForAxis(g_curLeft,  tL, (int)ticks);
  int limR = limitForAxis(g_curRight, tR, (int)ticks);

  auto stepTowardsLimited = [](int16_t cur, int16_t target, int maxDelta) -> int16_t {
    int diff = (int)target - (int)cur;
    if (diff >  maxDelta) diff =  maxDelta;
    if (diff < -maxDelta) diff = -maxDelta;
    return (int16_t)(cur + diff);
  };

  g_curLeft  = clampi16(stepTowardsLimited(g_curLeft,  tL, limL), -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  g_curRight = clampi16(stepTowardsLimited(g_curRight, tR, limR), -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
}

void motors_send() {
  uint32_t now = millis();
  if (now - g_lastSendMs < HOVER_SEND_MS) return;
  g_lastSendMs = now;

  // Convert left/right to speed/steer
  int16_t leftPct  = clampi16(g_curLeft,  -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  int16_t rightPct = clampi16(g_curRight, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);

  int32_t speedT = (int32_t)(leftPct + rightPct) * (int32_t)HOVER_MAX_CMD / (2 * MAX_SPEED_PERCENT);
  int32_t steerT = (int32_t)(rightPct - leftPct) * (int32_t)HOVER_MAX_CMD / (2 * MAX_SPEED_PERCENT);

  int16_t spT = clampi16(speedT, -HOVER_MAX_CMD, HOVER_MAX_CMD);
  int16_t stT = clampi16(steerT, -HOVER_MAX_CMD, HOVER_MAX_CMD);

  // Slew limiting
  g_cmdSpeed = stepToward16(g_cmdSpeed, spT, SLEW_SPEED_PER_SEND);
  g_cmdSteer = stepToward16(g_cmdSteer, stT, SLEW_STEER_PER_SEND);

  // Send command
  SerialCommand cmd;
  cmd.start = (uint16_t)START_FRAME;
  cmd.steer = g_cmdSteer;
  cmd.speed = g_cmdSpeed;
  cmd.checksum = (uint16_t)(cmd.start ^ cmd.steer ^ cmd.speed);
  MotorSerial.write((uint8_t*)&cmd, sizeof(cmd));
}

void motors_receive_feedback() {
  while (MotorSerial.available()) {
    fb_in = (uint8_t)MotorSerial.read();
    fb_bufStartFrame = ((uint16_t)fb_in << 8) | fb_in_prev;

    if (fb_bufStartFrame == START_FRAME) {
      fb_p = (uint8_t*)&NewFeedback;
      *fb_p++ = fb_in_prev;
      *fb_p++ = fb_in;
      fb_idx = 2;
    } else if (fb_idx >= 2 && fb_idx < sizeof(SerialFeedback)) {
      *fb_p++ = fb_in;
      fb_idx++;
    }

    if (fb_idx == sizeof(SerialFeedback)) {
      uint16_t cs = (uint16_t)(
        NewFeedback.start ^
        NewFeedback.cmd1 ^ NewFeedback.cmd2 ^
        NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^
        NewFeedback.batVoltage ^ NewFeedback.boardTemp ^
        NewFeedback.cmdLed
      );

      if (NewFeedback.start == START_FRAME && cs == NewFeedback.checksum) {
        memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
        g_batVoltage = Feedback.batVoltage;
        g_boardTemp = Feedback.boardTemp;
        g_haveFeedback = true;

        // Filter battery voltage
        float vPack = (float)Feedback.batVoltage * BAT_VOLT_SCALE;
        if (g_batVoltFiltered <= 0.01f) g_batVoltFiltered = vPack;
        g_batVoltFiltered = g_batVoltFiltered + BAT_FILTER_ALPHA * (vPack - g_batVoltFiltered);
      }
      fb_idx = 0;
    }
    fb_in_prev = fb_in;
  }
}
