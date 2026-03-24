#include "motion.h"
#include "io.h"

// ======================== Module State ========================
static Servo escR;
static Servo escL;

static volatile ControlMode g_controlMode = CONTROL_MANUAL;
static volatile int g_autoRightPercent = 0;
static volatile int g_autoLeftPercent = 0;
static volatile bool g_cancelRequested = false;
static volatile uint8_t g_autoModeCode = MODE_IDLE;

// =================== Internal Helpers ===================
static int percentToEscOffset(int percent);
static float getMeanDistanceCm();
static bool getSideDistancesCm(float &dist1Cm, float &dist2Cm, bool &valid1, bool &valid2);
static void setMotorPercent(int rightPercent, int leftPercent, bool running, uint8_t modeCode);
static bool shouldCancel();
static void resetAutoState();
static bool chooseAvoidTurnDirection(bool &turnRight);

enum LinearExitReason
{
  LINEAR_EXIT_ABORTED = 0,
  LINEAR_EXIT_TIMEOUT,
  LINEAR_EXIT_OBSTACLE
};

static bool runHs(int rightPercent, int leftPercent, uint32_t timeMs);
static bool runTurn(float angleDeg, bool turnRight, int speedPercent, int biasPercent);
static LinearExitReason runLinearInternal(float headingDeg, int speedPercent, uint32_t timeMs, float minDistCm);
static bool runLinear(float headingDeg, int speedPercent, uint32_t timeMs, float minDistCm);
static bool runNav(float headingDeg, int speedPercent, float minDistCm, int rounds);
static bool runNavAvoid(float headingDeg, int speedPercent, float minDistCm, int rounds);
static bool executeCommand(const MotionCommand &cmd);

// ======================== Utility ==========================
int clampPercent(int value)
{
  if (value > 100)
    return 100;
  if (value < -100)
    return -100;
  return value;
}

static int percentToEscOffset(int percent)
{
  int p = clampPercent(percent);
  return (p * ESC_MAX_US) / 100;
}

float wrap360(float angleDeg)
{
  while (angleDeg < 0.0f)
    angleDeg += 360.0f;
  while (angleDeg >= 360.0f)
    angleDeg -= 360.0f;
  return angleDeg;
}

float angleDiffSigned(float fromDeg, float toDeg)
{
  float a = wrap360(toDeg) - wrap360(fromDeg);
  if (a > 180.0f)
    a -= 360.0f;
  if (a < -180.0f)
    a += 360.0f;
  return a;
}

float turnProgressDeg(float startDeg, float currentDeg, bool turnRight)
{
  startDeg = wrap360(startDeg);
  currentDeg = wrap360(currentDeg);

  if (turnRight)
  {
    float d = currentDeg - startDeg;
    if (d < 0.0f)
      d += 360.0f;
    return d;
  }

  float d = startDeg - currentDeg;
  if (d < 0.0f)
    d += 360.0f;
  return d;
}

void publishTurnHeading(float headingDeg)
{
  if (!turnHeadingQueue)
    return;
  xQueueOverwrite(turnHeadingQueue, &headingDeg);
}

bool getCompassHeading(float &headingDeg)
{
  CompassData cd{};
  if (compassQueue && xQueuePeek(compassQueue, &cd, 0) == pdPASS && cd.valid)
  {
    headingDeg = wrap360(cd.angle_deg);
    return true;
  }

  headingDeg = 0.0f;
  return false;
}

static float getMeanDistanceCm()
{
  float d1 = -1.0f;
  float d2 = -1.0f;
  bool ok1 = distQueue1 && xQueuePeek(distQueue1, &d1, 0) == pdPASS;
  bool ok2 = distQueue2 && xQueuePeek(distQueue2, &d2, 0) == pdPASS;

  bool v1 = ok1 && d1 >= 0.0f;
  bool v2 = ok2 && d2 >= 0.0f;

  if (v1 && v2)
    return (d1 + d2) * 0.5f;
  if (v1)
    return d1;
  if (v2)
    return d2;
  return -1.0f;
}

static bool getSideDistancesCm(float &dist1Cm, float &dist2Cm, bool &valid1, bool &valid2)
{
  dist1Cm = -1.0f;
  dist2Cm = -1.0f;

  valid1 = distQueue1 && xQueuePeek(distQueue1, &dist1Cm, 0) == pdPASS && dist1Cm >= 0.0f;
  valid2 = distQueue2 && xQueuePeek(distQueue2, &dist2Cm, 0) == pdPASS && dist2Cm >= 0.0f;

  return valid1 || valid2;
}

static bool chooseAvoidTurnDirection(bool &turnRight)
{
  float d1 = -1.0f;
  float d2 = -1.0f;
  bool v1 = false;
  bool v2 = false;

  getSideDistancesCm(d1, d2, v1, v2);

  if (v1 && v2)
  {
    if (fabsf(d1 - d2) < 1.0f)
    {
      turnRight = NAV_AVOID_DEFAULT_TURN_RIGHT;
    }
    else
    {
#if NAV_AVOID_DIST1_IS_RIGHT
      turnRight = d1 > d2;
#else
      turnRight = d2 > d1;
#endif
    }
    return true;
  }

  turnRight = NAV_AVOID_DEFAULT_TURN_RIGHT;
  return false;
}

void setAutoMotorValues(int rightPercent, int leftPercent)
{
  g_autoRightPercent = clampPercent(rightPercent);
  g_autoLeftPercent = clampPercent(leftPercent);
}

void stopAutoDrive()
{
  setAutoMotorValues(0, 0);
}

static void setMotorPercent(int rightPercent, int leftPercent, bool running, uint8_t modeCode)
{
  int r = clampPercent(rightPercent);
  int l = clampPercent(leftPercent);

  escR.writeMicroseconds(ESC_BASE_US + percentToEscOffset(r));
  escL.writeMicroseconds(ESC_BASE_US + percentToEscOffset(l));

  if (!throttleQueue)
    return;

  ThrottleData td{};
  td.rightPercent = r;
  td.leftPercent = l;
  td.running = running;
  td.mode = modeCode;
  xQueueOverwrite(throttleQueue, &td);
}

static bool shouldCancel()
{
  return g_cancelRequested;
}

static void resetAutoState()
{
  g_cancelRequested = false;
  g_autoModeCode = MODE_IDLE;
  stopAutoDrive();
  publishTurnHeading(-1.0f);
}

// ======================== Command Queue =====================
bool submitMotionCommand(const MotionCommand &cmd)
{
  if (!motionCmdQueue)
    return false;
  xQueueOverwrite(motionCmdQueue, &cmd);
  return true;
}

void queueCancelCommand()
{
  g_cancelRequested = true;

  MotionCommand cmd{};
  cmd.type = CMD_CANCEL;
  submitMotionCommand(cmd);
}

void startHs(int rightPercent, int leftPercent, uint32_t timeMs)
{
  MotionCommand cmd{};
  cmd.type = CMD_HS;
  cmd.rightPercent = clampPercent(rightPercent);
  cmd.leftPercent = clampPercent(leftPercent);
  cmd.timeMs = timeMs;
  submitMotionCommand(cmd);
}

void startTurn(float angleDeg, bool turnRight, int speedPercent, int biasPercent)
{
  MotionCommand cmd{};
  cmd.type = CMD_TURN;
  cmd.angleDeg = fabsf(angleDeg);
  cmd.turnRight = turnRight;
  cmd.speedPercent = clampPercent(abs(speedPercent));
  cmd.biasPercent = clampPercent(biasPercent);
  submitMotionCommand(cmd);
}

void startLinear(float headingDeg, int speedPercent, uint32_t timeMs, float minDistCm)
{
  MotionCommand cmd{};
  cmd.type = CMD_LINEAR;
  cmd.headingDeg = wrap360(headingDeg);
  cmd.speedPercent = clampPercent(speedPercent);
  cmd.timeMs = timeMs;
  cmd.minDistCm = minDistCm;
  submitMotionCommand(cmd);
}

void startNav(float headingDeg, int speedPercent, float minDistCm, int rounds)
{
  MotionCommand cmd{};
  cmd.type = CMD_NAV;
  cmd.headingDeg = wrap360(headingDeg);
  cmd.speedPercent = clampPercent(speedPercent);
  cmd.minDistCm = minDistCm;
  cmd.rounds = rounds;
  submitMotionCommand(cmd);
}

void startNavAvoid(float headingDeg, int speedPercent, float minDistCm, int rounds)
{
  MotionCommand cmd{};
  cmd.type = CMD_NAV_AVOID;
  cmd.headingDeg = wrap360(headingDeg);
  cmd.speedPercent = clampPercent(speedPercent);
  cmd.minDistCm = minDistCm;
  cmd.rounds = rounds;
  submitMotionCommand(cmd);
}

// ======================== Motion Routines ====================
static bool runHs(int rightPercent, int leftPercent, uint32_t timeMs)
{
  uint32_t startMs = millis();
  publishTurnHeading(-1.0f);

  while ((uint32_t)(millis() - startMs) < timeMs)
  {
    if (shouldCancel())
      return false;

    setAutoMotorValues(rightPercent, leftPercent);
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  return true;
}

static bool runTurn(float angleDeg, bool turnRight, int speedPercent, int biasPercent)
{
  if (angleDeg < 0.5f)
    angleDeg = 0.5f;

  bool rightEff = turnRight ^ TURN_DIR_INVERT;
  float startHeading = 0.0f;
  uint32_t waitStart = millis();

  while (!getCompassHeading(startHeading))
  {
    if (shouldCancel())
      return false;
    if ((uint32_t)(millis() - waitStart) >= TURN_TIMEOUT_MS)
      return false;
    vTaskDelay(pdMS_TO_TICKS(20));
  }

  float targetHeading = rightEff ? wrap360(startHeading + angleDeg)
                                 : wrap360(startHeading - angleDeg);
  publishTurnHeading(targetHeading);

  uint32_t startMs = millis();
  while ((uint32_t)(millis() - startMs) < TURN_TIMEOUT_MS)
  {
    if (shouldCancel())
      return false;

    float currentHeading = 0.0f;
    if (getCompassHeading(currentHeading))
    {
      float progress = turnProgressDeg(startHeading, currentHeading, rightEff);

      if ((progress + TURN_TOL_DEG) >= angleDeg)
        return true;
    }

    int right = biasPercent + (rightEff ? speedPercent : -speedPercent);
    int left = biasPercent + (rightEff ? -speedPercent : speedPercent);
    setAutoMotorValues(right, left);
    vTaskDelay(pdMS_TO_TICKS(20));
  }

  logLine("TURN timeout");
  return false;
}

static LinearExitReason runLinearInternal(float headingDeg, int speedPercent, uint32_t timeMs, float minDistCm)
{
  headingDeg = wrap360(headingDeg);
  publishTurnHeading(headingDeg);

  uint32_t startMs = millis();
  uint8_t obstacleCount = 0;

  while ((uint32_t)(millis() - startMs) < timeMs)
  {
    if (shouldCancel())
      return LINEAR_EXIT_ABORTED;

    if (minDistCm > 0.0f)
    {
      float dist = getMeanDistanceCm();
      if (dist >= 0.0f && dist <= minDistCm)
      {
        obstacleCount++;
        if (obstacleCount >= 3)
        {
          logLineStr("LINEAR obstacle confirmed dist=" + String(dist, 1));
          return LINEAR_EXIT_OBSTACLE;
        }
      }
      else
      {
        obstacleCount = 0;
      }
    }

    int right = speedPercent;
    int left = speedPercent;

    float currentHeading = 0.0f;
    if (getCompassHeading(currentHeading))
    {
      float error = angleDiffSigned(currentHeading, headingDeg);
      if (fabsf(error) < LINEAR_TOL_DEG)
        error = 0.0f;

      int corr = (int)(LINEAR_KP * error);
      corr = constrain(corr, -LINEAR_MAX_CORR_PERCENT, LINEAR_MAX_CORR_PERCENT);

      right = speedPercent + corr;
      left = speedPercent - corr;
    }

    setAutoMotorValues(right, left);
    vTaskDelay(pdMS_TO_TICKS(LINEAR_LOOP_DELAY_MS));
  }

  return LINEAR_EXIT_TIMEOUT;
}

static bool runLinear(float headingDeg, int speedPercent, uint32_t timeMs, float minDistCm)
{
  return runLinearInternal(headingDeg, speedPercent, timeMs, minDistCm) != LINEAR_EXIT_ABORTED;
}

static bool runNav(float headingDeg, int speedPercent, float minDistCm, int rounds)
{
  if (rounds <= 0)
    return true;

  float currentHeading = wrap360(headingDeg);
  bool nextTurnRight = NAV_FIRST_TURN_RIGHT;

  logLineStr("NAV begin heading=" + String(currentHeading, 1) + " speed=" + String(speedPercent) + " minDist=" + String(minDistCm, 1) + " rounds=" + String(rounds));

  for (int round = 0; round < rounds; ++round)
  {
    logLineStr("NAV round " + String(round + 1) + "/" + String(rounds) + " heading=" + String(currentHeading, 1));

    LinearExitReason linearResult = runLinearInternal(currentHeading, speedPercent, NAV_LINEAR_TIMEOUT_MS, minDistCm);
    if (linearResult == LINEAR_EXIT_ABORTED)
    {
      logLine("NAV aborted in linear");
      return false;
    }

    logLineStr("NAV linear done, turning " + String(nextTurnRight ? "R" : "L"));

    if (!runTurn(NAV_TURN_ANGLE_DEG, nextTurnRight, NAV_TURN_SPEED_PERCENT, NAV_TURN_BIAS_PERCENT))
    {
      logLine("NAV aborted in turn");
      return false;
    }

    currentHeading = wrap360(currentHeading + 180.0f);
    nextTurnRight = !nextTurnRight;

    logLineStr("NAV turn done, next heading=" + String(currentHeading, 1));
  }

  logLine("NAV complete");
  return true;
}

static bool runNavAvoid(float headingDeg, int speedPercent, float minDistCm, int rounds)
{
  if (rounds <= 0)
    return true;

  float currentHeading = wrap360(headingDeg);

  logLineStr("NAV_AVOID begin heading=" + String(currentHeading, 1) +
             " speed=" + String(speedPercent) +
             " minDist=" + String(minDistCm, 1) +
             " rounds=" + String(rounds));

  for (int round = 0; round < rounds; ++round)
  {
    logLineStr("NAV_AVOID round " + String(round + 1) + "/" + String(rounds) +
               " heading=" + String(currentHeading, 1));

    LinearExitReason linearResult = runLinearInternal(currentHeading, speedPercent, NAV_LINEAR_TIMEOUT_MS, minDistCm);
    if (linearResult == LINEAR_EXIT_ABORTED)
    {
      logLine("NAV_AVOID aborted in linear");
      return false;
    }

    if (linearResult == LINEAR_EXIT_TIMEOUT)
    {
      logLine("NAV_AVOID no obstacle, keep heading");
      continue;
    }

    bool turnRight = NAV_AVOID_DEFAULT_TURN_RIGHT;
    bool haveSideInfo = chooseAvoidTurnDirection(turnRight);

    logLineStr("NAV_AVOID obstacle -> turning " +
               String(turnRight ? "R" : "L") +
               String(haveSideInfo ? "" : " (fallback)"));

    if (!runTurn(NAV_AVOID_TURN_ANGLE_DEG, turnRight, NAV_TURN_SPEED_PERCENT, NAV_TURN_BIAS_PERCENT))
    {
      logLine("NAV_AVOID aborted in turn");
      return false;
    }

    currentHeading = turnRight ? wrap360(currentHeading + NAV_AVOID_TURN_ANGLE_DEG)
                               : wrap360(currentHeading - NAV_AVOID_TURN_ANGLE_DEG);

    logLineStr("NAV_AVOID turn done, next heading=" + String(currentHeading, 1));
  }

  logLine("NAV_AVOID complete");
  return true;
}

static bool executeCommand(const MotionCommand &cmd)
{
  switch (cmd.type)
  {
  case CMD_HS:
    logLineStr("HS start R=" + String(cmd.rightPercent) +
               " L=" + String(cmd.leftPercent) +
               " t=" + String(cmd.timeMs));
    g_autoModeCode = MODE_HS;
    return runHs(cmd.rightPercent, cmd.leftPercent, cmd.timeMs);

  case CMD_TURN:
    logLineStr("TURN start angle=" + String(cmd.angleDeg, 2) +
               " dir=" + String(cmd.turnRight ? "R" : "L") +
               " speed=" + String(cmd.speedPercent) +
               " bias=" + String(cmd.biasPercent));
    g_autoModeCode = MODE_TURN;
    return runTurn(cmd.angleDeg, cmd.turnRight, cmd.speedPercent, cmd.biasPercent);

  case CMD_LINEAR:
    logLineStr("LINEAR start heading=" + String(cmd.headingDeg, 2) +
               " speed=" + String(cmd.speedPercent) +
               " time=" + String(cmd.timeMs) +
               " minDist=" + String(cmd.minDistCm, 1));
    g_autoModeCode = MODE_LINEAR;
    return runLinear(cmd.headingDeg, cmd.speedPercent, cmd.timeMs, cmd.minDistCm);

  case CMD_NAV:
    logLineStr("NAV start heading=" + String(cmd.headingDeg, 2) +
               " speed=" + String(cmd.speedPercent) +
               " minDist=" + String(cmd.minDistCm, 1) +
               " rounds=" + String(cmd.rounds));
    g_autoModeCode = MODE_NAV;
    return runNav(cmd.headingDeg, cmd.speedPercent, cmd.minDistCm, cmd.rounds);

  case CMD_NAV_AVOID:
    logLineStr("NAV_AVOID start heading=" + String(cmd.headingDeg, 2) +
               " speed=" + String(cmd.speedPercent) +
               " minDist=" + String(cmd.minDistCm, 1) +
               " rounds=" + String(cmd.rounds));
    g_autoModeCode = MODE_NAV_AVOID;
    return runNavAvoid(cmd.headingDeg, cmd.speedPercent, cmd.minDistCm, cmd.rounds);

  default:
    return true;
  }
}

// ======================== Tasks ==============================
void motionTask(void *parameter)
{
  resetAutoState();

  for (;;)
  {
    MotionCommand cmd{};
    if (!motionCmdQueue || xQueueReceive(motionCmdQueue, &cmd, portMAX_DELAY) != pdPASS)
      continue;

    if (cmd.type == CMD_CANCEL)
    {
      g_cancelRequested = true;
      g_controlMode = CONTROL_MANUAL;
      stopAutoDrive();
      publishTurnHeading(-1.0f);
      logLine("CMD cancel -> manual");
      continue;
    }

    g_cancelRequested = false;
    g_controlMode = (cmd.type == CMD_NAV || cmd.type == CMD_NAV_AVOID) ? CONTROL_AUTO_NAV : CONTROL_AUTO_CMD;

    bool ok = executeCommand(cmd);
    stopAutoDrive();
    publishTurnHeading(-1.0f);
    g_controlMode = CONTROL_MANUAL;
    g_cancelRequested = false;

    if (ok)
      logLine("Motion done -> manual");
    else
      logLine("Motion aborted -> manual");
  }
}

void ESCupdate(void *parameter)
{
  vTaskDelay(pdMS_TO_TICKS(200));

  escR.attach(ESC_PIN_R);
  escL.attach(ESC_PIN_L);
  escR.writeMicroseconds(ESC_BASE_US);
  escL.writeMicroseconds(ESC_BASE_US);
  setMotorPercent(0, 0, false, MODE_IDLE);

  for (;;)
  {
    if (g_controlMode == CONTROL_MANUAL)
    {
      WiiData wii{};
      if (wiiQueue && xQueuePeek(wiiQueue, &wii, 0) == pdPASS && wii.active)
      {
        int right = (int)((wii.nkcy + (wii.nkcx * 0.5f)) * 100.0f);
        int left = (int)((wii.nkcy - (wii.nkcx * 0.5f)) * 100.0f);
        bool driving = (fabsf(wii.nkcx) > 0.01f) || (fabsf(wii.nkcy) > 0.01f);
        setMotorPercent(right, left, false, driving ? MODE_MANUAL : MODE_IDLE);
      }
      else
      {
        setMotorPercent(0, 0, false, MODE_IDLE);
      }

      vTaskDelay(pdMS_TO_TICKS(19));
      continue;
    }

    setMotorPercent(g_autoRightPercent, g_autoLeftPercent, true, g_autoModeCode);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}