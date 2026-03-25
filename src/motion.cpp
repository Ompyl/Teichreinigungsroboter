#include "motion.h"
#include "io.h"

static Servo escR;
static Servo escL;

static volatile ControlMode controlMode = CONTROL_MANUAL;
static volatile int autoRightPercent = 0;
static volatile int autoLeftPercent = 0;
static volatile bool cancelRequested = false;
static volatile uint8_t autoModeCode = MODE_IDLE;

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

int clampPercent(int value)
{
  if (value > 100) {
    return 100;
  }

  if (value < -100) {
    return -100;
  }

  return value;
}

static int percentToEscOffset(int percent)
{
  const int clamped = clampPercent(percent);
  return (clamped * ESC_MAX_US) / 100;
}

float wrap360(float angleDeg)
{
  while (angleDeg < 0.0f) {
    angleDeg += 360.0f;
  }

  while (angleDeg >= 360.0f) {
    angleDeg -= 360.0f;
  }

  return angleDeg;
}

float angleDiffSigned(float fromDeg, float toDeg)
{
  float diff = wrap360(toDeg) - wrap360(fromDeg);

  if (diff > 180.0f) {
    diff -= 360.0f;
  }

  if (diff < -180.0f) {
    diff += 360.0f;
  }

  return diff;
}

float turnProgressDeg(float startDeg, float currentDeg, bool turnRight)
{
  startDeg = wrap360(startDeg);
  currentDeg = wrap360(currentDeg);

  if (turnRight) {
    float diff = currentDeg - startDeg;

    if (diff < 0.0f) {
      diff += 360.0f;
    }

    return diff;
  }

  float diff = startDeg - currentDeg;

  if (diff < 0.0f) {
    diff += 360.0f;
  }

  return diff;
}

void publishTurnHeading(float headingDeg)
{
  if (!turnHeadingQueue) {
    return;
  }

  xQueueOverwrite(turnHeadingQueue, &headingDeg);
}

bool getCompassHeading(float &headingDeg)
{
  CompassData compassData{};

  if (compassQueue &&
      xQueuePeek(compassQueue, &compassData, 0) == pdPASS &&
      compassData.valid) {
    headingDeg = wrap360(compassData.angle_deg);
    return true;
  }

  headingDeg = 0.0f;
  return false;
}

static float getMeanDistanceCm()
{
  float dist1 = -1.0f;
  float dist2 = -1.0f;

  const bool ok1 = distQueue1 && xQueuePeek(distQueue1, &dist1, 0) == pdPASS;
  const bool ok2 = distQueue2 && xQueuePeek(distQueue2, &dist2, 0) == pdPASS;

  const bool valid1 = ok1 && dist1 >= 0.0f;
  const bool valid2 = ok2 && dist2 >= 0.0f;

  if (valid1 && valid2) {
    return (dist1 + dist2) * 0.5f;
  }

  if (valid1) {
    return dist1;
  }

  if (valid2) {
    return dist2;
  }

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
  float dist1 = -1.0f;
  float dist2 = -1.0f;
  bool valid1 = false;
  bool valid2 = false;

  getSideDistancesCm(dist1, dist2, valid1, valid2);

  if (valid1 && valid2) {
    if (fabsf(dist1 - dist2) < 1.0f) {
      turnRight = NAV_AVOID_DEFAULT_TURN_RIGHT;
    } else {
#if NAV_AVOID_DIST1_IS_RIGHT
      turnRight = dist1 > dist2;
#else
      turnRight = dist2 > dist1;
#endif
    }

    return true;
  }

  turnRight = NAV_AVOID_DEFAULT_TURN_RIGHT;
  return false;
}

void setAutoMotorValues(int rightPercent, int leftPercent)
{
  autoRightPercent = clampPercent(rightPercent);
  autoLeftPercent = clampPercent(leftPercent);
}

void stopAutoDrive()
{
  setAutoMotorValues(0, 0);
}

static void setMotorPercent(int rightPercent, int leftPercent, bool running, uint8_t modeCode)
{
  const int right = clampPercent(rightPercent);
  const int left = clampPercent(leftPercent);

  escR.writeMicroseconds(ESC_BASE_US + percentToEscOffset(right));
  escL.writeMicroseconds(ESC_BASE_US + percentToEscOffset(left));

  if (!throttleQueue) {
    return;
  }

  ThrottleData throttleData{};
  throttleData.rightPercent = right;
  throttleData.leftPercent = left;
  throttleData.running = running;
  throttleData.mode = modeCode;
  xQueueOverwrite(throttleQueue, &throttleData);
}

static bool shouldCancel()
{
  return cancelRequested;
}

static void resetAutoState()
{
  cancelRequested = false;
  autoModeCode = MODE_IDLE;
  stopAutoDrive();
  publishTurnHeading(-1.0f);
}

bool submitMotionCommand(const MotionCommand &cmd)
{
  if (!motionCmdQueue) {
    return false;
  }

  xQueueOverwrite(motionCmdQueue, &cmd);
  return true;
}

void queueCancelCommand()
{
  cancelRequested = true;

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

static bool runHs(int rightPercent, int leftPercent, uint32_t timeMs)
{
  const uint32_t startMs = millis();
  publishTurnHeading(-1.0f);

  while (static_cast<uint32_t>(millis() - startMs) < timeMs) {
    if (shouldCancel()) {
      return false;
    }

    setAutoMotorValues(rightPercent, leftPercent);
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  return true;
}

static bool runTurn(float angleDeg, bool turnRight, int speedPercent, int biasPercent)
{
  if (angleDeg < 0.5f) {
    angleDeg = 0.5f;
  }

  const bool rightEffective = turnRight ^ TURN_DIR_INVERT;
  float startHeading = 0.0f;
  const uint32_t waitStartMs = millis();

  while (!getCompassHeading(startHeading)) {
    if (shouldCancel()) {
      return false;
    }

    if (static_cast<uint32_t>(millis() - waitStartMs) >= TURN_TIMEOUT_MS) {
      return false;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }

  const float targetHeading = rightEffective
                                  ? wrap360(startHeading + angleDeg)
                                  : wrap360(startHeading - angleDeg);

  publishTurnHeading(targetHeading);

  const uint32_t startMs = millis();

  while (static_cast<uint32_t>(millis() - startMs) < TURN_TIMEOUT_MS) {
    if (shouldCancel()) {
      return false;
    }

    float currentHeading = 0.0f;

    if (getCompassHeading(currentHeading)) {
      const float progress = turnProgressDeg(startHeading, currentHeading, rightEffective);

      if ((progress + TURN_TOL_DEG) >= angleDeg) {
        return true;
      }
    }

    const int right = biasPercent + (rightEffective ? speedPercent : -speedPercent);
    const int left = biasPercent + (rightEffective ? -speedPercent : speedPercent);

    setAutoMotorValues(right, left);
    vTaskDelay(pdMS_TO_TICKS(20));
  }

  logLine("[turn] timeout");
  return false;
}

static LinearExitReason runLinearInternal(float headingDeg, int speedPercent, uint32_t timeMs, float minDistCm)
{
  headingDeg = wrap360(headingDeg);
  publishTurnHeading(headingDeg);

  const uint32_t startMs = millis();
  uint8_t obstacleCount = 0;

  while (static_cast<uint32_t>(millis() - startMs) < timeMs) {
    if (shouldCancel()) {
      return LINEAR_EXIT_ABORTED;
    }

    if (minDistCm > 0.0f) {
      const float distanceCm = getMeanDistanceCm();

      if (distanceCm >= 0.0f && distanceCm <= minDistCm) {
        obstacleCount++;

        if (obstacleCount >= 3) {
          logLineStr("[linear] obstacle confirmed dist=" + String(distanceCm, 1));
          return LINEAR_EXIT_OBSTACLE;
        }
      } else {
        obstacleCount = 0;
      }
    }

    int right = speedPercent;
    int left = speedPercent;

    float currentHeading = 0.0f;

    if (getCompassHeading(currentHeading)) {
      float error = angleDiffSigned(currentHeading, headingDeg);

      if (fabsf(error) < LINEAR_TOL_DEG) {
        error = 0.0f;
      }

      int correction = static_cast<int>(LINEAR_KP * error);
      correction = constrain(correction, -LINEAR_MAX_CORR_PERCENT, LINEAR_MAX_CORR_PERCENT);

      right = speedPercent + correction;
      left = speedPercent - correction;
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
  if (rounds <= 0) {
    return true;
  }

  float currentHeading = wrap360(headingDeg);
  bool nextTurnRight = NAV_FIRST_TURN_RIGHT;

  logLineStr(
      "[nav] begin heading=" + String(currentHeading, 1) +
      " speed=" + String(speedPercent) +
      " minDist=" + String(minDistCm, 1) +
      " rounds=" + String(rounds));

  for (int round = 0; round < rounds; ++round) {
    logLineStr(
        "[nav] round " + String(round + 1) + "/" + String(rounds) +
        " heading=" + String(currentHeading, 1));

    const LinearExitReason linearResult =
        runLinearInternal(currentHeading, speedPercent, NAV_LINEAR_TIMEOUT_MS, minDistCm);

    if (linearResult == LINEAR_EXIT_ABORTED) {
      logLine("[nav] aborted in linear");
      return false;
    }

    logLineStr("[nav] linear done, turning " + String(nextTurnRight ? "R" : "L"));

    if (!runTurn(NAV_TURN_ANGLE_DEG, nextTurnRight, NAV_TURN_SPEED_PERCENT, NAV_TURN_BIAS_PERCENT)) {
      logLine("[nav] aborted in turn");
      return false;
    }

    currentHeading = wrap360(currentHeading + 180.0f);
    nextTurnRight = !nextTurnRight;

    logLineStr("[nav] turn done, next heading=" + String(currentHeading, 1));
  }

  logLine("[nav] complete");
  return true;
}

static bool runNavAvoid(float headingDeg, int speedPercent, float minDistCm, int rounds)
{
  if (rounds <= 0) {
    return true;
  }

  float currentHeading = wrap360(headingDeg);

  logLineStr(
      "[nav_avoid] begin heading=" + String(currentHeading, 1) +
      " speed=" + String(speedPercent) +
      " minDist=" + String(minDistCm, 1) +
      " rounds=" + String(rounds));

  for (int round = 0; round < rounds; ++round) {
    logLineStr(
        "[nav_avoid] round " + String(round + 1) + "/" + String(rounds) +
        " heading=" + String(currentHeading, 1));

    const LinearExitReason linearResult =
        runLinearInternal(currentHeading, speedPercent, NAV_LINEAR_TIMEOUT_MS, minDistCm);

    if (linearResult == LINEAR_EXIT_ABORTED) {
      logLine("[nav_avoid] aborted in linear");
      return false;
    }

    if (linearResult == LINEAR_EXIT_TIMEOUT) {
      logLine("[nav_avoid] no obstacle, keep heading");
      continue;
    }

    bool turnRight = NAV_AVOID_DEFAULT_TURN_RIGHT;
    const bool haveSideInfo = chooseAvoidTurnDirection(turnRight);

    logLineStr(
        "[nav_avoid] obstacle -> turning " +
        String(turnRight ? "R" : "L") +
        String(haveSideInfo ? "" : " (fallback)"));

    if (!runTurn(NAV_AVOID_TURN_ANGLE_DEG, turnRight, NAV_TURN_SPEED_PERCENT, NAV_TURN_BIAS_PERCENT)) {
      logLine("[nav_avoid] aborted in turn");
      return false;
    }

    currentHeading = turnRight
                         ? wrap360(currentHeading + NAV_AVOID_TURN_ANGLE_DEG)
                         : wrap360(currentHeading - NAV_AVOID_TURN_ANGLE_DEG);

    logLineStr("[nav_avoid] turn done, next heading=" + String(currentHeading, 1));
  }

  logLine("[nav_avoid] complete");
  return true;
}

static bool executeCommand(const MotionCommand &cmd)
{
  switch (cmd.type) {
    case CMD_HS:
      logLineStr(
          "[motion] hs start R=" + String(cmd.rightPercent) +
          " L=" + String(cmd.leftPercent) +
          " t=" + String(cmd.timeMs));
      autoModeCode = MODE_HS;
      return runHs(cmd.rightPercent, cmd.leftPercent, cmd.timeMs);

    case CMD_TURN:
      logLineStr(
          "[motion] turn start angle=" + String(cmd.angleDeg, 2) +
          " dir=" + String(cmd.turnRight ? "R" : "L") +
          " speed=" + String(cmd.speedPercent) +
          " bias=" + String(cmd.biasPercent));
      autoModeCode = MODE_TURN;
      return runTurn(cmd.angleDeg, cmd.turnRight, cmd.speedPercent, cmd.biasPercent);

    case CMD_LINEAR:
      logLineStr(
          "[motion] linear start heading=" + String(cmd.headingDeg, 2) +
          " speed=" + String(cmd.speedPercent) +
          " time=" + String(cmd.timeMs) +
          " minDist=" + String(cmd.minDistCm, 1));
      autoModeCode = MODE_LINEAR;
      return runLinear(cmd.headingDeg, cmd.speedPercent, cmd.timeMs, cmd.minDistCm);

    case CMD_NAV:
      logLineStr(
          "[motion] nav start heading=" + String(cmd.headingDeg, 2) +
          " speed=" + String(cmd.speedPercent) +
          " minDist=" + String(cmd.minDistCm, 1) +
          " rounds=" + String(cmd.rounds));
      autoModeCode = MODE_NAV;
      return runNav(cmd.headingDeg, cmd.speedPercent, cmd.minDistCm, cmd.rounds);

    case CMD_NAV_AVOID:
      logLineStr(
          "[motion] nav_avoid start heading=" + String(cmd.headingDeg, 2) +
          " speed=" + String(cmd.speedPercent) +
          " minDist=" + String(cmd.minDistCm, 1) +
          " rounds=" + String(cmd.rounds));
      autoModeCode = MODE_NAV_AVOID;
      return runNavAvoid(cmd.headingDeg, cmd.speedPercent, cmd.minDistCm, cmd.rounds);

    default:
      return true;
  }
}

void motionTask(void *parameter)
{
  resetAutoState();

  for (;;) {
    MotionCommand cmd{};

    if (!motionCmdQueue || xQueueReceive(motionCmdQueue, &cmd, portMAX_DELAY) != pdPASS) {
      continue;
    }

    if (cmd.type == CMD_CANCEL) {
      cancelRequested = true;
      controlMode = CONTROL_MANUAL;
      stopAutoDrive();
      publishTurnHeading(-1.0f);
      logLine("[motion] cancel -> manual");
      continue;
    }

    cancelRequested = false;
    controlMode =
        (cmd.type == CMD_NAV || cmd.type == CMD_NAV_AVOID)
            ? CONTROL_AUTO_NAV
            : CONTROL_AUTO_CMD;

    const bool ok = executeCommand(cmd);

    stopAutoDrive();
    publishTurnHeading(-1.0f);
    controlMode = CONTROL_MANUAL;
    cancelRequested = false;

    if (ok) {
      logLine("[motion] done -> manual");
    } else {
      logLine("[motion] aborted -> manual");
    }
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

  for (;;) {
    if (controlMode == CONTROL_MANUAL) {
      WiiData wiiData{};

      if (wiiQueue && xQueuePeek(wiiQueue, &wiiData, 0) == pdPASS && wiiData.active) {
        const int right =
            static_cast<int>((wiiData.nkcy + (wiiData.nkcx * 0.5f)) * 100.0f);
        const int left =
            static_cast<int>((wiiData.nkcy - (wiiData.nkcx * 0.5f)) * 100.0f);

        const bool driving =
            (fabsf(wiiData.nkcx) > 0.01f) || (fabsf(wiiData.nkcy) > 0.01f);

        setMotorPercent(right, left, false, driving ? MODE_MANUAL : MODE_IDLE);
      } else {
        setMotorPercent(0, 0, false, MODE_IDLE);
      }

      vTaskDelay(pdMS_TO_TICKS(19));
      continue;
    }

    setMotorPercent(autoRightPercent, autoLeftPercent, true, autoModeCode);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}