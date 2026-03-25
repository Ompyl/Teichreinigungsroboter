#pragma once

#include "main.h"

// Tasks
void ESCupdate(void *parameter);
void motionTask(void *parameter);

// Utility
int clampPercent(int value);
float wrap360(float angleDeg);
float angleDiffSigned(float fromDeg, float toDeg);
float turnProgressDeg(float startDeg, float currentDeg, bool turnRight);

// Motor control
void setAutoMotorValues(int rightPercent, int leftPercent);
void stopAutoDrive();

// Sensor / queue helpers
bool getCompassHeading(float &headingDeg);
void publishTurnHeading(float headingDeg);

// Command interface
bool submitMotionCommand(const MotionCommand &cmd);
void queueCancelCommand();
void startHs(int rightPercent, int leftPercent, uint32_t timeMs);
void startTurn(float angleDeg, bool turnRight, int speedPercent, int biasPercent = 0);
void startLinear(float headingDeg, int speedPercent, uint32_t timeMs, float minDistCm = -1.0f);
void startNav(float headingDeg, int speedPercent, float minDistCm, int rounds);
void startNavAvoid(float headingDeg, int speedPercent, float minDistCm, int rounds);