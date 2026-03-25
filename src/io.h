#pragma once

#include "main.h"

// Temperature measurement
float readTemp(int pin);

// System / logging
void ESPreboot();
void logLine(const char *msg);
void logLineStr(const String &s);
extern "C" void logPrintf(const char *format, ...);

// Tasks
void taskLogger(void *pv);
void taskWeb(void *pv);
void taskOTA(void *pv);
void taskHeartbeat(void *pv);
void DistTask(void *parameter);
void compassTask(void *parameter);
void LedBatteryLvlTask(void *pv);
void WiiCon(void *parameter);

// Event handlers
void onWiFiEvent(WiFiEvent_t event);

// External helpers
void externsetPlayerLEDs(uint8_t value);