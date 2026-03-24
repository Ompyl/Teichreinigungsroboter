#pragma once

#include "main.h"

float readTemp(int pin);
void ESPreboot();
void logLine(const char *msg);
void logLineStr(const String &s);
extern "C" void logPrintf(const char *format, ...);

void taskLogger(void *pv);
void taskWeb(void *pv);
void taskOTA(void *pv);
void taskHeartbeat(void *pv);
void DistTask(void *parameter);
void compassTask(void *parameter);
void LedBatteryLvlTask(void *pv);
void WiiCon(void *parameter);
void onWiFiEvent(WiFiEvent_t event);

void externsetPlayerLEDs(uint8_t value);
