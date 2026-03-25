#pragma once

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <AsyncTCP.h>
#include <ESP32Servo.h>
#include <ESP32Wiimote.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <math.h>
#include <stdarg.h>

// Pins
constexpr uint8_t BAT_PIN = 33;
constexpr uint8_t WATER_PIN = 32;
constexpr uint8_t R_NTC_PIN = 25;
constexpr uint8_t L_NTC_PIN = 26;
constexpr uint8_t TRIG1_PIN = 13;
constexpr uint8_t ECHO1_PIN = 18;
constexpr uint8_t TRIG2_PIN = 12;
constexpr uint8_t ECHO2_PIN = 17;
constexpr uint8_t BUZZER_PIN = 23;

// Distance measurement
constexpr uint32_t PING_GAP_MS = 70;
constexpr uint32_t ECHO_TIMEOUT_US = 30000;
constexpr float MIN_CM = 25.0f;
constexpr float MAX_CM = 200.0f;
constexpr float DIST_FILTER_ALPHA = 0.15f;
constexpr uint8_t DIST_INVALID_HOLD_COUNT = 2;

// Wii / input
constexpr uint32_t WII_TIMEOUT_MS = 300;
constexpr int NKC_THRESHOLD = 5;
constexpr float NO_NKC_V = 0.25f;

// ESC
constexpr int ESC_BASE_US = 1500;
constexpr int ESC_MAX_US = 300;
constexpr uint8_t ESC_PIN_R = 19;
constexpr uint8_t ESC_PIN_L = 16;

// NTC
constexpr float R_REF = 10000.0f;
constexpr float R_NOM = 10000.0f;
constexpr float T_NOM = 25.0f;
constexpr float B_VAL = 3950.0f;
constexpr float ADC_MAX = 4095.0f;

// Timing
constexpr uint32_t MEAS_MS = 1200;
constexpr uint32_t SEND_INTERVAL_MS = 100;
constexpr uint32_t PRINT_INTERVAL_MS = 500;

// Turn control
constexpr bool TURN_DIR_INVERT = false;
constexpr float TURN_TOL_DEG = 10.0f;
constexpr uint32_t TURN_TIMEOUT_MS = 8000;

// Linear control
constexpr float LINEAR_KP = 10.0f;
constexpr int LINEAR_MAX_CORR_PERCENT = 40;
constexpr float LINEAR_TOL_DEG = 3.0f;
constexpr uint32_t LINEAR_LOOP_DELAY_MS = 20;

// Button actions
constexpr int BTN_TURN_SPEED_PERCENT = 20;
constexpr int BTN_TURN_BIAS_PERCENT = 0;
constexpr int BTN_LINEAR_SPEED_PERCENT = 20;
constexpr uint32_t BTN_LINEAR_DURATION_MS = 10000;

// Navigation
constexpr float NAV_AVOID_TURN_ANGLE_DEG = 150.0f;
constexpr bool NAV_AVOID_DEFAULT_TURN_RIGHT = true;
constexpr bool NAV_AVOID_DIST1_IS_RIGHT = true;

constexpr int NAV_DEFAULT_SPEED_PERCENT = 15;
constexpr float NAV_DEFAULT_TRIGGER_CM = 35.0f;
constexpr int NAV_DEFAULT_ROUNDS = 4;
constexpr uint32_t NAV_LINEAR_TIMEOUT_MS = 12000;
constexpr bool NAV_FIRST_TURN_RIGHT = true;
constexpr float NAV_TURN_ANGLE_DEG = 180.0f;
constexpr int NAV_TURN_SPEED_PERCENT = 20;
constexpr int NAV_TURN_BIAS_PERCENT = 0;
constexpr uint32_t NAV_LOOP_DELAY_MS = 20;

// Compass
constexpr uint8_t HMC5883L_ADDRESS = 0x1E;
constexpr uint8_t COMP_FREQ = 20;
constexpr uint8_t COMP_SDA = 21;
constexpr uint8_t COMP_SCL = 22;

// Logging
constexpr size_t LOG_LINE_MAX = 192;

extern const uint8_t index_html_start[] asm("_binary_src_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_src_index_html_end");

struct WiiData
{
  ButtonState btn;
  float nkcx;
  float nkcy;
  bool active;
};

enum CommandType : uint8_t
{
  CMD_NONE = 0,
  CMD_HS,
  CMD_TURN,
  CMD_LINEAR,
  CMD_NAV,
  CMD_NAV_AVOID,
  CMD_CANCEL
};

enum RunModeCode : uint8_t
{
  MODE_IDLE = 0,
  MODE_MANUAL = 1,
  MODE_HS = 2,
  MODE_TURN = 3,
  MODE_LINEAR = 4,
  MODE_NAV = 5,
  MODE_NAV_AVOID = 6
};

enum ControlMode : uint8_t
{
  CONTROL_MANUAL = 0,
  CONTROL_AUTO_CMD = 1,
  CONTROL_AUTO_NAV = 2
};

struct MotionCommand
{
  CommandType type = CMD_NONE;

  int rightPercent = 0;
  int leftPercent = 0;

  float angleDeg = 0.0f;
  bool turnRight = true;
  int speedPercent = 0;
  int biasPercent = 0;

  float headingDeg = 0.0f;
  uint32_t timeMs = 0;
  float minDistCm = -1.0f;

  int rounds = 0;
};

struct ThrottleData
{
  int rightPercent;
  int leftPercent;
  bool running;
  uint8_t mode;
};

struct CompassData
{
  float angle_deg;
  int16_t x;
  int16_t y;
  int16_t z;
  bool valid;
};

extern QueueHandle_t logQueue;
extern QueueHandle_t wiiQueue;
extern QueueHandle_t throttleQueue;
extern QueueHandle_t turnHeadingQueue;
extern QueueHandle_t distQueue1;
extern QueueHandle_t distQueue2;
extern QueueHandle_t motionCmdQueue;
extern QueueHandle_t compassQueue;