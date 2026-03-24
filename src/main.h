#pragma once

#include <Arduino.h>
#include <stdarg.h>
#include <Wire.h>
#include "ESP32Wiimote.h"
#include <math.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// ======================== Defines ========================
#define BAT_PIN 33
#define WATER_PIN 32
#define R_NTC_PIN 25
#define L_NTC_PIN 26
#define TRIG1_PIN 13
#define ECHO1_PIN 18
#define TRIG2_PIN 12
#define ECHO2_PIN 17
#define BUZZER_PIN 23

#define PING_GAP_MS 70
#define ECHO_TIMEOUT_US 30000
#define MIN_CM 25.0f
#define MAX_CM 200.0f
#define DIST_FILTER_ALPHA 0.15f
#define DIST_INVALID_HOLD_COUNT 2

#define WII_TIMEOUT_MS 300
#define NKC_THRESHOLD 5
#define NO_NKC_V 0.25f

#define ESC_BASE_US 1500
#define ESC_MAX_US 300
#define ESC_PIN_R 19
#define ESC_PIN_L 16

#define R_REF 10000.0f    // reference resistor (10k)
#define R_NOM 10000.0f    // NTC nominal resistance at 25°C
#define T_NOM 25.0f       // nominal temperature (°C)
#define B_VAL 3950.0f     // B value of NTC
#define ADC_MAX 4095.0f   // 12-bit ADC

#define MEAS_MS 1200
#define SEND_INTERVAL_MS 100
#define PRINT_INTERVAL_MS 500

#define TURN_DIR_INVERT false
#define TURN_TOL_DEG 10.0f
#define TURN_TIMEOUT_MS 8000

#define LINEAR_KP 10.0f
#define LINEAR_MAX_CORR_PERCENT 40
#define LINEAR_TOL_DEG 3.0f
#define LINEAR_LOOP_DELAY_MS 20

#define BTN_TURN_SPEED_PERCENT 20
#define BTN_TURN_BIAS_PERCENT 0
#define BTN_LINEAR_SPEED_PERCENT 20
#define BTN_LINEAR_DURATION_MS 10000


#define NAV_AVOID_TURN_ANGLE_DEG      150.0f
#define NAV_AVOID_DEFAULT_TURN_RIGHT  true
#define NAV_AVOID_DIST1_IS_RIGHT      true

#define NAV_DEFAULT_SPEED_PERCENT 15
#define NAV_DEFAULT_TRIGGER_CM 35.0f
#define NAV_DEFAULT_ROUNDS 4
#define NAV_LINEAR_TIMEOUT_MS 12000
#define NAV_FIRST_TURN_RIGHT true
#define NAV_TURN_ANGLE_DEG 180.0f
#define NAV_TURN_SPEED_PERCENT 20
#define NAV_TURN_BIAS_PERCENT 0
#define NAV_LOOP_DELAY_MS 20

#define HMC5883L_ADDRESS 0x1E
#define COMP_FREQ 20
#define COMP_SDA 21
#define COMP_SCL 22

constexpr size_t LOG_LINE_MAX = 192;

extern const uint8_t index_html_start[] asm("_binary_src_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_src_index_html_end");

// ======================== Types ==========================
typedef struct
{
  ButtonState btn;
  float nkcx;
  float nkcy;
  bool active;
} WiiData;

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

typedef struct
{
  int rightPercent;
  int leftPercent;
  bool running;
  uint8_t mode;
} ThrottleData;

typedef struct
{
  float angle_deg;
  int16_t x, y, z;
  bool valid;
} CompassData;

// ======================== Shared Queues ========================
extern QueueHandle_t logQueue;
extern QueueHandle_t wiiQueue;
extern QueueHandle_t throttleQueue;
extern QueueHandle_t turnHeadingQueue;
extern QueueHandle_t distQueue1;
extern QueueHandle_t distQueue2;
extern QueueHandle_t motionCmdQueue;
extern QueueHandle_t compassQueue;
