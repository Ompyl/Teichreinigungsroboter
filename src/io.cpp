#include "io.h"
#include "motion.h"

static int batteryLevels[4] = {3000, 3600, 3900, 4050};
static int measVoltage = 4000;

static volatile uint32_t echoStartUs[2] = {0, 0};
static volatile uint32_t echoEndUs[2] = {0, 0};
static volatile bool echoDone[2] = {false, false};
static volatile bool waitingRise[2] = {true, true};

static ESP32Wiimote wiimote;
static long lastMs = 0;

static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");

static volatile bool wifiReady = false;
static volatile bool otaReady = false;
static volatile bool webReady = false;

static void IRAM_ATTR handleEchoISR(uint8_t idx, uint8_t echoPin);
static void IRAM_ATTR echoISR1();
static void IRAM_ATTR echoISR2();
static void trigPulse(uint8_t trigPin);
static void publishDistance(uint8_t idx, float distanceCm);

static void initCompass();
static bool readCompass(int16_t &x, int16_t &y, int16_t &z);
static bool compassBusRecover();
static void recoverCompass();

static float sanitizeStick(int raw, int threshold);
static void handleButtons(WiiData current, WiiData previous);

static void handleConsoleLine(const char *line);
static void onWsEvent(AsyncWebSocket *serverPtr,
                      AsyncWebSocketClient *client,
                      AwsEventType type,
                      void *arg,
                      uint8_t *data,
                      size_t len);
static void startWebOnce();
static void startOTAOnce();

float readTemp(int pin)
{
  const int raw = analogRead(pin);
  const float voltage = static_cast<float>(raw) / static_cast<float>(ADC_MAX);
  const float resistance = R_REF * (1.0f / (1.0f / voltage - 1.0f));

  // Beta equation, simplified form
  const float tempK =
      1.0f / (1.0f / (T_NOM + 273.15f) + (1.0f / B_VAL) * log(resistance / R_NOM));

  return tempK - 273.15f;
}

void ESPreboot()
{
  digitalWrite(BUZZER_PIN, HIGH);
  logLine("---REBOOT---");
  externsetPlayerLEDs(0x00);
  vTaskDelay(pdMS_TO_TICKS(500));
  ESP.restart();
}

void logLine(const char *msg)
{
  if (!logQueue) {
    return;
  }

  char line[LOG_LINE_MAX];
  snprintf(line, sizeof(line), "%s", msg);
  xQueueSend(logQueue, line, 0);
}

void logLineStr(const String &s)
{
  logLine(s.c_str());
}

extern "C" void logPrintf(const char *format, ...)
{
  char finalBuffer[300];

  va_list args;
  va_start(args, format);
  vsnprintf(finalBuffer, sizeof(finalBuffer), format, args);
  va_end(args);

  Serial.print("[wii] ");
  Serial.println(finalBuffer);

  logLineStr(String("[wii] ") + finalBuffer);
}

void taskLogger(void *pv)
{
  char line[LOG_LINE_MAX];
  uint32_t lastWebSendMs = 0;

  for (;;) {
    if (xQueueReceive(logQueue, line, portMAX_DELAY) == pdTRUE) {
      Serial.println(line);

      if (webReady) {
        const uint32_t now = millis();

        if (static_cast<uint32_t>(now - lastWebSendMs) >= 35) {
          lastWebSendMs = now;
          ws.textAll(line);
        }
      }
    }
  }
}

static void handleConsoleLine(const char *line)
{
  String cmd(line);
  cmd.trim();

  if (cmd.length() == 0) {
    return;
  }

  if (cmd.equalsIgnoreCase("cancel") || cmd.equalsIgnoreCase("stop")) {
    queueCancelCommand();
    logLine("[cmd] cancel queued");
    return;
  }

  if (cmd.equalsIgnoreCase("reset")) {
    ESPreboot();
    return;
  }

  if (cmd.equalsIgnoreCase("nav")) {
    float heading = 0.0f;
    getCompassHeading(heading);

    startNav(
        heading,
        NAV_DEFAULT_SPEED_PERCENT,
        NAV_DEFAULT_TRIGGER_CM,
        NAV_DEFAULT_ROUNDS);

    logLineStr(
        "[cmd] nav default heading=" + String(heading, 2) +
        " speed=" + String(NAV_DEFAULT_SPEED_PERCENT) +
        " dist=" + String(NAV_DEFAULT_TRIGGER_CM, 1) +
        " rounds=" + String(NAV_DEFAULT_ROUNDS));
    return;
  }

  int right;
  int left;
  unsigned long durationMs;

  if (sscanf(cmd.c_str(), "hs %d %d %lu", &right, &left, &durationMs) == 3) {
    startHs(right, left, static_cast<uint32_t>(durationMs));

    logLineStr(
        "[cmd] hs R=" + String(right) +
        " L=" + String(left) +
        " t=" + String(static_cast<uint32_t>(durationMs)));
    return;
  }

  float angle;
  char dir;
  int speed;
  int bias;

  const int turnCount =
      sscanf(cmd.c_str(), "turn %f %c %d %d", &angle, &dir, &speed, &bias);

  if (turnCount == 3 || turnCount == 4) {
    dir = static_cast<char>(tolower(static_cast<unsigned char>(dir)));

    if (dir != 'r' && dir != 'l') {
      logLine("[cmd] turn dir must be r or l");
      return;
    }

    startTurn(angle, dir == 'r', speed, turnCount == 4 ? bias : 0);

    logLineStr(
        "[cmd] turn angle=" + String(angle, 2) +
        " dir=" + String(dir) +
        " speed=" + String(speed) +
        " bias=" + String(turnCount == 4 ? bias : 0));
    return;
  }

  float heading;
  float minDist;
  unsigned long timeMs;

  const int linearCount =
      sscanf(cmd.c_str(), "linear %f %d %lu %f", &heading, &speed, &timeMs, &minDist);

  if (linearCount == 3 || linearCount == 4) {
    startLinear(
        heading,
        speed,
        static_cast<uint32_t>(timeMs),
        linearCount == 4 ? minDist : -1.0f);

    logLineStr(
        "[cmd] linear heading=" + String(heading, 2) +
        " speed=" + String(speed) +
        " t=" + String(static_cast<uint32_t>(timeMs)) +
        " minDist=" + String(linearCount == 4 ? minDist : -1.0f, 1));
    return;
  }

  int rounds;

  if (sscanf(cmd.c_str(), "nav %f %d %f %d", &heading, &speed, &minDist, &rounds) == 4) {
    startNav(heading, speed, minDist, rounds);

    logLineStr(
        "[cmd] nav heading=" + String(heading, 2) +
        " speed=" + String(speed) +
        " minDist=" + String(minDist, 1) +
        " rounds=" + String(rounds));
    return;
  }

  logLine(
      "[cmd] unknown cmd (use: hs <r> <l> <ms> | turn <angle> <r|l> <speed> [bias] | "
      "linear <heading> <speed> <ms> [minDist] | nav <heading> <speed> <minDist> "
      "<rounds> | cancel | reset)");
}

static void onWsEvent(AsyncWebSocket *serverPtr,
                      AsyncWebSocketClient *client,
                      AwsEventType type,
                      void *arg,
                      uint8_t *data,
                      size_t len)
{
  (void)serverPtr;
  (void)client;

  if (type != WS_EVT_DATA) {
    return;
  }

  AwsFrameInfo *info = static_cast<AwsFrameInfo *>(arg);

  if (!info->final || info->index != 0 || info->len != len) {
    return;
  }

  if (info->opcode != WS_TEXT) {
    return;
  }

  String msg;
  msg.reserve(len);

  for (size_t i = 0; i < len; i++) {
    msg += static_cast<char>(data[i]);
  }

  msg.trim();

  if (msg.length() == 0) {
    return;
  }

  handleConsoleLine(msg.c_str());
}

static void startWebOnce()
{
  if (webReady) {
    return;
  }

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(
        200,
        "text/html; charset=utf-8",
        index_html_start,
        index_html_end - index_html_start);
  });

  server.begin();
  webReady = true;
  logLine("[web] server started on port 80");
}

void taskWeb(void *pv)
{
  for (;;) {
    if (wifiReady && webReady) {
      ws.cleanupClients();
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

static void startOTAOnce()
{
  if (otaReady) {
    return;
  }

  ArduinoOTA.setHostname("ESP32-OTA");
  ArduinoOTA.setPassword("33333333");

  ArduinoOTA.onStart([]() {
    logLine("[ota] start");
  });

  ArduinoOTA.onEnd([]() {
    logLine("[ota] end");
  });

  ArduinoOTA.onError([](ota_error_t err) {
    char buffer[LOG_LINE_MAX];
    snprintf(buffer, sizeof(buffer), "[ota] error %u", static_cast<unsigned>(err));
    logLine(buffer);
  });

  ArduinoOTA.begin();
  otaReady = true;
  logLine("[ota] ready");
}

void onWiFiEvent(WiFiEvent_t event)
{
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      wifiReady = true;
      logLineStr("[wifi] connected, IP: " + WiFi.localIP().toString());
      startOTAOnce();
      startWebOnce();
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      wifiReady = false;
      logLine("[wifi] disconnected");
      break;

    default:
      break;
  }
}

void taskOTA(void *pv)
{
  for (;;) {
    if (wifiReady && otaReady) {
      ArduinoOTA.handle();
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void IRAM_ATTR handleEchoISR(uint8_t idx, uint8_t echoPin)
{
  const bool level = digitalRead(echoPin);

  if (level) {
    if (waitingRise[idx]) {
      echoStartUs[idx] = micros();
      waitingRise[idx] = false;
    }
  } else {
    if (!waitingRise[idx]) {
      echoEndUs[idx] = micros();
      echoDone[idx] = true;
      waitingRise[idx] = true;
    }
  }
}

static void IRAM_ATTR echoISR1()
{
  handleEchoISR(0, ECHO1_PIN);
}

static void IRAM_ATTR echoISR2()
{
  handleEchoISR(1, ECHO2_PIN);
}

static void trigPulse(uint8_t trigPin)
{
  digitalWrite(trigPin, LOW);
  ets_delay_us(2);
  digitalWrite(trigPin, HIGH);
  ets_delay_us(30);
  digitalWrite(trigPin, LOW);
}

static void publishDistance(uint8_t idx, float distanceCm)
{
  if (idx == 0) {
    xQueueOverwrite(distQueue1, &distanceCm);
  } else {
    xQueueOverwrite(distQueue2, &distanceCm);
  }
}

void DistTask(void *parameter)
{
  pinMode(TRIG1_PIN, OUTPUT);
  pinMode(ECHO1_PIN, INPUT);
  pinMode(TRIG2_PIN, OUTPUT);
  pinMode(ECHO2_PIN, INPUT);

  digitalWrite(TRIG1_PIN, LOW);
  digitalWrite(TRIG2_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(ECHO1_PIN), echoISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO2_PIN), echoISR2, CHANGE);

  const uint8_t trigPins[2] = {TRIG1_PIN, TRIG2_PIN};
  uint8_t sensor = 0;
  uint32_t lastPingMs = 0;

  float filteredDist[2] = {-1.0f, -1.0f};
  uint8_t invalidHold[2] = {0, 0};

  while (true) {
    const uint32_t nowMs = millis();

    if (static_cast<uint32_t>(nowMs - lastPingMs) < PING_GAP_MS) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    noInterrupts();
    echoDone[sensor] = false;
    waitingRise[sensor] = true;
    interrupts();

    trigPulse(trigPins[sensor]);
    lastPingMs = millis();
    const uint32_t trigSentUs = micros();

    float distanceCm = -1.0f;

    while (true) {
      bool done;

      noInterrupts();
      done = echoDone[sensor];
      interrupts();

      if (done) {
        uint32_t startUs;
        uint32_t endUs;

        noInterrupts();
        startUs = echoStartUs[sensor];
        endUs = echoEndUs[sensor];
        echoDone[sensor] = false;
        waitingRise[sensor] = true;
        interrupts();

        const uint32_t durationUs = static_cast<uint32_t>(endUs - startUs);
        distanceCm = static_cast<float>(durationUs) * 0.0343f * 0.5f;

        if (distanceCm < MIN_CM || distanceCm > MAX_CM) {
          distanceCm = -1.0f;
        }

        break;
      }

      if (static_cast<uint32_t>(micros() - trigSentUs) > ECHO_TIMEOUT_US) {
        distanceCm = -1.0f;
        break;
      }

      vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (distanceCm >= 0.0f) {
      if (filteredDist[sensor] < 0.0f) {
        filteredDist[sensor] = distanceCm;
      } else {
        filteredDist[sensor] +=
            DIST_FILTER_ALPHA * (distanceCm - filteredDist[sensor]);
        publishDistance(sensor, filteredDist[sensor]);
      }

      invalidHold[sensor] = 0;
    } else {
      if (invalidHold[sensor] < DIST_INVALID_HOLD_COUNT) {
        invalidHold[sensor]++;

        if (filteredDist[sensor] >= 0.0f) {
          publishDistance(sensor, filteredDist[sensor]);
        }
      } else {
        filteredDist[sensor] = -1.0f;
        publishDistance(sensor, -1.0f);
      }
    }

    sensor ^= 1;
  }
}

static void initCompass()
{
  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x00);
  Wire.write(0b01110100);
  Wire.endTransmission();

  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x01);
  Wire.write(0b00100000);
  Wire.endTransmission();

  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x02);
  Wire.write(0x00);
  Wire.endTransmission();
}

static bool readCompass(int16_t &x, int16_t &y, int16_t &z)
{
  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x03);

  if (Wire.endTransmission() != 0) {
    return false;
  }

  if (Wire.requestFrom(static_cast<uint16_t>(HMC5883L_ADDRESS), static_cast<uint8_t>(6)) != 6) {
    return false;
  }

  if (Wire.available() != 6) {
    return false;
  }

  x = static_cast<int16_t>(Wire.read() << 8 | Wire.read());
  z = static_cast<int16_t>(Wire.read() << 8 | Wire.read());
  y = static_cast<int16_t>(Wire.read() << 8 | Wire.read());

  return true;
}

static bool compassBusRecover()
{
  pinMode(COMP_SDA, INPUT_PULLUP);
  pinMode(COMP_SCL, OUTPUT_OPEN_DRAIN);
  digitalWrite(COMP_SCL, HIGH);
  delay(2);

  for (int i = 0; i < 9; i++) {
    digitalWrite(COMP_SCL, LOW);
    delayMicroseconds(10);
    digitalWrite(COMP_SDA, HIGH);
    delayMicroseconds(10);

    if (digitalRead(COMP_SDA) == HIGH) {
      break;
    }
  }

  vTaskDelay(pdMS_TO_TICKS(1));

  pinMode(COMP_SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(COMP_SDA, LOW);
  delayMicroseconds(10);
  digitalWrite(COMP_SCL, HIGH);
  delayMicroseconds(10);
  digitalWrite(COMP_SDA, HIGH);
  delayMicroseconds(10);

  pinMode(COMP_SDA, INPUT_PULLUP);
  pinMode(COMP_SCL, INPUT_PULLUP);

  return true;
}

static void recoverCompass()
{
  logLine("[comp] recover start");

  compassBusRecover();

  Wire.begin(COMP_SDA, COMP_SCL);
  Wire.setTimeOut(3);
  initCompass();

  logLine("[comp] recover init done");
}

void compassTask(void *parameter)
{
  Wire.begin(COMP_SDA, COMP_SCL);
  Wire.setTimeOut(3);
  initCompass();

  CompassData data{};
  int16_t lastX = 0;
  int16_t lastY = 0;
  int16_t lastZ = 0;
  uint32_t staleCounter = 0;
  uint32_t failCounter = 0;

  const uint32_t staleLimit = 25;
  const uint32_t failLimit = 5;
  uint32_t lastRecoverMs = 0;

  while (true) {
    int16_t x;
    int16_t y;
    int16_t z;

    const bool ok = readCompass(x, y, z);

    if (ok) {
      if (x == lastX && y == lastY && z == lastZ) {
        staleCounter++;
      } else {
        staleCounter = 0;
      }

      lastX = x;
      lastY = y;
      lastZ = z;
      failCounter = 0;

      float angleRad = atan2f(static_cast<float>(y), static_cast<float>(x));

      if (angleRad < 0.0f) {
        angleRad += 2.0f * PI;
      }

      data.x = x;
      data.y = y;
      data.z = z;
      data.angle_deg = angleRad * 180.0f / PI;
      data.valid = true;
    } else {
      data.valid = false;
      failCounter++;
      staleCounter = 0;
      vTaskDelay(pdMS_TO_TICKS(30));
    }

    xQueueOverwrite(compassQueue, &data);

    const uint32_t now = millis();
    const bool needRecover =
        (failCounter >= failLimit) || (staleCounter >= staleLimit);

    if (needRecover && (now - lastRecoverMs) > 1500) {
      lastRecoverMs = now;

      if (failCounter >= failLimit) {
        logLine("[comp] I2C fail -> reconnect");
      }

      if (staleCounter >= staleLimit) {
        logLine("[comp] stale values -> reconnect");
      }

      recoverCompass();
      failCounter = 0;
      staleCounter = 0;

      data.valid = false;
      xQueueOverwrite(compassQueue, &data);
    }

    vTaskDelay(pdMS_TO_TICKS(1000 / COMP_FREQ));
  }
}

static float sanitizeStick(int raw, int threshold)
{
  float expo = 0.4f;
  expo = constrain(expo, 0.0f, 1.0f);

  const float diff = static_cast<float>(raw) - 127.0f;
  const float absDiff = fabsf(diff);

  if (absDiff <= static_cast<float>(threshold)) {
    return 0.0f;
  }

  float linear = (absDiff - static_cast<float>(threshold)) /
                 (100.0f - static_cast<float>(threshold));
  linear = constrain(linear, 0.0f, 1.0f);

  const float quadratic = linear * linear;
  const float shaped = (1.0f - expo) * linear + expo * quadratic;

  return copysignf(shaped, diff);
}

static void handleButtons(WiiData current, WiiData previous)
{
  static int rebootCounter = 0;

  if (!current.active) {
    return;
  }

  const bool abNow =
      ((current.btn & BUTTON_A) != 0) &&
      ((current.btn & BUTTON_B) != 0);

  const bool abOld =
      ((previous.btn & BUTTON_A) != 0) &&
      ((previous.btn & BUTTON_B) != 0);

  if (abNow && !abOld) {
    float heading = 0.0f;
    getCompassHeading(heading);

    startNav(
        heading,
        NAV_DEFAULT_SPEED_PERCENT,
        NAV_DEFAULT_TRIGGER_CM,
        NAV_DEFAULT_ROUNDS);

    logLineStr(
        "[wii] A+B -> nav heading=" + String(heading, 2) +
        " speed=" + String(NAV_DEFAULT_SPEED_PERCENT) +
        " dist=" + String(NAV_DEFAULT_TRIGGER_CM, 1) +
        " rounds=" + String(NAV_DEFAULT_ROUNDS));

    externsetPlayerLEDs(0b0110);
    return;
  }

  if ((current.btn & BUTTON_B) || (current.btn & BUTTON_Z)) {
    rebootCounter++;

    if (rebootCounter >= 700) {
      ESPreboot();
      return;
    }
  } else {
    rebootCounter = 0;
  }

  if (((current.btn & BUTTON_B) && !(previous.btn & BUTTON_B) && !(current.btn & BUTTON_A)) ||
      ((current.btn & BUTTON_Z) && !(previous.btn & BUTTON_Z))) {
    queueCancelCommand();
    logLine("[wii] B/Z -> cancel all");
    externsetPlayerLEDs(0x00);
    return;
  }

  if ((current.btn & BUTTON_TWO) && !(previous.btn & BUTTON_TWO)) {
    xTaskCreate(LedBatteryLvlTask, "BatteryLED", 2048, nullptr, 1, nullptr);
  }

  if ((current.btn & BUTTON_PLUS) && !(previous.btn & BUTTON_PLUS)) {
    startTurn(90.0f, true, BTN_TURN_SPEED_PERCENT, BTN_TURN_BIAS_PERCENT);

    logLineStr(
        "[wii] + -> turn 90 r speed=" + String(BTN_TURN_SPEED_PERCENT) +
        " bias=" + String(BTN_TURN_BIAS_PERCENT));

    externsetPlayerLEDs(0b0001);
  }

  if ((current.btn & BUTTON_MINUS) && !(previous.btn & BUTTON_MINUS)) {
    startTurn(90.0f, false, BTN_TURN_SPEED_PERCENT, BTN_TURN_BIAS_PERCENT);

    logLineStr(
        "[wii] - -> turn 90 l speed=" + String(BTN_TURN_SPEED_PERCENT) +
        " bias=" + String(BTN_TURN_BIAS_PERCENT));

    externsetPlayerLEDs(0b1000);
  }

  if ((current.btn & BUTTON_HOME) && !(previous.btn & BUTTON_HOME)) {
    float heading = 0.0f;
    getCompassHeading(heading);

    startLinear(heading, BTN_LINEAR_SPEED_PERCENT, BTN_LINEAR_DURATION_MS);

    logLineStr(
        "[wii] HOME -> linear heading=" + String(heading, 2) +
        " speed=" + String(BTN_LINEAR_SPEED_PERCENT) +
        " dur=" + String(BTN_LINEAR_DURATION_MS));

    externsetPlayerLEDs(0b1001);
  }
}

void WiiCon(void *parameter)
{
  wiimote.init();
  lastMs = millis();

  static WiiData previousData;
  WiiData currentData{};

  while (true) {
    previousData = currentData;
    wiimote.task();

    if (wiimote.available() > 0) {
      currentData.btn = wiimote.getButtonState();
      const NunchukState nkc = wiimote.getNunchukState();

      if (nkc.xStick == 0 || nkc.yStick == 0) {
        currentData.nkcx = 0.0f;
        currentData.nkcy = 0.0f;
      } else {
        currentData.nkcx = sanitizeStick(nkc.xStick, NKC_THRESHOLD);
        currentData.nkcy = sanitizeStick(nkc.yStick, NKC_THRESHOLD);

        if (currentData.btn & BUTTON_LEFT) {
          currentData.nkcx = -NO_NKC_V;
        }

        if (currentData.btn & BUTTON_RIGHT) {
          currentData.nkcx = NO_NKC_V;
        }

        if (currentData.btn & BUTTON_DOWN) {
          currentData.nkcy = -NO_NKC_V;
        }

        if (currentData.btn & BUTTON_UP) {
          currentData.nkcy = NO_NKC_V;
        }
      }

      lastMs = millis();
    }

    currentData.active = (millis() - lastMs) < WII_TIMEOUT_MS;
    handleButtons(currentData, previousData);

    xQueueOverwrite(wiiQueue, &currentData);
    vTaskDelay(pdMS_TO_TICKS(7));
  }
}

void LedBatteryLvlTask(void *pv)
{
  uint8_t ledHigh = 0x00;
  uint8_t ledLow = 0x00;

  for (int i = 0; i < 4; i++) {
    if (measVoltage >= batteryLevels[i]) {
      ledLow = ledHigh;
      ledHigh |= (1 << i);
      externsetPlayerLEDs(ledHigh);
      vTaskDelay(pdMS_TO_TICKS(300));
    }
  }

  if (ledHigh == 0x00) {
    ledLow = 0b00001100;
    ledHigh = 0b00000011;
  }

  for (int i = 0; i < 6; i++) {
    externsetPlayerLEDs(ledLow);
    vTaskDelay(pdMS_TO_TICKS(200));
    externsetPlayerLEDs(ledHigh);
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void taskHeartbeat(void *pv)
{
  uint32_t lastSendMs = 0;
  uint32_t lastPrintMs = 0;
  uint32_t lastMeasureMs = 0;

  float distRight = -1.0f;
  float distLeft = -1.0f;
  float batteryVoltage = 0.0f;
  float tempRight = 25.0f;
  float tempLeft = 25.0f;

  for (;;) {
    const uint32_t now = millis();

    CompassData compassData{};
    ThrottleData throttleData{};

    const bool compassOk =
        (compassQueue && xQueuePeek(compassQueue, &compassData, 0) == pdPASS);

    const bool throttleOk =
        (throttleQueue && xQueuePeek(throttleQueue, &throttleData, 0) == pdPASS);

    const bool distanceOk =
        ((distQueue1 && xQueuePeek(distQueue1, &distRight, 0) == pdPASS) &&
         (distQueue2 && xQueuePeek(distQueue2, &distLeft, 0) == pdPASS));

    if (now - lastMeasureMs >= MEAS_MS) {
      lastMeasureMs = now;

      const int raw = analogRead(BAT_PIN);
      batteryVoltage = raw * (3.3f / 4095.0f);

      tempRight = readTemp(R_NTC_PIN);
      tempLeft = readTemp(L_NTC_PIN);
    }

    if (tempLeft > 60.0f || tempRight > 60.0f) {
      logLineStr(
          "[warn] high temp R/L " +
          String(tempRight, 1) + "°C/" +
          String(tempLeft, 1) + "°C");
    }

    if (batteryVoltage < 2.3f) {
      logLineStr(
          "[warn] low battery " +
          String(batteryVoltage, 2) + " V");
    }

    if (wifiReady && webReady && throttleOk && compassOk && distanceOk &&
        (now - lastSendMs >= SEND_INTERVAL_MS)) {
      lastSendMs = now;

      float targetHeading = -1.0f;

      if (turnHeadingQueue) {
        xQueuePeek(turnHeadingQueue, &targetHeading, 0);
      }

      // P <heading_deg> <compass_valid> <right_pct> <left_pct> <dist_r_cm> <dist_l_cm> <target_heading> <battery_v> <mode>
      char msg[180];
      snprintf(
          msg,
          sizeof(msg),
          "P %.2f %d %d %d %.1f %.1f %.2f %.2f %u",
          compassData.angle_deg,
          compassData.valid ? 1 : 0,
          throttleData.rightPercent,
          throttleData.leftPercent,
          distRight,
          distLeft,
          targetHeading,
          batteryVoltage,
          static_cast<unsigned>(throttleData.mode));

      ws.textAll(msg);
    }

    if (compassOk && (now - lastPrintMs >= PRINT_INTERVAL_MS)) {
      lastPrintMs = now;

      WiiData wiiData{};
      const bool wiiOk = (wiiQueue && xQueuePeek(wiiQueue, &wiiData, 0) == pdPASS);

      if (wiiOk && throttleOk) {
        float targetHeading = -1.0f;

        if (turnHeadingQueue) {
          xQueuePeek(turnHeadingQueue, &targetHeading, 0);
        }

        logLineStr(
            "X/Y " + String(wiiData.nkcx, 2) + "/" + String(wiiData.nkcy, 2) +
            "  WiiC: " + String(wiiData.active) +
            "  Heading: " + String(compassData.angle_deg, 2) +
            "  C Valid: " + String(compassData.valid) +
            "  DR: " + String(distRight, 1) +
            "  DL: " + String(distLeft, 1) +
            "  Tgt: " + String(targetHeading, 2) +
            "  Mode: " + String(throttleData.mode) +
            "  PctR: " + String(throttleData.rightPercent) +
            "  PctL: " + String(throttleData.leftPercent) +
            "  Voltage: " + String(batteryVoltage, 2) +
            "  Temp R: " + String(tempRight, 1) +
            "  Temp L: " + String(tempLeft, 1) +
            "  IP: " + WiFi.localIP().toString());
      } else {
        logLineStr(
            "Heading: " + String(compassData.angle_deg, 2) +
            "  C Valid: " + String(compassData.valid) +
            "  WIFI: " + String(wifiReady) +
            "  IP: " + WiFi.localIP().toString());
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}