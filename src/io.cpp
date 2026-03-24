#include "io.h"
#include "motion.h"

// ======================== Module State ========================
static int battery_lvl[4] = {3000, 3600, 3900, 4050};
static int measVoltage = 4000;

static volatile uint32_t echoStartUs[2] = {0, 0};
static volatile uint32_t echoEndUs[2] = {0, 0};
static volatile bool echoDone[2] = {false, false};
static volatile bool waitingRise[2] = {true, true};

static ESP32Wiimote wiimote;
static long last_ms = 0;

static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");

static volatile bool wifiReady = false;
static volatile bool otaReady = false;
static volatile bool webReady = false;

// =================== Internal Helpers ===================
static void IRAM_ATTR handleEchoISR(uint8_t idx, uint8_t echoPin);
static void IRAM_ATTR echoISR1();
static void IRAM_ATTR echoISR2();
static void trigPulse(uint8_t trigPin);
static void publishDistance(uint8_t idx, float dist);

static void init_compass();
static bool read_compass(int16_t &x, int16_t &y, int16_t &z);
static bool compassBusRecover();
static void compassRecover();

static float sanitizeStick(int raw, int trsh);
static void ButtonCheck(WiiData wii, WiiData wiiold);

static void handleConsoleLine(const char *line);
static void onWsEvent(AsyncWebSocket *serverPtr, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len);
static void startWebOnce();
static void startOTAOnce();

float readTemp(int pin) {
  int raw = analogRead(pin);
  float voltage = raw / ADC_MAX;
  float resistance = R_REF * (1.0 / (1.0 / voltage - 1.0));

  // simplified Beta formula (Steinhart-Hart)
  float tempK = 1.0 / (1.0 / (T_NOM + 273.15) + (1.0 / B_VAL) * log(resistance / R_NOM));
  return tempK - 273.15;
}

void ESPreboot() {
  digitalWrite(BUZZER_PIN, HIGH);
  logLine("---REBOOT---");
  externsetPlayerLEDs(0x00);
  vTaskDelay(pdMS_TO_TICKS(500));
  ESP.restart();
}

// ======================== Logging ==========================
void logLine(const char *msg)
{
  if (!logQueue)
    return;

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
  uint32_t lastWeb = 0;

  for (;;)
  {
    if (xQueueReceive(logQueue, line, portMAX_DELAY) == pdTRUE)
    {
      Serial.println(line);

      if (webReady)
      {
        uint32_t now = millis();
        if ((uint32_t)(now - lastWeb) >= 35)
        {
          lastWeb = now;
          ws.textAll(line);
        }
      }
    }
  }
}

// ======================== Console Parsing ==================
static void handleConsoleLine(const char *line)
{
  String s(line);
  s.trim();
  if (s.length() == 0)
    return;

  if (s.equalsIgnoreCase("cancel") || s.equalsIgnoreCase("stop"))
  {
    queueCancelCommand();
    logLine("CMD: cancel queued");
    return;
  }

  // Reset / Reboot command
  if (s.equalsIgnoreCase("reset"))
  {
    ESPreboot();
    return;
  }

  if (s.equalsIgnoreCase("nav"))
  {
    float h = 0.0f;
    getCompassHeading(h);
    startNav(h, NAV_DEFAULT_SPEED_PERCENT, NAV_DEFAULT_TRIGGER_CM, NAV_DEFAULT_ROUNDS);
    logLineStr("CMD: nav default heading=" + String(h, 2) +
               " speed=" + String(NAV_DEFAULT_SPEED_PERCENT) +
               " dist=" + String(NAV_DEFAULT_TRIGGER_CM, 1) +
               " rounds=" + String(NAV_DEFAULT_ROUNDS));
    return;
  }

  int r, l;
  unsigned long t;
  if (sscanf(s.c_str(), "hs %d %d %lu", &r, &l, &t) == 3)
  {
    startHs(r, l, (uint32_t)t);
    logLineStr("CMD: hs R=" + String(r) + " L=" + String(l) + " t=" + String((uint32_t)t));
    return;
  }

  float angle;
  char dir;
  int speed;
  int bias;
  int turnCount = sscanf(s.c_str(), "turn %f %c %d %d", &angle, &dir, &speed, &bias);
  if (turnCount == 3 || turnCount == 4)
  {
    dir = (char)tolower((unsigned char)dir);
    if (dir != 'r' && dir != 'l')
    {
      logLine("turn dir must be r or l");
      return;
    }

    startTurn(angle, dir == 'r', speed, turnCount == 4 ? bias : 0);
    logLineStr("CMD: turn angle=" + String(angle, 2) +
               " dir=" + String(dir) +
               " speed=" + String(speed) +
               " bias=" + String(turnCount == 4 ? bias : 0));
    return;
  }

  float heading;
  float minDist;
  unsigned long timeMs;
  int linearCount = sscanf(s.c_str(), "linear %f %d %lu %f", &heading, &speed, &timeMs, &minDist);
  if (linearCount == 3 || linearCount == 4)
  {
    startLinear(heading, speed, (uint32_t)timeMs, linearCount == 4 ? minDist : -1.0f);
    logLineStr("CMD: linear heading=" + String(heading, 2) +
               " speed=" + String(speed) +
               " t=" + String((uint32_t)timeMs) +
               " minDist=" + String(linearCount == 4 ? minDist : -1.0f, 1));
    return;
  }

  int rounds;
  if (sscanf(s.c_str(), "nav %f %d %f %d", &heading, &speed, &minDist, &rounds) == 4)
  {
    startNav(heading, speed, minDist, rounds);
    logLineStr("CMD: nav heading=" + String(heading, 2) +
               " speed=" + String(speed) +
               " minDist=" + String(minDist, 1) +
               " rounds=" + String(rounds));
    return;
  }

  logLine("unknown cmd (use: hs <r> <l> <ms> | turn <angle> <r|l> <speed> [bias] | linear <heading> <speed> <ms> [minDist] | nav <heading> <speed> <minDist> <rounds> | cancel | reset)");
}

// ======================== WebSocket ========================
static void onWsEvent(AsyncWebSocket *serverPtr, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  if (type != WS_EVT_DATA)
    return;

  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (!info->final || info->index != 0 || info->len != len)
    return;
  if (info->opcode != WS_TEXT)
    return;

  String msg;
  msg.reserve(len);
  for (size_t i = 0; i < len; i++)
    msg += (char)data[i];

  msg.trim();
  if (msg.length() == 0)
    return;

  handleConsoleLine(msg.c_str());
}

static void startWebOnce()
{
  if (webReady)
    return;

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/html; charset=utf-8",
                              index_html_start,
                              index_html_end - index_html_start); });

  server.begin();
  webReady = true;
  logLine("Web server started on port 80");
}

void taskWeb(void *pv)
{
  for (;;)
  {
    if (wifiReady && webReady)
      ws.cleanupClients();
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// ======================== OTA / WiFi =======================
static void startOTAOnce()
{
  if (otaReady)
    return;

  ArduinoOTA.setHostname("ESP32-OTA");
  ArduinoOTA.setPassword("33333333");

  ArduinoOTA.onStart([]()
                     { logLine("OTA: Start"); });
  ArduinoOTA.onEnd([]()
                   { logLine("OTA: End"); });
  ArduinoOTA.onError([](ota_error_t err)
                     {
    char b[LOG_LINE_MAX];
    snprintf(b, sizeof(b), "OTA: Error %u", (unsigned)err);
    logLine(b); });

  ArduinoOTA.begin();
  otaReady = true;
  logLine("OTA ready (password protected)");
}

void onWiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
  case ARDUINO_EVENT_WIFI_STA_GOT_IP:
    wifiReady = true;
    logLineStr("WiFi: connected, IP: " + WiFi.localIP().toString());
    startOTAOnce();
    startWebOnce();
    break;

  case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
    wifiReady = false;
    logLine("WiFi: disconnected");
    break;

  default:
    break;
  }
}

void taskOTA(void *pv)
{
  for (;;)
  {
    if (wifiReady && otaReady)
      ArduinoOTA.handle();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ======================== Distance =========================
static void IRAM_ATTR handleEchoISR(uint8_t idx, uint8_t echoPin)
{
  bool level = digitalRead(echoPin);
  if (level)
  {
    if (waitingRise[idx])
    {
      echoStartUs[idx] = micros();
      waitingRise[idx] = false;
    }
  }
  else
  {
    if (!waitingRise[idx])
    {
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

static void publishDistance(uint8_t idx, float dist)
{
  if (idx == 0)
    xQueueOverwrite(distQueue1, &dist);
  else
    xQueueOverwrite(distQueue2, &dist);
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

  while (true)
  {
    uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - lastPingMs) < PING_GAP_MS)
    {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    noInterrupts();
    echoDone[sensor] = false;
    waitingRise[sensor] = true;
    interrupts();

    trigPulse(trigPins[sensor]);
    lastPingMs = millis();
    uint32_t trigSentUs = micros();

    float dist = -1.0f;
    while (true)
    {
      bool done;
      noInterrupts();
      done = echoDone[sensor];
      interrupts();

      if (done)
      {
        uint32_t startUs, endUs;
        noInterrupts();
        startUs = echoStartUs[sensor];
        endUs = echoEndUs[sensor];
        echoDone[sensor] = false;
        waitingRise[sensor] = true;
        interrupts();

        uint32_t dur = (uint32_t)(endUs - startUs);
        dist = (float)dur * 0.0343f * 0.5f;

        if (dist < MIN_CM || dist > MAX_CM)
          dist = -1.0f;
        break;
      }

      if ((uint32_t)(micros() - trigSentUs) > ECHO_TIMEOUT_US)
      {
        dist = -1.0f;
        break;
      }

      vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (dist >= 0.0f)
    {
      if (filteredDist[sensor] < 0.0f){
        filteredDist[sensor] = dist;  // Initialwert setzen, aber noch nicht publishen
        }
      else
      {
        filteredDist[sensor] += DIST_FILTER_ALPHA * (dist - filteredDist[sensor]);
        publishDistance(sensor, filteredDist[sensor]);
      }
      invalidHold[sensor] = 0;  // immer zurücksetzen wenn Messung gültig
      }
      else
      {
      if (invalidHold[sensor] < DIST_INVALID_HOLD_COUNT)
      {
        invalidHold[sensor]++;
        if (filteredDist[sensor] >= 0.0f)
            publishDistance(sensor, filteredDist[sensor]);  // letzten gültigen Wert halten
      }
      else
      {
        filteredDist[sensor] = -1.0f;
        publishDistance(sensor, -1.0f);
      }
    }
    sensor ^= 1;
  }
}

// ======================== Compass ==========================
static void init_compass()
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

static bool read_compass(int16_t &x, int16_t &y, int16_t &z)
{
  Wire.beginTransmission(HMC5883L_ADDRESS);
  Wire.write(0x03);
  if (Wire.endTransmission() != 0)
    return false;

  if (Wire.requestFrom((uint16_t)HMC5883L_ADDRESS, (uint8_t)6) != 6)
    return false;
  if (Wire.available() != 6)
    return false;

  x = (int16_t)(Wire.read() << 8 | Wire.read());
  z = (int16_t)(Wire.read() << 8 | Wire.read());
  y = (int16_t)(Wire.read() << 8 | Wire.read());
  return true;
}

static bool compassBusRecover()
{

  pinMode(COMP_SDA, INPUT_PULLUP);
  pinMode(COMP_SCL, OUTPUT_OPEN_DRAIN);
  digitalWrite(COMP_SCL, HIGH);
  delay(2);

  for (int i = 0; i < 9; i++)
  {
    digitalWrite(COMP_SCL, LOW);
    delayMicroseconds(10);
    digitalWrite(COMP_SDA, HIGH);
    delayMicroseconds(10);
    if (digitalRead(COMP_SDA) == HIGH)
      break;
  }

  vTaskDelay(1);

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

static void compassRecover()
{
  logLine("[comp] recover start");

  compassBusRecover();

  Wire.begin(COMP_SDA, COMP_SCL);
  Wire.setTimeOut(3);
  init_compass();

  logLine("[comp] recover init done");
}

void compassTask(void *parameter)
{
  Wire.begin(COMP_SDA, COMP_SCL);
  Wire.setTimeOut(3);
  init_compass();

  CompassData d{};
  int16_t lastX = 0, lastY = 0, lastZ = 0;
  uint32_t staleCounter = 0;
  uint32_t failCounter = 0;

  const uint32_t STALE_LIMIT = 25;
  const uint32_t FAIL_LIMIT = 5;
  uint32_t lastRecoverMs = 0;

  while (true)
  {
    int16_t x, y, z;
    bool ok = read_compass(x, y, z);

    if (ok)
    {
      if (x == lastX && y == lastY && z == lastZ)
        staleCounter++;
      else
        staleCounter = 0;

      lastX = x;
      lastY = y;
      lastZ = z;
      failCounter = 0;

      float a = atan2f((float)y, (float)x);
      if (a < 0.0f)
        a += 2.0f * PI;

      d.x = x;
      d.y = y;
      d.z = z;
      d.angle_deg = a * 180.0f / PI;
      d.valid = true;
    }
    else
    {
      d.valid = false;
      failCounter++;
      staleCounter = 0;
      vTaskDelay(pdMS_TO_TICKS(30));
    }

    xQueueOverwrite(compassQueue, &d);

    uint32_t now = millis();
    bool needRecover = (failCounter >= FAIL_LIMIT) || (staleCounter >= STALE_LIMIT);

    if (needRecover && (now - lastRecoverMs) > 1500)
    {
      lastRecoverMs = now;

      if (failCounter >= FAIL_LIMIT)
        logLine("[comp] I2C fail -> reconnect");
      if (staleCounter >= STALE_LIMIT)
        logLine("[comp] stale values -> reconnect");

      compassRecover();
      failCounter = 0;
      staleCounter = 0;

      d.valid = false;
      xQueueOverwrite(compassQueue, &d);
    }

    vTaskDelay(pdMS_TO_TICKS(1000 / COMP_FREQ));
  }
}

// ======================== Wii / LED ========================
static float sanitizeStick(int raw, int trsh)
{
  float expo = 0.4f;
  expo = constrain(expo, 0.0f, 1.0f);

  float diff = (float)raw - 127.0f;
  float a = fabsf(diff);

  if (a <= (float)trsh)
    return 0.0f;

  float lin = (a - (float)trsh) / (100.0f - (float)trsh);
  lin = constrain(lin, 0.0f, 1.0f);

  float quad = lin * lin;
  float shaped = (1.0f - expo) * lin + expo * quad;

  return copysignf(shaped, diff);
}

static void ButtonCheck(WiiData wii, WiiData wiiold)
{
  static int times_b_pressed = 0;
  if (!wii.active)
    return;

  bool abNow = ((wii.btn & BUTTON_A) != 0) && ((wii.btn & BUTTON_B) != 0);
  bool abOld = ((wiiold.btn & BUTTON_A) != 0) && ((wiiold.btn & BUTTON_B) != 0);

  if (abNow && !abOld)
  {
    float h = 0.0f;
    getCompassHeading(h);
    startNav(h, NAV_DEFAULT_SPEED_PERCENT, NAV_DEFAULT_TRIGGER_CM, NAV_DEFAULT_ROUNDS);
    logLineStr("BTN A+B -> nav heading=" + String(h, 2) +
               " speed=" + String(NAV_DEFAULT_SPEED_PERCENT) +
               " dist=" + String(NAV_DEFAULT_TRIGGER_CM, 1) +
               " rounds=" + String(NAV_DEFAULT_ROUNDS));
    externsetPlayerLEDs(0b0110);
    return;
  }

  if ((wii.btn & BUTTON_B) || (wii.btn & BUTTON_Z))
  {
    times_b_pressed++;
    if (times_b_pressed >= 700)
    {
      ESPreboot();
      return;
    }
  }
  else
  {
    times_b_pressed = 0;
  }

  if (((wii.btn & BUTTON_B) && !(wiiold.btn & BUTTON_B) && !(wii.btn & BUTTON_A)) ||
      ((wii.btn & BUTTON_Z) && !(wiiold.btn & BUTTON_Z)))
  {
    queueCancelCommand();
    logLine("BTN B/Z -> cancel all");
    externsetPlayerLEDs(0x00);
    return;
  }

  if ((wii.btn & BUTTON_TWO) && !(wiiold.btn & BUTTON_TWO))
  {
    xTaskCreate(LedBatteryLvlTask, "BatteryLED", 2048, NULL, 1, NULL);
  }
  if ((wii.btn & BUTTON_PLUS) && !(wiiold.btn & BUTTON_PLUS))
  {
    startTurn(90.0f, true, BTN_TURN_SPEED_PERCENT, BTN_TURN_BIAS_PERCENT);
    logLineStr("BTN + -> turn 90 r speed=" + String(BTN_TURN_SPEED_PERCENT) +
               " bias=" + String(BTN_TURN_BIAS_PERCENT));
    externsetPlayerLEDs(0b0001);
  }

  if ((wii.btn & BUTTON_MINUS) && !(wiiold.btn & BUTTON_MINUS))
  {
    startTurn(90.0f, false, BTN_TURN_SPEED_PERCENT, BTN_TURN_BIAS_PERCENT);
    logLineStr("BTN - -> turn 90 l speed=" + String(BTN_TURN_SPEED_PERCENT) +
               " bias=" + String(BTN_TURN_BIAS_PERCENT));
    externsetPlayerLEDs(0b1000);
  }

  if ((wii.btn & BUTTON_HOME) && !(wiiold.btn & BUTTON_HOME))
  {
    float h = 0.0f;
    getCompassHeading(h);
    startLinear(h, BTN_LINEAR_SPEED_PERCENT, BTN_LINEAR_DURATION_MS);
    logLineStr("HOME -> linear heading=" + String(h, 2) +
               " speed=" + String(BTN_LINEAR_SPEED_PERCENT) +
               " dur=" + String(BTN_LINEAR_DURATION_MS));
    externsetPlayerLEDs(0b1001);
  }
}

void WiiCon(void *parameter)
{
  wiimote.init();
  last_ms = millis();

  static WiiData dataold;
  WiiData data{};

  while (true)
  {
    dataold = data;
    wiimote.task();

    if (wiimote.available() > 0)
    {
      data.btn = wiimote.getButtonState();
      NunchukState nkc = wiimote.getNunchukState();

      if (nkc.xStick == 0 || nkc.yStick == 0)
      {
        data.nkcx = 0.0f;
        data.nkcy = 0.0f;
      }
      else
      {
        data.nkcx = sanitizeStick(nkc.xStick, NKC_THRESHOLD);
        data.nkcy = sanitizeStick(nkc.yStick, NKC_THRESHOLD);

        if (data.btn & BUTTON_LEFT)
          data.nkcx = -NO_NKC_V;
        if (data.btn & BUTTON_RIGHT)
          data.nkcx = NO_NKC_V;
        if (data.btn & BUTTON_DOWN)
          data.nkcy = -NO_NKC_V;
        if (data.btn & BUTTON_UP)
          data.nkcy = NO_NKC_V;
      }

      last_ms = millis();
    }

    data.active = (millis() - last_ms) < WII_TIMEOUT_MS;
    ButtonCheck(data, dataold);

    xQueueOverwrite(wiiQueue, &data);
    vTaskDelay(pdMS_TO_TICKS(7));
  }
}

void LedBatteryLvlTask(void *pv)
{
  uint8_t LED_h = 0x00;
  uint8_t LED_l = 0x00;

  for (int i = 0; i < 4; i++)
    {
      if (measVoltage >= battery_lvl[i])
      {
        LED_l = LED_h;
        LED_h |= (1 << i);
        externsetPlayerLEDs(LED_h);
        vTaskDelay(pdMS_TO_TICKS(300));
      }
    }

    if (LED_h == 0x00)
    {
      LED_l = 0b00001100;
      LED_h = 0b00000011;
    }

    for (int i = 0; i < 6; i++)
    {
    externsetPlayerLEDs(LED_l);
    vTaskDelay(pdMS_TO_TICKS(200));
    externsetPlayerLEDs(LED_h);
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// ======================== Heartbeat ========================
void taskHeartbeat(void *pv)
{
  uint32_t lastSend = 0;
  uint32_t lastPrint = 0;
  uint32_t lastMeas = 0;
  uint32_t lastNTC = 0;

  float d1 = -1.0f;
  float d2 = -1.0f;
  float battvoltage = 0.0f;
  float tempR = 25.0f;
  float tempL = 25.0f;

  for (;;)
  {
    uint32_t now = millis();

    CompassData cd{};
    ThrottleData td{};
    bool okC = (compassQueue && xQueuePeek(compassQueue, &cd, 0) == pdPASS);
    bool okT = (throttleQueue && xQueuePeek(throttleQueue, &td, 0) == pdPASS);
    bool okD = ((distQueue1 && xQueuePeek(distQueue1, &d1, 0) == pdPASS) &&
                (distQueue2 && xQueuePeek(distQueue2, &d2, 0) == pdPASS));

    if (now - lastMeas >= MEAS_MS)
    {
      lastMeas = now;
      int raw = analogRead(BAT_PIN);
      battvoltage = raw * (3.3f / 4095.0f);

      tempR = readTemp(R_NTC_PIN);
      tempL = readTemp(L_NTC_PIN);
    }

    if(tempL > 60.0f || tempR > 60.0f)
    {
      logLineStr("WARNING: High temp R/L" + String(tempL, 1) + "°C/" + String(tempR, 1) + "°C");
    }
    if(battvoltage < 2.3f)
    {
      logLineStr("WARNING: Low battery" + String(battvoltage, 2) + "V"); //ADC voltage reading
    }

    if (wifiReady && webReady && okT && okC && okD && (now - lastSend >= SEND_INTERVAL_MS))
    {
      lastSend = now;

      float th = -1.0f;
      if (turnHeadingQueue)
        xQueuePeek(turnHeadingQueue, &th, 0);

      char msg[180];
      snprintf(msg, sizeof(msg), "P %.2f %d %d %d %.1f %.1f %.2f %.2f %u",
               cd.angle_deg,
               cd.valid ? 1 : 0,
               td.rightPercent,
               td.leftPercent,
               d1,
               d2,
               th,
               battvoltage,
               (unsigned)td.mode);

      ws.textAll(msg);
    }

    if (okC && (now - lastPrint >= PRINT_INTERVAL_MS))
    {
      lastPrint = now;

      WiiData wii{};
      bool okW = (wiiQueue && xQueuePeek(wiiQueue, &wii, 0) == pdPASS);

      if (okW && okT)
      {
        float th = -1.0f;
        if (turnHeadingQueue)
          xQueuePeek(turnHeadingQueue, &th, 0);

        logLineStr(
            "X/Y " + String(wii.nkcx, 2) + "/" + String(wii.nkcy, 2) +
            "  WiiC: " + String(wii.active) +
            "  Heading: " + String(cd.angle_deg, 2) +
            "  C Valid: " + String(cd.valid) +
            "  DR: " + String(d1, 1) +
            "  DL: " + String(d2, 1) +
            "  Tgt: " + String(th, 2) +
            "  Mode: " + String(td.mode) +
            "  PctR: " + String(td.rightPercent) +
            "  PctL: " + String(td.leftPercent) +
            "  Voltage: " + String(battvoltage, 2) +
            "  Temp R: " + String(tempR, 1) +
            "  Temp L: " + String(tempL, 1) +
            "  IP: " + WiFi.localIP().toString());
      }
      else
      {
        logLineStr(
            "Heading: " + String(cd.angle_deg, 2) +
            "  C Valid: " + String(cd.valid) +
            "  WIFI: " + String(wifiReady) +
            "  IP: " + WiFi.localIP().toString());
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
