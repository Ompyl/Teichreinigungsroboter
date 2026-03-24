#include "main.h"
#include "motion.h"
#include "io.h"

QueueHandle_t logQueue = nullptr;
QueueHandle_t wiiQueue = nullptr;
QueueHandle_t throttleQueue = nullptr;
QueueHandle_t turnHeadingQueue = nullptr;
QueueHandle_t distQueue1 = nullptr;
QueueHandle_t distQueue2 = nullptr;
QueueHandle_t motionCmdQueue = nullptr;
QueueHandle_t compassQueue = nullptr;

static const char *ssid = "W";
static const char *pass = "33333333";

void setup()
{
  Serial.begin(115200);
  Serial.println("Begin");

  logQueue = xQueueCreate(40, LOG_LINE_MAX);
  motionCmdQueue = xQueueCreate(1, sizeof(MotionCommand));
  throttleQueue = xQueueCreate(1, sizeof(ThrottleData));
  distQueue1 = xQueueCreate(1, sizeof(float));
  distQueue2 = xQueueCreate(1, sizeof(float));
  turnHeadingQueue = xQueueCreate(1, sizeof(float));
  compassQueue = xQueueCreate(1, sizeof(CompassData));
  wiiQueue = xQueueCreate(1, sizeof(WiiData));

  float thInit = -1.0f;
  xQueueOverwrite(turnHeadingQueue, &thInit);

  float dinit = -1.0f;
  xQueueOverwrite(distQueue1, &dinit);
  xQueueOverwrite(distQueue2, &dinit);

  ThrottleData td0{};
  td0.rightPercent = 0;
  td0.leftPercent = 0;
  td0.running = false;
  td0.mode = MODE_IDLE;
  xQueueOverwrite(throttleQueue, &td0);

  CompassData cd0{};
  cd0.valid = false;
  xQueueOverwrite(compassQueue, &cd0);
  
  pinMode(WATER_PIN, INPUT_PULLDOWN);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  xTaskCreatePinnedToCore(compassTask, "Compass", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(WiiCon, "Wii Controller Task", 4096, nullptr, 7, nullptr, 1);
  xTaskCreatePinnedToCore(ESCupdate, "ESC update Task", 4096, nullptr, 6, nullptr, 1);
  xTaskCreatePinnedToCore(motionTask, "Motion Task", 4096, nullptr, 5, nullptr, 1);

  xTaskCreatePinnedToCore(taskLogger, "logger", 4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(taskOTA, "ota", 4096, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(taskWeb, "web", 3072, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(taskHeartbeat, "heartbeat", 4096, nullptr, 4, nullptr, 0);
  xTaskCreatePinnedToCore(DistTask, "Distance Sensor Task", 4096, nullptr, 3, nullptr, 1);

  WiFi.onEvent(onWiFiEvent);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(ssid, pass);

  logLine("Setup done. Main loop idle.");
  Serial.println("Started");
}

void loop()
{
}
