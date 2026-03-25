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

static constexpr const char *wifiSsid = "W";
static constexpr const char *wifiPass = "33333333";

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

  const float initialHeading = -1.0f;
  xQueueOverwrite(turnHeadingQueue, &initialHeading);

  const float initialDistance = -1.0f;
  xQueueOverwrite(distQueue1, &initialDistance);
  xQueueOverwrite(distQueue2, &initialDistance);

  ThrottleData throttleInit{};
  throttleInit.rightPercent = 0;
  throttleInit.leftPercent = 0;
  throttleInit.running = false;
  throttleInit.mode = MODE_IDLE;
  xQueueOverwrite(throttleQueue, &throttleInit);

  CompassData compassInit{};
  compassInit.valid = false;
  xQueueOverwrite(compassQueue, &compassInit);

  pinMode(WATER_PIN, INPUT_PULLDOWN);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  xTaskCreatePinnedToCore(compassTask, "Compass", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(WiiCon, "Wii", 4096, nullptr, 7, nullptr, 1);
  xTaskCreatePinnedToCore(ESCupdate, "ESC", 4096, nullptr, 6, nullptr, 1);
  xTaskCreatePinnedToCore(motionTask, "Motion", 4096, nullptr, 5, nullptr, 1);

  xTaskCreatePinnedToCore(taskLogger, "Logger", 4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(taskOTA, "OTA", 4096, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(taskWeb, "Web", 3072, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(taskHeartbeat, "Heartbeat", 4096, nullptr, 4, nullptr, 0);
  xTaskCreatePinnedToCore(DistTask, "Distance", 4096, nullptr, 3, nullptr, 1);

  WiFi.onEvent(onWiFiEvent);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(wifiSsid, wifiPass);

  logLine("[main] setup done, loop idle");
  Serial.println("Started");
}

void loop()
{
}