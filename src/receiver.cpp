#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#define SENSOR_COUNT 6

typedef struct struct_telemetry {
  uint16_t sensors[SENSOR_COUNT];
  uint16_t position;
  int16_t error;
  int16_t leftMotorSpeed;
  int16_t rightMotorSpeed;
  uint32_t timestamp;
} struct_telemetry;

struct_telemetry incomingData;

#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 5
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingDataBytes, int len) {
#else
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingDataBytes, int len) {
#endif
  memcpy(&incomingData, incomingDataBytes, sizeof(incomingData));

  // --- TELEPLOT FORMATTING ---
  Serial.print(">position:"); Serial.print(incomingData.position); Serial.print(" ");
  Serial.print(">error:"); Serial.print(incomingData.error); Serial.print(" ");
  Serial.print(">leftSpeed:"); Serial.print(incomingData.leftMotorSpeed); Serial.print(" ");
  Serial.print(">rightSpeed:"); Serial.print(incomingData.rightMotorSpeed); Serial.print(" ");

  for (int i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(">S"); Serial.print(i + 1); Serial.print(":");
    Serial.print(incomingData.sensors[i]);
    if (i < SENSOR_COUNT - 1) Serial.print(" ");
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  Serial.println();
  Serial.print("RECEIVER MAC ADDRESS: ");
  Serial.println(WiFi.macAddress());
  Serial.println();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
}