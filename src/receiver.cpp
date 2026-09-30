#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// Must match the exact structure defined on the robot
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

// Callback function executed when ESP-NOW data is received
#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 5
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingDataBytes, int len) {
#else
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingDataBytes, int len) {
#endif
  memcpy(&incomingData, incomingDataBytes, sizeof(incomingData));

  // --- TELEPLOT SERIAL OUTPUT ---
  // Teleplot requires each variable message to be formatted as: >variable_name:value\n
  Serial.print(">timestamp:"); Serial.println(incomingData.timestamp);
  Serial.print(">position:"); Serial.println(incomingData.position);
  Serial.print(">error:"); Serial.println(incomingData.error);
  Serial.print(">leftMotorSpeed:"); Serial.println(incomingData.leftMotorSpeed);
  Serial.print(">rightMotorSpeed:"); Serial.println(incomingData.rightMotorSpeed);

  // Print individual sensor values
  for (int i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(">S"); Serial.print(i + 1); Serial.print(":");
    Serial.println(incomingData.sensors[i]);
  }
}

void setup() {
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register telemetry callback
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // Nothing needed here; receiving is handled asynchronously via callback
}