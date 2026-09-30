#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#include "telemetry.h"

TelemetryPacket incomingData;

// Callback function executed when ESP-NOW data is received
#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 5
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingDataBytes, int len) {
#else
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingDataBytes, int len) {
#endif
  if (len < static_cast<int>(sizeof(TelemetryPacket))) {
    Serial.printf("Discarded malformed packet: len=%d expected=%u\n", len, sizeof(TelemetryPacket));
    return;
  }

  memcpy(&incomingData, incomingDataBytes, sizeof(incomingData));

  if (incomingData.version != TELEMETRY_VERSION || incomingData.packetLength != sizeof(TelemetryPacket)) {
    Serial.printf("Discarded mismatched telemetry packet: version=%u length=%u expectedVersion=%u expectedLength=%u\n",
                  incomingData.version,
                  incomingData.packetLength,
                  TELEMETRY_VERSION,
                  sizeof(TelemetryPacket));
    return;
  }

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