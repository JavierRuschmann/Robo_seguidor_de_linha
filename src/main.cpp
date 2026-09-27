#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Set WiFi to Station mode to initialize the network interface
  WiFi.mode(WIFI_STA);

  // Read and print the default Wi-Fi Station MAC address
  Serial.println();
  Serial.print("ESP32 Board MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  // Nothing needed here
}