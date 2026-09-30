#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <QTRSensors.h>

#define LED_BUILTIN 2

// --- Time Limit Safety ---
const uint32_t RUN_TIME_LIMIT_MS = 10000; // 10-second safety cutoff
uint32_t runStartTime = 0;

// --- Sensors (QTR-8A configured for 6 ADC1 pins) ---
#define SENSOR_COUNT 6
const uint8_t QTR_PINS[SENSOR_COUNT] = {36, 39, 34, 35, 32, 33}; 
#define IR_EMITTER_PIN 4

QTRSensors qtr;
uint16_t sensorValues[SENSOR_COUNT];

// --- TB6612FNG Motor Driver ---
#define PWMA_PIN 23
#define AIN1_PIN 22
#define AIN2_PIN 21

#define PWMB_PIN 18
#define BIN1_PIN 17
#define BIN2_PIN 16

#define STBY_PIN 5

// Speed Settings
const int BASE_SPEED = 180;
const int MAX_SPEED  = 255;

// --- PID Tuning Parameters ---
float Kp = 0.06f;
float Ki = 0.0001f;
float Kd = 0.6f;

int lastError = 0;
float integral = 0;

// --- ESP-NOW Configuration ---
// Replace with your receiver ESP32 MAC address
uint8_t receiverAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct struct_telemetry {
  uint16_t sensors[SENSOR_COUNT];
  uint16_t position;
  int16_t error;
  int16_t leftMotorSpeed;
  int16_t rightMotorSpeed;
  uint32_t timestamp;
} struct_telemetry;

struct_telemetry telemetryData;

// Function Declarations
void initMotors();
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void stopRobot();
void initESPNow();
void sendTelemetry();

void setup() {
  Serial.begin(115200);

  initMotors();

  // Initialize QTR Sensors in ANALOG mode
  qtr.setTypeAnalog();
  qtr.setSensorPins(QTR_PINS, SENSOR_COUNT);
  qtr.setEmitterPin(IR_EMITTER_PIN);

  // Initialize Wireless ESP-NOW
  initESPNow();

  // Calibration Phase (~3 seconds)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 150; i++) {
    qtr.calibrate();
    delay(20);
  }
  digitalWrite(LED_BUILTIN, LOW);

  // Enable Motor Driver
  digitalWrite(STBY_PIN, HIGH);

  runStartTime = millis();
}

void loop() {
  uint32_t currentMillis = millis();

  // Safety cutoff
  if (currentMillis - runStartTime >= RUN_TIME_LIMIT_MS) {
    stopRobot();
    while (true) {
      delay(100);
    }
  }

  // Position ranges from 0 to 5000 for 6 sensors; 2500 is center
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - 2500;

  // PID Calculations
  integral += error;
  integral = constrain(integral, -2000, 2000);

  int derivative = error - lastError;
  lastError = error;

  float adjustment = (Kp * error) + (Ki * integral) + (Kd * derivative);

  int leftSpeed  = BASE_SPEED + adjustment;
  int rightSpeed = BASE_SPEED - adjustment;

  leftSpeed  = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  setMotorSpeeds(leftSpeed, rightSpeed);

  // Send Telemetry via ESP-NOW
  telemetryData.position = position;
  telemetryData.error = error;
  telemetryData.leftMotorSpeed = leftSpeed;
  telemetryData.rightMotorSpeed = rightSpeed;
  telemetryData.timestamp = currentMillis - runStartTime;

  for (int i = 0; i < SENSOR_COUNT; i++) {
    telemetryData.sensors[i] = sensorValues[i];
  }

  sendTelemetry();
}

void initMotors() {
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);

  pinMode(PWMB_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);

  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, LOW);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Motor A (Left)
  if (leftSpeed >= 0) {
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, LOW);
  } else {
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(PWMA_PIN, leftSpeed);

  // Motor B (Right)
  if (rightSpeed >= 0) {
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);
  } else {
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
    rightSpeed = -rightSpeed;
  }
  analogWrite(PWMB_PIN, rightSpeed);
}

void stopRobot() {
  digitalWrite(STBY_PIN, LOW);
  analogWrite(PWMA_PIN, 0);
  analogWrite(PWMB_PIN, 0);

  pinMode(IR_EMITTER_PIN, OUTPUT);
  digitalWrite(IR_EMITTER_PIN, LOW);
}

void initESPNow() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) return;

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  esp_now_add_peer(&peerInfo);
}

void sendTelemetry() {
  esp_now_send(receiverAddress, (uint8_t *)&telemetryData, sizeof(telemetryData));
}