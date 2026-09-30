#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <QTRSensors.h>

#include "telemetry.h"
#include "robot_config.h"

#define LED_BUILTIN 2

uint32_t runStartTime = 0;
uint32_t lastLoopTimeMs = 0;


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
const int BASE_SPEED = RobotConfig::BASE_SPEED;
const int MAX_SPEED = RobotConfig::MAX_SPEED;
const float MOTOR_RAMP_STEP_PER_SEC = RobotConfig::MOTOR_RAMP_STEP_PER_SEC;
const float PID_DT_FALLBACK_SECONDS = RobotConfig::PID_DT_FALLBACK_SECONDS;
const float MAX_PID_INTEGRAL = RobotConfig::MAX_PID_INTEGRAL;

// --- PID Tuning Parameters ---
constexpr float Kp = RobotConfig::Kp;
constexpr float Ki = RobotConfig::Ki;
constexpr float Kd = RobotConfig::Kd;

int lastError = 0;
float integral = 0.0f;
int currentLeftSpeed = 0;
int currentRightSpeed = 0;

// --- ESP-NOW Configuration ---
uint8_t receiverAddress[] = {
  RobotConfig::RECEIVER_ADDRESS[0],
  RobotConfig::RECEIVER_ADDRESS[1],
  RobotConfig::RECEIVER_ADDRESS[2],
  RobotConfig::RECEIVER_ADDRESS[3],
  RobotConfig::RECEIVER_ADDRESS[4],
  RobotConfig::RECEIVER_ADDRESS[5]
};

TelemetryPacket telemetryData;

// Function Declarations
void initMotors();
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void stopRobot();
void initESPNow();
void sendTelemetry();
int moveToward(int currentValue, int targetValue, float maxDelta);

void setup() {
  Serial.begin(RobotConfig::SERIAL_BAUD_RATE);

  initMotors();

  // Initialize QTR Sensors in ANALOG mode
  qtr.setTypeAnalog();
  qtr.setSensorPins(RobotConfig::QTR_PINS, SENSOR_COUNT);
  qtr.setEmitterPin(RobotConfig::IR_EMITTER_PIN);

  // Initialize Wireless ESP-NOW
  initESPNow();

  // Calibration Phase (~3 seconds)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < RobotConfig::CALIBRATION_SAMPLES; i++) {
    qtr.calibrate();
    delay(RobotConfig::CALIBRATION_DELAY_MS);
  }
  digitalWrite(LED_BUILTIN, LOW);

  digitalWrite(STBY_PIN, HIGH);
  runStartTime = millis();
  lastLoopTimeMs = runStartTime;

  telemetryData.version = TELEMETRY_VERSION;
  telemetryData.packetLength = sizeof(TelemetryPacket);
}

void loop() {
  uint32_t currentMillis = millis();

  // Safety cutoff
  if (currentMillis - runStartTime >= RobotConfig::RUN_TIME_LIMIT_MS) {
    stopRobot();
    while (true) {
      delay(100);
    }
  }

  float dtSeconds = (currentMillis - lastLoopTimeMs) / 1000.0f;
  if (dtSeconds <= 0.0f || dtSeconds > 0.2f) {
    dtSeconds = PID_DT_FALLBACK_SECONDS;
  }
  lastLoopTimeMs = currentMillis;

  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = static_cast<int>(position) - 2500;

  bool lineDetected = false;
  uint16_t maxSensorReading = 0;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (sensorValues[i] > maxSensorReading) {
      maxSensorReading = sensorValues[i];
    }
    if (sensorValues[i] > RobotConfig::LINE_DETECTION_THRESHOLD) {
      lineDetected = true;
    }
  }

  if (!lineDetected && maxSensorReading < RobotConfig::LINE_LOSS_THRESHOLD) {
    integral *= 0.5f;
    lastError = error;

    int searchLeftTarget = (lastError >= 0) ? RobotConfig::SEARCH_TURN_SPEED : -RobotConfig::SEARCH_TURN_SPEED;
    int searchRightTarget = (lastError >= 0) ? -RobotConfig::SEARCH_TURN_SPEED : RobotConfig::SEARCH_TURN_SPEED;

    currentLeftSpeed = moveToward(currentLeftSpeed, searchLeftTarget, MOTOR_RAMP_STEP_PER_SEC * dtSeconds);
    currentRightSpeed = moveToward(currentRightSpeed, searchRightTarget, MOTOR_RAMP_STEP_PER_SEC * dtSeconds);
    setMotorSpeeds(currentLeftSpeed, currentRightSpeed);
  } else {
    integral += error * dtSeconds;
    integral = constrain(integral, -MAX_PID_INTEGRAL, MAX_PID_INTEGRAL);

    float derivative = (error - lastError) / dtSeconds;
    lastError = error;

    float adjustment = (Kp * error) + (Ki * integral) + (Kd * derivative);

    int leftTarget = BASE_SPEED + static_cast<int>(adjustment);
    int rightTarget = BASE_SPEED - static_cast<int>(adjustment);

    leftTarget = constrain(leftTarget, -MAX_SPEED, MAX_SPEED);
    rightTarget = constrain(rightTarget, -MAX_SPEED, MAX_SPEED);

    currentLeftSpeed = moveToward(currentLeftSpeed, leftTarget, MOTOR_RAMP_STEP_PER_SEC * dtSeconds);
    currentRightSpeed = moveToward(currentRightSpeed, rightTarget, MOTOR_RAMP_STEP_PER_SEC * dtSeconds);

    setMotorSpeeds(currentLeftSpeed, currentRightSpeed);
  }

  telemetryData.version = TELEMETRY_VERSION;
  telemetryData.packetLength = sizeof(TelemetryPacket);
  telemetryData.position = position;
  telemetryData.error = static_cast<int16_t>(error);
  telemetryData.leftMotorSpeed = static_cast<int16_t>(currentLeftSpeed);
  telemetryData.rightMotorSpeed = static_cast<int16_t>(currentRightSpeed);
  telemetryData.timestamp = currentMillis - runStartTime;

  for (int i = 0; i < SENSOR_COUNT; i++) {
    telemetryData.sensors[i] = sensorValues[i];
  }

  sendTelemetry();
}

int moveToward(int currentValue, int targetValue, float maxDelta) {
  if (currentValue < targetValue) {
    return min(currentValue + static_cast<int>(maxDelta), targetValue);
  }
  if (currentValue > targetValue) {
    return max(currentValue - static_cast<int>(maxDelta), targetValue);
  }
  return targetValue;
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
  int leftOutput = leftSpeed;
  int rightOutput = rightSpeed;

  // Motor A (Left)
  if (leftOutput >= 0) {
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, LOW);
  } else {
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, HIGH);
    leftOutput = -leftOutput;
  }
  analogWrite(PWMA_PIN, leftOutput);

  // Motor B (Right)
  if (rightOutput >= 0) {
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);
  } else {
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
    rightOutput = -rightOutput;
  }
  analogWrite(PWMB_PIN, rightOutput);
}

void stopRobot() {
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
  digitalWrite(STBY_PIN, LOW);
  analogWrite(PWMA_PIN, 0);
  analogWrite(PWMB_PIN, 0);

  pinMode(RobotConfig::IR_EMITTER_PIN, OUTPUT);
  digitalWrite(RobotConfig::IR_EMITTER_PIN, LOW);
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
  esp_err_t result = esp_now_send(receiverAddress, reinterpret_cast<uint8_t *>(&telemetryData), sizeof(telemetryData));
  if (result != ESP_OK) {
    // The robot continues running even if a packet is dropped; the receiver can tolerate this.
  }
}