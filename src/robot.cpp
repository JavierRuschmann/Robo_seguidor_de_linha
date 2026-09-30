#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <QTRSensors.h>

// ============================================================================
// CONFIGURATION & PIN DEFINITIONS
// ============================================================================

#define LED_BUILTIN 2

// --- Time Limit ---
const uint32_t RUN_TIME_LIMIT_MS = 10000; // Run duration (e.g., 10 seconds)
uint32_t runStartTime = 0;

// --- Sensors (QTR-8RC) ---
#define SENSOR_COUNT 8
const uint8_t QTR_PINS[SENSOR_COUNT] = {13, 14, 27, 26, 25, 33, 32, 19};
#define IR_EMITTER_PIN 4

QTRSensors qtr;
uint16_t sensorValues[SENSOR_COUNT];

// --- TB6612FNG Motor Driver Pins ---
#define PWMA_PIN 23
#define AIN1_PIN 22
#define AIN2_PIN 21

#define PWMB_PIN 18
#define BIN1_PIN 17
#define BIN2_PIN 16

#define STBY_PIN 5

// --- Motor Speed Settings ---
const int BASE_SPEED = 180; // Default base PWM (0-255)
const int MAX_SPEED  = 255; // Maximum PWM limit

// --- PID Control Parameters ---
float Kp = 0.08f;
float Ki = 0.0001f;
float Kd = 0.8f;

int lastError = 0;
float integral = 0;

// --- ESP-NOW Configuration ---
// Replace with the MAC Address of your receiver ESP32 board
// 8C:94:DF:4C:71:90
uint8_t receiverAddress[] = {0x8C, 0x94, 0xDF, 0x4C, 0x71, 0x90};

typedef struct struct_telemetry {
  uint16_t sensors[SENSOR_COUNT];
  uint16_t position;
  int16_t error;
  int16_t leftMotorSpeed;
  int16_t rightMotorSpeed;
  uint32_t timestamp;
} struct_telemetry;

struct_telemetry telemetryData;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
void initMotors();
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void stopRobot();
void initESPNow();
void sendTelemetry();

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);

  // Initialize motor control pins
  initMotors();

  // Initialize QTR Sensors
  qtr.setTypeRC();
  qtr.setSensorPins(QTR_PINS, SENSOR_COUNT);
  qtr.setEmitterPin(IR_EMITTER_PIN);

  // Initialize Wireless ESP-NOW
  initESPNow();

  // Calibration Phase (5 seconds)
  // Move the robot back and forth over the line during power-up
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  for (uint16_t i = 0; i < 250; i++) {
    qtr.calibrate();
    delay(20);
  }
  
  digitalWrite(LED_BUILTIN, LOW); // LED off = calibration complete

  // Enable Motor Driver (STBY HIGH)
  digitalWrite(STBY_PIN, HIGH);

  // Record start time of the run
  runStartTime = millis();
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  uint32_t currentMillis = millis();

  // Check if run time limit is reached
  if (currentMillis - runStartTime >= RUN_TIME_LIMIT_MS) {
    stopRobot();
    while (true) {
      // Robot disabled; stay in idle loop
      delay(100);
    }
  }

  // 1. Read sensor array position (0 to 7000 for 8 sensors; 3500 is center)
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - 3500;

  // 2. PID Calculation
  integral += error;
  // Anti-windup constraint
  integral = constrain(integral, -2000, 2000); 

  int derivative = error - lastError;
  lastError = error;

  float adjustment = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // 3. Calculate target motor speeds
  int leftSpeed  = BASE_SPEED + adjustment;
  int rightSpeed = BASE_SPEED - adjustment;

  leftSpeed  = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  // 4. Drive Motors
  setMotorSpeeds(leftSpeed, rightSpeed);

  // 5. Send ESP-NOW Telemetry Data
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

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

void initMotors() {
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  
  pinMode(PWMB_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, LOW); // Disabled initially
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
  // Disable motor driver
  digitalWrite(STBY_PIN, LOW);
  analogWrite(PWMA_PIN, 0);
  analogWrite(PWMB_PIN, 0);

  // Turn off QTR IR Emitter to save power
  pinMode(IR_EMITTER_PIN, OUTPUT);
  digitalWrite(IR_EMITTER_PIN, LOW);
}

void initESPNow() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  esp_now_add_peer(&peerInfo);
}

void sendTelemetry() {
  esp_now_send(receiverAddress, (uint8_t *)&telemetryData, sizeof(telemetryData));
}