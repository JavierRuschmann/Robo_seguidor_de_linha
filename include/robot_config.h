#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <Arduino.h>
#include "telemetry.h"

namespace RobotConfig {
  // Serial and startup behavior
  constexpr uint32_t SERIAL_BAUD_RATE = 115200;
  constexpr uint32_t RUN_TIME_LIMIT_MS = 10000;
  constexpr uint16_t CALIBRATION_SAMPLES = 150;
  constexpr uint16_t CALIBRATION_DELAY_MS = 20;

  // Hardware mapping
  constexpr uint8_t QTR_PINS[SENSOR_COUNT] = {36, 39, 34, 35, 32, 33};
  constexpr uint8_t IR_EMITTER_PIN = 4;
  constexpr uint8_t RECEIVER_ADDRESS[6] = {0x8C, 0x94, 0xDF, 0x4C, 0x71, 0x90};

  // Motor and control tuning
  constexpr int BASE_SPEED = 180;
  constexpr int MAX_SPEED = 255;
  constexpr float MOTOR_RAMP_STEP_PER_SEC = 220.0f;
  constexpr float PID_DT_FALLBACK_SECONDS = 0.016f;
  constexpr float MAX_PID_INTEGRAL = 2000.0f;

  constexpr float Kp = 0.06f;
  constexpr float Ki = 0.0001f;
  constexpr float Kd = 0.6f;

  // Line detection and recovery
  constexpr uint16_t LINE_DETECTION_THRESHOLD = 50;
  constexpr uint16_t LINE_LOSS_THRESHOLD = 1500;
  constexpr int SEARCH_TURN_SPEED = 110;
}

#endif
