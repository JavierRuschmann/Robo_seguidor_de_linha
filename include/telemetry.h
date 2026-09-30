#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>

#define SENSOR_COUNT 6
#define TELEMETRY_VERSION 1u

struct TelemetryPacket {
  uint16_t version;
  uint16_t packetLength;
  uint16_t sensors[SENSOR_COUNT];
  uint16_t position;
  int16_t error;
  int16_t leftMotorSpeed;
  int16_t rightMotorSpeed;
  uint32_t timestamp;
};

static_assert(sizeof(TelemetryPacket) == 2 + 2 + (6 * 2) + 2 + 2 + 2 + 2 + 4,
              "TelemetryPacket layout must match the robot/receiver ABI");

#endif
