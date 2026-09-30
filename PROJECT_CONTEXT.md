# Robo Seguidor de Linha: Project Context

## 1. Project Overview

This is a PlatformIO/Arduino project for an ESP32 line-following robot. It
contains two separately compiled firmware variants:

- `robot`: reads six analog reflectance sensors, follows a black line with a
  PID controller, drives two motors through a TB6612FNG driver, and broadcasts
  telemetry with ESP-NOW.
- `receiver`: runs on a second ESP32, receives the robot telemetry, and writes
  it to the serial port in Teleplot format for visualization on a computer.

The primary goal is a short, observable autonomous run. The robot calibrates
the sensors for approximately three seconds, enables the motor driver, runs
for at most ten seconds, and then stops permanently until reset. Telemetry is
sent once per control-loop iteration; there is no command channel from the
receiver back to the robot.

Important constraints are:

- Both firmware variants target an `esp32dev` board with the Arduino framework.
- The robot expects a six-channel analog QTR sensor array and a TB6612FNG
  motor-driver wiring defined in `src/robot.cpp`.
- The robot must be programmed with the receiver ESP32 MAC address currently
  hard-coded in `receiverAddress`.
- The sender and receiver must agree on the telemetry struct layout and on the
  ESP-NOW/Wi-Fi channel conditions.
- The receiver's serial output is intended for a Teleplot-compatible monitor
  at 115200 baud.

## 2. Architecture

### Components

```text
QTR-8A / six analog sensors
          |
          v
     robot ESP32
       |  PID output
       v
 TB6612FNG -> left/right motors
       |
       +---- telemetry struct --ESP-NOW--> receiver ESP32
                                             |
                                             v
                                      USB serial, Teleplot lines
```

The two firmware images share no header file. The telemetry struct is copied
manually into both source files, so the two definitions must be kept identical.

### Robot execution flow

1. `setup()` starts serial output and configures the motor GPIOs with the
   driver in standby.
2. The QTR library is configured for analog sensors on GPIOs 36, 39, 34, 35,
   32, and 33, with the IR emitter on GPIO 4.
3. Wi-Fi is put in station mode and ESP-NOW is initialized. The configured
   receiver MAC is added as an unencrypted peer.
4. The QTR array is calibrated 150 times with a 20 ms delay between samples.
   The onboard LED is on during this approximately three-second phase.
5. Standby is released and `runStartTime` is recorded.
6. Each `loop()` iteration reads line position, computes PID correction,
   applies motor speeds, fills telemetry, and sends it to the receiver.
7. Once ten seconds have elapsed, `stopRobot()` disables the driver and the
   firmware remains in an infinite delay loop until reset.

### Receiver execution flow

1. `setup()` starts serial at 115200 baud, sets Wi-Fi station mode, initializes
   ESP-NOW, and registers `OnDataRecv()`.
2. ESP-NOW invokes the callback asynchronously when a packet arrives.
3. The callback copies the packet into `incomingData` and prints timestamp,
   position, error, motor speeds, and all six sensors using Teleplot's
   `>name:value` line format.
4. `loop()` does nothing; all receiver work is callback-driven.

## 3. File and Directory Structure

```text
platformio.ini                  PlatformIO environments and dependencies
PROJECT_CONTEXT.md              This project-specific engineering context
src/robot.cpp                   Robot control firmware entry point
src/receiver.cpp                ESP-NOW telemetry receiver entry point
include/README                  PlatformIO placeholder for project headers
lib/README                      PlatformIO placeholder for private libraries
test/README                     PlatformIO placeholder for test code
Robo_seguidor_de_linha.code-workspace  VS Code workspace definition
.gitignore                      Ignore rules for PlatformIO/IDE artifacts
.vscode/                         PlatformIO/VS Code integration settings
.pio/                            Generated builds and downloaded dependencies
```

The central files are `platformio.ini`, `src/robot.cpp`, and
`src/receiver.cpp`. `PROJECT_CONTEXT.md` is documentation and is not compiled.

`.pio/build` contains generated build products and `.pio/libdeps` contains
PlatformIO-managed dependencies. These should normally not be edited or
committed. The `.vscode` files are IDE integration/generated configuration;
change them only when the local editor setup requires it. The `include`, `lib`,
and `test` directories currently contain only PlatformIO instructional
placeholders, not application code.

## 4. Code Explanation

### Robot state and hardware constants

`SENSOR_COUNT` is 6 and `sensorValues` stores the latest raw/calibrated QTR
readings. `QTRSensors qtr` owns sensor configuration and calibration state.
The QTR position returned by `readLineBlack()` is treated as a value from 0 to
5000, with 2500 as the center for six sensors.

Motor A is treated as the left motor and uses PWM GPIO 23 plus direction GPIOs
22 and 21. Motor B is treated as the right motor and uses PWM GPIO 18 plus
direction GPIOs 17 and 16. GPIO 5 controls TB6612FNG standby.

### Control algorithm

The line error is:

```text
error = position - 2500
```

The controller accumulates the error, clamps the integral to -2000..2000,
calculates the change from the previous error, and computes:

```text
adjustment = Kp * error + Ki * integral + Kd * derivative
leftSpeed  = BASE_SPEED + adjustment
rightSpeed = BASE_SPEED - adjustment
```

The current gains are `Kp = 0.06`, `Ki = 0.0001`, and `Kd = 0.6`; the base
speed is 180 and the requested signed motor range is -255..255. The derivative
is based on successive loop iterations rather than elapsed time, so loop-rate
changes alter the effective controller behavior. There is also no explicit
integral reset at calibration completion beyond global initialization.

`setMotorSpeeds()` maps the sign to direction pins and the magnitude to
`analogWrite()`. Negative values reverse the corresponding motor. The final
clamp occurs before this function, so the magnitude is expected to be within
the PWM range.

### Safety and shutdown

The ten-second limit is measured with unsigned `millis()` subtraction, which
is the usual rollover-safe comparison pattern. On timeout, standby is disabled,
both PWM outputs are set to zero, the IR emitter is forced low, and execution
stops in a permanent delay loop.

### Telemetry

Both programs define this wire payload in the same order and with the same
fixed-width types:

```text
uint16_t sensors[6]
uint16_t position
int16_t  error
int16_t  leftMotorSpeed
int16_t  rightMotorSpeed
uint32_t timestamp
```

The timestamp is elapsed run time in milliseconds, not an absolute clock. The
receiver prints sensor names `S1` through `S6`, plus `position`, `error`,
`leftMotorSpeed`, `rightMotorSpeed`, and `timestamp`.

## 5. Configuration and Dependencies

`platformio.ini` defines shared settings:

- Platform: `espressif32`
- Board: `esp32dev`
- Framework: `arduino`
- Serial monitor speed: `115200`

The `[env:robot]` environment excludes all source files except `robot.cpp` and
declares `pololu/QTRSensors@^4.0.0`. ESP32 Arduino headers and ESP-NOW support
come from the selected framework/platform package.

The `[env:receiver]` environment excludes all source files except
`receiver.cpp`; it has no extra library dependency. The environment name
`receiver` is a second ESP32 firmware target, despite the comment calling it
"PC Receiver Code".

There are no environment variables, databases, web services, or runtime config
files. The receiver MAC address, GPIO mapping, PID gains, speed limits, and
run limit are compile-time constants in `robot.cpp`.

Typical commands from the project root are:

```bash
pio run -e robot
pio run -e receiver
pio run -e robot -t upload
pio run -e receiver -t upload
pio device monitor -b 115200
```

The exact upload port is not configured in this repository, so PlatformIO may
need automatic port detection or a user-supplied `upload_port`.

## 6. Important Design Decisions

- Separate PlatformIO environments allow either board to be flashed without
  compiling both entry points into one image.
- ESP-NOW provides low-overhead local wireless telemetry without a router or
  application-level server. This is an observed implementation choice; the
  repository does not document the original rationale.
- The QTR sensors use analog mode and ADC1 GPIOs. Using ADC1 pins appears
  deliberate because ESP32 ADC2 access has Wi-Fi-related limitations, but this
  rationale is inferred from the pin selection rather than stated in the code.
- The telemetry is a raw binary struct for simplicity and low overhead. This
  trades explicit serialization/versioning for a requirement that both builds
  use compatible ABI/layout and field order.
- Calibration happens at boot and motors remain disabled until it completes,
  reducing startup motion while sensor ranges are learned.
- The hard ten-second cutoff is a safety-oriented operational constraint, but
  it also makes the firmware unsuitable for an unattended continuous follower
  unless the behavior is intentionally changed.

## 7. Hardware and External Systems

### Hardware interface

| Function | GPIOs / value |
| --- | --- |
| QTR analog sensors, in array order | 36, 39, 34, 35, 32, 33 |
| QTR IR emitter control | 4 |
| Left motor PWM / direction | 23 / 22, 21 |
| Right motor PWM / direction | 18 / 17, 16 |
| TB6612FNG standby | 5 |
| Onboard status LED | 2 |

The code assumes the sensor array order corresponds to the physical left-to-
right order expected by `readLineBlack()`, and that the motor driver wiring
matches the left/right direction conventions in `setMotorSpeeds()`.

### Wireless interface

The robot uses Wi-Fi station mode and sends unencrypted ESP-NOW packets to the
six-byte MAC address in `receiverAddress`. The peer is configured with channel
0, allowing the ESP-NOW stack to use the current channel. No delivery status is
checked and no retries or application acknowledgements are implemented.

### Serial interface

The receiver emits newline-terminated Teleplot records at 115200 baud. The
receiver itself does not parse commands. Serial output from the robot is only
initialized; the robot does not print its control values.

## 8. Invariants and Assumptions

- `SENSOR_COUNT` must remain identical in both source files and must match the
  QTR pin array and telemetry sensor array length.
- The telemetry field order, widths, signedness, and alignment must remain
  compatible on both ESP32 builds. Any payload change must be made on both
  sides together.
- `position` is interpreted on a 0..5000 scale and center is exactly 2500.
- Sensor values and position are unsigned 16-bit values; error and motor speeds
  are signed 16-bit values; elapsed time is an unsigned 32-bit millisecond
  counter.
- `BASE_SPEED` and `MAX_SPEED` are PWM-scale values, currently 180 and 255.
- The receiver MAC must identify the actual receiver board. A stale MAC causes
  telemetry transmission to fail even if the robot control loop continues.
- The receiver must be powered and configured for compatible ESP-NOW reception;
  the robot does not require an acknowledgement to drive.
- Calibration should occur over representative line/background positions. The
  robot may produce poor positions if the array is stationary over unsuitable
  reflectance during calibration.
- The callback currently assumes each received packet is at least the size of
  `struct_telemetry`. This is an input-validity assumption, not enforced by
  `len`.
- Motor direction polarity is hardware-dependent. If a motor spins opposite to
  the assumed direction, the robot's steering correction will be inverted or
  asymmetric until the wiring or code is corrected.

## 9. Common Pitfalls and Failure Modes

- **Wrong target selected:** upload `robot` to the moving robot and `receiver`
  to the telemetry board. The environment names select different source files.
- **Wrong receiver MAC:** update `receiverAddress` before flashing the robot;
  the current value is board-specific.
- **ESP-NOW channel mismatch:** station-mode channel conditions can prevent
  reception even when both MAC addresses are correct. Verify both boards are
  operating under compatible channel conditions.
- **Unsafe packet handling:** `receiver.cpp` copies `sizeof(incomingData)`
  without checking `len`. A malformed or shorter packet can cause an out-of-
  bounds read; validate length before copying if the receiver is exposed to
  untrusted traffic.
- **Struct drift:** changing a field, type, array length, or compiler/build
  assumptions in only one firmware produces corrupted telemetry. A shared
  header and explicit serialization would reduce this risk.
- **No ESP-NOW error handling on the robot:** initialization, peer addition,
  and send results are ignored. A radio or pairing failure is therefore not
  visible through the robot's serial output.
- **Calibration conditions:** the three-second calibration is a blocking boot
  phase. Do not expect motor motion or normal telemetry control during it.
- **Control-loop timing:** PID derivative and integral terms use iteration
  counts, not measured `dt`; adding delays, heavy logging, or radio work changes
  tuning behavior.
- **Analog pin assumptions:** the selected GPIOs are input-only ADC pins. They
  must not be reassigned as outputs, and the sensor supply/reference wiring
  must match the QTR hardware.
- **Run timeout:** after ten seconds the robot intentionally stops and does not
  restart from `loop()`. Reset or power-cycle it for another run.
- **Telemetry rate:** telemetry is sent as fast as the control loop executes;
  there is no throttling. A visualization tool should tolerate a high update
  rate, and radio congestion can cause dropped packets.
- **Standby state:** motors are disabled during initialization and after
  timeout. A board reset is required after a stop, not merely another loop
  iteration.

## 10. Extension Guidance

When modifying this project, preserve the two-environment build filtering and
test both targets independently. For protocol changes, prefer moving the
telemetry definition into a shared header under `include/`, adding a packet
version or length check, and validating `len` before copying. For controller
changes, document the units and timing assumptions, then verify motor polarity,
sensor ordering, calibration behavior, timeout behavior, and telemetry output
on hardware. Keep generated `.pio` output out of source changes.

There is currently no automated unit or integration test suite. The most useful
baseline validation is a successful build of both environments followed by a
hardware smoke test covering boot calibration, line tracking, ten-second stop,
ESP-NOW reception, and Teleplot output.