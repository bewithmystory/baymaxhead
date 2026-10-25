# DOOM WORLD Baymax Head Help

This file captures the project understanding for the Baymax head module and the fixes that were applied while aligning the firmware with the controller system.

## 1. What this project is

This repository is the HEAD node for the DOOM WORLD Baymax controller ecosystem. It is not the main hand controller. The head node is a separate ESP32 board that:

- receives joystick and auxiliary commands from the hand controller through ESP-NOW,
- drives the pan/tilt servos,
- reads the ultrasonic range sensor,
- tracks heading with the MPU6500 gyroscope,
- controls the laser, head LED, camera power, and buzzer,
- sends telemetry back to the controller.

This means the head acts as a local actuator node, while the hand controller is the UI/input device.

## 2. The controller relationship

The architecture is built around a shared protocol contract:

- `shared/protocol.h` defines the packet format and a shared magic code `0xB808`.
- Control packets are sent from the hand controller to the head.
- Auxiliary packets control outputs such as camera power, head LED, laser, servos, and volume.
- The head sends `SensorPacket` telemetry back every 100 ms to the controller.

In practical terms:

- Hand = command center
- Head = sensor + servo + output node
- Body = motion platform, when present
- Camera = vision node

## 3. Pin configuration kept unchanged

The current pin map in [include/pins.h](include/pins.h) is the one used by the working head hardware. This pin assignment was preserved intentionally:

```cpp
#define SERVO_PAN_PIN     18
#define SERVO_TILT_PIN    19
#define ULTRA_TRIG_PIN    27
#define ULTRA_ECHO_PIN    35
#define MPU_SDA_PIN       21
#define MPU_SCL_PIN       22
#define LASER_PIN         23
#define HEAD_LED_PIN       4
#define CAM_POWER_PIN     16
#define HEAD_BUZZER_PIN   17
```

This is the baseline hardware contract. Do not change it unless the wiring is physically updated.

## 4. Main hardware blocks

### Pan/Tilt servos
- GPIO 18 and 19
- Controlled by LEDC hardware PWM
- Use the fixed 50 Hz servo timing and limited angle ranges

### Ultrasonic sensor
- TRIG = GPIO 27
- ECHO = GPIO 35
- Distance is measured and published back to the controller
- Alarm state triggers when distance is under 10 cm

### MPU6500 IMU
- SDA = GPIO 21
- SCL = GPIO 22
- Yaw angle is integrated from Z-axis gyroscope data
- Bias is calibrated at startup to reduce drift

### Outputs
- Laser = GPIO 23
- Head LED = GPIO 4
- Camera power control = GPIO 16
- Buzzer = GPIO 17

## 5. Fixes and changes that were applied

These are the important changes reflected in the current firmware and documentation:

1. ESP-NOW packet compatibility
   - The firmware accepts both V3 and V4 packet layouts.
   - Legacy 9-byte auxiliary commands are still supported.
   - This resolves controller-version mismatches.

2. Servo control normalization
   - Pan/tilt target values are clamped to allowed ranges.
   - Joystick-driven movement is smoothed to avoid jitter.
   - The servo motion is limited by the control timeout to prevent runaway behavior when link is lost.

3. Laser failsafe
   - A heartbeat-style timeout turns the laser off if the controller stops sending commands.
   - This prevents the laser from staying on forever after communication loss.

4. Buzzer and obstacle warning logic
   - The buzzer is treated as a controlled output and can be enabled/disabled by the controller.
   - The ultrasound alarm triggers a rapid warning beep when a detected object is too close.

5. Camera power handling
   - The ESP32-CAM power transistor is toggled using the auxiliary camera power target.
   - It powers on by default at startup for easier bring-up.

6. Diagnostic output
   - Periodic serial logging shows radio health, packet counts, and ESP-NOW errors.
   - This helps debug packet rejection and telemetry health.

## 6. File-by-file understanding

- `platformio.ini` — board and build configuration for the ESP32 head node.
- `include/pins.h` — hardware GPIO mapping, kept unchanged.
- `include/config.h` — servo ranges, fail-safe values, ultrasonic thresholds.
- `shared/protocol.h` — protocol packet layout and target IDs.
- `src/main.cpp` — main firmware logic for ESP-NOW, servos, IMU, sensors, and outputs.
- `file_directory_arrangement/arrangement.md` — explains the larger multi-node architecture.

## 7. Important project note

The original GitHub controller link in the request returned a 404, so this local workspace is the source of truth for the currently working design. The codebase here already contains the corrected and adapted head-node logic, and the pin map was intentionally left unchanged.

## 8. Recommended workflow

1. Keep the wiring exactly aligned with [include/pins.h](include/pins.h).
2. Build the project from this root using PlatformIO.
3. Verify controller-to-head ESP-NOW packets.
4. Check the serial logs for `rx`, `aux`, and `rejected` counts.
5. Validate servo motion, IMU heading drift, and ultrasonic alarm behavior.

## 9. Summary

The Baymax head is a compact ESP32-controlled sensor and actuator node. The key to making it work correctly is matching the protocol and wiring contract, keeping the pin assignment fixed, and supporting both old and new controller packet versions. That is the main fix reflected in the current code.
