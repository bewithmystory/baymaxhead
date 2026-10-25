# File Understanding and Problem Summary

## Purpose

This project is the HEAD controller part of the DOOM WORLD Baymax robot system. The main goal is to interpret controller commands, move the head, monitor the environment, and report telemetry back to the controller.

## What each file does

### `platformio.ini`

This file configures the PlatformIO environment for the ESP32 head node. It tells the build system which board and framework are used, and adds the shared folder to the include path so `protocol.h` can be accessed.

Important point:
- The file is configured for the ESP32 development board.
- The include path is set so shared packet definitions are available across all subsystem nodes.

### `include/pins.h`

This is the hardware pin map. It defines the exact ESP32 GPIO numbers used for:

- two servo channels,
- ultrasonic trigger/echo,
- I2C IMU bus,
- laser,
- head LED,
- camera power transistor,
- buzzer.

This file is intentionally kept as the fixed hardware contract. The current system wiring matches it.

### `include/config.h`

This file contains the operating limits and safety values:

- servo angles,
- PWM frequency,
- ultrasonic alarm threshold,
- timeout values,
- MPU6500 address.

It is the tuning file for safe behavior and sensor thresholds.

### `shared/protocol.h`

This file defines the shared ESP-NOW protocol used by the head, controller, and other nodes.

It includes:
- `DOOM_MAGIC` = `0xB808`
- `WIFI_CHANNEL` = 1
- auxiliary target IDs such as camera power, head LED, laser, servo outputs, and volume
- packet structs for telemetry and command packets

This is the most important contract file because the head must decode command packets exactly as the controller sends them.

### `src/main.cpp`

This is the main firmware. It handles all runtime work.

Core responsibilities:
- ESP-NOW initialization and packet receive handling
- joystick control decoding
- auxiliary command decoding
- servo target updates
- laser output control
- head LED control
- camera power switching
- IMU heading tracking
- ultrasonic distance sensing
- telemetry transmission back to the controller
- buzzer alarm behavior

## Problem understanding

The project was not just a simple sensor sketch; it was a multi-device controller system where packet compatibility and hardware mapping mattered a lot.

The main issues that needed fixing were:

1. Packet version mismatch between V3 and V4 controllers.
2. Servo motion becoming unstable or out of range without bounds checking.
3. Laser sticking on after a link loss.
4. Alarm logic not being triggered properly with obstacle range data.
5. Shared protocol mismatches across devices.
6. Wiring drift or mismatched GPIO assumptions.

The present code handles those issues by:
- accepting both old and new auxiliary packet layouts,
- clamping servo values to valid ranges,
- using control timeout logic,
- turning off the laser on a failsafe,
- sending diagnostics through serial output,
- keeping the hardware pin map stable and explicit.

## Why we kept `pins.h` unchanged

The user instruction was very clear: keep the current pin configuration same. This is correct because the wiring is already part of the physical hardware definition.

Changing `pins.h` without updating the actual wiring would create a mismatch between software and hardware. That creates non-obvious behavior and makes debugging much harder.

## Final project meaning

The head board is responsible for turning controller intent into physical head movement and sensing. The firmware is the bridge between the external controller protocol and the real-world actuators and sensors.

The important design rule is:

- Keep the physical wiring fixed.
- Keep all packet contracts aligned.
- Keep outputs fail-safe.
- Keep telemetry honest.

That is what the current code and docs represent.
