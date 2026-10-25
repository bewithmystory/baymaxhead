# Baymax Head

This project is the HEAD node for the DOOM WORLD Baymax controller system. It is designed to work with the hand/controller side and receive commands over ESP-NOW, then convert them into real head motion, sensor reporting, and output control.

## Project purpose

The head board handles:

- pan/tilt servo motion,
- HC-SR04 distance sensing,
- MPU6500 heading tracking,
- laser control,
- head LED output,
- camera power switching,
- buzzer warning alarm,
- telemetry sending back to the controller.

## Hardware contract

The current hardware pin map is kept unchanged in [include/pins.h](include/pins.h). The working mapping is:

- Servo pan: GPIO 18
- Servo tilt: GPIO 19
- Ultrasonic trigger: GPIO 27
- Ultrasonic echo: GPIO 35
- MPU SDA: GPIO 21
- MPU SCL: GPIO 22
- Laser: GPIO 23
- Head LED: GPIO 4
- Camera power: GPIO 16
- Buzzer: GPIO 17

This pin configuration is the fixed baseline for the current build.

## File overview

- [platformio.ini](platformio.ini) — PlatformIO build configuration.
- [include/pins.h](include/pins.h) — fixed GPIO map for the head hardware.
- [include/config.h](include/config.h) — servo limits, joystick ADC calibration, thresholds, and safety settings.
- [shared/protocol.h](shared/protocol.h) — shared command and telemetry packet definitions.
- [src/main.cpp](src/main.cpp) — the main ESP32 firmware.
- [headhelp.md](headhelp.md) — head-specific help and debugging notes.
- [files_understanding_problem.md](files_understanding_problem.md) — file-by-file understanding and issue summary.
- [file_directory_arrangement/arrangement.md](file_directory_arrangement/arrangement.md) — larger system layout reference.

## Key fixes reflected in the current code

- Added compatibility for both V3 and V4 controller packet layouts.
- Kept legacy auxiliary command support so the head still accepts older controller data.
- Added controller-level AUX ACK responses and periodic HEAD status packets for health monitoring.
- Added servo angle clamping and smoothing.
- Added laser failsafe so the laser turns off if commands stop.
- Added obstacle warning buzzer based on ultrasonic data.
- Added telemetry reporting and diagnostic serial logging.
- Kept hardware pin mapping stable and documented.

## Notes

The controller GitHub link mentioned in the request returned a 404, so the local workspace itself is the authoritative source for the working head system. This repository contains the corrected and adapted head-node logic, while the pin assignment was intentionally preserved exactly as wired.

For more detail, see [headhelp.md](headhelp.md) and [files_understanding_problem.md](files_understanding_problem.md).
