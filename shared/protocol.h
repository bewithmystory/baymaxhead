#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>

#define DOOM_MAGIC   0xB808
#define WIFI_CHANNEL 1

// Auxiliary target IDs (DOOM_CONTROLLER_CONTEXT.md contract)
#define AUX_CAMERA_POWER 1
#define AUX_HEAD_LED     2
#define AUX_LASER        3
#define AUX_SERVO_1      4
#define AUX_SERVO_2      5
#define AUX_VOLUME       6

// Receiver-to-HAND telemetry packet
typedef struct {
  uint32_t sequence;
  float distanceCm;
  uint8_t valid;
} SensorPacket;

// Legacy V3 laser command (type = 2) supported for backward compatibility
typedef struct {
  uint8_t type;       // 2 = laser command
  uint8_t laserOn;    // 0 = OFF, 1 = ON
  uint16_t magic;     // 0xB808 verification
} LaserCommand;

// V3/V4 Joystick & Mode Control Packet (sent every 50 ms)
typedef struct __attribute__((packed)) {
  uint16_t magic;     // 0xB808
  uint8_t type;       // 3 = joystick/zoom control
  uint8_t mode;       // 0 = HEAD servos, 1 = BODY motors
  int8_t x;           // -100..100
  int8_t y;           // -100..100
  uint8_t zoom;       // 0..100 from potentiometer
  uint32_t sequence;
} V3ControlPacket;

// V4 Auxiliary Output & Servo/Volume Command Packet (type = 4)
// Supports both 9-byte (V3 without value) and 10-byte (V4 with value) payloads
typedef struct __attribute__((packed)) {
  uint16_t magic;     // 0xB808
  uint8_t type;       // 4 = auxiliary output command
  uint8_t target;     // AUX_* target ID (1..6)
  uint8_t state;      // 0 = OFF, 1 = ON
  uint8_t value;      // 0..180 for servos, 0..100 for volume
  uint32_t sequence;
} V3AuxCommand;

#endif // PROTOCOL_H