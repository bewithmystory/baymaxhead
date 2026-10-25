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

// Joystick & Mode Control Packet (sent every 50 ms).
// The current format transmits raw 12-bit ADC readings for both axes.
typedef struct __attribute__((packed)) {
  uint16_t magic;     // 0xB808
  uint8_t type;       // 3 = joystick/zoom control
  uint8_t mode;       // 0 = HEAD servos, 1 = BODY motors
  uint16_t x;         // Raw X-axis ADC reading (0..4095)
  uint16_t y;         // Raw Y-axis ADC reading (0..4095)
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

// HEAD -> HAND acknowledged auxiliary-command response (type = 6)
// Fixed 10-byte format required by the controller protocol contract.
typedef struct __attribute__((packed)) {
  uint16_t magic;
  uint8_t type;       // 6 = AUX ACK
  uint8_t target;     // AUX_* target ID
  uint8_t state;      // 0 = OFF, 1 = ON
  uint8_t value;      // 0..180 for servo, 0..100 for volume
  uint32_t sequence;
} AuxAckPacket;

// HEAD -> HAND periodic status packet (type = 5)
// Fixed 66-byte packet for controller health/telemetry monitoring.
typedef struct __attribute__((packed)) {
  uint16_t magic;
  uint8_t type;       // 5 = HEAD status
  uint8_t version;    // status schema version
  uint8_t laser;
  uint8_t headLed;
  uint8_t camPower;
  uint8_t buzzer;
  uint8_t buzzerVolume;
  uint8_t joyMode;
  int8_t joyX;
  int8_t joyY;
  uint8_t lastTarget;
  uint8_t lastState;
  uint8_t lastValue;
  uint32_t lastSeq;
  float panTarget;
  float tiltTarget;
  float distanceCm;
  uint8_t distanceValid;
  uint32_t echoUs;
  uint32_t controlAgeMs;
  uint32_t rxPacketCount;
  uint32_t rxControlCount;
  uint32_t rxAuxCount;
  uint32_t rxLegacyCount;
  uint32_t rxRejectedCount;
  uint32_t telemetryTxFailureCount;
  uint8_t radioChannel;
  uint8_t reserved;
} HeadStatusPacket;

#endif // PROTOCOL_H